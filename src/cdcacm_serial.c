#include <stdio.h>
#include <device.h>
#include <zephyr.h>
#include <uart.h>
#include <gpio.h>
#include "soc.h"

#include "soc_ctrl.h"
#include "curie_shared_mem.h"

/**
 * Use the following defines just to make the tips of your finger happier.
 */
#define Rx_BUFF curie_shared_data->cdc_acm_shared_rx_buffer.data
#define Rx_HEAD curie_shared_data->cdc_acm_shared_rx_buffer.head
#define Rx_TAIL curie_shared_data->cdc_acm_shared_rx_buffer.tail
#define Rx_LOCK curie_shared_data->cdc_acm_shared_rx_buffer.lock
#define Tx_BUFF curie_shared_data->cdc_acm_shared_tx_buffer.data
#define Tx_HEAD curie_shared_data->cdc_acm_shared_tx_buffer.head
#define Tx_TAIL curie_shared_data->cdc_acm_shared_tx_buffer.tail
#define Tx_LOCK curie_shared_data->cdc_acm_shared_tx_buffer.lock
#define SBS     SERIAL_BUFFER_SIZE

#define RESET_BAUD              1200
#define BAUDRATE_RESET_SLEEP    50

/* Make sure BUFFER_LENGTH is not bigger then shared ring buffers */
#define BUFFER_LENGTH		128

#define USB_CONNECTED	    0x04
#define USB_DISCONNECTED    0x05

#define SERIAL_READ_TIMEOUT	1000
#define CDCACM_TX_DELAY		5000
#define CDCACM_TX_TIMEOUT	5000
#define CDCACM_TX_TIMEOUT_DELAY	10

#define SCSS_RSTC   (uint32_t*)0xb0800570

#define RSTC_WARM_RESET	(1 << 1)
#define RSTC_COLD_RESET (1 << 3)

#define TXRX_LED 12

typedef enum {
	ACM_RX_DISABLED,
	ACM_RX_READY,
	ACM_RX_PENDING
} acm_rx_states;

typedef enum {
	ACM_TX_DISABLED,
	ACM_TX_READY,
	ACM_TX_PENDING
} acm_tx_states;

struct device *dev;
struct device *gpio_dev;
bool usbSetupDone = false;
bool enableReboot = false;

// buffers
static unsigned char data_buf[128];

static volatile uint32_t acm_rx_state = ACM_RX_DISABLED;
static volatile uint32_t acm_tx_state = ACM_TX_DISABLED;

static volatile bool data_transmitted;
static volatile bool data_arrived = false;

#define USB_GRSTCTL 0xB0500010

void flush_usb_txfifo()
{
	*(uint32_t*)USB_GRSTCTL = *(uint32_t*)USB_GRSTCTL | 0x00000020;
}

static void interrupt_handler(struct device *dev)
{
	uart_irq_update(dev);

	if (uart_irq_tx_ready(dev)) {
		data_transmitted = true;
	}

	if (uart_irq_rx_ready(dev)) {
		data_arrived = true;
	}
}

static void read_data(struct device *dev, int *bytes_read)
{
	int timeout = SERIAL_READ_TIMEOUT;

	while (!data_arrived && !timeout)
		--timeout;

	data_arrived = false;

	/* Read all data */
	*bytes_read = uart_fifo_read(dev, data_buf, sizeof(data_buf));
}

static void write_data(struct device *dev, const char *buf, int len)
{	
	uint32_t dtr = 0;
	uart_irq_tx_enable(dev);
	
	for(int i = 0; i < len; i+=4)
	{
		data_transmitted = false;
		int timeout = 0;
		if((i+4) <= len)
		{
			uart_fifo_fill(dev, buf+i, 4);
		}
		else
		{
			uart_fifo_fill(dev, buf+i, len-i);
		}
		while (data_transmitted == false)
		{
			uart_line_ctrl_get(dev, LINE_CTRL_DTR, &dtr);
			if(!dtr)
				goto done_write;
				
			timeout += CDCACM_TX_TIMEOUT_DELAY;
			if(timeout >= CDCACM_TX_TIMEOUT)
				goto done_write;
		}
	}
done_write:
	uart_irq_tx_disable(dev);
}

void cdc_acm_tx()
{
	uint32_t dtr = 0;
	uint8_t write_buffer[BUFFER_LENGTH];
	volatile int head = Tx_HEAD;
	volatile int tail = Tx_TAIL;
	if (acm_tx_state == ACM_TX_READY) 
	{
		if(head!= tail)
		{
			uint8_t cnt = 0, index = Tx_TAIL;
			k_busy_wait(CDCACM_TX_DELAY);
			for (; (index != head) && (cnt < BUFFER_LENGTH);cnt++)
			{
				while(Tx_LOCK);
				write_buffer[cnt] = (uint8_t)*(Tx_BUFF+index);
				index = (index + 1)% SBS;
			}
			Tx_TAIL = (tail + cnt) % SBS;

			gpio_pin_write(gpio_dev, TXRX_LED, 0);	//turn TXRX led on
			write_data(dev, (const char*)write_buffer, cnt);
			gpio_pin_write(gpio_dev, TXRX_LED, 1);	//turn TXRX led off
		
		}
		else
		{

		}
	}
	else if (acm_tx_state == ACM_TX_DISABLED) 
	{
		Tx_TAIL = Tx_HEAD;
	}
}

void cdc_acm_rx()
{
	int bytes_read = 0, new_head, i;
	if (acm_rx_state == ACM_RX_READY)
	{
		read_data(dev, &bytes_read);
	}

	i=0;
	while(bytes_read)
	{
		gpio_pin_write(gpio_dev, TXRX_LED, 0);	//turn TXRX led on
		if (!curie_shared_data->cdc_acm_buffers_obj.device_open) 
		{
			// ARC is not ready to receive this data - discard it
			bytes_read = 0;
			gpio_pin_write(gpio_dev, TXRX_LED, 1);	//turn TXRX led off
			break;
		}
		// Check room in Rx buffer
		new_head = (Rx_HEAD + 1) % SBS;
		if (new_head != Rx_TAIL) 
		{
			Rx_BUFF[Rx_HEAD] = data_buf[i];
			Rx_HEAD = new_head;
			i++;
			bytes_read--;
		} 
		else 
		{
			gpio_pin_write(gpio_dev, TXRX_LED, 1);	//turn TXRX led off
			break;
		}
		gpio_pin_write(gpio_dev, TXRX_LED, 1);	//turn TXRX led off
	}
	
}

void init_cdc_acm()
{
	// setup shared memory pointers for cdc-acm buffers
	curie_shared_data->cdc_acm_buffers = &curie_shared_data->cdc_acm_buffers_obj;
	curie_shared_data->cdc_acm_buffers_obj.rx_buffer = &curie_shared_data->cdc_acm_shared_rx_buffer;
	curie_shared_data->cdc_acm_buffers_obj.tx_buffer = &curie_shared_data->cdc_acm_shared_tx_buffer;

	curie_shared_data->cdc_acm_buffers_obj.host_open = false;
}

void cdcacm_setup(void *dummy1, void *dummy2, void *dummy3)
{
	uint32_t baudrate, dtr = 0;
	int ret;

	dev = device_get_binding(CONFIG_CDC_ACM_PORT_NAME);

	while (1) {
		uart_line_ctrl_get(dev, LINE_CTRL_DTR, &dtr);
		if (dtr)
			break;
		k_yield();
	}

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(dev, LINE_CTRL_DCD, 1);
	
	acm_tx_state = ACM_TX_READY;
	acm_rx_state = ACM_RX_READY;
	
	ret = uart_line_ctrl_set(dev, LINE_CTRL_DSR, 1);

	enableReboot = true;
	k_yield();

	gpio_dev= device_get_binding("GPIO_0");
	gpio_pin_configure(gpio_dev, TXRX_LED, (GPIO_DIR_OUT));
	gpio_pin_write(gpio_dev, TXRX_LED, 1);
    
	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);

	uart_irq_callback_set(dev, interrupt_handler);
	flush_usb_txfifo();
	//reset head and tails values to 0
	Tx_TAIL = 0;
	Tx_HEAD = 0;
	Tx_LOCK = 0;
	Rx_TAIL = 0;
	Rx_HEAD = 0;
	Rx_LOCK = 0;
	usbSetupDone = true;
	curie_shared_data->cdc_acm_buffers_obj.host_open = true;
}

void baudrate_reset(void *dummy1, void *dummy2, void *dummy3)
{
	uint32_t baudrate, ret = 0;
	while(!enableReboot)
	{
		k_yield();
	}
	ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);	
	while(1)
	{
		ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);
		if(baudrate == RESET_BAUD)
		{
			soft_reboot();
		}
		k_sleep(BAUDRATE_RESET_SLEEP);
	}
}

void usb_serial(void *dummy1, void *dummy2, void *dummy3)
{
	while(!usbSetupDone)
	{
		k_yield();
	}

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);

	while (1) 
	{
		uint32_t dtr = 0;
		uart_line_ctrl_get(dev, LINE_CTRL_DTR, &dtr);
		if(dtr)
		{
			if(curie_shared_data->cdc_acm_buffers_obj.host_open == false)
			{
				curie_shared_data->cdc_acm_buffers_obj.host_open = true;
				uart_line_ctrl_set(dev, LINE_CTRL_DCD, 1);
				uart_line_ctrl_set(dev, LINE_CTRL_DSR, 1);
				flush_usb_txfifo();
			}
			else
			{
				cdc_acm_tx();
				cdc_acm_rx();
			}
		}
		else
		{
			if(curie_shared_data->cdc_acm_buffers_obj.host_open == true)
			{
				curie_shared_data->cdc_acm_buffers_obj.host_open = false;
				flush_usb_txfifo();
				uart_line_ctrl_set(dev, LINE_CTRL_DCD, 1);
				uart_line_ctrl_set(dev, LINE_CTRL_DSR, 1);
				k_busy_wait(1000000);
				Tx_TAIL = 0;
				Tx_HEAD = 0;
				Tx_LOCK = 0;
				Rx_TAIL = 0;
				Rx_HEAD = 0;
				Rx_LOCK = 0;
			}
		}
		k_yield();
	}
}

