#include <stdio.h>
#include <device.h>
#include <zephyr.h>
#include <uart.h>
#include <gpio.h>
#include "soc.h"

#include "soc_ctrl.h"
#include "curie_shared_mem.h"

#define RESET_BAUD 1200
#define BAUDRATE_RESET_SLEEP 50

/* Make sure BUFFER_LENGTH is not bigger then shared ring buffers */
#define BUFFER_LENGTH		128

#define USB_CONNECTED	    0x04
#define USB_DISCONNECTED    0x05

#define SERIAL_READ_TIMEOUT	1000
#define CDCACM_TX_DELAY		4000

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
static uint8_t write_buffer[BUFFER_LENGTH*2];

static volatile uint32_t acm_rx_state = ACM_RX_DISABLED;
static volatile uint32_t acm_tx_state = ACM_TX_DISABLED;

static volatile bool data_transmitted;
static volatile bool data_arrived = false;


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
	uart_irq_tx_enable(dev);

	data_transmitted = false;
	uart_fifo_fill(dev, (const uint8_t*)buf, len);
	while (data_transmitted == false)
	{
		task_yield();
	}
	sys_thread_busy_wait(CDCACM_TX_DELAY); //allow enough time for the FIFO to be emptied
	uart_irq_tx_disable(dev);
}

void cdc_acm_tx()
{
	if (acm_tx_state == ACM_TX_READY) 
	{
		if(Tx_HEAD != Tx_TAIL)
		{
			int cnt = 0, index = Tx_TAIL;
			gpio_pin_write(gpio_dev, TXRX_LED, 0);	//turn TXRX led on
			for (; (index != Tx_HEAD) && (cnt < BUFFER_LENGTH);cnt++) 
			{
				write_buffer[cnt] = Tx_BUFF[index];
				index = (index + 1) % SBS;
			}
			Tx_TAIL= (Tx_TAIL + cnt) % SBS;
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
	int bytes_read , new_head, i;
	if (acm_rx_state == ACM_RX_READY)
	{
		read_data(dev, &bytes_read);
	}

	i=0;
	while(bytes_read)
	{
		if (!curie_shared_data->cdc_acm_buffers_obj.device_open) 
		{
			// ARC is not ready to receive this data - discard it
			bytes_read = 0;
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
		} else 
		{

			break;
		}
	}
	
}

void init_cdc_acm()
{
	// setup shared memory pointers for cdc-acm buffers
	curie_shared_data->cdc_acm_buffers = &curie_shared_data->cdc_acm_buffers_obj;
	curie_shared_data->cdc_acm_buffers_obj.rx_buffer = &curie_shared_data->cdc_acm_shared_rx_buffer;
	curie_shared_data->cdc_acm_buffers_obj.tx_buffer = &curie_shared_data->cdc_acm_shared_tx_buffer;

	curie_shared_data->cdc_acm_buffers_obj.host_open = false;

	gpio_dev= device_get_binding("GPIO_0");
	gpio_pin_configure(gpio_dev, TXRX_LED, (GPIO_DIR_OUT));
	gpio_pin_write(gpio_dev, TXRX_LED, 1);
}

void cdcacm_setup(void)
{
	uint32_t baudrate, dtr = 0;
	int ret;


	dev = device_get_binding(CONFIG_CDC_ACM_PORT_NAME);

	while (1) {
		uart_line_ctrl_get(dev, LINE_CTRL_DTR, &dtr);
		if (dtr)
			break;
		task_yield();
	}

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(dev, LINE_CTRL_DCD, 1);
	
	acm_tx_state = ACM_TX_READY;
	acm_rx_state = ACM_RX_READY;
	
	ret = uart_line_ctrl_set(dev, LINE_CTRL_DSR, 1);

	enableReboot = true;
	task_yield();

	/* Wait 1 sec for the host to do all settings */
	sys_thread_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);

	uart_irq_callback_set(dev, interrupt_handler);

	curie_shared_data->cdc_acm_buffers_obj.host_open = true;
		
	//reset head and tails values to 0
	curie_shared_data->cdc_acm_shared_rx_buffer.head = 0;
	curie_shared_data->cdc_acm_shared_rx_buffer.tail = 0;
	curie_shared_data->cdc_acm_shared_tx_buffer.head = 0;
	curie_shared_data->cdc_acm_shared_tx_buffer.tail = 0;
	usbSetupDone = true;
}

void baudrate_reset(void)
{
	uint32_t baudrate, ret = 0;
	while(!enableReboot)
	{
		task_yield();
	}
	ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);	
	while(1)
	{
		ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);
		if(baudrate == RESET_BAUD)
		{
			soft_reboot();
		}
		task_sleep(BAUDRATE_RESET_SLEEP);
	}
}

void usb_serial(void)
{
	while(!usbSetupDone)
	{
		task_yield();
	}

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);

	while (1) {
		cdc_acm_tx();
		cdc_acm_rx();
		task_yield();
	}
	
}

