/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <stdio.h>
#include <string.h>
#include <device.h>
#include <uart.h>
#include <zephyr.h>
#include <gpio.h>

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#include "soc.h"
#include "curie_shared_mem.h"

#define SCSS_SS_CFG_REG (uint32_t*)0xb0800600
#define SCSS_SS_STS_REG (uint32_t*)0xb0800604
#define SCSS_RSTC   (uint32_t*)0xb0800570

#define MMIO_REG_VAL_FROM_BASE(base, offset) (*((volatile uint32_t *)(base+offset)))
#define SCSS_REG_VAL(offset) MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, offset)

#define RSTC_WARM_RESET	(1 << 1)
#define RSTC_COLD_RESET (1 << 3)

#define SOFTRESET_INTERRUPT_PIN		0

static volatile bool data_transmitted;
static volatile bool data_arrived = false;


struct device *dev;
bool usbSetupDone = false;

struct gpio_callback cb;



/**
 * Use the following defines just to make the tips of your finger happier.
 */

#define Rx_BUFF curie_shared_data->cdc_acm_shared_rx_buffer.data
#define Rx_HEAD curie_shared_data->cdc_acm_shared_rx_buffer.head
#define Rx_TAIL curie_shared_data->cdc_acm_shared_rx_buffer.tail
#define Tx_BUFF curie_shared_data->cdc_acm_shared_tx_buffer.data
#define Tx_HEAD curie_shared_data->cdc_acm_shared_tx_buffer.head
#define Tx_TAIL curie_shared_data->cdc_acm_shared_tx_buffer.tail
#define SBS     SERIAL_BUFFER_SIZE

/* Make sure BUFFER_LENGTH is not bigger then shared ring buffers */
#define BUFFER_LENGTH		128

#define LOOP_INTERVAL_MS 1
#define USB_ACM_TIMEOUT_MS (250)

#define USB_CONNECTED	    0x04
#define USB_DISCONNECTED    0x05


// buffers
static unsigned char data_buf[128];
static uint8_t write_buffer[BUFFER_LENGTH*2];

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

static volatile uint32_t acm_rx_state = ACM_RX_DISABLED;
static volatile uint32_t acm_tx_state = ACM_TX_DISABLED;

void cdc_acm_tx();
void cdc_acm_rx();
void softResetButton();
void reboot();

static void softReset_button_callback(struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	PRINT("SOFT RESET\r\n");
	reboot();
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
	int timeout = 1000;

	while (data_arrived == false)
	{
		if(!timeout)
			break;
		timeout--;
	}

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
		;

	uart_irq_tx_disable(dev);
}

void start_arc(unsigned int reset_vector)
{

	if (reset_vector != 0) {
		curie_shared_data->arc_start = reset_vector;
	}

	curie_shared_data->flags = 0;
	for(int i = 0; i < 6400000; i++)
	{
	}

	*SCSS_SS_CFG_REG |= ARC_RUN_REQ_A;

	for(int i = 0; i < 6400000; i++)
	{
	}
	
	PRINT("ARC core started\n");
}

void reboot(void)
{
	*SCSS_SS_CFG_REG |= ARC_HALT_REQ_A;
	*SCSS_RSTC = RSTC_WARM_RESET;
}

void main(void)
{
	// setup shared memory pointers for cdc-acm buffers
	curie_shared_data->cdc_acm_buffers_ptr = &curie_shared_data->cdc_acm_buffers;
	curie_shared_data->cdc_acm_buffers.rx_buffer = &curie_shared_data->cdc_acm_shared_rx_buffer;
	curie_shared_data->cdc_acm_buffers.tx_buffer = &curie_shared_data->cdc_acm_shared_tx_buffer;

	softResetButton();

	// start ARC core
	uint32_t *reset_vector;
	reset_vector = (uint32_t *)RESET_VECTOR;
	start_arc(*reset_vector);

	uint32_t baudrate, dtr = 0;
	int ret;


	dev = device_get_binding(CONFIG_CDC_ACM_PORT_NAME);

	PRINT("Wait for DTR\n");
	while (1) {
		uart_line_ctrl_get(dev, LINE_CTRL_DTR, &dtr);
		if (dtr)
			break;
	}
	
	PRINT("DTR set, start test\n");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(dev, LINE_CTRL_DCD, 1);
	if (ret)
		PRINT("Failed to set DCD, ret code %d\n", ret);
	
	acm_tx_state = ACM_TX_READY;
	acm_rx_state = ACM_RX_READY;
	curie_shared_data->cdc_acm_buffers.host_open = true;
	
	ret = uart_line_ctrl_set(dev, LINE_CTRL_DSR, 1);
	if (ret)
		PRINT("Failed to set DSR, ret code %d\n", ret);

	/* Wait 1 sec for the host to do all settings */
	sys_thread_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret)
		PRINT("Failed to get baudrate, ret code %d\n", ret);
	else
		PRINT("Baudrate detected: %d\n", baudrate);

	uart_irq_callback_set(dev, interrupt_handler);
	usbSetupDone = true;
	
	//reset head and tails values to 0
	curie_shared_data->cdc_acm_shared_rx_buffer.head = 0;
	curie_shared_data->cdc_acm_shared_rx_buffer.tail = 0;
	curie_shared_data->cdc_acm_shared_tx_buffer.head = 0;
	curie_shared_data->cdc_acm_shared_tx_buffer.tail = 0;
}

extern "C" void baudrateReset(void)
{
	PRINT("baudrateReset task\r\n");
	uint32_t baudrate, ret = 0;
	while(!usbSetupDone);
	ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);	
	
	while(1)
	{
		ret = uart_line_ctrl_get(dev, LINE_CTRL_BAUD_RATE, &baudrate);
		if(baudrate == 1200)
		{
			reboot();
		}
		task_sleep(100);
	}
}

extern "C" void usbSerialTask(void)
{
	while(!usbSetupDone);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);

	while (1) {
		cdc_acm_tx();
		cdc_acm_rx();
		task_sleep(1);
	}
	
}

void softResetButton()
{
	struct device *aon_gpio;

	aon_gpio = device_get_binding("GPIO_AON_0");
	if (!aon_gpio) 
	{
		PRINT("aon_gpio device not found.\n");
		return;
	}

	gpio_init_callback(&cb, softReset_button_callback, BIT(SOFTRESET_INTERRUPT_PIN));
	gpio_add_callback(aon_gpio, &cb);

	gpio_pin_configure(aon_gpio, SOFTRESET_INTERRUPT_PIN,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE);

	gpio_pin_enable_callback(aon_gpio, SOFTRESET_INTERRUPT_PIN);
}

void cdc_acm_tx()
{
	if (acm_tx_state == ACM_TX_READY) 
	{
		if(Tx_HEAD != Tx_TAIL)
		{
			int cnt = 0, index = Tx_TAIL;
			for (; (index != Tx_HEAD) && (cnt < BUFFER_LENGTH);cnt++) 
			{
				write_buffer[cnt] = Tx_BUFF[index];
				index = (index + 1) % SBS;
			}
			Tx_TAIL= (Tx_TAIL + cnt) % SBS;
			write_data(dev, (const char*)write_buffer, cnt);
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
		if (!curie_shared_data->cdc_acm_buffers.device_open) 
		{
			// ARC is not ready to receive this data - discard it
			write_data(dev, "arc not ready\r\n", 15);
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

