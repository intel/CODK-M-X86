/*
 * Copyright (c) 2017 Intel Corporation
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

#include <zephyr.h>
#include <misc/printk.h>
#include <gpio.h>
#include "soc.h"

#include "sharedmemory_com.h"
#include "soc_ctrl.h"
#include "cdcacm_serial.h"
#include "curie_shared_mem.h"

#define SOFTRESET_INTERRUPT_PIN		0

/* size of stack area used by each thread */
#define MAIN_STACKSIZE      2048
#define CDCACM_STACKSIZE    384
#define RESET_STACKSIZE     384
#define USBSERIAL_STACKSIZE 384

/* scheduling priority used by each thread */
#define PRIORITY 7
#define TASK_PRIORITY 8

char __noinit __stack cdcacm_setup_stack_area[CDCACM_STACKSIZE];
char __noinit __stack baudrate_reset_stack_area[RESET_STACKSIZE];
char __noinit __stack usb_serial_stack_area[USBSERIAL_STACKSIZE];

struct gpio_callback cb;

void softResetButton();

static void softReset_button_callback(struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	soft_reboot();
}

void threadMain(void *dummy1, void *dummy2, void *dummy3)
{

	init_cdc_acm();
	softResetButton();
    
	//start ARC core
	uint32_t *reset_vector;
	reset_vector = (uint32_t *)RESET_VECTOR;
	start_arc(*reset_vector);

	k_thread_spawn(cdcacm_setup_stack_area, CDCACM_STACKSIZE, cdcacm_setup, NULL, NULL,
			NULL, TASK_PRIORITY, 0, K_NO_WAIT);
	
	k_thread_spawn(baudrate_reset_stack_area, RESET_STACKSIZE, baudrate_reset, NULL, NULL,
			NULL, TASK_PRIORITY, 0, K_NO_WAIT);
	
	k_thread_spawn(usb_serial_stack_area, USBSERIAL_STACKSIZE, usb_serial, NULL, NULL,
			NULL, TASK_PRIORITY, 0, K_NO_WAIT);

}

void softResetButton()
{
	struct device *aon_gpio;
	char* gpio_aon_0 = (char*)"GPIO_1";
	aon_gpio = device_get_binding(gpio_aon_0);
	if (!aon_gpio) 
	{
		return;
	}

	gpio_init_callback(&cb, softReset_button_callback, BIT(SOFTRESET_INTERRUPT_PIN));
	gpio_add_callback(aon_gpio, &cb);

	gpio_pin_configure(aon_gpio, SOFTRESET_INTERRUPT_PIN,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE);

	gpio_pin_enable_callback(aon_gpio, SOFTRESET_INTERRUPT_PIN);
}

K_THREAD_DEFINE(threadMain_id, MAIN_STACKSIZE, threadMain, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);

