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
#include <gpio.h>
#include <stdint.h>
#include <fault_led.h>
#include "curie_shared_mem.h"

#define FAULT_LED	26

#define BLINK_DELAY	250
#define CHECK_DELAY	3000

struct device *gpio_dev;

int led_state = 0;

void check_arc_error(void *dummy1, void *dummy2, void *dummy3)
{
	while(1)
	{
		uint8_t error_code = curie_shared_data->error_code;
		if(error_code)
			blink_fault(error_code);

		k_sleep(CHECK_DELAY);
	}
}

void blink_fault(int error_code)
{
	gpio_dev= device_get_binding("GPIO_0");
	gpio_pin_configure(gpio_dev, FAULT_LED, (GPIO_DIR_OUT));
	if(error_code == 1)
	{
		gpio_pin_write(gpio_dev, FAULT_LED, 0);	//turn TXRX led on
		return;
	}
	for(int i = 0; i < error_code; i++)
	{
		
		gpio_pin_write(gpio_dev, FAULT_LED, 0);	//turn TXRX led on
		k_sleep(BLINK_DELAY);
		gpio_pin_write(gpio_dev, FAULT_LED, 1);	//turn TXRX led off
		k_sleep(BLINK_DELAY);
	}
}
