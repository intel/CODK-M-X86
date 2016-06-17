/*
 * Copyright (c) 2015-2016 Wind River Systems, Inc.
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

/**
 * @file C++ Synchronization demo.  Uses basic C++ functionality.
 */

#include <stdio.h>
#include <string.h>
#include <device.h>
#include <zephyr.h>
//#include <reboot.h>

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#include "soc.h"
#include "shared_mem.h"

#define SCSS_SS_CFG_REG (uint32_t*)0xb0800600
#define SCSS_SS_STS_REG (uint32_t*)0xb0800604
#define SCSS_RSTC   (uint32_t*)0xb0800570

#define MMIO_REG_VAL_FROM_BASE(base, offset) (*((volatile uint32_t *)(base+offset)))
#define SCSS_REG_VAL(offset) MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, offset)

#define RSTC_WARM_RESET	(1 << 1)
#define RSTC_COLD_RESET (1 << 3)


void start_arc(unsigned int reset_vector)
{

	if (reset_vector != 0) {
		shared_data->arc_start = reset_vector;
	}

	shared_data->flags = 0;
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
	//start ARC core
	uint32_t *reset_vector;
	reset_vector = (uint32_t *)RESET_VECTOR;
	start_arc(*reset_vector);
}

