/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2011-06-05     Bernard      modify for STM32F107 version
 */

#include <rthw.h>
#include <rtthread.h>
#include "adxl355.h"
#include "adxrs453.h"
/**
 * @addtogroup STM32
 */

/*@{*/

static char thread_adxl355_stack[2048];
struct rt_thread thread_adxl355_handle;


static char thread_adxrs453_stack[2048];
struct rt_thread thread_adxrs453_handle;

int main(void)
{
    /* user app entry */
	rt_err_t res;
	
	res = rt_thread_init(&thread_adxl355_handle,
						   "adxl355",
						   adxl355_entry,
						   RT_NULL,
						   &thread_adxl355_stack[0],
						   sizeof(thread_adxl355_stack),10,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_adxl355_handle);
	
		res = rt_thread_init(&thread_adxrs453_handle,
						   "adxrs453",
						   adxrs453_entry,
						   RT_NULL,
						   &thread_adxrs453_stack[0],
						   sizeof(thread_adxrs453_stack),10,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_adxrs453_handle);
	
	
    return 0;
}

/*@}*/
