/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>
#include "led.h"
static char thread_led_stack[512];
struct rt_thread thread_led_handle;

int main(void)
{
	rt_err_t res;
	res = rt_thread_init(&thread_led_handle,
						   "led",
						   led_entry,
						   RT_NULL,
						   &thread_led_stack[0],
						   sizeof(thread_led_stack),20,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_led_handle);
    /* user app entry */
    return 0;
}
