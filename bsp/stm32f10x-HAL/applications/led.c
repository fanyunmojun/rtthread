#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/pin.h>
#include "led.h"

static rt_device_t led_device_t;  

rt_uint8_t device_led_init(void)
{
		
		struct rt_device_pin_mode mode = {LED_PIN , PIN_MODE_OUTPUT };
		led_device_t = rt_device_find("pin");			//查找pin设备
		if(led_device_t == RT_NULL)								//失败处理
		{
			rt_kprintf("can't find pin device\r\n");
			return 1;
		}
		rt_device_open(led_device_t , RT_DEVICE_OFLAG_RDWR);	//打开pin设保,权限可读可写
		led_device_t->control(led_device_t , 0 , &mode);		//控制函数	0 代表 pos
		return 0;
}
void led_on(void)
{
	struct rt_device_pin_status pin_sta = {LED_PIN , 0};
	
	if(led_device_t != RT_NULL)
	{
		led_device_t->write(led_device_t, 0, (void*)&pin_sta, sizeof(&pin_sta));
	}
}

void led_off(void)
{
	struct rt_device_pin_status pin_sta = {LED_PIN , 1};;
	
	if(led_device_t != RT_NULL)
	{
		led_device_t->write(led_device_t, 0, (void*)&pin_sta, sizeof(&pin_sta));
	}
}

void led_entry(void *parameter)
{
	rt_uint8_t res;
	res = device_led_init();
	if(res)
	{
		rt_kprintf("led init err !");
		return ;
	}
	
	while(1)
	{
		led_on();
		rt_thread_delay(1000);
		led_off();
		rt_thread_delay(1000);
	}

}
