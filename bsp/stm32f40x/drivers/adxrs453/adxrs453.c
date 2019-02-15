#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "adxrs453.h"

static rt_device_t spi_device; 
static struct rt_device gyr_device;

#define DIR_READ                (1<<7)
#define DIR_WRITE               (0<<7)
#define ADDR_INCREMENT          (1<<6)

rt_err_t l3g_write_reg(rt_uint8_t reg , rt_uint8_t val)
{
	rt_size_t w_byte;
	rt_uint8_t send_buffer[2];
	
	send_buffer[0] = DIR_WRITE | reg;
	send_buffer[1] = val;
	w_byte = rt_device_write(spi_device , 0 , send_buffer , 2);
	
	return w_byte == sizeof(send_buffer) ? RT_EOK : RT_ERROR;
}

rt_err_t l3g_read_reg(rt_uint8_t reg , rt_uint8_t* buff)
{
	rt_uint8_t send_val , recv_val;
	rt_err_t res;

	send_val = DIR_READ | reg;
	
	res = rt_spi_send_then_recv((struct rt_spi_device *)spi_device , (void*)&send_val , 1 , (void*) &recv_val, 1);
	*buff = recv_val;

	return res;
}

rt_err_t l3g_write_checked_reg(rt_uint8_t reg , rt_uint8_t val)
{
	rt_err_t res = RT_EOK;
	rt_uint8_t r_buff;
	
	res |= l3g_write_reg(reg , val);
	res |= l3g_read_reg(reg , &r_buff);
	
	if(r_buff != val || res!= RT_EOK)
	{
		return RT_ERROR;
	}
	
	return RT_EOK;
}

rt_err_t gyr_init(rt_device_t dev)
{


}
rt_size_t gyr_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{


}
rt_err_t gyr_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{


}
rt_err_t rt_adxrs453_init(char* spi_device_name)
{	
	rt_err_t res = RT_EOK;
	
	/* set device type */
    gyr_device.type    = RT_Device_Class_SPIDevice;
    gyr_device.init    = gyr_init;
    gyr_device.open    = RT_NULL;
    gyr_device.close   = RT_NULL;
    gyr_device.read    = gyr_read;
    gyr_device.write   = RT_NULL;
    gyr_device.control = gyr_control;
    
    /* register to device manager */
    res |= rt_device_register(&gyr_device , "adxrs453", RT_DEVICE_FLAG_RDWR);
	
	spi_device = rt_device_find(spi_device_name);
	
	if(spi_device == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\r\n", spi_device_name);
        return RT_EEMPTY;
    }
	
	/* config spi */
	{
		struct rt_spi_configuration cfg;
		cfg.data_width = 8;
		cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */	
		cfg.max_hz = 4000000;
		
		struct rt_spi_device* spi_device_t = (struct rt_spi_device*)spi_device;
		
		spi_device_t->config.data_width = cfg.data_width;
		spi_device_t->config.mode       = cfg.mode & RT_SPI_MODE_MASK ;
		spi_device_t->config.max_hz     = cfg.max_hz;
		res |= rt_spi_configure(spi_device_t, &cfg);
	}
	
	return res;
}

void adxrs453_entry(void *parameter)
{

	while(1)
	{

		rt_thread_delay(100);

	}

}
