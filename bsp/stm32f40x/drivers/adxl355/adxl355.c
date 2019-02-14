#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "adxl355.h"

#define DIR_READ			0x1
#define DIR_WRITE			0x0

static rt_device_t spi_device;
static struct rt_device acc_device;

static rt_device_t acc_device_t;

static rt_err_t write_reg(rt_uint8_t reg , rt_uint8_t val)
{
	rt_uint8_t send_buffer[2];
	rt_size_t w_byte;
	
	send_buffer[0] = DIR_WRITE | (reg <<1);
	send_buffer[1] = val;
	w_byte = rt_device_write(spi_device , 0 , send_buffer , sizeof(send_buffer));
	
	return w_byte == sizeof(send_buffer) ? RT_EOK : RT_ERROR;
}

static rt_err_t read_reg(rt_uint8_t reg , rt_uint8_t* buff)
{
	rt_uint8_t send_val , recv_val;
	rt_err_t res;

		send_val = DIR_READ | (reg << 1);
	
	res = rt_spi_send_then_recv((struct rt_spi_device *)spi_device , (void*)&send_val , 1 , (void*) &recv_val, 1);
	*buff = recv_val;
	return res;
}

static rt_err_t write_reg_n(rt_uint8_t reg , rt_uint8_t val,rt_uint8_t val2,enWriteData enMode)
{
	rt_uint8_t send_buffer[3];
	rt_size_t w_byte;
	send_buffer[0] = DIR_WRITE | (reg <<1);
	if(enMode == SPI_WRITE_ONE_REG) {

		send_buffer[1] = val;
		w_byte = rt_device_write(spi_device , 0 , send_buffer , 2);
	}
	 if(enMode == SPI_WRITE_TWO_REG) {
		 
		send_buffer[1] = val;
		send_buffer[2] = val2;
		 w_byte = rt_device_write(spi_device , 0 , send_buffer , 3);
	 }
	 
	return w_byte == sizeof(send_buffer) ? RT_EOK : RT_ERROR;
}


static rt_err_t read_reg_n(rt_uint8_t reg , rt_uint8_t val,rt_uint8_t* buff,enRegsNum enMode)
{
	rt_uint8_t r_buff[3];
	rt_err_t res = RT_EOK;
	
	res |= write_reg(reg , val);
	if(enMode == SPI_READ_ONE_REG){
		res |= read_reg(reg , &r_buff[0]);
		if(res != RT_EOK)
		{
			return RT_ERROR;
		}
	}
	
	 if(enMode == SPI_READ_TWO_REG){
		res |= read_reg(reg , &r_buff[0]);
		res |= read_reg(reg , &r_buff[1]);
		if(res != RT_EOK)
		{
			return RT_ERROR;
		}
	 }
	  if(enMode == SPI_READ_THREE_REG){
		res |= read_reg(reg , &r_buff[0]);
		res |= read_reg(reg , &r_buff[1]);
		res |= read_reg(reg , &r_buff[2]);
		if(res != RT_EOK)
		{
			return RT_ERROR;
		}
	 }
	*buff = *r_buff;
	return RT_EOK;

}


static void ADXL355_Start_Sensor(void)
{
		
		uint8_t ui8temp;
    read_reg_n(POWER_CTL, 0x00,&ui8temp,SPI_READ_ONE_REG);       /* Read POWER_CTL register, before modifying it */

    ui8temp = ui8temp & 0xFE;                                       /* Set measurement bit in POWER_CTL register */
	
    read_reg_n(POWER_CTL, ui8temp, 0x00, SPI_READ_ONE_REG);         /* Write the new value to POWER_CTL register */  

}
rt_err_t acc_init(rt_device_t dev)
{	
	rt_err_t res = RT_EOK;
	rt_uint8_t id_ad,id_mst,id_pa;

	rt_device_open(spi_device , RT_DEVICE_OFLAG_RDWR);
	
	do{

 
    ADXL355_Start_Sensor();
		read_reg_n(DEVID_AD,0x00,&id_ad,SPI_READ_ONE_REG);
		read_reg_n(DEVID_MST,0x00,&id_mst,SPI_READ_ONE_REG);
		read_reg_n(PARTID,0x00,&id_pa,SPI_READ_ONE_REG);
		rt_kprintf("ADXL355 ID1: %d,ID2: %d,ID3: %d\n",id_ad,id_mst,id_pa);
    rt_thread_delay(1000);

  }while( id_ad == 0x00 );
	
	return res;
}
rt_err_t acc_read_raw(int16_t acc[4])
{
	rt_err_t res = RT_EOK;
	uint8_t r_val[2];
	
	res |= read_reg_n(TEMP2 ,0x00,r_val, SPI_READ_TWO_REG);
	acc[0] = (int16_t)((r_val[1]<<8) | r_val[0]);
	rt_kprintf("temp %d %d\n", r_val[1], r_val[0]);
	
	res |= read_reg_n(XDATA3 ,0x00,r_val, SPI_READ_THREE_REG);
	acc[1] = (int16_t)((r_val[1]<<8) | r_val[0]);
	rt_kprintf("X %d %d\n", r_val[1], r_val[0]);
	
	res |= read_reg_n(YDATA3 ,0x00, r_val, SPI_READ_THREE_REG);
	acc[2] = (int16_t)((r_val[1]<<8) | r_val[0]);
	rt_kprintf("Y %d %d\n", r_val[1], r_val[0]);
	
	res |= read_reg_n(ZDATA3 , 0x00,r_val,SPI_READ_THREE_REG);
	acc[3] = (int16_t)((r_val[1]<<8) | r_val[0]);
	rt_kprintf("Z %d %d\n", r_val[1], r_val[0]);
	
	rt_kprintf("drive raw acc:%d %d %d\n", acc[0], acc[1], acc[2], acc[3]);
	
	return res;
}

rt_size_t acc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	rt_err_t res = RT_EOK;
	if(pos == 1)	/* read raw acc data */
	{
		res = acc_read_raw(((int16_t*)buffer));
		if(res != RT_EOK)
		{
			return 0;
		}
	}else
	{
		/* unknow pos */
		return 0;
	}
	return size;
}

rt_err_t acc_control(rt_device_t dev, int cmd, void *args)
{
		rt_err_t res = RT_EOK;
	
		switch(cmd)
		{
			case 1:   //设置加速度计量程
			{
			//	res = accel_set_range(*(uint8_t*)args);
			}break;
			
			case 2:  //设置加速度计灵敏度
			{
			//	res = accel_set_samplerate(*(uint32_t*)args);
			}break;
				
			default:
				return RT_ERROR;
		}
		
		return res;
}

rt_err_t rt_adxl355_init(char* spi_device_name)
{	
	rt_err_t res = RT_EOK;
	
	/* set device type */
    acc_device.type    = RT_Device_Class_SPIDevice;
    acc_device.init    = acc_init;
    acc_device.open    = RT_NULL;
    acc_device.close   = RT_NULL;
    acc_device.read    = acc_read;
    acc_device.write   = RT_NULL;
    acc_device.control = acc_control;
    
    /* register to device manager */
    res |= rt_device_register(&acc_device , "adxl355", RT_DEVICE_FLAG_RDWR);
	
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
		cfg.max_hz = 2000000;
		
		struct rt_spi_device* spi_device_t = (struct rt_spi_device*)spi_device;
		
		spi_device_t->config.data_width = cfg.data_width;
		spi_device_t->config.mode       = cfg.mode & RT_SPI_MODE_MASK ;
		spi_device_t->config.max_hz     = cfg.max_hz;
		res |= rt_spi_configure(spi_device_t, &cfg);
	}
	
	return res;
}


rt_err_t sensor_acc_raw_measure(int16_t acc[4])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(acc_device_t, 1, (void*)acc, 8);
	
	return r_byte == 8 ? RT_EOK : RT_ERROR;
}
void adxl355_entry(void *parameter)
{
	rt_err_t res = RT_EOK;
	int16_t acc[4];
	res |= rt_adxl355_init("spi_d1");
  /* init acc device */
	acc_device_t = rt_device_find("adxl355");
	if(acc_device_t == RT_NULL)
	{
		rt_kprintf("can't find acc device\r\n");
		return ;
	}
	rt_device_open(acc_device_t , RT_DEVICE_OFLAG_RDWR);
	while(1)
	{
		sensor_acc_raw_measure(acc);
		rt_thread_delay(1000);
	}



}

