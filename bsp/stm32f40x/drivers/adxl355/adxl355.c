#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "adxl355.h"

#define DIR_READ			0x1
#define DIR_WRITE			0x0

static rt_device_t spi_device;	//定义驱动层SPI设备

static struct rt_device acc_device;		//定义驱动层acc设备

static rt_device_t acc_device_t;		//应用层 acc 设备

/* 寄存器读取函数 */
static rt_err_t write_reg(rt_uint8_t reg , rt_uint8_t val)
{
	rt_uint8_t send_buffer[2];
	rt_size_t w_byte;
	send_buffer[0] = DIR_WRITE | (reg <<1);
	send_buffer[1] = val;
	w_byte = rt_device_write(spi_device , 0 , send_buffer , sizeof(send_buffer));
	return w_byte == sizeof(send_buffer) ? RT_EOK : RT_ERROR;
}
static rt_err_t write_reg_n(rt_uint8_t reg , rt_uint8_t val1, rt_uint8_t val2)
{
	rt_uint8_t send_buffer[3];
	rt_size_t w_byte;
	send_buffer[0] = DIR_WRITE | (reg <<1);
	send_buffer[1] = val1;
	send_buffer[2] = val2;
	w_byte = rt_device_write(spi_device , 0 , send_buffer , sizeof(send_buffer));
	return w_byte == sizeof(send_buffer) ? RT_EOK : RT_ERROR;
}

static void read_reg(rt_uint8_t reg, rt_uint8_t *readedData) 
{
		rt_uint8_t send_val[2] , recv_val;
    send_val[0] = DIR_READ | (reg << 1);
	//rt_device_write(spi_device , 0 , &send_val , 1);
		send_val[1] = 0x00;
		rt_spi_send_then_recv((struct rt_spi_device *)spi_device , (void*)&send_val , 2 , (void*) &recv_val, 1);
		*readedData = recv_val;
}
static void read_reg_n(rt_uint8_t *reg, rt_uint8_t dataSize, rt_uint8_t *readedData) {

		for(int i = 0; i < dataSize; i = i + 1) {
		rt_uint8_t send_val , recv_val;
    send_val = DIR_READ | (reg[i] << 1);
		rt_device_write(spi_device , 0 , &send_val , 1);
		send_val = 0x00;
		rt_spi_send_then_recv((struct rt_spi_device *)spi_device , (void*)&send_val , 1 , (void*) &recv_val, 1);
		readedData[i] = recv_val;
		}
}

static void ADXL355_Start_Sensor(void)
{
	  rt_uint8_t r_byte;
		rt_thread_delay(100);
		read_reg(DEVID_AD,&r_byte);				//读取DEVID_AD ID
		rt_kprintf("\nID: %d\n",r_byte);		
	
//		  write_reg(RANGE,RANGE_2G);			//设置量程 2G
//			read_reg(POWER_CTL,&r_byte);		//读取电源控制寄存器
//			r_byte = r_byte & 0xFE; 				//配置电源控制寄存器开始测量
//			write_reg(POWER_CTL,r_byte);		//设置电源控制寄存器
				
		rt_thread_delay(100);
	
}

/* 驱动层-传感器初始化函数*/
rt_err_t acc_init(rt_device_t dev)
{	
	rt_err_t res = RT_EOK;
	
	res = rt_device_open(spi_device , RT_DEVICE_OFLAG_RDWR);
	if(res != RT_EOK)
	{
		rt_kprintf("spi device not open!\n");
	}
	ADXL355_Start_Sensor();

	return res;
}
rt_err_t acc_read_raw(int16_t acc[5])
{
	rt_err_t res = RT_EOK;
	rt_uint8_t axisAddresses[] = {TEMP1,TEMP2,XDATA1, XDATA2, XDATA3, YDATA1, YDATA2, YDATA3, ZDATA1, ZDATA2, ZDATA3};
  rt_uint8_t axisMeasures[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	uint8_t r_val[2];
	read_reg_n(axisAddresses,sizeof(axisAddresses),axisMeasures);
	acc[0] = (int16_t)((axisMeasures[1]<<8) | axisMeasures[0]);
	acc[1] = (int16_t)((axisMeasures[3]<<8) | axisMeasures[2]);
	acc[2] = (int16_t)((axisMeasures[5]<<8) | axisMeasures[4]);
	acc[3] = (int16_t)((axisMeasures[7]<<8) | axisMeasures[6]);
	acc[4] = (int16_t)((axisMeasures[9]<<8) | axisMeasures[10]);
	
//	rt_kprintf("drive raw acc:%d %d %d\n", acc[0], acc[1], acc[2], acc[3],acc[4]);
	
	return res;
}


/*驱动层-读取函数*/
rt_size_t acc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	rt_err_t res = RT_EOK;
	if(pos == 1)	/* 读取传感器原始数据 */
	{
		
	//	res = acc_read_raw(((int16_t*)buffer));
		if(res != RT_EOK)
		{
			return 0;
		}
	}else
	{
		/* 不知到标识 */
		return 0;
	}
	return size;
}

/* 驱动层-命令控制函数*/
rt_err_t acc_control(rt_device_t dev, int cmd, void *args)
{
		rt_err_t res = RT_EOK;
	
		switch(cmd)
		{
			case 1:   //设置acc控制量程
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
/* 驱动层-驱动注册函数*/
rt_err_t rt_adxl355_init(char* spi_device_name)
{	
		rt_err_t res = RT_EOK;
	
	/* 设置设备类型 */
    acc_device.type    = RT_Device_Class_SPIDevice;	//设备类型为SPI
    acc_device.init    = acc_init;		
    acc_device.open    = RT_NULL;
    acc_device.close   = RT_NULL;
    acc_device.read    = acc_read;
    acc_device.write   = RT_NULL;
    acc_device.control = acc_control;
    
    /* 向设备管理器注册 adxl355*/
    res |= rt_device_register(&acc_device , "adxl355", RT_DEVICE_FLAG_RDWR);
	
		spi_device = rt_device_find(spi_device_name);	//查找设备名称
	
		if(spi_device == RT_NULL)		//失败处理
    {
			
        rt_kprintf("spi device %s not found!\r\n", spi_device_name);
        return RT_EEMPTY;
			
    }
	
		/* 配置SPI */
		{
			struct rt_spi_configuration cfg;
			cfg.data_width = 8;
			cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI 兼容模式3 */	
			cfg.max_hz = 2000000;	//最大运行频率为 2MHZ
			
			struct rt_spi_device* spi_device_t = (struct rt_spi_device*)spi_device;
			
			spi_device_t->config.data_width = cfg.data_width;
			spi_device_t->config.mode       = cfg.mode & RT_SPI_MODE_MASK ;
			spi_device_t->config.max_hz     = cfg.max_hz;
			res |= rt_spi_configure(spi_device_t, &cfg);
		}
		
		return res;
}

/*应用层读取传感器原始数据函数*/
rt_err_t sensor_acc_raw_measure(int16_t acc[4])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(acc_device_t, 1, (void*)acc, 10);	// 1：代表读取原始数据	10个字节
	
	return r_byte == 8 ? RT_EOK : RT_ERROR;
}

/*adxl355 线程*/
void adxl355_entry(void *parameter)
{
	rt_err_t res = RT_EOK;
	
	int16_t acc[4];														//存储temp,X,Y,Z 原始数据
	
	res |= rt_adxl355_init("spi_d2");					//初始化注册adxl355驱动程序
  
	acc_device_t = rt_device_find("adxl355"); //应用层查找adxl355设备
	
	if(acc_device_t == RT_NULL)								//失败处理
	{
	
		rt_kprintf("can't find acc device\r\n");
		return ;

	}
	
	rt_device_open(acc_device_t , RT_DEVICE_OFLAG_RDWR); //打开adxl355设备

	while(1)
	{
		
		sensor_acc_raw_measure(acc);	//读取传感器数据
		rt_thread_delay(1000);	//延时1ms
	
	}



}

