/*
 * File      : ms5611_sensor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2016-6-30      zoujiachi   	the first version
 */
 
#ifndef __MS5611_SENSOR_H__
#define __MS5611_SENSOR_H__

#include "stm32f4xx.h"
#include <rtthread.h>

typedef struct
{
	u16 factory_data;
	u16 c1;
	u16 c2;
	u16 c3;
	u16 c4;
	u16 c5;
	u16 c6;
	u16 crc;
}MS5611_PROM_Def;

typedef struct
{
	u32 raw_temperature;
	u32 raw_pressure;
	float temperature;
	float pressure;
	float altitude;
	u32 time_stamp;
}MS5611_REPORT_Def;

#define RAW_TEMPERATURE_POS			0
#define RAW_PRESSURE_POS			1
#define COLLECT_DATA_POS			2
//baro cmd
#define SENSOR_CONVERSION			0x30
#define SENSOR_IS_CONV_FIN			0x31

typedef enum
{
	S_CONV_1 = 0,
	S_RAW_PRESS,
	S_CONV_2,
	S_RAW_TEMP,
	S_COLLECT_REPORT
}Baro_Machine_State;



void ms5611_entry(void *parameter);
#endif
