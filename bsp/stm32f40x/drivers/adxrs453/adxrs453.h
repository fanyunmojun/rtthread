#ifndef _ADXRS453_H_
#define _ADXRS453_H_

#define   RATE1   0x00	
#define   RATE0   0x01
#define   TEM1    0x02
#define   TEM0    0x03
#define   LOSCT1  0x04
#define   LOSCT0  0x05
#define   HICST1  0x06
#define   HICST0  0x07
#define   QUAD1   0x08
#define   QUAD0   0x09
#define   FAULT1  0x0A
#define   FAULT0  0x0B
#define   PID1    0x0C
#define   PID0    0x0D
#define   SN3     0x0E
#define   SN2     0x0F
#define   SN1     0x10
#define   SN0     0x11

void adxrs453_entry(void *parameter);

#endif
