/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#include "drv_i2c.h" 

#include "fsl_common.h" 
#include "fsl_iocon.h"
#include "fsl_i2c.h"

struct lpc_i2c
{
    struct rt_i2c_bus_device bus; 
    
    I2C_Type *base;
    
    char *device_name;
};

static uint32_t get_i2c_freq(I2C_Type *base)
{
    uint32_t freq = 0;

#if defined(BSP_USING_I2C4)
    if(base == I2C4)
    {
        freq = CLOCK_GetFreq(kCLOCK_Flexcomm4);
    }
#endif 

    return freq; 
}

static rt_size_t master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    rt_size_t ret = (0);

    rt_uint32_t index = 0;
    struct lpc_i2c *lpc_i2c = RT_NULL;
    struct rt_i2c_msg *msg = RT_NULL;
    i2c_master_transfer_t xfer = {0};

    RT_ASSERT(bus != RT_NULL); 

    lpc_i2c = (struct lpc_i2c *)bus;

    for(index = 0; index < num; index++)
    {
        msg = &msgs[index];
        
        xfer.slaveAddress   = msg->addr;
        xfer.flags          = kI2C_TransferDefaultFlag;
        xfer.subaddress     = 0;
        xfer.subaddressSize = 1;
        xfer.data           = msg->buf;
        xfer.dataSize       = msg->len;

        if (msg->flags & RT_I2C_RD)
        {
            xfer.direction  = kI2C_Read;
        }
        else
        {
            xfer.direction  = kI2C_Write;
        }
        
        if(I2C_MasterTransferBlocking(lpc_i2c->base, &xfer) != kStatus_Success)
        {
            rt_kprintf("i2c bus write failed, i2c bus stop!\n");
            goto exit;
        }
    }

    ret = index;

exit:
    return ret;
}

static rt_size_t slave_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    return 0;
}

static rt_err_t i2c_bus_control(struct rt_i2c_bus_device *bus, rt_uint32_t cmd, rt_uint32_t arg)
{
    return RT_EOK;
}

static const struct rt_i2c_bus_device_ops ops =
{
    master_xfer, 
    slave_xfer,
    i2c_bus_control,
}; 

#if defined(BSP_USING_I2C4)
static struct lpc_i2c i2c4 = {0}; 
#endif

int rt_hw_i2c_init(void)
{
    CLOCK_EnableClock(kCLOCK_Iocon);

#if defined(BSP_USING_I2C4)
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);
    
    i2c4.base = I2C4;
    i2c4.device_name = "i2c4"; 
    i2c4.bus.ops = &ops; 
    
    IOCON_PinMuxSet(IOCON, 1, 1, IOCON_MODE_PULLUP | IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
    IOCON_PinMuxSet(IOCON, 1, 2, IOCON_MODE_PULLUP | IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

    i2c_master_config_t masterConfig;

    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 100*1000U;
    
    I2C_MasterInit(I2C4, &masterConfig, get_i2c_freq(I2C4));

    rt_i2c_bus_device_register(&i2c4.bus, i2c4.device_name);
#endif

    return RT_EOK; 
}
INIT_BOARD_EXPORT(rt_hw_i2c_init); 
