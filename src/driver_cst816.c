#include "driver_cst816.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

// clang-format off
/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "Hynitron Microelectronics CST816"    /**< chip name */
#define MANUFACTURER_NAME         "Hynitron Microelectronics"           /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        2.8f                            		/**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        3.6f                             		/**< chip max supply voltage */
#define MAX_CURRENT               0.2f                            		/**< chip max current */
#define TEMPERATURE_MIN           -40.0f                          		/**< chip min operating temperature */
#define TEMPERATURE_MAX           125.0f                            	/**< chip max operating temperature */
#define DRIVER_VERSION            2000                             		/**< driver version */


/**
 * @brief chip register definition
 */
#define CST816_REG_GestureID		0x01        /**< gesture ID register */
#define CST816_REG_FingerNum		0x02        /**< finger number register */
#define CST816_REG_XposH			0x03        /**< X position high byte register */
#define CST816_REG_XposL			0x04        /**< X position low byte register */
#define CST816_REG_YposH			0x05        /**< Y position high byte register */
#define CST816_REG_YposL			0x06        /**< Y position low byte register */
#define CST816_REG_ChipID			0xA7        /**< chip ID register */
#define CST816_REG_ProjID			0xA8        /**< project ID register */
#define CST816_REG_FwVersion		0xA9        /**< firmware version register */
#define CST816_REG_FactoryID		0xAA        /**< factory ID register */

// #define CST816_REG_BPC0H			0xB0         /**< BPC0 high byte register */
// #define CST816_REG_BPC0L			0xB1		 /**< BPC0 low byte register */
// #define CST816_REG_BPC1H 	  		0xB2         /**< BPC1 high byte register */
// #define CST816_REG_BPC1L 	  		0xB3		 /**< BPC1 low byte register */

#define CST816_REG_SleepMode        0xE5        /**< sleep mode register */
#define CST816_REG_ErrResetCtl      0xEA	    /**< error reset control register */
#define CST816_REG_LongPressTick	0xEB		/**< long press ticks register */
#define CST816_REG_MotionMask		0xEC        /**< motion mask register */
#define CST816_REG_IrqPluseWidth	0xED        /**< irq pulse width register */
#define CST816_REG_NorScanPer		0xEE        /**< normal scan period register */
#define CST816_REG_MotionSlAnggle	0xEF        /**< motion sl angle register */

// #define CST816_REG_LpScanRaw1H		0xF0
// #define CST816_REG_LpScanRaw1L		0xF1
// #define CST816_REG_LpScanRaw2H		0xF2
// #define CST816_REG_LpScanRaw2L		0xF3
#define CST816_REG_LpAutoWakeTime	0xF4
#define CST816_REG_LpScanTH			0xF5
#define CST816_REG_LpScanWindow		0xF6
#define CST816_REG_LpScanFreq		0xF7
#define CST816_REG_LpScanIdac		0xF8
#define CST816_REG_AutoSleepTime	0xF9        /**< auto sleep time register */
#define CST816_REG_IrqCtl			0xFA        /**< irq control register */
#define CST816_REG_AutoReset		0xFB        /**< auto reset register */
#define CST816_REG_LongPressTime	0xFC        /**< auto sleep register */
#define CST816_REG_IOCtl			0xFD        /**< long press time register */
#define CST816_REG_DisAutoSleep		0xFE        /**< disable auto sleep register */


#define CST816_ADDRESS            (0x15<<1)        /**< iic device address */
// clang-format on

/**
 * @brief      read bytes
 * @param[in]  *handle pointer to an cst816 handle structure
 * @param[out] *data pointer to a data buffer
 * @param[in]  len length of data
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
static uint8_t a_cst816_iic_read(cst816_handle_t *handle, uint8_t addr, uint8_t *buf, uint16_t len)
{
	if (handle->iic_read(CST816_ADDRESS, addr, buf, len) != 0) /* read the register */
	{
		return 1; /* return error */
	}
	else
	{
		return 0; /* success return 0 */
	}
}

/**
 * @brief     write bytes
 * @param[in] *handle pointer to an cst816 handle structure
 * @param[in] *data pointer to a data buffer
 * @param[in] len length of data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
static uint8_t a_cst816_iic_write(cst816_handle_t *handle, uint8_t addr, uint8_t *buf, uint16_t len)
{
	if (handle->iic_write(CST816_ADDRESS, addr, buf, len) != 0) /* write the register */
	{
		return 1; /* return error */
	}
	else
	{
		return 0; /* success return 0 */
	}
}

/**
 * @brief     initialize the chip
 * @param[in] *handle pointer to an ssd1306 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic or spi initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 reset failed
 *            - 5 command && data init failed
 *            - 6 interface param is invalid
 * @note      none
 */
uint8_t cst816_init(cst816_handle_t *handle)
{
	if (handle == NULL)                                                             /* check handle */
    {
        return 2;                                                                   /* return error */
    }
    if (handle->debug_print == NULL)                                                /* check debug_print */
    {
        return 3;                                                                   /* return error */
    }
    if (handle->iic_init == NULL)                                                   /* check iic_init */
    {
        handle->debug_print("ssd1306: iic_init is null.\n");                        /* iic_init is null */
        
        return 3;                                                                   /* return error */
    }
    if (handle->iic_deinit == NULL)                                                 /* check iic_deinit */
    {
        handle->debug_print("ssd1306: iic_deinit is null.\n");                      /* iic_deinit is null */
       
        return 3;                                                                   /* return error */
    }
    if (handle->iic_write == NULL)                                                  /* check iic_write */
    {
        handle->debug_print("ssd1306: iic_write is null.\n");                       /* iic_write is null */
       
        return 3;                                                                   /* return error */
    }
    if (handle->delay_ms == NULL)                                                   /* check delay_ms */
    {
        handle->debug_print("ssd1306: delay_ms is null.\n");                        /* delay_ms is null */
        
        return 3;                                                                   /* return error */
    }
    if (handle->reset_gpio_init == NULL)                                            /* check reset_gpio_init */
    {
        handle->debug_print("ssd1306: reset_gpio_init is null.\n");                 /* reset_gpio_init is null */
        
        return 3;                                                                   /* return error */
    }
    if (handle->reset_gpio_deinit == NULL)                                          /* check reset_gpio_deinit */
    {
        handle->debug_print("ssd1306: reset_gpio_deinit is null.\n");               /* reset_gpio_deinit is null */
        
        return 3;                                                                   /* return error */
    }
    if(handle->reset_gpio_write == NULL)                                            /* check reset_gpio_write */
    {
        handle->debug_print("ssd1306: reset_gpio_write is null.\n");                /* reset_gpio_write is null */ 
        
        return 3;                                                                   /* return error */
    }
	if (handle->reset_gpio_init() != 0)                                             /* reset gpio init */
    {
        handle->debug_print("ssd1306: reset gpio init failed.\n");                  /* reset gpio init failed */
          
        return 4;                                                                   /* return error */
    }
	if (handle->reset_gpio_write(1) != 0)                                           /* write 1 */
    {
        handle->debug_print("ssd1306: reset gpio write failed.\n");                 /* reset gpio write failed */
        (void)handle->reset_gpio_deinit();                                          /* reset_gpio_deinit */
        
        return 4;                                                                   /* return error */
    }
	handle->delay_ms(100);														    /* delay 100 ms */
    if (handle->reset_gpio_write(0) != 0)                                           /* write 0 */
    {
        handle->debug_print("ssd1306: reset gpio write failed.\n");                 /* reset gpio write failed */
        (void)handle->reset_gpio_deinit();                                          /* reset_gpio_deinit */
        
        return 4;                                                                   /* return error */
    }
    handle->delay_ms(100);                                                          /* delay 100 ms */
    if (handle->reset_gpio_write(1) != 0)                                           /* write 1 */
    {
        handle->debug_print("ssd1306: reset gpio write failed.\n");                 /* reset gpio write failed */
        (void)handle->reset_gpio_deinit();                                          /* reset_gpio_deinit */
        
        return 4;                                                                   /* return error */
    }
	handle->delay_ms(100);

	if (handle->iic_init() != 0)                                                	/* iic init */
	{
		handle->debug_print("ssd1306: iic init failed.\n");                     	/* iic init failed */
		(void)handle->reset_gpio_deinit();                                      	/* reset_gpio_deinit */
		
		return 1;                                                               	/* return error */
	}
	handle->inited = 1;                                                             /* flag inited */
    return 0;                                                                       /* success return 0 */
}

/**
 * @brief     close the chip
 * @param[in] *handle pointer to an ssd1306 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic or spi deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 power down failed
 *            - 5 reset gpio deinit failed
 *            - 6 command && data deinit failed
 *            - 7 interface param is invalid
 * @note      none
 */
uint8_t cst816_deinit(cst816_handle_t *handle)
{
    
    if (handle == NULL)                                                              /* check handle */
    {
        return 2;                                                                    /* return error */
    }
    if (handle->inited != 1)                                                         /* check handle initialization */
    {
        return 3;                                                                    /* return error */
    }
    
    
    if (handle->reset_gpio_deinit() != 0)                                            /* reset gpio deinit */
    {
        handle->debug_print("ssd1306: reset gpio deinit failed.\n");                 /* reset gpio deinit failed */
            
        return 5;                                                                    /* return error */
    }
    
    handle->inited = 0;                                                              /* flag close */
    
    return 0;                                                                        /* success return 0 */
}



/**
 * @brief 获取CST816触摸手势
 * @param[in] handle CST816设备句柄，必须已初始化
 * @param[out] gesture 指向手势数据存储位置的指针
 * @return uint8_t 返回状态码
 *         - 0: 成功获取手势数据
 *         - 1: I2C读取失败
 *         - 2: 句柄为空
 *         - 3: 设备未初始化
 */
uint8_t cst816_get_gesture(cst816_handle_t *handle, CST816_gesture_t *gesture)
{
	if (handle == NULL)                                                              /* check handle */
	{
		return 2;                                                                    /* return error */
	}
	if (handle->inited != 1)                                                         /* check handle initialization */
	{
		return 3;                                                                    /* return error */
	}

	return a_cst816_iic_read(handle, CST816_REG_GestureID, gesture, 1);
}

uint8_t cst816_get_fingernum(cst816_handle_t *handle, uint8_t *num)
{ 
    if (handle == NULL)                                                              /* check handle */
    {
        return 2;                                                                    /* return error */
    }
    if (handle->inited != 1)                                               /* check handle initialization */
    {
        return 3;                                                                    /* return error */
    }
    return a_cst816_iic_read(handle, CST816_REG_FingerNum, num, 1);
}

// 仅低八位有效
uint8_t cst816_get_pos(cst816_handle_t *handle, cst816_pos_t *pos)
{
    uint8_t buf[4];
    if (handle == NULL)                                                              /* check handle */
    {
        return 2;                                                                    /* return error */
    }
    if (handle->inited != 1)                                                         /* check handle initialization */
    {
        return 3;                                                                    /* return error */
    }
    if (a_cst816_iic_read(handle, CST816_REG_XposH, buf, 4) != 0)                   /* read pos */
    {
        handle->debug_print("cst816: read pos failed.\n");                          /* read pos failed */

        return 1;                                                                    /* return error */
    }
    // 根据大小端进行转换
    pos->u8_data.x_h = buf[0];
    pos->u8_data.x_l = buf[1];
    pos->u8_data.y_h = buf[2];
    pos->u8_data.y_l = buf[3];
    return 0;
}


/**
 * @brief 获取CST816芯片ID
 * @param[in] handle CST816设备句柄，必须已初始化
 * @param[out] chipid 指向芯片ID存储位置的指针
 * @return uint8_t 返回状态码
 *         - 0: 成功获取芯片ID
 *         - 1: I2C读取失败
 *         - 2: 句柄为空
 *         - 3: 设备未初始化
 */
uint8_t cst816_get_chipID(cst816_handle_t *handle, uint8_t *chipid)
{
	if (handle == NULL)                                                             /* check handle */
	{
		return 2;                                                                   /* return error */
	}
	if (handle->inited != 1)                                                        /* check handle initialization */
	{
		return 3;                                                                   /* return error */
	}

	if (a_cst816_iic_read(handle, CST816_REG_ChipID, chipid, 1) != 0)               /* read chip id */
	{
		handle->debug_print("cst816: read chip id failed.\n");                      /* read chip id failed */

		return 1;                                                                   /* return error */
	}
	
	return 0;                                                                    	/* success return 0 */
}


/**
 * @brief 进入休眠状态，无触摸唤醒功能，休眠后无法读取触摸数据，需复位唤醒
 * @param[in] handle CST816设备句柄，必须已初始化
 * @param[out] mode 1: 休眠模式，0：唤醒模式
 * @return uint8_t 返回状态码
 *         - 0: 成功设置休眠模式
 *         - 1: I2C读取失败
 *         - 2: 句柄为空
 *         - 3: 设备未初始化
 */
uint8_t cst816_set_sleep_mode(cst816_handle_t *handle, CST816_sleep_mode_t mode)
{
    if (handle == NULL)                                                             /* check handle */
    {
        return 2;                                                                   /* return error */
    }
    if (handle->inited != 1)                                                        /* check handle initialization */
    {
        return 3;                                                                   /* return error */
    }

    return a_cst816_iic_write(handle, CST816_REG_SleepMode, &mode, 1);
}

uint8_t cst816_get_sleep_mode(cst816_handle_t *handle, CST816_sleep_mode_t *mode)
{ 
    if (handle == NULL)                                                             /* check handle */
    {
        return 2;                                                                   /* return error */
    }
    if (handle->inited != 1)                                                        /* check handle initialization */
    {
        return 3;                                                                   /* return error */
    }
    return a_cst816_iic_read(handle, CST816_REG_SleepMode, mode, 1);
}

uint8_t cst816_set_reset_mode(cst816_handle_t *handle,CST816_reset_mode_t mode)
{
    if (handle == NULL)                                                             /* check handle */
    {
        return 2;                                                                   /* return error */
    }
    if (handle->inited != 1)                                                        /* check handle initialization */
    {
        return 3;                                                                   /* return error */
    }
    return a_cst816_iic_write(handle, CST816_REG_ErrResetCtl, &mode, 1);
}

// 没用
uint8_t cst816_get_reset_mode(cst816_handle_t *handle, CST816_reset_mode_t *mode)
{ 
    if (handle == NULL)                                                             /* check handle */
    {
        return 2;                                                                   /* return error */
    }
    if (handle->inited != 1)                                                        /* check handle initialization */
    {
        return 3;                                                                   /* return error */
    }
    return a_cst816_iic_read(handle, CST816_REG_ErrResetCtl, mode, 1);
}

// 单位 10ms
uint8_t cst816_set_long_press_tick(cst816_handle_t *handle, uint8_t time)
{
    return a_cst816_iic_write(handle, CST816_REG_LongPressTick, &time, 1);
}

uint8_t cst816_get_long_press_tick(cst816_handle_t *handle, uint8_t *time)
{
    return a_cst816_iic_read(handle, CST816_REG_LongPressTick, time, 1);
}

uint8_t cst816_set_motion_mask(cst816_handle_t *handle, uint8_t mask)
{
    return a_cst816_iic_write(handle, CST816_REG_MotionMask, &mask, 1);
}

// 设置中断低脉冲宽度，单位 1ms
uint8_t cst816_set_irq_pluse_width(cst816_handle_t *handle, uint8_t width)
{
    return a_cst816_iic_write(handle, CST816_REG_IrqPluseWidth, &width, 1);
}

// 获取中断低脉冲宽度，单位 1ms
uint8_t cst816_get_irq_pluse_width(cst816_handle_t *handle, uint8_t *width)
{
    return a_cst816_iic_read(handle, CST816_REG_IrqPluseWidth, width, 1);
}

// 获取手势角度阈值
uint8_t cst816_get_motion_sl_angle(cst816_handle_t *handle, uint8_t *angle)
{
    return a_cst816_iic_read(handle, CST816_REG_MotionSlAnggle, angle, 1);
}

// 设置中断控制
uint8_t cst816_set_irq_ctl(cst816_handle_t *handle, cst816_irq_ctl_t ctl)
{
    return a_cst816_iic_write(handle, CST816_REG_IrqCtl, &ctl, 1);
}

// 设置自动复位时间 单位 1s ，x秒内有触摸但无有效手势时，自动复位。
uint8_t cst816_set_auto_resset_time(cst816_handle_t *handle, uint8_t time)
{
    return a_cst816_iic_write(handle, CST816_REG_AutoReset, &time, 1);
}

// 单位 1s
uint8_t cst816_set_long_press_time(cst816_handle_t *handle, uint8_t time)
{
    return a_cst816_iic_write(handle, CST816_REG_LongPressTime, &time, 1);
}


uint8_t cst816_set_IO_ctl(cst816_handle_t *handle, CST816_IO_ctl_mode_t mode, CST816_IO_ctl_voltage_t voltage)
{
    uint8_t data = mode | voltage;
    return a_cst816_iic_write(handle, CST816_REG_LongPressTime, &data, 1);
}

uint8_t cst816_soft_reset(cst816_handle_t *handle)
{
    uint8_t data = CST816_IO_CTL_SOFT_RESET_ENABLE;
    return a_cst816_iic_write(handle, CST816_REG_LongPressTime, &data, 1);
}





uint8_t cst816_irq_handler(cst816_handle_t *handle)
{
    CST816_gesture_t gesture;
    cst816_pos_t pos;
    uint8_t fingernum;
    
    if (handle == NULL)                                                              /* check handle */
    {
        return 2;                                                                    /* return error */
    }
    if (handle->inited != 1)                                                         /* check handle initialization */
    {
        return 3;                                                                    /* return error */
    }
    // 处理中断
    if ((handle->irq_mode) == 0)                                                
    {
        cst816_interface_debug_print("cst816: no interrupt mode.\n");
    }
    else if ((handle->irq_mode) == CST816_IRQ_MODE_GESTURE)                                              
    {
        cst816_get_gesture(handle,&gesture);
        cst816_interface_debug_print("cst816: gesture is 0x%02X\n",gesture);
        handle->receive_callback_gesture((uint8_t)gesture);
    }
    else if ((handle->irq_mode) == CST816_IRQ_MODE_TOUCH)                                              
    {
        cst816_get_pos(handle, &pos);
        cst816_interface_debug_print("cst816: position is x:%d y:%d\n", pos.u8_data.x_l, pos.u8_data.y_l);
        handle->receive_callback_xy(pos.u8_data.x_l, pos.u8_data.y_l);
    }
    else if ((handle->irq_mode) == CST816_IRQ_MODE_CHANGE)                                               
    {
        cst816_get_fingernum(handle, &fingernum);
        cst816_interface_debug_print("cst816: fingernum is %d.\n", fingernum);
        handle->receive_callback_change(fingernum);
    }

    return 0;
}

