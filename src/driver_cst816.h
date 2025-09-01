#ifndef _DRIVER_CST816T_H_
#define _DRIVER_CST816T_H_

#include "driver_cst816_interface.h"
#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// /*
//  *芯片 ID
//  */
// cst716 与cst816s ,cst816T, cst816D 封装为 pin 对 pin, 可以直接替换。
// Cst816S 和 cst816T 有触摸唤醒功能，cst716与cst816D没有触摸唤醒
#define CHIPID_CST716   0x20
#define CHIPID_CST816S  0xB4
#define CHIPID_CST816T  0xB5
#define CHIPID_CST816D  0xB6

// 手势
typedef enum
{
    CST816_GESTURE_NONE = 0x00,
    CST816_GESTURE_UP = 0x01,
    CST816_GESTURE_DOWN = 0x02,
    CST816_GESTURE_LEFT = 0x03,
    CST816_GESTURE_RIGHT = 0x04,
    CST816_GESTURE_CLICK = 0x05,
    CST816_GESTURE_DOUBLE_CLICK = 0x0B,
    CST816_GESTURE_LONG_PRESS = 0x0C,
} CST816_gesture_t;

typedef enum
{
    CST816_SLEEPMODE_DISENABLE = 0x00,
    CST816_SLEEPMODE_ENABLE = 0x03,
} CST816_sleep_mode_t;

typedef enum
{
    CST816_RESET_MODE_ALL_DISENABLE = 0x00, // 禁止所有复位
    CST816_RESET_MODE_EN2FRST = 0x01,       // 使能双指复位
    CST816_RESET_MODE_ENFATRST = 0x02,      // 使能大面积触摸复位
    CST816_RESET_MODE_ENLTRST = 0x04,       // 使能长按复位
} CST816_reset_mode_t;

typedef enum
{
    CST816_MOTION_MASK_ALL_DISENABLE = 0x00, // 禁止所有动作
    CST816_MOTION_MASK_ENCONLR = 0x04,       // 使能连续左右滑动
    CST816_MOTION_MASK_ENCONUD = 0x02,       // 使能连续上下滑动
    CST816_MOTION_MASK_ENDCLICK = 0x01,      // 使能双击
} CST816_motion_mask_t;

typedef enum
{
    CST816_IRQ_CTL_ENTEST = 0x80,       // 中断引脚测试
    CST816_IRQ_CTL_ENTOUCH = 0x40,      // 触摸中断使能
    CST816_IRQ_CTL_ENCHANGE = 0x20,     // 触摸状态改变中断使能
    CST816_IRQ_CTL_ENMOTION = 0x10,     // 手势中断使能
    CST816_IRQ_CTL_ONCE_WLP = 0x01,     // 长按单次唤醒
} cst816_irq_ctl_t;

typedef enum
{
    CST816_IO_CTL_MODE_PULLUP = 0x00,   // 上拉
    CST816_IO_CTL_MODE_OD = 0x02,       // 开漏
} CST816_IO_ctl_mode_t;
typedef enum
{
    CST816_IO_CTL_VOLTAGE_VDD = 0x00, // VDD
    CST816_IO_CTL_VOLTAGE_1V8 = 0x01, // 1.8V
} CST816_IO_ctl_voltage_t;

typedef enum
{
    CST816_IO_CTL_SOFT_RESET_DISENABLE = 0x00,  // 禁止软件复位
    CST816_IO_CTL_SOFT_RESET_ENABLE = 0x04,     // 使能软件复位
} CST816_IO_ctl_soft_reset_t;

typedef enum{
    CST816_IRQ_MODE_TOUCH=0x01,  // 触摸模式 处理XY坐标
    CST816_IRQ_MODE_GESTURE=0x02,// 手势模式 处理手势
    CST816_IRQ_MODE_CHANGE=0x03, // 改变模式 处理点击
}cst816_irq_mode_t;

typedef union cst816_pos_s
{
    struct 
    {
        uint8_t x_h;
        uint8_t x_l;
        uint8_t y_h;
        uint8_t y_l;
    } u8_data;
    struct 
    {
        uint16_t x;
        uint16_t y;
    } u16_data;

} cst816_pos_t;

/**
 * @brief cst816 handle structure definition
 */
typedef struct cst816_handle_s
{
    uint8_t (*iic_init)(void);                                                   /**< point to an iic_init function address */
    uint8_t (*iic_deinit)(void);                                                 /**< point to an iic_deinit function address */
    uint8_t (*iic_read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);  /**< point to an iic_read function address */
    uint8_t (*iic_write)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len); /**< point to an iic_write function address */
    uint8_t (*reset_gpio_init)(void);                                            /**< point to a reset_gpio_init function address */
    uint8_t (*reset_gpio_deinit)(void);                                          /**< point to a reset_gpio_deinit function address */
    uint8_t (*reset_gpio_write)(uint8_t value);                                  /**< point to a reset_gpio_write function address */
    void (*delay_ms)(uint32_t ms);                                               /**< point to a delay_ms function address */
    void (*debug_print)(const char *const fmt, ...);                             /**< point to a debug_print function address */
    void (*receive_callback_gesture)(uint8_t type);                              /**< point to a receive_callback function address */
    void (*receive_callback_xy)(uint8_t x, uint8_t y);
    void (*receive_callback_change)(uint8_t type);
    uint8_t inited;                                                              /**< inited flag */
    cst816_irq_mode_t irq_mode;
} cst816_handle_t;

/**
 * @brief cst816 information structure definition
 */
typedef struct cst816_info_s
{
    char chip_name[32];         /**< chip name */
    char manufacturer_name[32]; /**< manufacturer name */
    char interface[8];          /**< chip interface name */
    float supply_voltage_min_v; /**< chip min supply voltage */
    float supply_voltage_max_v; /**< chip max supply voltage */
    float max_current_ma;       /**< chip max current */
    float temperature_min;      /**< chip min operating temperature */
    float temperature_max;      /**< chip max operating temperature */
    uint32_t driver_version;    /**< driver version */
} cst816_info_t;

/**
 * @defgroup cst816_link_driver cst816 link driver function
 * @brief    cst816 link driver modules
 * @ingroup  cst816_driver
 * @{
 */
#define DRIVER_CST816_LINK_INIT(HANDLE, STRUCTURE) memset(HANDLE, 0, sizeof(STRUCTURE))
#define DRIVER_CST816_LINK_IIC_INIT(HANDLE, FUC) (HANDLE)->iic_init = FUC
#define DRIVER_CST816_LINK_IIC_DEINIT(HANDLE, FUC) (HANDLE)->iic_deinit = FUC
#define DRIVER_CST816_LINK_IIC_READ(HANDLE, FUC) (HANDLE)->iic_read = FUC
#define DRIVER_CST816_LINK_IIC_WRITE(HANDLE, FUC) (HANDLE)->iic_write = FUC
#define DRIVER_CST816_LINK_RESST_GPIO_INIT(HANDLE, FUC) (HANDLE)->reset_gpio_init = FUC
#define DRIVER_CST816_LINK_RESST_GPIO_DEINIT(HANDLE, FUC) (HANDLE)->reset_gpio_deinit = FUC
#define DRIVER_CST816_LINK_RESST_GPIO_WRITE(HANDLE, FUC) (HANDLE)->reset_gpio_write = FUC
#define DRIVER_CST816_LINK_RECEIVE_CALLBACK_GESTURE(HANDLE, FUC) (HANDLE)->receive_callback_gesture = FUC
#define DRIVER_CST816_LINK_RECEIVE_CALLBACK_XY(HANDLE, FUC) (HANDLE)->receive_callback_xy = FUC
#define DRIVER_CST816_LINK_RECEIVE_CALLBACK_CHANGE(HANDLE, FUC) (HANDLE)->receive_callback_change = FUC
#define DRIVER_CST816_LINK_DELAY_MS(HANDLE, FUC) (HANDLE)->delay_ms = FUC
#define DRIVER_CST816_LINK_DEBUG_PRINT(HANDLE, FUC) (HANDLE)->debug_print = FUC

uint8_t cst816_init(cst816_handle_t *handle);
uint8_t cst816_deinit(cst816_handle_t *handle);

uint8_t cst816_get_gesture(cst816_handle_t *handle, CST816_gesture_t *gesture);
uint8_t cst816_get_fingernum(cst816_handle_t *handle, uint8_t *num);
uint8_t cst816_get_pos(cst816_handle_t *handle, cst816_pos_t *pos);
uint8_t cst816_get_chipID(cst816_handle_t *handle, uint8_t *chipid);

uint8_t cst816_set_sleep_mode(cst816_handle_t *handle, CST816_sleep_mode_t mode);
uint8_t cst816_get_sleep_mode(cst816_handle_t *handle, CST816_sleep_mode_t *mode);

uint8_t cst816_set_reset_mode(cst816_handle_t *handle,CST816_reset_mode_t mode);
uint8_t cst816_set_long_press_tick(cst816_handle_t *handle, uint8_t time);
uint8_t cst816_set_motion_mask(cst816_handle_t *handle, uint8_t mask);
uint8_t cst816_set_irq_pluse_width(cst816_handle_t *handle, uint8_t width);
uint8_t cst816_set_irq_ctl(cst816_handle_t *handle, cst816_irq_ctl_t ctl);
uint8_t cst816_set_auto_resset_time(cst816_handle_t *handle, uint8_t time);
uint8_t cst816_set_long_press_time(cst816_handle_t *handle, uint8_t time);
uint8_t cst816_set_IO_ctl(cst816_handle_t *handle, CST816_IO_ctl_mode_t mode, CST816_IO_ctl_voltage_t voltage);
uint8_t cst816_soft_reset(cst816_handle_t *handle);

uint8_t cst816_irq_handler(cst816_handle_t *handle);

#endif
