#ifndef DRIVER_CST816_INTERFACE_H
#define DRIVER_CST816_INTERFACE_H

#include "driver_cst816.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "stdio.h"
#include <cstdarg>

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup cst816_interface_driver cst816 interface driver function
 * @brief    cst816 interface driver modules
 * @ingroup  cst816_driver
 * @{
 */

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t cst816_interface_iic_init(void);

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t cst816_interface_iic_deinit(void);

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t cst816_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t cst816_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 gpio init failed
 * @note   none
 */
uint8_t cst816_interface_reset_gpio_init(void);

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 gpio deinit failed
 * @note   none
 */
uint8_t cst816_interface_reset_gpio_deinit(void);

/**
 * @brief     interface reset gpio write
 * @param[in] value written value
 * @return    status code
 *            - 0 success
 *            - 1 gpio write failed
 * @note      none
 */
uint8_t cst816_interface_reset_gpio_write(uint8_t value);

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void cst816_interface_delay_ms(uint32_t ms);

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void cst816_interface_debug_print(const char *const fmt, ...);

/**
 * @brief     interface receive callback
 * @param[in] type irq type
 * @note      none
 */
void cst816_interface_receive_callback_change(uint8_t type);
void cst816_interface_receive_callback_gesture(uint8_t type);
void cst816_interface_receive_callback_xy(uint8_t x,uint8_t y);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
