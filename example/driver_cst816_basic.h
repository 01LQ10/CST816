#ifndef DRIVER_CST816_BASIC_H
#define DRIVER_CST816_BASIC_H

#include "driver_cst816_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup cst816_example_driver cst816 example driver function
 * @brief    cst816 example driver modules
 * @ingroup  cst816_driver
 * @{
 */

/**
 * @brief  basic example init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t cst816_basic_init(cst816_irq_mode_t irq_mode);

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t cst816_basic_deinit(void);



uint8_t cst816_basic_TEST(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
