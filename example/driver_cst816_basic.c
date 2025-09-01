#include "driver_cst816_basic.h"

static cst816_handle_t gs_handle; /**< cst816 handle */

/**
 * @brief  basic example init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t cst816_basic_init(cst816_irq_mode_t irq_mode_input)
{
    uint8_t res;
    uint8_t id;

    /* link interface function */
    DRIVER_CST816_LINK_INIT(&gs_handle, cst816_handle_t);
    DRIVER_CST816_LINK_IIC_INIT(&gs_handle, cst816_interface_iic_init);
    DRIVER_CST816_LINK_IIC_DEINIT(&gs_handle, cst816_interface_iic_deinit);
    DRIVER_CST816_LINK_IIC_READ(&gs_handle, cst816_interface_iic_read);
    DRIVER_CST816_LINK_IIC_WRITE(&gs_handle, cst816_interface_iic_write);
    DRIVER_CST816_LINK_RESST_GPIO_INIT(&gs_handle, cst816_interface_reset_gpio_init);
    DRIVER_CST816_LINK_RESST_GPIO_DEINIT(&gs_handle, cst816_interface_reset_gpio_deinit);
    DRIVER_CST816_LINK_RESST_GPIO_WRITE(&gs_handle, cst816_interface_reset_gpio_write);
    DRIVER_CST816_LINK_RECEIVE_CALLBACK_GESTURE(&gs_handle, cst816_interface_receive_callback_gesture);
    DRIVER_CST816_LINK_RECEIVE_CALLBACK_XY(&gs_handle, cst816_interface_receive_callback_xy);
    DRIVER_CST816_LINK_RECEIVE_CALLBACK_CHANGE(&gs_handle, cst816_interface_receive_callback_change);
    DRIVER_CST816_LINK_DELAY_MS(&gs_handle, cst816_interface_delay_ms);
    DRIVER_CST816_LINK_DEBUG_PRINT(&gs_handle, cst816_interface_debug_print);

    /* cst816 init */
    res = cst816_init(&gs_handle);
    if (res != 0)
    {
        cst816_interface_debug_print("cst816: init failed.\n");

        return 1;
    }

    if (cst816_get_chipID(&gs_handle, (uint8_t *)&id) != 0)
    {
        cst816_interface_debug_print("cst816: get chip id failed.\n");
        (void)cst816_deinit(&gs_handle);

        return 1;
    }
    if (id != CHIPID_CST716 && id != CHIPID_CST816S && id != CHIPID_CST816T && id != CHIPID_CST816D)
    {
        cst816_interface_debug_print("cst816: chip id [0x%02X] is invalid.\n", id);
        (void)cst816_deinit(&gs_handle);

        return 1;
    }
    else
    {
        cst816_interface_debug_print("cst816: chip id is 0x%02x.\n", id);
    }

    gs_handle.irq_mode = irq_mode_input;
    switch (gs_handle.irq_mode)
    {
    case CST816_IRQ_MODE_GESTURE:
        /* 手势设置 */
        cst816_set_motion_mask(&gs_handle, CST816_MOTION_MASK_ENDCLICK | CST816_MOTION_MASK_ENCONLR | CST816_MOTION_MASK_ENCONUD); // 特殊手势使能
        cst816_set_long_press_tick(&gs_handle, 100);                     // 长按时间 1s
        /* 中断设置 */
        cst816_set_irq_ctl(&gs_handle, CST816_IRQ_CTL_ENMOTION | CST816_IRQ_CTL_ONCE_WLP); // 中断控制
                                                                                           /* 复位设置 */
        cst816_set_reset_mode(&gs_handle, CST816_RESET_MODE_ENLTRST);
        cst816_set_auto_resset_time(&gs_handle, 5); // 5s内无有效手势自动复位
        cst816_set_long_press_time(&gs_handle, 10); // 长按 10s 后自动复位
        break;
    case CST816_IRQ_MODE_TOUCH:
        /* 中断设置 */
        cst816_set_irq_ctl(&gs_handle, CST816_IRQ_CTL_ENTOUCH); // 中断控制

        break;
    case CST816_IRQ_MODE_CHANGE:
        /* 中断设置 */
        cst816_set_irq_ctl(&gs_handle, CST816_IRQ_CTL_ENCHANGE); // 中断控制

        break;
    default:
        cst816_interface_debug_print("cst816: irq mode is invalid.\n");
        return 1;
    }

    /* 中断脉冲设置 */
    cst816_set_irq_pluse_width(&gs_handle, 2); // 中断低脉冲宽度 2ms
    /* IO 设置 */
    cst816_set_IO_ctl(&gs_handle, CST816_IO_CTL_MODE_PULLUP, CST816_IO_CTL_VOLTAGE_VDD);

    return 0;
}

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t cst816_basic_deinit(void)
{
    /* deinit cst816 and close bus */
    if (cst816_deinit(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t cst816_basic_get_gesture(uint8_t *gesture)
{
    return cst816_get_gesture(&gs_handle, gesture);
}

uint8_t cst816_basic_get_pos(uint8_t *x, uint8_t *y)
{
    cst816_pos_t pos;
    cst816_get_pos(&gs_handle, &pos);
    *x = pos.u8_data.x_l;
    *y = pos.u8_data.y_l;
    return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        cst816_irq_handler(&gs_handle);
    }
}
