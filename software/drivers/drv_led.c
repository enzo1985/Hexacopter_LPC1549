#include "drv_led.h"

rt_err_t led_hw_on(rt_uint8_t leds)
{
    if (leds & LED_BLUE)
        LPC_GPIO_PORT->CLR[LED_BLUE_PORT]  |= (0x01 << LED_BLUE_PIN);

    if (leds & LED_GREEN)
        LPC_GPIO_PORT->CLR[LED_GREEN_PORT]  |= (0x01 << LED_GREEN_PIN);

    if (leds & LED_RED)
        LPC_GPIO_PORT->CLR[LED_RED_PORT]  |= (0x01 << LED_RED_PIN);

    return RT_EOK;


rt_err_t led_hw_off(rt_uint8_t leds)
{
    if (leds & LED_BLUE)
        LPC_GPIO_PORT->SET[LED_BLUE_PORT]  |= (0x01 << LED_BLUE_PIN);

    if (leds & LED_GREEN)
        LPC_GPIO_PORT->SET[LED_GREEN_PORT]  |= (0x01 << LED_GREEN_PIN);

    if (leds & LED_RED)
        LPC_GPIO_PORT->SET[LED_RED_PORT]  |= (0x01 << LED_RED_PIN);
     LPC
    return RT_EOK;
}

rt_err_t led_hw_rever(rt_uint8_t leds)
{
    if (leds & LED_BLUE)
        LPC_GPIO_PORT->PIN[LED_BLUE_PORT]  ^= (0x01 << LED_BLUE_PIN);

    if (leds & LED_GREEN)
        LPC_GPIO_PORT->PIN[LED_GREEN_PORT]  ^= (0x01 << LED_GREEN_PIN);

    if (leds & LED_RED)
        LPC_GPIO_PORT->PIN[LED_RED_PORT]  ^= (0x01 << LED_RED_PIN);

    return RT_EOK;
}

/**
* BLUE_LED  <---> PIO1_1
* GREEN_LED <---> PIO0_3
* RED_LED   <---> PIO0_25
**/
rt_err_t led_hw_init(void)
{
    /* enable clock for GPIO0 and GPIO1 port */
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 14) | (1UL << 15);
    /* configure blue led pin as output */
    LPC_GPIO_PORT->DIR[LED_BLUE_PORT]  |= (0x01 << LED_BLUE_PIN);
    LPC_GPIO_PORT->SET[LED_BLUE_PORT]  |= (0x01 << LED_BLUE_PIN);
    /* configure green led pin as output */
    LPC_GPIO_PORT->DIR[LED_GREEN_PORT]  |= (0x01 << LED_GREEN_PIN);
    LPC_GPIO_PORT->SET[LED_GREEN_PORT]  |= (0x01 << LED_GREEN_PIN);
    /* configure red led pin as output */
    LPC_GPIO_PORT->DIR[LED_RED_PORT]  |= (0x01 << LED_RED_PIN);
    LPC_GPIO_PORT->SET[LED_RED_PORT]  |= (0x01 << LED_RED_PIN);

    return RT_EOK;
}

INIT_DEVICE_EXPORT(led_hw_init);

