#ifndef __DRV_LED_H
#include <lpc15xx.h>
#include <rtthread.h>

#define LED_BLUE  0x01
#define LED_GREEN 0x02
#define LED_RED   0x04

#define LED_BLUE_PORT 1
#define LED_BLUE_PIN  1

#define LED_GREEN_PORT 0
#define LED_GREEN_PIN  3

#define LED_RED_PORT 0
#define LED_RED_PIN  25

rt_err_t led_hw_on(rt_uint8_t leds);
rt_err_t led_hw_off(rt_uint8_t leds);
rt_err_t led_hw_rever(rt_uint8_t leds);
rt_err_t led_hw_init(void);

#endif
