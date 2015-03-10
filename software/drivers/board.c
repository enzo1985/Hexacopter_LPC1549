/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 * 2015-01-20     xiaonong      ported to LPC15xx
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"


#ifdef RT_USING_COMPONENTS_INIT
#include <components.h>
#endif

// cycles per microsecond
static volatile uint32_t usTicks = 0;

uint32_t GetSysTime_us(void) 
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = rt_tick_get();
        cycle_cnt = SysTick->VAL;
    } while (ms != rt_tick_get());
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}
/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This function will initial LPC15xx board.
 */
void rt_hw_board_init()
{

    usTicks = SystemCoreClock / 1000000;
    /* init systick */
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    /* set pend exception priority */
    NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    /*init uart device*/
    rt_hw_uart_init();
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);

#ifdef RT_USING_COMPONENTS_INIT
    /* initialization board with RT-Thread Components */
    rt_components_board_init();
#endif
}

