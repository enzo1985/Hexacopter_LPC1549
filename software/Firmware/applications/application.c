/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_led.h"
#include "drv_spi.h"
#include "drv_i2c.h"
#include "drv_mpu6050.h"
#ifdef RT_USING_FINSH
#include <finsh.h>
#include <shell.h>
#endif

void rt_init_thread_entry(void* parameter)
{
	rt_i2c_core_init();
	rt_hw_i2c_init();
	rt_hw_spi_init();
	rt_hw_mpu6050_init("i2c0", MPU6050_DEFAULT_ADDRESS);
#ifdef RT_USING_FINSH
	/* initialization finsh shell Component */
    finsh_system_init();
	  finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
}

void rt_led_thread_entry(void* parameter)
{
 uint16_t count = 0;
 led_hw_init();
 while(1)
   {
		 count ++;
		 if((count%3)==0)
			 led_hw_rever(LED_BLUE);
		 if((count%5)==0)
			 led_hw_rever(LED_GREEN);
		 if((count%7)==0)
			 led_hw_rever(LED_RED);
		 
		 rt_thread_delay(RT_TICK_PER_SECOND/10);
	 }
}

int rt_application_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
  
	  tid = rt_thread_create("led",
        rt_led_thread_entry, RT_NULL,
        512, RT_THREAD_PRIORITY_MAX-3, 5);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return 0;
}
