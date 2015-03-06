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

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_led.h"
#include "drv_spi.h"
#include "copter.h"
//#include "drv_motors.h"
#include "drv_i2c.h"
#include "drv_mpu6050.h"

#ifdef RT_USING_FINSH
#include "shell.h"
#endif

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "tcpserver.h"
#include "telnet.h"
#endif

extern "C" void set_if(char* netif_name, char* ip_addr, char* gw_addr, char* nm_addr);
extern "C" int lwip_system_init(void);

void rt_init_thread_entry(void* parameter)
{
	rt_i2c_core_init();
	rt_hw_i2c_init();
	rt_hw_spi_init();
	rt_hw_mpu6050_init("i2c1", MPU6050_DEFAULT_ADDRESS);
	//rt_motors_hw_init();

	apps_copter_init();
		
 #ifdef RT_USING_FINSH
 	/* init finsh */
 	finsh_system_init();
 	finsh_set_device(RT_CONSOLE_DEVICE_NAME);
 #endif
  
}

int rt_application_init()
{
	rt_thread_t tid;

	tid = rt_thread_create("init",
								rt_init_thread_entry, RT_NULL,
								2048, RT_THREAD_PRIORITY_MAX/3, 20);

	if (tid != RT_NULL)
		rt_thread_startup(tid);

	return 0;
}

/*@}*/
