#include <rtthread.h>
#include "board.h"
#include "drv_motors.h"

/**
*  PWM1 <--> PIO0_24  SCT0_OUT0
*  PWM2 <--> PIO0_2   SCT0_OUT1
*  PWM3 <--> PIO0_13  SCT1_OUT0
*  PWM4 <--> PIO0_7   SCT1_OUT1
*  PWM5 <--> PIO0_25  SCT2_OUT0
*  PWM6 <--> PIO0_12  SCT2_OUT1
**/
#define SCT_PWM_RATE   20000        /* PWM frequency 20 KHz */


static rt_uint16_t pwm_value[MOTORS_NUM_MAX];


static rt_err_t rt_motors_init(rt_device_t dev)
{
   /* Enable SCT0 SCT1 SCT2 clock */
   LPC_SYSCON->SYSAHBCLKCTRL1 |= (0x01<<2)|(0x01<<3)|(0x01<<4);
	 /* Peripheral reset control to SCT0 SCT1 SCT2 */
	 LPC_SYSCON->PRESETCTRL1 |= (0x01<<2)|(0x01<<3)|(0x01<<4);
	 LPC_SYSCON->PRESETCTRL1 &= ~((0x01<<2)|(0x01<<3)|(0x01<<4));
	
  return RT_EOK;
}

static rt_err_t rt_motors_open(rt_device_t dev, rt_uint16_t oflag)
{	
    return RT_EOK;
}

static rt_err_t rt_motors_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_motors_read(rt_device_t dev, rt_off_t pos, void *buffer,
                             rt_size_t size)
{
    rt_ubase_t index = 0;
    rt_uint16_t *value = buffer;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT((pos + size) <= MOTORS_NUM_MAX);

    for (index = 0; index < size; index++)
    {
        *value = pwm_value[pos+index];
			   value++;
    }
    return size;
}

static rt_size_t rt_motors_write(rt_device_t dev, rt_off_t pos,
                              const void *buffer, rt_size_t size)
{
    rt_ubase_t index = 0;
   const rt_uint16_t *value = buffer;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT((pos + size) <= MOTORS_NUM_MAX);

    for (index = 0; index < size; index++)
    {
			if((value[index] >= MOTORS_PWM_MIN)&&(value[index] <= MOTORS_PWM_MAX))
				{
				pwm_value[pos+index]= value[index];
				}
				else if(value[index] < MOTORS_PWM_MIN)
				{
			pwm_value[pos+index] = MOTORS_PWM_MIN;
				}
		else {
			pwm_value[pos+index] = MOTORS_PWM_MAX;
		  }
    }
//	TIM2->CCR1 = pwm_value[3] - MOTORS_PWM_MIN;
//	TIM2->CCR2 = pwm_value[2] - MOTORS_PWM_MIN;
//	TIM1->CCR1 = pwm_value[0] - MOTORS_PWM_MIN;
//	TIM1->CCR4 = pwm_value[1] - MOTORS_PWM_MIN;

    return size;
}

static rt_err_t rt_motors_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{

    return RT_EOK;
}

void rt_motors_hw_init(void)
{
	 rt_device_t device = RT_NULL;
	 device = rt_malloc(sizeof(struct rt_device));
	 if(device == RT_NULL)
    {
		 rt_kprintf("no mem to malloc motors device!\n");
			return ;
		}
    device->type         = RT_Device_Class_Char;
    device->rx_indicate  = RT_NULL;
    device->tx_complete  = RT_NULL;
    device->init         = rt_motors_init;
    device->open         = rt_motors_open;
    device->close        = rt_motors_close;
    device->read         = rt_motors_read;
    device->write        = rt_motors_write;
    device->control      = rt_motors_control;
    device->user_data    = RT_NULL;

    /* register a character device */
    rt_device_register(device, "motors", RT_DEVICE_FLAG_RDWR);
  
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void motors_test(rt_uint32_t motors_num, rt_uint32_t value)
{
    rt_device_t dev = RT_NULL;
	  dev = rt_device_find("motors");
	 if(dev == RT_NULL)
   {
	  rt_kprintf("can not found motors device!\n");
		 return ;
	 }
	 rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
	 
    rt_motors_write(dev, motors_num, &value, 1);
}
FINSH_FUNCTION_EXPORT(motors_test, e.g: motors_test(0, 100).)
#endif
