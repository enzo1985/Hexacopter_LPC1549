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
	       /* Enable the clock for Switch Matrix */
        LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 12);
        LPC_IOCON->PIO0_0 = (0x01 << 7);
        LPC_IOCON->PIO0_16 = (0x01 << 7);
        LPC_IOCON->PIO0_10 = (0x01 << 7);
        LPC_IOCON->PIO0_9 = (0x01 << 7);

        LPC_SWM->PINASSIGN7 &= ~(0xff << 8);
        LPC_SWM->PINASSIGN7 |= (3 << 8);
	
        LPC_SWM->PINASSIGN7 &= ~(0xff << 16);
        LPC_SWM->PINASSIGN7 |= (25 << 16);

        LPC_SWM->PINASSIGN8 &= ~(0xffUL << 0);
        LPC_SWM->PINASSIGN8 |= (33 << 0);

        /* Disable the clock to the Switch Matrix to save power */
        LPC_SYSCON->SYSAHBCLKCTRL0 &=  ~(1UL << 12);
	 /* Peripheral reset control to SCT0 SCT1 SCT2 */
	 LPC_SYSCON->PRESETCTRL1 |= (0x01<<2)|(0x01<<3)|(0x01<<4);
	 LPC_SYSCON->PRESETCTRL1 &= ~((0x01<<2)|(0x01<<3)|(0x01<<4));
	 
	LPC_SCT0->CONFIG |= (1 << 0) | (1 << 17);                 // unified timer, auto limit
	LPC_SCT1->CONFIG |= (1 << 0) | (1 << 17);                 // unified timer, auto limit
	LPC_SCT2->CONFIG |= (1 << 0) | (1 << 17);                 // unified timer, auto limit
	
	LPC_SCT0->MATCH0            = SCT_PWM_RATE;               // match 0 on PWM cycle
  LPC_SCT0->MATCHREL0         = SCT_PWM_RATE;
  LPC_SCT0->MATCH1            = 0;                          // match 1 on val1 (PWM1)
  LPC_SCT0->MATCHREL1         = 0;
  LPC_SCT0->MATCH2            = 0;                          // match 2 on val2 (PWM2)
  LPC_SCT0->MATCHREL2         = 0;
	
	LPC_SCT1->MATCH0            = SCT_PWM_RATE;               // match 0 on PWM cycle
  LPC_SCT1->MATCHREL0         = SCT_PWM_RATE;
  LPC_SCT1->MATCH1            = 0;                          // match 1 on val1 (PWM3)
  LPC_SCT1->MATCHREL1         = 0;
  LPC_SCT1->MATCH2            = 0;                          // match 2 on val2 (PWM4)
  LPC_SCT1->MATCHREL2         = 0;
	
	LPC_SCT2->MATCH0            = SCT_PWM_RATE;               // match 0 on PWM cycle
  LPC_SCT2->MATCHREL0         = SCT_PWM_RATE;
  LPC_SCT2->MATCH1            = 0;                          // match 1 on val1 (PWM5)
  LPC_SCT2->MATCHREL1         = 0;
  LPC_SCT2->MATCH2            = 0;                          // match 2 on val2 (PWM6)
  LPC_SCT2->MATCHREL2         = 0;

  LPC_SCT0->EV0_STATE    = 0xFFFFFFFF;                      // event 0 happens in all states
  LPC_SCT0->EV0_CTRL     = (0 << 0) | (1 << 12);            // match 0 (pwm_cycle) only condition
  LPC_SCT0->EV1_STATE    = 0xFFFFFFFF;                      // event 1 happens in all states
  LPC_SCT0->EV1_CTRL     = (1 << 0) | (1 << 12);            // match 1 (pwm1) only condition
  LPC_SCT0->EV2_STATE    = 0xFFFFFFFF;                      // event 2 happens in all states
  LPC_SCT0->EV2_CTRL     = (2 << 0) | (1 << 12);            // match 2 (pwm2) only condition
	
	LPC_SCT1->EV0_STATE    = 0xFFFFFFFF;                      // event 0 happens in all states
  LPC_SCT1->EV0_CTRL     = (0 << 0) | (1 << 12);            // match 0 (pwm_cycle) only condition
  LPC_SCT1->EV1_STATE    = 0xFFFFFFFF;                      // event 1 happens in all states
  LPC_SCT1->EV1_CTRL     = (1 << 0) | (1 << 12);            // match 1 (pwm3) only condition
  LPC_SCT1->EV2_STATE    = 0xFFFFFFFF;                      // event 2 happens in all states
  LPC_SCT1->EV2_CTRL     = (2 << 0) | (1 << 12);            // match 2 (pwm4) only condition
	
  LPC_SCT2->EV0_STATE    = 0xFFFFFFFF;                      // event 0 happens in all states
  LPC_SCT2->EV0_CTRL     = (0 << 0) | (1 << 12);            // match 0 (pwm_cycle) only condition
  LPC_SCT2->EV1_STATE    = 0xFFFFFFFF;                      // event 1 happens in all states
  LPC_SCT2->EV1_CTRL     = (1 << 0) | (1 << 12);            // match 1 (pwm5) only condition
  LPC_SCT2->EV2_STATE    = 0xFFFFFFFF;                      // event 2 happens in all states
  LPC_SCT2->EV2_CTRL     = (2 << 0) | (1 << 12);            // match 2 (pwm6) only condition

  LPC_SCT0->OUT0_SET        = (1 << 0);                     // event 0       sets  OUT0 (PWM1)
  LPC_SCT0->OUT0_CLR        = (1 << 1);                     // event 1       clear OUT0 (PWM1)
  LPC_SCT0->OUT1_SET        = (1 << 0);                     // event 0       sets  OUT1 (PWM2)
  LPC_SCT0->OUT1_CLR        = (1 << 2);                     // event 2       clear OUT1 (PWM2)
	
	LPC_SCT1->OUT0_SET        = (1 << 0);                     // event 0       sets  OUT0 (PWM3)
  LPC_SCT1->OUT0_CLR        = (1 << 1);                     // event 1       clear OUT0 (PWM3)
  LPC_SCT1->OUT1_SET        = (1 << 0);                     // event 0       sets  OUT1 (PWM4)
  LPC_SCT1->OUT1_CLR        = (1 << 2);                     // event 2       clear OUT1 (PWM4)
	
  LPC_SCT2->OUT0_SET        = (1 << 0);                     // event 0       sets  OUT0 (PWM5)
  LPC_SCT2->OUT0_CLR        = (1 << 1);                     // event 1       clear OUT0 (PWM5)
  LPC_SCT2->OUT1_SET        = (1 << 0);                     // event 0       sets  OUT1 (PWM6)
  LPC_SCT2->OUT1_CLR        = (1 << 2);                     // event 2       clear OUT1 (PWM6)

  LPC_SCT0->OUTPUT            = 0x00000000;                 // default clear OUT0/1
	LPC_SCT0->RES               = 0x0000000A;                 // conflict resolution: Inactive state takes precedence
                                                            // SCT0_OUT0/1: Inactive state low
  LPC_SCT1->OUTPUT            = 0x00000000;                 // default clear OUT0/1
	LPC_SCT1->RES               = 0x0000000A;                 // conflict resolution: Inactive state takes precedence
                                                            // SCT1_OUT0/1: Inactive state low
	LPC_SCT2->OUTPUT            = 0x00000000;                 // default clear OUT0/1
	LPC_SCT2->RES               = 0x0000000A;                 // conflict resolution: Inactive state takes precedence
                                                            // SCT2_OUT0/1: Inactive state low
	LPC_SCT0->CTRL           &= ~(1 << 2);                    // start timer
	LPC_SCT1->CTRL           &= ~(1 << 2);                    // start timer
	LPC_SCT2->CTRL           &= ~(1 << 2);                    // start timer
	
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
		
  LPC_SCT0->MATCH1            = pwm_value[0] - MOTORS_PWM_MIN;                          // match 1 on val1 (PWM1)
  LPC_SCT0->MATCHREL1         = pwm_value[0] - MOTORS_PWM_MIN;
  LPC_SCT0->MATCH2            = pwm_value[1] - MOTORS_PWM_MIN;                          // match 2 on val2 (PWM2)
  LPC_SCT0->MATCHREL2         = pwm_value[1] - MOTORS_PWM_MIN;
	LPC_SCT1->MATCH1            = pwm_value[2] - MOTORS_PWM_MIN;                          // match 1 on val1 (PWM3)
  LPC_SCT1->MATCHREL1         = pwm_value[2] - MOTORS_PWM_MIN;
  LPC_SCT1->MATCH2            = pwm_value[3] - MOTORS_PWM_MIN;                          // match 2 on val2 (PWM4)
  LPC_SCT1->MATCHREL2         = pwm_value[3] - MOTORS_PWM_MIN;
	LPC_SCT2->MATCH1            = pwm_value[4] - MOTORS_PWM_MIN;                          // match 1 on val1 (PWM5)
  LPC_SCT2->MATCHREL1         = pwm_value[4] - MOTORS_PWM_MIN;
  LPC_SCT2->MATCH2            = pwm_value[5] - MOTORS_PWM_MIN;                          // match 2 on val2 (PWM6)
  LPC_SCT2->MATCHREL2         = pwm_value[5] - MOTORS_PWM_MIN;

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
