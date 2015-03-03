#include "drv_pwm.h"


void SCT0_Init(void)
{
    LPC_SYSCON->SYSAHBCLKCTRL1 |= EN1_SCT0;                // enable the SCT0 clock
    LPC_SCT0->CONFIG           |= (1 << 0) | (1 << 17);    // unified, auto limit

    LPC_SCT0->MATCHREL[0].U  = delay;                      // match_cycle
    LPC_SCT0->MATCHREL[1].U  = match_green_OFF;            // match_green_OFF
    LPC_SCT0->MATCHREL[2].U  = match_green_ON;             // match_green_ON
    LPC_SCT0->MATCHREL[3].U  = match_red_OFF;              // match_red_OFF
    LPC_SCT0->MATCHREL[4].U  = match_red_ON;               // match_red_ON

    LPC_SCT0->EVENT[0].STATE = (1 << 0);                   // event 0 happens in state 0
    LPC_SCT0->EV0_CTRL  = (0 << 0)  |                 // related to match_cycle
                               (0 << 10) |                 // IN_0 low
                               (3 << 12) |                 // match AND IO condition
                               (1 << 14) |                 // STATEV is loaded into state
                               (1 << 15);                  // new state is 1

    LPC_SCT0->EVENT[1].STATE = (1 << 0);                   // event 1 happens in state 0
    LPC_SCT0->EVENT[1].CTRL  = (3 << 0) | (1 << 12);       // match_red_OFF only condition

    LPC_SCT0->EVENT[2].STATE = (1 << 0);                   // event 2 happens in state 0
    LPC_SCT0->EVENT[2].CTRL  = (4 << 0) | (1 << 12);       // match_red_ON only condition

    LPC_SCT0->EVENT[3].STATE = (1 << 1);                   // event 3 happens in state 1
    LPC_SCT0->EVENT[3].CTRL  = (0 << 0)  |                 // related to match_cycle
                               (3 << 10) |                 // IN_0 high
                               (3 << 12) |                 // match AND IO condition
                               (1 << 14) |                 // STATEV is loaded into state
                               (0 << 15);                  // new state is 0

    LPC_SCT0->EVENT[4].STATE = (1 << 1);                   // event 4 happens in state 1
    LPC_SCT0->EVENT[4].CTRL  = (2 << 0) | (1 << 12);       // match_green_ON only condition

    LPC_SCT0->EVENT[5].STATE = (1 << 1);                   // event 5 happens in state 1
    LPC_SCT0->EVENT[5].CTRL  = (1 << 0) | (1 << 12);       // match_green_OFF only condition

    LPC_SCT0->OUT[0].SET = (1 << 0) | (1 << 3) | (1 << 5); // event 0, 3 and 5 set OUT0 (green LED)
    LPC_SCT0->OUT[0].CLR = (1 << 4);                       // event 4 clear OUT0 (green LED)
    LPC_SCT0->OUT[1].SET = (1 << 0) | (1 << 1) | (1 << 3); // event 0, 1 and 3 set OUT1 (red LED)
    LPC_SCT0->OUT[1].CLR = (1 << 2);                       // event 2 clear OUT1 (red LED)
    LPC_SCT0->OUTPUT    |= 3;                              // default set OUT0 and OUT1

    LPC_SCT0->CTRL_U           &= ~(1 << 2);               // start timer
}

rt_err_t pwm_hw_init(void)
{


    return RT_EOK;
}

INIT_DEVICE_EXPORT(pwm_hw_init);

