/*
 * File      : drv_uart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-18     xiaonong      The first version for LPC15xx
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_uart.h"

struct lpc_uart
{
    LPC_USART0_Type *UART;
    IRQn_Type UART_IRQn;
};

static rt_err_t lpc_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct lpc_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct lpc_uart *)serial->parent.user_data;


    /* 115200 baud @ 12MHz */
    LPC_SYSCON->UARTCLKDIV = 6;                   /* UART clock =  PCLK / 6     */
    LPC_SYSCON->FRGCTRL    = 0x15FF;
    uart->UART->BRG  = ((SystemCoreClock / LPC_SYSCON->UARTCLKDIV / 16 / cfg->baud_rate) - 1);

    uart->UART->CFG  = ((1UL << 0) |                    /* Enable USART               */
                        (1UL << 2) |                    /* 8 data bits                */
                        (0UL << 4) |                    /* no parity                  */
                        (0UL << 6));                    /* 1 stop bit                 */


    return RT_EOK;
}

static rt_err_t lpc_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct lpc_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct lpc_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        uart->UART->INTENCLR = UART_INTEN_RXRDY;
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        uart->UART->INTENSET = UART_INTEN_RXRDY;
        break;
    }

    return RT_EOK;
}

static int lpc_putc(struct rt_serial_device *serial, char c)
{
    struct lpc_uart *uart;

    uart = (struct lpc_uart *)serial->parent.user_data;
    while (!(uart->UART->STAT & (1UL << 2)));
    uart->UART->TXDATA = c;

    return 1;
}

static int lpc_getc(struct rt_serial_device *serial)
{
    struct lpc_uart *uart;

    uart = (struct lpc_uart *)serial->parent.user_data;
    if (uart->UART->STAT & 0x01)
        return (uart->UART->RXDATA);
    else
        return -1;
}

static const struct rt_uart_ops lpc_uart_ops =
{
    lpc_configure,
    lpc_control,
    lpc_putc,
    lpc_getc,
};

#ifdef RT_USING_UART0
/* UART0 device driver structure */
struct lpc_uart uart0 =
{
    LPC_USART0,
    UART0_IRQn,
};
struct rt_serial_device serial0;

void UART0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if ((LPC_USART0->INTSTAT & UART_STAT_RXRDY) != 0x00)
    {
        rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#ifdef RT_USING_UART1
/* UART1 device driver structure */
struct lpc_uart uart1 =
{
    LPC_USART1,
    UART1_IRQn,
};
struct rt_serial_device serial1;

void UART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if ((LPC_USART1->INTSTAT & UART_STAT_RXRDY) != 0x00)
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif


void rt_hw_uart_init(void)
{
    struct lpc_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_UART0
    uart = &uart0;

    serial0.ops    = &lpc_uart_ops;
    serial0.config = config;
    serial0.parent.user_data = uart;
    /*
     * Initialize USART0 pin connect
     * P0.18: U0_TXD
     * P0.13: U0_RXD
     */
    /* Enable the clock for GPIO0     */
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 14);

    /* Enable the clock for Switch Matrix */
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 12);

    LPC_SWM->PINASSIGN0 &= ~((0xFF <<  0) |       /* clear PIN assign UART0_TXD */
                             (0xFF <<  8));       /* clear PIN assign UART0_RXD */
    LPC_SWM->PINASSIGN0 |= ((18 <<  0) |          /* PIN assign UART0_TXD  P0.18 */
                            (13 <<  8));         /* PIN assign UART0_RXD  P0.13 */

    /* Disable the clock for Switch Matrix to save power */
    LPC_SYSCON->SYSAHBCLKCTRL0 &=  ~(1UL << 12);
    /* configure UART0 */
    LPC_SYSCON->SYSAHBCLKCTRL1 |= (1UL << 17);    /* Enable clock to UART0      */

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(uart->UART_IRQn, ((0x01 << 3) | 0x01));

    /* Enable Interrupt for UART channel */
    NVIC_EnableIRQ(uart->UART_IRQn);

    /* register UART0 device */
    rt_hw_serial_register(&serial0, "uart0",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          uart);
#endif

#ifdef RT_USING_UART1
    uart = &uart1;

    serial1.ops    = &lpc_uart_ops;
    serial1.config = config;
    serial1.parent.user_data = uart;
    /*
     * Initialize UART1 pin connect
     * P0.11: U1_TXD
     * P0.31: U1_RXD
     */
    /* Enable the clock for GPIO0     */
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 14);

    /* Enable the clock for Switch Matrix */
    LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 12);

    LPC_SWM->PINASSIGN1 &= ~((0xFF <<  8) |        /* clear PIN assign UART0_TXD */
                             (0xFF <<  16));       /* clear PIN assign UART0_RXD */
    LPC_SWM->PINASSIGN1 |= ((31 <<  0) |           /* PIN assign UART1_TXD  P0.11 */
                            (11 <<  8));           /* PIN assign UART1_RXD  P0.31 */

    /* Disable the clock for Switch Matrix to save power */
    LPC_SYSCON->SYSAHBCLKCTRL0 &=  ~(1UL << 12);
    /* configure UART1 */
    LPC_SYSCON->SYSAHBCLKCTRL1 |= (1UL << 18);    /* Enable clock to UART1      */

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(uart->UART_IRQn, ((0x01 << 3) | 0x01));

    /* Enable Interrupt for UART channel */
    NVIC_EnableIRQ(uart->UART_IRQn);

    /* register UART1 device */
    rt_hw_serial_register(&serial1, "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          uart);
#endif

}
