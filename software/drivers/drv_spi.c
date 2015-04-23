/*
 * File      : drv_spi.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-21     xiaonong      The first version for LPC15xx
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#ifdef RT_USING_SPI
#include "drv_spi.h"

/* private rt-thread spi ops function */
static rt_err_t configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration);
static rt_uint32_t xfer(struct rt_spi_device *device, struct rt_spi_message *message);

static struct rt_spi_ops lpc_spi_ops =
{
    configure,
    xfer
};

#ifdef SPI_USE_DMA
static uint8_t dummy = 0xFF;
static void DMA_RxConfiguration(struct lpc_spi_bus *lpc_spi_bus,
                                const void *send_addr,
                                void *recv_addr,
                                rt_size_t size)
{

}
#endif

static rt_err_t configure(struct rt_spi_device *device,
                          struct rt_spi_configuration *configuration)
{
    struct lpc_spi_bus *spi_bus = (struct lpc_spi_bus *)device->bus;

    /* Disable spi device */
    spi_bus->SPI->CFG &= ~(0x01 << 0);
    /* data_width */
    if (configuration->data_width > 16)
    {
        return RT_EIO;
    }

    /* baudrate */
    {
        uint16_t div_val = 0;
        div_val = SystemCoreClock / configuration->max_hz;

        spi_bus->SPI->DIV = div_val;
    }

    /* CPOL */
    if (configuration->mode & RT_SPI_CPOL)
    {
        spi_bus->SPI->CFG |= SPI_CFG_CPOL_LO;
    }
    else
    {
        spi_bus->SPI->CFG |= SPI_CFG_CPOL_HI;
    }
    /* CPHA */
    if (configuration->mode & RT_SPI_CPHA)
    {
        spi_bus->SPI->CFG |= SPI_CFG_CPHA_SECOND;
    }
    else
    {
        spi_bus->SPI->CFG |= SPI_CFG_CPHA_FIRST;
    }
    /* data order */
    if (configuration->mode & RT_SPI_LSB)
    {
        spi_bus->SPI->CFG |= SPI_CFG_LSB_FIRST_EN;
    }
    else
    {
        spi_bus->SPI->CFG |= SPI_CFG_MSB_FIRST_EN;
    }
    /* configure the delay */
    spi_bus->SPI->DLY = SPI_DLY_PRE_DELAY(2);
    spi_bus->SPI->DLY |= SPI_DLY_POST_DELAY(2);
    spi_bus->SPI->DLY |= SPI_DLY_FRAME_DELAY(2);
    spi_bus->SPI->DLY |= SPI_DLY_TRANSFER_DELAY(1);

    /* Enable SPI_MASTER */
    spi_bus->SPI->CFG |= (0x05);

    return RT_EOK;
}

static rt_uint32_t xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    struct lpc_spi_bus *spi_bus = (struct lpc_spi_bus *)device->bus;
    struct rt_spi_configuration *config = &device->config;
    struct lpc_spi_cs *spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;
    rt_uint32_t send_count = 0 , recv_count = 0 , status = 0;

    /* take CS */
    if (message->cs_take)
    {
        /* clean the status */
        spi_bus->SPI->STAT = SPI_STAT_CLR_RXOV | SPI_STAT_CLR_TXUR | SPI_STAT_CLR_SSA | SPI_STAT_CLR_SSD | SPI_STAT_FORCE_EOT;

        /* Set control information including SSEL, EOT, EOF RXIGNORE and FLEN */
        spi_bus->SPI->TXCTL = ((spi_cs->ncs | SPI_TXCTL_EOF) & SPI_TXCTL_BITMASK) | SPI_TXDATCTL_LEN(config->data_width - 1);
    }

    {
        const rt_uint8_t *send_ptr = message->send_buf;
        rt_uint8_t *recv_ptr = message->recv_buf;
        //  rt_kprintf("size =%d",size);
        while (send_count < size || recv_count < size)
        {
            rt_uint8_t data = 0x00;


            status = spi_bus->SPI->STAT;

            /* In case of TxReady */
            if ((status & SPI_STAT_TXRDY) && (send_count < size))
            {
                if (send_ptr != RT_NULL)
                {
                    data = *send_ptr++;
                }
                if ((send_count < (size - 1)) && (message->cs_release))
                {
                    spi_bus->SPI->TXDATCTL = (spi_cs->ncs & SPI_TXDATCTL_SSEL_MASK) | SPI_TXDATCTL_EOF | SPI_TXDATCTL_EOT |
                                             SPI_TXDATCTL_LEN(config->data_width - 1) | SPI_TXDATCTL_DATA(data);
                }
                else
                {
                    spi_bus->SPI->TXDAT = SPI_TXDAT_DATA(data);
                }
                send_count ++;
            }

            /*In case of Rx ready */
            if ((status & SPI_STAT_RXRDY) && (recv_count < size))
            {
                /* Get the received data */
                data = SPI_RXDAT_DATA(spi_bus->SPI->RXDAT);
                recv_count++;
                if (recv_ptr != RT_NULL)
                {
                    *recv_ptr++ = data;
                }
            }

        }
        /* Check error */
        if (spi_bus->SPI->STAT & (SPI_STAT_RXOV | SPI_STAT_TXUR))
        {
            return 0;
        }
    }


    return message->length;
};

/** \brief init and register lpc spi bus.
 *
 * \param SPI: lpc SPI, e.g: LPC_SSP0,LPC_SSP1,LPC_SSP2.
 * \param lpc_spi: lpc spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t lpc_spi_register(LPC_SPI0_Type *SPI,
                          struct lpc_spi_bus *lpc_spi,
                          const char *spi_bus_name)
{
    if (SPI == LPC_SPI0)
    {
        lpc_spi->SPI = LPC_SPI0;
        /* Enable the clock for SPI0 */
        LPC_SYSCON->SYSAHBCLKCTRL1 |= (1UL << 9);
    }
    else if (SPI == LPC_SPI1)
    {
        lpc_spi->SPI = LPC_SPI1;
        /* Enable the clock for SPI1 */
        LPC_SYSCON->SYSAHBCLKCTRL1 |= (1UL << 10);
    }
    else
    {
        return RT_ENOSYS;
    }

    return rt_spi_bus_register(&lpc_spi->parent, spi_bus_name, &lpc_spi_ops);
}
/* SPI0
SPI0_SCK  <--->PIO0_0
SPI0_MOSI <--->PIO0_16
SPI0_MISO <--->PIO0_10
SPI0_SSEL <--->PIO0_20
*/
int rt_hw_spi_init(void)
{
    /* register spi bus */
    {
        static struct lpc_spi_bus lpc_spi0;
        lpc_spi_register(LPC_SPI0, &lpc_spi0, "spi0");
        /* Enable the clock for Switch Matrix */
        LPC_SYSCON->SYSAHBCLKCTRL0 |= (1UL << 12);
        /*
         * Initialize SPI0 pins connect
         * SCK0: PINASSIGN3[15:8]: Select P0.0
         * MOSI0: PINASSIGN3[23:16]: Select P0.16
         * MISO0: PINASSIGN3[31:24] : Select P0.10
         * SSEL0: PINASSIGN4[7:0]: Select P0.9
         */
        LPC_IOCON->PIO0_0 = (0x01 << 7);
        LPC_IOCON->PIO0_16 = (0x01 << 7);
        LPC_IOCON->PIO0_10 = (0x01 << 7);
        LPC_IOCON->PIO0_9 = (0x01 << 7);

        LPC_SWM->PINASSIGN3 &= ~(0xff << 8);
        LPC_SWM->PINASSIGN3 |= (0 << 8);

        LPC_SWM->PINASSIGN3 &= ~(0xff << 16);
        LPC_SWM->PINASSIGN3 |= (16 << 16);

        LPC_SWM->PINASSIGN3 &= ~(0xffUL << 24);
        LPC_SWM->PINASSIGN3 |= (10 << 24);

        LPC_SWM->PINASSIGN4 &= ~(0xff << 0);
        LPC_SWM->PINASSIGN4 |= (9 << 0);


        /* Disable the clock to the Switch Matrix to save power */
        LPC_SYSCON->SYSAHBCLKCTRL0 &=  ~(1UL << 12);
    }
    /* attach cs */
    {
        static struct rt_spi_device spi_device;
        static struct lpc_spi_cs  spi_cs1;
        spi_cs1.ncs = 0;
        rt_spi_bus_attach_device(&spi_device, "spi00", "spi0", (void *)&spi_cs1);
    }



    return 0;
}
INIT_BOARD_EXPORT(rt_hw_spi_init);

#endif
