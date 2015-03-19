/*
 * File      : drv_spi.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-20    xiaonong       The first version for LPC15xx
 */

#ifndef __DRV_SPI_H
#define __DRV_SPI_H


#include <stdint.h>
#include <rtthread.h>
#include <drivers/spi.h>

#include <lpc15xx.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
//#define SPI_USE_DMA

struct lpc_spi_bus
{
    struct rt_spi_bus parent;
    LPC_SPI0_Type *SPI;
};

struct lpc_spi_cs
{
    uint8_t ncs;
};

/**
 * Macro defines for SPI Configuration register
 */
/* SPI CFG Register BitMask */
#define SPI_CFG_BITMASK     ((uint32_t) 0xFBD)
/** SPI enable  */
#define SPI_CFG_SPI_EN      ((uint32_t) (1 << 0))
/** SPI Slave Mode Select */
#define SPI_CFG_SLAVE_EN   ((uint32_t) (0 << 2))
/** SPI Master Mode Select */
#define SPI_CFG_MASTER_EN   ((uint32_t) (1 << 2))
/** SPI MSB First mode enable */
#define SPI_CFG_MSB_FIRST_EN   ((uint32_t) (0 << 3))    /*Data will be transmitted and received in standard order (MSB first).*/
/** SPI LSB First mode enable */
#define SPI_CFG_LSB_FIRST_EN   ((uint32_t) (1 << 3))/*Data will be transmitted and received in reverse order (LSB first).*/
/** SPI Clock Phase Select*/
#define SPI_CFG_CPHA_FIRST   ((uint32_t) (0 << 4))  /*Capture data on the first edge, Change data on the following edge*/
#define SPI_CFG_CPHA_SECOND  ((uint32_t) (1 << 4))  /*Change data on the first edge, Capture data on the following edge*/
/** SPI Clock Polarity Select*/
#define SPI_CFG_CPOL_LO     ((uint32_t) (0 << 5))   /* The rest state of the clock (between frames) is low.*/
#define SPI_CFG_CPOL_HI     ((uint32_t) (1 << 5))   /* The rest state of the clock (between frames) is high.*/
/** SPI control 1 loopback mode enable  */
#define SPI_CFG_LBM_EN      ((uint32_t) (1 << 7))
/** SPI SSEL0 Polarity Select*/
#define SPI_CFG_SPOL0_LO     ((uint32_t) (0 << 8))  /* SSEL0 is active Low */
#define SPI_CFG_SPOL0_HI     ((uint32_t) (1 << 8))  /* SSEL0 is active High */
/** SPI SSEL1 Polarity Select*/
#define SPI_CFG_SPOL1_LO     ((uint32_t) (0 << 9))  /* SSEL1 is active Low */
#define SPI_CFG_SPOL1_HI     ((uint32_t) (1 << 9))  /* SSEL1 is active High */
/** SPI SSEL2 Polarity Select*/
/** Note that SSEL2, SSEL3 is only available on SPI0 not on SPI1 */
#define SPI_CFG_SPOL2_LO     ((uint32_t) (0 << 10)) /* SSEL2 is active Low */
#define SPI_CFG_SPOL2_HI     ((uint32_t) (1 << 10)) /* SSEL2 is active High */
/** SPI SSEL3 Polarity Select*/
#define SPI_CFG_SPOL3_LO     ((uint32_t) (0 << 11)) /* SSEL3 is active Low */
#define SPI_CFG_SPOL3_HI     ((uint32_t) (1 << 11)) /* SSEL3 is active High */

/**
 * Macro defines for SPI Delay register
 */
/** SPI DLY Register Mask   */
#define  SPI_DLY_BITMASK        ((uint32_t) 0xFFFF)
/** Controls the amount of time between SSEL assertion and the beginning of a data frame.   */
#define  SPI_DLY_PRE_DELAY(n)        ((uint32_t) ((n) & 0x0F))              /* Time Unit: SPI clock time */
/** Controls the amount of time between the end of a data frame and SSEL deassertion.   */
#define  SPI_DLY_POST_DELAY(n)       ((uint32_t) (((n) & 0x0F) << 4))       /* Time Unit: SPI clock time */
/** Controls the minimum amount of time between adjacent data frames.   */
#define  SPI_DLY_FRAME_DELAY(n)      ((uint32_t) (((n) & 0x0F) << 8))       /* Time Unit: SPI clock time */
/** Controls the minimum amount of time that the SSEL is deasserted between transfers.  */
#define  SPI_DLY_TRANSFER_DELAY(n)   ((uint32_t) (((n) & 0x0F) << 12))  /* Time Unit: SPI clock time */

/**
 * Macro defines for SPI Status register
 */
/* SPI STAT Register BitMask */
#define SPI_STAT_BITMASK        ((uint32_t) 0x1FF)
/* Receiver Ready Flag */
#define SPI_STAT_RXRDY          ((uint32_t) (1 << 0))   /* Data is ready for read */
/* Transmitter Ready Flag */
#define SPI_STAT_TXRDY          ((uint32_t) (1 << 1))   /* Data may be written to transmit buffer */
/* Receiver Overrun interrupt flag */
#define SPI_STAT_RXOV           ((uint32_t) (1 << 2))   /* Data comes while receiver buffer is in used */
/* Transmitter Underrun interrupt flag (In Slave Mode only) */
#define SPI_STAT_TXUR           ((uint32_t) (1 << 3))   /* There is no data to be sent in the next input clock */
/* Slave Select Assert */
#define SPI_STAT_SSA            ((uint32_t) (1 << 4))   /* There is SSEL transition from deasserted to asserted */
/* Slave Select Deassert */
#define SPI_STAT_SSD            ((uint32_t) (1 << 5))   /* There is SSEL transition from asserted to deasserted */
/* Stalled status flag */
#define SPI_STAT_STALLED        ((uint32_t) (1 << 6))   /* SPI is currently in a stall condition. */
/* End Transfer flag. */
#define SPI_STAT_EOT            ((uint32_t) (1 << 7))   /* The current frame is the last frame of the current  transfer. */
/* Master Idle status flag. */
#define SPI_STAT_MSTIDLE         ((uint32_t) (1 << 8))  /* SPI master function is fully idle. */

/* Clear RXOV Flag */
#define SPI_STAT_CLR_RXOV       ((uint32_t) (1 << 2))
/* Clear TXUR Flag */
#define SPI_STAT_CLR_TXUR       ((uint32_t) (1 << 3))
/* Clear SSA Flag */
#define SPI_STAT_CLR_SSA        ((uint32_t) (1 << 4))
/* Clear SSD Flag */
#define SPI_STAT_CLR_SSD        ((uint32_t) (1 << 5))
/*Force an end to the current transfer */
#define SPI_STAT_FORCE_EOT      ((uint32_t) (1 << 7))

/**
 * Macro defines for SPI Interrupt Enable read and Set register
 */
/* SPI INTENSET Register BitMask */
#define SPI_INTENSET_BITMASK    ((uint32_t) 0x3F)
/** Enable Interrupt when receiver data is available */
#define SPI_INTENSET_RXRDYEN     ((uint32_t) (1 << 0))
/** Enable Interrupt when the transmitter holding register is available. */
#define SPI_INTENSET_TXRDYEN     ((uint32_t) (1 << 1))
/**  Enable Interrupt when a receiver overrun occurs */
#define SPI_INTENSET_RXOVEN     ((uint32_t) (1 << 2))
/**  Enable Interrupt when a transmitter underrun occurs (In Slave Mode Only)*/
#define SPI_INTENSET_TXUREN     ((uint32_t) (1 << 3))
/**  Enable Interrupt when the Slave Select is asserted.*/
#define SPI_INTENSET_SSAEN      ((uint32_t) (1 << 4))
/**  Enable Interrupt when the Slave Select is deasserted..*/
#define SPI_INTENSET_SSDEN      ((uint32_t) (1 << 5))

/**
 * Macro defines for SPI Interrupt Enable Clear register
 */
/* SPI INTENCLR Register BitMask */
#define SPI_INTENCLR_BITMASK    ((uint32_t) 0x3F)
/** Disable Interrupt when receiver data is available */
#define SPI_INTENCLR_RXRDYEN     ((uint32_t) (1 << 0))
/** Disable Interrupt when the transmitter holding register is available. */
#define SPI_INTENCLR_TXRDYEN     ((uint32_t) (1 << 1))
/** Disable Interrupt when a receiver overrun occurs */
#define SPI_INTENCLR_RXOVEN     ((uint32_t) (1 << 2))
/** Disable Interrupt when a transmitter underrun occurs (In Slave Mode Only)*/
#define SPI_INTENCLR_TXUREN     ((uint32_t) (1 << 3))
/** Disable Interrupt when the Slave Select is asserted.*/
#define SPI_INTENCLR_SSAEN      ((uint32_t) (1 << 4))
/** Disable Interrupt when the Slave Select is deasserted..*/
#define SPI_INTENCLR_SSDEN      ((uint32_t) (1 << 5))

/**
 * Macro defines for SPI Receiver Data register
 */
/* SPI RXDAT Register BitMask */
#define SPI_RXDAT_BITMASK       ((uint32_t) 0x1FFFFF)
/** Receiver Data  */
#define SPI_RXDAT_DATA(n)       ((uint32_t) ((n) & 0xFFFF))
/** The state of SSEL0 pin  */
#define SPI_RXDAT_RXSSEL0_ACTIVE    ((uint32_t) (0 << 16))  /* SSEL0 is in active state */
#define SPI_RXDAT_RXSSEL0_INACTIVE  ((uint32_t) (1 << 16))  /* SSEL0 is in inactive state */
#define SPI_RXDAT_RXSSEL0_FLAG          ((uint32_t) (1 << 16))  /* SSEL0 Rx Flag */
/** The state of SSEL1 pin  */
#define SPI_RXDAT_RXSSEL1_ACTIVE    ((uint32_t) (0 << 17))  /* SSEL1 is in active state */
#define SPI_RXDAT_RXSSEL1_INACTIVE  ((uint32_t) (1 << 17))  /* SSEL1 is in inactive state */
#define SPI_RXDAT_RXSSEL1_FLAG          ((uint32_t) (1 << 17))  /* SSEL1 Rx Flag */
/** The state of SSEL2 pin  */
#define SPI_RXDAT_RXSSEL2_ACTIVE    ((uint32_t) (0 << 18))  /* SSEL2 is in active state */
#define SPI_RXDAT_RXSSEL2_INACTIVE  ((uint32_t) (1 << 18))  /* SSEL2 is in inactive state */
#define SPI_RXDAT_RXSSEL2_FLAG          ((uint32_t) (1 << 18))  /* SSEL2 Rx Flag */
/** The state of SSEL3 pin  */
#define SPI_RXDAT_RXSSEL3_ACTIVE    ((uint32_t) (0 << 19))  /* SSEL3 is in active state */
#define SPI_RXDAT_RXSSEL3_INACTIVE  ((uint32_t) (1 << 19))  /* SSEL3 is in inactive state */
#define SPI_RXDAT_RXSSEL3_FLAG          ((uint32_t) (1 << 19))  /* SSEL3 Rx Flag */
/** Start of Transfer flag  */
#define SPI_RXDAT_SOT           ((uint32_t) (1 << 20))  /* This is the first frame received after SSEL is asserted */

/**
 * Macro defines for SPI Transmitter Data and Control register
 */
/* SPI TXDATCTL Register BitMask */
#define SPI_TXDATCTL_BITMASK    ((uint32_t) 0xF7FFFFF)
/* SPI Transmit Data */
#define SPI_TXDATCTL_DATA(n)    ((uint32_t) ((n) & 0xFFFF))
/*Assert/Deassert SSEL0 pin*/
#define SPI_TXDATCTL_ASSERT_SSEL0    ((uint32_t) (0 << 16))
#define SPI_TXDATCTL_DEASSERT_SSEL0  ((uint32_t) (1 << 16))
/*Assert/Deassert SSEL1 pin*/
#define SPI_TXDATCTL_ASSERT_SSEL1    ((uint32_t) (0 << 17))
#define SPI_TXDATCTL_DEASSERT_SSEL1  ((uint32_t) (1 << 17))
/*Assert/Deassert SSEL2 pin*/
/** Note that SSEL2, SSEL3 is only available on SPI0 not on SPI1 */
#define SPI_TXDATCTL_ASSERT_SSEL2    ((uint32_t) (0 << 18))
#define SPI_TXDATCTL_DEASSERT_SSEL2  ((uint32_t) (1 << 18))
/*Assert/Deassert SSEL3 pin*/
#define SPI_TXDATCTL_ASSERT_SSEL3    ((uint32_t) (0 << 19))
#define SPI_TXDATCTL_DEASSERT_SSEL3  ((uint32_t) (1 << 19))
/* Mask for Slave Select bits */
#define SPI_TXDATCTL_SSEL_MASK       ((uint32_t) (0x0F0000))

/** End of Transfer flag (TRANSFER_DELAY is applied after sending the current frame)  */
#define SPI_TXDATCTL_EOT            ((uint32_t) (1 << 20))  /* This is the last frame of the current transfer */
/** End of Frame flag (FRAME_DELAY is applied after sending the current part) */
#define SPI_TXDATCTL_EOF            ((uint32_t) (1 << 21))  /* This is the last part of the current frame */
/** Receive Ignore Flag */
#define SPI_TXDATCTL_RXIGNORE       ((uint32_t) (1 << 22))  /* Received data is ignored */
/** Transmit Data Length */
#define SPI_TXDATCTL_LEN(n)        ((uint32_t) (((n) & 0x0F) << 24))    /* Frame Length -1 */

/**
 * Macro defines for SPI Transmitter Data Register
 */
/* SPI Transmit Data */
#define SPI_TXDAT_DATA(n)   ((uint32_t) ((n) & 0xFFFF))

/**
 * Macro defines for SPI Transmitter Control register
 */
/* SPI TXDATCTL Register BitMask */
#define SPI_TXCTL_BITMASK   ((uint32_t) 0xF7F0000)
/*Assert/Deassert SSEL0 pin*/
#define SPI_TXCTL_ASSERT_SSEL0   ((uint32_t) (0 << 16))
#define SPI_TXCTL_DEASSERT_SSEL0 ((uint32_t) (1 << 16))
/*Assert/Deassert SSEL1 pin*/
#define SPI_TXCTL_ASSERT_SSEL1   ((uint32_t) (0 << 17))
#define SPI_TXCTL_DEASSERT_SSEL1 ((uint32_t) (1 << 17))
/*Assert/Deassert SSEL2 pin*/
/** Note that SSEL2, SSEL3 is only available on SPI0 not on SPI1 */
#define SPI_TXCTL_ASSERT_SSEL2   ((uint32_t) (0 << 18))
#define SPI_TXCTL_DEASSERT_SSEL2 ((uint32_t) (1 << 18))
/*Assert/Deassert SSEL3 pin*/
#define SPI_TXCTL_ASSERT_SSEL3   ((uint32_t) (0 << 19))
#define SPI_TXCTL_DEASSERT_SSEL3 ((uint32_t) (1 << 19))
/** End of Transfer flag (TRANSFER_DELAY is applied after sending the current frame)  */
#define SPI_TXCTL_EOT           ((uint32_t) (1 << 20))  /* This is the last frame of the current transfer */
/** End of Frame flag (FRAME_DELAY is applied after sending the current part) */
#define SPI_TXCTL_EOF           ((uint32_t) (1 << 21))  /* This is the last part of the current frame */
/** Receive Ignore Flag */
#define SPI_TXCTL_RXIGNORE      ((uint32_t) (1 << 22))  /* Received data is ignored */
/** Transmit Data Length */
#define SPI_TXCTL_LEN(n)       ((uint32_t) (((n) & 0x0F) << 24))    /* Frame Length -1 */

/**
 * Macro defines for SPI Divider register
 */
/** Rate divider value  (In Master Mode only)*/
#define SPI_DIV_VAL(n)          ((uint32_t) ((n) & 0xFFFF)) /* SPI_CLK = PCLK/(DIV_VAL+1)*/

/**
 * Macro defines for SPI Interrupt Status register
 */
/* SPI INTSTAT Register Bitmask */
#define SPI_INTSTAT_BITMASK     ((uint32_t) 0x3F)
/* Receiver Ready Flag */
#define SPI_INTSTAT_RXRDY           ((uint32_t) (1 << 0))   /* Data is ready for read */
/* Transmitter Ready Flag */
#define SPI_INTSTAT_TXRDY           ((uint32_t) (1 << 1))   /* Data may be written to transmit buffer */
/* Receiver Overrun interrupt flag */
#define SPI_INTSTAT_RXOV            ((uint32_t) (1 << 2))   /* Data comes while receiver buffer is in used */
/* Transmitter Underrun interrupt flag (In Slave Mode only) */
#define SPI_INTSTAT_TXUR            ((uint32_t) (1 << 3))   /* There is no data to be sent in the next input clock */
/* Slave Select Assert */
#define SPI_INTSTAT_SSA         ((uint32_t) (1 << 4))   /* There is SSEL transition from deasserted to asserted */
/* Slave Select Deassert */
#define SPI_INTSTAT_SSD         ((uint32_t) (1 << 5))   /* There is SSEL transition from asserted to deasserted */

/* public function */
rt_err_t lpc_spi_register(LPC_SPI0_Type *SPI,
                          struct lpc_spi_bus *lpc_spi,
                          const char *spi_bus_name);
int rt_hw_spi_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __DRV_SPI_H
