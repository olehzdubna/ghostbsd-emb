/**
 *  \file   mcasp.h
 *
 *  \brief  mcasp_ API prototypes and macros.
 *
 *   This file contains the driver API prototypes and macro definitions.
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef __MCASP_H__
#define __MCASP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/ti/ti_cpuid.h>
#include <arm/ti/ti_prcm.h>
#include <arm/ti/ti_hwmods.h>

#include "ti_mcasp_hw.h"
#include "ti_mcasp_ioctl.h"

/*****************************************************************************/
/*
** Macros to combine to pass as formatVal to the API mcasp_TxFmtSet.
** The value formatVal is directly written to the register. So
** proper combination of the below macros shall be selected
** Thus the default value is 
** (MCASP_TX_SYNC_DELAY_0BIT | MCASP_TX_BITSTREAM_LSB_FIRST |
**  MCASP_TX_PAD_WITH_0 | MCASP_TX_SLOTSIZE_8BITS |MCASP_TX_BUF_DMAPORT
**  MCASP_TX_ROT_RIGHT_NONE)
*/
/* Transmit Sync Bit delay */
#define MCASP_TX_SYNC_DELAY_0BIT             (MCASP_XFMT_XDATDLY_0BIT \
                                              << MCASP_XFMT_XDATDLY_SHIFT)
#define MCASP_TX_SYNC_DELAY_1BIT             (MCASP_XFMT_XDATDLY_1BIT \
                                              << MCASP_XFMT_XDATDLY_SHIFT)
#define MCASP_TX_SYNC_DELAY_2BIT             (MCASP_XFMT_XDATDLY_2BIT \
                                              << MCASP_XFMT_XDATDLY_SHIFT)

/* Bit Stream output, whether MSB or LSB shall be out first */
#define MCASP_TX_BITSTREAM_LSB_FIRST         (0x0u)
#define MCASP_TX_BITSTREAM_MSB_FIRST         (0x00008000u)

/* Padding options for unused bits */
#define MCASP_TX_PAD_WITH_0                  (0x00000000u)
#define MCASP_TX_PAD_WITH_1                  (0x00002000u)
#define MCASP_TX_PAD_WITH_PBIT(n)            ((0x00004000u) | (n \
                                               << MCASP_XFMT_XPBIT_SHIFT))

/* Transmit slot size to be used */
#define MCASP_TX_SLOTSIZE_8BITS              (MCASP_XFMT_XSSZ_8BITS \
                                              << MCASP_XFMT_XSSZ_SHIFT)
#define MCASP_TX_SLOTSIZE_12BITS             (MCASP_XFMT_XSSZ_12BITS \
                                              << MCASP_XFMT_XSSZ_SHIFT)
#define MCASP_TX_SLOTSIZE_16BITS             (MCASP_XFMT_XSSZ_16BITS \
                                              << MCASP_XFMT_XSSZ_SHIFT)
#define MCASP_TX_SLOTSIZE_20BITS             (MCASP_XFMT_XSSZ_20BITS \
                                               << MCASP_XFMT_XSSZ_SHIFT)
#define MCASP_TX_SLOTSIZE_24BITS             (MCASP_XFMT_XSSZ_24BITS \
                                               << MCASP_XFMT_XSSZ_SHIFT)
#define MCASP_TX_SLOTSIZE_28BITS             (MCASP_XFMT_XSSZ_28BITS \
                                               << MCASP_XFMT_XSSZ_SHIFT)
#define MCASP_TX_SLOTSIZE_32BITS             (MCASP_XFMT_XSSZ_32BITS \
                                               << MCASP_XFMT_XSSZ_SHIFT)

/* Transmit buffer write origin */
#define MCASP_TX_BUF_DMAPORT                  (0x0u)
#define MCASP_TX_BUF_PERICONFIGPORT           (0x8u)

/* Rotate value for the transmit rotate right format unit */
#define MCASP_TX_ROT_RIGHT_NONE                MCASP_XFMT_XROT_NONE
#define MCASP_TX_ROT_RIGHT_4BITS               MCASP_XFMT_XROT_4BITS
#define MCASP_TX_ROT_RIGHT_8BITS               MCASP_XFMT_XROT_8BITS
#define MCASP_TX_ROT_RIGHT_12BITS              MCASP_XFMT_XROT_12BITS
#define MCASP_TX_ROT_RIGHT_16BITS              MCASP_XFMT_XROT_16BITS
#define MCASP_TX_ROT_RIGHT_20BITS              MCASP_XFMT_XROT_20BITS
#define MCASP_TX_ROT_RIGHT_24BITS              MCASP_XFMT_XROT_24BITS
#define MCASP_TX_ROT_RIGHT_28BITS              MCASP_XFMT_XROT_28BITS

/*****************************************************************************/
/*
** Macros to combine to pass as formatVal to the API mcasp_RxFmtSet.
** The value formatVal is directly written to the register. So
** proper combination of the below macros shall be selected
** Thus the default value is 
** (MCASP_RX_SYNC_DELAY_0BIT | MCASP_RX_BITSTREAM_LSB_FIRST |
**  MCASP_RX_PAD_WITH_0 | MCASP_RX_SLOTSIZE_8BITS |MCASP_RX_BUF_DMAPORT
**  MCASP_RX_ROT_RIGHT_NONE)
*/
/* Receive Bit delay */
#define MCASP_RX_SYNC_DELAY_0BIT             (MCASP_RFMT_RDATDLY_0BIT \
                                              << MCASP_RFMT_RDATDLY_SHIFT)
#define MCASP_RX_SYNC_DELAY_1BIT             (MCASP_RFMT_RDATDLY_1BIT \
                                              << MCASP_RFMT_RDATDLY_SHIFT)
#define MCASP_RX_SYNC_DELAY_2BIT             (MCASP_RFMT_RDATDLY_2BIT \
                                              << MCASP_RFMT_RDATDLY_SHIFT)

/* Bit Stream input, whether MSB or LSB shall be out first */
#define MCASP_RX_BITSTREAM_LSB_FIRST         (0x0u)
#define MCASP_RX_BITSTREAM_MSB_FIRST         (0x00008000u)

/* Padding options for unused bits */
#define MCASP_RX_PAD_WITH_0                  (0x00000000u)
#define MCASP_RX_PAD_WITH_1                  (0x00002000u)
#define MCASP_RX_PAD_WITH_PBIT(n)            ((0x00004000u) | (n \
                                               << MCASP_RFMT_RPBIT_SHIFT))

/* Receive slot size to be used */
#define MCASP_RX_SLOTSIZE_8BITS              (MCASP_RFMT_RSSZ_8BITS \
                                              << MCASP_RFMT_RSSZ_SHIFT)
#define MCASP_RX_SLOTSIZE_12BITS             (MCASP_RFMT_RSSZ_12BITS \
                                              << MCASP_RFMT_RSSZ_SHIFT)
#define MCASP_RX_SLOTSIZE_16BITS             (MCASP_RFMT_RSSZ_16BITS \
                                              << MCASP_RFMT_RSSZ_SHIFT)
#define MCASP_RX_SLOTSIZE_20BITS             (MCASP_RFMT_RSSZ_20BITS \
                                               << MCASP_RFMT_RSSZ_SHIFT)
#define MCASP_RX_SLOTSIZE_24BITS             (MCASP_RFMT_RSSZ_24BITS \
                                               << MCASP_RFMT_RSSZ_SHIFT)
#define MCASP_RX_SLOTSIZE_28BITS             (MCASP_RFMT_RSSZ_28BITS \
                                               << MCASP_RFMT_RSSZ_SHIFT)
#define MCASP_RX_SLOTSIZE_32BITS             (MCASP_RFMT_RSSZ_32BITS \
                                               << MCASP_RFMT_RSSZ_SHIFT)

/* Receive buffer origin */
#define MCASP_RX_BUF_DMAPORT                  (0x0u)
#define MCASP_RX_BUF_PERICONFIGPORT           (0x8u)

/* Rotate value for the receive rotate right format unit */
#define MCASP_RX_ROT_RIGHT_NONE                MCASP_RFMT_RROT_NONE
#define MCASP_RX_ROT_RIGHT_4BITS               MCASP_RFMT_RROT_4BITS
#define MCASP_RX_ROT_RIGHT_8BITS               MCASP_RFMT_RROT_8BITS
#define MCASP_RX_ROT_RIGHT_12BITS              MCASP_RFMT_RROT_12BITS
#define MCASP_RX_ROT_RIGHT_16BITS              MCASP_RFMT_RROT_16BITS
#define MCASP_RX_ROT_RIGHT_20BITS              MCASP_RFMT_RROT_20BITS
#define MCASP_RX_ROT_RIGHT_24BITS              MCASP_RFMT_RROT_24BITS
#define MCASP_RX_ROT_RIGHT_28BITS              MCASP_RFMT_RROT_28BITS

/*****************************************************************************/
/*
** Macros which can be passed as txMode to mcasp_TxFmtI2SSet API.
*/
#define MCASP_TX_MODE_NON_DMA                  MCASP_TX_BUF_PERICONFIGPORT
#define MCASP_TX_MODE_DMA                      MCASP_TX_BUF_DMAPORT

/*****************************************************************************/
/*
i** Macros which can be passed as rxMode to mcasp_RxFmtI2SSet API.
*/
#define MCASP_RX_MODE_NON_DMA                  MCASP_RX_BUF_PERICONFIGPORT
#define MCASP_RX_MODE_DMA                      MCASP_RX_BUF_DMAPORT

/*****************************************************************************/
/*
** Macros which can be passed as fsWidth to mcasp_TxFrameSyncCfg API.
*/
#define MCASP_TX_FS_WIDTH_BIT                  (0x00000000u)
#define MCASP_TX_FS_WIDTH_WORD                 (0x00000010u)

/*
** Macros which can be passed as fsSetting to mcasp_TxFrameSyncCfg API.
*/
#define MCASP_TX_FS_INT_BEGIN_ON_RIS_EDGE      (0x00000002u)
#define MCASP_TX_FS_INT_BEGIN_ON_FALL_EDGE     (0x00000003u)
#define MCASP_TX_FS_EXT_BEGIN_ON_RIS_EDGE      (0x00000000u)
#define MCASP_TX_FS_EXT_BEGIN_ON_FALL_EDGE     (0x00000001u)

/*****************************************************************************/
/*
** Macros which can be passed as fsWidth to mcasp_RxFrameSyncCfg API.
*/
#define MCASP_RX_FS_WIDTH_BIT                  (0x00000000u)
#define MCASP_RX_FS_WIDTH_WORD                 (0x00000010u)

/*
** Macros which can be passed as fsSetting to mcasp_rx_frame_sync_cfg API.
*/
#define MCASP_RX_FS_INT_BEGIN_ON_RIS_EDGE      (0x00000002u)
#define MCASP_RX_FS_INT_BEGIN_ON_FALL_EDGE     (0x00000003u)
#define MCASP_RX_FS_EXT_BEGIN_ON_RIS_EDGE      (0x00000000u)
#define MCASP_RX_FS_EXT_BEGIN_ON_FALL_EDGE     (0x00000001u)

/*****************************************************************************/
/*
** Macros which can be passed as clkSrc to mcasp_TxClkCfg API and
** mcasp_tx_clk_start API.
*/
#define MCASP_TX_CLK_INTERNAL                  (0x00008020u)
#define MCASP_TX_CLK_EXTERNAL                  (0x00000000u)
#define MCASP_TX_CLK_MIXED                     (0x00000020u)

/*****************************************************************************/
/*
** Macros which can be passed as clkSrc to mcasp_RxClkCfg API and
** mcasp_rx_clk_start API.
*/
#define MCASP_RX_CLK_INTERNAL                  (0x00008020u)
#define MCASP_RX_CLK_EXTERNAL                  (0x00000000u)
#define MCASP_RX_CLK_MIXED                     (0x00000020u)

/*****************************************************************************/
/*
** Macros which can be passed as polarity to mcasp_tx_clk_polarity_set API.
*/
#define MCASP_TX_CLK_POL_RIS_EDGE              (0x00000000u)
#define MCASP_TX_CLK_POL_FALL_EDGE             (0x00000080u)

/*****************************************************************************/
/*
** Macros which can be passed as polarity to mcasp_rx_clk_polarity_set API.
*/
#define MCASP_RX_CLK_POL_RIS_EDGE              (0x00000000u)
#define MCASP_RX_CLK_POL_FALL_EDGE             (0x00000080u)

/*****************************************************************************/
/*
** Macros which can be passed as polarity to the API mcasp_tx_hf_clk_polarity_set.
*/
#define MCASP_TX_HI_FREQ_CLK_NO_INVERT         (0x00000000u)
#define MCASP_TX_HI_FREQ_CLK_INVERT            (MCASP_AHCLKXCTL_HCLKXP)

/*****************************************************************************/
/*
** Macros which can be passed as polarity to the API mcasp_RxHFClkPolaritySet.
*/
#define MCASP_RX_HI_FREQ_CLK_NO_INVERT         (0x00000000u)
#define MCASP_RX_HI_FREQ_CLK_INVERT            (MCASP_AHCLKRCTL_HCLKRP)

/*****************************************************************************/
/*
** Macros which can be passed as intMask to the APIs mcasp_TxIntEnable/Disable
** intMask can be an OR combination of the below macros
*/
#define MCASP_TX_STARTOFFRAME                  (MCASP_XINTCTL_XSTAFRM)
#define MCASP_TX_DATAREADY                     (MCASP_XINTCTL_XDATA)
#define MCASP_TX_LASTSLOT                      (MCASP_XINTCTL_XLAST)
#define MCASP_TX_DMAERROR                      (MCASP_XINTCTL_XDMAERR)
#define MCASP_TX_CLKFAIL                       (MCASP_XINTCTL_XCKFAIL)
#define MCASP_TX_SYNCERROR                     (MCASP_XINTCTL_XSYNCERR)
#define MCASP_TX_UNDERRUN                      (MCASP_XINTCTL_XUNDRN)

/*****************************************************************************/
/*
** Macros which can be passed as intMask to the APIs mcasp_RxIntEnable/Disable
** intMask can be an OR combination of the below macros
*/
#define MCASP_RX_STARTOFFRAME                  (MCASP_RINTCTL_RSTAFRM)
#define MCASP_RX_DATAREADY                     (MCASP_RINTCTL_RDATA)
#define MCASP_RX_LASTSLOT                      (MCASP_RINTCTL_RLAST)
#define MCASP_RX_DMAERROR                      (MCASP_RINTCTL_RDMAERR)
#define MCASP_RX_CLKFAIL                       (MCASP_RINTCTL_RCKFAIL)
#define MCASP_RX_SYNCERROR                     (MCASP_RINTCTL_RSYNCERR)
#define MCASP_RX_OVERRUN                       (MCASP_RINTCTL_ROVRN)

/*****************************************************************************/
/*
** Macros which can be used in pinMask to the APIs 
** mcasp_PinDirOutput and Setmcasp_PinDirInputSet, mcasp_pinmcasp_set
** and mcasp_PinGPIOSet
*/
#define MCASP_PIN_AFSR                         (MCASP_PDIR_AFSR)
#define MCASP_PIN_AHCLKR                       (MCASP_PDIR_AHCLKR)
#define MCASP_PIN_ACLKR                        (MCASP_PDIR_ACLKR)
#define MCASP_PIN_AFSX                         (MCASP_PDIR_AFSX)
#define MCASP_PIN_AHCLKX                       (MCASP_PDIR_AHCLKX)
#define MCASP_PIN_ACLKX                        (MCASP_PDIR_ACLKX)
#define MCASP_PIN_AMUTE                        (MCASP_PDIR_AMUTE)
#define MCASP_PIN_AXR(n)                       (1u << n)

/*****************************************************************************/
/*
** Macros which can be passed as errFlags to the API mcasp_TxAMuteEnable
*/
#define MCASP_AMUTE_TX_DMAERROR                (MCASP_AMUTE_XDMAERR)
#define MCASP_AMUTE_TX_CLKFAIL                 (MCASP_AMUTE_XCKFAIL)
#define MCASP_AMUTE_TX_SYNCERROR               (MCASP_AMUTE_XSYNCERR)
#define MCASP_AMUTE_TX_UNDERRUN                (MCASP_AMUTE_XUNDRN)
#define MCASP_AMUTE_RX_DMAERROR                (MCASP_AMUTE_RDMAERR)
#define MCASP_AMUTE_RX_CLKFAIL                 (MCASP_AMUTE_RCKFAIL)
#define MCASP_AMUTE_RX_SYNCERROR               (MCASP_AMUTE_RSYNCERR)
#define MCASP_AMUTE_RX_OVERRUN                 (MCASP_AMUTE_ROVRN)

/*
** Macros which can be passed as pinState to the API mcasp_TxAMuteEnable
*/
#define MCASP_AMUTE_PIN_HIGH                   (0x00000001)
#define MCASP_AMUTE_PIN_LOW                    (0x00000002)

/*****************************************************************************/
/*
** Macros which can be passed as clkDiv to the API mcasp_TxClkCheckConfig
*/
#define MCASP_TX_CLKCHCK_DIV1                  (MCASP_XCLKCHK_XPS_DIVBY1)
#define MCASP_TX_CLKCHCK_DIV2                  (MCASP_XCLKCHK_XPS_DIVBY2)
#define MCASP_TX_CLKCHCK_DIV4                  (MCASP_XCLKCHK_XPS_DIVBY4)
#define MCASP_TX_CLKCHCK_DIV8                  (MCASP_XCLKCHK_XPS_DIVBY8)
#define MCASP_TX_CLKCHCK_DIV16                 (MCASP_XCLKCHK_XPS_DIVBY16)
#define MCASP_TX_CLKCHCK_DIV32                 (MCASP_XCLKCHK_XPS_DIVBY32)
#define MCASP_TX_CLKCHCK_DIV64                 (MCASP_XCLKCHK_XPS_DIVBY64)
#define MCASP_TX_CLKCHCK_DIV128                (MCASP_XCLKCHK_XPS_DIVBY128)
#define MCASP_TX_CLKCHCK_DIV256                (MCASP_XCLKCHK_XPS_DIVBY256)

/*****************************************************************************/
/*
** Macros which can be passed as clkDiv to the API mcasp_RxClkCheckConfig
*/
#define MCASP_RX_CLKCHCK_DIV1                  (MCASP_RCLKCHK_RPS_DIVBY1)
#define MCASP_RX_CLKCHCK_DIV2                  (MCASP_RCLKCHK_RPS_DIVBY2)
#define MCASP_RX_CLKCHCK_DIV4                  (MCASP_RCLKCHK_RPS_DIVBY4)
#define MCASP_RX_CLKCHCK_DIV8                  (MCASP_RCLKCHK_RPS_DIVBY8)
#define MCASP_RX_CLKCHCK_DIV16                 (MCASP_RCLKCHK_RPS_DIVBY16)
#define MCASP_RX_CLKCHCK_DIV32                 (MCASP_RCLKCHK_RPS_DIVBY32)
#define MCASP_RX_CLKCHCK_DIV64                 (MCASP_RCLKCHK_RPS_DIVBY64)
#define MCASP_RX_CLKCHCK_DIV128                (MCASP_RCLKCHK_RPS_DIVBY128)
#define MCASP_RX_CLKCHCK_DIV256                (MCASP_RCLKCHK_RPS_DIVBY256)

/*****************************************************************************/
/*
** Macros which can be passed as vBit to the API mcasp_DITEnable
*/
#define MCASP_DIT_VBIT_ODD_0_EVEN_1            (0x00000004)
#define MCASP_DIT_VBIT_ODD_1_EVEN_0            (0x00000008)
#define MCASP_DIT_VBIT_BOTHSLOTS_0             (0x00000000)
#define MCASP_DIT_VBIT_BOTHSLOTS_1             (0x0000000C)

/*****************************************************************************/
/*
** Macros which can be passed as channel to the APIs mcasp_DITChanStatWrite
** and mcasp_DITChanStatRead, mcasp_dit_chan_usr_data_write and
** mcasp_DITChanUsrDataRead
*/
#define MCASP_DIT_CHANNEL_LEFT                 (0x000000F0)
#define MCASP_DIT_CHANNEL_RIGHT                (0x0000000F)
#define MCASP_DIT_CHANNEL_BOTH                 (0x000000FF)

/*****************************************************************************/
/*
** Macros which can be passed as chStatBits to the APIs mcasp_dit_chan_stat_write
** and mcasp_DITChanStatRead
*/
#define MCASP_DIT_CHSTAT_BITS_0_31             (0x00000000)
#define MCASP_DIT_CHSTAT_BITS_32_63            (0x00000001)
#define MCASP_DIT_CHSTAT_BITS_64_95            (0x00000002)
#define MCASP_DIT_CHSTAT_BITS_96_127           (0x00000003)
#define MCASP_DIT_CHSTAT_BITS_128_159          (0x00000004)
#define MCASP_DIT_CHSTAT_BITS_160_191          (0x00000005)

/*****************************************************************************/
/*
** Macros which can be passed as usrDataBits to the APIs 
** mcasp_dit_chan_usr_data_write and mcasp_dit_chan_usr_data_read.
*/
#define MCASP_DIT_USRDATA_BITS_0_31            (0x00000000)
#define MCASP_DIT_USRDATA_BITS_32_63           (0x00000001)
#define MCASP_DIT_USRDATA_BITS_64_95           (0x00000002)
#define MCASP_DIT_USRDATA_BITS_96_127          (0x00000003)
#define MCASP_DIT_USRDATA_BITS_128_159         (0x00000004)
#define MCASP_DIT_USRDATA_BITS_160_191         (0x00000005)

/*****************************************************************************/
/*
** Macros for tokens returned by the API mcasp_tx_status_get
*/
#define MCASP_TX_STAT_ERR                      (MCASP_XSTAT_XERR)
#define MCASP_TX_STAT_DMAERR                   (MCASP_XSTAT_XDMAERR)
#define MCASP_TX_STAT_STARTOFFRAME             (MCASP_XSTAT_XSTAFRM)
#define MCASP_TX_STAT_DATAREADY                (MCASP_XSTAT_XDATA)
#define MCASP_TX_STAT_LASTSLOT                 (MCASP_XSTAT_XLAST)
#define MCASP_TX_STAT_CURRSLOT_EVEN            (MCASP_XSTAT_XTDMSLOT)
#define MCASP_TX_STAT_CURRSLOT_ODD             (0x00000000)
#define MCASP_TX_STAT_CLKFAIL                  (MCASP_XSTAT_XCKFAIL)
#define MCASP_TX_STAT_SYNCERR                  (MCASP_XSTAT_XSYNCERR)
#define MCASP_TX_STAT_UNDERRUN                 (MCASP_XSTAT_XUNDRN)

/*****************************************************************************/
/*
** Macros for tokens returned by the API mcasp_rx_status_get
*/
#define MCASP_RX_STAT_ERR                      (MCASP_RSTAT_RERR)
#define MCASP_RX_STAT_DMAERR                   (MCASP_RSTAT_RDMAERR)
#define MCASP_RX_STAT_STARTOFFRAME             (MCASP_RSTAT_RSTAFRM)
#define MCASP_RX_STAT_DATAREADY                (MCASP_RSTAT_RDATA)
#define MCASP_RX_STAT_LASTSLOT                 (MCASP_RSTAT_RLAST)
#define MCASP_RX_STAT_CURRSLOT_EVEN            (MCASP_RSTAT_RTDMSLOT)
#define MCASP_RX_STAT_CURRSLOT_ODD             (0x00000000)
#define MCASP_RX_STAT_CLKFAIL                  (MCASP_RSTAT_RCKFAIL)
#define MCASP_RX_STAT_SYNCERR                  (MCASP_RSTAT_RSYNCERR)
#define MCASP_RX_STAT_OVERRUN                  (MCASP_RSTAT_ROVRN)

/*****************************************************************************/
/*
** Macros to be used for the variable 'sectFlag' for 'mcasp_ContextSave'
** and 'mcasp_context_restore'
*/
#define MCASP_CONTEXT_TX                       (0x01)
#define MCASP_CONTEXT_RX                       (0x02)
#define MCASP_CONTEXT_BOTH                     (0x03)

struct ti_mcasp_softc {
    device_t		dev;
    struct resource *	mem_res;
    struct resource *	irq_res;
    void *		intr_cookie;
    clk_ident_t		mcasp_clk_id;
    uint32_t		baseclk_hz;
};

/*****************************************************************************/
/*
** Structure to store the mcasp_ context
*/
typedef struct mcasp_context {
    uint32_t fifoWfifoCtl;
    uint32_t xmask;
    uint32_t xfmt;
    uint32_t afsxctl;
    uint32_t aclkxctl;
    uint32_t ahclkxctl;
    uint32_t xclkchk;
    uint32_t xtdm;
    uint32_t srctl[16];
    uint32_t pfunc;
    uint32_t pdir;
    uint32_t fifoRfifoCtl;
    uint32_t rmask;
    uint32_t rfmt;
    uint32_t afsrctl;
    uint32_t aclkrctl;
    uint32_t ahclkrctl;
    uint32_t rclkchk;
    uint32_t rtdm;
    uint32_t gblctl;
} mcasp_context_t;
/*****************************************************************************/

/*
** Prototypes for the APIs
*/
void mcasp_tx_reset(struct ti_mcasp_softc *sc);
void mcasp_rx_reset(struct ti_mcasp_softc *sc);
void mcasp_write_fifo_enable(struct ti_mcasp_softc *sc, uint32_t numTxSer,
                                 uint32_t minWdPerSer);
void mcasp_read_fifo_enable(struct ti_mcasp_softc *sc, uint32_t numRxSer,
                                uint32_t minWdPerSer);
void mcasp_tx_fmt_mask_set(struct ti_mcasp_softc *sc, uint32_t mask);
void mcasp_rx_fmt_mask_set(struct ti_mcasp_softc *sc, uint32_t mask);
void mcasp_tx_fmt_set(struct ti_mcasp_softc *sc, uint32_t formatVal);
void mcasp_rx_fmt_set(struct ti_mcasp_softc *sc, uint32_t formatVal);
void mcasp_tx_fmt_i2s_set(struct ti_mcasp_softc *sc, uint32_t wordSize,
                             uint32_t slotSize, uint32_t txMode);
void mcasp_rx_fmt_i2s_set(struct ti_mcasp_softc *sc, uint32_t wordSize,
                             uint32_t slotSize, uint32_t rxMode);
void mcasp_tx_frame_sync_cfg(struct ti_mcasp_softc *sc, uint32_t fsMode,
                                uint32_t fsWidth, uint32_t fsSetting);
void mcasp_rx_frame_sync_cfg(struct ti_mcasp_softc *sc, uint32_t fsMode,
                                uint32_t fsWidth, uint32_t fsSetting);
void mcasp_tx_clk_cfg(struct ti_mcasp_softc *sc, uint32_t clkSrc,
                          uint32_t mixClkDiv, uint32_t auxClkDiv);
void mcasp_rx_clk_cfg(struct ti_mcasp_softc *sc, uint32_t clkSrc,
                          uint32_t mixClkDiv, uint32_t auxClkDiv);
void mcasp_tx_clk_polarity_set(struct ti_mcasp_softc *sc, uint32_t polarity);
void mcasp_rx_clk_polarity_set(struct ti_mcasp_softc *sc, uint32_t polarity);
void mcasp_tx_hf_clk_polarity_set(struct ti_mcasp_softc *sc,
                                    uint32_t polarity);
void mcasp_rx_hf_clk_polarity_set(struct ti_mcasp_softc *sc,
                                    uint32_t polarity);
void mcasp_tx_rx_clk_sync_enable(struct ti_mcasp_softc *sc);
void mcasp_tx_rx_clk_sync_disable(struct ti_mcasp_softc *sc);
void mcasp_serializer_tx_set(struct ti_mcasp_softc *sc, uint32_t serNum);
void mcasp_serializer_rx_set(struct ti_mcasp_softc *sc, uint32_t serNum);
void mcasp_serializer_inactivate(struct ti_mcasp_softc *sc,
                                      uint32_t serNum);
void mcasp_pin_dir_output_set(struct ti_mcasp_softc *sc, uint32_t pinMask);
void mcasp_pin_dir_input_set(struct ti_mcasp_softc *sc, uint32_t pinMask);
void mcasp_pin_mcasp_set(struct ti_mcasp_softc *sc, uint32_t pinMask);
void mcasp_pin_gpio_set(struct ti_mcasp_softc *sc, uint32_t pinMask);
void mcasp_tx_time_slot_set(struct ti_mcasp_softc *sc, uint32_t slotMask);
void mcasp_rx_time_slot_set(struct ti_mcasp_softc *sc, uint32_t slotMask);
void mcasp_tx_int_disable(struct ti_mcasp_softc *sc, uint32_t intMask);
void mcasp_rx_int_disable(struct ti_mcasp_softc *sc, uint32_t intMask);
void mcasp_tx_int_enable(struct ti_mcasp_softc *sc, uint32_t intMask);
void mcasp_rx_int_enable(struct ti_mcasp_softc *sc, uint32_t intMask);
void mcasp_tx_clk_start(struct ti_mcasp_softc *sc, uint32_t clkSrc);
void mcasp_rx_clk_start(struct ti_mcasp_softc *sc, uint32_t clkSrc);
void mcasp_tx_ser_activate(struct ti_mcasp_softc *sc);
void mcasp_rx_ser_activate(struct ti_mcasp_softc *sc);
void mcasp_tx_enable(struct ti_mcasp_softc *sc);
void mcasp_rx_enable(struct ti_mcasp_softc *sc);
void mcasp_a_mute_enable(struct ti_mcasp_softc *sc, uint32_t errFlags,
                               uint32_t pinState);
void mcasp_a_mute_disable(struct ti_mcasp_softc *sc);
void mcasp_a_mute_in_activate(struct ti_mcasp_softc *sc, uint32_t polarity);
void mcasp_tx_clk_check_config(struct ti_mcasp_softc *sc, uint32_t clkDiv,
                                  uint8_t boundMin, 
                                  uint8_t boundMax);
void mcasp_rx_clk_check_config(struct ti_mcasp_softc *sc, uint32_t clkDiv,
                                  uint8_t boundMin,
                                  uint8_t boundMax);
void mcasp_tx_buf_write(struct ti_mcasp_softc *sc, uint32_t serNum,
                     uint32_t data);
void mcasp_dit_enable(struct ti_mcasp_softc *sc, uint32_t vBit);
void mcasp_dit_disable(struct ti_mcasp_softc *sc);
void mcasp_dit_chan_stat_write(struct ti_mcasp_softc *sc,
                                  uint32_t chStatBits,
                                  uint32_t channel, 
                                  uint32_t data);
void mcasp_dit_chan_usr_data_write(struct ti_mcasp_softc *sc,
                                     uint32_t chUsrDataBits,
                                     uint32_t channel, 
                                     uint32_t data);
uint32_t mcasp_dit_chan_stat_read(struct ti_mcasp_softc *sc,
                                         uint32_t chStatBits,
                                         uint32_t channel);
uint32_t mcasp_dit_chan_usr_data_read(struct ti_mcasp_softc *sc,
                                            uint32_t chUsrDataBits,
                                            uint32_t channel);
uint32_t mcasp_rx_buf_read(struct ti_mcasp_softc *sc, uint32_t serNum);
uint32_t mcasp_tx_status_get(struct ti_mcasp_softc *sc);
uint32_t mcasp_rx_status_get(struct ti_mcasp_softc *sc);
void mcasp_context_save(struct ti_mcasp_softc *scCtrl, struct ti_mcasp_softc *scFifo,
                             mcasp_context_t *contextPtr, uint32_t sectFlag);
void mcasp_context_restore(struct ti_mcasp_softc *scCtrl, struct ti_mcasp_softc *scFifo,
                                mcasp_context_t *contextPtr, uint32_t sectFlag);

uint32_t
ti_mcasp_read_4(struct ti_mcasp_softc *sc, bus_size_t off);
void
ti_mcasp_write_4(struct ti_mcasp_softc *sc, bus_size_t off, uint32_t val);
int
mcasp_chdev_ioctl_calls(struct ti_mcasp_softc *sc, u_long cmd, caddr_t data, int fflag);


#ifdef __cplusplus
}
#endif
#endif /* __MCASP_H__ */
