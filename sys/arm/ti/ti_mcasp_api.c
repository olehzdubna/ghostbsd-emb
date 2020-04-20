/**
 *  \file   mcasp.c
 *
 *  \brief  McASP APIs.
 *
 *   This file contains the device abstraction layer APIs for McASP.
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

/* HW Macros and Peripheral Defines */
#include "ti_mcasp_hw.h"

/* Driver APIs */
#include "ti_mcasp_api.h"


uint32_t
ti_mcasp_read_4(struct ti_mcasp_softc *sc, bus_size_t off)
{
    return (bus_read_4(sc->mem_res[0], off));
}

void
ti_mcasp_write_4(struct ti_mcasp_softc *sc, bus_size_t off, uint32_t val)
{

    bus_write_4(sc->mem_res[0], off, val);
}

/*******************************************************************************
*                        API FUNCTION DEFINITIONS
*******************************************************************************/
/**
 * \brief   Resets the Transmit section of the McASP
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
void mcasp_tx_reset(struct ti_mcasp_softc *sc)
{
    ti_mcasp_write_4(sc, MCASP_XGBLCTL, 0x0);
}

/**
 * \brief   Resets the Receive section of the McASP
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
void mcasp_rx_reset(struct ti_mcasp_softc *sc)
{
    ti_mcasp_write_4(sc, MCASP_RGBLCTL, 0x0);
}

/**
 * \brief   Enables the Write FIFO for McASP. This shall be done before
 *          taking McASP out of Reset. First this API sets the FIFO parameters
 *          keeping the FIFO disabled. Then it enables the FIFO
 *
 * \param   baseAddr      Base Address of the McASP FIFO Registers.
 * \param   numTxSer      Number of Transmit Serializers to be used.
 * \param   minWdPerSer   The minimum number of words per serializer 
 *                        to be available in FIFO to issue a DMA event.
 *
 * \return  None.
 *
 **/
void mcasp_write_fifo_enable(struct ti_mcasp_softc *sc, uint32_t numTxSer,
                          uint32_t minWdPerSer)
{
    ti_mcasp_write_4(sc, MCASP_FIFO_WFIFOCTL, numTxSer |
                                            ((minWdPerSer * numTxSer)
                                             << AFIFO_WFIFOCTL_WNUMEVT_SHIFT) );

    /* The configuration is done. now set the enable bit */
    uint32_t val = ti_mcasp_read_4(sc, MCASP_FIFO_WFIFOCTL);
    ti_mcasp_write_4(sc, MCASP_FIFO_WFIFOCTL, val | AFIFO_WFIFOCTL_WENA);

    printf("+++ mcasp_write_fifo_enable %8.8x\n", ti_mcasp_read_4(sc, MCASP_FIFO_WFIFOCTL));
}

/**
 * \brief   Enables the Read FIFO for McASP. This shall be done before
 *          taking McASP out of Reset. First this API sets the FIFO parameters
 *          keeping the FIFO disabled. Then it enables the FIFO
 *
 * \param   baseAddr      Base Address of the McASP FIFO Registers.
 * \param   numRxSer      Number of Receive Serializers to be used.
 * \param   minWdPerSer   The minimum number of words per serializer 
 *                        to be available in FIFO to issue a DMA event.
 *
 * \return  None.
 *
 **/
void mcasp_read_fifo_enable(struct ti_mcasp_softc *sc, uint32_t numRxSer,
                         uint32_t minWdPerSer)
{
    ti_mcasp_write_4(sc, MCASP_FIFO_RFIFOCTL,  numRxSer | 
                                            ((minWdPerSer * numRxSer)
                                             << AFIFO_RFIFOCTL_RNUMEVT_SHIFT) );

    /* The configuration is done. now set the enable bit */
    uint32_t val = ti_mcasp_read_4(sc, MCASP_FIFO_RFIFOCTL);
    ti_mcasp_write_4(sc, MCASP_FIFO_RFIFOCTL, val | AFIFO_RFIFOCTL_RENA);

    printf("+++ mcasp_write_fifo_enable %8.8x\n", ti_mcasp_read_4(sc, MCASP_FIFO_RFIFOCTL));
}

/**
 * \brief   Sets the Format Mask for McASP Transmit section. The bits of
 *          the mask, which are set are transmitted out of McASP. The bits
 *          which are clear are padded according to the format settings.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   mask          Mask Value to be written, corresponding bits will
 *                        be transmitted out of McASP without any change
 *
 * \return  None.
 *
 **/
void mcasp_tx_fmt_mask_set(struct ti_mcasp_softc *sc, uint32_t mask)
{
    ti_mcasp_write_4(sc, MCASP_XMASK, mask);

    printf("+++ mcasp_tx_fmt_mask_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_XMASK));
}

/**
 * \brief   Sets the Format Mask for McASP Receive section. The bits of
 *          the mask, which are set are returned to the CPU/DMA. The bits
 *          which are clear are padded according to the format settings.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   mask          Mask Value to be written, corresponding bits will
 *                        be returned to the CPU/DMA
 *
 * \return  None.
 *
 **/
void mcasp_rx_fmt_mask_set(struct ti_mcasp_softc *sc, uint32_t mask)
{
    ti_mcasp_write_4(sc, MCASP_RMASK, mask);

    printf("+++ mcasp_rx_fmt_mask_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_RMASK));
}

/**
 * \brief   Sets the format for Transmit section of McASP with the format value
 *          input.  
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   formatVal     The format to be written to the transmit section
 *
 *    The formatVal shall be a logical OR combination of the parameters \n
 *     1. Transmit Sync Bit Dealy, which can take one of the values \n
 *            MCASP_TX_SYNC_DELAY_0BIT \n
 *            MCASP_TX_SYNC_DELAY_1BIT \n
 *            MCASP_TX_SYNC_DELAY_2BIT \n
 *
 *     2. Bit Stream Output, which can take one of the values \n
 *            MCASP_TX_BITSTREAM_LSB_FIRST \n
 *            MCASP_TX_BITSTREAM_MSB_FIRST \n
 *
 *     3. Padding Options, which can take one of the values \n
 *            MCASP_TX_PAD_WITH_0 \n
 *            MCASP_TX_PAD_WITH_1 \n
 *            MCASP_TX_PAD_WITH_PBIT(n) - pad the extra bytes with the 
 *                                        n-th pad bit \n
 *
 *     4. Transmit Slot Size, which can take one of the values \n
 *            MCASP_TX_SLOTSIZE_8BITS \n
 *            MCASP_TX_SLOTSIZE_12BITS \n
 *            MCASP_TX_SLOTSIZE_16BITS \n
 *            MCASP_TX_SLOTSIZE_20BITS \n
 *            MCASP_TX_SLOTSIZE_24BITS \n
 *            MCASP_TX_SLOTSIZE_28BITS \n
 *            MCASP_TX_SLOTSIZE_32BITS \n
 *
 *     5. Transmit Buffer Origin, which can take one of the values \n
 *            MCASP_TX_BUF_DMAPORT \n
 *            MCASP_TX_BUF_PERICONFIGPORT \n
 *
 *     6. Value for Transmit Rotate unit, which can take one of the values \n
 *            MCASP_TX_ROT_RIGHT_NONE \n
 *            MCASP_TX_ROT_RIGHT_4BITS \n
 *            MCASP_TX_ROT_RIGHT_8BITS \n
 *            MCASP_TX_ROT_RIGHT_12BITS \n
 *            MCASP_TX_ROT_RIGHT_16BITS \n
 *            MCASP_TX_ROT_RIGHT_20BITS \n
 *            MCASP_TX_ROT_RIGHT_24BITS \n
 *            MCASP_TX_ROT_RIGHT_28BITS \n
 *
 *     For example, the reset value is 
 *      (MCASP_TX_SYNC_DELAY_0BIT | MCASP_TX_BITSTREAM_LSB_FIRST | 
 *       MCASP_TX_PAD_WITH_0 | MCASP_TX_SLOTSIZE_8BITS | MCASP_TX_BUF_DMAPORT |
 *       MCASP_TX_ROT_RIGHT_NONE)
 *
 * \return  None.
 *
 **/
void mcasp_tx_fmt_set(struct ti_mcasp_softc *sc, uint32_t formatVal)
{
    ti_mcasp_write_4(sc, MCASP_XFMT, formatVal);

    printf("+++ mcasp_tx_fmt_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_XFMT));
} 

/**
 * \brief   Sets the format for Receive section of McASP with the format value
 *          input.  
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   formatVal     The format to be written to the Receive section
 *
 *    The formatVal shall be a logical OR combination of the parameters \n
 *     1. Receive Bit Dealy, which can take one of the values \n
 *            MCASP_RX_SYNC_DELAY_0BIT \n
 *            MCASP_RX_SYNC_DELAY_1BIT \n
 *            MCASP_RX_SYNC_DELAY_2BIT \n
 *
 *     2. Receive serial bit stream order, which can take one of the values \n
 *            MCASP_RX_BITSTREAM_LSB_FIRST \n
 *            MCASP_RX_BITSTREAM_MSB_FIRST \n
 *
 *     3. Padding Options, which can take one of the values \n
 *            MCASP_RX_PAD_WITH_0 \n
 *            MCASP_RX_PAD_WITH_1 \n
 *            MCASP_RX_PAD_WITH_RPBIT(n) - pad the extra bytes with the 
 *                                         n-th pad bit \n
 *
 *     4. Receive Slot Size, which can take one of the values \n
 *            MCASP_RX_SLOTSIZE_8BITS \n
 *            MCASP_RX_SLOTSIZE_12BITS \n
 *            MCASP_RX_SLOTSIZE_16BITS \n
 *            MCASP_RX_SLOTSIZE_20BITS \n
 *            MCASP_RX_SLOTSIZE_24BITS \n
 *            MCASP_RX_SLOTSIZE_28BITS \n
 *            MCASP_RX_SLOTSIZE_32BITS \n
 *
 *     5. Receive Buffer Origin, which can take one of the values \n
 *            MCASP_RX_BUF_DMAPORT \n
 *            MCASP_RX_BUF_PERICONFIGPORT \n
 *
 *     6. Value for Receive Rotate unit, which can take one of the values \n
 *            MCASP_RX_ROT_RIGHT_NONE \n
 *            MCASP_RX_ROT_RIGHT_4BITS \n
 *            MCASP_RX_ROT_RIGHT_8BITS \n
 *            MCASP_RX_ROT_RIGHT_12BITS \n
 *            MCASP_RX_ROT_RIGHT_16BITS \n
 *            MCASP_RX_ROT_RIGHT_20BITS \n
 *            MCASP_RX_ROT_RIGHT_24BITS \n
 *            MCASP_RX_ROT_RIGHT_28BITS \n
 *
 *     For example, the reset value is 
 *      (MCASP_RX_SYNC_DELAY_0BIT | MCASP_RX_BITSTREAM_LSB_FIRST | 
 *       MCASP_RX_PAD_WITH_0 | MCASP_RX_SLOTSIZE_8BITS | MCASP_RX_BUF_DMAPORT |
 *       MCASP_RX_ROT_RIGHT_NONE)
 *
 * \return  None.
 *
 **/
void mcasp_rx_fmt_set(struct ti_mcasp_softc *sc, uint32_t formatVal)
{
    ti_mcasp_write_4(sc, MCASP_RFMT, formatVal);

    printf("+++ mcasp_rx_fmt_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_RFMT));
} 

/**
 * \brief   Sets the I2S format in the Transmit Format unit. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   wordSize      The word size in bits.
 * \param   slotSize      Slot size in bits.
 * \param   txMode        The mode of Transmission.
 *             txMode can take one of the values \n
 *                  MCASP_TX_MODE_NON_DMA - transmission without using DMA \n
 *                  MCASP_TX_MODE_DMA - DMA is used for transmission 
 *
 * \return  None.
 * 
 *  Note : The Frame Sync shall be separately configured for I2S.
 *         It is assumed that the word size is a multiple of 8.
 *
 **/
void mcasp_tx_fmt_i2s_set(struct ti_mcasp_softc *sc, uint32_t wordSize,
                      uint32_t slotSize, uint32_t txMode)
{  
    /* Set the mask assuming integer format */
	uint32_t mask = (wordSize==32)?0xffffffff:(1<<wordSize)-1;
	mcasp_tx_fmt_mask_set(sc, mask);

    /* Set the transmit format unit for I2S */
	uint32_t rotate = (wordSize==32)?0:(wordSize >> 2);
	mcasp_tx_fmt_set(sc, (MCASP_TX_PAD_WITH_0 | MCASP_TX_BITSTREAM_MSB_FIRST
                             | MCASP_TX_SYNC_DELAY_1BIT | rotate
                             | ((slotSize/2 -1) << MCASP_XFMT_XSSZ_SHIFT)
                             | txMode));
}

/**
 * \brief   Sets the I2S format in the Receive Format unit. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   wordSize      The word size in bits.
 * \param   slotSize      Slot size in bits.
 * \param   rxMode        The mode of reception.
 *             rxMode can take one of the values \n
 *                  MCASP_RX_MODE_NON_DMA - reception without using DMA \n
 *                  MCASP_RX_MODE_DMA - DMA is used for reception
 *
 * \return  None.
 * 
 *  Note : The Frame Sync shall be separately configured for I2S.
 *         It is assumed that the word size is a multiple of 8.
 *
 **/
void mcasp_rx_fmt_i2s_set(struct ti_mcasp_softc *sc, uint32_t wordSize,
                      uint32_t slotSize, uint32_t rxMode)
{  
    /* Set the mask assuming integer format */
	uint32_t mask = (wordSize==32)?0xffffffff:(1<<wordSize)-1;
	mcasp_rx_fmt_mask_set(sc,  mask);

    /* Set the receive format unit for I2S */
	uint32_t rotate = (wordSize==32)?0:(wordSize >> 2);
	mcasp_rx_fmt_set(sc, (MCASP_RX_PAD_WITH_0 | MCASP_RX_BITSTREAM_MSB_FIRST
                             | MCASP_RX_SYNC_DELAY_1BIT | rotate
                             | ((slotSize/2 -1) << MCASP_RFMT_RSSZ_SHIFT)
                             | rxMode));
}

/**
 * \brief   Configures the Transmit Frame Sync signal.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   fsMode        The mode of Transmission
 * \param   fsWidth       The width of frame sync signal
 * \param   fsSetting     The signal settings for frame sync
 *
 *    fsMode is 0 for burst mode \n
 *    0x1 < fsMode < 0x21 for TDM mode with fsMode number of slots.
 *    Hence, fsMode = 2 for I2S mode \n
 *    fsMode is 384 for 384 slot DIT mode \n
 *
 *    fsWidth can take the values \n
 *         MCASP_TX_FS_WIDTH_BIT \n
 *         MCASP_TX_FS_WIDTH_WORD \n
 *
 *    fsSetting can take the values \n
 *         MCASP_TX_FS_INT_BEGIN_ON_RIS_EDGE - fs source is internal \n
 *         MCASP_TX_FS_INT_BEGIN_ON_FALL_EDGE - fs source is internal \n
 *         MCASP_TX_FS_EXT_BEGIN_ON_RIS_EDGE - fs source is external \n
 *         MCASP_TX_FS_EXT_BEGIN_ON_FALL_EDGE - fs source is external \n
 *   
 * \return  None.
 *
 **/
void mcasp_tx_frame_sync_cfg(struct ti_mcasp_softc *sc, uint32_t fsMode,
                         uint32_t fsWidth, uint32_t fsSetting)
{
    ti_mcasp_write_4(sc, MCASP_AFSXCTL, ((fsMode << MCASP_AFSXCTL_XMOD_SHIFT) 
                                       | fsWidth | fsSetting) );

    printf("+++ mcasp_tx_frame_sync_cfg %8.8x\n", ti_mcasp_read_4(sc, MCASP_AFSXCTL));
}

/**
 * \brief   Configures the Receive Frame Sync signal.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   fsMode        The mode of Reception
 * \param   fsWidth       The width of frame sync signal
 * \param   fsSetting     The signal settings for frame sync
 *
 *    fsMode is 0 for burst mode \n
 *    0x1 < fsMode < 0x21 for TDM mode with fsMode number of slots.
 *    Hence, fsMode = 2 for I2S mode \n
 *    fsMode is 384 for 384 slot DIR mode \n
 *
 *    fsWidth can take the values \n
 *         MCASP_RX_FS_WIDTH_BIT \n
 *         MCASP_RX_FS_WIDTH_WORD \n
 *
 *    fsSetting can take the values \n
 *         MCASP_RX_FS_INT_BEGIN_ON_RIS_EDGE - fs source is internal \n
 *         MCASP_RX_FS_INT_BEGIN_ON_FALL_EDGE - fs source is internal \n
 *         MCASP_RX_FS_EXT_BEGIN_ON_RIS_EDGE - fs source is external \n
 *         MCASP_RX_FS_EXT_BEGIN_ON_FALL_EDGE - fs source is external \n
 *   
 * \return  None.
 *
 **/
void mcasp_rx_frame_sync_cfg(struct ti_mcasp_softc *sc, uint32_t fsMode,
                         uint32_t fsWidth, uint32_t fsSetting)
{
    ti_mcasp_write_4(sc, MCASP_AFSRCTL, ((fsMode << MCASP_AFSRCTL_RMOD_SHIFT) 
                                       | fsWidth | fsSetting) );

    printf("+++ mcasp_rx_frame_sync_cfg %8.8x\n", ti_mcasp_read_4(sc, MCASP_AFSRCTL));
}

/**
 * \brief   Configures the clock for the Transmit Section for outputing bits.
 *          The source and divide values shall be pre-determined. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   clkSrc        The source of the clock 
 * \param   mixClkDiv     Value which divides the mixed clock
 * \param   auxClkDiv     Value which divides the internal high frequency clock
 *
 *    clkSrc can take the values \n
 *         MCASP_TX_CLK_INTERNAL \n
 *         MCASP_TX_CLK_EXTERNAL \n
 *         MCASP_TX_CLK_MIXED \n
 *
 * \return  None.
 *
 * Note: If external clock is selected, the divide values will not be used. \n
 *       If mixed clock is selected, the clock divide mixClkDiv only 
 *       will be used. \n
 *       If internal clock is selected, both the divide values will be used.
 *
 **/
void mcasp_tx_clk_cfg(struct ti_mcasp_softc *sc, uint32_t clkSrc,
                   uint32_t mixClkDiv, uint32_t auxClkDiv)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_ACLKXCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKXCTL, val & ~(MCASP_ACLKXCTL_CLKXM 
                                          | MCASP_ACLKXCTL_CLKXDIV) );

    val = ti_mcasp_read_4(sc, MCASP_AHCLKXCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKXCTL, val & ~(MCASP_AHCLKXCTL_HCLKXM
                                           | MCASP_AHCLKXCTL_HCLKXDIV) );
 
    /* Set the clock source to chose internal/external with clkdiv */
    val = ti_mcasp_read_4(sc, MCASP_ACLKXCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKXCTL, val | ((clkSrc & MCASP_ACLKXCTL_CLKXM)
                                          | mixClkDiv) );
    val = ti_mcasp_read_4(sc, MCASP_AHCLKXCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKXCTL, val | ((clkSrc & MCASP_AHCLKXCTL_HCLKXM)
                                          | auxClkDiv) );

    printf("+++ mcasp_tx_clk_cfg Clk: %8.8x HClk: %8.8x\n",  ti_mcasp_read_4(sc, MCASP_ACLKXCTL), ti_mcasp_read_4(sc, MCASP_AHCLKXCTL));
}

/**
 * \brief   Configures the clock for the Receive Section for receiving bits.
 *          The source and divide values shall be pre-determined. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   clkSrc        The source of the clock 
 * \param   mixClkDiv     Value which divides the mixed clock
 * \param   auxClkDiv     Value which divides the internal high frequency clock
 *
 *    clkSrc can take the values \n
 *         MCASP_RX_CLK_INTERNAL \n
 *         MCASP_RX_CLK_EXTERNAL \n
 *         MCASP_RX_CLK_MIXED \n
 *
 * \return  None.
 *
 * Note: If external clock is selected, the divide values will not be used. \n
 *       If mixed clock is selected, the clock divide mixClkDiv only 
 *       will be used. \n
 *       If internal clock is selected, both the divide values will be used.
 *
 **/
void mcasp_rx_clk_cfg(struct ti_mcasp_softc *sc, uint32_t clkSrc,
                   uint32_t mixClkDiv, uint32_t auxClkDiv)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_ACLKRCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKRCTL, val & ~(MCASP_ACLKRCTL_CLKRM 
                                          | MCASP_ACLKRCTL_CLKRDIV) );
    val = ti_mcasp_read_4(sc, MCASP_AHCLKRCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKRCTL, val & ~(MCASP_AHCLKRCTL_HCLKRM
                                           | MCASP_AHCLKRCTL_HCLKRDIV) );
 
    /* Set the clock source to chose internal/external with clkdiv */
    val = ti_mcasp_read_4(sc, MCASP_ACLKRCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKRCTL, val | ((clkSrc & MCASP_ACLKRCTL_CLKRM)
                                          | mixClkDiv) );
    val = ti_mcasp_read_4(sc, MCASP_AHCLKRCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKRCTL, val | ((clkSrc & MCASP_AHCLKRCTL_HCLKRM)
                                          | auxClkDiv) );
    printf("+++ mcasp_rx_clk_cfg Clk: %8.8x HClk: %8.8x\n",  ti_mcasp_read_4(sc, MCASP_ACLKRCTL), ti_mcasp_read_4(sc, MCASP_AHCLKRCTL));
}

/**
 * \brief   Sets the polarity of the Transmitter Clock. If an external receiver
 *          samples data on the falling edge of the serial clock,  the 
 *          transmitter  must shift data out on the rising edge of the 
 *          serial clock and vice versa.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   polarity      Polarity to be chosen
 *
 *    polarity can take the values \n
 *         MCASP_TX_CLK_POL_RIS_EDGE \n
 *         MCASP_TX_CLK_POL_FALL_EDGE \n
 *
 * \return  None.
 *
 **/
void mcasp_tx_clk_polarity_set(struct ti_mcasp_softc *sc, uint32_t polarity)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_ACLKXCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKXCTL, val & ~MCASP_ACLKXCTL_CLKXP);
    val = ti_mcasp_read_4(sc, MCASP_ACLKXCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKXCTL, val | polarity);
}

/**
 * \brief   Sets the polarity of the Rceiver Clock. If an external receiver
 *          shifts data on the falling edge of the serial clock,  the 
 *          receiver  must sample the data on the rising edge of the 
 *          serial clock and vice versa.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   polarity      Polarity to be chosen
 *
 *    polarity can take the values \n
 *         MCASP_RX_CLK_POL_RIS_EDGE \n
 *         MCASP_RX_CLK_POL_FALL_EDGE \n
 *
 * \return  None.
 *
 **/
void mcasp_rx_clk_polarity_set(struct ti_mcasp_softc *sc, uint32_t polarity)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_ACLKRCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKRCTL, val & ~MCASP_ACLKRCTL_CLKRP);
    val = ti_mcasp_read_4(sc, MCASP_ACLKRCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKRCTL, val | polarity);
}

/**
 * \brief   Sets the polarity/inversion of the  High Frequency clock. This is
 *          valid if the transmitter clock source is internal or mixed.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   polarity      polarity to be chosen
 *
 *    polarity can take the values \n
 *         MCASP_TX_HI_FREQ_CLK_INVERT \n
 *         MCASP_TX_HI_FREQ_CLK_NO_INVERT \n
 *     
 * \return  None.
 *
 **/
void mcasp_tx_hf_clk_polarity_set(struct ti_mcasp_softc *sc, uint32_t polarity)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_AHCLKXCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKXCTL, val & ~MCASP_AHCLKXCTL_HCLKXP);
    val = ti_mcasp_read_4(sc, MCASP_AHCLKXCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKXCTL, val | polarity);
}

/**
 * \brief   Sets the polarity/inversion of the  High Frequency clock. This is
 *          valid if the receiver clock source is internal or mixed.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   polarity      polarity to be chosen
 *
 *    polarity can take the values \n
 *         MCASP_RX_HI_FREQ_CLK_INVERT \n
 *         MCASP_RX_HI_FREQ_CLK_NO_INVERT \n
 *     
 * \return  None.
 *
 **/
void mcasp_rx_hf_clk_polarity_set(struct ti_mcasp_softc *sc, uint32_t polarity)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_AHCLKRCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKRCTL, val & ~MCASP_AHCLKRCTL_HCLKRP);
    val = ti_mcasp_read_4(sc, MCASP_AHCLKRCTL);
    ti_mcasp_write_4(sc, MCASP_AHCLKRCTL, val | polarity);
}

/**
 * \brief   Synchronizes the transmitter and receiver Clocks 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
void mcasp_tx_rx_clk_sync_enable(struct ti_mcasp_softc *sc)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_ACLKXCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKXCTL, val & ~MCASP_ACLKXCTL_ASYNC);
}

/**
 * \brief   Disable synchronization of the transmitter and receiver Clocks
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
void mcasp_tx_rx_clk_sync_disable(struct ti_mcasp_softc *sc)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_ACLKXCTL);
    ti_mcasp_write_4(sc, MCASP_ACLKXCTL, val | MCASP_ACLKXCTL_ASYNC);
}

/**
 * \brief   Sets a serializer as transmitter
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   serNum        Serializer which is to be used as Transmitter.
 *
 * \return  None.
 *
 **/
void mcasp_serializer_tx_set(struct ti_mcasp_softc *sc, uint32_t serNum)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_SRCTL(serNum));
    ti_mcasp_write_4(sc, MCASP_SRCTL(serNum), val & ~MCASP_SRCTL0_SRMOD);
    val = ti_mcasp_read_4(sc, MCASP_SRCTL(serNum));
    ti_mcasp_write_4(sc, MCASP_SRCTL(serNum), val | MCASP_SRCTL_SRMOD_TX);

    printf("+++ mcasp_serializer_tx_set #%d, %8.8x\n", serNum, ti_mcasp_read_4(sc, MCASP_SRCTL(serNum)));
}

/**
 * \brief   Sets a serializer as receiver
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   serNum        Serializer which is to be used as Receiver.
 *
 * \return  None.
 *
 **/
void mcasp_serializer_rx_set(struct ti_mcasp_softc *sc, uint32_t serNum)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_SRCTL(serNum));
    ti_mcasp_write_4(sc, MCASP_SRCTL(serNum), val & ~MCASP_SRCTL0_SRMOD);
    val = ti_mcasp_read_4(sc, MCASP_SRCTL(serNum));
    ti_mcasp_write_4(sc, MCASP_SRCTL(serNum), val | MCASP_SRCTL_SRMOD_RX);

    printf("+++ mcasp_serializer_rx_set #%d, %8.8x\n", serNum, ti_mcasp_read_4(sc, MCASP_SRCTL(serNum)));
}


/**
 * \brief   Gets a serializer status
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   serNum        Serializer which is to be used.
 *
 * \return  register value.
 *
 **/
uint32_t mcasp_serializer_get(struct ti_mcasp_softc *sc, uint32_t serNum)
{
    return ti_mcasp_read_4(sc, MCASP_SRCTL(serNum));
}

/**
 * \brief   Inactivates a serializer.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   serNum        Serializer which is to be inactivated.
 *
 * \return  None.
 *
 **/
void mcasp_serializer_inactivate(struct ti_mcasp_softc *sc, uint32_t serNum)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_SRCTL(serNum));
    ti_mcasp_write_4(sc, MCASP_SRCTL(serNum), val & ~MCASP_SRCTL0_SRMOD);
}

/**
 * \brief   Sets McASP pins as GPIO for general purpose use
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   pinMask       Mask of the pins to be configured as GPIO.
 *            pinMask can be a combination of the below values \n
 *                MCASP_PIN_AFSR  \n
 *                MCASP_PIN_AHCLKR \n                                       
 *                MCASP_PIN_ACLKR \n                                       
 *                MCASP_PIN_AFSX \n                                       
 *                MCASP_PIN_AHCLKX \n                                       
 *                MCASP_PIN_ACLKX \n                                        
 *                MCASP_PIN_AMUTE \n                                       
 *                MCASP_PIN_AXR(n) - For serializer 'n'           
 *    
 * \return  None.
 *
 **/
void mcasp_pin_gpio_set(struct ti_mcasp_softc *sc, uint32_t pinMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_PFUNC);
    ti_mcasp_write_4(sc, MCASP_PFUNC, val | pinMask);
}

/**
 * \brief   Sets McASP pins to use for McASP functionality
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   pinMask       Mask of the pins to be configured for McASP.
 *            pinMask can be a combination of the below values \n
 *                MCASP_PIN_AFSR  \n
 *                MCASP_PIN_AHCLKR \n                                       
 *                MCASP_PIN_ACLKR \n                                       
 *                MCASP_PIN_AFSX \n                                       
 *                MCASP_PIN_AHCLKX \n                                       
 *                MCASP_PIN_ACLKX \n                                        
 *                MCASP_PIN_AMUTE \n                                       
 *                MCASP_PIN_AXR(n) - For serializer 'n'           
 *
 * \return  None.
 *
 **/
void mcasp_pin_mcasp_set(struct ti_mcasp_softc *sc, uint32_t pinMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_PFUNC);
    ti_mcasp_write_4(sc, MCASP_PFUNC, val & ~pinMask);

    printf("+++ mcasp_pin_mcasp_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_PFUNC));
}

void mcasp_dout_set(struct ti_mcasp_softc *sc, uint32_t data)
{
    ti_mcasp_write_4(sc, MCASP_PDOUT, data);
}

uint32_t mcasp_din_get(struct ti_mcasp_softc *sc)
{
    return ti_mcasp_read_4(sc, MCASP_PDIN);
}

/**
 * \brief   Configures a McASP pin as an output pin.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   pinMask       Mask of the pins to be configured as output
 *            pinMask can be a combination of the below values \n
 *                MCASP_PIN_AFSR  \n
 *                MCASP_PIN_AHCLKR \n                                       
 *                MCASP_PIN_ACLKR \n                                       
 *                MCASP_PIN_AFSX \n                                       
 *                MCASP_PIN_AHCLKX \n                                       
 *                MCASP_PIN_ACLKX \n                                        
 *                MCASP_PIN_AMUTE \n                                       
 *                MCASP_PIN_AXR(n) - For serializer 'n'           
 *
 * \return  None.
 *
 **/
void mcasp_pin_dir_output_set(struct ti_mcasp_softc *sc, uint32_t pinMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_PDIR);
    ti_mcasp_write_4(sc, MCASP_PDIR, val | pinMask);

    printf("+++ mcasp_pin_dir_output_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_PDIR));
}

/**
 * \brief   Configures a McASP pin as an input pin.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   pinMask       Mask of the pins to be configured as input
 *            pinMask can be a combination of the below values \n
 *                MCASP_PIN_AFSR  \n
 *                MCASP_PIN_AHCLKR \n                                       
 *                MCASP_PIN_ACLKR \n                                       
 *                MCASP_PIN_AFSX \n                                       
 *                MCASP_PIN_AHCLKX \n                                       
 *                MCASP_PIN_ACLKX \n                                        
 *                MCASP_PIN_AMUTE \n                                       
 *                MCASP_PIN_AXR(n) - For serializer 'n'           
 *
 * \return  None.
 *
 **/
void mcasp_pin_dir_input_set(struct ti_mcasp_softc *sc, uint32_t pinMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_PDIR);
    ti_mcasp_write_4(sc, MCASP_PDIR, val & ~pinMask);

    printf("+++ mcasp_pin_dir_input_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_PDIR));
}

/**
 * \brief   Sets the active time slot for the Transmitter section. The
 *          bit which is set in the mask will indicate that data will be
 *          transmitted during that time slot. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   slotMask      The time slot mask. 
 *
 * \return  None.
 *
 **/
void mcasp_tx_time_slot_set(struct ti_mcasp_softc *sc, uint32_t slotMask)
{
     ti_mcasp_write_4(sc, MCASP_XTDM, slotMask);

     printf("+++ mcasp_tx_time_slot_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_XTDM));
}

/**
 * \brief   Sets the active time slot for the receiver section. The
 *          bit which is set in the mask will indicate that data will be
 *          shifted in during that time slot. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   slotMask      The time slot mask. 
 *
 * \return  None.
 *
 **/
void mcasp_rx_time_slot_set(struct ti_mcasp_softc *sc, uint32_t slotMask)
{
     ti_mcasp_write_4(sc, MCASP_RTDM, slotMask);

     printf("+++ mcasp_rx_time_slot_set %8.8x\n", ti_mcasp_read_4(sc, MCASP_RTDM));
}


/**
 * \brief   Enables Audio Mute on detecting the specified errors.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   errFlags      Errors for which AMUTE to be enabled
 * \param   pinState      State of AMUTE pin to be driven if enabled.
 *            errFlags can be a combination of the following macros \n
 *                MCASP_AMUTE_TX_DMAERROR \n
 *                MCASP_AMUTE_TX_CLKFAIL \n
 *                MCASP_AMUTE_TX_SYNCERROR \n
 *                MCASP_AMUTE_TX_UNDERRUN \n
 *                MCASP_AMUTE_RX_DMAERROR \n
 *                MCASP_AMUTE_RX_CLKFAIL \n
 *                MCASP_AMUTE_RX_SYNCERROR \n
 *                MCASP_AMUTE_RX_OVERRUN \n
 *            pinState can take one of the values \n
 *                MCASP_AMUTE_PIN_HIGH \n
 *                MCASP_AMUTE_PIN_LOW 
 *
 * \return  None.
 *
 **/
void mcasp_a_mute_enable(struct ti_mcasp_softc *sc, uint32_t errFlags,
                        uint32_t pinState)
{
    ti_mcasp_write_4(sc, MCASP_AMUTE, errFlags | (MCASP_AMUTE_MUTEN & pinState));
}

/**
 * \brief   Disables Audio Mute on detecting error
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
void mcasp_a_mute_disable(struct ti_mcasp_softc *sc)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_AMUTE);
    ti_mcasp_write_4(sc, MCASP_AMUTE, val & ~MCASP_AMUTE_MUTEN);
}

/**
 * \brief   Configures the transmitter clock check circuit.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   clkDiv        Divide value for the McASP system clock. The McASP
 *                        system clock is divided with this value before passing
 *                        to the clock check circuit
 * \param   boundMin      Transmit clock minimum boundary
 * \param   boundMax      Transmit clock maximum boundary
 *            clkDiv can take one of the following values \n
 *               MCASP_TX_CLKCHCK_DIV1 \n  
 *               MCASP_TX_CLKCHCK_DIV2 \n  
 *               MCASP_TX_CLKCHCK_DIV4 \n  
 *               MCASP_TX_CLKCHCK_DIV8 \n  
 *               MCASP_TX_CLKCHCK_DIV16 \n  
 *               MCASP_TX_CLKCHCK_DIV32 \n  
 *               MCASP_TX_CLKCHCK_DIV64 \n  
 *               MCASP_TX_CLKCHCK_DIV128 \n  
 *               MCASP_TX_CLKCHCK_DIV256  
 *
 * \return  None.
 *
 **/
void mcasp_tx_clk_check_config(struct ti_mcasp_softc *sc, uint32_t clkDiv,
                           unsigned char boundMin, unsigned char boundMax)
{
    ti_mcasp_write_4(sc, MCASP_XCLKCHK, clkDiv
                                      | boundMin << MCASP_XCLKCHK_XMIN_SHIFT 
                                      | boundMax << MCASP_XCLKCHK_XMAX_SHIFT);
}

/**
 * \brief   Configures the receiver clock check circuit.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   clkDiv        Divide value for the McASP system clock. The McASP
 *                        system clock is divided with this value prior to passing
 *                        to the clock check circuit
 * \param   boundMin      Receive clock minimum boundary
 * \param   boundMax      Receive clock maximum boundary
 *            clkDiv can take one of the following values \n
 *               MCASP_RX_CLKCHCK_DIV1 \n  
 *               MCASP_RX_CLKCHCK_DIV2 \n  
 *               MCASP_RX_CLKCHCK_DIV4 \n  
 *               MCASP_RX_CLKCHCK_DIV8 \n  
 *               MCASP_RX_CLKCHCK_DIV16 \n  
 *               MCASP_RX_CLKCHCK_DIV32 \n  
 *               MCASP_RX_CLKCHCK_DIV64 \n  
 *               MCASP_RX_CLKCHCK_DIV128 \n  
 *               MCASP_RX_CLKCHCK_DIV256  
 *
 * \return  None.
 *
 **/
void mcasp_rx_clk_check_config(struct ti_mcasp_softc *sc, uint32_t clkDiv,
                           unsigned char boundMin, unsigned char boundMax)
{
    ti_mcasp_write_4(sc, MCASP_RCLKCHK, clkDiv
                                      | boundMin << MCASP_RCLKCHK_RMIN_SHIFT 
                                      | boundMax << MCASP_RCLKCHK_RMAX_SHIFT);
}

/**
 * \brief   Activates the AMUTEIN pin and drives the AMUTE active.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   polarity      polarity of AMUTEIN which shall drive the 
 *                        AMUTE active. 
 *            polarity can take one of the following values. \n
 *                MCASP_AMUTEIN_POL_HIGH - high on AMUTEIN pin  \n
 *                MCASP_AMUTEIN_POL_LOW - low on AMUTEIN pin 
 *             
 * \return  None.
 *
 **/
void mcasp_a_mute_in_activate(struct ti_mcasp_softc *sc, uint32_t polarity)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_AMUTE);
    ti_mcasp_write_4(sc, MCASP_AMUTE, val &= ~MCASP_AMUTE_INPOL);

    val = ti_mcasp_read_4(sc, MCASP_AMUTE);
    ti_mcasp_write_4(sc, MCASP_AMUTE, val | (MCASP_AMUTE_INSTAT | MCASP_AMUTE_INEN
                                      |(polarity & MCASP_AMUTE_INPOL)) );
}

/**
 * \brief   Enables the specified Transmitter interrupts.
 *
 * \param   baseAddr     Base Address of the McASP Module Registers.
 * \param   intMask      The transmitter interrupts to be enabled 
 *            intMask can be a logical OR combination of the values \n
 *                 MCASP_TX_STARTOFFRAME \n
 *                 MCASP_TX_DATAREADY \n
 *                 MCASP_TX_LASTSLOT \n
 *                 MCASP_TX_DMAERROR \n
 *                 MCASP_TX_CLKFAIL \n
 *                 MCASP_TX_SYNCERROR \n
 *                 MCASP_TX_UNDERRUN
 *
 * \return  None.
 *
 **/
void mcasp_tx_int_enable(struct ti_mcasp_softc *sc, uint32_t intMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_XINTCTL);
    ti_mcasp_write_4(sc, MCASP_XINTCTL, val | intMask);
}

/**
 * \brief   Enables the specified Receiver interrupts.
 *
 * \param   baseAddr     Base Address of the McASP Module Registers.
 * \param   intMask      The receive interrupts to be enabled 
 *            intMask can be a logical OR combination of the values \n
 *                 MCASP_RX_STARTOFFRAME \n
 *                 MCASP_RX_DATAREADY \n
 *                 MCASP_RX_LASTSLOT \n
 *                 MCASP_RX_DMAERROR \n
 *                 MCASP_RX_CLKFAIL \n
 *                 MCASP_RX_SYNCERROR \n
 *                 MCASP_RX_OVERRUN
 *
 * \return  None.
 *
 **/
void mcasp_rx_int_enable(struct ti_mcasp_softc *sc, uint32_t intMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_RINTCTL);
    ti_mcasp_write_4(sc, MCASP_RINTCTL, val | intMask);
}

/**
 * \brief   Disables the specified Transmitter interrupts.
 *
 * \param   baseAddr     Base Address of the McASP Module Registers.
 * \param   intMask      The transmitter interrupts to be disabled 
 *            intMask can be a logical OR combination of the values \n
 *                 MCASP_TX_STARTOFFRAME \n
 *                 MCASP_TX_DATAREADY \n
 *                 MCASP_TX_LASTSLOT \n
 *                 MCASP_TX_DMAERROR \n
 *                 MCASP_TX_CLKFAIL \n
 *                 MCASP_TX_SYNCERROR \n
 *                 MCASP_TX_UNDERRUN
 *
 * \return  None.
 *
 **/
void mcasp_tx_int_disable(struct ti_mcasp_softc *sc, uint32_t intMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_XINTCTL);
    ti_mcasp_write_4(sc, MCASP_XINTCTL, val & ~intMask);
}

/**
 * \brief   Disables the specified Receiver interrupts.
 *
 * \param   baseAddr     Base Address of the McASP Module Registers.
 * \param   intMask      The receive interrupts to be disabled 
 *            intMask can be a logical OR combination of the values \n
 *                 MCASP_RX_STARTOFFRAME \n
 *                 MCASP_RX_DATAREADY \n
 *                 MCASP_RX_LASTSLOT \n
 *                 MCASP_RX_DMAERROR \n
 *                 MCASP_RX_CLKFAIL \n
 *                 MCASP_RX_SYNCERROR \n
 *                 MCASP_RX_OVERRUN
 *
 * \return  None.
 *
 **/
void mcasp_rx_int_disable(struct ti_mcasp_softc *sc, uint32_t intMask)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_RINTCTL);
    ti_mcasp_write_4(sc, MCASP_RINTCTL, val & ~intMask);
}

/**
 * \brief   Activates the Transmit Serializers
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
int mcasp_tx_ser_activate(struct ti_mcasp_softc *sc)
{
    int i=0;
    int tries = 1000000;

    ti_mcasp_write_4(sc, MCASP_XSTAT, 0x1FF);

    /* Release transmit serializers from reset*/
    uint32_t val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_XSRCLR);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_XSRCLR) 
          != MCASP_GBLCTL_XSRCLR) && i>0; --i);

    printf("+++ mcasp_tx_ser_activate %8.8x\n", ti_mcasp_read_4(sc, MCASP_GBLCTL));

   return i;
}

/**
 * \brief   Activates the Receive Serializers
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
int mcasp_rx_ser_activate(struct ti_mcasp_softc *sc)
{
    int i=0;
    int tries = 1000000;

    ti_mcasp_write_4(sc, MCASP_RSTAT, 0x1FF);

    /* Release transmit serializers from reset*/
    uint32_t val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_RSRCLR);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_RSRCLR)
          != MCASP_GBLCTL_RSRCLR) && i>0; --i);

    printf("+++ mcasp_rx_ser_activate %8.8x\n", ti_mcasp_read_4(sc, MCASP_GBLCTL));

    return i;
}

/**
 * \brief   Starts the McASP Transmitter Clock. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   clkSrc        The transmitter clock source.
 *
 *    clkSrc can take the values \n
 *         MCASP_TX_CLK_INTERNAL \n
 *         MCASP_TX_CLK_EXTERNAL \n
 *         MCASP_TX_CLK_MIXED \n
 *
 * \return  None.
 *
 **/
int mcasp_tx_clk_start(struct ti_mcasp_softc *sc, uint32_t clkSrc)
{
    int i=0;
    int tries = 1000000;
    
    /* Release the high frequency clock from reset*/
    uint32_t val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_XHCLKRST);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_XHCLKRST)
          != MCASP_GBLCTL_XHCLKRST) && i>0; --i);

    if(!i)
      return 0;

    if(clkSrc != MCASP_TX_CLK_EXTERNAL)
    {
       /* Release the clock from reset*/
        val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
	ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_XCLKRST);
	for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_XCLKRST)
            != MCASP_GBLCTL_XCLKRST) && i>0; --i);
    }

    return i;
}

/**
 * \brief   Starts the McASP Receiver Clock. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   clkSrc        The receiver clock source.
 *
 *    clkSrc can take the values \n
 *         MCASP_RX_CLK_INTERNAL \n
 *         MCASP_RX_CLK_EXTERNAL \n
 *         MCASP_RX_CLK_MIXED \n
 *
 * \return  None.
 *
 **/
int mcasp_rx_clk_start(struct ti_mcasp_softc *sc, uint32_t clkSrc)
{
    int i=0;
    int tries = 1000000;
    
    /* Release the high frequency clock from reset*/
    uint32_t val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_RHCLKRST);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_RHCLKRST)
          != MCASP_GBLCTL_RHCLKRST) && i>0; --i);

    if(!i)
      return 0;

    if(clkSrc != MCASP_RX_CLK_EXTERNAL)
    {
        /* Release the clock from reset*/
        val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
	ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_RCLKRST);
	for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_RCLKRST)
            != MCASP_GBLCTL_RCLKRST) && i>0; --i);
    }

    return i;
}

/**
 * \brief   Enables the McASP Transmission. When this API is called, 
 *          The transmit state machine and the frame sync generators are 
 *          released from reset. The McASP starts transmission on the
 *          first frame sync after this.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
int mcasp_tx_enable(struct ti_mcasp_softc *sc)
{
    int i=0;
    int tries = 1000000;

    /* Release the Transmit State machine from reset*/
    uint32_t val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_XSMRST);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_XSMRST)
          != MCASP_GBLCTL_XSMRST) && i>0; --i);

    if(!i)
      return 0;

    /* Release the frame sync generator from reset*/
    val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_XFRST);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_XFRST)
          != MCASP_GBLCTL_XFRST) && i>0; --i);

    return i;
}

/**
 * \brief   Enables the McASP Reception. When this API is called, 
 *          The receive state machine and the frame sync generators are 
 *          released from reset. The McASP starts reception on the
 *          first frame sync after this.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
int mcasp_rx_enable(struct ti_mcasp_softc *sc)
{
    int i=0;
    int tries = 1000000;

    /* Release the Receive State machine from reset*/
    uint32_t val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_RSMRST);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_RSMRST)
          != MCASP_GBLCTL_RSMRST) && i>0; --i);

    if(!i)
      return 0;

    /* Release the frame sync generator from reset*/
    val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_GBLCTL, val | MCASP_GBLCTL_RFRST);
    for(i=tries;((ti_mcasp_read_4(sc, MCASP_GBLCTL) & MCASP_GBLCTL_RFRST)
          != MCASP_GBLCTL_RFRST) && i>0; --i);

    return i;
}

/**
 * \brief   Reads a receive buffer through peripheral configuration port. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   serNum        Serializer number
 *
 * \return  Buffer contents
 *
 **/
uint32_t mcasp_rx_buf_read(struct ti_mcasp_softc *sc, uint32_t serNum)
{
    return ti_mcasp_read_4(sc, MCASP_RBUF(serNum));
}

/**
 * \brief   Writes to a transmit buffer through peripheral configuration port. 
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   serNum        Serializer number
 * \param   data          Data to be written
 *
 * \return  None.
 *
 **/
void mcasp_tx_buf_write(struct ti_mcasp_softc *sc, uint32_t serNum,
                     uint32_t data)
{
    ti_mcasp_write_4(sc, MCASP_XBUF(serNum), data);
}

/**
 * \brief   Enables the DIT mode of transmission.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   vBit          Valid bit values for the even and odd
 *                        time slots.
 *             vBit can take one of the following values \n
 *                MCASP_DIT_VBIT_ODD_0_EVEN_1 - V bit is 0 for odd slots
 *                                              and 1 for even slots  \n    
 *                MCASP_DIT_VBIT_ODD_1_EVEN_0 - V bit is 1 for odd slots
 *                                              and 0 for even slots  \n    
 *                MCASP_DIT_VBIT_BOTHSLOTS_0 - V bit is 0 for both slots \n
 *                MCASP_DIT_VBIT_BOTHSLOTS_1 - V bit is 1 for both slots
 * \return  None.
 *
 **/
void mcasp_dit_enable(struct ti_mcasp_softc *sc, uint32_t vBit)
{
    ti_mcasp_write_4(sc, MCASP_DITCTL, vBit | MCASP_DITCTL_DITEN);
}

/**
 * \brief   Disables the DIT mode of transmission.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  None.
 *
 **/
void mcasp_dit_disable(struct ti_mcasp_softc *sc)
{
    uint32_t val = ti_mcasp_read_4(sc, MCASP_GBLCTL);
    ti_mcasp_write_4(sc, MCASP_DITCTL, val & ~MCASP_DITCTL_DITEN);
}

/**
 * \brief   Writes the DIT Channel Status bits
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   chStatBits    Channel status bits to be written
 * \param   channel       Channel of which status bits to be written
 * \param   data          Data to be written
 *            chStatBits can take one of the following values \n
 *                MCASP_DIT_CHSTAT_BITS_0_31 \n 
 *                MCASP_DIT_CHSTAT_BITS_32_63 \n 
 *                MCASP_DIT_CHSTAT_BITS_64_95 \n 
 *                MCASP_DIT_CHSTAT_BITS_96_127 \n 
 *                MCASP_DIT_CHSTAT_BITS_128_159 \n 
 *                MCASP_DIT_CHSTAT_BITS_160_191 \n
 *
 *           channel can take the following values \n
 *                MCASP_DIT_CHANNEL_LEFT - for left channel only \n
 *                MCASP_DIT_CHANNEL_RIGHT - for right channel only \n
 *                MCASP_DIT_CHANNEL_BOTH - for both channels 
 *
 * \return  None.
 *
 **/
void mcasp_dit_chan_stat_write(struct ti_mcasp_softc *sc, uint32_t chStatBits,
                           uint32_t channel,  uint32_t data)
{
    /* Write for the left channel */
    if(MCASP_DIT_CHANNEL_LEFT & channel)
    {
        ti_mcasp_write_4(sc, MCASP_DITCSRA(chStatBits), data);
    }

    /* Write for the right channel */
    if(MCASP_DIT_CHANNEL_RIGHT & channel)
    {
        ti_mcasp_write_4(sc, MCASP_DITCSRB(chStatBits), data);
    }
}

/**
 * \brief   Writes the DIT Channel User Data bits
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   chStatBits    Channel user data bits to be written
 * \param   channel       Channel of which user data bits to be written
 * \param   data          Data to be written
 *            chStatBits can take one of the following values \n
 *                MCASP_DIT_USRDATA_BITS_0_31 \n 
 *                MCASP_DIT_USRDATA_BITS_32_63 \n 
 *                MCASP_DIT_USRDATA_BITS_64_95 \n 
 *                MCASP_DIT_USRDATA_BITS_96_127 \n 
 *                MCASP_DIT_USRDATA_BITS_128_159 \n 
 *                MCASP_DIT_USRDATA_BITS_160_191 \n
 *
 *           channel can take the following values \n
 *                MCASP_DIT_CHANNEL_LEFT - for left channel only \n
 *                MCASP_DIT_CHANNEL_RIGHT - for right channel only \n
 *                MCASP_DIT_CHANNEL_BOTH - for both channels 
 *
 * \return  None.
 *
 **/
void mcasp_dit_chan_usr_data_write(struct ti_mcasp_softc *sc, uint32_t chUsrDataBits,
                              uint32_t channel,  uint32_t data)
{
    /* Write for the left channel */
    if(MCASP_DIT_CHANNEL_LEFT & channel)
    {
        ti_mcasp_write_4(sc, MCASP_DITUDRA(chUsrDataBits), data);
    }

    /* Write for the right channel */
    if(MCASP_DIT_CHANNEL_RIGHT & channel)
    {
        ti_mcasp_write_4(sc, MCASP_DITUDRB(chUsrDataBits), data);
    }
}

/**
 * \brief   Reads the DIT Channel Status bits
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   chStatBits    Channel status bits to be read
 * \param   channel       Channel of which status bits to be read
 *            chStatBits can take one of the following values \n
 *                MCASP_DIT_CHSTAT_BITS_0_31 \n 
 *                MCASP_DIT_CHSTAT_BITS_32_63 \n 
 *                MCASP_DIT_CHSTAT_BITS_64_95 \n 
 *                MCASP_DIT_CHSTAT_BITS_96_127 \n 
 *                MCASP_DIT_CHSTAT_BITS_128_159 \n 
 *                MCASP_DIT_CHSTAT_BITS_160_191 \n
 *
 *           channel can take the following values \n
 *                MCASP_DIT_CHANNEL_LEFT  \n
 *                MCASP_DIT_CHANNEL_RIGHT 
 *
 * \return  DIT channel status.
 *
 **/
uint32_t mcasp_dit_chan_stat_read(struct ti_mcasp_softc *sc,
                                  uint32_t chStatBits, 
                                  uint32_t channel)
{
    uint32_t retVal = 0;

    /* Read from the left channel */
    if(MCASP_DIT_CHANNEL_LEFT == channel)
    {
        retVal = ti_mcasp_read_4(sc, MCASP_DITCSRA(chStatBits));
    }

    /* Read from the right channel */
    if(MCASP_DIT_CHANNEL_RIGHT == channel)
    {
        retVal = ti_mcasp_read_4(sc, MCASP_DITCSRB(chStatBits));
    }

    return retVal;
}

/**
 * \brief   Reads the DIT Channel User Data bits
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 * \param   chStatBits    Channel user data bits to be read
 * \param   channel       Channel of which user data bits to be read
 *            chStatBits can take one of the following values \n
 *                MCASP_DIT_USRDATA_BITS_0_31 \n 
 *                MCASP_DIT_USRDATA_BITS_32_63 \n 
 *                MCASP_DIT_USRDATA_BITS_64_95 \n 
 *                MCASP_DIT_USRDATA_BITS_96_127 \n 
 *                MCASP_DIT_USRDATA_BITS_128_159 \n 
 *                MCASP_DIT_USRDATA_BITS_160_191 \n
 *
 *           channel can take the following values \n
 *                MCASP_DIT_CHANNEL_LEFT \n
 *                MCASP_DIT_CHANNEL_RIGHT
 *
 * \return  DIT channel user data.
 *
 **/
uint32_t mcasp_dit_chan_usr_data_read(struct ti_mcasp_softc *sc,
                                     uint32_t chUsrDataBits, 
                                     uint32_t channel)
{
    uint32_t retVal = 0;

    /* Read from the left channel */
    if(MCASP_DIT_CHANNEL_LEFT == channel)
    {
        retVal = ti_mcasp_read_4(sc, MCASP_DITUDRA(chUsrDataBits));
    }

    /* Read from the right channel */
    if(MCASP_DIT_CHANNEL_RIGHT == channel)
    {
        retVal = ti_mcasp_read_4(sc, MCASP_DITUDRB(chUsrDataBits));
    }

    return retVal;
}

/**
 * \brief   Gets the status of McASP transmission.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  Status of McASP Transmission. This is the contents of the 
 *          register XSTAT. \n
 *          The below tokens can be used for each status bits returned. \n
 *              MCASP_TX_STAT_ERR  \n                    
 *              MCASP_TX_STAT_DMAERR \n                  
 *              MCASP_TX_STAT_STARTOFFRAME \n             
 *              MCASP_TX_STAT_DATAREADY \n            
 *              MCASP_TX_STAT_LASTSLOT \n                
 *              MCASP_TX_STAT_CURRSLOT_EVEN \n           
 *              MCASP_TX_STAT_CURRSLOT_ODD \n             
 *              MCASP_TX_STAT_CLKFAIL \n                 
 *              MCASP_TX_STAT_SYNCERR \n                
 *              MCASP_TX_STAT_UNDERRUN               
 *
 **/
uint32_t mcasp_tx_status_get(struct ti_mcasp_softc *sc)
{
    return ti_mcasp_read_4(sc, MCASP_XSTAT);
}

/**
 * \brief   Gets the status of McASP reception
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 * \return  Status of McASP reception. This is the contents of the
 *          register RSTAT. \n
 *          The below tokens can be used for each status bits returned. \n
 *              MCASP_RX_STAT_ERR  \n
 *              MCASP_RX_STAT_DMAERR \n
 *              MCASP_RX_STAT_STARTOFFRAME \n
 *              MCASP_RX_STAT_DATAREADY \n
 *              MCASP_RX_STAT_LASTSLOT \n
 *              MCASP_RX_STAT_CURRSLOT_EVEN \n
 *              MCASP_RX_STAT_CURRSLOT_ODD \n
 *              MCASP_RX_STAT_CLKFAIL \n
 *              MCASP_RX_STAT_SYNCERR \n
 *              MCASP_RX_STAT_OVERRUN
 *
 **/
uint32_t mcasp_rx_status_get(struct ti_mcasp_softc *sc)
{
    return ti_mcasp_read_4(sc, MCASP_RSTAT);
}

uint32_t mcasp_global_status_get(struct ti_mcasp_softc *sc)
{
    return ti_mcasp_read_4(sc, MCASP_GBLCTL);
}

/**
 * \brief   Sets the status of McASP transmission.
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 *          The below tokens can be used for each status bits set. \n
 *              MCASP_TX_STAT_ERR  \n
 *              MCASP_TX_STAT_DMAERR \n
 *              MCASP_TX_STAT_STARTOFFRAME \n
 *              MCASP_TX_STAT_DATAREADY \n
 *              MCASP_TX_STAT_LASTSLOT \n
 *              MCASP_TX_STAT_CURRSLOT_EVEN \n
 *              MCASP_TX_STAT_CURRSLOT_ODD \n
 *              MCASP_TX_STAT_CLKFAIL \n
 *              MCASP_TX_STAT_SYNCERR \n
 *              MCASP_TX_STAT_UNDERRUN
 *
 **/
void mcasp_tx_status_set(struct ti_mcasp_softc *sc, uint32_t status)
{
    return ti_mcasp_write_4(sc, MCASP_XSTAT, status);
}

/**
 * \brief   Sets the status of McASP reception
 *
 * \param   baseAddr      Base Address of the McASP Module Registers.
 *
 *          The below tokens can be used for each status bits set. \n
 *              MCASP_RX_STAT_ERR  \n
 *              MCASP_RX_STAT_DMAERR \n
 *              MCASP_RX_STAT_STARTOFFRAME \n
 *              MCASP_RX_STAT_DATAREADY \n
 *              MCASP_RX_STAT_LASTSLOT \n
 *              MCASP_RX_STAT_CURRSLOT_EVEN \n
 *              MCASP_RX_STAT_CURRSLOT_ODD \n
 *              MCASP_RX_STAT_CLKFAIL \n
 *              MCASP_RX_STAT_SYNCERR \n
 *              MCASP_RX_STAT_OVERRUN \n
 *
 **/
void mcasp_rx_status_set(struct ti_mcasp_softc *sc, uint32_t status)
{
    return ti_mcasp_write_4(sc, MCASP_RSTAT, status);
}

/**
* \brief  This function saves the context of McASP registers.
*         This is useful in power management, where the power supply to McASP
*         controller will be cut off.
*
* \param  baseAddrCtrl     Base Address of the McASP Module Control Registers.
* \param  baseAddrFifo     Base Address of the McASP FIFO Registers.
* \param  contextPtr       The pointer to structure where McASP context is saved
* \param  sectFlag         Sections for saving context
*           sectFlag can take the following values \n
*                McASP_CONTEXT_TX - Context of Transmit section will be saved
*                McASP_CONTEXT_RX - Context of Receive section will be saved
*                McASP_CONTEXT_BOTH - Both section contexts will be saved
*
* \return  None
**/
void mcasp_context_save(struct ti_mcasp_softc *scCtrl, struct ti_mcasp_softc *scFifo,
                      mcasp_context_t *contextPtr, uint32_t sectFlag)
{
    uint32_t idx;

    /* Check if Transmit section context needs to be saved */
    if(MCASP_CONTEXT_TX & sectFlag)
    {
        contextPtr->fifoWfifoCtl = ti_mcasp_read_4(scFifo, MCASP_FIFO_WFIFOCTL);
        contextPtr->xmask        = ti_mcasp_read_4(scCtrl, MCASP_XMASK);
        contextPtr->xfmt         = ti_mcasp_read_4(scCtrl, MCASP_XFMT);
        contextPtr->afsxctl      = ti_mcasp_read_4(scCtrl, MCASP_AFSXCTL);
        contextPtr->aclkxctl     = ti_mcasp_read_4(scCtrl, MCASP_ACLKXCTL);
        contextPtr->ahclkxctl    = ti_mcasp_read_4(scCtrl, MCASP_AHCLKXCTL);
        contextPtr->xclkchk      = ti_mcasp_read_4(scCtrl, MCASP_XCLKCHK);
        contextPtr->xtdm         = ti_mcasp_read_4(scCtrl, MCASP_XTDM);
    }

    /* Check if receive section context needs to be saved */
    if(MCASP_CONTEXT_RX & sectFlag)
    {
        contextPtr->fifoRfifoCtl = ti_mcasp_read_4(scFifo, MCASP_FIFO_RFIFOCTL);
        contextPtr->rmask        = ti_mcasp_read_4(scCtrl, MCASP_RMASK);
        contextPtr->rfmt         = ti_mcasp_read_4(scCtrl, MCASP_RFMT);
        contextPtr->afsrctl      = ti_mcasp_read_4(scCtrl, MCASP_AFSRCTL);
        contextPtr->aclkrctl     = ti_mcasp_read_4(scCtrl, MCASP_ACLKRCTL);
        contextPtr->ahclkrctl    = ti_mcasp_read_4(scCtrl, MCASP_AHCLKRCTL);
        contextPtr->rclkchk      = ti_mcasp_read_4(scCtrl, MCASP_RCLKCHK);
        contextPtr->rtdm         = ti_mcasp_read_4(scCtrl, MCASP_RTDM);
    }

    for(idx = 0; idx < 15; idx++)
    {
        contextPtr->srctl[idx] = ti_mcasp_read_4(scCtrl, MCASP_SRCTL(idx));
    }

    contextPtr->pfunc  = ti_mcasp_read_4(scCtrl, MCASP_PFUNC);
    contextPtr->pdir   = ti_mcasp_read_4(scCtrl, MCASP_PDIR);
    contextPtr->gblctl = ti_mcasp_read_4(scCtrl, MCASP_GBLCTL);
}

/**
* \brief  This function restores the context of McASP registers.
*         This is useful in power management, where the power supply to McASP
*         controller will be cut off. Note that this API does not enable McASP 
*         clocks. Also McASP state machine shall be brought out of reset 
*         separately.
*
* \param  baseAddrCtrl     Base Address of the McASP Module Control Registers.
* \param  baseAddrFifo     Base Address of the McASP FIFO Registers.
* \param  contextPtr       The pointer to structure where McASP context is saved
* \param  sectFlag         Sections for saving context
*           sectFlag can take the following values \n
*                McASP_CONTEXT_TX - Context of Transmit section will be saved
*                McASP_CONTEXT_RX - Context of Receive section will be saved
*                McASP_CONTEXT_BOTH - Both section contexts will be saved
*
* \return  None
**/
void mcasp_context_restore(struct ti_mcasp_softc *scCtrl, struct ti_mcasp_softc *scFifo,
                         mcasp_context_t *contextPtr, uint32_t sectFlag)
{
    uint32_t idx;

    if(MCASP_CONTEXT_TX & sectFlag)
    {
        ti_mcasp_write_4(scCtrl, MCASP_XGBLCTL, 0x0);
        ti_mcasp_write_4(scFifo, MCASP_FIFO_WFIFOCTL, contextPtr->fifoWfifoCtl);
        ti_mcasp_write_4(scCtrl, MCASP_XMASK, contextPtr->xmask);
        ti_mcasp_write_4(scCtrl, MCASP_XFMT, contextPtr->xfmt);
        ti_mcasp_write_4(scCtrl, MCASP_AFSXCTL, contextPtr->afsxctl);
        ti_mcasp_write_4(scCtrl, MCASP_ACLKXCTL, contextPtr->aclkxctl);
        ti_mcasp_write_4(scCtrl, MCASP_AHCLKXCTL, contextPtr->ahclkxctl);
        ti_mcasp_write_4(scCtrl, MCASP_XCLKCHK, contextPtr->xclkchk);
        ti_mcasp_write_4(scCtrl, MCASP_XTDM, contextPtr->xtdm);
    }

    if(MCASP_CONTEXT_RX & sectFlag)
    {
        ti_mcasp_write_4(scCtrl, MCASP_RGBLCTL, 0x0);
        ti_mcasp_write_4(scFifo, MCASP_FIFO_RFIFOCTL, contextPtr->fifoRfifoCtl);
        ti_mcasp_write_4(scCtrl, MCASP_RMASK, contextPtr->rmask);
        ti_mcasp_write_4(scCtrl, MCASP_RFMT, contextPtr->rfmt);
        ti_mcasp_write_4(scCtrl, MCASP_AFSRCTL, contextPtr->afsrctl);
        ti_mcasp_write_4(scCtrl, MCASP_ACLKRCTL, contextPtr->aclkrctl);
        ti_mcasp_write_4(scCtrl, MCASP_AHCLKRCTL, contextPtr->ahclkrctl);
        ti_mcasp_write_4(scCtrl, MCASP_RCLKCHK, contextPtr->rclkchk);
        ti_mcasp_write_4(scCtrl, MCASP_RTDM, contextPtr->rtdm);
    }

    for(idx = 0; idx < 15; idx++)
    {
	ti_mcasp_write_4(scCtrl, MCASP_SRCTL(idx), contextPtr->srctl[idx]);
    }

    ti_mcasp_write_4(scCtrl, MCASP_PFUNC, contextPtr->pfunc);
    ti_mcasp_write_4(scCtrl, MCASP_PDIR, contextPtr->pdir);
}

/***************************** End Of File ***********************************/

