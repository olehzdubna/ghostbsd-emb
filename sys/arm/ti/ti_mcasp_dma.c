/**
 *  \file   mcasp_dma.c
 *
 *  \brief  McASP DMA.
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

#include <sys/types.h>
#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/errno.h>

#include <vm/vm.h>
#include <vm/pmap.h>

/* HW Macros and Peripheral Defines */
#include "ti_mcasp_hw.h"

/* Driver APIs */
#include "ti_mcasp_api.h"

#include "ti_mcasp_dma.h"

#include "ti_edma3.h"
#include "ti_edma3_event.h"

#include "toneRaw.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
******************************************************************************/
/* Slot size to send/receive data */
#define SLOT_SIZE                     (32u)

#define WORD_SIZE                     (32u)
/* Number of linked parameter set used per tx/rx */
#define NUM_PAR                       (2u)

/* Specify where the parameter set starting is */
#define PAR_ID_START                  (70u)

/*
** Below Macros are calculated based on the above inputs
*/
#define PAR_RX_START                  (PAR_ID_START)
#define PAR_TX_START                  (PAR_RX_START + NUM_PAR)

/*
** Definitions which are not configurable
*/
#define SIZE_PARAMSET                 (32u)

MALLOC_DEFINE(M_MCASP_TX_BUF_TYPE, "mcasp_tx_buffer", "buffer mcasp tx data");
MALLOC_DEFINE(M_MCASP_RX_BUF_TYPE, "mcasp_rx_buffer", "buffer mcasp rx data");

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
******************************************************************************/
/*
** Values which are configurable
*/

/* Number of channels, L & R */
#define NUM_I2S_CHANNELS                      (2u) 

/* Number of samples to be used per audio buffer */
#define NUM_SAMPLES_PER_AUDIO_BUF             (2048u)

/* Number of buffers used per tx/rx */
#define NUM_BUF                               (2u)

/*
** Below Macros are calculated based on the above inputs
*/
#define NUM_TX_SERIALIZERS                    ((NUM_I2S_CHANNELS >> 1) \
                                               + (NUM_I2S_CHANNELS & 0x01))
#define NUM_RX_SERIALIZERS                    ((NUM_I2S_CHANNELS >> 1) \
                                               + (NUM_I2S_CHANNELS & 0x01))
#define I2S_SLOTS                             ((1 << NUM_I2S_CHANNELS) - 1)

#define BYTES_PER_SAMPLE                      (4u)

#define PAR_RX_START                          (PAR_ID_START)
#define PAR_TX_START                          (PAR_RX_START + NUM_PAR)

/* McASP Serializer for Receive */
#define MCASP_XSER_RX                         (1u)

/* McASP Serializer for Transmit */
#define MCASP_XSER_TX                         (0u)

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
******************************************************************************/

/*
** Next buffer to receive data. The data will be received in this buffer.
*/
static volatile unsigned int nxtBufToRcv = 0;

/*
** The RX buffer which filled latest.
*/
static volatile unsigned int lastFullRxBuf = 0;

/*
** The offset of the paRAM ID, from the starting of the paRAM set.
*/
static volatile unsigned short parOffRcvd = 0;

/*
** The offset of the paRAM ID sent, from starting of the paRAM set.
*/
static volatile unsigned short parOffSent = 0;

/*
** The offset of the paRAM ID to be sent next, from starting of the paRAM set. 
*/
//static volatile unsigned short parOffTxToSend = 0;

/*
** The transmit buffer which was sent last.
*/
static volatile unsigned int lastSentTxBuf = NUM_BUF - 1;

static volatile unsigned int parOffTxToSend = 0;

/*
** Default paRAM for Transmit section. This will be transmitting from 
** a loop buffer.
*/
static struct ti_edma3cc_param_set const txParamsDeflt = {
  .opt.sam        = 0,                            /* Source address mode */
  .opt.dam        = 0,                            /* Destination address mode */
  .opt.syncdim    = 0,                            /* Transfer synchronization dimension */
  .opt.static_set = 0,                            /* Static Set */
  .opt.fwid       = 2,                            /* FIFO Width */
  .opt.tccmode    = 0,                            /* Transfer complete code mode */
  .opt.tcc        = 0,                            /* Transfer complete code */
  .opt.tcinten    = 0,                            /* Transfer complete interrupt enable */
  .opt.itcinten   = 0,                            /* Intermediate xfer completion intr. ena */
  .opt.tcchen     = 0,                            /* Transfer complete chaining enable */
  .opt.itcchen    = 0,                            /* Intermediate xfer completion chaining ena */
  .opt.privid     = 0,                            /* Privilege identification */
  .opt.priv       = 0,                            /* Privilege level */
  .src            = 0,                            /* Channel Source Address */
  .acnt           = BYTES_PER_SAMPLE,             /* Count for 1st Dimension */
  .bcnt           = NUM_SAMPLES_PER_AUDIO_BUF/2,  /* Count for 2nd Dimension */
  .dst            = 0x46000000,                   /* Channel Destination Address */
  .srcbidx        = BYTES_PER_SAMPLE,             /* Source B Index */
  .dstbidx        = 0,                            /* Destination B Index */
  .link           = PAR_TX_START * SIZE_PARAMSET, /* Link Address */
  .bcntrld        = 0,		                      /* BCNT Reload */
  .srccidx        = 0,		                      /* Source C Index */
  .dstcidx        = 0,		                      /* Destination C Index */
  .ccnt           = 1,	                          /* Count for 3rd Dimension */
  .reserved       = 0
};

/*
** Default paRAM for Receive section.  
*/
static struct ti_edma3cc_param_set const rxParamsDflt = {
  .opt.sam        = 0,                            /* Source address mode */
  .opt.dam        = 0,                            /* Destination address mode */
  .opt.syncdim    = 0,                            /* Transfer synchronization dimension */
  .opt.static_set = 0,                            /* Static Set */
  .opt.fwid       = 2,                            /* FIFO Width */
  .opt.tccmode    = 0,                            /* Transfer complete code mode */
  .opt.tcc        = 0,                            /* Transfer complete code */
  .opt.tcinten    = 0,                            /* Transfer complete interrupt enable */
  .opt.itcinten   = 0,                            /* Intermediate xfer completion intr. ena */
  .opt.tcchen     = 0,                            /* Transfer complete chaining enable */
  .opt.itcchen    = 0,                            /* Intermediate xfer completion chaining ena */
  .opt.privid     = 0,                            /* Privilege identification */
  .opt.priv       = 0,                            /* Privilege level */
  .src            = 0x46000000,                   /* Channel Source Address */
  .acnt           = BYTES_PER_SAMPLE,             /* Count for 1st Dimension */
  .bcnt           = NUM_SAMPLES_PER_AUDIO_BUF/2,  /* Count for 2nd Dimension */
  .dst            = 0,                            /* Channel Destination Address */
  .srcbidx        = 0,                            /* Source B Index */
  .dstbidx        = BYTES_PER_SAMPLE,             /* Destination B Index */
  .link           = PAR_RX_START * SIZE_PARAMSET, /* Link Address */
  .bcntrld        = 0,		                      /* BCNT Reload */
  .srccidx        = 0,		                      /* Source C Index */
  .dstcidx        = 0,		                      /* Destination C Index */
  .ccnt           = 1,	                          /* Count for 3rd Dimension */
  .reserved       = 0
};

#define MCASP_DMA_RX (0u)
#define MCASP_DMA_TX (1u)

static bus_dma_tag_t  dmatag[2];
static bus_dmamap_t   dmamap[2];
static uint32_t*      dmamem[2];
static bus_addr_t     paddr[2]; /* DMA buffer address */

static int dma_bufsiz = NUM_SAMPLES_PER_AUDIO_BUF*BYTES_PER_SAMPLE*NUM_BUF;

static bus_addr_t rx_bufs[NUM_BUF];
static bus_addr_t tx_bufs[NUM_BUF];

/******************************************************************************
**                          FUNCTION DEFINITIONS
******************************************************************************/

static int  mcasp_dma_alloc(struct ti_mcasp_softc *sc, int mem_idx);
static void mcasp_dma_free(int mem_idx);
static void mcasp_getaddr_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error);


int release_mcasp_dma(void)
{
	mcasp_dma_free(MCASP_DMA_RX);
	mcasp_dma_free(MCASP_DMA_TX);

	return 0;
}

static int
mcasp_dma_alloc(struct ti_mcasp_softc *sc, int mem_idx)
{
	int err;

	/*
	 * Allocate the DMA tag for an EDMA3 buffer.
	 */
	err = bus_dma_tag_create(bus_get_dma_tag(sc->dev), dma_bufsiz,
	    0, BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
		dma_bufsiz, 1, dma_bufsiz, BUS_DMA_ALLOCNOW,
	    NULL, NULL, &dmatag[mem_idx]);
	if (err != 0) {
		printf("McASP DMA: Can't create DMA tag for DMA\n");
		return (err);
	}
	/* Allocate DMA memory for the EDMA3 bounce buffer. */
	err = bus_dmamem_alloc(dmatag[mem_idx], (void **)&dmamem[mem_idx],
	    BUS_DMA_NOWAIT, &dmamap[mem_idx]);
	if (err != 0) {
		printf("McASP DMA: Can't allocate DMA memory for DMA\n");
		bus_dma_tag_destroy(dmatag[mem_idx]);
		return (err);
	}
	/* Map the memory of the EDMA3 bounce buffer. */
	err = bus_dmamap_load(dmatag[mem_idx], dmamap[mem_idx],
	    (void *)dmamem[mem_idx], dma_bufsiz, mcasp_getaddr_cb, &paddr[mem_idx], 0);

	if (err != 0 || paddr[mem_idx] == 0) {
		printf("McASP DMA: Can't load DMA memory for DMA\n");
		bus_dmamem_free(dmatag[mem_idx], dmamem[mem_idx], dmamap[mem_idx]);
		bus_dma_tag_destroy(dmatag[mem_idx]);

		if (err)
			return (err);
		else
			return (EFAULT);
	}

	return (0);
}

static void mcasp_dma_free(int mem_idx)
{
	bus_dmamap_unload(dmatag[mem_idx], dmamap[mem_idx]);
	bus_dmamem_free(dmatag[mem_idx], dmamem[mem_idx], dmamap[mem_idx]);
	bus_dma_tag_destroy(dmatag[mem_idx]);
}

int mcasp_dma_get_size(void) {
  return NUM_SAMPLES_PER_AUDIO_BUF;
}

uint32_t* mcasp_dma_get_rx_mem() {
  return &dmamem[MCASP_DMA_RX][lastFullRxBuf * NUM_SAMPLES_PER_AUDIO_BUF];
}
uint32_t* mcasp_dma_get_tx_mem() {
  return &dmamem[MCASP_DMA_TX][lastSentTxBuf * NUM_SAMPLES_PER_AUDIO_BUF];
}

static void
mcasp_getaddr_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error != 0) {
		printf("McASP DMA: getaddr_cb: error %d\n", error);
		return;
	}

	for(int i=0; i<nsegs;++i) {
		printf("+++ mcasp_getaddr_cb segment %d: %8.8lx, len: %ld\n", i, segs[i].ds_addr, segs[i].ds_len);
	}

	*(bus_addr_t *)arg = segs[0].ds_addr;
}


/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static void McASPI2SConfigure(struct ti_mcasp_softc *sc);
static void McASPTxDMAComplHandler(void);
static void McASPRxDMAComplHandler(void);
static void EDMA3CCComplIsr(uint32_t);
static void I2SDataTxRxActivate(struct ti_mcasp_softc *sc);
static void I2SDMAParamInit(void);
static void ParamTxLoopJobSet(unsigned short parId);
static void BufferTxDMAActivate(unsigned int txBuf, unsigned short numSamples,
                                unsigned short parToUpdate, 
                                unsigned short linkAddr);
static void BufferRxDMAActivate(unsigned int rxBuf, unsigned short parId,
                                unsigned short parLink);

/*
** Assigns loop job for a parameter set
*/
static void ParamTxLoopJobSet(unsigned short parId)
{
   struct ti_edma3cc_param_set paramSet;

    memcpy(&paramSet, &txParamsDeflt, SIZE_PARAMSET - 2);

    /* link the paRAM to itself */
    paramSet.src = tx_bufs[0];
    paramSet.link = parId * SIZE_PARAMSET;

//    printf("+++ ParamTxLoopJobSet params: ");
//    uint32_t* pptr = (uint32_t*)&paramSet;
//    for(int i=0; i<8; i++) {
//    	printf("%8.8x ", pptr[i] );
//    }
//    printf("\n");

    ti_edma3_param_write(parId, &paramSet);
}

/*
** Initializes the DMA parameters.
** The RX basic paRAM set(channel) is 0 and TX basic paRAM set (channel) is 1.
**
** The RX paRAM set 0 will be initialized to receive data in the rx buffer 0.
** The transfer completion interrupt will not be enabled for paRAM set 0;
** paRAM set 0 will be linked to linked paRAM set starting (PAR_RX_START) of RX.
** and further reception only happens via linked paRAM set. 
** For example, if the PAR_RX_START value is 40, and the number of paRAMS is 2, 
** reception paRAM set linking will be initialized as 0-->40-->41-->40
**
** The TX paRAM sets will be initialized to transmit from the loop buffer.
** The size of the loop buffer can be configured.   
** The transfer completion interrupt will not be enabled for paRAM set 1;
** paRAM set 1 will be linked to linked paRAM set starting (PAR_TX_START) of TX.
** All other paRAM sets will be linked to itself.
** and further transmission only happens via linked paRAM set.
** For example, if the PAR_RX_START value is 42, and the number of paRAMS is 2, 
** So transmission paRAM set linking will be initialized as 1-->42-->42, 43->43. 
*/
static void I2SDMAParamInit(void)
{
    struct ti_edma3cc_param_set paramSet;
    int idx; 

    /* Initialize the 0th paRAM set for receive */ 
    memcpy(&paramSet, &rxParamsDflt, SIZE_PARAMSET - 2);

    paramSet.dst = rx_bufs[0];

    printf("+++ I2SDMAParamInit params: %d, ", EDMA3_CHA_MCASP0_RX  );
    uint32_t* pptr = (uint32_t*)&paramSet;
    for(int i=0; i<8; i++) {
    	printf("%8.8x ", pptr[i] );
    }
    printf("\n");
    ti_edma3_param_write(EDMA3_CHA_MCASP0_RX, &paramSet);

    /* further paramsets, enable interrupt */
    paramSet.opt.tcc = EDMA3_CHA_MCASP0_RX;
    paramSet.opt.tcinten = 1;
 
    for(idx = 0 ; idx < NUM_PAR; idx++)
    {
        paramSet.dst = rx_bufs[idx];

        paramSet.link = (PAR_RX_START + ((idx + 1) % NUM_PAR))
                             * (SIZE_PARAMSET);        

        paramSet.bcnt =  NUM_SAMPLES_PER_AUDIO_BUF;

        /* 
        ** for the first linked paRAM set, start receiving the second
        ** sample only since the first sample is already received in
        ** rx buffer 0 itself.
        */
        if( 0 == idx)
        {
            paramSet.dst  += BYTES_PER_SAMPLE;
            paramSet.bcnt -= BYTES_PER_SAMPLE;
        }

        printf("+++ I2SDMAParamInit params: %d, ", PAR_RX_START + idx);
        uint32_t* pptr = (uint32_t*)&paramSet;
        for(int i=0; i<8; i++) {
        	printf("%8.8x ", pptr[i] );
        }
        printf("\n");

        ti_edma3_param_write(PAR_RX_START + idx, &paramSet);
    } 

    /* Initialize the required variables for reception */
    nxtBufToRcv = idx % NUM_BUF;
    lastFullRxBuf = NUM_BUF - 1;
    parOffRcvd = 0;

    /* Initialize the 1st paRAM set for transmit */ 
    memcpy(&paramSet, &txParamsDeflt, SIZE_PARAMSET - 2);

    paramSet.src = tx_bufs[0];
    ti_edma3_param_write(EDMA3_CHA_MCASP0_TX, &paramSet);

    /* rest of the params, enable loop job */
    for(idx = 0 ; idx < NUM_PAR; idx++)
    {
        ParamTxLoopJobSet(PAR_TX_START + idx);
    }
 
    /* Initialize the variables for transmit */
    parOffSent = 0;
    lastSentTxBuf = NUM_BUF - 1; 
}

/*
** Configures the McASP Transmit Section in I2S mode.
*/
static void McASPI2SConfigure(struct ti_mcasp_softc *sc)
{
	mcasp_rx_reset(sc);
	mcasp_tx_reset(sc);

    /* Enable the FIFOs for DMA transfer */
	mcasp_read_fifo_enable(sc, 1, 1);
	mcasp_write_fifo_enable(sc, 1, 1);

    /* Set I2S format in the transmitter/receiver format units */
	mcasp_rx_fmt_i2s_set(sc, WORD_SIZE, SLOT_SIZE,
                     MCASP_RX_MODE_DMA);
    mcasp_tx_fmt_i2s_set(sc, WORD_SIZE, SLOT_SIZE,
                     MCASP_TX_MODE_DMA);

    /* Configure the frame sync. I2S shall work in TDM format with 2 slots */
    mcasp_rx_frame_sync_cfg(sc, 2, MCASP_RX_FS_WIDTH_WORD,
                        MCASP_RX_FS_EXT_BEGIN_ON_FALL_EDGE);
    mcasp_tx_frame_sync_cfg(sc, 2, MCASP_TX_FS_WIDTH_WORD,
                        MCASP_TX_FS_INT_BEGIN_ON_FALL_EDGE);

    /* configure the clock for receiver */
    mcasp_rx_clk_cfg(sc, MCASP_RX_CLK_EXTERNAL, 16, 16);
    mcasp_rx_clk_polarity_set(sc, MCASP_RX_CLK_POL_RIS_EDGE);
    mcasp_rx_clk_check_config(sc, MCASP_RX_CLKCHCK_DIV32,
                          0x00, 0xFF);

    /* configure the clock for transmitter */
    mcasp_tx_clk_cfg(sc, MCASP_TX_CLK_MIXED, 16, 16);
    mcasp_tx_clk_polarity_set(sc, MCASP_TX_CLK_POL_FALL_EDGE);
    mcasp_tx_clk_check_config(sc, MCASP_TX_CLKCHCK_DIV32,
                          0x00, 0xFF);
 
    /* Enable synchronization of RX and TX sections  */  
    mcasp_tx_rx_clk_sync_disable(sc);

    /* Enable the transmitter/receiver slots. I2S uses 2 slots */
    mcasp_rx_time_slot_set(sc, I2S_SLOTS);
    mcasp_tx_time_slot_set(sc, I2S_SLOTS);

    /*
    ** Set the serializers, Currently only one serializer is set as
    ** transmitter and one serializer as receiver.
    */
    mcasp_serializer_rx_set(sc, MCASP_XSER_RX);
    mcasp_serializer_tx_set(sc, MCASP_XSER_TX);

    /*
    ** Configure the McASP pins 
    ** Input - Frame Sync, Clock and Serializer Rx
    ** Output - Serializer Tx is connected to the input of the codec 
    */

    mcasp_pin_mcasp_set(sc, MCASP_PIN_AFSR |
    		                MCASP_PIN_AHCLKR |
    		                MCASP_PIN_ACLKR |
    		                MCASP_PIN_AFSX |
    		                MCASP_PIN_AHCLKX |
    		                MCASP_PIN_ACLKX |
    		                MCASP_PIN_AXR(MCASP_XSER_TX) |
    		                MCASP_PIN_AXR(MCASP_XSER_RX));

//    mcasp_pin_dir_input_set(sc, 0xFFFFFFFF);
    mcasp_pin_dir_input_set(sc,   MCASP_PIN_AXR(MCASP_XSER_RX)
								| MCASP_PIN_ACLKR
								| MCASP_PIN_AFSR
                                | MCASP_PIN_AHCLKX);

    mcasp_pin_dir_output_set(sc, MCASP_PIN_AXR(MCASP_XSER_TX)
                               | MCASP_PIN_ACLKX
                               | MCASP_PIN_AFSX);

//    mcasp_rx_int_enable(sc, MCASP_RX_DMAERROR | MCASP_RX_OVERRUN);
//    mcasp_tx_int_enable(sc, MCASP_TX_DMAERROR | MCASP_TX_UNDERRUN);
}

/*
** Activates the data transmission/reception
** The DMA parameters shall be ready before calling this function.
*/
static void I2SDataTxRxActivate(struct ti_mcasp_softc *sc)
{
    /* Start the clocks */
	mcasp_rx_clk_start(sc, MCASP_RX_CLK_EXTERNAL);
	mcasp_tx_clk_start(sc, MCASP_TX_CLK_MIXED);

	printf("+++ I2SDataTxRxActivate clocks started\n");

    /* Enable EDMA for the transfer */
    ti_edma3_enable_transfer_event(EDMA3_CHA_MCASP0_RX);
    ti_edma3_enable_transfer_event(EDMA3_CHA_MCASP0_TX);

	printf("+++ I2SDataTxRxActivate DMAs enabled\n");

    /* Activate the  serializers */
    mcasp_rx_ser_activate(sc);
    mcasp_tx_ser_activate(sc);

	printf("+++ I2SDataTxRxActivate serializedrs activated\n");

    /* make sure that the XDATA bit is cleared to zero */
    while(mcasp_tx_status_get(sc) & MCASP_TX_STAT_DATAREADY);

    /* Activate the state machines */
    mcasp_rx_enable(sc);
    mcasp_tx_enable(sc);

	printf("+++ I2SDataTxRxActivate state machines activated\n");
}

/*
** Activates the DMA transfer for a parameterset from the given buffer.
*/
void BufferTxDMAActivate(unsigned int txBuf, unsigned short numSamples,
                         unsigned short parId, unsigned short linkPar)
{
	struct ti_edma3cc_param_set paramSet;

    /* Copy the default paramset */
    memcpy(&paramSet, &txParamsDeflt, SIZE_PARAMSET - 2);
    
    /* Enable completion interrupt */
    paramSet.opt.tcc = EDMA3_CHA_MCASP0_TX;
    paramSet.opt.tcinten = 1;
    paramSet.src =  tx_bufs[txBuf];
    paramSet.link = linkPar * SIZE_PARAMSET;
    paramSet.bcnt = numSamples;
//
//    printf("+++ BufferTxDMAActivate params, id: %d ", parId);
//    uint32_t* pptr = (uint32_t*)&paramSet;
//    for(int i=0; i<8; i++) {
//    	printf("%8.8x ", pptr[i] );
//    }
//    printf("\n");

	bus_dmamap_sync(dmatag[MCASP_DMA_TX], dmamap[MCASP_DMA_TX], BUS_DMASYNC_PREWRITE);

    ti_edma3_param_write(parId, &paramSet);
}

static void* rx_chan;
static void* tx_chan;

/*
** The main function. Application starts here.
*/
void mcasp_start_test(struct ti_mcasp_softc *sc)
{
    int rxrc = mcasp_dma_alloc(sc, MCASP_DMA_RX);
    int txrc = mcasp_dma_alloc(sc, MCASP_DMA_TX);

    if(rxrc || txrc) {
    	printf("DMA Memory allocation error, exiting\n");
    	return;
    }

    rx_chan = &sc->rx_chan;
    tx_chan = &sc->tx_chan;

    for(int i=0; i<NUM_BUF; i++) {
    	rx_bufs[i] = paddr[MCASP_DMA_RX] + i * NUM_SAMPLES_PER_AUDIO_BUF;
    	tx_bufs[i] = paddr[MCASP_DMA_TX] + i * NUM_SAMPLES_PER_AUDIO_BUF;
    }

    printf("+++ RX memory: %8.8lx %p\n", paddr[MCASP_DMA_RX], dmamem[MCASP_DMA_RX]);
    printf("+++ TX memory: %8.8lx %p\n", paddr[MCASP_DMA_TX], dmamem[MCASP_DMA_TX]);

    for(int i=0;i<NUM_SAMPLES_PER_AUDIO_BUF;i++) {
    	if(i&1)
    		dmamem[MCASP_DMA_TX][i] = 0x01020304;
    	else
    		dmamem[MCASP_DMA_TX][i] = 0x0f0e0d0c;
    }

	ti_edma3_init(0);

	ti_edma3_enable_event_intr(EDMA3_CHA_MCASP0_TX, EDMA3CCComplIsr);
	ti_edma3_enable_event_intr(EDMA3_CHA_MCASP0_RX, EDMA3CCComplIsr);

	/* Request TX EDMA channel */
	ti_edma3_request_dma_ch(EDMA3_CHA_MCASP0_TX, EDMA3_CHA_MCASP0_TX, 0);

	/* Request RX EDMA channel */
  	ti_edma3_request_dma_ch(EDMA3_CHA_MCASP0_RX, EDMA3_CHA_MCASP0_RX, 0);

    /* Initialize the DMA parameters */
    I2SDMAParamInit();

    /* Configure the McASP for I2S */
    McASPI2SConfigure(sc);
}

void mcasp_enable_transfers(struct ti_mcasp_softc *sc)
{
    /* Activate the audio transmission and reception */ 
    I2SDataTxRxActivate(sc);
}

int mcasp_run_test(void)
{
    unsigned short parToSend;
    unsigned short parToLink;

	/*
	** Start the transmission from the link paramset. The param set
	** 1 will be linked to param set at PAR_TX_START. So do not
	** update paRAM set1.
	*/
	parToSend =  PAR_TX_START + (parOffTxToSend % NUM_PAR);
	parOffTxToSend = (parOffTxToSend + 1) % NUM_PAR;
	parToLink  = PAR_TX_START + parOffTxToSend;

	lastSentTxBuf = (lastSentTxBuf + 1) % NUM_BUF;


	/* Copy the buffer */
//	memcpy(dmamem[MCASP_DMA_TX], toneRaw, 1024);
//
//	printf("+++ mcasp_run_test lastSentTxBuf: %d, parToSend: %d, parToLink: %d\n", lastSentTxBuf, parToSend, parToLink);

	bus_dmamap_sync(dmatag[MCASP_DMA_TX], dmamap[MCASP_DMA_TX], BUS_DMASYNC_PREWRITE);

	/*
	** Send the buffer by setting the DMA params accordingly.
	** Here the buffer to send and number of samples are passed as
	** parameters. This is important, if only transmit section
	** is to be used.
	*/
	BufferTxDMAActivate(lastSentTxBuf, NUM_SAMPLES_PER_AUDIO_BUF,
						(unsigned short)parToSend,
						(unsigned short)parToLink);

	return 0;
}  

/*
** Activates the DMA transfer for a parameter set from the given buffer.
*/
static void BufferRxDMAActivate(unsigned int rxBuf, unsigned short parId,
                                unsigned short parLink)
{
	struct ti_edma3cc_param_set paramSet;

    /* Copy the default paramset */
    memcpy(&paramSet, &rxParamsDflt, SIZE_PARAMSET - 2);

    /* Enable completion interrupt */
    paramSet.opt.tcc     = EDMA3_CHA_MCASP0_RX;
    paramSet.opt.tcinten = 1;
    paramSet.dst =  rx_bufs[rxBuf];
    paramSet.bcnt = NUM_SAMPLES_PER_AUDIO_BUF;
    paramSet.link = parLink * SIZE_PARAMSET ;

//    printf("+++ BufferRxDMAActivate params, id: %d ", parId);
//    uint32_t* pptr = (uint32_t*)&paramSet;
//    for(int i=0; i<8; i++) {
//    	printf("%8.8x ", pptr[i] );
//    }
//    printf("\n");

	bus_dmamap_sync(dmatag[MCASP_DMA_RX], dmamap[MCASP_DMA_RX], BUS_DMASYNC_PREREAD);

    ti_edma3_param_write(parId, &paramSet);
}

/*
** This function will be called once receive DMA is completed
*/
static void McASPRxDMAComplHandler(void)
{
    unsigned short nxtParToUpdate;

	bus_dmamap_sync(dmatag[MCASP_DMA_RX], dmamap[MCASP_DMA_RX], BUS_DMASYNC_POSTREAD);

    /*
    ** Update lastFullRxBuf to indicate a new buffer reception
    ** is completed.
    */
    lastFullRxBuf = (lastFullRxBuf + 1) % NUM_BUF;
    nxtParToUpdate =  PAR_RX_START + parOffRcvd;  
    parOffRcvd = (parOffRcvd + 1) % NUM_PAR;

	printf("+++ McASPRxDMAComplHandler lastFullRxBuf %d, nxtParToUpdate %d, parOffRcvd %d, nxtBufToRcv %d\n",
			lastFullRxBuf, nxtParToUpdate, parOffRcvd, nxtBufToRcv);

	wakeup(rx_chan);

//    printf("+++ McASPRxDMAComplHandler\n");
//
//    for(int j=0; j<16; j++) {
//    	printf("+++ start: %8.8lx ", paddr[MCASP_DMA_RX] + j*4);
//    	for(int i=0; i<8; i++) {
//    		printf("%8.8x ", dmamem[MCASP_DMA_RX][j*4 + i] );
//    	}
//        printf("\n");
//    }


    /*
    ** Update the DMA parameters for the received buffer to receive
    ** further data in proper buffer
    */
    BufferRxDMAActivate(nxtBufToRcv, nxtParToUpdate,
                        PAR_RX_START + parOffRcvd);
    
    /* update the next buffer to receive data */ 
    nxtBufToRcv = (nxtBufToRcv + 1) % NUM_BUF;
}

/*
** This function will be called once transmit DMA is completed
*/
static void McASPTxDMAComplHandler(void)
{
	bus_dmamap_sync(dmatag[MCASP_DMA_RX], dmamap[MCASP_DMA_RX], BUS_DMASYNC_POSTWRITE);

	wakeup(tx_chan);

    ParamTxLoopJobSet((unsigned short)(PAR_TX_START + parOffSent));

    parOffSent = (parOffSent + 1) % NUM_PAR;
}

/*
** EDMA transfer completion ISR
*/
static void EDMA3CCComplIsr(uint32_t event)
{

//	printf("+++ EDMA3CCComplIsr %8.8x\n", event);


    /* Check if receive DMA completed */
    if(event & (1 << EDMA3_CHA_MCASP0_RX))
    { 
        McASPRxDMAComplHandler();
    }
    
    /* Check if transmit DMA completed */
    if(event & (1 << EDMA3_CHA_MCASP0_TX))
    { 
        McASPTxDMAComplHandler();
    }
}


/***************************** End Of File ***********************************/


