


#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");


#include "ti_mcasp_api.h"
#include "ti_mcasp_ioctl.h"


int
mcasp_chdev_ioctl_calls(struct ti_mcasp_softc *sc, u_long cmd, caddr_t data, int fflag)
{
    mcasp_fifo_enable_t* fe = NULL;
    mcasp_fmt_i2s_t* i2cfmt = NULL;
    mcasp_frame_sync_t* frsync = NULL;
    mcasp_a_mute_enable_t* mute = NULL;
    mcasp_clk_t* clk =  NULL;
    mcasp_clk_check_t* check = NULL;
    mcasp_buf_t* buf = NULL;
    uint32_t mask = 0;
    uint32_t val = 0;

    switch(cmd) 
    {
       case MCASP_IOCTL_TX_RESET:
         mcasp_tx_reset(sc);
       break;
       case MCASP_IOCTL_RX_RESET:
         mcasp_rx_reset(sc);
       break;
       case MCASP_IOCTL_WRITE_FIFO_ENABLE:
         fe =  (mcasp_fifo_enable_t*)data;
         mcasp_write_fifo_enable(sc, fe->numTxSer, fe->minWdPerSer);
       break; 
       case MCASP_IOCTL_READ_FIFO_ENABLE:
         fe =  (mcasp_fifo_enable_t*)data;
         mcasp_read_fifo_enable(sc, fe->numTxSer, fe->minWdPerSer);
       break;
       case MCASP_IOCTL_TX_FMT_MASK_SET:
         mcasp_tx_fmt_mask_set(sc, mask);
       break;
       case MCASP_IOCTL_RX_FMT_MASK_SET:
         mcasp_rx_fmt_mask_set(sc, mask);
       break;
       case MCASP_IOCTL_TX_FMT_SET:
         mcasp_tx_fmt_set(sc, val);
       break;
       case MCASP_IOCTL_RX_FMT_SET:
         mcasp_rx_fmt_set(sc, val);
       break;
       case MCASP_IOCTL_TX_FMT_I2S_SET:
         i2cfmt = (mcasp_fmt_i2s_t*)data;
         mcasp_tx_fmt_i2s_set(sc, i2cfmt->wordSize,
                             i2cfmt->slotSize, i2cfmt->txMode);
       break;
       case MCASP_IOCTL_RX_FMT_I2S_SET:
         i2cfmt = (mcasp_fmt_i2s_t*)data;
         mcasp_rx_fmt_i2s_set(sc, i2cfmt->wordSize,
                             i2cfmt->slotSize, i2cfmt->txMode);
       break;
       case MCASP_IOCTL_TX_FRAME_SYNC_CFG:
          frsync = (mcasp_frame_sync_t*)data;
          mcasp_tx_frame_sync_cfg(sc, frsync->fsMode,
                                frsync->fsWidth, frsync->fsSetting);
       break;
       case MCASP_IOCTL_RX_FRAME_SYNC_CFG:
          frsync = (mcasp_frame_sync_t*)data;
          mcasp_rx_frame_sync_cfg(sc, frsync->fsMode,
                                frsync->fsWidth, frsync->fsSetting);
       break;
       case MCASP_IOCTL_TX_CLK_CFG:
          clk = (mcasp_clk_t*)data;
          mcasp_tx_clk_cfg(sc, clk->clkSrc,
                          clk->mixClkDiv, clk->auxClkDiv);
       break;
       case MCASP_IOCTL_RX_CLK_CFG:
          clk = (mcasp_clk_t*)data;
          mcasp_rx_clk_cfg(sc, clk->clkSrc,
                          clk->mixClkDiv, clk->auxClkDiv);
       break;
       case MCASP_IOCTL_TX_CLK_POLARITY_SET:
          val = *(uint32_t*)data;
          mcasp_tx_clk_polarity_set(sc, val);
       break;
       case MCASP_IOCTL_RX_CLK_POLARITY_SET:
          val = *(uint32_t*)data;
          mcasp_rx_clk_polarity_set(sc, val);
       break;
       case MCASP_IOCTL_TX_HF_CLK_POLARITY_SET:
          val = *(uint32_t*)data;
          mcasp_tx_hf_clk_polarity_set(sc, val);
       break;
       case MCASP_IOCTL_RX_HF_CLK_POLARITY_SET:
          val = *(uint32_t*)data;
          mcasp_rx_hf_clk_polarity_set(sc, val);
       break;
       case MCASP_IOCTL_TX_RX_CLK_SYNC_ENABLE:
          mcasp_tx_rx_clk_sync_enable(sc);
       break;
       case MCASP_IOCTL_TX_RX_CLK_SYNC_DISABLE:
          mcasp_tx_rx_clk_sync_disable(sc);
       break;
       case MCASP_IOCTL_SERIALIZER_TX_SET:
          val = *(uint32_t*)data;
          mcasp_serializer_tx_set(sc, val);
       break;
       case MCASP_IOCTL_SERIALIZER_RX_SET:
          val = *(uint32_t*)data;
          mcasp_serializer_rx_set(sc, val);
       break;
       case MCASP_IOCTL_SERIALIZER_INACTIVATE:
          val = *(uint32_t*)data;
          mcasp_serializer_inactivate(sc, val);
       break;
       case MCASP_IOCTL_PIN_DIR_OUTPUT_SET:
          val = *(uint32_t*)data;
          mcasp_pin_dir_output_set(sc, val);
       break;
       case MCASP_IOCTL_PIN_DIR_INPUT_SET:
          val = *(uint32_t*)data;
          mcasp_pin_dir_input_set(sc, val);
       break;
       case MCASP_IOCTL_PIN_MCASP_SET:
          val = *(uint32_t*)data;
          mcasp_pin_mcasp_set(sc, val);
       break;
       case MCASP_IOCTL_PIN_GPIO_SET:
          val = *(uint32_t*)data;
          mcasp_pin_gpio_set(sc, val);
       break;
       case MCASP_IOCTL_TX_TIME_SLOT_SET:
          val = *(uint32_t*)data;
          mcasp_tx_time_slot_set(sc, val);
       break;
       case MCASP_IOCTL_RX_TIME_SLOT_SET:
          val = *(uint32_t*)data;
          mcasp_rx_time_slot_set(sc, val);
       break;
       case MCASP_IOCTL_TX_INT_DISABLE:
          val = *(uint32_t*)data;
          mcasp_tx_int_disable(sc, val);
       break;
       case MCASP_IOCTL_RX_INT_DISABLE:
          val = *(uint32_t*)data;
          mcasp_rx_int_disable(sc, val);
       break;
       case MCASP_IOCTL_TX_INT_ENABLE:
          val = *(uint32_t*)data;
          mcasp_tx_int_enable(sc, val);
       break;
       case MCASP_IOCTL_RX_INT_ENABLE:
          val = *(uint32_t*)data;
          mcasp_rx_int_enable(sc, val);
       break;
       case MCASP_IOCTL_TX_CLK_START:
          val = *(uint32_t*)data;
          mcasp_tx_clk_start(sc, val);
       break;
       case MCASP_IOCTL_RX_CLK_START:
          val = *(uint32_t*)data;
          mcasp_rx_clk_start(sc, val);
       break;
       case MCASP_IOCTL_TX_SER_ACTIVATE:
          mcasp_tx_ser_activate(sc);
       break;
       case MCASP_IOCTL_RX_SER_ACTIVATE:
          mcasp_rx_ser_activate(sc);
       break;
       case MCASP_IOCTL_TX_ENABLE:
          mcasp_tx_enable(sc);
       break;
       case MCASP_IOCTL_RX_ENABLE:
          mcasp_rx_enable(sc);
       break;
       case MCASP_IOCTL_A_MUTE_ENABLE:
          mute = (mcasp_a_mute_enable_t*)data;
          mcasp_a_mute_enable(sc, mute->errFlags,
                               mute->pinState);
       break;
       case MCASP_IOCTL_A_MUTE_DISABLE:
          mcasp_a_mute_disable(sc);
       break;
       case MCASP_IOCTL_A_MUTE_IN_ACTIVATE:
          val = *(uint32_t*)data;
          mcasp_a_mute_in_activate(sc, val);
       break;
       case MCASP_IOCTL_TX_CLK_CHECK_CONFIG:
          check = (mcasp_clk_check_t*)data;
          mcasp_tx_clk_check_config(sc, check->clkDiv,
                                  check->boundMin, 
                                  check->boundMax);
       break;
       case MCASP_IOCTL_RX_CLK_CHECK_CONFIG:
          check = (mcasp_clk_check_t*)data;
          mcasp_rx_clk_check_config(sc, check->clkDiv,
                                  check->boundMin, 
                                  check->boundMax);
       break;
       case MCASP_IOCTL_TX_BUF_WRITE:
          buf = (mcasp_buf_t*)data;
          mcasp_tx_buf_write(sc, buf->serNum,
                     buf->data);
       break;
       case MCASP_IOCTL_DIT_ENABLE:
       case MCASP_IOCTL_DIT_DISABLE:
       case MCASP_IOCTL_DIT_CHAN_STAT_WRITE:
       case MCASP_IOCTL_DIT_CHAN_USR_DATA_WRITE:
       case MCASP_IOCTL_DIT_CHAN_STAT_READ:
       case MCASP_IOCTL_DIT_CHAN_USR_DATA_READ:
       case MCASP_IOCTL_RX_BUF_READ:
          buf = (mcasp_buf_t*)data;
          buf->data = mcasp_rx_buf_read(sc, buf->serNum);
       break;
       case MCASP_IOCTL_TX_STATUS_GET:
          *(uint32_t*)data = mcasp_tx_status_get(sc);
       case MCASP_IOCTL_RX_STATUS_GET:
          *(uint32_t*)data = mcasp_rx_status_get(sc);
       case MCASP_IOCTL_CONTEXT_SAVECTRL:
       case MCASP_IOCTL_CONTEXT_RESTORECTRL:
       break;

    }
    return 0;
}

