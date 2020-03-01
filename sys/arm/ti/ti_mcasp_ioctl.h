#ifndef _IOCTL_MCASP_H_
#define _IOCTL_MCASP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/ioccom.h>

#define MCASP_IOCTL_MAGIC 'A'

#define MCASP_IOCTL_TX_RESET                _IO(MCASP_IOCTL_MAGIC,  1)
#define MCASP_IOCTL_RX_RESET                _IO(MCASP_IOCTL_MAGIC,  2)
typedef struct {
    uint32_t numTxSer;
    uint32_t minWdPerSer;
} mcasp_fifo_enable_t;
#define MCASP_IOCTL_WRITE_FIFO_ENABLE       _IOW(MCASP_IOCTL_MAGIC, 3, mcasp_fifo_enable_t)
#define MCASP_IOCTL_READ_FIFO_ENABLE        _IOW(MCASP_IOCTL_MAGIC, 4, mcasp_fifo_enable_t)
#define MCASP_IOCTL_TX_FMT_MASK_SET         _IOW(MCASP_IOCTL_MAGIC, 5, uint32_t)
#define MCASP_IOCTL_RX_FMT_MASK_SET         _IOW(MCASP_IOCTL_MAGIC, 6, uint32_t)
#define MCASP_IOCTL_TX_FMT_SET              _IOW(MCASP_IOCTL_MAGIC, 7, uint32_t)
#define MCASP_IOCTL_RX_FMT_SET              _IOW(MCASP_IOCTL_MAGIC, 8, uint32_t)
typedef struct {
    uint32_t wordSize;
    uint32_t slotSize;
    uint32_t txMode;
} mcasp_fmt_i2s_t;
#define MCASP_IOCTL_TX_FMT_I2S_SET          _IOW(MCASP_IOCTL_MAGIC,  9, mcasp_fmt_i2s_t)
#define MCASP_IOCTL_RX_FMT_I2S_SET          _IOW(MCASP_IOCTL_MAGIC, 10, mcasp_fmt_i2s_t)
typedef struct {
    uint32_t fsMode;
    uint32_t fsWidth;
    uint32_t fsSetting;
} mcasp_frame_sync_t;
#define MCASP_IOCTL_TX_FRAME_SYNC_CFG       _IOW(MCASP_IOCTL_MAGIC, 11, mcasp_frame_sync_t)
#define MCASP_IOCTL_RX_FRAME_SYNC_CFG       _IOW(MCASP_IOCTL_MAGIC, 12, mcasp_frame_sync_t)
typedef struct {
    uint32_t clkSrc;
    uint32_t mixClkDiv;
    uint32_t auxClkDiv;
} mcasp_clk_t;
#define MCASP_IOCTL_TX_CLK_CFG              _IOW(MCASP_IOCTL_MAGIC, 13, mcasp_clk_t)
#define MCASP_IOCTL_RX_CLK_CFG              _IOW(MCASP_IOCTL_MAGIC, 14, mcasp_clk_t)
#define MCASP_IOCTL_TX_CLK_POLARITY_SET     _IOW(MCASP_IOCTL_MAGIC, 15, uint32_t)
#define MCASP_IOCTL_RX_CLK_POLARITY_SET     _IOW(MCASP_IOCTL_MAGIC, 16, uint32_t)
#define MCASP_IOCTL_TX_HF_CLK_POLARITY_SET  _IOW(MCASP_IOCTL_MAGIC, 17, uint32_t)
#define MCASP_IOCTL_RX_HF_CLK_POLARITY_SET  _IOW(MCASP_IOCTL_MAGIC, 18, uint32_t)
#define MCASP_IOCTL_TX_RX_CLK_SYNC_ENABLE   _IO(MCASP_IOCTL_MAGIC, 19)
#define MCASP_IOCTL_TX_RX_CLK_SYNC_DISABLE  _IO(MCASP_IOCTL_MAGIC, 20)
#define MCASP_IOCTL_SERIALIZER_TX_SET       _IOW(MCASP_IOCTL_MAGIC, 21, uint32_t)
#define MCASP_IOCTL_SERIALIZER_RX_SET       _IOW(MCASP_IOCTL_MAGIC, 22, uint32_t)
#define MCASP_IOCTL_SERIALIZER_INACTIVATE   _IOW(MCASP_IOCTL_MAGIC, 23, uint32_t)
#define MCASP_IOCTL_PIN_DIR_OUTPUT_SET      _IOW(MCASP_IOCTL_MAGIC, 24, uint32_t)
#define MCASP_IOCTL_PIN_DIR_INPUT_SET       _IOW(MCASP_IOCTL_MAGIC, 25, uint32_t)
#define MCASP_IOCTL_PIN_MCASP_IOCTL_SET     _IOW(MCASP_IOCTL_MAGIC, 26, uint32_t)
#define MCASP_IOCTL_PIN_GPIO_SET            _IOW(MCASP_IOCTL_MAGIC, 27, uint32_t)
#define MCASP_IOCTL_TX_TIME_SLOT_SET        _IOW(MCASP_IOCTL_MAGIC, 28, uint32_t)
#define MCASP_IOCTL_RX_TIME_SLOT_SET        _IOW(MCASP_IOCTL_MAGIC, 29, uint32_t)
#define MCASP_IOCTL_TX_INT_DISABLE          _IOW(MCASP_IOCTL_MAGIC, 30, uint32_t)
#define MCASP_IOCTL_RX_INT_DISABLE          _IOW(MCASP_IOCTL_MAGIC, 31, uint32_t)
#define MCASP_IOCTL_TX_INT_ENABLE           _IOW(MCASP_IOCTL_MAGIC, 32, uint32_t)
#define MCASP_IOCTL_RX_INT_ENABLE           _IOW(MCASP_IOCTL_MAGIC, 33, uint32_t)
#define MCASP_IOCTL_TX_CLK_START            _IOW(MCASP_IOCTL_MAGIC, 34, uint32_t)
#define MCASP_IOCTL_RX_CLK_START            _IOW(MCASP_IOCTL_MAGIC, 35, uint32_t)
#define MCASP_IOCTL_TX_SER_ACTIVATE         _IO(MCASP_IOCTL_MAGIC, 36)
#define MCASP_IOCTL_RX_SER_ACTIVATE         _IO(MCASP_IOCTL_MAGIC, 37)
#define MCASP_IOCTL_TX_ENABLE               _IO(MCASP_IOCTL_MAGIC, 38)
#define MCASP_IOCTL_RX_ENABLE               _IO(MCASP_IOCTL_MAGIC, 39)
typedef struct {
    uint32_t errFlags;
    uint32_t pinState;
} mcasp_a_mute_enable_t;
#define MCASP_IOCTL_A_MUTE_ENABLE           _IOW(MCASP_IOCTL_MAGIC, 40, mcasp_a_mute_enable_t)
#define MCASP_IOCTL_A_MUTE_DISABLE          _IO(MCASP_IOCTL_MAGIC, 41)
#define MCASP_IOCTL_A_MUTE_IN_ACTIVATE      _IOW(MCASP_IOCTL_MAGIC, 42, uint32_t)
typedef struct {
    uint32_t clkDiv;
    uint8_t boundMin;
    uint8_t boundMax;
} mcasp_clk_check_t;
#define MCASP_IOCTL_TX_CLK_CHECK_CONFIG     _IOW(MCASP_IOCTL_MAGIC, 43, mcasp_clk_check_t)
#define MCASP_IOCTL_RX_CLK_CHECK_CONFIG     _IOW(MCASP_IOCTL_MAGIC, 44, mcasp_clk_check_t)
typedef struct {
    uint32_t serNum;
    uint32_t data;
} mcasp_buf_t;
#define MCASP_IOCTL_TX_BUF_WRITE            _IOW(MCASP_IOCTL_MAGIC, 45, mcasp_buf_t)
#define MCASP_IOCTL_DIT_ENABLE              _IO(MCASP_IOCTL_MAGIC, 46)
#define MCASP_IOCTL_DIT_DISABLE             _IO(MCASP_IOCTL_MAGIC, 47)
#define MCASP_IOCTL_DIT_CHAN_STAT_WRITE     _IO(MCASP_IOCTL_MAGIC, 48)
#define MCASP_IOCTL_DIT_CHAN_USR_DATA_WRITE _IO(MCASP_IOCTL_MAGIC, 49)
#define MCASP_IOCTL_DIT_CHAN_STAT_READ      _IO(MCASP_IOCTL_MAGIC, 50)
#define MCASP_IOCTL_DIT_CHAN_USR_DATA_READ  _IO(MCASP_IOCTL_MAGIC, 51)
#define MCASP_IOCTL_RX_BUF_READ             _IOR(MCASP_IOCTL_MAGIC, 52, mcasp_buf_t)
#define MCASP_IOCTL_TX_STATUS_GET           _IOR(MCASP_IOCTL_MAGIC, 53, uint32_t)
#define MCASP_IOCTL_RX_STATUS_GET           _IOR(MCASP_IOCTL_MAGIC, 54, uint32_t)
#define MCASP_IOCTL_CONTEXT_SAVECTRL        _IO(MCASP_IOCTL_MAGIC, 55)
#define MCASP_IOCTL_CONTEXT_RESTORECTRL     _IO(MCASP_IOCTL_MAGIC, 56)

#ifdef __cplusplus
}
#endif

#endif // _IOCTL_MCASP_H_
