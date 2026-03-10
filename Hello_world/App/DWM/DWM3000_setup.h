#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "deca_device_api.h"

// DWM3000 configuration parameters
#define DWM_UWB_CHANNEL       5
#define DWM_UWB_PREAMBLE_LEN  DWT_PLEN_128
#define DWM_UWB_PAC           DWT_PAC8
#define DWM_UWB_TX_CODE       9
#define DWM_UWB_RX_CODE       9
#define DWM_UWB_DATA_RATE     DWT_BR_6M8
#define DWM_TX_PGDLY          0x34
#define DWM_UWB_SFD_TYPE      DWT_SFD_DW_8
#define DWM_UWB_PHR_MODE      DWT_PHRMODE_STD
#define DWM_UWB_PHR_RATE      DWT_PHRRATE_STD
#define DWM_UWB_SFD_TO        (128 + 1 + 8 - 8)
#define DWM_UWB_STS_MODE      DWT_STS_MODE_OFF
#define DWM_UWB_STS_LENGTH    DWT_STS_LEN_64
#define DWM_UWB_PDOA_MODE     DWT_PDOA_M0

#define TX_RF_PGdly    0x34
#define TX_RF_Power    0xfdfdfdfd
#define TX_RF_PGcount  0

#define DWM_IRQ_MASK  (DWT_INT_RXFCG_BIT_MASK  |  \
                       DWT_INT_RXFCE_BIT_MASK   |  \
                       DWT_INT_RXPHE_BIT_MASK   |  \
                       DWT_INT_RXFSL_BIT_MASK   |  \
                       DWT_INT_RXSTO_BIT_MASK   |  \
                       DWT_INT_RXFTO_BIT_MASK   |  \
                       DWT_INT_TXFRS_BIT_MASK   |  \
                       DWT_INT_SPIRDY_BIT_MASK)


#define DWT_DEVICE_ID 0xDECA0302UL      
#define DWM_PAN_ID 0xDECA

#define DWM_MAX_FRAME_LEN  127

