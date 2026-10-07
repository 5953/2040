#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "tusb_option.h"

#define CFG_TUSB_MCU                OPT_MCU_RP2040
#define CFG_TUSB_OS                 OPT_OS_PICO
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_SPEED              OPT_MODE_FULL_SPEED

#define CFG_TUD_ENABLED             1
#define CFG_TUD_CDC                 1
#define CFG_TUD_MSC                 1
#define CFG_TUD_MSC_EP_BUFSIZE      512
#define CFG_TUD_CDC_RX_BUFSIZE      256
#define CFG_TUD_CDC_TX_BUFSIZE      256

#endif
