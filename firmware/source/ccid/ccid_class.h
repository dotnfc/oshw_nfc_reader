/**
  **************************************************************************
  * @file     ccid_class.h
  * @version  v2.0.0
  * @date     2020-11-02
  * @brief    usb ccid class file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

 /* define to prevent recursive inclusion -------------------------------------*/
#ifndef __CCID_CLASS_H
#define __CCID_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_std.h"
#include "usbd_core.h"

/** @addtogroup USB_ccid_class
  * @{
  */

/** @defgroup USB_ccid_class_definition
  * @{
  */
/* request command*/
#define CCID_REQ_ABORT                  0x01
#define CCID_REQ_GET_CLOCK_FREQUENCIES  0x02
#define CCID_REQ_GET_DATA_RATES         0x03

#define TPDU_EXCHANGE                  0x01
#define SHORT_APDU_EXCHANGE            0x02
#define EXTENDED_APDU_EXCHANGE         0x04
#define CHARACTER_EXCHANGE             0x00

#define EXCHANGE_LEVEL_FEATURE         TPDU_EXCHANGE



#define SMARTCARD_SIZ_CONFIG_DESC      93 
/**
  * @}
  */

/** @defgroup USB_cdc_class_exported_types
  * @{
  */

/**
  * @brief usb cdc class struct
  */
typedef struct
{
  uint32_t alt_setting;
  uint32_t ccid_ept0_buff[16] ;
}ccid_struct_type;


/**
  * @}
  */

/** @defgroup USB_cdc_class_exported_functions
  * @{
  */
extern usbd_class_handler ccid_class_handler;
uint16_t usb_vcp_get_rxdata(void *udev, uint8_t *recv_data);
error_status usb_vcp_send_data(void *udev, uint8_t *send_data, uint16_t len);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif



