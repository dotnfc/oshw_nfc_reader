/**
  **************************************************************************
  * @file     ccid_desc.h
  * @version  v2.0.0
  * @date     2020-11-02
  * @brief    usb ccid descriptor header file
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
#ifndef __CCID_DESC_H
#define __CCID_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ccid_class.h"
#include "usbd_core.h"

/** @addtogroup USB_ccid_desc
  * @{
  */

/** @defgroup USB_ccid_desc_definition
  * @{
  */
/**
  * @brief usb bcd number define
  */
#define CCID_BCD_NUM                      0x0110

/**
  * @brief usb vendor id and product id define
  */
#define USBD_CCID_VENDOR_ID               0x2E3C
#define USBD_CCID_PRODUCT_ID              0x3233

/**
  * @brief usb descriptor size define
  */
#define USBD_CCID_CONFIG_DESC_SIZE        93
#define USBD_CCID_SIZ_STRING_LANGID       4
#define USBD_CCID_SIZ_STRING_SERIAL       0x1A

/**
  * @brief usb string define(vendor, product configuration, interface)
  */
#define USBD_CCID_DESC_MANUFACTURER_STRING    "Artery"
#define USBD_CCID_DESC_PRODUCT_STRING         "AT32 Smart Card"
#define USBD_CCID_DESC_CONFIGURATION_STRING   "Smart Card Config"
#define USBD_CCID_DESC_INTERFACE_STRING       "Smart Card Interface"

/**
  * @brief usb endpoint interval define
  */
#define CCID_HID_BINTERVAL_TIME                0xFF

/**
  * @brief usb mcu id address deine
  */
#define         MCU_ID1                   (0x1FFFF7E8)
#define         MCU_ID2                   (0x1FFFF7EC)
#define         MCU_ID3                   (0x1FFFF7F0)
/**
  * @}
  */

extern usbd_desc_handler ccid_desc_handler;


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
