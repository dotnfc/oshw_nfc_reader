/**
  **************************************************************************
  * @file     ccid_class.c
  * @version  v2.0.0
  * @date     2020-11-02
  * @brief    usb ccid class type
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
#include "usbd_core.h"
#include "ccid_class.h"
#include "ccid_desc.h"
#include "ccid_interface.h"
#include "ccid_cmd.h"
#include <string.h>

/** @defgroup USB_ccid_class
  * @brief usb device class ccid demo
  * @{
  */

/** @defgroup USB_ccid_class_private_functions
  * @{
  */

static usb_sts_type class_init_handler(void *udev);
static usb_sts_type class_clear_handler(void *udev);
static usb_sts_type class_setup_handler(void *udev, usb_setup_type *setup);
static usb_sts_type class_ept0_tx_handler(void *udev);
static usb_sts_type class_ept0_rx_handler(void *udev);
static usb_sts_type class_in_handler(void *udev, uint8_t ept_num);
static usb_sts_type class_out_handler(void *udev, uint8_t ept_num);
static usb_sts_type class_sof_handler(void *udev);
static usb_sts_type class_event_handler(void *udev, usbd_event_type event);

/* ccid data struct */
ccid_struct_type ccid_struct;

/* usb device class handler */
usbd_class_handler ccid_class_handler =
{
  class_init_handler,
  class_clear_handler,
  class_setup_handler,
  class_ept0_tx_handler,
  class_ept0_rx_handler,
  class_in_handler,
  class_out_handler,
  class_sof_handler,
  class_event_handler,
  &ccid_struct
};

const uint8_t clock_frequencies[4] = { 0xa0, 0x0f, 0x00, 0x00 };
const uint8_t ccid_data_rates[] = { 0x00, 0x2A, 0x00, 0x00 };
static uint8_t ccid_get_clock_frequencies(usb_setup_type *setup, uint8_t* pbuf, uint16_t* plen);
static uint8_t ccid_get_rats(usb_setup_type *setup, uint8_t* pbuf, uint16_t* plen);

/**
  * @brief  initialize usb endpoint
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_init_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;

  /* open in endpoint */
  usbd_ept_open(pudev, CCID_INTR_IN_EPT, EPT_INT_TYPE, CCID_INTR_EP_MAX_PACKET);

  /* open in endpoint */
  usbd_ept_open(pudev, CCID_BULK_IN_EPT, EPT_BULK_TYPE, CCID_BULK_EP_MAX_PACKET);

  /* open out endpoint */
  usbd_ept_open(pudev, CCID_BULK_OUT_EPT, EPT_BULK_TYPE, CCID_BULK_EP_MAX_PACKET);

  ccid_init(pudev);

  return status;
}

/**
  * @brief  clear endpoint or other state
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_clear_handler(void *udev)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;

  /* close in endpoint */
  usbd_ept_close(pudev, CCID_INTR_IN_EPT);

  /* close in endpoint */
  usbd_ept_close(pudev, CCID_BULK_IN_EPT);

  /* close out endpoint */
  usbd_ept_close(pudev, CCID_BULK_OUT_EPT);
  
  ccid_deinit(pudev);

  return status;
}

/**
  * @brief  usb device class setup request handler
  * @param  udev: to the structure of usbd_core_type
  * @param  setup: setup packet
  * @retval status of usb_sts_type
  */
static usb_sts_type class_setup_handler(void *udev, usb_setup_type *setup)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;
  ccid_struct_type *pccid = (ccid_struct_type *)pudev->class_handler->pdata;
  uint8_t slot_nb;
  uint8_t seq_nb;
  uint16_t len;

  switch(setup->bmRequestType & USB_REQ_TYPE_RESERVED)
  {
    case USB_REQ_TYPE_CLASS:
      switch(setup->bRequest)
      {
        case CCID_REQ_GET_CLOCK_FREQUENCIES :
          if((setup->wValue  == 0) && 
             (setup->wLength != 0) &&
            ((setup->bmRequestType & 0x80) == 0x80))
            {
              if ( ccid_get_clock_frequencies(setup, (uint8_t *)pccid->ccid_ept0_buff, &len) != 0 )
              {
                usbd_ctrl_unsupport(pudev);
                return USB_FAIL;  
              }
              else
              {
                if (len > 64)
                {
                  len = 64;
                }
                usbd_ctrl_send(pudev,(uint8_t *)pccid->ccid_ept0_buff,len);
              }          
          }
          else
          {
             usbd_ctrl_unsupport(pudev);
             return USB_FAIL; 
          }
          break;
        
        case CCID_REQ_ABORT:
          if ((setup->wLength == 0) &&
             ((setup->bmRequestType & 0x80) != 0x80))
          {
            slot_nb = (uint8_t) ((setup->wValue) & 0x00ff);
            seq_nb = (uint8_t) (((setup->wValue) & 0xff00)>>8);
            if(ccid_cmd_abort(slot_nb, seq_nb) != 0)
            {
              usbd_ctrl_unsupport(pudev);
              return USB_FAIL;
            }
          }
          else
          {
            usbd_ctrl_unsupport(pudev);
            return USB_FAIL;
          }
          break;
          
        case CCID_REQ_GET_DATA_RATES:
          if((setup->wValue  == 0) && 
             (setup->wLength != 0) &&
            ((setup->bmRequestType & 0x80) == 0x80))
           {
              if ( ccid_get_rats(setup, (uint8_t *)pccid->ccid_ept0_buff, &len) != 0 )
              {
                usbd_ctrl_unsupport(pudev);
                return USB_FAIL;
              }
              else
              {
                if (len > 64)
                {
                  len = 64;
                }
                usbd_ctrl_send(pudev,(uint8_t *)pccid->ccid_ept0_buff,len);
              } 
          }
          else
          {
            usbd_ctrl_unsupport(pudev);
            return USB_FAIL;
          }
          break;
      
        default:
           usbd_ctrl_unsupport(pudev);
           return USB_FAIL;
          
      }
      break;
    /* standard request */
    case USB_REQ_TYPE_STANDARD:
      switch(setup->bRequest)
      {
        case USB_STD_REQ_GET_DESCRIPTOR:
          usbd_ctrl_unsupport(pudev);
          break;
        case USB_STD_REQ_GET_INTERFACE:
          usbd_ctrl_send(pudev, (uint8_t *)&pccid->alt_setting, 1);
          break;
        case USB_STD_REQ_SET_INTERFACE:
          pccid->alt_setting = setup->wValue;
          break;
        case USB_STD_REQ_CLEAR_FEATURE:
          usbd_ept_close(pudev, setup->wIndex);
          usbd_ept_open(pudev, setup->wIndex, EPT_BULK_TYPE, CCID_BULK_EP_MAX_PACKET);
          break;
        default:
          break;
      }
      break;
    default:
      usbd_ctrl_unsupport(pudev);
      break;
  }
  return status;
}

/**
  * @brief  usb device class endpoint 0 in status stage complete
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_ept0_tx_handler(void *udev)
{
  usb_sts_type status = USB_OK;

  /* ...user code... */

  return status;
}

/**
  * @brief  usb device class endpoint 0 out status stage complete
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_ept0_rx_handler(void *udev)
{
  usb_sts_type status = USB_OK;

  return status;
}

/**
  * @brief  usb device class transmision complete handler
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_num: endpoint number
  * @retval status of usb_sts_type
  */
static usb_sts_type class_in_handler(void *udev, uint8_t ept_num)
{
  usbd_core_type *pudev = (usbd_core_type *)udev;
  usb_sts_type status = USB_OK;

  /* ...user code...
    trans next packet data
  */
  ccid_in_handler(pudev, ept_num);

  return status;
}

/**
  * @brief  usb device class endpoint receive data
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_num: endpoint number
  * @retval status of usb_sts_type
  */
static usb_sts_type class_out_handler(void *udev, uint8_t ept_num)
{
  usb_sts_type status = USB_OK;
  usbd_core_type *pudev = (usbd_core_type *)udev;

  ccid_out_handler(pudev, ept_num);

  return status;
}

/**
  * @brief  usb device class sof handler
  * @param  udev: to the structure of usbd_core_type
  * @retval status of usb_sts_type
  */
static usb_sts_type class_sof_handler(void *udev)
{
  usb_sts_type status = USB_OK;

  /* ...user code... */

  return status;
}

/**
  * @brief  usb device class event handler
  * @param  udev: to the structure of usbd_core_type
  * @param  event: usb device event
  * @retval status of usb_sts_type
  */
static usb_sts_type class_event_handler(void *udev, usbd_event_type event)
{
  usb_sts_type status = USB_OK;
  switch(event)
  {
    case USBD_RESET_EVENT:

      /* ...user code... */

      break;
    case USBD_SUSPEND_EVENT:

      /* ...user code... */

      break;
    case USBD_WAKEUP_EVENT:
      /* ...user code... */

      break;
    default:
      break;
  }
  return status;
}

uint8_t ccid_get_clock_frequencies(usb_setup_type *setup, uint8_t* pbuf, uint16_t* plen)
{
  if (plen != NULL)
  {
    * plen = sizeof(clock_frequencies);
    return 0;
  }
   
  if (pbuf != NULL)
  {
    memcpy(pbuf, (uint8_t *)clock_frequencies, sizeof(clock_frequencies));
  }
  return 0;
} /* end of get_clock_frequencies */

uint8_t ccid_get_rats(usb_setup_type *setup, uint8_t* pbuf, uint16_t* plen)
{
  if (plen != NULL)
  {
    * plen = sizeof(ccid_data_rates);
    return 0;
  }
    
  if (pbuf != NULL)
  {
    memcpy(pbuf, (uint8_t *)ccid_data_rates, sizeof(ccid_data_rates));
  }
        
  return 0;
} /* end of get_rats */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

