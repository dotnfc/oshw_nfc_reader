#include "ccid_interface.h"
#include "ccid_cmd.h"

ccid_interface ccid_itf;

static void ccid_send_data (usbd_core_type *pudev, 
                              uint8_t* pbuf, 
                              uint16_t len);

void ccid_init(usbd_core_type *pudev)
{  
  ccid_itf.int_complete = 1;
  ccid_itf.slot_status.change  = 1;

  usbd_ept_recv(pudev, 
                CCID_BULK_OUT_EPT, 
                (uint8_t *)&ccid_itf.bulk_rx_data, 
                CCID_BULK_EP_MAX_PACKET);
}

void ccid_deinit (usbd_core_type *pudev)
{
   ccid_itf.state = CCID_STATE_IDLE;
}

void ccid_in_handler (usbd_core_type *pudev, uint8_t ept_num)
{  
  if(ept_num == (CCID_BULK_IN_EPT & 0x7F))
  {
    if(ccid_itf.state == CCID_STATE_SEND_RESPOND)
    {
      usbd_ept_recv(pudev, CCID_BULK_OUT_EPT, 
                    (uint8_t *)&ccid_itf.bulk_rx_data, 
                    CCID_BULK_EP_MAX_PACKET);
      
      ccid_itf.state = CCID_STATE_IDLE;  
    }
  }
  else if (ept_num == (CCID_INTR_IN_EPT & 0x7F))
  {
    ccid_itf.int_complete = 1;
  }
}

void ccid_out_handler (usbd_core_type *pudev, 
                           uint8_t ept_num)
{
  uint16_t length = usbd_get_recv_len(pudev, CCID_BULK_OUT_EPT);
   
  switch(ccid_itf.state)
  {
    case CCID_STATE_IDLE:
      if(length == 0x00)
      {
        ccid_itf.state = CCID_STATE_IDLE;
      }
      else if(length >= CCID_MESSAGE_HEADER_SIZE)
      {
        ccid_itf.data_len = length;
        ccid_itf.p_rx_data = (uint8_t*)&ccid_itf.ccid_out_cmd;
        ccid_itf.ccid_in_data.bSlot = ccid_itf.ccid_out_cmd.bSlot;
        ccid_itf.ccid_in_data.bSeq = ccid_itf.ccid_out_cmd.bSeq;
        ccid_copy_cmd_data(ccid_itf.p_rx_data, length); 
        
        if(length >= CCID_BULK_EP_MAX_PACKET)
        {
          if (ccid_itf.ccid_out_cmd.dwLength > CCID_ABDATA_SIZE)
          {
            ccid_itf.state = CCID_STATE_ERROR;
          }
          else 
          { 
            ccid_itf.state = CCID_STATE_RX_DATA;
            ccid_itf.p_rx_data  += length;
            usbd_ept_recv(pudev,CCID_BULK_OUT_EPT, 
                  (uint8_t *)&ccid_itf.bulk_rx_data, 
                  CCID_BULK_EP_MAX_PACKET);
          }
        }
        else
        {
          ccid_command_decode(pudev);
        }
      }
      break;
      
    case CCID_STATE_RX_DATA:
      ccid_itf.data_len += length;
      if(length < CCID_BULK_EP_MAX_PACKET)
      {
        ccid_copy_cmd_data(ccid_itf.p_rx_data, length);
        ccid_command_decode(pudev); 
      }
      else if(length == CCID_BULK_EP_MAX_PACKET)
      {  
        if(ccid_itf.data_len < (ccid_itf.ccid_out_cmd.dwLength + CCID_CMD_HEADER_SIZE))
        {
          ccid_copy_cmd_data(ccid_itf.p_rx_data , length);
          ccid_itf.p_rx_data  += length; 
          
          usbd_ept_recv(pudev, CCID_BULK_OUT_EPT, 
                  (uint8_t *)&ccid_itf.bulk_rx_data, 
                  CCID_BULK_EP_MAX_PACKET);
        }
        else if(ccid_itf.data_len == (ccid_itf.ccid_out_cmd.dwLength + CCID_CMD_HEADER_SIZE))
        { 
          ccid_copy_cmd_data(ccid_itf.p_rx_data , length);
          ccid_command_decode(pudev);
        }
        else
        {
          ccid_itf.state = CCID_STATE_ERROR;
        }
      }
      break;
    
    case CCID_STATE_ERROR:
      ccid_itf.state = CCID_STATE_IDLE;
      break;
    
    default:
      break;
  }
}

void ccid_command_decode(usbd_core_type *pudev)
{
  uint8_t errno;
  
  switch (ccid_itf.ccid_out_cmd.bMessageType)
  {
  case PC_TO_RDR_ICCPOWERON:
    errno = pc_to_rdr_icc_power_on();
    rdr_to_pc_data_block(errno);
    break;
  case PC_TO_RDR_ICCPOWEROFF:
    errno = pc_to_rdr_icc_power_off();
    rdr_to_pc_slot_status(errno);
    break;
  case PC_TO_RDR_GETSLOTSTATUS:
    errno = pc_to_rdr_get_slot_status();
    rdr_to_pc_slot_status(errno);
    break;
  case PC_TO_RDR_XFRBLOCK:
    errno = pc_to_rdr_xfr_block();
    rdr_to_pc_data_block(errno);
    break;
  case PC_TO_RDR_GETPARAMETERS:
    errno = pc_to_rdr_get_parameters();
    rdr_to_pc_parameters(errno);
    break;
  case PC_TO_RDR_RESETPARAMETERS:
    errno = pc_to_rdr_reset_parameters();
    rdr_to_pc_parameters(errno);
    break;
  case PC_TO_RDR_SETPARAMETERS:
    errno = pc_to_rdr_set_parameters();
    rdr_to_pc_parameters(errno);
    break;
  case PC_TO_RDR_ESCAPE:
    errno = pc_to_rdr_escape();
    rdr_to_pc_escape(errno);
    break;
  case PC_TO_RDR_ICCCLOCK:
    errno = pc_to_rdr_icc_clock();
    rdr_to_pc_slot_status(errno);
    break;
  case PC_TO_RDR_ABORT:
    errno = pc_to_rdr_abort();
    rdr_to_pc_slot_status(errno);
    break;
  case PC_TO_RDR_T0APDU:
    errno = pc_to_rdr_t0_apdu();
    rdr_to_pc_slot_status(errno);
    break;
  case PC_TO_RDR_MECHANICAL:
    errno = pc_to_rdr_mechanical();
    rdr_to_pc_slot_status(errno);
    break;   
  case PC_TO_RDR_SETDATARATEANDCLOCKFREQUENCY:
    errno = pc_to_rdr_set_data_rate_clock_frequency();
    rdr_to_pc_data_rate_clock_frequency(errno);
    break;
  case PC_TO_RDR_SECURE:
    errno = pc_to_rdr_secure();
    rdr_to_pc_data_block(errno);
    break;
  default:
    rdr_to_pc_slot_status(SLOT_ERROR_CMD_NOT_SUPPORTED);
    break;
  }
  
  if(ccid_itf.state == CCID_STATE_SEND_RESPOND)
  {
    ccid_send_data(pudev, (uint8_t*)&ccid_itf.ccid_in_data, 
                   ccid_itf.ccid_in_data.u16SizeToSend);
  }
}

void send_data_req(uint8_t* dataPointer, uint16_t length)
{
   ccid_itf.ccid_in_data.u16SizeToSend = length;
   ccid_itf.state = CCID_STATE_SEND_RESPOND;    
}   
  
static void  ccid_send_data(usbd_core_type *pudev,
                              uint8_t* buf, 
                              uint16_t length)
{  
  usbd_ept_send(pudev, CCID_BULK_IN_EPT, buf, length);
}

void ccid_state_change_handler(usbd_core_type *pudev)
{
  if(ccid_itf.slot_status.change && ccid_itf.int_complete)
  {
    rdr_to_pc_notify_slot_change();
    ccid_itf.slot_status.change = 0;
    ccid_itf.int_complete = 0;
   
    usbd_ept_send(pudev, CCID_INTR_IN_EPT, 
                 (uint8_t *)ccid_itf.int_data, 2);
  }
}  

void ccid_copy_cmd_data(uint8_t* p_dst, uint8_t length)
{
  uint32_t idx;
  uint8_t *p_rx = (uint8_t *)ccid_itf.bulk_rx_data;

  for (idx = 0; idx < length; idx++)
  {
    *p_dst++ = p_rx[idx];
  }
}

