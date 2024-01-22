#include "ccid_cmd.h"
#include "ccid_interface.h"

#if 0
ccid_in_data_struct Ccid_resp_buff;

static uint8_t ccid_check_cmd_params (uint32_t param_type);

uint8_t pc_to_rdr_icc_power_on(void)
{
  uint8_t vol, sc_vol = 0, idx, err;
  
  ccid_itf.ccid_in_data.dwLength = 0; 
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_DWLENGTH | \
                                  CHK_PARAM_abRFU2 |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_ABORT );
  if (err != 0) 
  {
    return err;
  }

  vol = ccid_itf.ccid_out_cmd.bData_0;
  if (vol >= VOLTAGE_SELE_1V8)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
    return SLOT_ERROR_BAD_POWERSELECT; 
  }
  
  if ((vol == VOLTAGE_SELE_AUTOMATIC) || 
      (vol == VOLTAGE_SELE_3V))
  {
    sc_vol = SC_VOLTAGE_3V;
  }
  else if (vol == VOLTAGE_SELE_5V)
  {
    sc_vol = SC_VOLTAGE_5V;
  }
  
  sc_response_to_reset(sc_vol);
  
  err = ccid_check_cmd_params(CHK_ACTIVE_STATE);
  if (err != 0)
  {
    if (vol != 0)
    {
      return err;
    }
    else
    {
      if (sc_vol != SC_VOLTAGE_5V)
      {
        sc_vol = SC_VOLTAGE_5V;
        sc_response_to_reset(sc_vol);
   
        err = ccid_check_cmd_params(CHK_ACTIVE_STATE);
        if (err != 0) 
         return err;

      }
      else 
      { 
        ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_INACTIVE;
        return err;
      }
    } 
  }
  
  ccid_itf.ccid_in_data.dwLength = SIZE_OF_ATR;
  ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;

  for (idx = 0; idx < SIZE_OF_ATR ; idx++)
  {
    ccid_itf.ccid_in_data.abData[idx] = sc_struct.art_buf[idx];
  }
  
  return SLOT_NO_ERROR;
}

uint8_t pc_to_rdr_icc_power_off(void)
{
  uint8_t err;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_abRFU3 |\
                                  CHK_PARAM_DWLENGTH );
  if (err != 0) 
  {
    return err;
  }
  
  if (sc_present_detect())
  { 
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_INACTIVE;
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_NO_ICC_PRESENT;
  }
  sc_power_enable(FALSE);
  sc_set_state(SC_POWER_OFF);
  
  return SLOT_NO_ERROR;
}

uint8_t pc_to_rdr_get_slot_status(void)
{
  uint8_t err;

  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_DWLENGTH |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU3 );
  if (err != 0) 
  {
    return err;
  }
  
  ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;
  return SLOT_NO_ERROR;
}


uint8_t pc_to_rdr_xfr_block(void)
{
  uint16_t ex_len;
  
  uint8_t err;

  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU3 |\
                                  CHK_PARAM_ABORT |\
                                  CHK_ACTIVE_STATE );
  if (err != 0) 
    return err;
    
  if (ccid_itf.ccid_out_cmd.dwLength > CCID_ABDATA_SIZE)
  { 
    return SLOT_ERROR_BAD_DWLENGTH;
  }

  ex_len = (ccid_itf.ccid_out_cmd.bData_2 << 8) | 
                    ccid_itf.ccid_out_cmd.bData_1;   
  
  ccid_itf.ccid_in_data.dwLength = (uint16_t)ex_len;  


  err = sc_xfer_block(&ccid_itf.ccid_out_cmd.abData[0], 
                   ccid_itf.ccid_out_cmd.dwLength, 
                   ex_len); 

   if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;
    err = SLOT_NO_ERROR;
  }
  
  return err;
}

uint8_t pc_to_rdr_get_parameters(void)
{
  uint8_t err;

  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_DWLENGTH |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU3 );
  if (err != 0) 
    return err;
  
  ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;

  return SLOT_NO_ERROR;
}


uint8_t pc_to_rdr_reset_parameters(void)
{
  uint8_t err;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_DWLENGTH |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU3 |\
                                  CHK_ACTIVE_STATE);
  if (err != 0) 
    return err;
  
  ccid_itf.ccid_out_cmd.abData[0] = DEFAULT_FIDI;
  ccid_itf.ccid_out_cmd.abData[1] = DEFAULT_T01CONVCHECKSUM;
  ccid_itf.ccid_out_cmd.abData[2] = DEFAULT_EXTRA_GUARDTIME;
  ccid_itf.ccid_out_cmd.abData[3] = DEFAULT_WAITINGINTEGER;
  ccid_itf.ccid_out_cmd.abData[4] = DEFAULT_CLOCKSTOP;
  ccid_itf.ccid_out_cmd.abData[5] = 0x00;
  ccid_itf.ccid_out_cmd.abData[6] = 0x00;
  
  err = sc_set_params((sc_protocol0_type *)(&(ccid_itf.ccid_out_cmd.abData[0])));
  
   if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;
    err = SLOT_NO_ERROR;
  }
  
  return err;
}


uint8_t pc_to_rdr_set_parameters(void)
{
  uint8_t err;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU2 |\
                                  CHK_ACTIVE_STATE);
  if (err != 0) 
    return err;
  
  err = SLOT_NO_ERROR;
  
  if ( (ccid_itf.ccid_out_cmd.dwLength == 5) &&
      (ccid_itf.ccid_out_cmd.bData_0 != 0))
    err = SLOT_ERROR_BAD_PROTOCOLNUM;
  
  if ( (ccid_itf.ccid_out_cmd.dwLength == 7) &&
      (ccid_itf.ccid_out_cmd.bData_0 != 1))
    err = SLOT_ERROR_CMD_NOT_SUPPORTED;
    
  if (ccid_itf.ccid_out_cmd.abData[3] != 0)
    err = SLOT_ERROR_BAD_WAITINGINTEGER;
      
  if (ccid_itf.ccid_out_cmd.abData[4] != DEFAULT_CLOCKSTOP)
     err = SLOT_ERROR_BAD_CLOCKSTOP;
   
  if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
  } 
  
  err = sc_set_params((sc_protocol0_type*)(&(ccid_itf.ccid_out_cmd.abData[0])));
  
   if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;
    err = SLOT_NO_ERROR;
  }
  
  return err;
}


uint8_t pc_to_rdr_escape(void)
{
  uint8_t err;
  uint16_t size;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU3 |\
                                  CHK_PARAM_ABORT |\
                                  CHK_ACTIVE_STATE);
  
  if (err != 0) 
    return err;
  
  err = sc_execute_escape(&ccid_itf.ccid_out_cmd.abData[0],
                           ccid_itf.ccid_out_cmd.dwLength,
                           &ccid_itf.ccid_in_data.abData[0],
                           &size);
  
  ccid_itf.ccid_in_data.dwLength = size;
  
  if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;
  }
  
  return err;
}


uint8_t pc_to_rdr_icc_clock(void)
{
  uint8_t err;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU2 |\
                                  CHK_PARAM_DWLENGTH|\
                                  CHK_ACTIVE_STATE);
  if (err != 0) 
    return err;

  if (ccid_itf.ccid_out_cmd.bData_0 > 1)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
    return SLOT_ERROR_BAD_CLOCKCOMMAND;
  }
  
  err = sc_set_clock(ccid_itf.ccid_out_cmd.bData_0);
  
  if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;
  }
  
  return err;
}


uint8_t pc_to_rdr_abort(void)
{
  uint8_t err;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_abRFU3 |\
                                  CHK_PARAM_DWLENGTH);
  if (err != 0) 
    return err;
  
  ccid_cmd_abort (ccid_itf.ccid_out_cmd.bSlot, ccid_itf.ccid_out_cmd.bSeq); 
  ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;  
  return SLOT_NO_ERROR;
}


uint8_t ccid_cmd_abort(uint8_t slot, uint8_t seq)
{
   if (slot >= CCID_NUMBER_OF_SLOTS)
   {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_NO_ICC_PRESENT; 
      return SLOT_ERROR_BAD_SLOT;
   }
    
  if (ccid_itf.ccid_param.bAbortRequestFlag == 1)
  {
    if (( ccid_itf.ccid_param.bSeq == seq) && (ccid_itf.ccid_param.bSlot == slot))
    {
      ccid_itf.ccid_param.bAbortRequestFlag = 0;
    }
  }
  else
  {
    ccid_itf.ccid_param.bAbortRequestFlag = 1;
    ccid_itf.ccid_param.bSeq = seq ;
    ccid_itf.ccid_param.bSlot = slot;
  }
  
  return 0;
}


uint8_t pc_to_rdr_t0_apdu(void)
{
  uint8_t err;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_DWLENGTH |
                                  CHK_PARAM_ABORT );
  if (err != 0) 
    return err;
  
  if (ccid_itf.ccid_out_cmd.bData_0 > 0x03)
  {
   ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE; 
   return SLOT_ERROR_BAD_BMCHANGES;
  }
  
  err = sc_t0_apdu(ccid_itf.ccid_out_cmd.bData_0, 
                    ccid_itf.ccid_out_cmd.bData_1, 
                    ccid_itf.ccid_out_cmd.bData_2);
  
   if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE; 
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE; 
  }
  
  return err;
}


uint8_t pc_to_rdr_mechanical(void)
{
  uint8_t err;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_abRFU2 |\
                                  CHK_PARAM_DWLENGTH 
                                   );
  if (err != 0) 
    return err;
  
  if (ccid_itf.ccid_out_cmd.bData_0 > 0x05)
  {
   ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE; 
   return SLOT_ERROR_BAD_BFUNCTION_MECHANICAL;
  }
  
  err = sc_mechanical(ccid_itf.ccid_out_cmd.bData_0);
  
   if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE; 
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE; 
  }
  
  return err;
}


uint8_t pc_to_rdr_set_data_rate_clock_frequency(void)
{
  uint8_t err;
  uint32_t frequency, data_rate, temp = 0;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
    CHK_PARAM_CARD_PRESENT |\
      CHK_PARAM_abRFU3);
  if (err != 0) 
    return err;
  
  if (ccid_itf.ccid_out_cmd.dwLength != 0x08)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE; 
    return SLOT_ERROR_BAD_LENTGH;
  } 
  
  temp = (ccid_itf.ccid_out_cmd.abData[0]) & 0x000000FF;
  frequency = temp;
  
  temp = (ccid_itf.ccid_out_cmd.abData[1]) & 0x000000FF;
  frequency |= temp << 8;
  
  temp = (ccid_itf.ccid_out_cmd.abData[2]) & 0x000000FF;
  frequency |= temp << 16;
  
  temp = (ccid_itf.ccid_out_cmd.abData[3]) & 0x000000FF;
  frequency |= temp << 24;
  
  temp = (ccid_itf.ccid_out_cmd.abData[4]) & 0x000000FF;
  data_rate = temp;
  
  temp = (ccid_itf.ccid_out_cmd.abData[5]) & 0x000000FF;
  data_rate |= temp << 8;
  
  temp = (ccid_itf.ccid_out_cmd.abData[6]) & 0x000000FF;
  data_rate |= temp << 16;
  
  temp = (ccid_itf.ccid_out_cmd.abData[7]) & 0x000000FF;
  data_rate |= temp << 24;
  
  err = sc_set_data_rate_clock_frequency(frequency, data_rate);
  ccid_itf.ccid_in_data.bError = err;
  
  if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.dwLength = 0;
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE; 
  }
  else
  {
    ccid_itf.ccid_in_data.dwLength = 8;
    (ccid_itf.ccid_in_data.abData[0]) = frequency & 0x000000FF ;
    (ccid_itf.ccid_in_data.abData[1]) = (frequency & 0x0000FF00) >> 8;
    (ccid_itf.ccid_in_data.abData[2]) = (frequency & 0x00FF0000) >> 16;
    (ccid_itf.ccid_in_data.abData[3]) = (frequency & 0xFF000000) >> 24; 
    (ccid_itf.ccid_in_data.abData[4]) = data_rate & 0x000000FF ;
    (ccid_itf.ccid_in_data.abData[5]) = (data_rate & 0x0000FF00) >> 8;  
    (ccid_itf.ccid_in_data.abData[6]) = (data_rate & 0x00FF0000) >> 16;
    (ccid_itf.ccid_in_data.abData[7]) = (data_rate & 0xFF000000) >> 24; 
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE; 
  }
  
  return err;
}

uint8_t pc_to_rdr_secure(void)
{
  uint8_t err;
  uint8_t b_bwi;
  uint16_t w_level_p;
  uint32_t res_len; 
  
  ccid_itf.ccid_in_data.dwLength = 0;
  
  err = ccid_check_cmd_params(CHK_PARAM_SLOT |\
                                  CHK_PARAM_CARD_PRESENT |\
                                  CHK_PARAM_ABORT );
  
  if (err != 0) 
    return err;
  
  b_bwi = ccid_itf.ccid_out_cmd.bData_0;
  w_level_p = (ccid_itf.ccid_out_cmd.bData_1 + ((uint16_t)ccid_itf.ccid_out_cmd.bData_2<<8));
  
  if ((EXCHANGE_LEVEL_FEATURE == TPDU_EXCHANGE) || 
    (EXCHANGE_LEVEL_FEATURE == SHORT_APDU_EXCHANGE))
  {
    if (w_level_p != 0 )
    {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
      err = SLOT_ERROR_BAD_LEVELPARAMETER;
      return err;
    }
  }

  err = sc_secure(ccid_itf.ccid_out_cmd.dwLength, b_bwi, w_level_p, 
                    &ccid_itf.ccid_out_cmd.abData[0], &res_len);

  ccid_itf.ccid_in_data.dwLength = res_len;
  
  if (err != SLOT_NO_ERROR)
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
  }
  else
  {
    ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_NO_ERROR | BM_ICC_PRESENT_ACTIVE;
  }
  
  return err;
}


void rdr_to_pc_data_block(uint8_t errno)
{
  uint16_t length = CCID_RESPONSE_HEADER_SIZE;
  
  ccid_itf.ccid_in_data.bMessageType = RDR_TO_PC_DATABLOCK; 
  ccid_itf.ccid_in_data.bError = errno; 
  ccid_itf.ccid_in_data.bData = 0;
  
  if(errno == SLOT_NO_ERROR)
  {
    length += ccid_itf.ccid_in_data.dwLength;
  }
  
  send_data_req((uint8_t*)(&ccid_itf.ccid_in_data), length);

}


void rdr_to_pc_slot_status(uint8_t errno)
{
  ccid_itf.ccid_in_data.bMessageType = RDR_TO_PC_SLOTSTATUS; 
  ccid_itf.ccid_in_data.dwLength = 0;
  ccid_itf.ccid_in_data.bError = errno; 
  ccid_itf.ccid_in_data.bData = 0; 
  send_data_req((uint8_t*)(&ccid_itf.ccid_in_data), 
                         LEN_RDR_TO_PC_SLOTSTATUS);

}


void rdr_to_pc_parameters(uint8_t errno)
{
  uint16_t length = CCID_RESPONSE_HEADER_SIZE;
  
  ccid_itf.ccid_in_data.bMessageType = RDR_TO_PC_PARAMETERS; 
  ccid_itf.ccid_in_data.bError = errno; 
  
  if(errno == SLOT_NO_ERROR)
  { 
   ccid_itf.ccid_in_data.dwLength = LEN_PROTOCOL_STRUCT_T0;
   length += LEN_PROTOCOL_STRUCT_T0;
  }
  else
  {
   ccid_itf.ccid_in_data.dwLength = 0;
  }
    
  ccid_itf.ccid_in_data.abData[0] = sc_struct.protocol0.bmFindexDindex ;
  ccid_itf.ccid_in_data.abData[1] = sc_struct.protocol0.bmTCCKST0;
  ccid_itf.ccid_in_data.abData[2] = sc_struct.protocol0.bGuardTimeT0;
  ccid_itf.ccid_in_data.abData[3] = sc_struct.protocol0.bWaitingIntegerT0;
  ccid_itf.ccid_in_data.abData[4] = sc_struct.protocol0.bClockStop;
  ccid_itf.ccid_in_data.bData = BPROTOCOL_NUM_T0; 
  
  send_data_req((uint8_t*)(&ccid_itf.ccid_in_data), length);
}


void rdr_to_pc_escape(uint8_t errno)
{
  uint16_t length = CCID_RESPONSE_HEADER_SIZE;
  
  ccid_itf.ccid_in_data.bMessageType = RDR_TO_PC_ESCAPE; 
  ccid_itf.ccid_in_data.bData = 0;
  ccid_itf.ccid_in_data.bError = errno; 
  
  if(errno == SLOT_NO_ERROR)
  {
    length += ccid_itf.ccid_in_data.dwLength;
  }  
  
  send_data_req((uint8_t*)(&ccid_itf.ccid_in_data), length);
}

void rdr_to_pc_data_rate_clock_frequency(uint8_t errno)
{
  uint16_t length = CCID_RESPONSE_HEADER_SIZE;
  
  ccid_itf.ccid_in_data.bMessageType = RDR_TO_PC_DATARATEANDCLOCKFREQUENCY; 
  ccid_itf.ccid_in_data.bError = errno; 
  ccid_itf.ccid_in_data.bData = 0;
  
  if(errno == SLOT_NO_ERROR)
  {
    length += ccid_itf.ccid_in_data.dwLength;
  }  
  
  send_data_req((uint8_t*)(&ccid_itf.ccid_in_data), length);
}

void rdr_to_pc_notify_slot_change(void)
{
  uint8_t *pdata = (uint8_t *)ccid_itf.int_data;
  pdata[OFFSET_INT_BMESSAGETYPE] = RDR_TO_PC_NOTIFYSLOTCHANGE;
  
  if (sc_present_detect())
  {	
    pdata[OFFSET_INT_BMSLOTICCSTATE] = SLOT_ICC_PRESENT | SLOT_ICC_CHANGE;	
  }
  else
  {	
    pdata[OFFSET_INT_BMSLOTICCSTATE] = SLOT_ICC_CHANGE;
    
    sc_power_enable(FALSE);
    sc_set_state(SC_POWER_OFF);
  }
}

void ccid_upd_slot_status(uint8_t status)
{
   ccid_itf.slot_status.status = status;
}

void ccid_upd_slot_change(uint8_t status)
{
   ccid_itf.slot_status.change = status;
}

uint8_t ccid_is_slot_status_change(void)
{
  return ccid_itf.slot_status.change;
}

static uint8_t ccid_check_cmd_params (uint32_t param_type)
{
  uint32_t parameter;
  
  ccid_itf.ccid_in_data.bStatus = BM_ICC_PRESENT_ACTIVE | BM_COMMAND_STATUS_NO_ERROR ;
  
  parameter = (uint32_t)param_type;
  
  if (parameter & CHK_PARAM_SLOT)
  {
    if (ccid_itf.ccid_out_cmd.bSlot >= CCID_NUMBER_OF_SLOTS)
    {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_NO_ICC_PRESENT;
      return SLOT_ERROR_BAD_SLOT;
    }
  }
  
  if (parameter & CHK_PARAM_CARD_PRESENT)
  { 
    if (sc_present_detect() == 0)
    {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_NO_ICC_PRESENT;
      return SLOT_ERROR_ICC_MUTE;
    }
  }
  
  if (parameter & CHK_PARAM_DWLENGTH)
  {
    if (ccid_itf.ccid_out_cmd.dwLength != 0)
    {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
      return SLOT_ERROR_BAD_LENTGH;
    } 
  }
  
  if (parameter & CHK_PARAM_abRFU2)
  {
    
    if ((ccid_itf.ccid_out_cmd.bData_1 != 0) ||
        (ccid_itf.ccid_out_cmd.bData_2 != 0))
    {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
      return SLOT_ERROR_BAD_ABRFU_2B;
    }
  }
  
  if (parameter & CHK_PARAM_abRFU3)
  {
    if ((ccid_itf.ccid_out_cmd.bData_0 != 0) || 
       (ccid_itf.ccid_out_cmd.bData_1 != 0) ||
       (ccid_itf.ccid_out_cmd.bData_2 != 0))
    {
       ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_ACTIVE;
       return SLOT_ERROR_BAD_ABRFU_3B;    
    }
  }
  
 
  if (parameter & CHK_PARAM_ABORT)
  {
    if( ccid_itf.ccid_param.bAbortRequestFlag )
    {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_INACTIVE;      
      return SLOT_ERROR_CMD_ABORTED; 
    }
  }
  
  if (parameter & CHK_ACTIVE_STATE)
  {
    if (sc_get_state() != SC_ACTIVE_ON_T0)
    {
      ccid_itf.ccid_in_data.bStatus = BM_COMMAND_STATUS_FAILED | BM_ICC_PRESENT_INACTIVE;
      return SLOT_ERROR_HW_ERROR;
    }    
  }
  
  return 0; 
}

#else

#endif // 
