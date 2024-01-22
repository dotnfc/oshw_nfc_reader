#ifndef __CCID_INTERFACE_H
#define __CCID_INTERFACE_H
#include "usbd_core.h"
#include "usb_conf.h"


#define BULK_MAX_PACKET_SIZE       0x40
#define INTR_MAX_PACKET_SIZE       0x08
#define CCID_NUMBER_OF_SLOTS       1       

#define VOLTAGE_SELE_AUTOMATIC     0x00
#define VOLTAGE_SELE_3V            0x02
#define VOLTAGE_SELE_5V            0x01
#define VOLTAGE_SELE_1V8           0x03

#define	PC_TO_RDR_ICCPOWERON		   0x62
#define	PC_TO_RDR_ICCPOWEROFF		   0x63
#define	PC_TO_RDR_GETSLOTSTATUS		 0x65
#define	PC_TO_RDR_XFRBLOCK	       0x6F
#define	PC_TO_RDR_GETPARAMETERS		 0x6C
#define	PC_TO_RDR_RESETPARAMETERS	 0x6D
#define	PC_TO_RDR_SETPARAMETERS		 0x61
#define	PC_TO_RDR_ESCAPE		       0x6B
#define	PC_TO_RDR_ICCCLOCK		     0x6E
#define	PC_TO_RDR_T0APDU		       0x6A
#define	PC_TO_RDR_SECURE		       0x69
#define	PC_TO_RDR_MECHANICAL		   0x71
#define	PC_TO_RDR_ABORT			       0x72
#define	PC_TO_RDR_SETDATARATEANDCLOCKFREQUENCY		0x73

#define	RDR_TO_PC_DATABLOCK		     0x80
#define	RDR_TO_PC_SLOTSTATUS		   0x81
#define	RDR_TO_PC_PARAMETERS		   0x82
#define	RDR_TO_PC_ESCAPE		       0x83
#define	RDR_TO_PC_DATARATEANDCLOCKFREQUENCY		0x84
#define	RDR_TO_PC_NOTIFYSLOTCHANGE 0x50
#define	RDR_TO_PC_HARDWAREERROR		 0x51
#define OFFSET_INT_BMESSAGETYPE    0
#define OFFSET_INT_BMSLOTICCSTATE  1
#define SLOT_ICC_PRESENT           0x01  
#define SLOT_ICC_CHANGE            0x02

#define CCID_ABDATA_SIZE                         261
#define CCID_CMD_HEADER_SIZE                     10
#define CCID_MESSAGE_HEADER_SIZE                 10
#define CCID_RESPONSE_HEADER_SIZE                10


typedef enum
{
  CCID_STATE_IDLE,
  CCID_STATE_DATA_OUT,
  CCID_STATE_RX_DATA,
  CCID_STATE_SEND_RESPOND,
  CCID_STATE_DATA_IN,
  CCID_STATE_ERROR
}CCID_STATE;

typedef enum
{
  DIR_IN,
  DIR_OUT,
  BOTH_DIR
}CCID_DIR;

typedef struct 
{ 
  uint8_t bMessageType;
  uint32_t dwLength; 
  uint8_t bSlot;
  uint8_t bSeq;
  uint8_t bData_0;
  uint8_t bData_1;
  uint8_t bData_2;
  uint8_t abData[CCID_ABDATA_SIZE];
} ccid_out_cmd_struct; 

typedef struct 
{ 
  uint8_t bMessageType;
  uint32_t dwLength;
  uint8_t bSlot;
  uint8_t bSeq;
  uint8_t bStatus;
  uint8_t bError;
  uint8_t bData;
  uint8_t abData[CCID_ABDATA_SIZE];
  uint16_t u16SizeToSend; 
} ccid_in_data_struct; 

typedef struct 
{ 
  __IO uint8_t status;
  __IO uint8_t change;
} ccid_slot_status_struct; 

typedef struct 
{ 
  __IO uint8_t bAbortRequestFlag; 
  __IO uint8_t bSeq; 
  __IO uint8_t bSlot;
} ccid_param_struct; 

typedef struct
{
  uint8_t state;
  uint8_t int_complete;
  uint32_t data_len;
  ccid_slot_status_struct slot_status;
  uint8_t *p_rx_data;
  
  uint32_t bulk_rx_data[BULK_MAX_PACKET_SIZE/4 + 1];
  uint32_t int_data[INTR_MAX_PACKET_SIZE/4 + 1];
  
  ccid_in_data_struct ccid_in_data __attribute__ ((aligned (4)));
  ccid_out_cmd_struct ccid_out_cmd;
  ccid_param_struct ccid_param;
}ccid_interface;


extern ccid_interface ccid_itf;

void ccid_in_handler(usbd_core_type *pudev, 
                     uint8_t epnum);
void ccid_out_handler(usbd_core_type *pudev, 
                            uint8_t epnum);
void ccid_copy_cmd_data(uint8_t* pDst, uint8_t u8length);
void ccid_command_decode(usbd_core_type *pudev);
void ccid_state_change_handler(usbd_core_type *pudev);
void ccid_init(usbd_core_type *pudev);
void ccid_deinit(usbd_core_type *pudev);
void send_data_req(uint8_t* dataPointer, uint16_t dataLen);

#endif

