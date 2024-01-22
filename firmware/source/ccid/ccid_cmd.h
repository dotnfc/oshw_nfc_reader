#ifndef __CCID_CMD_H
#define __CCID_CMD_H
#include "ccid_class.h"

#define  SLOT_NO_ERROR                              0x81
#define  SLOT_ERROR_UNKNOWN                         0x82
#define  SLOT_ERROR_BAD_LENTGH                      0x01
#define  SLOT_ERROR_BAD_SLOT                        0x05
#define  SLOT_ERROR_BAD_POWERSELECT                 0x07
#define  SLOT_ERROR_BAD_PROTOCOLNUM                 0x07
#define  SLOT_ERROR_BAD_CLOCKCOMMAND                0x07
#define  SLOT_ERROR_BAD_ABRFU_3B                    0x07
#define  SLOT_ERROR_BAD_BMCHANGES                   0x07
#define  SLOT_ERROR_BAD_BFUNCTION_MECHANICAL        0x07
#define  SLOT_ERROR_BAD_ABRFU_2B                    0x08
#define  SLOT_ERROR_BAD_LEVELPARAMETER              0x08
#define  SLOT_ERROR_BAD_FIDI                        0x0A
#define  SLOT_ERROR_BAD_T01CONVCHECKSUM             0x0B
#define  SLOT_ERROR_BAD_GUARDTIME                   0x0C
#define  SLOT_ERROR_BAD_WAITINGINTEGER              0x0D
#define  SLOT_ERROR_BAD_CLOCKSTOP                   0x0E
#define  SLOT_ERROR_BAD_IFSC                        0x0F
#define  SLOT_ERROR_BAD_NAD                         0x10
#define  SLOT_ERROR_BAD_DWLENGTH                    0x08
#define  SLOT_ERROR_CMD_ABORTED                     0xFF
#define  SLOT_ERROR_ICC_MUTE                        0xFE
#define  SLOT_ERROR_XFR_PARITY_ERROR                0xFD
#define  SLOT_ERROR_XFR_OVERRUN                     0xFC
#define  SLOT_ERROR_HW_ERROR                        0xFB
#define  SLOT_ERROR_BAD_ATR_TS                      0xF8
#define  SLOT_ERROR_BAD_ATR_TCK                     0xF7
#define  SLOT_ERROR_ICC_PROTOCOL_NOT_SUPPORTED      0xF6
#define  SLOT_ERROR_ICC_CLASS_NOT_SUPPORTED         0xF5
#define  SLOT_ERROR_PROCEDURE_BYTE_CONFLICT         0xF4
#define  SLOT_ERROR_DEACTIVATED_PROTOCOL            0xF3
#define  SLOT_ERROR_BUSY_WITH_AUTO_SEQUENCE         0xF2
#define  SLOT_ERROR_PIN_TIMEOUT                     0xF0
#define  SLOT_ERROR_PIN_CANCELLED                   0xEF
#define  SLOT_ERROR_CMD_SLOT_BUSY                   0xE0
#define  SLOT_ERROR_CMD_NOT_SUPPORTED               0x00


#define  DEFAULT_FIDI                               0x11 
#define  DEFAULT_T01CONVCHECKSUM                    0x00
#define  DEFAULT_EXTRA_GUARDTIME                    0x00
#define  DEFAULT_WAITINGINTEGER                     0x0A
#define  DEFAULT_CLOCKSTOP                          0x00
#define  DEFAULT_IFSC                               0x20
#define  DEFAULT_NAD                                0x00

#define  DEFAULT_DATA_RATE                         0x000025CD
#define  DEFAULT_CLOCK_FREQ                        0x00000E10

#define BM_ICC_PRESENT_ACTIVE                       0x00
#define BM_ICC_PRESENT_INACTIVE                     0x01
#define BM_ICC_NO_ICC_PRESENT                       0x02

#define BM_COMMAND_STATUS_OFFSET                    0x06
#define BM_COMMAND_STATUS_NO_ERROR                  0x00 
#define BM_COMMAND_STATUS_FAILED                    (0x01 << BM_COMMAND_STATUS_OFFSET)
#define BM_COMMAND_STATUS_TIME_EXTN                 (0x02 << BM_COMMAND_STATUS_OFFSET)

#define SIZE_OF_ATR                                 33
#define LEN_RDR_TO_PC_SLOTSTATUS                    10
#define LEN_PROTOCOL_STRUCT_T0                      5

#define BPROTOCOL_NUM_T0                            0
#define BPROTOCOL_NUM_T1                            1

#define HARDWARE_ERROR_CODE_OVERCURRENT             0x01
#define HARDWARE_ERROR_CODE_VOLTAGEERROR            0x02
#define HARDWARE_ERROR_CODE_OVERCURRENT_IT          0x04
#define HARDWARE_ERROR_CODE_VOLTAGEERROR_IT         0x08

typedef enum 
{
  CHK_PARAM_SLOT = 1,
  CHK_PARAM_DWLENGTH = (1<<1),
  CHK_PARAM_abRFU2 = (1<<2), 
  CHK_PARAM_abRFU3 = (1<<3),
  CHK_PARAM_CARD_PRESENT = (1<<4),
  CHK_PARAM_ABORT = (1<<5),
  CHK_ACTIVE_STATE = (1<<6)
} check_param_type;

uint8_t pc_to_rdr_icc_power_on(void);
uint8_t pc_to_rdr_icc_power_off(void);
uint8_t pc_to_rdr_get_slot_status(void);
uint8_t pc_to_rdr_xfr_block(void);
uint8_t pc_to_rdr_get_parameters(void);
uint8_t pc_to_rdr_reset_parameters(void);
uint8_t pc_to_rdr_set_parameters(void);
uint8_t pc_to_rdr_escape(void);
uint8_t pc_to_rdr_icc_clock(void);
uint8_t pc_to_rdr_abort(void);
uint8_t pc_to_rdr_t0_apdu(void);
uint8_t pc_to_rdr_mechanical(void);
uint8_t pc_to_rdr_set_data_rate_clock_frequency(void);
uint8_t pc_to_rdr_secure(void);

void rdr_to_pc_data_block(uint8_t);
void rdr_to_pc_notify_slot_change(void);
void rdr_to_pc_slot_status(uint8_t);
void rdr_to_pc_parameters(uint8_t);
void rdr_to_pc_escape(uint8_t );
void rdr_to_pc_data_rate_clock_frequency(uint8_t);

void ccid_upd_slot_status(uint8_t);
void ccid_upd_slot_change(uint8_t);
uint8_t ccid_is_slot_status_change(void);
uint8_t ccid_cmd_abort(uint8_t slot, uint8_t seq);


#endif

