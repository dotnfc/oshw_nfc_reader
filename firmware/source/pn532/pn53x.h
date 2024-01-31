/*-
 * Free/Libre Near Field Communication (NFC) library
 *
 * Libnfc historical contributors:
 * Copyright (C) 2009      Roel Verdult
 * Copyright (C) 2009-2013 Romuald Conty
 * Copyright (C) 2010-2012 Romain Tartière
 * Copyright (C) 2010-2013 Philippe Teuwen
 * Copyright (C) 2012-2013 Ludovic Rousseau
 * See AUTHORS file for a more comprehensive list of contributors.
 * Additional contributors of this file:
 * Copyright (C) 2020      Adam Laurie
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

/**
 * @file pn53x.h
 * @brief PN531, PN532 and PN533 common functions
 */

#ifndef __NFC_CHIPS_PN53X_H__
#define __NFC_CHIPS_PN53X_H__

#include "nfc-types.h"
#include "pn53x-internal.h"

#include <stdint.h>
#include <stddef.h>

/* PN532 Hardware Settings */
#define PN532_FIFO_SIZE                     64
#define PN532_ACK_NACK_SIZE                 6
#define PN532_MAX_PAYLAOADSIZE              264
#define PN532_MAX_PACKET_SIZE               (PN532_MAX_PAYLAOADSIZE+11)

/* PN532 Miscellaneous command set */
#define PN532_CMD_Diagnose                  0x00
#define PN532_CMD_GetFirmwareVersion        0x02
#define PN532_CMD_GetGeneralStatus          0x04
#define PN532_CMD_ReadRegister              0x06
#define PN532_CMD_WriteRegister             0x08
#define PN532_CMD_ReadGPIO                  0x0C
#define PN532_CMD_WriteGPIO                 0x0E
#define PN532_CMD_SetSerialBaudRate         0x10
#define PN532_CMD_SetParameters             0x12
#define PN532_CMD_SAMConfiguration          0x14
#define PN532_CMD_PowerDown                 0x16

/* PN532 RF Communication command set */
#define PN532_CMD_RFConfiguration           0x32
#define PN532_CMD_RFRegulationTest          0x58

/* PN532 as Initiator */
#define PN532_CMD_InJumpForDEP              0x56
#define PN532_CMD_InJumpForPSL              0x46
#define PN532_CMD_InListPassiveTarget       0x4A
#define PN532_CMD_InATR                     0x50
#define PN532_CMD_InPSL                     0x4E
#define PN532_CMD_InDataExchange            0x40
#define PN532_CMD_InCommunicateThru         0x42
#define PN532_CMD_InDeselect                0x44
#define PN532_CMD_InRelease                 0x52
#define PN532_CMD_InSelect                  0x54
#define PN532_CMD_InAutoPoll                0x60

/* PN532 as Target */
#define PN532_CMD_TgInitAsTarget            0x8C
#define PN532_CMD_TgSetGeneralBytes         0x92
#define PN532_CMD_TgGetData                 0x86
#define PN532_CMD_TgSetData                 0x8E
#define PN532_CMD_TgSetMetaData             0x94
#define PN532_CMD_TgGetInitiatorCommand     0x88
#define PN532_CMD_TgResponseToInitiator     0x90
#define PN532_CMD_TgGetTargetStatus         0x8A

/* PN532 SPI Stuff: UM 6.2.5 SPI communication details */
#define PN532_SPI_DATAWRITE         0x01
#define PN532_SPI_STATREAD          0x02
#define PN532_SPI_DATAREAD          0x03

#define PN532_SPI_READY             0x01    // for PN532_SPI_STATREAD

#define PN532_ERROR_OK              0
#define PN532_ERROR_ERR             1
#define PN532_ERROR_INVALID_PARAM   2

/**
 *
 */
void    pn532_enable (void);
void    pn532_disable (void);
int     pn53x_init (void);

/**
 * miscellaneous functions 
 */
bool  pn53x_sam_configuration (uint8_t sam_mode);
bool  pn53x_get_firmware_version (uint8_t *pbBuf, uint8_t *bLen);
void  pn53x_power_down (void);
int   pn53x_transceive(const uint8_t *pbtTx, const size_t szTx, uint8_t *pbtRx, const size_t szRxLen, int timeout);

/**
 * RF configurations
 */
void  pn53x_cfg_rf_field (uint8_t on);
void  pn53x_cfg_various_timings (uint8_t fATR_RES_Timeout, uint8_t fRetryTimeout);
void  pn53x_cfg_auto_in_layer4 (uint8_t on);
bool  pn53x_set_parameters (uint8_t ui8Value);
int   pn53x_write_register(uint16_t ui16RegisterAddress, uint8_t ui8SymbolMask, uint8_t ui8Value);

void  pn53x_cfg_max_retries (uint8_t bAtr, uint8_t Psl, uint8_t bPassiveActivation);
void  pn53x_cfg_max_rty_com (uint8_t MaxRtyCOM);
int   pn53x_set_property_bool(const nfc_property property, const bool bEnable);
void  pn53x_reset_settings (void);
bool  pn53x_set_parameters (uint8_t ui8Value);

/**
 * RF test functions
 */
void  pn53x_test_regualtion (uint8_t txMode);
void  pn53x_test_diagnose (void);
void  pn53x_current_target_new (const nfc_target_t *pnt);

int   pn53x_initiator_target_is_present (const nfc_target_p pnt);

int   pn53x_poll (nfc_target_p p_nfc_tgt);
int   pn53x_poll_type_a (nfc_target_p p_nfc_tgt);
int   pn53x_poll_type_b (nfc_target_p p_nfc_tgt);
int   pn53x_poll_type_f (nfc_target_p p_nfc_tgt);

uint8_t* pn53x_in_data_exchange_ex (const uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t *out_len, uint8_t *status);
                         
int pn53x_decode_target_data(const uint8_t *pbtRawData, size_t szRawData, nfc_modulation_type nmt, nfc_target_t *pnt);

uint8_t pn53x_in_data_xchg (uint8_t bTag, uint8_t *pbDataOut, uint8_t bDataOutLen, 
                        uint8_t *pbDataIn, uint8_t *pbDataInLen	);
uint8_t pn53x_in_data_xchg_tcl (uint8_t bTag, uint8_t *pbDataOut, uint8_t bDataOutLen, 
                        uint8_t *pbDataIn, uint8_t *pbDataInLen, uint16_t *pnSw12);

int pn53x_tg_get_data (uint8_t *data, int *len, uint8_t *status);

int pn53x_tg_set_data (const uint8_t *data, int len, uint8_t *status);

int pn53x_tg_init_as_target(pn53x_target_mode ptm,
                             const uint8_t *mifare_params,
                             const uint8_t *tkt, size_t tkt_len,
                             const uint8_t *felicaparams,
                             const uint8_t *nfcid3t, const uint8_t *gbt, const size_t gbt_len,
                             uint8_t *rx, const size_t rx_len, uint8_t *mode
                            );
                            
int pn53x_tg_get_status (uint8_t *state, uint8_t *Brc);

#endif // __PN532_H__
