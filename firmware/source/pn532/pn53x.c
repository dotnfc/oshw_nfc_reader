/**
 * SPDX-License-Identifier: MIT License
 * Project: oshw nfc reader
 */

#include <string.h>
#include "pn53x.h"
#include "pn53x_bus.h"
#include "pn53x_error.h"

#define NFC_COMM_PASSIVE 1
#define NFC_COMM_ACTIVE 2

#define NFC_DEP_106 0
#define NFC_DEP_212 1
#define NFC_DEP_424 2

#define NFC_BR_106A 0
#define NFC_BR_212F 1
#define NFC_BR_424F 2
#define NFC_BR_106B 3
#define NFC_BR_106J 4

#define PN532_DEFAULT_TIMEOUT 2000

const uint8_t PN532_ACK_FRAME[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

typedef struct _nfc_buffer_t
{
    uint16_t length;
    uint8_t buf[PN53x_NORMAL_FRAME__DATA_MAX_LEN];
} nfc_buffer_t;
// typedef nfc_buffer_t* nfc_buffer_p;

static int pn53x_transmit(const uint8_t *command, int16_t command_length,
                          uint8_t *response, int16_t *response_length, uint16_t timeout);
static bool pn53x_spi_send_frame(const uint8_t *buf, int16_t len);
static bool pn53x_spi_recv_frame(uint8_t *rx_buf, int16_t *rx_len, uint16_t timeout);
static int pn53x_in_communicate_thru(const uint8_t *tbuf, int16_t tlen);
static int pn53x_in_list_passive_target(uint8_t br_ty, const uint8_t *cbuf, int16_t clen, int16_t *rlen);
static bool pn53x_check_ack_frame(void);

/** pn532 frame buffer */
static uint8_t bus_buf[PN53x_EXTENDED_FRAME__DATA_MAX_LEN];     // bus layer
static uint8_t pn532_trx_buf[PN53x_NORMAL_FRAME__DATA_MAX_LEN]; // api layer
static nfc_buffer_t pn532_tx_buf_t, pn532_rx_buf_t;

/* @brief Enable the Pn532 module.
 *
 * see {pn53x_disable}
 */
int pn53x_init(void)
{
    pn532_tx_buf_t.length = 0;
    pn532_rx_buf_t.length = 0;

    pn53x_bus_init();
    return NFC_SUCCESS;
}

/**
 * @brief send pn532 command & data, and receive response
 *
 * @param command command buffer
 * @param command_length, buffer length
 * @param response response buffer
 * @param response_length, response buffer size and received length
 * @param timeout in ms
 * @return NFC_XXXX code
 */
int pn53x_transmit(const uint8_t *command, int16_t command_length,
                   uint8_t *response, int16_t *response_length,
                   uint16_t timeout)
{
    // send command frame first
    if (!pn53x_spi_send_frame(command, command_length))
    {
        return NFC_ETIMEOUT;
    }

    // wait for ACK frame
    if (!pn53x_bus_wait(PN532_DEFAULT_TIMEOUT))
    {
        return NFC_ETIMEOUT;
    }

    // check ack frame
    if (!pn53x_check_ack_frame())
    {
        return NFC_ETIMEOUT;
    }

    // wait for response frame: WTX may occured many times, so timeout should be large enough
    if (!pn53x_bus_wait(timeout))
    {
        return NFC_ETIMEOUT;
    }

    if (pn53x_spi_recv_frame(response, response_length, timeout))
    {
        return NFC_SUCCESS;
    }
    else
    {
        return NFC_EBUS_TRANS;
    }
}

/**
 * @brief send frame to pn532
 *  e.g. 01  00 00 FF  02 FE D4 02 2A 00
 *
 * @param tx_buf user buffer to send
 * @param tx_len buffer length
 * @return true for success, false otherwise
 */
bool pn53x_spi_send_frame(const uint8_t *tx_buf, int16_t tx_len)
{
    int32_t result;
    bool ret;
    uint8_t tlen;
    uint8_t tfi = FD_HOST_TO_CLFE;
    uint8_t *pl = &bus_buf[7];

    if (tx_len > PN53x_NORMAL_FRAME__DATA_MAX_LEN)
    {
        return false; // too many data to send using a 'Normal information frame'
    }
    bus_buf[0] = PN532_SPI_DATAWRITE;
    bus_buf[1] = 0x00; // Preamble
    bus_buf[2] = 0x00; //
    bus_buf[3] = 0xFF; // Start of Packet Code

    tlen = tx_len + 1;         // with TFI
    bus_buf[4] = tlen;         // Packet Length
    bus_buf[5] = 0x100 - tlen; // Packet Length Checksum
    bus_buf[6] = tfi;          // TFI: Host -> PN532

    // Packet Data
    tlen--;
    while (tlen > 0)
    {
        tfi += *tx_buf;
        *pl++ = *tx_buf++;
        tlen--;
    }

    *pl++ = 0x100 - tfi; // PCS (Packet Data Checksum)
    *pl++ = 0x00;        // Postamble
    
    pn53x_bus_begin_transmission();
    result = pn53x_bus_send(bus_buf, 1 + tx_len + PN53x_NORMAL_FRAME__OVERHEAD, PN532_DEFAULT_TIMEOUT);
    ret = (result == PN532_BUS_ERR_SUCCESS);
    pn53x_bus_end_transmission();

    return ret;
}

bool pn53x_check_ack_frame(void)
{
    pn53x_bus_begin_transmission();

    // receive ACK frame
    pn53x_spi_trx_byte(PN532_SPI_DATAREAD);
    if (pn53x_bus_recv(bus_buf, sizeof(PN532_ACK_FRAME), PN532_DEFAULT_TIMEOUT) != PN532_BUS_ERR_SUCCESS)
    {
        pn53x_bus_end_transmission();
        return false;
    }
    pn53x_bus_end_transmission();
    // verify ACK
    if (memcmp(bus_buf, PN532_ACK_FRAME, sizeof(PN532_ACK_FRAME)) != 0)
    {
        pn53x_bus_end_transmission();
        return false;
    }
    pn53x_bus_end_transmission();
    return true;
}

/**
 * @brief recevie pn532 response frame using spi
 *
 * @param rx_buf user buffer
 * @param rx_len buffer length
 * @param timeout in ms
 * @return true for success
 */
bool pn53x_spi_recv_frame(uint8_t *rx_buf, int16_t *rx_len, uint16_t timeout)
{
    int16_t i, rlen;
    uint8_t tfi = FD_CLFE_TO_HOST;

    pn53x_bus_begin_transmission();

    pn53x_spi_trx_byte(PN532_SPI_DATAREAD);
    bus_buf[0] = pn53x_spi_trx_byte(0xff); // Preamble
    bus_buf[1] = pn53x_spi_trx_byte(0xff); //
    bus_buf[2] = pn53x_spi_trx_byte(0xff); // Start of Packet Code

    if ((bus_buf[0] != 0x00) || (bus_buf[1] != 0x00) || (bus_buf[2] != 0xFF))
    {
        pn53x_bus_end_transmission();
        return false; // invalid head
    }
    bus_buf[3] = pn53x_spi_trx_byte(0xff); // Packet Length
    bus_buf[4] = pn53x_spi_trx_byte(0xff); // Packet Length Checksum
    rlen = bus_buf[3] - 1;                 // excluding pdc

    if (bus_buf[3] != (0x100 - bus_buf[4]))
    {
        pn53x_bus_end_transmission();
        return false; // invalid LEN
    }

    bus_buf[5] = pn53x_spi_trx_byte(0xff); // TFI: Host -> PN532
    if (bus_buf[5] != FD_CLFE_TO_HOST)
    {
        pn53x_bus_end_transmission();
        return false; // invlaid direction
    }

    for (i = 0; i < bus_buf[3]; i++)
    {
        bus_buf[6 + i] = pn53x_spi_trx_byte(0xff);
        tfi += bus_buf[6 + i];
    }

    if (tfi != 0)
    {
        pn53x_bus_end_transmission();
        return false; // invalid Packet Data Checksum
    }

    bus_buf[6 + i] = pn53x_spi_trx_byte(0xff); // Postamble
    if (bus_buf[6 + i] != 0x00)
    {
        pn53x_bus_end_transmission();
        return false; // invalid postamble
    }

    if (*rx_len < rlen)
    {
        pn53x_bus_end_transmission();
        // rlen = *rx_len;
        return false; // user buffer is too small
    }
    *rx_len = rlen;
    memcpy(rx_buf, &bus_buf[6], rlen);

    pn53x_bus_end_transmission();
    return true;
}

/**
 * @brief set pn532 chip boolean parameters
 *
 * @param ui8Parameter param to modify
 * @param bEnable enable or not
 * @return NFC_XXX code
 */
int pn53x_set_parameters_bool(const uint8_t ui8Parameter, const bool bEnable)
{
    uint8_t ui8Value = ui8Parameter;

    return pn53x_set_parameters(ui8Value);
}

/**
 * @brief set pn532 chip parameters
 *
 * @param ui8Value values
 * @return true for success
 */
bool pn53x_set_parameters(uint8_t ui8Value)
{
    int rv = 0;
    uint8_t abtCmd[] = {PN532_CMD_SetParameters, ui8Value};

    rv = pn53x_transmit(abtCmd, sizeof(abtCmd), NULL, 0, PN532_DEFAULT_TIMEOUT);
    if (rv >= 0)
        return true;
    else
        return false;
}

/**
 * @brief 写寄存器值
 *
 * @param ui16Reg, REG ID
 * @param ui8Value, new value to set
 * @return boolean
 */
bool pn53x_write_register_llayer(uint16_t ui16Reg, uint8_t ui8Value)
{
    // uint8_t  abtCmd[] = { PN532_CMD_WriteRegister, ui16Reg >> 8, ui16Reg & 0xff, ui8Value };
    uint8_t abtCmd[] = {PN532_CMD_WriteRegister, 0x00, 0x00, 0x00};

    abtCmd[1] = ui16Reg >> 8;
    abtCmd[2] = ui16Reg & 0xff;
    abtCmd[3] = ui8Value;
    if (pn53x_transmit(abtCmd, sizeof(abtCmd), NULL, 0, PN532_DEFAULT_TIMEOUT) < 0)
    {
        return false;
    }

    return true;
}

/**
 * @brief 读取寄存器值
 */
bool pn53x_read_register(uint16_t ui16Reg, uint8_t *ui8Value)
{
    // uint8_t  abtCmd[] = { PN532_CMD_ReadRegister, ui16Reg >> 8, ui16Reg & 0xff };
    uint8_t abtCmd[] = {PN532_CMD_ReadRegister, 0x00, 0x00};
    uint8_t abtRegValue[2];
    int16_t szRegValue = sizeof(abtRegValue);

    abtCmd[1] = ui16Reg >> 8;
    abtCmd[2] = ui16Reg & 0xff;

    if (pn53x_transmit(abtCmd, sizeof(abtCmd), abtRegValue, &szRegValue, PN532_DEFAULT_TIMEOUT))
    {
        return false;
    }

    *ui8Value = abtRegValue[0];

    return true;
}

int pn53x_write_register(uint16_t ui16RegisterAddress, uint8_t ui8SymbolMask, uint8_t ui8Value)
{
    int res = 0;
    uint8_t ui8NewValue = 0;

    if ((ui16RegisterAddress < PN53X_CACHE_REGISTER_MIN_ADDRESS) || (ui16RegisterAddress > PN53X_CACHE_REGISTER_MAX_ADDRESS))
    {
        // Direct write
        if (ui8SymbolMask != 0xff)
        {
            uint8_t ui8CurrentValue;
            if ((res = pn53x_read_register(ui16RegisterAddress, &ui8CurrentValue)) < 0)
                return res;
            ui8NewValue = ((ui8Value & ui8SymbolMask) | (ui8CurrentValue & (~ui8SymbolMask)));
            if (ui8NewValue != ui8CurrentValue)
            {
                return pn53x_write_register_llayer(ui16RegisterAddress, ui8NewValue);
            }
        }
        else
        {
            return pn53x_write_register_llayer(ui16RegisterAddress, ui8Value);
        }
    }
    else
    {
        // Write-back cache area
        // const int internal_address = ui16RegisterAddress - PN53X_CACHE_REGISTER_MIN_ADDRESS;
    }
    return NFC_SUCCESS;
}

int pn53x_set_property_bool(const nfc_property property, const bool bEnable)
{
    uint8_t btValue;
    int ret = NFC_SUCCESS;

    switch (property)
    {
    case NP_HANDLE_CRC:
        // Enable or disable automatic receiving/sending of CRC bytes

        // TX and RX are both represented by the symbol 0x80
        btValue = (bEnable) ? 0x80 : 0x00;
        if ((ret = pn53x_write_register(PN53X_REG_CIU_TxMode, SYMBOL_TX_CRC_ENABLE, btValue)) < 0)
            return ret;
        if ((ret = pn53x_write_register(PN53X_REG_CIU_RxMode, SYMBOL_RX_CRC_ENABLE, btValue)) < 0)
            return ret;

        break;

    case NP_HANDLE_PARITY:
        // Handle parity bit by PN53X chip or parse it as data bit

        btValue = (bEnable) ? 0x00 : SYMBOL_PARITY_DISABLE;
        if ((ret = pn53x_write_register(PN53X_REG_CIU_ManualRCV, SYMBOL_PARITY_DISABLE, btValue)) < 0)
            return ret;

        break;

    case NP_EASY_FRAMING:

        break;

    case NP_ACTIVATE_FIELD:
        pn53x_cfg_rf_field(bEnable);
        break;

    case NP_ACTIVATE_CRYPTO1:
        btValue = (bEnable) ? SYMBOL_MF_CRYPTO1_ON : 0x00;
        ret = pn53x_write_register(PN53X_REG_CIU_Status2, SYMBOL_MF_CRYPTO1_ON, btValue);
        break;

    case NP_INFINITE_SELECT:
        // TODO Made some research around this point:
        // timings could be tweak better than this, and maybe we can tweak timings
        // to "gain" a sort-of hardware polling (ie. like PN532 does)
        pn53x_cfg_max_retries(
            (bEnable) ? 0xff : 0x00, // MxRtyATR, default: active = 0xff, passive = 0x02
            (bEnable) ? 0xff : 0x01, // MxRtyPSL, default: 0x01
            (bEnable) ? 0xff : 0x02  // MxRtyPassiveActivation, default: 0xff (0x00 leads to problems with PN531)
        );
        break;

    case NP_ACCEPT_INVALID_FRAMES:
        btValue = (bEnable) ? SYMBOL_RX_NO_ERROR : 0x00;
        ret = pn53x_write_register(PN53X_REG_CIU_RxMode, SYMBOL_RX_NO_ERROR, btValue);
        break;

    case NP_ACCEPT_MULTIPLE_FRAMES:
        btValue = (bEnable) ? SYMBOL_RX_MULTIPLE : 0x00;
        ret = pn53x_write_register(PN53X_REG_CIU_RxMode, SYMBOL_RX_MULTIPLE, btValue);
        break;

    case NP_AUTO_ISO14443_4:

        if (pn53x_set_parameters_bool(PARAM_AUTO_RATS, bEnable) == 0)
            break;

    case NP_FORCE_ISO14443_A:
        if (!bEnable)
        {
            return NFC_SUCCESS; // Nothing to do
        }
        // Force pn53x to be in ISO14443-A mode
        if ((ret = pn53x_write_register(PN53X_REG_CIU_TxMode, SYMBOL_TX_FRAMING, 0x00)) < 0)
        {
            return ret;
        }
        if ((ret = pn53x_write_register(PN53X_REG_CIU_RxMode, SYMBOL_RX_FRAMING, 0x00)) < 0)
        {
            return ret;
        }
        // Set the PN53X to force 100% ASK Modified miller decoding (default for 14443A cards)
        ret = pn53x_write_register(PN53X_REG_CIU_TxAuto, SYMBOL_FORCE_100_ASK, 0x40);
        break;

    case NP_FORCE_ISO14443_B:
        if (!bEnable)
        {
            return NFC_SUCCESS; // Nothing to do
        }
        // Force pn53x to be in ISO14443-B mode
        if ((ret = pn53x_write_register(PN53X_REG_CIU_TxMode, SYMBOL_TX_FRAMING, 0x03)) < 0)
        {
            return ret;
        }
        ret = pn53x_write_register(PN53X_REG_CIU_RxMode, SYMBOL_RX_FRAMING, 0x03);
        break;

    case NP_FORCE_SPEED_106:
        if (!bEnable)
        {
            return NFC_SUCCESS; // Nothing to do
        }
        // Force pn53x to be at 106 kbps
        if ((ret = pn53x_write_register(PN53X_REG_CIU_TxMode, SYMBOL_TX_SPEED, 0x00)) < 0)
        {
            return ret;
        }
        ret = pn53x_write_register(PN53X_REG_CIU_RxMode, SYMBOL_RX_SPEED, 0x00);
        break;
        // Following properties are invalid (not bool)
    case NP_TIMEOUT_COMMAND:
    case NP_TIMEOUT_ATR:
    case NP_TIMEOUT_COM:
        ret = NFC_EINVARG;
        break;
    }

    return ret;
}

void pn53x_reset_settings(void)
{
    // Reset the ending transmission bits register, it is unknown what the last tranmission used there

    if (pn53x_write_register(PN53X_REG_CIU_BitFraming, SYMBOL_TX_LAST_BITS, 0x00) < 0)
        return;

    // Make sure we reset the CRC and parity to chip handling.
    if (pn53x_set_property_bool(NP_HANDLE_CRC, true) < 0)
        return;

    if (pn53x_set_property_bool(NP_HANDLE_PARITY, true) < 0)
        return;

    // Activate "easy framing" feature by default
    if (pn53x_set_property_bool(NP_EASY_FRAMING, true) < 0)
        return;

    // Deactivate the CRYPTO1 cipher, it may could cause problems when still active
    if (pn53x_set_property_bool(NP_ACTIVATE_CRYPTO1, false) < 0)
        return;
}

/**
 * @brief SAM configuration
 *
 * @param sam_mode - 1 turn on, 0 turn off
 */
bool pn53x_sam_configuration(uint8_t sam_mode)
{
    int rv;
    uint8_t abtCmd[] = {PN532_CMD_SAMConfiguration, 0x00, 0x00, 0x01};
    size_t szCmd = sizeof(abtCmd);

    abtCmd[1] = sam_mode;

    switch (sam_mode)
    {
    case PSM_NORMAL:     // Normal mode
    case PSM_WIRED_CARD: // Wired card mode
        szCmd = 2;
        break;
    case PSM_VIRTUAL_CARD: // Virtual card mode
    case PSM_DUAL_CARD:    // Dual card mode
        // TODO Implement timeout handling
        szCmd = 3;
        break;
    default:
        return false;
    }

    rv = pn53x_transmit(abtCmd, szCmd, NULL, 0, PN532_DEFAULT_TIMEOUT);
    if (rv >= 0)
        return true;
    else
        return false;
}

/**
 * @brief 取 pn532 固件版本
 */
bool pn53x_get_firmware_version(uint8_t *pbBuf, uint8_t *bLen)
{
    const uint8_t abtCmd[] = {PN532_CMD_GetFirmwareVersion};
    uint8_t abtFw[5]; // 03 IC Ver Rev Support
    int16_t szFwLen = sizeof(abtFw);

    if (pn53x_transmit(abtCmd, sizeof(abtCmd), abtFw, &szFwLen, PN532_DEFAULT_TIMEOUT))
    {
        return false;
    }

    if (*bLen > szFwLen)
        *bLen = szFwLen;

    memcpy(pbBuf, abtFw, *bLen);
    return true;
}

/**
 * @brief 将 PN532 (包括非接前端) 下电以省电
 */
void pn53x_power_down(void)
{
    uint8_t abtCmd[] = {PN532_CMD_PowerDown, 0xf0};

    pn53x_transmit(abtCmd, sizeof(abtCmd), NULL, 0, PN532_DEFAULT_TIMEOUT);
}

/**
 * @brief Switch the antenna ON/OFF.
 *
 * @param on - 1 turn on, 0 turn off
 */
void pn53x_cfg_rf_field(uint8_t on)
{
    uint8_t abtCmd[] = {PN532_CMD_RFConfiguration, RFCI_FIELD, 0x00};

    if (on)
    {
        abtCmd[2] = 1; // Auto_RFCA = bit0, RF_on_off = bit1
    }
    pn53x_transmit(abtCmd, sizeof(abtCmd), NULL, 0, PN532_DEFAULT_TIMEOUT);
}

/**
 * @brief 配置不同的超时值
 */
void pn53x_cfg_various_timings(uint8_t fATR_RES_Timeout, uint8_t fRetryTimeout)
{
    uint8_t abtCmd[] = {PN532_CMD_RFConfiguration, RFCI_TIMING, 0x00, 0x00, 0x00};

    abtCmd[2] = 0x00;             // RFU
    abtCmd[3] = fATR_RES_Timeout; // ATR_RES timeout (default: 0x0B 102.4 ms)
    abtCmd[4] = fRetryTimeout;    // TimeOut during non-DEP communications, aka.InCommunicateThru
                                  // (default: 0x0A 51.2 ms)
                                  // This timeout definition is also used with InDataExchange(§7.3.8, p: 127) when
                                  // the target is aFeliCa or a Mifare card (Ultralight, Standard …).

    pn53x_transmit(abtCmd, sizeof(abtCmd), NULL, 0, PN532_DEFAULT_TIMEOUT);
}

/**
 * @brief communication max retry settings
 *
 * @param MaxRtyCOMM max retry count
 * @note
 *      - Defines the retry number that the PN532 will use in the InCommunicateThru.
 *      - This information is also used with InDataExchange when the target is a FeliCa or a Mifare card
 *      - The specific value 0xFF means that the PN532 retries eternally.
 *      - The default value of this parameter is 0x00 (no retry, only one try).
 */
void pn53x_cfg_max_rty_com(uint8_t MaxRtyCOM)
{
    uint8_t abtCmd[] = {PN532_CMD_RFConfiguration, RFCI_RETRY_DATA, MaxRtyCOM};

    pn53x_transmit(abtCmd, sizeof(abtCmd), NULL, 0, PN532_DEFAULT_TIMEOUT);
}

/**
 * @brief selection max retry settings
 *
 * @param bAtr	Times that the PN532 will retry to send the ATR_REQ
 * @param bPsl	Times to send the PSL_REQ, or to send the PPS request.
 * @param bPassive	Times that retry to activate a target in InListPassiveTargetcommand
 */
void pn53x_cfg_max_retries(uint8_t bAtr, uint8_t Psl, uint8_t bPassiveActivation)
{
    uint8_t abtCmd[] = {PN532_CMD_RFConfiguration, RFCI_RETRY_SELECT, 0x00, 0x00, 0x00};

    abtCmd[2] = bAtr;
    abtCmd[3] = Psl;
    abtCmd[4] = bPassiveActivation;

    pn53x_transmit(abtCmd, sizeof(abtCmd), NULL, 0, PN532_DEFAULT_TIMEOUT);
}

/**
 * @brief to detect as many targets (maximum MaxTg) as possible in passive mode.
 * | D4 4A MaxTg BrTy [ InitiatorData[ ] ] |   for this command, MaxTg === 1
 * 
 * 
 * @param br_ty baud rate and the modulation type
 * @param cbuf InitiatorData buffer
 * @param clen InitiatorData length
 * @param rlen response data length in 'pn532_trx_buf'
 * @return NFC_XXX 
 */
int pn53x_in_list_passive_target(uint8_t br_ty, const uint8_t *cbuf, int16_t clen, int16_t *rlen)
{
    int result;
    int16_t tlen = 3;
    pn532_trx_buf[0] = PN532_CMD_InListPassiveTarget;
    pn532_trx_buf[1] = 1;
    pn532_trx_buf[2] = br_ty;

    if (clen > 0)
    {
        memcpy (&pn532_trx_buf[3], cbuf, clen);
        tlen += clen;
    }

    result = pn53x_transmit(pn532_trx_buf, tlen, pn532_trx_buf, rlen, PN532_DEFAULT_TIMEOUT);
    if (result != NFC_SUCCESS)
    {
        return result;
    }

    if (pn532_trx_buf[1] == 1)
    {
        *rlen -= 2;
        memmove(pn532_trx_buf, &pn532_trx_buf[2], *rlen);
        return NFC_SUCCESS;
    }
    else
    {
        return PN532_ERROR_ERR;
    }
}

/**
 * @brief Poll for a tag; restricted to Type A tags. see UM0502-06 pg 113
 *
 * @param pbList	Buffer to recieve the list of tags descovered.
 * @param pnListSize	Pointer.  [in] = size of the Rx buffer.
 *                               [out] = volume of data placed in the Rx buffer.
 * @return Count of discovered tags
 */
int pn53x_poll_type_a(nfc_target_p p_nfc_tgt)
{
    int result;
    int16_t rlen = sizeof(pn532_trx_buf);

    result = pn53x_in_list_passive_target(NFC_BR_106A, NULL, 0, &rlen);
    if (result != NFC_SUCCESS)
    {
        return result;
    }

    result = pn53x_decode_target_data(pn532_trx_buf, rlen, NMT_ISO14443A, p_nfc_tgt);
    if (result != NFC_SUCCESS)
    {
        return result;
    }

    return NFC_SUCCESS;
}

/**
 * @brief Poll for a tag; restricted to Type B tags.
 *
 * @param pbList	Buffer to recieve the list of tags descovered.
 * @param pnListSize	Pointer.  [in] = size of the Rx buffer.
 *                               [out] = volume of data placed in the Rx buffer.
 * @return Count of discovered tags
 */
int pn53x_poll_type_b(nfc_target_p p_nfc_tgt)
{
    int result;
    uint8_t param[] = { 0 }; // AFI
    int16_t rlen = sizeof(pn532_trx_buf);

    result = pn53x_in_list_passive_target(NFC_BR_106B, param, 1, &rlen);
    if (result != NFC_SUCCESS)
    {
        return result;
    }

    result = pn53x_decode_target_data(pn532_trx_buf, rlen, NMT_ISO14443B, p_nfc_tgt);
    if (result != NFC_SUCCESS)
    {
        return result;
    }

    return NFC_SUCCESS;
}

/**
 * @brief Poll for a tag; restricted to Type F tags.
 *
 * @param pbList	Buffer to recieve the list of tags descovered.
 * @param pnListSize	Pointer.  [in] = size of the Rx buffer.
 *                               [out] = volume of data placed in the Rx buffer.
 * @return Count of discovered tags
 */
int pn53x_poll_type_f(nfc_target_p p_nfc_tgt)
{
    int result, i, upper_index;
    int16_t rlen = sizeof(pn532_trx_buf);
    uint8_t brc[2];

    // const uint8_t poll_ffff[] = { 0x00, 0xFF, 0xFF, 0x01, 0x03 };
    const uint8_t poll_ffff[] = {0x00, 0xFF, 0xFF, 0x00, 0x01};
    const uint8_t poll_12fc[] = {0x00, 0x12, 0xFC, 0x01, 0x03};

    brc[0] = NFC_BR_424F;
    brc[1] = NFC_BR_212F;

    for (i = 0; i < 2; i++)
    {
        result = pn53x_in_list_passive_target(brc[i], poll_ffff, sizeof(poll_ffff), &rlen);
        if (result != NFC_SUCCESS)
        {
            continue;
        }

        if (rlen < 5)
        {
            continue;
        }

        if ((pn532_trx_buf[3] == (uint8_t)0x01) && (pn532_trx_buf[4] == (uint8_t)0xFE))
        {
            p_nfc_tgt->ntt = TGT_DEP;
            return NFC_SUCCESS;
        }

        upper_index = rlen - 1;
        if ((pn532_trx_buf[upper_index - 1] != 0x12) && (pn532_trx_buf[upper_index] != 0xFC))
        {
            result = pn53x_in_list_passive_target(brc[i], poll_12fc, sizeof(poll_12fc), &rlen);
        }

        result = pn53x_decode_target_data(pn532_trx_buf, rlen, NMT_FELICA, p_nfc_tgt);
        if (result != NFC_SUCCESS)
        {
            return result;
        }

        return NFC_SUCCESS;
    }

    return NFC_EINVARG;
}

/**
 * @brief Initiator 模式的单帧发送接口
 */
uint8_t *pn53x_in_data_exchange_ex(const uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t *out_len, uint8_t *status)
{
    bool result;
    int16_t rlen = sizeof(pn532_trx_buf);

    pn532_trx_buf[0] = PN532_CMD_InDataExchange;
    pn532_trx_buf[1] = 0x01;

    if (in_len > 0 && in != NULL)
    {
        memcpy(&pn532_trx_buf[2], in, in_len);
    }

    result = pn53x_transmit(pn532_trx_buf, 2 + in_len, pn532_trx_buf, &rlen, PN532_DEFAULT_TIMEOUT);
    if (!result)
    {
        *out_len = 0;
        return NULL; // PN532_ERROR_ERR;
    }

    *status = out[1];
    if ((*status & 0x3f) != 0)
    {
        return NULL;
    }

    *out_len = rlen - 2;

    return &pn532_trx_buf[2];
}

int pn53x_in_communicate_thru(const uint8_t *tbuf, int16_t tlen)
{
    uint16_t result;
    int16_t rx_len = sizeof(pn532_trx_buf);

    pn532_trx_buf[0] = PN532_CMD_InCommunicateThru;
    if (tlen > 0)
    {
        memcpy(pn532_trx_buf + 1, tbuf, tlen);
    }
    result = pn53x_transmit(pn532_trx_buf, 1 + tlen, pn532_trx_buf, &rx_len, PN532_DEFAULT_TIMEOUT);
    if (result != NFC_SUCCESS)
    {
        return result;
    }
    if ((pn532_trx_buf[1] & 0x3f) != 0)
    {
        return PN532_ERROR_ERR;
    }
    
    // caller donot want the response info.
    //rx_len -= 1;
    //memmove(pn532_trx_buf, &pn532_trx_buf[1], rx_len);

    return NFC_SUCCESS;
}

int pn53x_dep_target_is_present(void)
{
    int result = NFC_SUCCESS;
    int16_t rlen = 0;
    const uint8_t poll_ffff[] = {0x00, 0xFF, 0xFF, 0x01, 0x03};

    result = pn53x_in_list_passive_target(NFC_BR_212F, poll_ffff, sizeof(poll_ffff), &rlen);
    if (result != NFC_SUCCESS)
    {
        return NFC_ETGRELEASED;
    }

    if (rlen < 5)
    {
        return NFC_ETGRELEASED;
    }

    if ((pn532_trx_buf[3] == (uint8_t)0x01) && (pn532_trx_buf[4] == (uint8_t)0xFE))
    {
        return NFC_SUCCESS;
    }
    else
    {
        return NFC_ETGRELEASED;
    }
}

int pn53x_decode_target_data(const uint8_t *pbtRawData, size_t szRawData, nfc_modulation_type nmt, nfc_target_t *pnt)
{
    uint8_t szAttribRes;
    int ret = NFC_SUCCESS;
    nfc_target_info *pnti = &pnt->nti;
    pnt->nm.nmt = nmt;

    switch (nmt)
    {
    case NMT_ISO14443A:
        pnt->nm.nbr = NBR_106;
        // We skip the first byte: its the target number (Tg)
        pbtRawData++;

        pnti->nai.abtAtqa[0] = *(pbtRawData++);
        pnti->nai.abtAtqa[1] = *(pbtRawData++);

        pnti->nai.btSak = *(pbtRawData++);
        if ((pnti->nai.btSak & 0xF0) == 0x00 || (pnti->nai.btSak & 0xF0) == 0x10)
            pnt->ntt = TGT_T2T;
        else if ((pnti->nai.btSak & 0x20) != 0)
            pnt->ntt = TGT_T4T;
        else if ((pnti->nai.btSak & 0x40) != 0)
            pnt->ntt = TGT_DEP;
        else
            pnt->ntt = TGT_NA;

        // Copy the NFCID1
        pnti->nai.szUidLen = *(pbtRawData++);
        memcpy(pnti->nai.abtUid, pbtRawData, pnti->nai.szUidLen);
        pbtRawData += pnti->nai.szUidLen;

        // Did we received an optional ATS (Smardcard ATR)
        if (szRawData > (pnti->nai.szUidLen + 5))
        {
            pnti->nai.szAtsLen = ((*(pbtRawData++)) - 1); // In pbtRawData, ATS Length byte is counted in ATS Frame.
            memcpy(pnti->nai.abtAts, pbtRawData, pnti->nai.szAtsLen);
        }
        else
        {
            pnti->nai.szAtsLen = 0;
        }

        // Strip CT (Cascade Tag) to retrieve and store the _real_ UID
        // (e.g. 0x8801020304050607 is in fact 0x01020304050607)
        if ((pnti->nai.szUidLen == 8) && (pnti->nai.abtUid[0] == 0x88))
        {
            pnti->nai.szUidLen = 7;
            memmove(pnti->nai.abtUid, pnti->nai.abtUid + 1, 7);
        }
        else if ((pnti->nai.szUidLen == 12) && (pnti->nai.abtUid[0] == 0x88) && (pnti->nai.abtUid[4] == 0x88))
        {
            pnti->nai.szUidLen = 10;
            memmove(pnti->nai.abtUid, pnti->nai.abtUid + 1, 3);
            memmove(pnti->nai.abtUid + 3, pnti->nai.abtUid + 5, 7);
        }
        break;

    case NMT_ISO14443B:
        pnt->ntt = TGT_T4T;
        // We skip the first byte: its the target number (Tg)
        pbtRawData++;

        // Now we are in ATQB, we skip the first ATQB byte always equal to 0x50
        pbtRawData++;

        // Store the PUPI (Pseudo-Unique PICC Identifier)
        memcpy(pnti->nbi.abtPupi, pbtRawData, 4);
        pbtRawData += 4;

        // Store the Application Data
        memcpy(pnti->nbi.abtApplicationData, pbtRawData, 4);
        pbtRawData += 4;

        // Store the Protocol Info
        memcpy(pnti->nbi.abtProtocolInfo, pbtRawData, 3);
        pbtRawData += 3;

        // We leave the ATQB field, we now enter in Card IDentifier
        szAttribRes = *(pbtRawData++);
        if (szAttribRes)
        {
            pnti->nbi.ui8CardIdentifier = *(pbtRawData++);
        }
        break;

    case NMT_ISO14443BI:
        pnt->ntt = TGT_T4T;
        // Skip V & T Addresses
        pbtRawData++;
        if (*pbtRawData != 0x07)
        { // 0x07 = REPGEN
            return NFC_ECHIP;
        }
        pbtRawData++;
        // Store the UID
        memcpy(pnti->nii.abtDIV, pbtRawData, 4);
        pbtRawData += 4;
        pnti->nii.btVerLog = *(pbtRawData++);
        if (pnti->nii.btVerLog & 0x80)
        { // Type = long?
            pnti->nii.btConfig = *(pbtRawData++);
            if (pnti->nii.btConfig & 0x40)
            {
                memcpy(pnti->nii.abtAtr, pbtRawData, szRawData - 8);
                pbtRawData += szRawData - 8;
                pnti->nii.szAtrLen = szRawData - 8;
            }
        }
        break;

    case NMT_ISO14443B2SR:
        pnt->ntt = TGT_T4T;
        // Store the UID
        memcpy(pnti->nsi.abtUID, pbtRawData, 8);
        pbtRawData += 8;
        break;

    case NMT_ISO14443B2CT:
        pnt->ntt = TGT_T4T;
        // Store UID LSB
        memcpy(pnti->nci.abtUID, pbtRawData, 2);
        pbtRawData += 2;
        // Store Prod Code & Fab Code
        pnti->nci.btProdCode = *(pbtRawData++);
        pnti->nci.btFabCode = *(pbtRawData++);
        // Store UID MSB
        memcpy(pnti->nci.abtUID + 2, pbtRawData, 2);
        pbtRawData += 2;
        break;

    case NMT_FELICA:
        if ((pbtRawData[3] == (uint8_t)0x01) && (pbtRawData[4] == (uint8_t)0xFE))
            pnt->ntt = TGT_DEP;
        else
            pnt->ntt = TGT_T3T;
        // We skip the first byte: its the target number (Tg)
        pbtRawData++;

        // Store the mandatory info
        pnti->nfi.szLen = *(pbtRawData++);
        pnti->nfi.btResCode = *(pbtRawData++);
        // Copy the NFCID2t
        memcpy(pnti->nfi.abtId, pbtRawData, 8);
        pbtRawData += 8;
        // Copy the felica padding
        memcpy(pnti->nfi.abtPad, pbtRawData, 8);
        pbtRawData += 8;
        // Test if the System code (SYST_CODE) is available
        if (pnti->nfi.szLen > 18)
        {
            memcpy(pnti->nfi.abtSysCode, pbtRawData, 2);
        }
        break;

    case NMT_JEWEL:
        pnt->ntt = TGT_DEP;
        // We skip the first byte: its the target number (Tg)
        pbtRawData++;

        // Store the mandatory info
        memcpy(pnti->nji.btSensRes, pbtRawData, 2);
        pbtRawData += 2;
        memcpy(pnti->nji.btId, pbtRawData, 4);
        break;

    case NMT_ISO14443A_TAG_EMU:
        pnt->ntt = TGT_T4T;
        break;

    case NMT_DEP:
        pnt->ntt = TGT_DEP;
        ret = NFC_ECHIP;
        break;

    default:
        break;
    }
    return ret;
}

/**
 * @brief 判断被动标签是否在场
 */
int pn53x_initiator_target_is_present(const nfc_target_p pnt)
{
    int result = NFC_ETGRELEASED;
    uint8_t tbuf[16]; // uid max 10 bytes
    int16_t tlen = 0;

    if (pnt->ntt == TGT_T2T)
    { // send RNAK
        if (pnt->nti.nai.szUidLen > 4)
        {
            tbuf[0] = PN532_CMD_TgGetInitiatorCommand;
            tlen = 1;
        }
        else 
        {
            tlen = 0;
        }

        memcpy (&tbuf[tlen], pnt->nti.nai.abtUid, pnt->nti.nai.szUidLen);
        tlen += pnt->nti.nai.szUidLen;

        result = pn53x_in_list_passive_target(NFC_BR_106A, tbuf, tlen, &tlen);
    }
    else if (pnt->ntt == TGT_T4T)
    { // send RNAK

        if (pnt->nm.nmt == NMT_ISO14443A)
        {
            tbuf[0] = 0xB2; // R(nak)
            tlen = 1;
        }
        else if (pnt->nm.nmt == NMT_ISO14443B ||
                 pnt->nm.nmt == NMT_ISO14443BI ||
                 pnt->nm.nmt == NMT_ISO14443B2SR)
        {
            tbuf[0] = 0xBA; // R(nak)
            tbuf[0] = 0x01;
            tlen = 2;
        }
        result = pn53x_in_communicate_thru(tbuf, tlen);
    }
    else if (pnt->ntt == TGT_DEP)
    { // send RNAK
        result = pn53x_dep_target_is_present();
    }

    // Target is not reachable anymore
    return result;
}

int pn53x_tg_get_data(uint8_t *data, int *len, uint8_t *status)
{
    int result;
    int16_t rx_len = sizeof(pn532_trx_buf);

    *status = 0xFF;
    pn532_trx_buf[0] = PN532_CMD_TgGetData;
    memcpy(&pn532_trx_buf[1], data, *len);
    result = pn53x_transmit(pn532_trx_buf, 1 + *len, pn532_trx_buf, &rx_len, PN532_DEFAULT_TIMEOUT);
    if (result != NFC_SUCCESS)
    {
        return result;
    }

    *status = pn532_trx_buf[1];
    if ((*status & 0x3f) != 0)
    {
        return PN532_ERROR_ERR;
    }

    *len = rx_len - 1;
    memcpy(data, &pn532_trx_buf[2], *len);

    return NFC_SUCCESS;
}

int pn53x_tg_set_data(const uint8_t *data, int len, uint8_t *status)
{
    int result;
    int16_t rx_len = sizeof(pn532_trx_buf);

    pn532_trx_buf[0] = PN532_CMD_TgSetData;
    memcpy(&pn532_trx_buf[1], data, len);
    result = pn53x_transmit(pn532_trx_buf, 1 + len, pn532_trx_buf, &rx_len, PN532_DEFAULT_TIMEOUT);
    if (result != NFC_SUCCESS)
    {
        return result;
    }

    *status = pn532_trx_buf[1];
    if ((*status & 0x3f) != 0)
    {
        return PN532_ERROR_ERR;
    }

    return NFC_SUCCESS;
}

int pn53x_tg_get_status(uint8_t *state, uint8_t *Brc)
{
    uint8_t abtCmd[] = {PN532_CMD_TgGetTargetStatus};
    uint8_t abtRes[3];
    int16_t rlen = 3;

    memset(&abtRes, 0, sizeof(abtRes));
    int result = pn53x_transmit(abtCmd, 1, abtRes, &rlen, PN532_DEFAULT_TIMEOUT);
    if ((result != NFC_SUCCESS) || (rlen != 3))
    {
        return PN532_ERROR_ERR;
    }

    if (abtRes[0] != PN532_CMD_TgGetTargetStatus + 1)
    {
        return PN532_ERROR_ERR;
    }

    *state = abtRes[1];
    *Brc = abtRes[2];
    return NFC_SUCCESS;
}

int pn532_test(void)
{
    uint8_t version[4];
    uint8_t ver_len = 4;

    pn53x_init();
    bool ret = pn53x_get_firmware_version(version, &ver_len);
    return ret;
}