/**
 * SPDX-License-Identifier: MIT License
 * Project: oshw nfc reader
 */
#ifndef _PN532_BUS_H
#define _PN532_BUS_H

#include <stdint.h>

#define PN532_BUS_ERR_SUCCESS   0
#define PN532_BUS_ERR_FAILED   -1
#define PN532_BUS_ERR_TIMEOUT  -2


void pn53x_bus_init(void);

void pn53x_bus_begin_transmission(void);
void pn53x_bus_end_transmission(void);

bool pn53x_bus_wait(uint32_t timeout);
int32_t pn53x_bus_send(const uint8_t *tx_buf, int16_t tx_len, uint16_t timeout);
int32_t pn53x_bus_recv(uint8_t *rx_buf, int16_t rx_len, uint16_t timeout);
uint8_t pn53x_spi_trx_byte(uint8_t dat);

int32_t pn53x_bus_transceive(const uint8_t *cbuf, int16_t clen, uint8_t *rbuf, int16_t rlen, int16_t timeout);

#endif // _PN532_BUS_H
