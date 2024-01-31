/**
 * SPDX-License-Identifier: MIT License
 * Project: oshw nfc reader
 */

#include <stdbool.h>
#include "at32f415.h"
#include "at32f415_board.h"
#include "pn53x_bus.h"

#define PN532_USE_IRQ       1
#define SPI_READ_DELAY_MS   1
#define PN532_IRQ_RISED     (uint8_t)0x55

static uint8_t _pn532_irq_rised;

static void pn532_irq_extint_config(void);
static void pn532_spi_wakeup(void);
static void pn532_bus_reset(void);

/**
 * @brief pn532 spi bus init
 */
void pn53x_bus_init(void)
{
    gpio_init_type gpio_init_struct;
    
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    
    // GPIO
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    
    /* spi1 sck pin */
    gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_pull           = GPIO_PULL_DOWN;
    gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = PN532_SCK_PIN;
    gpio_init(PN532_SCK_PORT, &gpio_init_struct);

    /* spi1 miso pin */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = PN532_MISO_PIN;
    gpio_init(PN532_MISO_PORT, &gpio_init_struct);

    /* spi1 mosi pin */
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = PN532_MOSI_PIN;
    gpio_init(PN532_MOSI_PORT, &gpio_init_struct);

    /* spi1 nss pin */
    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = PN532_NSS_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(PN532_NSS_PORT, &gpio_init_struct);
    gpio_bits_set(PN532_NSS_PORT, PN532_NSS_PIN);

    /* nfc rst pin */
    gpio_init_struct.gpio_pins = PN532_RST_PIN;
    gpio_init(PN532_RST_PORT, &gpio_init_struct);    
    gpio_bits_set(PN532_RST_PORT, PN532_RST_PIN);

    /* nfc irq pin */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = PN532_IRQ_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(PN532_IRQ_PORT, &gpio_init_struct);

    /// SPI1
    spi_init_type spi_init_struct;
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    
    /* PN532 SPI interface: 
     * Synchronous, Serial, Half-Duplex communication, 5 MHz max
     * The mode used for the clock is Mode 0:
     * Data is always sampled on the first clock edge of SCK
     * SCK is active high.
     * The data order used is LSB first.
    */
    spi_default_para_init(&spi_init_struct);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32; // 144/32 = 4.5 Mhz
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_LSB;
    spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(SPI1, &spi_init_struct);

    spi_enable(SPI1, TRUE);

    pn532_irq_extint_config();
    
    pn532_bus_reset();
    pn532_spi_wakeup();
}

void pn532_bus_reset(void)
{
    gpio_bits_reset(PN532_RST_PORT, PN532_RST_PIN);
    delay_ms(2);
    gpio_bits_set(PN532_RST_PORT, PN532_RST_PIN);
    delay_ms(2);
}

void pn532_spi_wakeup(void)
{
    gpio_bits_reset(PN532_NSS_PORT, PN532_NSS_PIN);
    delay_ms(2);
    gpio_bits_set(PN532_NSS_PORT, PN532_NSS_PIN);
    delay_ms(2);
}

void pn532_irq_extint_config(void)
{
    exint_init_type exint_init_struct;

    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOB, GPIO_PINS_SOURCE1);

    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_enable = TRUE;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_select = EXINT_LINE_1;
    exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
    exint_init(&exint_init_struct);

    // nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(EXINT1_IRQn, 1, 0);
}

/**
 * @brief pn532 spi bus send raw data (frame format be ensured by the caller)
 * 
 * @param tx_buf buffer to send
 * @param tx_len buffer length
 * @param timeout timeout of opration
 * @return PN532_BUS_ERR_SUCCESS or PN532_BUS_ERR_TIMEOUT 
 */
int32_t pn53x_bus_send(const uint8_t *tx_buf, int16_t tx_len, uint16_t timeout)
{
    uint8_t dat;
    uint32_t start = delay_get_counter();
    uint32_t delta = 0;

    _pn532_irq_rised = 0;
    
    while (tx_len -- > 0) {
        while(spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET) {
            delta = delay_get_counter_delta(start);
            if(delta > timeout) {
                return PN532_BUS_ERR_TIMEOUT;
            }
        }

        spi_i2s_data_transmit(SPI1, *tx_buf ++);
        
        while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) == SET)  {
            delta = delay_get_counter_delta(start);
            if(delta > timeout) {
                return PN532_BUS_ERR_TIMEOUT;
            }
        }
        dat = spi_i2s_data_receive(SPI1) & 0xff;
    }

    return PN532_BUS_ERR_SUCCESS;
}

/**
 * @brief wait for clf ready to response
 * 
 * @param timeout 
 * @return true or false 
 */
bool pn53x_bus_wait(uint32_t timeout)
{
    int32_t tmo = timeout & 0x7fffffff;

#if PN532_USE_IRQ // (Advanced SPI communication)
    while (tmo > 0)
    {
        if (_pn532_irq_rised == PN532_IRQ_RISED)
        {
            _pn532_irq_rised = 0;
            return true;
        }
        delay_ms(1);
        tmo -= 1;
    }
#else // USE STATUS_READ (Classic SPI communication)
    uint8_t status[] = {PN532_SPI_STATREAD, 0x00};

    while (tmo > 0)
    {
        delay_ms(10);
        spi_rw(status, sizeof(status));
        if (status[1] == PN532_SPI_READY)
        {
            return true;
        }
        else
        {
            delay_ms(5);
        }

        tmo -= 15;
    }
#endif// PN532_USE_IRQ

    return false;
}


/**
 * @brief read PN532 response
 * 
 * @param buf, read buffer
 * @param len, buffer length
 * @return the real response length from PN532
 */
int32_t pn53x_bus_recv(uint8_t *rx_buf, int16_t rx_len, uint16_t timeout)
{
    uint32_t start = delay_get_counter();
    uint32_t delta = 0;

    _pn532_irq_rised = 0;

    while (rx_len -- > 0) {
        while(spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET) {
            delta = delay_get_counter_delta(start);
            if(delta > timeout) {
                return PN532_BUS_ERR_TIMEOUT;
            }
        }

        spi_i2s_data_transmit(SPI1, 0x00);
        
        while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) == SET) {
            delta = delay_get_counter_delta(start);
            if(delta > timeout) {
                return PN532_BUS_ERR_TIMEOUT;
            }
        }
        *rx_buf ++ = spi_i2s_data_receive(SPI1) & 0xff;
    }

    return PN532_BUS_ERR_SUCCESS;
}

/**
 * @brief pn53x bus send and receive
 * 
 * @param cbuf buffer to send
 * @param clen buffer length
 * @param rbuf response buffer
 * @param rlen response max length
 * @return the number of received bytes
 */
int32_t pn53x_bus_transceive(const uint8_t *cbuf, int16_t clen, uint8_t *rbuf, int16_t rlen, int16_t timeout)
{
    gpio_bits_reset(PN532_NSS_PORT, PN532_NSS_PIN);
    delay_ms(1);

    if (pn53x_bus_send (cbuf, clen, 100) != PN532_BUS_ERR_SUCCESS)
    {
        return -1;
    }

    if (!pn53x_bus_wait(timeout)) {
        return -2;  // PN532 not response
    }

    rlen = pn53x_bus_recv (rbuf, rlen, timeout);

    delay_ms(1);
    gpio_bits_set(PN532_NSS_PORT, PN532_NSS_PIN);

    return rlen;
}

void pn53x_bus_begin_transmission(void)
{
    gpio_bits_reset(PN532_NSS_PORT, PN532_NSS_PIN);
    delay_ms(1);
}

void pn53x_bus_end_transmission(void)
{
    gpio_bits_set(PN532_NSS_PORT, PN532_NSS_PIN);
}

uint8_t pn53x_spi_trx_byte(uint8_t dat)
{
    uint32_t start = delay_get_counter();
    uint32_t delta = 0;
    uint32_t timeout = 30;

    while(spi_i2s_flag_get(SPI1, SPI_I2S_TDBE_FLAG) == RESET) {
        delta = delay_get_counter_delta(start);
        if(delta > timeout) {
            return 0x00;
        }
    }
    spi_i2s_data_transmit(SPI1, dat);

    while(spi_i2s_flag_get(SPI1, SPI_I2S_BF_FLAG) == SET) {
        delta = delay_get_counter_delta(start);
        if(delta > timeout) {
            return 0x00;
        }
    }
    dat = spi_i2s_data_receive(SPI1) & 0xff;

    return dat;
}

/* PN532 IRQ GPIO RISE Handler */
void EXINT1_IRQHandler(void)
{
    if(exint_interrupt_flag_get(EXINT_LINE_1) != RESET)
    {
        _pn532_irq_rised = PN532_IRQ_RISED;
        exint_flag_clear(EXINT_LINE_1);
    }
}
