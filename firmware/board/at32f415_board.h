/**
  **************************************************************************
  * @file     at32f415_board.h
  * @brief    header file for at-start board. set of firmware functions to
  *           manage leds and push-button. initialize delay function.
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

#ifndef __AT32F415_BOARD_H
#define __AT32F415_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "at32f415.h"

/** @addtogroup AT32F415_board
  * @{
  */

/** @addtogroup BOARD
  * @{
  */

/** @defgroup BOARD_pins_definition
  * @{
  */


/******************** define led ********************/
typedef enum
{
  LED_RED                                = 0,
  LED_GREEN                              = 1,
} led_type;

#define LED_NUM                          2

#define LED_RED_PIN                      GPIO_PINS_0
#define LED_RED_GPIO                     GPIOA
#define LED_RED_GPIO_CRM_CLK             CRM_GPIOA_PERIPH_CLOCK

#define LED_GREEN_PIN                    GPIO_PINS_1
#define LED_GREEN_GPIO                   GPIOA
#define LED_GREEN_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK

/**************** define print uart ******************/
#define PRINT_UART                       USART2
#define PRINT_UART_CRM_CLK               CRM_USART2_PERIPH_CLOCK
#define PRINT_UART_TX_PIN                GPIO_PINS_2
#define PRINT_UART_TX_GPIO               GPIOA
#define PRINT_UART_TX_GPIO_CRM_CLK       CRM_GPIOA_PERIPH_CLOCK

/***************** define PN532 IO *******************/
#define PN532_SPI_X             SPI2
#define PN532_NSS_PIN           GPIO_PINS_4
#define PN532_NSS_PORT          GPIOA
#define PN532_RST_PIN           GPIO_PINS_0
#define PN532_RST_PORT          GPIOB
#define PN532_REQ_PIN           GPIO_PINS_1
#define PN532_REQ_PORT          GPIOB
#define PN532_SCK_PIN           GPIO_PINS_5
#define PN532_SCK_PORT          GPIOA
#define PN532_MISO_PIN          GPIO_PINS_6
#define PN532_MISO_PORT         GPIOA
#define PN532_MOSI_PIN          GPIO_PINS_7
#define PN532_MOSI_PORT         GPIOA

/***************** define BUZZER IO *******************/
#define BUZZER_PIN             GPIO_PINS_5
#define BUZZER_PORT            GPIOB  

/***************** define PSAM IO *******************/
#define PSAM_VCC_CTRL_PIN      GPIO_PINS_8
#define PSAM_VCC_CTRL_PORT     GPIOB  
#define PSAM_RST_PIN           GPIO_PINS_10
#define PSAM_RST_PORT          GPIOA  
#define PSAM_CLK_PIN           GPIO_PINS_8
#define PSAM_CLK_PORT          GPIOA  
#define PSAM_IO_PIN            GPIO_PINS_9
#define PSAM_IO_PORT           GPIOA
#define PSAM_UART_X            USART1 

/******************* define button *******************/
typedef enum
{
  USER_BUTTON                            = 0,
  NO_BUTTON                              = 1
} button_type;

#define USER_BUTTON_PIN                  GPIO_PINS_2
#define USER_BUTTON_PORT                 GPIOB
#define USER_BUTTON_CRM_CLK              CRM_GPIOB_PERIPH_CLOCK

/**
  * @}
  */

/** @defgroup BOARD_exported_functions
  * @{
  */

/******************** functions ********************/
void at32_board_init(void);

/* led operation function */
void at32_led_init(led_type led);
void at32_led_on(led_type led);
void at32_led_off(led_type led);
void at32_led_toggle(led_type led);

/* button operation function */
void at32_button_init(void);
button_type at32_button_press(void);
uint8_t at32_button_state(void);

/* delay function */
void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_sec(uint16_t sec);

/* printf uart init function */
void uart_print_init(uint32_t baudrate);

/**
  * @}
  */

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

