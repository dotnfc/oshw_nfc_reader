/**
 **************************************************************************
 * @file     at32f415_board.c
 * @brief    set of firmware functions to manage leds and push-button.
 *           initialize delay function.
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

#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "usbd_ccid.h"

/* at-start led resouce array */
gpio_type *led_gpio_port[LED_NUM] = {LED_RED_GPIO, LED_GREEN_GPIO};
uint16_t led_gpio_pin[LED_NUM] = {LED_RED_PIN, LED_GREEN_PIN};
crm_periph_clock_type led_gpio_crm_clk[LED_NUM] = {LED_RED_GPIO_CRM_CLK, LED_GREEN_GPIO_CRM_CLK};


/* support printf function, usemicrolib is unnecessary */
#if (__ARMCC_VERSION > 6000000)
__asm(".global __use_no_semihosting\n\t");
void _sys_exit(int x)
{
    x = x;
}
/* __use_no_semihosting was requested, but _ttywrch was */
void _ttywrch(int ch)
{
    ch = ch;
}
FILE __stdout;
#else
#ifdef __CC_ARM
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};
FILE __stdout;
void _sys_exit(int x)
{
    x = x;
}
/* __use_no_semihosting was requested, but _ttywrch was */
void _ttywrch(int ch)
{
    ch = ch;
}
#endif
#endif

#if defined(__GNUC__) && !defined(__clang__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
 * @brief  retargets the c library printf function to the usart.
 * @param  none
 * @retval none
 */
PUTCHAR_PROTOTYPE
{
    while (usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET)
        ;
    usart_data_transmit(PRINT_UART, (uint16_t)ch);
    while (usart_flag_get(PRINT_UART, USART_TDC_FLAG) == RESET)
        ;
    return ch;
}

#if (defined(__GNUC__) && !defined(__clang__)) || (defined(__ICCARM__))
#if defined(__GNUC__) && !defined(__clang__)
int _write(int fd, char *pbuffer, int size)
#elif defined(__ICCARM__)
#pragma module_name = "?__write"
int __write(int fd, char *pbuffer, int size)
#endif
{
    for (int i = 0; i < size; i++)
    {
        while (usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET)
            ;
        usart_data_transmit(PRINT_UART, (uint16_t)(*pbuffer++));
        while (usart_flag_get(PRINT_UART, USART_TDC_FLAG) == RESET)
            ;
    }

    return size;
}
#endif


/**
 * @brief  initialize uart
 * @param  baudrate: uart baudrate
 * @retval none
 */
void uart_print_init(uint32_t baudrate)
{
    gpio_init_type gpio_init_struct;

#if defined(__GNUC__) && !defined(__clang__)
    setvbuf(stdout, NULL, _IONBF, 0);
#endif

    /* enable the uart and gpio clock */
    crm_periph_clock_enable(PRINT_UART_CRM_CLK, TRUE);
    crm_periph_clock_enable(PRINT_UART_TX_GPIO_CRM_CLK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* configure the uart tx pin */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins = PRINT_UART_TX_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(PRINT_UART_TX_GPIO, &gpio_init_struct);

    /* configure uart param */
    usart_init(PRINT_UART, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(PRINT_UART, TRUE);
    usart_enable(PRINT_UART, TRUE);
}

/**
 * @brief  board initialize interface init led and button
 * @param  none
 * @retval none
 */
void at32_board_init()
{
    /* initialize delay function */
    delay_init();

    /* configure led in at_start_board */
    at32_led_init(LED_RED);
    at32_led_init(LED_GREEN);

    at32_led_off(LED_RED);
    at32_led_off(LED_GREEN);

    /* configure button in at_start board */
    at32_button_init();

    uart_print_init(115200);
    printf("hello world!\n");

    usbd_ccid_init();
}

/**
 * @brief  configure button gpio
 * @param  button: specifies the button to be configured.
 * @retval none
 */
void at32_button_init(void)
{
    gpio_init_type gpio_init_struct;

    /* enable the button clock */
    crm_periph_clock_enable(USER_BUTTON_CRM_CLK, TRUE);

    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);

    /* configure button pin as input with pull-up/pull-down */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins = USER_BUTTON_PIN;
    gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
    gpio_init(USER_BUTTON_PORT, &gpio_init_struct);
}

/**
 * @brief  returns the selected button state
 * @param  none
 * @retval the button gpio pin value
 */
uint8_t at32_button_state(void)
{
    return gpio_input_data_bit_read(USER_BUTTON_PORT, USER_BUTTON_PIN);
}

/**
 * @brief  returns which button have press down
 * @param  none
 * @retval the button have press down
 */
button_type at32_button_press()
{
    static uint8_t pressed = 1;
    /* get button state in at_start board */
    if ((pressed == 1) && (at32_button_state() != RESET))
    {
        /* debounce */
        pressed = 0;
        delay_ms(10);
        if (at32_button_state() != RESET)
            return USER_BUTTON;
    }
    else if (at32_button_state() == RESET)
    {
        pressed = 1;
    }
    return NO_BUTTON;
}

/**
 * @brief  configure led gpio
 * @param  led: specifies the led to be configured.
 * @retval none
 */
void at32_led_init(led_type led)
{
    gpio_init_type gpio_init_struct;

    /* enable the led clock */
    crm_periph_clock_enable(led_gpio_crm_clk[led], TRUE);

    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);

    /* configure the led gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = led_gpio_pin[led];
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(led_gpio_port[led], &gpio_init_struct);
}

/**
 * @brief  configure led gpio
 * @param  led: specifies the led to be configured.
 * @retval none
 */
void at32_gpio_init_output(gpio_type *port, uint16_t pin)
{
    gpio_init_type gpio_init_struct;

    /* enable the led clock */
    if (port == GPIOA)
    {
        crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    }
    else if (port == GPIOB)
    {
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    }
    else if (port == GPIOC)
    {
        crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    }
    else if (port == GPIOD)
    {
        crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    }
    else if (port == GPIOF)
    {
        crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
    }
    else
    {
        return;
    }

    /* set default parameter */
    gpio_default_para_init(&gpio_init_struct);

    /* configure the led gpio */
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_pins = pin;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(port, &gpio_init_struct);
}

/**
 * @brief  turns selected led on.
 * @param  led: specifies the led to be set on.
 *   this parameter can be one of following parameters:
 *     @arg LED2
 *     @arg LED3
 *     @arg LED4
 * @retval none
 */
void at32_led_on(led_type led)
{
    if (led > (LED_NUM - 1))
        return;
    if (led_gpio_pin[led])
        led_gpio_port[led]->clr = led_gpio_pin[led];
}

/**
 * @brief  turns selected led off.
 * @param  led: specifies the led to be set off.
 *   this parameter can be one of following parameters:
 *     @arg LED2
 *     @arg LED3
 *     @arg LED4
 * @retval none
 */
void at32_led_off(led_type led)
{
    if (led > (LED_NUM - 1))
        return;
    if (led_gpio_pin[led])
        led_gpio_port[led]->scr = led_gpio_pin[led];
}

/**
 * @brief  turns selected led toggle.
 * @param  led: specifies the led to be set off.
 *   this parameter can be one of following parameters:
 *     @arg LED2
 *     @arg LED3
 *     @arg LED4
 * @retval none
 */
void at32_led_toggle(led_type led)
{
    if (led > (LED_NUM - 1))
        return;
    if (led_gpio_pin[led])
        led_gpio_port[led]->odt ^= led_gpio_pin[led];
}

