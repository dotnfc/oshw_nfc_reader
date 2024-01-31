#include "at32f415_board.h"

volatile uint32_t _counter = 0; // in ms

void delay_init()
{
    crm_clocks_freq_type crm_clocks_freq_struct = {0};
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    tmr_32_bit_function_enable(TMR2, TRUE);

    crm_clocks_freq_get(&crm_clocks_freq_struct);

    /* tmr2 configuration */
    /* time base configuration */
    /* systemclock/14400/10 = 1 khz */
    tmr_base_init(TMR2, 9, (crm_clocks_freq_struct.ahb_freq / 10000) - 1);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);

    /* overflow interrupt enable */
    tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);

    /* TMR2 overflow interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR2_GLOBAL_IRQn, 0, 0);

    /* enable tmr1 */
    tmr_counter_enable(TMR2, TRUE);

    _counter = 0;

    // at32_gpio_init_output(GPIOA, GPIO_PINS_8);
}

void TMR2_GLOBAL_IRQHandler(void)
{
    if (tmr_interrupt_flag_get(TMR2, TMR_OVF_FLAG) != RESET)
    {
        // GPIOA->odt ^= GPIO_PINS_8;
        _counter++;
        tmr_flag_clear(TMR2, TMR_OVF_FLAG);
    }
}

/**
 * @brief  inserts a delay time.
 * @param  nms: specifies the delay time length, in milliseconds.
 * @retval none
 */
void delay_ms(uint16_t nms)
{
    uint32_t end = _counter + nms;
    if (end < _counter) {
        nms -= (0xffffffff - _counter);
        end = _counter + nms;
        while (end > _counter) {
        }
        
        _counter = 0;
        end = 0 + nms;
    }
    
    while (end > _counter) {
    }
}

/**
 * @brief  inserts a delay time.
 * @param  sec: specifies the delay time, in seconds.
 * @retval none
 */
void delay_sec(uint16_t sec)
{
    uint16_t index;
    for (index = 0; index < sec; index++)
    {
        delay_ms(500);
        delay_ms(500);
    }
}

/**
 * @brief get timer counter
 * 
 * @return current timer counter 
 */
uint32_t delay_get_counter(void)
{
    return _counter;
}

/**
 * @brief get timer counter delta ms
 * 
 * @param start ms
 * @return delta value 
 */
uint32_t delay_get_counter_delta(uint32_t start)
{
    if (start <= _counter) {
        return _counter - start;
    }
    else {  // fix '_counter' overflow
        return (_counter + 0xffffffff - start);
    }
}

int8_t delay_check_timemout(uint32_t start, uint32_t timeout)
{
    
}
