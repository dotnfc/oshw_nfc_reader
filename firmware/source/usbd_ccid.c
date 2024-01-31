#include "at32f415_board.h"
#include "at32f415_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "ccid_class.h"
#include "ccid_desc.h"

/* usb global struct define */
otg_core_type otg_core_struct;

/**
 * @brief  initialize uart
 * @param  baudrate: uart baudrate
 * @retval none
 */
void usbd_ccid_init(void)
{
    /* select usb 48m clcok source */
    usbd_clock48m_select(USB_CLK_HICK);

    /* enable usb clock */
    crm_periph_clock_enable(OTG_CLOCK, TRUE);

    /* enable usb interrupt */
    nvic_irq_enable(OTG_IRQ, 0, 0);

    /* init usb */
    usbd_init(&otg_core_struct, USB_FULL_SPEED_CORE_ID, USB_ID, &ccid_class_handler, &ccid_desc_handler);
}

/**
  * @brief  usb re-connect
  * @param  none
  * @retval none
  */
void usb_reset()
{
  delay_ms(1000);
  usbd_disconnect(&otg_core_struct.dev);
  delay_ms(1000);
  usbd_connect(&otg_core_struct.dev);       /* enable usb pull-up */
}

/**
 * @brief  this function handles otgfs interrupt.
 * @param  none
 * @retval none
 */
void OTG_IRQ_HANDLER(void)
{
    usbd_irq_handler(&otg_core_struct);
}

/**
 * @brief  usb delay millisecond function.
 * @param  ms: number of millisecond delay
 * @retval none
 */
void usb_delay_ms(uint32_t ms)
{
    /* user can define self delay function */
    delay_ms(ms);
}

/**
 * @brief  usb delay microsecond function.
 * @param  us: number of microsecond delay
 * @retval none
 */
void usb_delay_us(uint32_t us)
{
    delay_us(us);
}
