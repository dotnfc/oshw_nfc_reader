#include "bsp.h"
#include "at32f415_board.h"
#include "at32f415_clock.h"

void test_rtos(void);
int pn532_test(void);

int main(void)
{
    system_clock_config();
    at32_board_init();

    //test_rtos();
    pn532_test();
    
    while(1) {
        at32_led_toggle(LED_RED);
        delay_ms(500);
        at32_led_toggle(LED_GREEN);
        delay_ms(500);
    }

    return 0;
}

#include "FreeRTOS.h"
#include "task.h"

void LEDTask(void *pvParameters) {
    while (1) {
        at32_led_toggle(LED_GREEN);
        vTaskDelay(pdMS_TO_TICKS(500));  // 500ms延时
    }
}

void test_rtos(void)
{
    xTaskCreate(LEDTask, "LED Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
}

