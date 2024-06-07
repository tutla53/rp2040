#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

static void main_task (void *args) {
	/*It will be changed to ISR*/
	(void)args;

	for(;;) {
		printf("LED on\n", CYW43_WL_GPIO_LED_PIN);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
		printf("LED off\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
		
	}
}

int main() {
	set_sys_clock_khz(125000, true);
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }
	
	xTaskCreate(main_task,"main_task",400,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();
	
    for(;;);
}