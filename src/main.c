#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

#define mainECHO_TASK_PRIORITY (tskIDLE_PRIORITY + 1)

static QueueHandle_t xQueueOut = NULL, xQueueIn = NULL;
static SemaphoreHandle_t h_mutex;

extern void vApplicationStackOverflowHook(TaskHandle_t *pxTask,signed portCHAR *pcTaskName);

void vApplicationStackOverflowHook(TaskHandle_t *pxTask,signed portCHAR *pcTaskName) {
	(void)pxTask;
	(void)pcTaskName;
	for(;;);
}

typedef struct Message {
	bool start;
	uint16_t del;
} Message_t;

/* mutex_lock()/mutex_unlock()
	- prevents competing tasks from printing in the middle of our own line of text
*/
static void mutex_lock(void) {
	xSemaphoreTake(h_mutex,portMAX_DELAY);
}

static void mutex_unlock(void) {
	xSemaphoreGive(h_mutex);
}

static void input_task(void *args) {
	(void)args;
	char ch_buff = 0;
	uint32_t val = 0;

	for (;;) {
		ch_buff = getchar();
		if(ch_buff == '\n'){
			printf("Delay Value = %lu\n", val);
			xQueueSend(xQueueIn, &val, 0U);
			val = 0;
		}
			
		if(ch_buff >= '0' && ch_buff <= '9'){
			val = val*10 + ch_buff-'0';
		}
	}
}

static void main_task (void *args) {
	/*It will be changed to ISR*/
	(void)args;
	TickType_t valueToSend = 0;
	uint32_t del = 500;
	bool out_led = 1;
  
	for(;;) {
	  TickType_t t0 = xTaskGetTickCount();
	  
	  /*Receive delay from USB*/
	  xQueueReceive(xQueueIn, &del, 0U);
	  
	  gpio_put(LED_PIN, out_led); 
	  out_led = !(out_led);
	  vTaskDelay(pdMS_TO_TICKS(del));
	  
	  valueToSend = xTaskGetTickCount()-t0;
		
	  /*Send the time value to USB*/
	  xQueueSend(xQueueOut, &valueToSend, 0U);
		
	}
}

static void output_task (void *args) {
	(void)args;
	TickType_t valueToReceived = 0, t0 = 0;

	for (;;) {
		t0 = xTaskGetTickCount();
		xQueueReceive(xQueueOut, &valueToReceived, portMAX_DELAY);
		
		mutex_lock();
		printf("Elapsed time: main_task = %lu, usb_task = %lu\n", valueToReceived, xTaskGetTickCount()-t0);
		mutex_unlock();
	}
}

int main() {
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
	set_sys_clock_khz(125000, true);
	stdio_init_all();
	
	xQueueOut 	= xQueueCreate(10, sizeof(TickType_t));
	xQueueIn  	= xQueueCreate(10, sizeof(uint32_t));
	h_mutex 	= xSemaphoreCreateMutex();
	
	xTaskCreate(main_task,"main_task",400,NULL,configMAX_PRIORITIES-2,NULL);
	xTaskCreate(input_task,"input_task",400,NULL,mainECHO_TASK_PRIORITY,NULL);
	xTaskCreate(output_task,"output_task",400,NULL,mainECHO_TASK_PRIORITY,NULL);
	
	vTaskStartScheduler();
	
    for(;;);
}
