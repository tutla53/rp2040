#include <stdio.h>
#include <string.h>
/*Pico Lib*/
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
/*FreeRTOS Lib*/
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/*Custom Lib*/
#include "Servo.h"
#include "PWMmgr.h"

#define mainECHO_TASK_PRIORITY	(tskIDLE_PRIORITY + 1)
/*Hardware Setup*/
#define SERVO_PIN				20
#define LED_PIN					PICO_DEFAULT_LED_PIN
#define TEMP_SENS_PIN			4
#define PWM_PIN					21
#define ADC_X_PIN				26
#define ADC_Y_PIN				27

/*Add Servo Motor*/
Servo_t servo_1;
PWM_t	pwm_1;

const float conversion_factor = 3.3f/(1<<12); /*For ADC*/

static QueueHandle_t xQueueOut = NULL, xQueueIn = NULL;
static SemaphoreHandle_t h_mutex;

extern void vApplicationStackOverflowHook(TaskHandle_t *pxTask,signed portCHAR *pcTaskName);

void vApplicationStackOverflowHook(TaskHandle_t *pxTask,signed portCHAR *pcTaskName) {
	(void)pxTask;
	(void)pcTaskName;
	for(;;);
}

typedef struct {
	float duty_1;
	float duty_2;
} Message_t;

static void mutex_lock(void) {
	// prevents competing tasks from printing in the middle of our own line of text
	xSemaphoreTake(h_mutex,portMAX_DELAY);
}

static void mutex_unlock(void) {
	// prevents competing tasks from printing in the middle of our own line of text
	xSemaphoreGive(h_mutex);
}

int map(int s, int a1, int a2, int b1, int b2) {
   return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
 }

float get_temp(){
	adc_select_input(4);
	uint16_t raw = adc_read();
	
	float result = raw * conversion_factor;
	float temp = 27 - (result-0.706)/0.001721;
	return temp;
}

float get_duty(uint8_t adc_pin){
        adc_select_input(adc_pin);
        uint16_t raw = adc_read();
        float result = ((float) raw * 100)/(1<<12); /*duty 0 - 100*/
        return result;        
}

static void input_task(void *args) {
	(void)args;
	char ch_buff = 0;
	uint32_t val = 0;

	for (;;) {
		ch_buff = getchar();
		if(ch_buff == '\n'){
			printf("PWM = %lu\n", val);
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
	Message_t send_value;
	uint16_t pos = 50;
	float duty_1 = 0, duty_2 = 0;
	send_value.duty_1 = 0;
	send_value.duty_2 = 0;

	for(;;) {
		TickType_t t0 = xTaskGetTickCount();

		/*Receive delay from USB*/
		xQueueReceive(xQueueIn, &pos, 0U);

		/*servo*/
		duty_1 = get_duty(0);
		duty_2 = get_duty(1);
		
		ServoPosition(&servo_1, duty_1);
		SetPWM_Duty(&pwm_1, duty_2);

		/*Send Pos*/
		send_value.duty_1 = duty_1;
		send_value.duty_2 = duty_2;

		vTaskDelay(pdMS_TO_TICKS(pos));
		
		/*Send the time value to USB*/
		xQueueSend(xQueueOut, &send_value, 0U);
	}
}

static void output_task (void *args) {
	(void)args;
	TickType_t t0 = 0;
	Message_t received_value;
	bool out_led = 1;
	float V_Servo = 0, servo_duty = 0, V_PWM = 0, pwm_duty =0;

	for (;;) {
		t0 = xTaskGetTickCount();
		xQueueReceive(xQueueOut, &received_value, portMAX_DELAY);
		servo_duty = (received_value.duty_1 * ((float)(servo_1.max_duty - servo_1.min_duty)/100)+servo_1.min_duty)/100;
		pwm_duty  = received_value.duty_2;
		V_Servo = (servo_duty * 3.3)/100;
		V_PWM = (pwm_duty * 3.3)/100;

		gpio_put(LED_PIN, out_led);
		out_led = !out_led;
		
		mutex_lock();
		printf("SD:%.2f, Vs:%.3f, PD:%.2f, Vp:%.3f\n", 
				servo_duty, 
				V_Servo,
				pwm_duty,
				V_PWM
				);
		mutex_unlock();
		
	}
}

static void GPIO_SETUP_INIT(){
	set_sys_clock_khz(125000, true);
	/*Communication*/
	stdio_init_all();
	
	/*LED*/
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	
	/*ADC*/
	adc_init();
	adc_gpio_init(ADC_X_PIN);
	adc_gpio_init(ADC_Y_PIN);
	adc_set_temp_sensor_enabled(true);
	adc_select_input(TEMP_SENS_PIN);
	
	/*Servo*/
	ServoInit(&servo_1, SERVO_PIN, 0.5, 2.5, false);
	
	/*PWM*/
	PWMInit(&pwm_1, PWM_PIN, 1000, 72.5, false);
}

int main() {
	GPIO_SETUP_INIT();
	ServoOn(&servo_1);
	PWMOn(&pwm_1);
	
	xQueueOut 	= xQueueCreate(10, sizeof(Message_t));
	xQueueIn  	= xQueueCreate(10, sizeof(uint32_t));
	h_mutex 	= xSemaphoreCreateMutex();
	
	xTaskCreate(main_task,"main_task",400,NULL,configMAX_PRIORITIES-2,NULL);
	xTaskCreate(input_task,"input_task",400,NULL,mainECHO_TASK_PRIORITY,NULL);
	xTaskCreate(output_task,"output_task",400,NULL,mainECHO_TASK_PRIORITY,NULL);
	
	vTaskStartScheduler();
	
    for(;;);
}
