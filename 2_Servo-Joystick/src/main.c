#include <stdio.h>
#include <string.h>
/*Pico Lib*/
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
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
#define LED_PIN					PICO_DEFAULT_LED_PIN
#define TEMP_SENS_PIN			4
#define PWM_PIN					18 /*PWM Channel 1A*/
#define SERVO_PIN_1				20 /*PWM Channel 2A*/
#define SERVO_PIN_2				21 /*PWM Channel 2A*/
#define ADC_X_PIN				26
#define ADC_Y_PIN				27
#define ADC_PWM_PIN				28

/*Add Servo Motor*/
Servo_t servo_1, servo_2;
PWM_t	pwm_1;

const float conversion_factor = 3.3f/(1<<12); /*For ADC*/

static QueueHandle_t xQueue_USB_Out = NULL, xQueue_USB_In = NULL, xQueueServo = NULL;
static SemaphoreHandle_t h_mutex;

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

void servo_IRQ_handler(){
	Message_t d;
	pwm_clear_irq(servo_1.slice);
	if (xQueueReceive(xQueueServo, &d, 0U) == pdPASS ){
		set_servo_pos(&servo_1, d.duty_1);
		set_servo_pos(&servo_2, d.duty_2);
	}
}

float get_temp(){
	adc_select_input(TEMP_SENS_PIN);
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
	int ch_buff = 0;
	uint32_t val = 0;

	while(true){
		/*Create Non-Blocking getchar -> return PICO_ERROR_TIMEOUT = -1 if No Input*/
		while ((ch_buff = getchar_timeout_us(100)) != '\n') {
			if(ch_buff >= '0' && ch_buff <= '9'){
				val = val*10 + ch_buff-'0';
			}				
        }
		
		mutex_lock();
		printf("delay = %lu\n", val);
		mutex_unlock();
		
		xQueueSend(xQueue_USB_In, &val, 0U);
		val = 0;
	}
}

static void main_task(void *args) {
	/*It will be changed to ISR*/
	(void)args;
	Message_t send_value;
	uint16_t pos = 10;
	float duty_1 = 0, duty_2 = 0, duty_3;
	send_value.duty_1 = 0;
	send_value.duty_2 = 0;

	while(true) {
		TickType_t t0 = xTaskGetTickCount();

		/*Receive delay from USB*/
		xQueueReceive(xQueue_USB_In, &pos, 0U);

		/*Get Duty*/
		duty_1 = get_duty(0);
		duty_2 = get_duty(1);
		duty_3 = get_duty(3);
		set_pwm_duty(&pwm_1, duty_3);

		/*Send Pos*/
		send_value.duty_1 = duty_1;
		send_value.duty_2 = duty_2;
		
		/*Send the time value to USB*/
		xQueueSend(xQueue_USB_Out, &send_value, 0U);
		xQueueSend(xQueueServo, &send_value, 0U);

		vTaskDelay(pdMS_TO_TICKS(pos));
	}
}

static void output_task(void *args) {
	(void)args;
	TickType_t t0 = 0;
	Message_t received_value;
	bool out_led = 1;
	float V_Servo_1 = 0, V_Servo_2 = 0, servo_duty_1 = 0, servo_duty_2 = 0;

	while(true){
		t0 = xTaskGetTickCount();
		if(xQueueReceive(xQueue_USB_Out, &received_value, portMAX_DELAY) == pdPASS){
			servo_duty_1 = (received_value.duty_1 * ((float)(servo_1.max_duty - servo_1.min_duty)/100)+servo_1.min_duty)/100;
			servo_duty_2  = (received_value.duty_2 * ((float)(servo_2.max_duty - servo_2.min_duty)/100)+servo_2.min_duty)/100;
			V_Servo_1 = (servo_duty_1 * 3.3)/100;
			V_Servo_2 = (servo_duty_2 * 3.3)/100;

			gpio_put(LED_PIN, out_led);
			out_led = !out_led;
			
			mutex_lock();
			printf("D1:%.2f, Vs1:%.3f, D2:%.2f, Vs1:%.3f\n", 
					servo_duty_1, V_Servo_1, servo_duty_2, V_Servo_2);
			mutex_unlock();
		}
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
	adc_gpio_init(ADC_PWM_PIN);
	adc_set_temp_sensor_enabled(true);
	
	/*Servo*/
	Servo_Init(&servo_1, SERVO_PIN_1, 0.5, 2.5, false);
	Servo_Init(&servo_2, SERVO_PIN_2, 0.5, 2.5, false);
	pwm_clear_irq(servo_1.slice);
	pwm_clear_irq(servo_1.slice);
	pwm_set_irq_enabled(servo_1.slice, true);
	irq_set_exclusive_handler(PWM_IRQ_WRAP, servo_IRQ_handler);
	irq_set_enabled(PWM_IRQ_WRAP, true);
	set_servo_on(&servo_1);
	set_servo_on(&servo_2);
	
	/*PWM*/
	PWM_Init(&pwm_1, PWM_PIN, 1000, 72.5, false);
	set_pwm_on(&pwm_1);
}

int main() {
	GPIO_SETUP_INIT();
	
	xQueue_USB_Out 	= xQueueCreate(10, sizeof(Message_t));
	xQueue_USB_In  	= xQueueCreate(10, sizeof(uint32_t));
	xQueueServo	= xQueueCreate(10, sizeof(Message_t));
	h_mutex 	= xSemaphoreCreateMutex();
	
	xTaskCreate(main_task,"main_task",400,NULL,configMAX_PRIORITIES-1,NULL);
	xTaskCreate(input_task,"input_task",400,NULL,tskIDLE_PRIORITY,NULL);
	xTaskCreate(output_task,"output_task",400,NULL,tskIDLE_PRIORITY,NULL);
	
	vTaskStartScheduler();
	
    while(true){

	}
}
