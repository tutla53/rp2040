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
#define LED_PIN				PICO_DEFAULT_LED_PIN
#define TEMP_SENS_PIN       4
#define PWM_PIN             18 /*PWM Channel 1A*/
#define SERVO_END_PIN       20 /*PWM Channel 2A*/
#define SERVO_MID_PIN       21 /*PWM Channel 2A*/
#define SERVO_BASE_PIN      22 /*PWM Channel 3A*/
#define ADC_X_PIN           26 /*ADC 0*/
#define ADC_Y_PIN           27 /*ADC 1*/
#define ADC_SPEED_PIN       28 /*ADC 2*/

/*Add Servo Motor*/
Servo_t mid_servo, end_servo, base_servo;
PWM_t	pwm_1;

const float conversion_factor = 3.3f/(1<<12); /*For ADC*/

static QueueHandle_t xQueue_USB_Out = NULL, xQueue_USB_In = NULL, xQueueServo = NULL;
static SemaphoreHandle_t h_mutex;

typedef struct {
    float base_duty;
    float mid_duty;
    float end_duty;
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
    pwm_clear_irq(mid_servo.slice);
    pwm_clear_irq(base_servo.slice);
    if (xQueueReceive(xQueueServo, &d, 0U) == pdPASS ){
        set_servo_pos(&base_servo, d.base_duty);
        set_servo_pos(&mid_servo, d.mid_duty);
        set_servo_pos(&end_servo, d.end_duty);
    }
}

float get_temp(){
    adc_select_input(TEMP_SENS_PIN);
    uint16_t raw = adc_read();

    float result = raw * conversion_factor;
    float temp = 27 - (result-0.706)/0.001721;
    return temp;
}

float get_duty(Servo_t *s, uint8_t adc_pin){
        /*Pos 0-100*/
        float pos = s->current_pos;
        uint16_t raw = 0;

        adc_select_input(adc_pin);
        raw = adc_read();

        adc_select_input(2);
        raw = adc_read();
        float speed = ((float) raw /5120.0) + 0.1; /*0.1 - 0.9 */

        if (raw < 256) pos -= speed;
        else if (raw > 3840) pos += speed;

        if (pos > 100) 	pos = 100;
        if (pos < 0)	pos = 0;

        return pos;
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
    float mid_duty = 0, end_duty = 0, base_duty=0;
    send_value.mid_duty = 0;
    send_value.end_duty = 0;

    while(true) {
        TickType_t t0 = xTaskGetTickCount();

        /*Receive delay from USB*/
        xQueueReceive(xQueue_USB_In, &pos, 0U);

        /*Get Duty*/
        base_duty = get_duty(&base_servo, 0);
        mid_duty = get_duty(&mid_servo, 1);
        end_duty = 50;

        /*Send Pos*/
        send_value.base_duty = base_duty;
        send_value.mid_duty = mid_duty;
        send_value.end_duty = end_duty;

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
    float base_servo_duty = 0, mid_servo_duty = 0, end_servo_duty = 0;

    while(true){
        t0 = xTaskGetTickCount();
        if(xQueueReceive(xQueue_USB_Out, &received_value, portMAX_DELAY) == pdPASS){
            mid_servo_duty  = received_value.mid_duty;
            end_servo_duty  = received_value.end_duty;
            base_servo_duty = received_value.base_duty;

            gpio_put(LED_PIN, out_led);
            out_led = !out_led;

            mutex_lock();
            printf("base_d:%.2f, mid_d:%.2f, end_d:%.2f\n",
                    base_servo_duty, mid_servo_duty, end_servo_duty);
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
    adc_gpio_init(ADC_SPEED_PIN);
    adc_set_temp_sensor_enabled(true);

    /*Servo*/
    Servo_Init(&mid_servo, SERVO_END_PIN, 0.5, 2.5, 50, false);
    Servo_Init(&end_servo, SERVO_MID_PIN, 0.5, 2.5, 50, false);
    Servo_Init(&base_servo, SERVO_BASE_PIN, 0.5, 2.5, 50, false);

    pwm_clear_irq(mid_servo.slice);
    pwm_set_irq_enabled(mid_servo.slice, true);

    pwm_clear_irq(base_servo.slice);
    pwm_set_irq_enabled(base_servo.slice, true);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, servo_IRQ_handler);

    irq_set_enabled(PWM_IRQ_WRAP, true);
    set_servo_on(&mid_servo);
    set_servo_on(&end_servo);
    set_servo_on(&base_servo);

    /*PWM*/
    PWM_Init(&pwm_1, PWM_PIN, 1000, 72.5, false);
    set_pwm_on(&pwm_1);
}

int main() {
    GPIO_SETUP_INIT();

    xQueue_USB_Out  = xQueueCreate(10, sizeof(Message_t));
    xQueue_USB_In   = xQueueCreate(10, sizeof(uint32_t));
    xQueueServo = xQueueCreate(10, sizeof(Message_t));
    h_mutex = xSemaphoreCreateMutex();

    xTaskCreate(main_task,"main_task",400,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(input_task,"input_task",400,NULL,tskIDLE_PRIORITY,NULL);
    xTaskCreate(output_task,"output_task",400,NULL,tskIDLE_PRIORITY,NULL);

    vTaskStartScheduler();

    while(true){

    }
}
