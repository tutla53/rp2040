#include <stdio.h>
#include <string.h>

/*Pico Lib*/
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

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
#define LED_PIN             PICO_DEFAULT_LED_PIN
#define TEMP_SENS_PIN       4

/*Servo*/
#define SERVO_MID_PIN       18 /*PWM Channel 1A*/
#define SERVO_END_PIN       20 /*PWM Channel 2A*/
#define SERVO_BASE_PIN      22 /*PWM Channel 3A*/
#define ADC_X_PIN           26 /*ADC 0*/
#define ADC_Y_PIN           27 /*ADC 1*/
#define ADC_SPEED_PIN       28 /*ADC 2*/

/*UART 1: HC-05 Bluetooth*/
#define UART_ID     uart1
#define BAUD_RATE   9600
#define DATA_BITS   8
#define STOP_BITS   1
#define PARITY      UART_PARITY_NONE
#define UART_TX_PIN 4
#define UART_RX_PIN 5

typedef struct Message{
    float base_duty;
    float mid_duty;
    float end_duty;
} Message_t;

/*Add Servo Motor*/
Servo_t mtrServoMid, mtrServoEnd, mtrServoBase;
Message_t xServoMessage;
PWM_t	pwm_1;

static QueueHandle_t xQueue_USB_Out = NULL, xQueue_UART_In = NULL, xQueueServo = NULL;
static SemaphoreHandle_t h_mutex;

const float adc_volt_per_unit = 3.3f/(1<<12); /*For ADC*/

/* Prevents competing tasks from printing in the middle of our own line of text*/
static void mutex_lock(void) {
    xSemaphoreTake(h_mutex,portMAX_DELAY);
}

/* Prevents competing tasks from printing in the middle of our own line of text*/
static void mutex_unlock(void) {
    xSemaphoreGive(h_mutex);
}

int map(int s, int a1, int a2, int b1, int b2) {
    return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}

/*****************************************************
 * Communication Section
 * UART0 --> Stdio USB
 * UART1 --> HC-05 Bluetooth
 *****************************************************/
int getchar_timeout_us_uart(uint32_t timeout_us, uart_inst_t *uart) {
    absolute_time_t t = make_timeout_time_us(timeout_us);
    while (!uart_is_readable(uart)) {
        if (absolute_time_diff_us(t, get_absolute_time()) > 0) {
            return -1;
        }
        sleep_ms(1);
    }
    return uart_getc(uart);
}

static void USBOutputTask(void *args) {
    (void)args;
    TickType_t t0 = 0;
    Message_t* pxDuty;
    bool out_led = 1;
    int del = 0;

    while(true){
        t0 = xTaskGetTickCount();
        if(xQueueReceive(xQueue_USB_Out, &pxDuty, portMAX_DELAY) == pdTRUE){
            gpio_put(LED_PIN, out_led);
            out_led = !out_led;

            mutex_lock();
            printf("base_d:%.2f, mid_d:%.2f, end_d:%.2f, t:%lu\n",
                    pxDuty->base_duty, pxDuty->mid_duty, pxDuty->end_duty, xTaskGetTickCount()-t0);
            mutex_unlock();
        }
    }
}

void UART1_Rx_Handler(void *args) {
    (void)args;
    int ch_buff = 0;
    uint32_t val = 0;

    while(true){
        /*Create Non-Blocking getchar -> return PICO_ERROR_TIMEOUT = -1 if No Input*/
        while ((ch_buff = getchar_timeout_us_uart(100, uart1)) != '\n') {
            if(ch_buff >= '0' && ch_buff <= '9'){
                val = val*10 + ch_buff-'0';
            }
        }

        mutex_lock();
        printf("uart val = %lu\n", val);
        mutex_unlock();

        xQueueSend(xQueue_UART_In, &val, 0U);
        val = 0;
    }
}

void Servo_IRQ_Handler(){
    Message_t* pxDuty;
    if (xQueueReceive(xQueueServo, &pxDuty, 0U) == pdTRUE){
        pwm_clear_irq(mtrServoBase.slice);
        vServoSetPos(&mtrServoBase, pxDuty->base_duty);

        pwm_clear_irq(mtrServoMid.slice);
        vServoSetPos(&mtrServoMid, pxDuty->mid_duty);

        pwm_clear_irq(mtrServoEnd.slice);
        vServoSetPos(&mtrServoEnd, pxDuty->end_duty);
    }
}

float get_temp(){
    adc_select_input(TEMP_SENS_PIN);
    uint16_t raw = adc_read();

    float temp_volt = raw * adc_volt_per_unit;
    float temp = 27 - (temp_volt-0.706)/0.001721;
    return temp;
}

float get_duty(Servo_t *s, uint8_t adc_pin){
    /*Pos 0-100*/
    float pos = s->current_pos;
    uint16_t raw = 0, raw_speed=0;

    adc_select_input(adc_pin);
    raw = adc_read();

    adc_select_input(2);
    raw_speed = adc_read();
    float speed = ((float) raw_speed/5120.0) + 0.1; /*0.1 - 0.9 */

    if (raw < 256) pos -= speed;
    else if (raw > 3840) pos += speed;

    if (pos > 100)  pos = 100;
    if (pos < 0)    pos = 0;

    return pos;
}

static void main_task(void *args) {
    /*It will be changed to ISR*/
    (void)args;
    Message_t* pxToxServoMessage;
    uint16_t del = 10;

    while(true) {
        TickType_t t0 = xTaskGetTickCount();

        /*Receive delay from UART*/
        xQueueReceive(xQueue_UART_In, &del, 0U);

        /*Get Duty*/
        xServoMessage.base_duty = get_duty(&mtrServoBase, 0);
        xServoMessage.mid_duty  = get_duty(&mtrServoMid, 1);
        xServoMessage.end_duty  = 3 + xServoMessage.mid_duty;

        /*Send the time value to USB*/
        pxToxServoMessage = &xServoMessage;
        xQueueSend(xQueue_USB_Out, &pxToxServoMessage, 0U);
        xQueueSend(xQueueServo, &pxToxServoMessage, 0U);

        if(del <= 0) del = 1;
        vTaskDelay(pdMS_TO_TICKS(del));
    }
}

static void GPIO_SETUP_INIT(){
    set_sys_clock_khz(125000, true);
    
    /*UART0 - USB*/
    stdio_init_all(); 
    
    /*UART1 - HC-05*/
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);

    /*LED*/
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    /*ADC*/
    adc_init();
    adc_gpio_init(ADC_X_PIN);
    adc_gpio_init(ADC_Y_PIN);
    adc_gpio_init(ADC_SPEED_PIN);
    adc_set_temp_sensor_enabled(false);

    /*Servo*/
    vServoInit(&mtrServoMid, SERVO_MID_PIN, 0.5, 2.5, 0, false);
    vServoInit(&mtrServoEnd, SERVO_END_PIN, 0.5, 2.5, 3, false);
    vServoInit(&mtrServoBase, SERVO_BASE_PIN, 0.5, 2.5, 50, false);

    uint32_t pwm_slice_mask = 0;
    pwm_slice_mask = (1<<(mtrServoMid.slice))|(1<<(mtrServoBase.slice))|(1<<(mtrServoEnd.slice));
    pwm_clear_irq(mtrServoMid.slice);
    pwm_clear_irq(mtrServoBase.slice);
    pwm_clear_irq(mtrServoEnd.slice);
    pwm_set_irq_mask_enabled(pwm_slice_mask, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, Servo_IRQ_Handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    vServo_on(&mtrServoMid);
    vServo_on(&mtrServoEnd);
    vServo_on(&mtrServoBase);
}

int main() {
    GPIO_SETUP_INIT();

    xQueue_USB_Out  = xQueueCreate(5, sizeof(Message_t*));
    xQueue_UART_In   = xQueueCreate(5, sizeof(uint32_t));
    xQueueServo     = xQueueCreate(20, sizeof(Message_t*));
    h_mutex         = xSemaphoreCreateMutex();

    xTaskCreate(main_task,"main_task",400,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(UART1_Rx_Handler, "uart_task",400,NULL,tskIDLE_PRIORITY,NULL);
    xTaskCreate(USBOutputTask,"output_task",400,NULL,tskIDLE_PRIORITY,NULL);

    vTaskStartScheduler();

    while(true){

    }
}
