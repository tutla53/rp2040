#ifndef SERVO_H
#define SERVO_H

#include "PWMmgr.h"

/*PWM CONFIG*/
#define SERVO_FREQ	50

typedef struct Servo {
	uint gpio;
	uint slice;
	uint chan;
	uint speed;
	uint resolution;
	float min_duty;
	float max_duty;
	float current_pos;
	bool on;
	bool invert;
} Servo_t;

static inline void vServo_on(Servo_t *pxServo){
    pwm_set_enabled(pxServo->slice, true);
    pxServo->on = true;
}

static inline void vServo_off(Servo_t *pxServo){
    pwm_set_enabled(pxServo->slice, false);
    pxServo->on = false;
} 

static inline void vServoSetPos(Servo_t *pxServo, float pos){
    float duty = pos*((pxServo->max_duty - pxServo->min_duty)/100.0)+pxServo->min_duty;
    pxServo->current_pos =  pos;
    if (duty>100) duty = 100;
    if (duty<0) duty = 0;
    pwm_set_chan_level(pxServo->slice, pxServo->chan, uPWM_GetWrap(pxServo->slice)*duty/100.0);
}

static inline void vServoInit(Servo_t *pxServo, uint gpio, float min_period, float max_period, float init_duty, bool invert){  	
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pxServo->gpio = gpio;
    pxServo->slice = pwm_gpio_to_slice_num(gpio);
    pxServo->chan = pwm_gpio_to_channel(gpio);
    pwm_set_enabled(pxServo->slice, false);
    pxServo->on = false;
    pxServo->speed = 0;
    pxServo->resolution = uPWM_SetFreqDuty(pxServo->slice, pxServo->chan, SERVO_FREQ, 0);

    float servo_min_duty = (min_period*SERVO_FREQ)/10.0; /*in %*/
    float servo_max_duty = (max_period*SERVO_FREQ)/10.0; /*in %*/
    vServoSetPos(pxServo, init_duty);
    pxServo->min_duty = servo_min_duty;
    pxServo->max_duty = servo_max_duty;
    
    if (pxServo->chan) {
        pwm_set_output_polarity(pxServo->slice, false, invert);
    }
    else {
        pwm_set_output_polarity(pxServo->slice, invert, false);
    }
    pxServo->invert = invert;
}

#endif
