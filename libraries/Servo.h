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

static inline void set_servo_on(Servo_t *s){
    pwm_set_enabled(s->slice, true);
    s->on = true;
}

static inline void set_servo_off(Servo_t *s){
    pwm_set_enabled(s->slice, false);
    s->on = false;
} 

static inline void set_servo_pos(Servo_t *s, float p){
    float duty = p*((s->max_duty - s->min_duty)/100.0)+s->min_duty;
    s->current_pos =  p;
    pwm_set_duty(s->slice, s->chan, duty);
}

static inline void Servo_Init(Servo_t *s, uint gpio, float min_period, float max_period, float duty_init, bool invert){  	
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    s->gpio = gpio;
    s->slice = pwm_gpio_to_slice_num(gpio);
    s->chan = pwm_gpio_to_channel(gpio);
    pwm_set_enabled(s->slice, false);
    s->on = false;
    s->speed = 0;
    s->resolution = pwm_set_freq_duty(s->slice, s->chan, SERVO_FREQ, 0);

    float servo_min_duty = (min_period*SERVO_FREQ)/10.0; /*in %*/
    float servo_max_duty = (max_period*SERVO_FREQ)/10.0; /*in %*/
    set_servo_pos(s, duty_init);
    s->min_duty = servo_min_duty;
    s->max_duty = servo_max_duty;
    
    if (s->chan) {
        pwm_set_output_polarity(s->slice, false, invert);
    }
    else {
        pwm_set_output_polarity(s->slice, invert, false);
    }
    s->invert = invert;
}

#endif
