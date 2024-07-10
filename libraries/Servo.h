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

static inline void set_servo_duty(uint slice_num, uint chan, float d){
    /*For Servo*/
    pwm_set_chan_level(slice_num, chan, get_pwm_wrap(slice_num) * d / 10000);
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

    float servo_min_duty = (min_period*10*SERVO_FREQ);
    float servo_max_duty = (max_period*10*SERVO_FREQ);
    set_servo_duty(s->slice, s->chan, duty_init);
    s->min_duty = servo_min_duty;
    s->max_duty = servo_max_duty;
    s->current_pos = duty_init;
    
    if (s->chan) {
        pwm_set_output_polarity(s->slice, false, invert);
    }
    else {
        pwm_set_output_polarity(s->slice, invert, false);
    }
    s->invert = invert;
}

static inline void set_servo_on(Servo_t *s){
    pwm_set_enabled(s->slice, true);
    s->on = true;
}

static inline void set_servo_off(Servo_t *s){
    pwm_set_enabled(s->slice, false);
    s->on = false;
} 

static inline void set_servo_pos(Servo_t *s, float p){
    uint16_t M = (s->max_duty - s->min_duty)/100;
    set_servo_duty(s->slice, s->chan, p*M+s->min_duty);
    s->current_pos =  p;
}

#endif
