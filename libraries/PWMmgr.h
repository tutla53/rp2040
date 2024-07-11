#ifndef PWMmgr_H
#define PWMmgr_H

#define	PWM_CLOCK	125000000

typedef struct PWM_struct{
	uint gpio;
	uint slice;
	uint chan;
	uint resolution;
	float current_duty;
	bool on;
	bool invert;
} PWM_t;

static inline uint32_t uPWM_GetWrap(uint slice_num){
    valid_params_if(PWM, slice_num >= 0 && slice_num < NUM_PWM_SLICES);
    return pwm_hw->slice[slice_num].top;
}

static inline uint32_t uPWM_SetFreqDuty(uint slice_num,uint chan, uint32_t freq, int duty){
    uint32_t clock = PWM_CLOCK;
    uint32_t divider16 = clock / freq / 4096 + (clock % (freq * 4096) != 0);
    if (divider16 / 16 == 0)
        divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / freq - 1;
    pwm_set_clkdiv_int_frac(slice_num, divider16/16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * duty / 100);
    return wrap;
}

static inline void vPWM_on(PWM_t *pxServo){
    pwm_set_enabled(pxServo->slice, true);
    pxServo->on = true;
}

static inline void vPWM_off(PWM_t *pxServo){
    pwm_set_enabled(pxServo->slice, false);
    pxServo->on = false;
} 

static inline void vPWM_SetDuty(PWM_t *pxServo, float duty){
    pxServo->current_duty = duty;
    pwm_set_chan_level(pxServo->slice, pxServo->chan, uPWM_GetWrap(pxServo->slice)*duty/100.0);
}

static inline void vPWM_Init(PWM_t *pxServo, uint gpio, uint16_t f_pwm, float init_duty, bool invert){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pxServo->gpio = gpio;
    pxServo->slice = pwm_gpio_to_slice_num(gpio);
    pxServo->chan = pwm_gpio_to_channel(gpio);
    pwm_set_enabled(pxServo->slice, false);
    pxServo->on = false;
    pxServo->resolution = uPWM_SetFreqDuty(pxServo->slice, pxServo->chan, f_pwm, 0);
    pwm_set_chan_level(pxServo->slice, pxServo->chan, uPWM_GetWrap(pxServo->slice)*init_duty/100.0);
    pxServo->current_duty = init_duty;
    if (pxServo->chan) {
        pwm_set_output_polarity(pxServo->slice, false, invert);
    }
    else {
        pwm_set_output_polarity(pxServo->slice, invert, false);
    }
    pxServo->invert = invert;
}

#endif