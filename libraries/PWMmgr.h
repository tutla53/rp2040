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

static inline void vPWM_on(PWM_t *pxPWM){
    pwm_set_enabled(pxPWM->slice, true);
    pxPWM->on = true;
}

static inline void vPWM_off(PWM_t *pxPWM){
    pwm_set_enabled(pxPWM->slice, false);
    pxPWM->on = false;
} 

static inline void vPWM_SetDuty(PWM_t *pxPWM, float duty){
    if (duty>100) duty = 100;
    if (duty<0) duty = 0;
    pxPWM->current_duty = duty;

    pwm_set_chan_level(pxPWM->slice, pxPWM->chan, uPWM_GetWrap(pxPWM->slice)*duty/100.0);
}

static inline void vPWM_Init(PWM_t *pxPWM, uint gpio, uint16_t f_pwm, float init_duty, bool invert){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pxPWM->gpio = gpio;
    pxPWM->slice = pwm_gpio_to_slice_num(gpio);
    pxPWM->chan = pwm_gpio_to_channel(gpio);
    pwm_set_enabled(pxPWM->slice, false);
    pxPWM->on = false;
    pxPWM->resolution = uPWM_SetFreqDuty(pxPWM->slice, pxPWM->chan, f_pwm, 0);
    pwm_set_chan_level(pxPWM->slice, pxPWM->chan, uPWM_GetWrap(pxPWM->slice)*init_duty/100.0);
    pxPWM->current_duty = init_duty;
    if (pxPWM->chan) {
        pwm_set_output_polarity(pxPWM->slice, false, invert);
    }
    else {
        pwm_set_output_polarity(pxPWM->slice, invert, false);
    }
    pxPWM->invert = invert;
}

#endif