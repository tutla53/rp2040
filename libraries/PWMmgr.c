#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include "PWMmgr.h"

uint32_t pwm_get_wrap(uint slice_num){
    valid_params_if(PWM, slice_num >= 0 && slice_num < NUM_PWM_SLICES);
    return pwm_hw->slice[slice_num].top;
}

void pwm_set_duty(uint slice_num, uint chan, float d){
    pwm_set_chan_level(slice_num,chan,pwm_get_wrap(slice_num)*d/100);
}

uint32_t pwm_set_freq_duty(uint slice_num,uint chan, uint32_t f, int d){
    uint32_t clock = PWM_CLOCK;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
    if (divider16 / 16 == 0)
        divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / f - 1;
    pwm_set_clkdiv_int_frac(slice_num, divider16/16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);
    return wrap;
}

void PWMInit(PWM_t *s, uint gpio, uint16_t f_pwm, float duty_init, bool invert){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    s->gpio = gpio;
    s->slice = pwm_gpio_to_slice_num(gpio);
    s->chan = pwm_gpio_to_channel(gpio);
    pwm_set_enabled(s->slice, false);
    s->on = false;
    s->resolution = pwm_set_freq_duty(s->slice, s->chan, f_pwm, 0);
    pwm_set_duty(s->slice, s->chan, duty_init);
    if (s->chan) {
        pwm_set_output_polarity(s->slice, false, invert);
    }
    else {
        pwm_set_output_polarity(s->slice, invert, false);
    }
    s->invert = invert;
}

void PWMOn(PWM_t *s){
    pwm_set_enabled(s->slice, true);
    s->on = true;
}

void PWMOff(PWM_t *s){
    pwm_set_enabled(s->slice, false);
    s->on = false;
} 

void SetPWM_Duty(PWM_t *s, float d){
    pwm_set_duty(s->slice, s->chan, d);
}