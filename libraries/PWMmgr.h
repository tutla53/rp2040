#ifndef PWMmgr_H
#define PWMmgr_H

#define	PWM_CLOCK	125000000

typedef struct {
	uint gpio;
	uint slice;
	uint chan;
	uint resolution;
	bool on;
	bool invert;
} PWM_t;

uint32_t get_pwm_wrap(uint slice_num);
void pwm_set_duty(uint slice_num, uint chan, float d);
uint32_t pwm_set_freq_duty(uint slice_num,uint chan, uint32_t f, int d);
void PWM_Init(PWM_t *s, uint gpio, uint16_t f_pwm, float duty_init, bool invert);
void set_pwm_on(PWM_t *s);
void set_pwm_off(PWM_t *s);
void set_pwm_duty(PWM_t *s, float d);

#endif