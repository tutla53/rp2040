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

uint32_t pwm_get_wrap(uint slice_num);
void pwm_set_duty(uint slice_num, uint chan, float d);
uint32_t pwm_set_freq_duty(uint slice_num,uint chan, uint32_t f, int d);
void PWMInit(PWM_t *s, uint gpio, uint16_t f_pwm, float duty_init, bool invert);
void PWMOn(PWM_t *s);
void PWMOff(PWM_t *s);
void SetPWM_Duty(PWM_t *s, float d);




#endif