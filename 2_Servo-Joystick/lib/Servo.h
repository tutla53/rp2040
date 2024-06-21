#ifndef SERVO_H
#define SERVO_H

/*PWM CONFIG*/
#define SERVO_FREQ	50
#define	PWM_CLOCK	125000000
#define	MIN_PERIOD	0.5	/*ms*/
#define	MAX_PERIOD	2.5	/*ms*/
#define MIN_DUTY	(uint16_t)(MIN_PERIOD*10*SERVO_FREQ)
#define MAX_DUTY	(uint16_t)(MAX_PERIOD*10*SERVO_FREQ)	

typedef struct {
	uint gpio;
	uint slice;
	uint chan;
	uint speed;
	uint resolution;
	bool on;
	bool invert;
} Servo_t;

uint32_t pwm_get_wrap(uint slice_num);
void pwm_set_dutyH(uint slice_num, uint chan, int d);
uint32_t pwm_set_freq_duty(uint slice_num,uint chan, uint32_t f, int d);
void ServoInit(Servo_t *s, uint gpio, bool invert);
void ServoOn(Servo_t *s);
void ServoOff(Servo_t *s); 
void ServoPosition(Servo_t *s, uint p);

#endif