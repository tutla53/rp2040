#ifndef SERVO_H
#define SERVO_H

typedef struct {
	uint gpio;
	uint slice;
	uint chan;
	uint speed;
	uint resolution;
	bool on;
	bool invert;
} Servo;

uint32_t pwm_get_wrap(uint slice_num);
void pwm_set_dutyH(uint slice_num, uint chan, int d);
uint32_t pwm_set_freq_duty(uint slice_num,uint chan, uint32_t f, int d);
void ServoInit(Servo *s, uint gpio, bool invert);
void ServoOn(Servo *s);
void ServoOff(Servo *s); 
void ServoPosition(Servo *s, uint p);

#endif