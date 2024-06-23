#ifndef SERVO_H
#define SERVO_H

/*PWM CONFIG*/
#define SERVO_FREQ	  		50

typedef struct {
	uint gpio;
	uint slice;
	uint chan;
	uint speed;
	uint resolution;
	float min_duty;
	float max_duty;
	bool on;
	bool invert;
} Servo_t;

void pwm_set_dutyH(uint slice_num, uint chan, float d);
void ServoInit(Servo_t *s, uint gpio, float min_period, float max_period, bool invert);
void ServoOn(Servo_t *s);
void ServoOff(Servo_t *s); 
void ServoPosition(Servo_t *s, float p);

#endif
