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

void set_servo_duty(uint slice_num, uint chan, float d);
void Servo_Init(Servo_t *s, uint gpio, float min_period, float max_period, bool invert);
void set_servo_on(Servo_t *s);
void set_servo_off(Servo_t *s); 
void set_servo_pos(Servo_t *s, float p);

#endif
