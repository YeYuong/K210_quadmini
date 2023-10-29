#ifndef _MOTOR_H
#define _MOTOR_H

#define UNLOCKED			0
#define LOCKED				1
#define LOCKED_LOW_THR 		2

#define LOCK_STATE1			1
#define LOCK_STATE2			2

#define MIN_THROTTLE		6
#define MAX_THROTTLE		100

typedef enum
{
    MOTOR1_PWM = 0,
    MOTOR2_PWM,
    MOTOR3_PWM,
    MOTOR4_PWM,
} Motor_number_t;

extern uint8_t Motor_spd[4];

struct MotorThrottle {
	unsigned int motor1;
	unsigned int motor2;
	unsigned int motor3;
	unsigned int motor4;
};

struct Motors {
	struct MotorThrottle motor_throttle;
	unsigned char motor_lock;
};

int motor_init(struct Motors * motor);
void close_motor(void);
int unlock_motor(void);
int lock_motor_low_thr(void);
int lock_motor(void);
unsigned char get_motor_lock(void);
int set_motor_throttle(struct Motors * motor);
void motor_number_test(void);
void motor_scale(void);
void pwm_slow_up(double set_pwm);

#endif
