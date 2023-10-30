#include <fpioa.h>
#include <motor.h>
#include <pwm.h>
#include <sleep.h>
#include <math.h>
#include <io_func.h>
#include <param.h>


#define MOTOR1_PWM      PWM_CHANNEL_0
#define MOTOR2_PWM      PWM_CHANNEL_1
#define MOTOR3_PWM      PWM_CHANNEL_2
#define MOTOR4_PWM      PWM_CHANNEL_3

#define MAX_DUTY 100
#define MIN_DUTY 0

#define THROTTLE_TO_DUTY(th) (100 * th)
#define LIMIT(x,min,max) 	((x) = ((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x) )))

static unsigned char* motor_lock;
uint8_t Motor_spd[4];

int motor_init(struct Motors* motor);
int lock_motor(void);
int unlock_motor(void);
int lock_motor_low_thr(void);
unsigned char get_motor_lock(void);
int set_motor_throttle(struct Motors* motor);
void set_pwm_duty_cycle(pwm_channel_number_t PIN,int DUTY);
//int LIMIT(int num1, int num2, int num3);

int motor_init(struct Motors* motor)
{
    struct Motors motor_init_data =
    {
        .motor_throttle        = { 6,  6,  6,  6},
        .motor_lock            = LOCKED
    };
  /*变量初始化*/
    motor_lock = &(motor->motor_lock);
    *motor = motor_init_data;


  /*电机初始化，500HZ，占空比为0， 输出关闭状态*/
    pwm_init(PWM_DEVICE_0);

    if (fpioa_set_function(MOTOR1_PWM_PIN, FUNC_TIMER0_TOGGLE1))
        return -1;
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, 10000, 0.0);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_0, 1);
    if (fpioa_set_function(MOTOR2_PWM_PIN, FUNC_TIMER0_TOGGLE2))
        return -1;
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_1, 10000, 0.0);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_1, 1);
    if (fpioa_set_function(MOTOR3_PWM_PIN, FUNC_TIMER0_TOGGLE3))
        return -1;
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_2, 10000, 0.0);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_2, 1);
    if (fpioa_set_function(MOTOR4_PWM_PIN, FUNC_TIMER0_TOGGLE4))
        return -1;
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_3, 10000, 0.0);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_3, 1);
    //set_pwm_duty_cycle(MOTOR1_PWM, 10);
    /*设定油门行程*/

   /* set_pwm_duty_cycle(MOTOR1_PWM, THROTTLE_TO_DUTY(MAX_DUTY));
    set_pwm_duty_cycle(MOTOR2_PWM, THROTTLE_TO_DUTY(MAX_DUTY));
    set_pwm_duty_cycle(MOTOR3_PWM, THROTTLE_TO_DUTY(MAX_DUTY));
    set_pwm_duty_cycle(MOTOR4_PWM, THROTTLE_TO_DUTY(MAX_DUTY));
    usleep(1);
    set_pwm_duty_cycle(MOTOR1_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    set_pwm_duty_cycle(MOTOR2_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    set_pwm_duty_cycle(MOTOR3_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    set_pwm_duty_cycle(MOTOR4_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    */
    /*进入初始状态*/
    lock_motor();
    return 0;
}

void close_motor(void)
{
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_0, 0);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_1, 0);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_2, 0);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_3, 0);
    lock_motor();
}

int unlock_motor(void)
{
    *motor_lock = UNLOCKED;
    return 0;
}

int lock_motor_low_thr(void)
{
    *motor_lock = LOCKED_LOW_THR;
    return 0;
}

int lock_motor(void)
{
    *motor_lock = LOCKED;
    set_pwm_duty_cycle(MOTOR1_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    set_pwm_duty_cycle(MOTOR2_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    set_pwm_duty_cycle(MOTOR3_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    set_pwm_duty_cycle(MOTOR4_PWM, THROTTLE_TO_DUTY(MIN_DUTY));
    return 0;
}

unsigned char get_motor_lock(void)
{
    return *motor_lock;
}

int set_motor_throttle(struct Motors * motor)
{
    if (LOCKED == motor->motor_lock)
    {
        motor->motor_throttle.motor1 = 0;
        motor->motor_throttle.motor2 = 0;
        motor->motor_throttle.motor3 = 0;
        motor->motor_throttle.motor4 = 0;

        set_pwm_duty_cycle(MOTOR1_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor1));
        set_pwm_duty_cycle(MOTOR2_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor2));
        set_pwm_duty_cycle(MOTOR3_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor3));
        set_pwm_duty_cycle(MOTOR4_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor4));
    }
    else if (LOCKED_LOW_THR == motor->motor_lock)
    {
        motor->motor_throttle.motor1 = MIN_THROTTLE;
		motor->motor_throttle.motor2 = MIN_THROTTLE;
		motor->motor_throttle.motor3 = MIN_THROTTLE;
		motor->motor_throttle.motor4 = MIN_THROTTLE;

        //printf("low\n");
        //printf("%d\n",motor->motor_throttle.motor1);
        set_pwm_duty_cycle(MOTOR1_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor1));
        set_pwm_duty_cycle(MOTOR2_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor2));
        set_pwm_duty_cycle(MOTOR3_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor3));
        set_pwm_duty_cycle(MOTOR4_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor4));
    }
    else
    {
        LIMIT(motor->motor_throttle.motor1, MIN_THROTTLE, MAX_THROTTLE);
        LIMIT(motor->motor_throttle.motor2, MIN_THROTTLE, MAX_THROTTLE);
        LIMIT(motor->motor_throttle.motor3, MIN_THROTTLE, MAX_THROTTLE);
        LIMIT(motor->motor_throttle.motor4, MIN_THROTTLE, MAX_THROTTLE);

        set_pwm_duty_cycle(MOTOR1_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor1));
        set_pwm_duty_cycle(MOTOR2_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor2));
        set_pwm_duty_cycle(MOTOR3_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor3));
        set_pwm_duty_cycle(MOTOR4_PWM,THROTTLE_TO_DUTY(motor->motor_throttle.motor4));
    }

    Motor_spd[0]=motor->motor_throttle.motor1;
    Motor_spd[1]=motor->motor_throttle.motor2;
    Motor_spd[2]=motor->motor_throttle.motor3;
    Motor_spd[3]=motor->motor_throttle.motor4;

  return 0;
}

void set_pwm_duty_cycle(pwm_channel_number_t PIN, int DUTY)
{
    double temp_throttle = (double) DUTY/10000;
    pwm_set_frequency(PWM_DEVICE_0, PIN,10000,temp_throttle);
}

#define STANDARD_Hz 440 * 4
#define A4	Notes(0)
#define B4	Notes(2)
#define C5	Notes(3)
#define D5	Notes(5)
#define E5	Notes(7)
#define F5	Notes(8)
#define G5	Notes(10)
#define A5	Notes(12)
#define B5	Notes(14)
#define C6	Notes(15)
#define D6	Notes(17)
#define E6	Notes(19)
#define F6	Notes(20)
#define G6	Notes(22)
#define A6	Notes(24)
#define B6	Notes(26)
uint32_t Notes(int note)
{
    uint32_t pulseCnt=0;
    float Hz = STANDARD_Hz*pow(1.05946309,(float)note);   //计算音高频率1.05946309 = 2^(1/12)
    return (uint32_t)Hz;
}
uint32_t Scales(int note)
{
    note -= 1;
    // printf("note:%d", note);
    int idx = 2;
    if(note >= 0)
    {
        idx = 3 + (note/7)*12;
        note %= 7;
        // printf("|%d|", note);
        idx += note>=3 ? 5:0;
        note = note>=3 ? note-3:note;
        idx += note*2;
    }
    // printf("idx:%d\n", idx);
    return Notes(idx);
}

void motor_scale(void)
{
    int i = 0;
    for(i=0; i<22; i++)
    {
        pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, Scales(i), 0.4);
        pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_0, 1);
        msleep(20);
        pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, 10000, 0.0);
        msleep(200);
    }
}
//电机依次启动测试
void motor_number_test(void)
{   //初始化
    pwm_init(PWM_DEVICE_0);
    //motor1启动
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, Scales(global_param.Kuadmini_ID), 0.3);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_0, 1);
    msleep(20);
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, 10000, 0.0);
    msleep(180);
    //motor2启动
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_1, Scales(global_param.Kuadmini_ID + 2), 0.3);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_1, 1);
    msleep(20);
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_1, 10000, 0.0);
    msleep(180);
    //motor3启动
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_2, Scales(global_param.Kuadmini_ID + 4), 0.3);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_2, 1);
    msleep(20);
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_2, 10000, 0.0);
    msleep(180);
    //motor4启动
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_3, Scales(global_param.Kuadmini_ID + 7), 0.3);
    pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_3, 1);
    msleep(20);
    pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_3, 10000, 0.0);
    msleep(180);

    // pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, 10000, 0.6);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_0, 1);
    // pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_1, 10000, 0.6);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_1, 1);
    // pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_2, 10000, 0.6);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_2, 1);
    // pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_3, 10000, 0.6);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_3, 1);
    // msleep(800);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_0, 0);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_1, 0);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_2, 0);
    // pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_3, 0);
}

void pwm_slow_up(double set_pwm)
{
    double temp=0.05;
    while(temp < set_pwm )
    {
        temp =temp + 0.05;
        pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, 10000, temp);
        pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_1, 10000, temp);
        pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_2, 10000, temp);
        pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_3, 10000, temp);
        msleep(200);
    }
}