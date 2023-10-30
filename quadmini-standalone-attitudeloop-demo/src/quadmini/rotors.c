#include <rotors.h>
#include <pwm.h>

uint8_t rotor_spd[4];

void rotor_init(void)
{
	pwm_init(PWM_DEVICE_0);
    pwm_set_frequency(PWM_DEVICE_0, ROTOR1_PWM, ROT_PWM_FREQ, 0);
	pwm_set_frequency(PWM_DEVICE_0, ROTOR2_PWM, ROT_PWM_FREQ, 0);
	pwm_set_frequency(PWM_DEVICE_0, ROTOR3_PWM, ROT_PWM_FREQ, 0);
	pwm_set_frequency(PWM_DEVICE_0, ROTOR4_PWM, ROT_PWM_FREQ, 0);
    pwm_set_enable(PWM_DEVICE_0, ROTOR1_PWM, 1);
    pwm_set_enable(PWM_DEVICE_0, ROTOR2_PWM, 1);
    pwm_set_enable(PWM_DEVICE_0, ROTOR3_PWM, 1);
    pwm_set_enable(PWM_DEVICE_0, ROTOR4_PWM, 1);
}

void rotor_set_speed(rotor_number_t rotn, float speed)//speed from 0 to 100
{
    float duty;
    speed = speed<0.0?0.0:(speed>100.0?100.0:speed);
    rotor_spd[(int)rotn] = (uint8_t)speed;
    duty = ROT_Speed2Duty(speed);
    //pwm_set_enable(PWM_DEVICE_0, rotn, 0);
    pwm_set_frequency(PWM_DEVICE_0, rotn, ROT_PWM_FREQ, duty);
    pwm_set_enable(PWM_DEVICE_0, rotn, 1);
}


