#ifndef ROTORS_H
#define ROTORS_H

#include <stdio.h>

#define ROT_PWM_FREQ 1000

#define ROT_MAX_Duty 0.90
#define ROT_MIN_Duty 0.0

#define ROT_Speed2Duty(spd) ((spd)/100.0)

typedef enum
{
    ROTOR1_PWM = 0,
    ROTOR2_PWM,
    ROTOR3_PWM,
    ROTOR4_PWM,
} rotor_number_t;

extern uint8_t rotor_spd[4];

void rotor_init(void);
void rotor_set_speed(rotor_number_t rotn, float speed);

#endif
