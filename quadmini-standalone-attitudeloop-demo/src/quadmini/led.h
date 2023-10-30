#ifndef _LED_H
#define _LED_H

#include <stdio.h>

void LED_Init(void);

void led_set(uint8_t led_num, int8_t open);
void rgb_set(uint8_t R, uint8_t G, uint8_t B);

void key_reboot(void);

#endif
