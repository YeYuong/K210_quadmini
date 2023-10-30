#include <led.h>
#include <io_func.h>
#include <pwm.h>
#include <gpio.h>
#include <gpiohs.h>
#include <utils.h>
#include <sysctl.h>
#include <task.h>

//8bit 0~256
#define R_BRIGHTNESS_TO_DUTY(brt) (1.0 - ((float)(brt))/256.0*0.727)
#define GB_BRIGHTNESS_TO_DUTY(brt) (1.0 - ((float)(brt))/256.0*0.97)

void key_reboot(void)
{
    static uint8_t key = 0;
    if(gpio_get_pin(SWITCH_GPIO) == 0 || (global_data.body_ctrl.flight_status == FLIGHT_LOCKED && global_data.remote_ctrl.restart_btn == 1))
        key++;
    else
        key = 0;
    if(key >= 2)
    {
        key = 0;
        sysctl_reset(SYSCTL_RESET_SOC);
    }
}

void LED_Init(void)
{
    // LED
    gpiohs_set_drive_mode(LED1_GPIOHS, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(LED2_GPIOHS, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(LED3_GPIOHS, GPIO_DM_OUTPUT);
    gpiohs_set_pin(LED1_GPIOHS, GPIO_PV_LOW);
    gpiohs_set_pin(LED2_GPIOHS, GPIO_PV_HIGH);
    gpiohs_set_pin(LED3_GPIOHS, GPIO_PV_HIGH);

    // KEY
    gpio_set_drive_mode(SWITCH_GPIO, GPIO_DM_INPUT_PULL_UP);

    // RGB
    pwm_init(PWM_DEVICE_1);
    pwm_set_frequency(PWM_DEVICE_1, PWM_CHANNEL_0, 1000, 1.0);
    pwm_set_enable(PWM_DEVICE_1, PWM_CHANNEL_0, 1);
    pwm_set_frequency(PWM_DEVICE_1, PWM_CHANNEL_1, 1000, 1.0);
    pwm_set_enable(PWM_DEVICE_1, PWM_CHANNEL_1, 1);
    pwm_set_frequency(PWM_DEVICE_1, PWM_CHANNEL_2, 1000, 1.0);
    pwm_set_enable(PWM_DEVICE_1, PWM_CHANNEL_2, 1);
}

//led_num:1,2,3; open:0灭1亮-1翻转
void led_set(uint8_t led_num, int8_t open)
{
    if(open == -1)
        gpiohs_set_pin(LED1_GPIOHS + led_num - 1, !get_gpio_bit(gpiohs->output_val.u32, LED1_GPIOHS + led_num - 1));
    else
        gpiohs_set_pin(LED1_GPIOHS + led_num - 1, !open);
}

void rgb_set(uint8_t R, uint8_t G, uint8_t B)
{
    pwm_set_frequency(PWM_DEVICE_1, PWM_CHANNEL_0, 1000, R_BRIGHTNESS_TO_DUTY(R));
    pwm_set_enable(PWM_DEVICE_1, PWM_CHANNEL_0, 1);
    pwm_set_frequency(PWM_DEVICE_1, PWM_CHANNEL_1, 1000, GB_BRIGHTNESS_TO_DUTY(G));
    pwm_set_enable(PWM_DEVICE_1, PWM_CHANNEL_1, 1);
    pwm_set_frequency(PWM_DEVICE_1, PWM_CHANNEL_2, 1000, GB_BRIGHTNESS_TO_DUTY(B));
    pwm_set_enable(PWM_DEVICE_1, PWM_CHANNEL_2, 1);
}
