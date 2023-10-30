#include <data_exchange.h>
#include <data_exchange_printf.h>
#include <entry.h>
#include <esp8266.h>
#include <pmw3901mb.h>
#include <vl53l1x.h>
#include <ads1015.h>
#include <fpioa.h>
#include <imu.h>
#include <io_func.h>
#include <plic.h>
#include <printf.h>
#include <sleep.h>
#include <stdio.h>
#include <rotors.h>
#include <sysctl.h>
#include <task.h>
#include <uarths.h>
#include <uart.h>
#include <gpio.h>
#include <gpiohs.h>
#include <motor.h>
#include <led.h>
#include <param.h>
#include <ov7740.h>
#include <spi_sw.h>

struct global_data_ty global_data;

int main() {
    sysctl_pll_set_freq(SYSCTL_PLL0, 800000000UL);
    sysctl_pll_set_freq(SYSCTL_PLL2, 45158400UL);
    plic_init();

    printf("\n>>>> Quadmini <<<<\n");

    gpio_init();
    fpgio_init();
    param_init();
    LED_Init();
	rgb_set(5,10,5);
    motor_init(&(global_data.motor));
    ctrl_init(&global_data.body_ctrl);
    global_data.flags.ready_to_takeoff = 1;
    // OV7740_Init();
    opticalFlow_Init();
    imu_init();
    VL53L1X_Init();
    ADS1015_Init();
    esp8266_init(global_param.wifi_saved_data.wifi_name,
                global_param.wifi_saved_data.password,
                global_param.wifi_saved_data.ip_address,
                global_param.wifi_saved_data.gateway,
                global_param.wifi_saved_data.use_tcp);
    serial_comm_init();
    motor_number_test();
    // esp8266_init("ETLAB410-2","qwe~~asd","192.168.0.160","192.168.0.1", 0);
    // esp8266_init("ETLAB410-2","qwe~~asd","192.168.0.161","192.168.0.1");
    // esp8266_init("ETLAB410-2","qwe~~asd","192.168.0.162","192.168.0.1");
    // esp8266_init("HYC","13913927559","192.168.2.160","192.168.2.1",1);
    // esp8266_init("Xiaomi_F5D5","13913927559","192.168.1.160","192.168.1.1",1);
    // esp8266_init("asdf","12346789","192.168.43.160","192.168.43.1",0);
    // OV7740_Init();
    

    task_loop(&global_data);

    return 1;
}
