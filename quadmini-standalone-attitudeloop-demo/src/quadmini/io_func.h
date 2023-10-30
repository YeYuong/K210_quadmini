#ifndef MAIN_H
#define MAIN_H

#include <fpioa.h>

#define I2C_SCL 22
#define I2C_SDA 21
#define IST_RSTn 19
// #define IST_DRDY 20
  #define IST_RSTn_GPIOHS 0
//   #define IST_DRDY_GPIOHS 1

#define SPI_CLK 26
#define SPI_MOSI 27
#define SPI_MISO 28
#define BMI_Acc_CS 29
#define BMI_Gyr_CS 30
#define PMW_CS 24
  #define PMW_NCS_GPIOHS 1
#define BARO_CS 25
  #define SPI_CLK_GPIOHS 8
  #define SPI_MOSI_GPIOHS 9
  #define SPI_MISO_GPIOHS 10
  #define BMI_Acc_CS_GPIOHS 11
  #define BMI_Gyr_CS_GPIOHS 12

#define WIFI_SCK 1
#define WIFI_MISO 2
#define WIFI_MOSI 3
#define WIFI_NSS 0
#define WIFI_RDY 6
#define WIFI_IRQ 7
#define WIFI_EN 8
  #define WIFI_RDY_GPIOHS 2
  #define WIFI_IRQ_GPIOHS 3
  #define WIFI_EN_GPIOHS  4

#define LED1_PIN 10
#define LED2_PIN 18
#define LED3_PIN 33
  #define LED1_GPIO 0
  #define LED2_GPIO 1
  #define LED3_GPIO 2
  #define LED1_GPIOHS 5
  #define LED2_GPIOHS 6
  #define LED3_GPIOHS 7
#define RGB_R_PIN 11
#define RGB_G_PIN 12
#define RGB_B_PIN 13

#define MOTOR1_PWM_PIN 35
#define MOTOR2_PWM_PIN 9
#define MOTOR3_PWM_PIN 17
#define MOTOR4_PWM_PIN 34

#define SWITCH_PIN 15
  #define SWITCH_GPIO 4

#define DVP_SDA   40
#define DVP_SCL   41
#define DVP_RST   42
#define DVP_VSYNC 43
#define DVP_PWDN  44
#define DVP_HSYNC 45
#define DVP_XCLK  46
#define DVP_PCLK  47

static inline void fpgio_init(void) {
    fpioa_set_function(I2C_SCL, FUNC_I2C0_SCLK);
    fpioa_set_function(I2C_SDA, FUNC_I2C0_SDA);
    fpioa_set_function(IST_RSTn, FUNC_GPIOHS0 + IST_RSTn_GPIOHS);
    // fpioa_set_function(IST_DRDY, FUNC_GPIOHS0 + IST_DRDY_GPIOHS);
    fpioa_set_function(SPI_CLK, FUNC_SPI0_SCLK);
    fpioa_set_function(SPI_MOSI, FUNC_SPI0_D0);
    fpioa_set_function(SPI_MISO, FUNC_SPI0_D1);
    fpioa_set_function(BMI_Acc_CS, FUNC_SPI0_SS0);
    fpioa_set_function(BMI_Gyr_CS, FUNC_SPI0_SS1);
    fpioa_set_function(PMW_CS, FUNC_GPIOHS0 + PMW_NCS_GPIOHS);
    fpioa_set_function(WIFI_RDY, FUNC_GPIOHS0 + WIFI_RDY_GPIOHS);
    fpioa_set_function(WIFI_IRQ, FUNC_GPIOHS0 + WIFI_IRQ_GPIOHS);
    fpioa_set_function(WIFI_EN, FUNC_GPIOHS0 + WIFI_EN_GPIOHS);
    fpioa_set_function(WIFI_SCK, FUNC_SPI1_SCLK);
    fpioa_set_function(WIFI_MOSI, FUNC_SPI1_D0);
    fpioa_set_function(WIFI_MISO, FUNC_SPI1_D1);
    fpioa_set_function(LED1_PIN, FUNC_GPIOHS0 + LED1_GPIOHS);
    fpioa_set_function(LED2_PIN, FUNC_GPIOHS0 + LED2_GPIOHS);
    fpioa_set_function(LED3_PIN, FUNC_GPIOHS0 + LED3_GPIOHS);
    fpioa_set_function(SWITCH_PIN, FUNC_GPIO0 + SWITCH_GPIO);
    fpioa_set_function(RGB_R_PIN, FUNC_TIMER1_TOGGLE1);
    fpioa_set_function(RGB_G_PIN, FUNC_TIMER1_TOGGLE2);
    fpioa_set_function(RGB_B_PIN, FUNC_TIMER1_TOGGLE3);
    fpioa_set_function(5, FUNC_UART3_TX);
    fpioa_set_function(4, FUNC_UART3_RX);
    
    fpioa_set_function(DVP_SDA  , FUNC_I2C2_SDA);
    fpioa_set_function(DVP_SCL  , FUNC_I2C2_SCLK);
    fpioa_set_function(DVP_RST  , FUNC_CMOS_RST);
    fpioa_set_function(DVP_VSYNC, FUNC_CMOS_VSYNC);
    fpioa_set_function(DVP_PWDN , FUNC_CMOS_PWDN);
    fpioa_set_function(DVP_HSYNC, FUNC_CMOS_HREF);
    fpioa_set_function(DVP_XCLK , FUNC_CMOS_XCLK);
    fpioa_set_function(DVP_PCLK , FUNC_CMOS_PCLK);
}

#endif
