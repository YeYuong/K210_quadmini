#include <spi_sw.h>
#include <sleep.h>
#include <BMI088.h>
#include "sysctl.h"
// #define SPI_DO(pv) gpiohs_set_pin(SPI_MISO_PIN, pv)
// #define GPIOHS_SET(io_num) *(uint32_t*)(0x38001000u + 0x0cu) |=  (0x1u<<(io_num))
// #define GPIOHS_RESET(io_num) *(uint32_t*)(0x38001000u + 0x0cu) &= ~(0x1u<<(io_num))
// #define GPIOHS_GET(io_num) ((*(uint32_t*)(0x38001000u) & (0x1u<<(io_num))) == 0 ? 0x00u : 0x01u)
// #define SPI_CLK_0() GPIOHS_RESET(SPI_CLK_GPIOHS)
// #define SPI_CLK_1() GPIOHS_SET(SPI_CLK_GPIOHS)
// #define SPI_MOSI_0() GPIOHS_RESET(SPI_MOSI_GPIOHS)
// #define SPI_MOSI_1() GPIOHS_SET(SPI_MOSI_GPIOHS)
// #define SPI_MOSI_V(pv) (pv)==0?(SPI_MOSI_0()):(SPI_MOSI_1())
// #define SPI_MISO_V() GPIOHS_GET(SPI_MISO_GPIOHS)
// #define SPI_CS_V(io_num, pv) (pv)==0?(GPIOHS_RESET(io_num)):(GPIOHS_SET(io_num))
#define SPI_CLK_0() gpiohs_set_pin(SPI_CLK_GPIOHS, 0)
#define SPI_CLK_1() gpiohs_set_pin(SPI_CLK_GPIOHS, 1)
#define SPI_MOSI_V(pv) gpiohs_set_pin(SPI_MOSI_GPIOHS, pv)
#define SPI_MISO_V() gpiohs_get_pin(SPI_MISO_GPIOHS)
#define SPI_CS_V(io_num, pv) gpiohs_set_pin(io_num, pv)

void spi_sw_delay(void)
{
    uint64_t cycle = read_cycle();
    while (1)
    {
        if(read_cycle() - cycle >= SPI_SW_DELAY)
            break;
    }
}

void spi_sw_init(void)
{
    // remap IO function
    fpioa_set_function(SPI_CLK, FUNC_GPIOHS0 + SPI_CLK_GPIOHS);
    fpioa_set_function(SPI_MOSI, FUNC_GPIOHS0 + SPI_MOSI_GPIOHS);
    fpioa_set_function(SPI_MISO, FUNC_GPIOHS0 + SPI_MISO_GPIOHS);
    fpioa_set_function(BMI_Acc_CS, FUNC_GPIOHS0 + BMI_Acc_CS_GPIOHS);
    fpioa_set_function(BMI_Gyr_CS, FUNC_GPIOHS0 + BMI_Gyr_CS_GPIOHS);
    fpioa_set_function(PMW_CS, FUNC_GPIOHS0 + PMW_NCS_GPIOHS);

    // configure IO drive mode
    gpiohs_set_drive_mode(SPI_CLK_GPIOHS, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(SPI_MOSI_GPIOHS, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(SPI_MISO_GPIOHS, GPIO_DM_INPUT_PULL_UP);
    gpiohs_set_drive_mode(BMI_Acc_CS_GPIOHS, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(BMI_Gyr_CS_GPIOHS, GPIO_DM_OUTPUT);
    gpiohs_set_drive_mode(PMW_NCS_GPIOHS, GPIO_DM_OUTPUT);

    // preset IO value
    gpiohs_set_pin(SPI_CLK_GPIOHS, GPIO_PV_LOW);
    gpiohs_set_pin(SPI_MOSI_GPIOHS, GPIO_PV_LOW);
    gpiohs_set_pin(BMI_Acc_CS_GPIOHS, GPIO_PV_HIGH);
    gpiohs_set_pin(BMI_Gyr_CS_GPIOHS, GPIO_PV_HIGH);
    gpiohs_set_pin(PMW_NCS_GPIOHS, GPIO_PV_HIGH);

    // while ((1))
    // {
    //     SPI_CLK_0();
    //     spi_sw_delay();
    //     SPI_CLK_1();
    //     spi_sw_delay();
    // }
    
}


void spi_sw_send_data_standard(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *cmd_buff, size_t cmd_len, const uint8_t *tx_buff, size_t tx_len)
{
    uint8_t data_out;
    if(chip_select == CS_ACCEL) {
        SPI_CS_V(BMI_Acc_CS_GPIOHS, 0);
        spi_sw_delay();
    } else if(chip_select == CS_GYRO) {
        SPI_CS_V(BMI_Gyr_CS_GPIOHS, 0);
        spi_sw_delay();
    }
    for (size_t i = 0; i < cmd_len; ++i) {
        data_out = cmd_buff[i];
        for (int j = 0; j < 8; ++j, data_out <<= 1) {
            SPI_MOSI_V((data_out >> 7) & 1);
            spi_sw_delay();
            SPI_CLK_1();
            spi_sw_delay();
            SPI_CLK_0();
        }
    }
    for (size_t i = 0; i < tx_len; ++i) {
        data_out = tx_buff[i];
        for (int j = 0; j < 8; ++j, data_out <<= 1) {
            SPI_MOSI_V((data_out >> 7) & 1);
            spi_sw_delay();
            SPI_CLK_1();
            spi_sw_delay();
            SPI_CLK_0();
        }
    }
    if(chip_select == CS_ACCEL) {
        SPI_CS_V(BMI_Acc_CS_GPIOHS, 1);
        spi_sw_delay();
    } else if(chip_select == CS_GYRO) {
        SPI_CS_V(BMI_Gyr_CS_GPIOHS, 1);
        spi_sw_delay();
    }
}

void spi_sw_receive_data_standard(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *cmd_buff, size_t cmd_len, uint8_t *rx_buff, size_t rx_len)
{
    uint8_t data_out, data_in=0;
    if(chip_select == CS_ACCEL) {
        SPI_CS_V(BMI_Acc_CS_GPIOHS, 0);
        spi_sw_delay();
        spi_sw_delay();
    } else if(chip_select == CS_GYRO) {
        SPI_CS_V(BMI_Gyr_CS_GPIOHS, 0);
        spi_sw_delay();
        spi_sw_delay();
    }
    for (size_t i = 0; i < cmd_len; ++i) {
        data_out = cmd_buff[i];
        for (int j = 0; j < 8; ++j, data_out <<= 1) {
            SPI_MOSI_V((data_out >> 7) & 1);
            spi_sw_delay();
            SPI_CLK_1();
            spi_sw_delay();
            SPI_CLK_0();
        }
    }
    spi_sw_delay();
    for (size_t i = 0; i < rx_len; ++i) {
        data_out = 0x00;
        for (int j = 0; j < 8; ++j, data_out <<= 1) {
            SPI_MOSI_V((data_out >> 7) & 1);
            spi_sw_delay();
            SPI_CLK_1();
            data_in = (data_in << 1) | SPI_MISO_V();
            spi_sw_delay();
            SPI_CLK_0();
        }
        rx_buff[i] = data_in;
    }
    if(chip_select == CS_ACCEL) {
        SPI_CS_V(BMI_Acc_CS_GPIOHS, 1);
        spi_sw_delay();
    } else if(chip_select == CS_GYRO) {
        SPI_CS_V(BMI_Gyr_CS_GPIOHS, 1);
        spi_sw_delay();
    }
}
