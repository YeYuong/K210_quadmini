#ifndef SPI_SW_H
#define SPI_SW_H
#include <stdio.h>
#include <gpiohs.h>
#include <io_func.h>
#include <spi.h>

/*software spi pola=0 phase=0*/

#define USE_SOFTWARE_SPI

#define SPI_SW_DELAY 80     // 0:5MHz | 40:2MHz


void spi_sw_init(void);

void spi_sw_send_data_standard(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *cmd_buff, size_t cmd_len, const uint8_t *tx_buff, size_t tx_len);

void spi_sw_receive_data_standard(spi_device_num_t spi_num, spi_chip_select_t chip_select, const uint8_t *cmd_buff, size_t cmd_len, uint8_t *rx_buff, size_t rx_len);

#endif
