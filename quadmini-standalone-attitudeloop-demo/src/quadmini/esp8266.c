#include <stdlib.h>
#include <dmac.h>
#include <esp8266.h>
#include <gpiohs.h>
#include <sleep.h>
#include <spi.h>
#include <dmac.h>
#include <sysctl.h>
#include <uart.h>
#include <string.h>
#include <atomic.h>
#include <io_func.h>

#include "utils.h"
#include "ring_buffer.h"

#define ESP_RDY() gpiohs_get_pin(WIFI_RDY_GPIOHS)
#define ESP_IRQ() gpiohs_get_pin(WIFI_IRQ_GPIOHS)

enum {
	ESP_IDLE = 0,
	ESP_WRITE,
	ESP_READ
};

struct esp_pack_ty{
	uint8_t pack_info;
	uint8_t pack_data[63];
};

volatile uint8_t tx_flag = 0;

struct ring_buffer_ty tx_cmd_buf;
struct ring_buffer_ty tx_buf;
struct ring_buffer_ty rx_buf;
volatile uint8_t transmit_status = IDLE;

corelock_t esp_core_lock;

// uint32_t big_buffer[16 * 5];

void esp_transmit(void);
void esp_read(void);

// esp8266 spi-communication

void esp_transmit(void);
void esp_read(void);
int esp_dma_irq(void *ctx);

/*
void esp8266_read_multi(uint8_t *data, size_t len) {
	uint8_t send[2] = {0x03, 0x00};
	spi_receive_data_standard(ESP8266_USE_SPI, CS_ESP, send, 2, data, len);
}
void esp8266_write_multi(uint8_t *data, size_t len) {
  	uint8_t send[2] = {0x02, 0x00};
  	spi_send_data_standard(ESP8266_USE_SPI, CS_ESP, send, 2, data, len);
}
*/

//spi transmit using DMA
void esp8266_read_multi_dma(uint32_t *rx_buff, size_t rx_len) {
    volatile spi_t *spi_handle = spi[ESP8266_USE_SPI];

	set_bit(&spi_handle->ctrlr0, 3 << 8, SPI_TMOD_EEROM << 8);	//set tmod
    spi_handle->ctrlr1 = (uint32_t)(rx_len - 1);
    spi_handle->dmacr = 0x01;	//only receive dma
    spi_handle->ssienr = 0x01;

	spi_handle->dr[0] = SPI_R_CMD;	//spi command bytes

    sysctl_dma_select(ESP8266_USE_DMA, SYSCTL_DMA_SELECT_SSI0_RX_REQ + ESP8266_USE_SPI * 2);
	dmac_irq_register(ESP8266_USE_DMA, esp_dma_irq, rx_buff, 4);	//enable dma interrupt
    dmac_set_single_mode(ESP8266_USE_DMA, (void *)(&spi_handle->dr[0]), rx_buff, DMAC_ADDR_NOCHANGE, DMAC_ADDR_INCREMENT,
                         DMAC_MSIZE_1, DMAC_TRANS_WIDTH_32, rx_len);

    spi_handle->ser = 1U << CS_ESP;

	// dmac_wait_done(ESP8266_USE_DMA);

    // spi_handle->ser = 0x00;
    // spi_handle->ssienr = 0x00;
}
void esp8266_write_multi_dma(uint32_t *tx_buff, size_t tx_len)
{
	volatile spi_t *spi_handle = spi[ESP8266_USE_SPI];
	int i;
	set_bit(&spi_handle->ctrlr0, 3 << 8, SPI_TMOD_TRANS << 8);	//set tmod

	spi_handle->dmacr = 0x2; /*enable dma transmit*/
	spi_handle->ssienr = 0x01;

	spi_handle->dr[0] = SPI_T_CMD;	//spi command bytes

	sysctl_dma_select((sysctl_dma_channel_t)ESP8266_USE_DMA, SYSCTL_DMA_SELECT_SSI0_TX_REQ + ESP8266_USE_SPI * 2);
	dmac_irq_register(ESP8266_USE_DMA, esp_dma_irq, NULL, 1);	//enable dma interrupt
	dmac_set_single_mode(ESP8266_USE_DMA, tx_buff, (void *)(&spi_handle->dr[0]),
	                     DMAC_ADDR_INCREMENT, DMAC_ADDR_NOCHANGE, DMAC_MSIZE_4,
	                     DMAC_TRANS_WIDTH_32, tx_len);

	spi_handle->ser = 1U << CS_ESP;
}

// end of esp8266 spi api

void bufcpy(uint8_t *src, uint8_t *des, size_t len)
{
	for(uint32_t i = 0; i<len; i++)
	{
		*(des + i) = *(src + i);
	}
}

volatile static struct esp_pack_ty tx_pack[2];	//双缓冲区加速
volatile static int tx_pack_flag[2] = {0, 0};
volatile static struct esp_pack_ty rx_pack;

volatile int transmitting_flag = 0;
volatile int rdy_edge = 0;

void rx_handle(void)
{
	transmit_status = ESP_READ;
	transmitting_flag = 2;
	esp8266_read_multi_dma((uint32_t *)&rx_pack, 16);
}

void tx_handle(void)
{
	// int pack_index = 0;
	transmit_status = ESP_WRITE;
	transmitting_flag = 2;
	// if(tx_pack_flag[0] == 1)
	// {
	// 	pack_index = 0;
	// }
	// else if(tx_pack_flag[1] == 1)
	// {
	// 	pack_index = 1;
	// }
	// else
	// {
	// 	pack_index = 0;
	corelock_lock(&esp_core_lock);
	switch(packinfo_type)
	{
		case PACKINFO_NORM:
			tx_pack[0].pack_info = (uint8_t)pop(&tx_buf, (uint8_t *)(tx_pack[0].pack_data), 63);
			break;
		case PACKINFO_CMD:
			tx_pack[0].pack_info = (uint8_t)pop(&tx_cmd_buf, (uint8_t *)(tx_pack[0].pack_data), 63) | 0x40;
			// printf("cmddata:%s|||\n", tx_pack[0].pack_data);
			if(is_empty(&tx_cmd_buf))
				packinfo_type = PACKINFO_NORM;
			break;
		case PACKINFO_2:
		case PACKINFO_3:
		default:
			break;
	}
	corelock_unlock(&esp_core_lock);
	// 	tx_pack_flag[0] = 1;
	// }

	esp8266_write_multi_dma((uint32_t *)&tx_pack[0], 16);

	// int tmp = 1 - pack_index;
	// if(tx_pack_flag[tmp] == 0 && is_empty(&tx_buf) == 0)
	// {
	// 	tx_pack[tmp].pack_info = pop(&tx_buf, (uint8_t *)(tx_pack[tmp].pack_data), 63);
	// 	tx_pack_flag[tmp] = 1;
	// }

	// tx_pack_flag[pack_index] = 0;
}

void esp_rdy_handle(void)
{
	sysctl_disable_irq();
	for(int i = 10; i > 0; i--)
		;
	int can_send = ((!is_empty(&tx_cmd_buf) || !is_empty(&tx_buf)) && ESP_RDY() == 0);
	// printf("%d\n", can_send);
	int can_recv = ESP_IRQ();

	if(transmitting_flag == 2)
		return;
	// else if(transmitting_flag == 1)
	// 	transmitting_flag = 0;

	if(transmit_status == ESP_IDLE)
	{
		if(can_recv)
		{
			rx_handle();
		}
		else if(can_send)
		{
			tx_handle();
		}
		else
		{
			transmit_status = ESP_IDLE;
		}
	}
	else if(transmit_status == ESP_WRITE)
	{
		if(can_recv)
		{
			rx_handle();
		}
		else if(can_send)
		{
			tx_handle();
		}
		else
		{
			transmit_status = ESP_IDLE;
		}

	}
	else if(transmit_status == ESP_READ)
	{
		if(can_send)
		{
			tx_handle();
		}
		else if(can_recv)
		{
			rx_handle();
		}
		else
		{
			transmit_status = ESP_IDLE;
		}
	}
	sysctl_enable_irq();
}

void esp_irq_handle(void)
{
	sysctl_disable_irq();
	if(ESP_IRQ() == 1)
	{
		gpiohs_set_pin_edge(WIFI_IRQ_GPIOHS, GPIO_PE_LOW);
		gpiohs_set_irq(WIFI_IRQ_GPIOHS, 3, esp_irq_handle);
		if(transmit_status == ESP_IDLE)
			esp_rdy_handle();
	}
	else
	{
		gpiohs_set_pin_edge(WIFI_IRQ_GPIOHS, GPIO_PE_HIGH);
		gpiohs_set_irq(WIFI_IRQ_GPIOHS, 3, esp_irq_handle);
	}
	sysctl_enable_irq();
}

int esp_dma_irq(void *ctx)	//to close spi-device after transmition
{
	sysctl_disable_irq();
	transmitting_flag = 0;
	//dmac_irq_unregister(ESP8266_USE_DMA);
	if(ctx == NULL)
		while ((spi[ESP8266_USE_SPI]->sr & 0x05) != 0x04);
	spi[ESP8266_USE_SPI]->ser = 0x00;
	spi[ESP8266_USE_SPI]->ssienr = 0x00;

	if(ctx == NULL)
	{

	}
	else
	{
		push(&rx_buf, (uint8_t *)(((struct esp_pack_ty *)ctx)->pack_data), ((struct esp_pack_ty *)ctx)->pack_info);
	}
	sysctl_enable_irq();

	return 0;
}

int esp_send(uint8_t *msg, size_t len)	//发送调用函数
{
	sysctl_disable_irq();
	corelock_lock(&esp_core_lock);
	push(&tx_buf, msg, len);
	corelock_unlock(&esp_core_lock);
	sysctl_enable_irq();

	return 0;
}

int esp_recv(uint8_t *msg, size_t len)	//发送调用函数
{
	sysctl_disable_irq();
	int _len = pop(&rx_buf, msg, len);
    sysctl_enable_irq();

    return _len;
}

volatile packinfo_type_enum packinfo_type = PACKINFO_NORM;
/*给ESP8266发送控制信息
	支持的命令：
	1.wifi_init$
	2.wifi_deinit$
	3.wifi_connect$[name]$[password]$[ip]$[gw]$[tcp/udp]$
	4.wifi_disconnect$
	5.tcp_server_start$
	6.udp_server_start$
	7.tcp_server_stop$
	8.tcp_hello$
	一次发送一个命令，结尾使用标识符$，字符总数不得超过251
*/
void esp_send_command(const char *cmd)	//发送控制信息
{
	packinfo_type = PACKINFO_CMD;
	sysctl_disable_irq();
	corelock_lock(&esp_core_lock);
	push(&tx_cmd_buf, cmd, strlen(cmd));
	// push(&tx_cmd_buf, "\n", 1);
	corelock_unlock(&esp_core_lock);
	sysctl_enable_irq();
}


#include <fpioa.h>

void esp8266_spi_uart_init(void) {
	gpiohs_set_drive_mode(WIFI_EN_GPIOHS, GPIO_DM_OUTPUT);
	gpiohs_set_drive_mode(WIFI_RDY_GPIOHS, GPIO_DM_INPUT);
	gpiohs_set_drive_mode(WIFI_IRQ_GPIOHS, GPIO_DM_INPUT);
	gpiohs_set_pin_edge(WIFI_RDY_GPIOHS, GPIO_PE_FALLING);
	gpiohs_set_pin_edge(WIFI_IRQ_GPIOHS, GPIO_PE_HIGH);
	gpiohs_set_irq(WIFI_IRQ_GPIOHS, 3, esp_irq_handle);
	gpiohs_set_irq(WIFI_RDY_GPIOHS, 2, esp_rdy_handle);
	gpiohs_set_pin(WIFI_EN_GPIOHS, 0);
	msleep(500);
	gpiohs_set_pin(WIFI_EN_GPIOHS, 1);
	msleep(1000);
	fpioa_set_function(WIFI_NSS, FUNC_SPI1_SS0);

	// fpioa_set_function(11, FUNC_SPI1_SS0);
	spi_init(ESP8266_USE_SPI, SPI_WORK_MODE_0, SPI_FF_STANDARD, 32, 1);  // SPI1
	spi_set_clk_rate(ESP8266_USE_SPI, EPS_SPI_CLK_RATE);  // 1MHz/10MHz/16MHz

	// printf("esp init ok\n\r");
}

void esp8266_init(const char *wifi_name, const char *password, const char *ip_address, const char *gateway, uint8_t use_tcp)
{
	ring_buffer_init(&tx_buf, 4096 * 1000);
	ring_buffer_init(&tx_cmd_buf, 63 * 4);
	ring_buffer_init(&rx_buf, 4096 * 100);
	esp8266_spi_uart_init();
	char cmd[252];
	sprintf(cmd, "wifi_connect$%s$%s$%s$%s$%s$\n", wifi_name, password, ip_address, gateway, use_tcp?"tcp":"udp");
    printf("WIFI: | %s | %s | %s |\n", wifi_name, ip_address, use_tcp?"tcp":"udp");
	esp_send_command(cmd);
}
