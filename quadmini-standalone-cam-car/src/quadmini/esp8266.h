#ifndef ESP8266_H
#define ESP8266_H

#include <stdio.h>

#define ESP8266_USE_SPI SPI_DEVICE_1
#define ESP8266_USE_UART UART_DEVICE_2
#define ESP8266_USE_DMA DMAC_CHANNEL1
#define CS_ESP SPI_CHIP_SELECT_0

/*
ESP protocol

传输：
传输请求：程序中随处调用
将数据搬运到动态分配的缓冲区，调用发送函数
发送函数：
判断RDY是否为低 直接调中断函数
否则开启RDY下降沿中断
中断函数内：
若缓冲区不为空：发送flag置1
配置SPI-DMA发送
发送完成中断：
若有接收任务（IRQ高电平）则调用接收函数
否则检查缓冲区，继续调用发送函数


发送缓冲区：大小64byte * len	以64byte为单位 64byte数组（动态分配）+指针数组
缓冲区长度大于一帧图片大小
调用一次发送函数：
若小于64字节，塞入缓冲区空位
若大于64字节，分批塞入缓冲区空位
若缓冲区满，返回发送失败（尽量避免发生）

接收：
IRQ上升沿中断
若不正在发送，直接调用接收函数
接收函数：
分配接收内存
配置SPI-DMA接收
接收完成DMA中断：
解析帧有效信息，写入接收缓冲区
检查发送缓冲区，调用发送函数
无发送任务：
判断IRQ电平，调用接收函数

接收缓冲区：	4096 Byte
缓冲区长度大于程序包大小
*/
#define EPS_SPI_CLK_RATE 10000000

#define SPI_T_CMD 0x00000200
#define SPI_R_CMD 0x00000300

#define TX_BUF_LEN 100
#define RX_BUF_LEN 4

typedef struct {
	uint8_t p;  //当前缓冲区尾部的空闲位置
	uint32_t *buf[TX_BUF_LEN];
} tx_buffer_t;

typedef struct {
	uint16_t h;  //当前缓冲区头部数据位置
	uint16_t t;  //当前缓冲区尾部空闲位置
	uint16_t n;  //数据总量
	uint32_t temp[16];
	uint8_t buf[RX_BUF_LEN];
} rx_buffer_t;

typedef enum {
	PACKINFO_NORM,
	PACKINFO_CMD,
	PACKINFO_2,
	PACKINFO_3,
} packinfo_type_enum;

typedef struct 
{
	char wifi_name[24];
	char password[24];
	char ip_address[16];
	char gateway[16];
    uint16_t use_tcp;
} wifi_connect_t;

extern volatile packinfo_type_enum packinfo_type;

extern struct ring_buffer_ty rx_buf;
extern struct ring_buffer_ty tx_buf;
extern volatile int transmitting_flag;

#define QUEUE_ADD(var, val) var = (var + val) % RX_BUF_LEN  //队列首尾标志自增

void esp8266_init(const char *wifi_name, const char *password, const char *ip_address, const char *gateway, uint8_t use_tcp);
void esp_send_command(const char *cmd);
int esp_send(uint8_t *msg, size_t len);
int esp_recv(uint8_t *msg, size_t len);
void esp_test(void);

#endif
