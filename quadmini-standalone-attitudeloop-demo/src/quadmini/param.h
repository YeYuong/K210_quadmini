#ifndef _PARAM_
#define _PARAM_

#include <control.h>
#include <esp8266.h>
#include <imu.h>

/*
开机读取flash中内容，校验是否存在正确参数包，若存在则读到内存，否则将程序里的参数包写入flash

*/

typedef struct
{
    uint32_t header; // 头部校验
    uint16_t Kuadmini_ID;
    uint8_t send_enable;
    uint8_t image_transmit_enable;
    struct vector_f_3 gyro_zero;
    wifi_connect_t wifi_saved_data;
    struct pid_data_ty pid_saved_data;
    uint32_t tail; // 尾部校验
} param_save_t;

typedef enum
{
    READ_PARAM,
    WRITE_PARAM
} param_cmd_enum;

typedef struct
{
    uint32_t cmd;
    param_save_t new_param;
} param_cmd_t;


extern param_save_t global_param;

void param_init(void);
void param_refresh(void);
void param_cmd_handler(param_cmd_t *data);
void param_write_require(param_save_t *new_param);
void param_read_require(void);
void serial_comm_init(void);
void serial_transmit(uint8_t *str, size_t len);



#endif
