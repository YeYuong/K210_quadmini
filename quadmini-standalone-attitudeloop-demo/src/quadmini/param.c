#include "w25qxx.h"
#include <param.h>
#include <task.h>
#include <printf.h>
#include <data_exchange_printf.h>
#include <data_exchange.h>

// 在外置flash上1M的位置存储参数（程序大小约120kb）
#define PARAM_FLASH_ADDR 256 * w25qxx_FLASH_SECTOR_SIZE
// #define PARAM_HEADER 0x1234
#define PARAM_HEADER 0xB38F
#define PARAM_TAIL 0xF1CD

// 默认初始参数，如需烧录改参，可以修改头尾定义烧录一次，再改回原定义重新烧录
param_save_t global_param = {
    .header = PARAM_HEADER,
    .Kuadmini_ID = 0,
    .send_enable = 1,
    .image_transmit_enable = 0,
    .gyro_zero = {0.0, 0.0, 0.0},
    .wifi_saved_data = {
        .wifi_name = "ETLAB410-2",
        .password = "qwe~~asd",
        .ip_address = "192.168.0.170",
        .gateway = "192.168.0.1",
        .use_tcp = 1,
    },
    // .wifi_saved_data = {
    //     .wifi_name = "HYC",
    //     .password = "13913927559",
    //     .ip_address = "192.168.2.160",
    //     .gateway = "192.168.2.1",
    //     .use_tcp = 1,
    // },
    .pid_saved_data = {
		.angle_speed_pid_param[0] = {0.7, 0.1, 0.007},
		.angle_speed_pid_param[1] = {0.7, 0.1, 0.007},
		.angle_speed_pid_param[2] = {2.0, 2.5, 0.0},

		.angle_pid_param[0] = 		{6.0, 0.1, 0.0},
		.angle_pid_param[1] = 		{6.0, 0.1, 0.0},
		.angle_pid_param[2] = 		{8.0, 0.0, 0.0},

		.height_speed_pid_param = 	{250.0, 30.0, 8.0},
		.height_pid_param = 		{2.5, 0.0, 0.0},

		.x_speed_pid_param =		{25.0, 2.0, 12.0},
		.y_speed_pid_param =		{25.0, 2.0, 12.0},

		.x_posi_pid_param = 		{1.2, 0.0, 0.5},
		.y_posi_pid_param = 		{1.2, 0.0, 0.5},
		.thr_hover = 40
    },
    .tail = PARAM_TAIL,
};

void param_refresh(void)
{ // 刷新程序变量
	data_transmit_enable = global_param.send_enable;
	attitude_data.imu.gyro_zero = global_param.gyro_zero;
	pid_data = global_param.pid_saved_data;
	ctrl_init(&global_data.body_ctrl);
}

void param_init(void)
{
    w25qxx_init(3, 0, 60000000);
    // 读参数结构体到内存
    param_save_t param_buffer;
    w25qxx_read_data(PARAM_FLASH_ADDR, (uint8_t *)&param_buffer, sizeof(param_save_t));
    if(param_buffer.header == PARAM_HEADER && param_buffer.tail == PARAM_TAIL)
    { // 存在存储的参数
        global_param = param_buffer;
        printf("read param from flash\n");
    }
    else
    { // 不存在参数存储
        w25qxx_write_data(PARAM_FLASH_ADDR, (uint8_t *)&global_param, sizeof(param_save_t));
        printf("write param to flash\n");
    }
	param_refresh();
}

void param_read_require(void)
{
	if(global_data.body_ctrl.flight_status == FLIGHT_LOCKED)
    {
        printf("send params to Kground!\n");
		struct frame_ty frame;
        frame.frame_head = FRAME_HEAD;
        frame.frame_cnt = 0;
        frame.pack_type = CMD_PACK_TYPE;
        frame.pack_len = sizeof(param_save_t);
        frame.crc = 0;
        frame.pack = (uint8_t *)&global_param;
        frame.frame_tail = FRAME_TAIL;
        send_frame(&frame);
        if(global_data.flags.serial_connected)
        {
            serial_transmit((uint8_t *)&(frame.frame_head), sizeof(frame.frame_head) + sizeof(frame.frame_cnt) +
                       			   sizeof(frame.pack_type) + sizeof(frame.pack_len) +
                                   			   sizeof(frame.crc));
            serial_transmit((uint8_t *)(frame.pack), frame.pack_len);
            serial_transmit((uint8_t *)&(frame.frame_tail), sizeof(frame.frame_tail));
        }
    }
}

void param_write_require(param_save_t *new_param)
{
	if(global_data.body_ctrl.flight_status == FLIGHT_LOCKED && new_param->header == PARAM_HEADER && new_param->tail == PARAM_TAIL)
    {
        if(global_param.wifi_saved_data.use_tcp != new_param->wifi_saved_data.use_tcp) {
            if(new_param->wifi_saved_data.use_tcp)
                esp_send_command("tcp_server_start$");
            else
                esp_send_command("udp_server_start$");
        }
        global_param = *new_param;
		param_refresh();
		w25qxx_write_data(PARAM_FLASH_ADDR, (uint8_t *)&global_param, sizeof(param_save_t));
        printf("overwrite param!\n");
        param_read_require();
    }
}

void param_cmd_handler(param_cmd_t *data)
{
    switch (data->cmd)
    {
    case READ_PARAM:
        param_read_require();
        break;
    case WRITE_PARAM:
        param_write_require(&data->new_param);
        break;
    
    default:
        break;
    }
}

#include <uart.h>
#include <string.h>
#include <esp8266.h>
#include <ring_buffer.h>
char serial_buf[256];

void serial_transmit(uint8_t *str, size_t len)
{
    uart_send_data(UART_DEVICE_3, str, len);
}

int on_uart_recv(void* ctx)
{
    int len = uart_receive_data(UART_DEVICE_3, serial_buf, 256);
    if(global_data.flags.serial_connected)
    {
        if(len == 6)
        {
            char cmd[10];
            sscanf(serial_buf, "$%[a-z]", cmd);
            if(strcmp(cmd, "quit") == 0)
            {
                global_data.flags.serial_connected = 0;
                uart_send_data(UART_DEVICE_3, "Serial Disconnect\n", 19);
            }
        }
        push(&rx_buf, serial_buf, len);
    }
    else
    {
        char cmd[10];
        sscanf(serial_buf, "$%[a-z]", cmd);
        if(strcmp(cmd, "enter") == 0)
        {
            global_data.flags.serial_connected = 1;
            uart_send_data(UART_DEVICE_3, "Serial Connected\n", 18);
        }
    }
    
    return 0;
    // uart_send_data(UART_DEVICE_3, serial_buf, len);
}

void serial_comm_init(void)
{
    uart_set_receive_trigger(UART_DEVICE_3, UART_RECEIVE_FIFO_8);
    global_data.flags.serial_connected = 0;
    uart_irq_register(UART_DEVICE_3, UART_RECEIVE, on_uart_recv, NULL, 2);

}