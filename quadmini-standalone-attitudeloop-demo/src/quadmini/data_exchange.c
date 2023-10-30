#include "data_exchange.h"
#include "data_exchange_printf.h"

#include <printf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sysctl.h>
#include <imu.h>
#include <rotors.h>
#include <atomic.h>
#include <motor.h>
#include <task.h>
#include <vl53l1x.h>
#include <pmw3901mb.h>
#include <motioncap.h>
#include <io_func.h>
#include <gpio.h>
#include <motioncap.h>
#include <led.h>
#include <param.h>
#include <ads1015.h>

#include "esp8266.h"
#include "ring_buffer.h"

uint32_t frame_count = 0;
uint32_t lost_frame = 0;
uint32_t wrong_frame = 0;

uint8_t data_transmit_enable = 1;
uint8_t pic_transmit_enable = 0;

struct ota_file_ty ota_file = {0, 0, 0, 0, 0, 0, 0, NULL};

void generate_dynamic_pic(uint8_t image[][80]) {
	// static int cnt = 0;
	static int start_x = 10, start_y = 20;
	static int flag = 0;
	if (flag == 0)
	{
		start_x += 4;
		if (start_x > 50) flag = 1;
	}
	else if (flag == 1)
	{
		start_y += 4;
		if (start_y > 60) flag = 2;
	}
	else if (flag == 2)
	{
		start_x -= 4;
		if (start_x < 10) flag = 3;
	}
	else
	{
		start_y -= 4;
		if (start_y < 20) flag = 0;
	}

	for (int i = start_x; i < start_x + 5; i++)
	{
		for (int j = start_y; j < start_y + 5; j++)
		{
			image[i][j] = 0x00;
		}
	}
}

void generate_data_pack(struct data_pack_ty *data_pack) {
	//构造数传数据包
	memset(data_pack, 0, sizeof(struct data_pack_ty));
	//data_pack->time = sysctl_get_time_us();
/*基础飞行数据*/

/*传感器测试数据*/
    // 欧拉角
	data_pack->data0 = attitude_data.heading_angle.roll;
	data_pack->data1 = attitude_data.heading_angle.pitch;
	data_pack->data2 = attitude_data.heading_angle.yaw;
	// 位置
	data_pack->data3 = getBodyPosition(MC_X);
	data_pack->data4 = getBodyPosition(MC_Y);
	data_pack->data5 = getBodyPosition(MC_Z);
    // IMU原始数据
    data_pack->data6 = attitude_data.imu.gyro.x * RAD_TO_ANG;
	data_pack->data7 = attitude_data.imu.gyro.y * RAD_TO_ANG;
	data_pack->data8 = attitude_data.imu.gyro.z * RAD_TO_ANG;
    data_pack->data9 = attitude_data.imu.acc.x;
	data_pack->data10= attitude_data.imu.acc.y;
	data_pack->data11= attitude_data.imu.acc.z;
    // 光流原始数据
    data_pack->data12= opFlow.pixSum[OP_X];
	data_pack->data13= opFlow.pixSum[OP_Y];
	data_pack->data14= getMotioncapSpeed(OP_X);
    data_pack->data15= getMotioncapSpeed(OP_Y);
	data_pack->data16= getOpFlowSpeed(OP_X); 
	data_pack->data17= getOpFlowSpeed(OP_Y);
    // TOF原始数据
	data_pack->data18= getTOFHeight();
	data_pack->data19= getObHeight();
	data_pack->data20= getObHeightSpeed();
    // 磁力计数据
	data_pack->data21= mag_data.mag_X;
	data_pack->data22= mag_data.mag_Y;
	data_pack->data23= mag_data.mag_Z;

	// 其他数据
	data_pack->data28= global_data.motor_current;
	data_pack->data29= global_data.batt_voltage;
	data_pack->data30= global_data.debug_data;

/*PID调试数据*/
	// 欧拉角
	// data_pack->data0 = attitude_data.heading_angle.roll;
	// data_pack->data1 = attitude_data.heading_angle.pitch;
	// data_pack->data2 = attitude_data.heading_angle.yaw;
	// // 位置
	// data_pack->data3 = getBodyPosition(MC_X);
	// data_pack->data4 = getBodyPosition(MC_Y);
	// data_pack->data5 = getBodyPosition(MC_Z);
	// // 位置速度
	// // data_pack->data6 = getBodySpeed(MC_X);
	// // data_pack->data7 = getBodySpeed(MC_Y);
	// // data_pack->data8 = getBodySpeed(MC_Z);
	// // 角速度环--roll
	// data_pack->data6 = global_data.body_ctrl.angle_speed_ctrl.expect_angle_speed.x;
	// data_pack->data7 = attitude_data.imu.gyro.x * RAD_TO_ANG;
	// data_pack->data8 = global_data.body_ctrl.angle_speed_ctrl.out_motor_speed.x;
	// // 角度环--roll
	// data_pack->data9 = global_data.body_ctrl.angle_ctrl.expect_angle.x;
	// data_pack->data10= attitude_data.heading_angle.roll;
	// data_pack->data11= global_data.body_ctrl.angle_ctrl.out_angle_speed.x;
	// // 高度速度环
	// data_pack->data31= global_data.body_ctrl.height_speed_ctrl.expect_height_speed;
    // data_pack->data32= getBodySpeed(MC_Z);
    // data_pack->data33= (global_data.body_ctrl.height_speed_ctrl.out_throttle-global_data.body_ctrl.thr_hover)/100.0;
	// // 高度环
    // data_pack->data34= global_data.body_ctrl.height_ctrl.expect_height;
    // data_pack->data35= getBodyPosition(MC_Z);
    // data_pack->data36= global_data.body_ctrl.height_ctrl.out_height_speed;
	// // 位置速度环--y
    data_pack->data31= global_data.body_ctrl.y_speed_ctrl.expect_y_speed;
    data_pack->data32= getBodySpeed(MC_Y);
	data_pack->data33= global_data.body_ctrl.y_speed_ctrl.out_roll;
	// // 位置环--y
	data_pack->data34= global_data.body_ctrl.y_posi_ctrl.expect_y_posi;
	data_pack->data35= getBodyPosition(MC_Y);
	data_pack->data36= global_data.body_ctrl.y_posi_ctrl.out_y_sp;
	// // 光流数据
	// data_pack->data24= getOpFlowPosition(OP_X); 
	// data_pack->data25= getOpFlowPosition(OP_Y);
	// data_pack->data26= getOpFlowSpeed(OP_X); 
	// data_pack->data27= getOpFlowSpeed(OP_Y);
	// // 其他数据
	// data_pack->data28= global_data.motor_current;
	// data_pack->data29= global_data.batt_voltage;
	// data_pack->data30= global_data.debug_data;
 
/*帧尾*/
	data_pack->tail[0] = 0x00;data_pack->tail[1] = 0x00;data_pack->tail[2] = 0x80;data_pack->tail[3] = 0x7f;
}

// void generate_compressed_pic_pack(struct pic_pack_ty *pic_pack) {
// 	//构造图传数据包
// 	uint8_t image[60][80];
// 	//生成图像
// 	memset(image, 0xff, 60 * 80);
// 	generate_dynamic_pic(image);
// 	//压缩图像
// 	for (int i = 0; i < 60; i++) {
// 		for (int j = 0; j < 10; j++) {
// 			uint8_t tmp = 0;
// 			for (int w = 7; w > -1; w--) {
// 				tmp = tmp << 1;
// 				tmp |= (image[i][8 * j + w] & 1);
// 			}
// 			pic_pack->data[i * 10 + j] = tmp;
// 		}
// 	}
// }

corelock_t core_lock;

void send_frame(struct frame_ty *_frame) {
    corelock_lock(&core_lock);
	esp_send((uint8_t *)&(_frame->frame_head),
		   sizeof(_frame->frame_head) + sizeof(_frame->frame_cnt) +
			   sizeof(_frame->pack_type) + sizeof(_frame->pack_len) +
			   sizeof(_frame->crc));
	esp_send((uint8_t *)(_frame->pack), _frame->pack_len);
	esp_send((uint8_t *)&(_frame->frame_tail), sizeof(_frame->frame_tail));
    corelock_unlock(&core_lock);
}

void data_pack_send(struct data_pack_ty data_pack) {
	struct frame_ty frame;
	frame.frame_head = FRAME_HEAD;
	frame.frame_cnt = ++frame_count;
	frame.pack_type = DATA_PACK_TYPE;
	frame.pack_len = sizeof(data_pack);
	frame.crc = 0;
	frame.pack = (uint8_t *)&data_pack;
	frame.frame_tail = FRAME_TAIL;
	send_frame(&frame);
}

void pic_compressed_pack_send(uint8_t *image) {
	struct frame_ty frame;
	frame.frame_head = FRAME_HEAD;
	frame.frame_cnt = ++frame_count;
	frame.pack_type = PIC_PACK_TYPE;
	frame.pack_len = 600;
	frame.crc = 0;
	frame.pack = image;
	frame.frame_tail = FRAME_TAIL;
	send_frame(&frame);
}

void pic_pack_send(uint8_t *image, uint16_t width, uint16_t height, uint8_t index) {
	uint32_t pic_head[7];
    pic_head[0] = index;
    pic_head[1] = width*height;
    pic_head[2] = width;
    pic_head[3] = height;
    pic_head[4] = 24;
    pic_head[5] = 0x7F800000;
    pic_head[6] = 0x7F800000;
	struct frame_ty frame;
	frame.frame_head = FRAME_HEAD;
	frame.frame_cnt = ++frame_count;
	frame.pack_type = DATA_PACK_TYPE; //datapack数据会透传给vofa
	frame.pack_len = 7 * 4 + width * height;
	frame.crc = 0;
	frame.pack = image;
	frame.frame_tail = FRAME_TAIL;
    corelock_lock(&core_lock);
	esp_send((uint8_t *)&(frame.frame_head),
		   sizeof(frame.frame_head) + sizeof(frame.frame_cnt) +
			   sizeof(frame.pack_type) + sizeof(frame.pack_len) +
			   sizeof(frame.crc));
	esp_send((uint8_t *)(pic_head), 7 * 4);
	esp_send((uint8_t *)(image), width * height);
	esp_send((uint8_t *)&(frame.frame_tail), sizeof(frame.frame_tail));
    corelock_unlock(&core_lock);
	// send_frame(&frame);
}


void pic_pack_send_vofa(uint8_t *image, uint16_t width, uint16_t height, uint8_t index) {
	uint32_t pic_head[7];
    pic_head[0] = index;
    pic_head[1] = width*height;
    pic_head[2] = width;
    pic_head[3] = height;
    pic_head[4] = 24;
    pic_head[5] = 0x7F800000;
    pic_head[6] = 0x7F800000;
    corelock_lock(&core_lock);
	esp_send((uint8_t *)(pic_head), 7 * 4);
	esp_send((uint8_t *)(image), width * height);
    corelock_unlock(&core_lock);
	// send_frame(&frame);
}

void pic_pack_send_kgs(uint8_t *image, uint16_t width, uint16_t height, uint8_t index) {
	uint32_t pic_head[4];
    pic_head[0] = index;
    pic_head[1] = width;
    pic_head[2] = height;
	pic_head[3] = 0;
	struct frame_ty frame;
	frame.frame_head = FRAME_HEAD;
	frame.frame_cnt = ++frame_count;
	frame.pack_type = PIC_PACK_TYPE;
	frame.pack_len = 4 * 4 + width * height;
	frame.crc = 0;
	frame.pack = image;
	frame.frame_tail = FRAME_TAIL;
    corelock_lock(&core_lock);
	esp_send((uint8_t *)&(frame.frame_head),
		   sizeof(frame.frame_head) + sizeof(frame.frame_cnt) +
			   sizeof(frame.pack_type) + sizeof(frame.pack_len) +
			   sizeof(frame.crc));
	esp_send((uint8_t *)(pic_head), 4 * 4);
	esp_send((uint8_t *)(image), width * height);
	esp_send((uint8_t *)&(frame.frame_tail), sizeof(frame.frame_tail));
    corelock_unlock(&core_lock);
	// send_frame(&frame);
}

void ack_pack_send(struct ack_pack_ty ack_pack) {
	struct frame_ty frame;
	frame.frame_head = FRAME_HEAD;
	frame.frame_cnt = ++frame_count;
	frame.pack_type = ACK_PACK_TYPE;
	frame.pack_len = sizeof(ack_pack);
	frame.crc = 0;
	frame.pack = (uint8_t *)&ack_pack;
    frame.frame_tail = FRAME_TAIL;
	send_frame(&frame);
}

void str_pack_send(char *str) {
	struct frame_ty frame;
	frame.frame_head = FRAME_HEAD;
	frame.frame_cnt = ++frame_count;
	frame.pack_type = STR_PACK_TYPE;
	frame.pack_len = strlen(str) + 1;
	frame.crc = 0;
	frame.pack = (uint8_t *)str;
	frame.frame_tail = FRAME_TAIL;
	send_frame(&frame);
}

void cmd_pack_handle(struct cmd_pack_ty cmd_pack) {
	struct ack_pack_ty ack_pack;
	ack_pack.cmd_pack_cnt = cmd_pack.cmd_pack_cnt;
	ack_pack.cmd = cmd_pack.cmd;

	enum cmd_type_enum
	{
		OTA_CMD,
		REMOTE_CONTROL_CMD,
		COMMANDER_CMD,
		MOTIONCAP_DATA_CMD,
		PARAM_CMD
	};
	if (cmd_pack.cmd == OTA_CMD)
	{
		data_transmit_enable = pic_transmit_enable = 0;
		struct ota_ack_ty *ota_ack = (struct ota_ack_ty *)ack_pack.data;
		struct ota_cmd_ty *ota_cmd = (struct ota_cmd_ty *)cmd_pack.data;
		if (ota_file.data == NULL)
		{
			ota_file.start_time = sysctl_get_time_us();
			ota_file.total_len = ota_cmd->total_len;
			ota_file.data = malloc(ota_file.total_len);
			ota_file.index_num = (ota_cmd->total_len + OTA_DATA_SIZE - 1) / OTA_DATA_SIZE;
			ota_file.index_list = (uint8_t *)malloc(ota_file.index_num);
            memset(ota_file.index_list, 0, ota_file.index_num);
		}
        // printf("ota recv pack(id:%d), pack_num:%d\n\r", ota_cmd->ota_index, ota_file.index_num);

		for (int i = 0; i < ((struct ota_cmd_ty *)cmd_pack.data)->len; i++)
		{
			ota_file.data[((struct ota_cmd_ty *)cmd_pack.data)->start_byte + i] = ((struct ota_cmd_ty *)cmd_pack.data)->data[i];
		}
		ota_file.index_list[ota_cmd->ota_index] = 1;
		ota_file.curr_len += ((struct ota_cmd_ty *)cmd_pack.data)->len;

		int tmp = 1;
        // printf("lost id:");
		for (int i = 0; i < ota_file.index_num; i++)
		{
			if (ota_file.index_list[i] == 0) {
                tmp = 0;
                // printf("%d ", i);
            }
		}
        // printf("\n\r");

		if (ota_file.curr_len >= ota_file.total_len && tmp == 1) {
			ota_file.file_whole = 1;
            ota_ack->end = 1;
        } else {
            ota_ack->end = 0;
        }

		ota_ack->total_len = ((struct ota_cmd_ty *)cmd_pack.data)->total_len;
		ota_ack->ota_index = ((struct ota_cmd_ty *)cmd_pack.data)->ota_index;
		ota_ack->start_byte = ((struct ota_cmd_ty *)cmd_pack.data)->start_byte;
		ota_ack->len = ((struct ota_cmd_ty *)cmd_pack.data)->len;

		ack_pack_send(ack_pack);
	}
	else if (cmd_pack.cmd == REMOTE_CONTROL_CMD)
	{
		struct rmctrl_data_ty
		{
			float axis_[6];
			uint8_t button_[9];
			uint8_t connect_state_;
		} *rmctrl_data = (struct rmctrl_data_ty *)cmd_pack.data;
		enum btn_name_enum
		{
			BTN_A,
			BTN_B,
			BTN_X,
			BTN_Y,
			BTN_LB,
			BTN_RB,
			BTN_BACK,
			BTN_START,
			BTN_UNKNOWN,
		};
		global_data.remote_ctrl.yaw = -rmctrl_data->axis_[0];
		global_data.remote_ctrl.throttle = -rmctrl_data->axis_[1];
		global_data.remote_ctrl.roll = rmctrl_data->axis_[2];
		global_data.remote_ctrl.pitch = -rmctrl_data->axis_[3];
		global_data.remote_ctrl.lock_btn = rmctrl_data->button_[BTN_A];
		global_data.remote_ctrl.unlock_btn = rmctrl_data->button_[BTN_B];
		global_data.remote_ctrl.takeoff_btn = rmctrl_data->button_[BTN_Y];
		global_data.remote_ctrl.land_btn = rmctrl_data->button_[BTN_X];
		global_data.remote_ctrl.restart_btn = rmctrl_data->button_[BTN_LB];
		global_data.remote_ctrl.cali_btn = rmctrl_data->button_[BTN_RB];
	}
	else if (cmd_pack.cmd == COMMANDER_CMD)
	{
		command_executor((commander_t *)cmd_pack.data);
	}
	else if (cmd_pack.cmd == MOTIONCAP_DATA_CMD)
	{
		motioncap_on_pack_received((void *)cmd_pack.data);
	}
	else if (cmd_pack.cmd == PARAM_CMD)
	{
		param_cmd_handler((void *)cmd_pack.data);
	}

	// if (data_transmit_enable)
		// ack_pack_send(ack_pack);
}



uint16_t crc16(const uint8_t *buf, int len) {
	static const uint16_t crc16tab[256] = {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0 };
    int counter;
    uint16_t crc = 0;
    for (counter = 0; counter < len; counter++)
            crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++)&0x00FF];
    return crc;
}

void cmd_pack_recv(void) {
	static struct frame_ty frame;
	static struct cmd_pack_ty cmd_pack;
	static int frame_state = 0;
	int recv_size = available_pop_bytes(&rx_buf);
	uint8_t *recv_buf = (unsigned char *)malloc(recv_size);
	pop(&rx_buf, recv_buf, recv_size);

	frame.pack = (uint8_t *)&cmd_pack;

	for (int i = 0; i < recv_size; i++) {
		// printk("-0x%02x 0x%02x 0x%02x 0x%02x\n\r", ((uint8_t
		// *)frame.frame_head)[0], ((uint8_t *)frame.frame_head)[1], ((uint8_t
		// *)frame.frame_head)[2], ((uint8_t *)frame.frame_head)[3], recv_buf[i]);

		if (frame_state == 0)  //解析到帧头
		{
			uint8_t *p = (uint8_t *)&frame.frame_head;
			p[0] = p[1];
			p[1] = p[2];
			p[2] = p[3];
			p[3] = recv_buf[i];
			if (frame.frame_head == FRAME_HEAD) {
				// printf("Find head\n\r");
				led_set(1, -1);
				frame_state = 1;
			}
		}
		else if (frame_state == 1)  //解析帧序、包类型、包长度、crc
		{
			static int index = 0;
			uint8_t *p = (uint8_t *)&frame.frame_cnt;
			// printf("0x%02x ", recv_buf[i]);
			p[index++] = recv_buf[i];
			if (index == sizeof(frame.frame_cnt) + sizeof(frame.pack_type) +
						   sizeof(frame.pack_len) + sizeof(frame.crc))
			{
				// printf("%d %d %d %d\n\r"
				//         , frame.frame_cnt
				//         , frame.pack_type
				//         , frame.pack_len
				// 		, frame.crc);
				index = 0;

				static uint32_t last_frame_cnt = 0;
				uint32_t frame_cnt = frame.frame_cnt;
				if (frame_cnt > last_frame_cnt && last_frame_cnt != 0)
				  lost_frame += frame_cnt - last_frame_cnt - 1;
				last_frame_cnt = frame_cnt;

				if (frame.pack_len > 1500 || frame.pack_len == 0)  //帧长错误
				{
					wrong_frame++;
					lost_frame++;
					frame_state = 0;
				}
				else
				{
					frame_state = 2;
				}
			}
		}
		else if (frame_state == 2)  //解析pack
		{
			static int index = 0;
			uint8_t *p = (uint8_t *)frame.pack;
			p[index++] = recv_buf[i];
			// printf("0x%02x ", recv_buf[i]);
			if (index == frame.pack_len)
			{
				index = 0;
				frame_state = 3;
				// printf("\n");
			}
		}
		else if (frame_state == 3)  //解析帧尾
		{
			static int index = 0;
			uint8_t *p = (uint8_t *)&frame.frame_tail;
			p[index++] = recv_buf[i];
			// printf("0x%02x ", recv_buf[i]);
			if (index == sizeof(frame.frame_tail))
			{
				// printk("Find tail\n\r");
				index = 0;
				if (frame.frame_tail != FRAME_TAIL)  //帧尾错误
				{
                    printf("Wrong frame\n\r");
					wrong_frame++;
					lost_frame++;
					frame_state = 0;
				}
				else if (crc16(frame.pack, frame.pack_len) != frame.crc)
				{
					printf("Wrong crc\n\r");
					wrong_frame++;
					lost_frame++;
					frame_state = 0;
				}
				else
				{
					// printf("crc:%f\n", crc16(frame.pack, frame.pack_len)/10000.0);
					cmd_pack_handle(*(struct cmd_pack_ty *)frame.pack);
					frame_state = 0;
				}
			}
		}
	}

  free(recv_buf);
}

void data_exchange_task(void) {
	struct data_pack_ty data_pack;
	// struct pic_pack_ty pic_pack;
	// generate_pic_pack(&pic_pack);

	if (data_transmit_enable)
	{
		generate_data_pack(&data_pack);
		data_pack_send(data_pack);
	}
	// if (pic_transmit_enable) pic_pack_send(pic_pack);

	cmd_pack_recv();

	// 	printw("WaitToTx: %d(%d) WaitToRx: %d(%d) Lost: %u Wrong: %u Trans: %d\n\r",
	// 		   available_pop_bytes(&tx_buf), tx_buf.size,
	// 		   available_pop_bytes(&rx_buf), rx_buf.size, lost_frame, wrong_frame,
	// 		   transmitting_flag);
}

