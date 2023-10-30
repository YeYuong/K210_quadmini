#ifndef _DATA_EXCHANGE_H
#define _DATA_EXCHANGE_H

#include <stdio.h>

#define FRAME_HEAD      0xff7f7f01
#define FRAME_TAIL      0x3fff3f01

#define DATA_PACK_TYPE  0
#define PIC_PACK_TYPE   1
#define ACK_PACK_TYPE   2
#define CMD_PACK_TYPE   3
#define STR_PACK_TYPE   4

#define OTA_DATA_SIZE   (1024 - 4 * 4)

extern uint8_t data_transmit_enable;

struct ota_file_ty {
    uint32_t total_len;
    uint32_t index_num;
    uint8_t * index_list;
    uint32_t curr_len;
    uint16_t file_whole;
    uint16_t file_flashed;
    uint64_t start_time;
    uint8_t * data;
};

struct ota_cmd_ty {
    uint32_t total_len;
    uint32_t ota_index;
    uint32_t start_byte;
    uint32_t len;
    uint8_t data[OTA_DATA_SIZE];
};

struct ota_ack_ty {
	uint32_t total_len;
	uint32_t ota_index;
	uint32_t start_byte;
	uint32_t len;
    uint32_t end;
};

struct data_pack_ty {
    //uint64_t time;
    float data0;
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
    float data7;
    float data8;
    float data9;
    float data10;
    float data11;
    float data12;
    float data13;
    float data14;
    float data15;
    float data16;
    float data17;
    float data18;
    float data19;
    float data20;
    float data21;
    float data22;
    float data23;
    float data24;
    float data25;
    float data26;
    float data27;
    float data28;
    float data29;
    float data30;
    float data31;
    float data32;
    float data33;
    float data34;
    float data35;
    float data36;
    
    uint8_t tail[4];
};

// struct pic_pack_ty {
//     int pic_head[7];
//     uint8_t *data;
// };

struct cmd_pack_ty {
	uint32_t cmd_pack_cnt;
	uint32_t cmd;
	uint8_t data[1024];
};

struct ack_pack_ty {
	uint32_t cmd_pack_cnt;
	uint32_t cmd;
	uint8_t data[128];
};

struct frame_ty {
    uint32_t frame_head;
    uint32_t frame_cnt;
    uint16_t pack_type;
    uint16_t pack_len;
    uint16_t crc;
    uint8_t * pack;
    uint32_t frame_tail;
};

extern struct ota_file_ty ota_file;

void data_exchange_task(void);
void send_frame(struct frame_ty *_frame);
void str_pack_send(char *str);
void pic_compressed_pack_send(uint8_t *image);
void pic_pack_send(uint8_t *image, uint16_t width, uint16_t height, uint8_t index);
void pic_pack_send_vofa(uint8_t *image, uint16_t width, uint16_t height, uint8_t index);
void pic_pack_send_kgs(uint8_t *image, uint16_t width, uint16_t height, uint8_t index);


#endif
