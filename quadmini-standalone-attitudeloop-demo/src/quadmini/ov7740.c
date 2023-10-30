/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for
 * details.
 *
 * OV7725 driver.
 *
 */
#include <dvp.h>
#include <i2c.h>
#include <malloc.h>
#include <ov7740.h>
#include <plic.h>
#include <sleep.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sysctl.h>
#include <uart.h>
#include <printf.h>
#include <string.h>

uint32_t *graph_buf0;
uint32_t *graph_buf1;
uint32_t graph_buf_addr;
framesize_t now_fsize;

// SCCB 读取、写入字节 使用硬件I2C

static int sccb_readb(uint8_t reg_addr, uint8_t *reg_data)
{
    int ret = 0;
    i2c_recv_data(SCCB_I2C_DEVICE_NUM, OV7740_Device_ADDR, &reg_addr, 1, reg_data, 1);
    if (0xff == *reg_data)
        ret = -1;

    return ret;
}

static int sccb_writeb(uint8_t reg_addr, uint8_t reg_data)
{
    uint8_t send[2] = {reg_addr, reg_data};
    i2c_send_data(SCCB_I2C_DEVICE_NUM, OV7740_Device_ADDR, send, 2);
    msleep(10);
    return 0;
}

// 读取芯片ID 成功返回0
int OV7740_Check(void)
{
    uint16_t device_id = 0;
    uint8_t tmp;

    if (sccb_readb(0x0A, &tmp))
        return -1;
    device_id = tmp << 8;
    if (sccb_readb(0x0B, &tmp))
        return -1;
    device_id |= tmp;

    if (device_id != OV7740_ID)
        return -1;

    return 0;
}

/********************************** OV7740 Settings *************************************/

int ov7740_reset(void)
{
    // // Reset all registers
    int ret;
    const uint8_t(*regs)[2];
    ret = sccb_writeb(0x12, 0x80);

    // Delay 2 ms
    msleep(2);

    // Write default regsiters
    regs = default_regs;
    for (int i = 0; regs[i][0]; i++)
    {
        ret |= sccb_writeb(regs[i][0], regs[i][1]);
    }

    return ret;
}

int ov7740_sleep(int enable) // 休眠模式
{
    if (enable)
    {
        dvp->cmos_cfg |= DVP_CMOS_POWER_DOWN;
    }
    else
    {
        dvp->cmos_cfg &= ~DVP_CMOS_POWER_DOWN;
    }
    return 0;
}

int set_pixformat(pixformat_t pixformat) { return 0; }

int set_framesize(framesize_t framesize)
{
    int ret = 0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];
    now_fsize = framesize;

    // VGA
    if ((w == 640) || (h == 480))
    {
        ret |= sccb_writeb(0x31, 0xA0);
        ret |= sccb_writeb(0x32, 0xF0);
        ret |= sccb_writeb(0x82, 0x32);
    }
    // QVGA
    else if ((w == 320) && (h == 240))
    {
        ret |= sccb_writeb(0x31, 0x50);
        ret |= sccb_writeb(0x32, 0x78);
        ret |= sccb_writeb(0x82, 0x3F);
    }
    // QQVGA
    else if ((w == 160) || (h == 120))
    {
        ret |= sccb_writeb(0x31, 0x28);
        ret |= sccb_writeb(0x32, 0x3c);
        ret |= sccb_writeb(0x82, 0x3F);
    }
    // QQQVGA
    else if ((w == 80) || (h == 60))
    {
        ret |= sccb_writeb(0x31, 0x14);
        ret |= sccb_writeb(0x32, 0x1E);
        ret |= sccb_writeb(0x82, 0x32);
        dvp_disable_burst(); // 需要关闭这个位，否则dvp无法设置宽度<160
    }
    else
    {
        return -1;
    }

    // 缓存空间分配 (此处仅考虑了灰度应用)
    if (graph_buf0 != NULL || graph_buf1 != NULL)
    {
        free((void *)graph_buf0);
        free((void *)graph_buf1);
    }
    graph_buf0 = (uint32_t *)malloc(w * h);
    graph_buf1 = (uint32_t *)malloc(w * h);
    graph_buf_addr = (uint32_t)(long)graph_buf0;
    g_ram_mux = 1;
    // dvp接口配置
    dvp_set_image_size(w, h);
    dvp_set_ai_addr((uint32_t)((long)graph_buf_addr), 0, 0); // AI输出地址

    msleep(30);
    return ret;
}

int set_framerate(framerate_t framerate)
{
    int ret = 0;
    switch (framerate)
    {
    case FRAMERATE_60FPS:
        ret |= sccb_writeb(0x11, 0x00);
        // ret |= sccb_writeb(0x55, 0x40);
        // ret |= sccb_writeb(0x2b, 0xF0);
        // ret |= sccb_writeb(0x2c, 0x01);
        break;
    case FRAMERATE_30FPS:
        ret |= sccb_writeb(0x11, 0x01);
        // ret |= sccb_writeb(0x55, 0x40);
        // ret |= sccb_writeb(0x2b, 0xF0);
        // ret |= sccb_writeb(0x2c, 0x01);
        break;
    case FRAMERATE_25FPS:
        ret |= sccb_writeb(0x11, 0x01);
        // ret |= sccb_writeb(0x55, 0x40);
        // ret |= sccb_writeb(0x2b, 0x5E);
        // ret |= sccb_writeb(0x2c, 0x02);
        break;
    case FRAMERATE_15FPS:
        ret |= sccb_writeb(0x11, 0x03);
        // ret |= sccb_writeb(0x55, 0x40);
        // ret |= sccb_writeb(0x2b, 0xF0);
        // ret |= sccb_writeb(0x2c, 0x01);
        break;
    default:
        return -1;
    }
    return ret;
}

int set_contrast(int level)
{
    int ret = 0;
    uint8_t tmp = 0;

    level += (NUM_CONTRAST_LEVELS / 2);
    if (level < 0 || level >= NUM_CONTRAST_LEVELS)
    {
        return -1;
    }
    ret |= sccb_readb(0x81, &tmp);
    tmp |= 0x20;
    ret |= sccb_writeb(0x81, tmp);
    ret |= sccb_readb(0xDA, &tmp);
    tmp |= 0x04;
    ret |= sccb_writeb(0xDA, tmp);
    ret |= sccb_writeb(0xE1, contrast_regs[level][0]);
    ret |= sccb_writeb(0xE2, contrast_regs[level][1]);
    ret |= sccb_writeb(0xE3, contrast_regs[level][2]);
    ret |= sccb_readb(0xE4, &tmp);
    tmp &= 0xFB;
    ret |= sccb_writeb(0xE4, tmp);

    return ret;
}

int set_brightness(int level)
{
    int ret = 0;
    uint8_t tmp = 0;

    level += (NUM_BRIGHTNESS_LEVELS / 2);
    if (level < 0 || level >= NUM_BRIGHTNESS_LEVELS)
    {
        return -1;
    }
    ret |= sccb_readb(0x81, &tmp);
    tmp |= 0x20;
    ret |= sccb_writeb(0x81, tmp);
    ret |= sccb_readb(0xDA, &tmp);
    tmp |= 0x04;
    ret |= sccb_writeb(0xDA, tmp);
    ret |= sccb_writeb(0xE4, brightness_regs[level][0]);
    ret |= sccb_writeb(0xE3, brightness_regs[level][1]);
    return ret;
}

int set_saturation(int level)
{
    int ret = 0;
    uint8_t tmp = 0;

    level += (NUM_SATURATION_LEVELS / 2);
    if (level < 0 || level >= NUM_SATURATION_LEVELS)
    {
        return -1;
    }
    ret |= sccb_readb(0x81, &tmp);
    tmp |= 0x20;
    ret |= sccb_writeb(0x81, tmp);
    ret |= sccb_readb(0xDA, &tmp);
    tmp |= 0x02;
    ret |= sccb_writeb(0xDA, tmp);
    ret |= sccb_writeb(0xDD, saturation_regs[level][0]);
    ret |= sccb_writeb(0xDE, saturation_regs[level][1]);
    return ret;
}

int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;
    uint8_t tmp = 0;
    uint8_t ceiling = (uint8_t)gainceiling;
    if (ceiling > GAINCEILING_32X)
        ceiling = GAINCEILING_32X;
    tmp = (ceiling & 0x07) << 4;
    ret |= sccb_writeb(0x14, tmp);
    return ret;
}

int set_colorbar(int enable)
{
    int ret = 0;
    if (enable)
    {
        ret |= sccb_writeb(0x38, 0x07);
        ret |= sccb_writeb(0x84, 0x02);
    }
    else
    {
        ret |= sccb_writeb(0x38, 0x07);
        ret |= sccb_writeb(0x84, 0x00);
    }
    return ret;
}

int set_auto_gain(int enable, float gain_db, float gain_db_ceiling)
{
    int ret = 0;
    uint8_t tmp = 0;
    uint16_t gain = (uint16_t)gain_db;
    uint8_t ceiling = (uint8_t)gain_db_ceiling;

    ret |= sccb_readb(0x13, &tmp);
    if (enable != 0)
    {
        ret |= sccb_writeb(0x13, tmp | 0x04);
    }
    else
    {
        ret |= sccb_writeb(0x13, tmp & 0xFB);
        if (gain != 0xFFFF && (uint16_t)gain_db_ceiling != 0xFFFF)
        {
            ret |= sccb_readb(0x15, &tmp);
            tmp = (tmp & 0xFC) | (gain >> 8 & 0x03);
            ret |= sccb_writeb(0x15, tmp);
            tmp = gain & 0xFF;
            ret |= sccb_writeb(0x00, tmp);
            tmp = (ceiling & 0x07) << 4;
            ret |= sccb_writeb(0x14, tmp);
        }
    }
    return ret;
}

int get_gain_db(float *gain_db)
{
    int ret = 0;
    uint8_t tmp = 0;
    uint16_t gain;

    ret |= sccb_readb(0x00, &tmp);
    gain = tmp;
    ret |= sccb_readb(0x15, &tmp);
    gain |= ((uint16_t)(tmp & 0x03)) << 8;
    *gain_db = (float)gain;
    return ret;
}

int set_auto_exposure(int enable, int exposure_us)
{
    int ret = 0;
    uint8_t tmp = 0;

    ret |= sccb_readb(0x13, &tmp);
    if (enable != 0)
    {
        ret |= sccb_writeb(0x13, tmp | 0x01);
    }
    else
    {
        ret |= sccb_writeb(0x13, tmp & 0xFE);
        ret |= sccb_writeb(0x0F, (uint8_t)(exposure_us >> 8));
        ret |= sccb_writeb(0x10, (uint8_t)exposure_us);
    }
    return ret;
}

int get_exposure_us(int *exposure_us)
{
    int ret = 0;
    uint8_t tmp = 0;

    ret |= sccb_readb(0x0F, &tmp);
    *exposure_us = tmp << 8 & 0xFF00;
    ret |= sccb_readb(0x10, &tmp);
    *exposure_us = tmp | *exposure_us;
    return ret;
}

int set_auto_whitebal(int enable, float r_gain_db, float g_gain_db,
                      float b_gain_db)
{
    int ret = 0;
    uint8_t tmp = 0;

    ret |= sccb_readb(0x80, &tmp);
    if (enable != 0)
    {
        ret |= sccb_writeb(0x80, tmp | 0x14);
    }
    else
    {
        if ((uint16_t)r_gain_db != 0xFFFF && (uint16_t)g_gain_db != 0xFFFF &&
            (uint16_t)b_gain_db != 0xFFFF)
        {
            ret |= sccb_writeb(0x80, tmp & 0xEF);
            ret |= sccb_writeb(0x01, (uint8_t)b_gain_db);
            ret |= sccb_writeb(0x02, (uint8_t)r_gain_db);
            ret |= sccb_writeb(0x03, (uint8_t)g_gain_db);
        }
        else
        {
            ret |= sccb_writeb(0x80, tmp & 0xEB);
        }
    }
    return ret;
}

int get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    int ret = 0;
    uint8_t tmp = 0;

    ret |= sccb_readb(0x02, &tmp);
    *r_gain_db = (float)tmp;
    ret |= sccb_readb(0x03, &tmp);
    *g_gain_db = (float)tmp;
    ret |= sccb_readb(0x01, &tmp);
    *b_gain_db = (float)tmp;

    return ret;
}

int set_hmirror(int enable)
{
    uint8_t reg;
    int ret = sccb_readb(0x0C, &reg);
    ret |= sccb_writeb(0x0C, OV7740_SET_MIRROR(reg, enable));

    // Sensor Horizontal Output Start Point
    ret = sccb_readb(0x16, &reg);
    ret |= sccb_writeb(0x16, OV7740_SET_SP(reg, enable));

    return ret;
}

int set_vflip(int enable)
{
    uint8_t reg;
    int ret = sccb_readb(0x0C, &reg);
    ret |= sccb_writeb(0x0C, OV7740_SET_FLIP(reg, enable));

    return ret;
}

int set_special_effect(sde_t sde)
{
    int ret;
    uint8_t reg;
    switch (sde)
    {
    case SDE_NORMAL:
        ret = sccb_readb(0x81, &reg);
        ret |= sccb_writeb(0x81, reg & 0xFE);
        ret = sccb_readb(0xDA, &reg);
        ret |= sccb_writeb(0xDA, reg & 0xBF);
        break;
    case SDE_NEGATIVE:
        ret = sccb_readb(0x81, &reg);
        ret |= sccb_writeb(0x81, reg | 0x01);
        ret = sccb_readb(0xDA, &reg);
        ret |= sccb_writeb(0xDA, reg | 0x40);
        break;

    default:
        return -1;
    }
    return ret;
}

int set_lens_correction(int enable, int radi, int coef) { return -1; }

/***************************** End of OV7740 Settings ********************************/

// void seekfree_sendimg(uint8_t *image, uint16_t width, uint16_t height, uint8_t RGB1_Gray0)
// {
//   uarths_putchar(0x00);
//   uarths_putchar(0xff);
//   uarths_putchar(0x01);
//   uarths_putchar(0x01);  //发送命令
//   if(RGB1_Gray0) //RGB
//     for (uint32_t i = 0; i < 2 * width * height; i++) {
//       uarths_putchar(*(image + 2 * i + 1));
//       uarths_putchar(*(image + 2 * i));
//     }
//   else //Gray
//     uarths_send_data(image, width * height);
// }

void image_compress_crop(uint8_t *img_in, uint8_t *img_out, uint16_t width_in, uint16_t height_in, uint8_t ratio)
{
    for (int j = 0; j < height_in / ratio; j++)
    {
        for (int i = 0; i < width_in / ratio; i++)
        {
            *(img_out + width_in / ratio * j + i) = *(img_in + width_in * j * ratio + i * ratio);
        }
    }
}

void image_compress_blackwhite(uint8_t *img_in, uint8_t *img_out, uint16_t width, uint16_t height, uint8_t threshold)
{
    int pixel = width * height;
    for (int i = 0; i < pixel; i++)
    {
        *(img_out + i) = *(img_in + i) > threshold ? 0xFF : 0x00;
    }
}

void image_compress_onebit(uint8_t *img_in, uint8_t *img_out, uint16_t width, uint16_t height, uint8_t threshold)
{
    int pixel = width * height;
    for (int i = 0; i < pixel; i++)
    {
        if (*(img_in + i) > threshold)
            *(img_out + i / 8) |= 0x01 << (i % 8);
        else
            *(img_out + i / 8) &= ~(0x01 << (i % 8));
    }

    // unsigned char pic[60 * 80];

    // //解压缩图像
    // for (int i = 0; i < 60; i++)
    // {
    //     for (int j = 0; j < 10; j++)
    //     {
    //         int tmp = img_out[i * 10 + j];
    //         for (int w = 0; w < 8; w++)
    //         {
    //             if ((tmp & (1 << w)))
    //             {
    //                 pic[80 * i + j * 8 + w] = 0xff;
    //             }
    //             else
    //             {
    //                 pic[80 * i + j * 8 + w] = 0;
    //             }
    //         }
    //     }
    // }
    // int pic_head[7];
    // // int len = sprintf(str, "image:%d,%d,%d,%d,%d" , 0, 60 * 80, 80, 60, 24);
    // pic_head[0] = 1;
    // pic_head[1] = 6*8;
    // pic_head[2] = 8;
    // pic_head[3] = 6;
    // pic_head[4] = 23;
    // pic_head[5] = 0x7F800000;
    // pic_head[6] = 0x7F800000;
    // uart_send_data(UART_DEVICE_3, (char *)&pic_head[0], 7 * 4);
    // uart_send_data(UART_DEVICE_3, pic, 96);
    // msleep(100);
}

void vofa_sendimg(uint8_t *image, uint16_t width, uint16_t height, uint8_t RGB1_Gray0)
{
    char str[40];
    int len = sprintf(str, "image:%d,%d,%d,%d,%d\n", RGB1_Gray0, width * height, width, height, 24);
    // if(RGB1_Gray0) //RGB
    //   for (uint32_t i = 0; i < 2 * width * height; i++) {
    //     uarths_putchar(*(image + 2 * i + 1));
    //     uarths_putchar(*(image + 2 * i));
    //   }
    // else //Gray
    uart_send_data(UART_DEVICE_3, str, len);
    uart_send_data(UART_DEVICE_3, image, width * height);
    uart_send_data(UART_DEVICE_3, "\n", 1);
}

void OV7740_Stop(void)
{
    ov7740_sleep(1);
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE, 0);
    plic_irq_disable(IRQN_DVP_INTERRUPT);
}

void OV7740_Start(void)
{
    ov7740_sleep(0);
    dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE, 1);
    plic_irq_enable(IRQN_DVP_INTERRUPT);
}

static void io_set_power(void)
{
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
}

static void OV7740_dvp_Init(void)
{
    // dvp 接口初始化
    dvp_init(8);
    uint32_t rate = dvp_set_xclk_rate(25000000); // 给摄像头提供时钟
    // printf("xclk rate:%d\n", rate);
    sysctl_set_spi0_dvp_data(1); // dvp数据口使能

    dvp_enable_burst(); // 突发传输模式 意义不明
    // dvp_disable_burst();
    dvp_disable_auto();
    dvp_set_output_enable(DVP_OUTPUT_AI, 1);      // AI输出
    dvp_set_output_enable(DVP_OUTPUT_DISPLAY, 0); // 显示输出
    dvp_set_image_format(DVP_CFG_Y_FORMAT);       // 接收摄像头YUV输出,并且只存储Y分量
    dvp_set_image_size(320, 240);                 // VGA
    // dvp_set_display_addr((uint32_t)((long)graph_buf1));  //显示输出地址
    // dvp_set_ai_addr((uint32_t)((long)graph_buf0), 0, 0);  // AI输出地址

    // dvp 中断设置
    plic_set_priority(IRQN_DVP_INTERRUPT, 1);
    plic_irq_register(IRQN_DVP_INTERRUPT, on_irq_dvp, NULL);
    plic_irq_enable(IRQN_DVP_INTERRUPT);
    sysctl_enable_irq();
    dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
}

int OV7740_Init(void)
{
    // 引脚电平设置
    io_set_power();

    // dvp 接口初始化
    OV7740_dvp_Init();

    // i2c初始化 波特率不能高于400k
    i2c_init(SCCB_I2C_DEVICE_NUM, OV7740_Device_ADDR, 7, 100000);

    // 检查OV7740是否存在
    msleep(10);
    int timeout;
    for (timeout = 0; timeout < 100 && OV7740_Check(); timeout++)
    {
        msleep(10);
    }
    if (timeout < 99)
        printf("Find OV7740!\n");
    else
    {
        printf("OV7740 not found\n");
        return 0;
    }

    // OV7740寄存器配置
    ov7740_reset();
    // sccb_writeb(0x12, 0x40);
    // sccb_writeb(0x89, 0x35);

    // 摄像头效果配置
    set_framesize(FRAMESIZE_QQVGA);
    set_framerate(FRAMERATE_60FPS);
    // uint8_t data;
    // sccb_readb(0x11, &data);
    // printf("0x11:%02X\n", data);
    // sccb_readb(0x55, &data);
    // printf("0x55:%02X\n", data);
    // sccb_readb(0x2b, &data);
    // printf("0x2b:%02X\n", data);
    // sccb_readb(0x2c, &data);
    // printf("0x2c:%02X\n", data);
    // set_colorbar(1);

    // 开启中断
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);

    return 0;
}

#include <param.h>
#include <data_exchange.h>

volatile uint8_t g_dvp_finish_flag = 0;
volatile uint8_t g_ram_mux = 1;
uint32_t frame_cnt = 0;

void cam_task(void)
{
    if(g_dvp_finish_flag == 1)
    {
        // static uint32_t start_time = 0;
        // uint32_t now_time = 0;
        // start_time = sysctl_get_time_us();
        image_compress_crop(graph_buf0, graph_buf1, 160, 120, 2);
        // image_compress_blackwhite(graph_buf1, graph_buf1, 80, 60, 128);
        // image_compress_onebit(graph_buf1, graph_buf1, 80, 60, 150);
        // vofa_sendimg(graph_buf0, 320, 240, 0);
        // vofa_sendimg(graph_buf1, 80, 60, 1);
        // pic_pack_send_kgs(graph_buf0, 160, 120, 0);
        if(global_param.image_transmit_enable) {
            pic_pack_send_kgs(graph_buf1, 80, 60, 0);
        }
        // now_time = sysctl_get_time_us();
        // global_data.debug_data = (now_time-start_time)/1000.0;
        // printf("interv_time:%.3f\n", global_data.debug_data);
        g_dvp_finish_flag = 0;
    }
}
/********************** dvp中断服务函数 ***********************/
int on_irq_dvp(void *ctx)
{
    if (dvp_get_interrupt(DVP_STS_FRAME_FINISH))
    { // 帧结束中断
        /* switch gram */
        
        graph_buf_addr = g_ram_mux ? (uint32_t)(long)graph_buf0 : (uint32_t)(long)graph_buf1;
        dvp_set_ai_addr(graph_buf_addr, 0, 0);
        g_dvp_finish_flag = 1;
        dvp_clear_interrupt(DVP_STS_FRAME_FINISH | DVP_STS_FRAME_START);
    }
    else
    { // 帧开始中断
        if (g_dvp_finish_flag == 0)
        {
            frame_cnt++;
            
            dvp_start_convert();
        }
        dvp_clear_interrupt(DVP_STS_FRAME_START);
    }

    return 0;
}
