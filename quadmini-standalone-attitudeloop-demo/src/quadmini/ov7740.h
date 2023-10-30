#ifndef _OV7740_H
#define _OV7740_H


#include <ov7740_regs.h>


#define SCCB_I2C_DEVICE_NUM I2C_DEVICE_2
#define OV7740_Device_ADDR (0x21)
#define OV7740_ID (0x7742)

#define OV7740_SET_MIRROR(r, x) ((r & 0xBF) | ((x & 1) << 6))
#define OV7740_SET_FLIP(r, x) ((r & 0x7F) | ((x & 1) << 7))
#define OV7740_SET_SP(r, x) ((r & 0xEE) | ((x & 1) << 4) | (x & 1))

#define DCMI_RESET_LOW() dvp->cmos_cfg &= ~DVP_CMOS_RESET
#define DCMI_RESET_HIGH() dvp->cmos_cfg |= DVP_CMOS_RESET
#define DCMI_PWDN_LOW() dvp->cmos_cfg |= DVP_CMOS_POWER_DOWN
#define DCMI_PWDN_HIGH() dvp->cmos_cfg &= ~DVP_CMOS_POWER_DOWN

typedef enum {
  PIXFORMAT_INVLAID = 0,
  PIXFORMAT_BAYER,      // 1BPP/RAW
  PIXFORMAT_RGB565,     // 2BPP/RGB565
  PIXFORMAT_YUV422,     // 2BPP/YUV422
  PIXFORMAT_GRAYSCALE,  // 1BPP/GRAYSCALE
  PIXFORMAT_JPEG,       // JPEG/COMPRESSED
} pixformat_t;

typedef enum {
  FRAMESIZE_INVALID = 0,
  // C/SIF Resolutions
  FRAMESIZE_QQCIF,  // 88x72
  FRAMESIZE_QCIF,   // 176x144
  FRAMESIZE_CIF,    // 352x288
  FRAMESIZE_QQSIF,  // 88x60
  FRAMESIZE_QSIF,   // 176x120
  FRAMESIZE_SIF,    // 352x240
  // VGA Resolutions
  FRAMESIZE_QQQQVGA,  // 40x30
  FRAMESIZE_QQQVGA,   // 80x60
  FRAMESIZE_QQVGA,    // 160x120
  FRAMESIZE_QVGA,     // 320x240
  FRAMESIZE_VGA,      // 640x480
  FRAMESIZE_HQQQVGA,  // 60x40
  FRAMESIZE_HQQVGA,   // 120x80
  FRAMESIZE_HQVGA,    // 240x160
  // FFT Resolutions
  FRAMESIZE_64X32,    // 64x32
  FRAMESIZE_64X64,    // 64x64
  FRAMESIZE_128X64,   // 128x64
  FRAMESIZE_128X128,  // 128x128
  FRAMESIZE_240X240,  // 240x240
  // Other
  FRAMESIZE_LCD,     // 128x160
  FRAMESIZE_QQVGA2,  // 128x160
  FRAMESIZE_WVGA,    // 720x480
  FRAMESIZE_WVGA2,   // 752x480
  FRAMESIZE_SVGA,    // 800x600
  FRAMESIZE_SXGA,    // 1280x1024
  FRAMESIZE_UXGA,    // 1600x1200
} framesize_t;

typedef enum {
  FRAMERATE_2FPS,
  FRAMERATE_8FPS,
  FRAMERATE_15FPS,
  FRAMERATE_25FPS,
  FRAMERATE_30FPS,
  FRAMERATE_60FPS,
} framerate_t;

typedef enum {
  GAINCEILING_2X,
  GAINCEILING_4X,
  GAINCEILING_8X,
  GAINCEILING_16X,
  GAINCEILING_32X,
  GAINCEILING_64X,
  GAINCEILING_128X,
} gainceiling_t;

typedef enum {
  SDE_NORMAL,
  SDE_NEGATIVE,
} sde_t;

extern uint32_t *graph_buf0;
extern uint32_t *graph_buf1;
extern volatile uint8_t g_dvp_finish_flag;
extern volatile uint8_t g_ram_mux;
extern framesize_t now_fsize;

int OV7740_Init(void);
int OV7740_Check(void);
int ov7740_reset(void);
int ov7740_sleep(int enable);
int set_pixformat(pixformat_t pixformat);
int set_framesize(framesize_t framesize);
int set_framerate(framerate_t framerate);
int set_contrast(int level);
int set_brightness(int level);
int set_saturation(int level);
int set_gainceiling(gainceiling_t gainceiling);
int set_colorbar(int enable);
int set_auto_gain(int enable, float gain_db, float gain_db_ceiling);
int get_gain_db(float *gain_db);
int set_auto_exposure(int enable, int exposure_us);
int get_exposure_us(int *exposure_us);
int set_auto_whitebal(int enable, float r_gain_db, float g_gain_db,
                      float b_gain_db);
int get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db);
int set_hmirror(int enable);
int set_vflip(int enable);
int set_special_effect(sde_t sde);
int set_lens_correction(int enable, int radi, int coef);

// void seekfree_sendimg(uint8_t *image, uint16_t width, uint16_t height, uint8_t RGB1_Gray0);
void image_compress_crop(uint8_t *img_in, uint8_t *img_out , uint16_t width_in, uint16_t height_in, uint8_t ratio);
void image_compress_blackwhite(uint8_t *img_in, uint8_t *img_out , uint16_t width, uint16_t height, uint8_t threshold);
void image_compress_onebit(uint8_t *img_in, uint8_t *img_out , uint16_t width, uint16_t height, uint8_t threshold);
void vofa_sendimg(uint8_t *image, uint16_t width, uint16_t height, uint8_t RGB1_Gray0);

void OV7740_Start(void);
void OV7740_Stop(void);
void cam_task(void);

int on_irq_dvp(void *ctx);

#endif
