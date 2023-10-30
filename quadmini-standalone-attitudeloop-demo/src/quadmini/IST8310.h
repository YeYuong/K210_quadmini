#ifndef IST8310_H
#define IST8310_H

#include <stdio.h>

#define IST_I2C_RATE 400000

#define IST_ADDR 0x0E

//指定坐标轴替换
#define BODY_X_IS_IST (-'X')
#define BODY_Y_IS_IST (-'Y')
#define BODY_Z_IS_IST (-'Z')

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct {
  int16_t mag_raw_X;
  int16_t mag_raw_Y;
  int16_t mag_raw_Z;
  float mag_X;
  float mag_Y;
  float mag_Z;
} mag_data_t;

int ist8310_init(void);
void IST_read(mag_data_t * mag_data);


#endif
