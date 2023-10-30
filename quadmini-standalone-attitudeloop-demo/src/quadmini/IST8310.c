/*
3轴磁力计
*/

#include "io_func.h"
#include <IST8310.h>
#include <gpiohs.h>
#include <i2c.h>
#include <sleep.h>

#define IST_NRST(pv) gpiohs_set_pin(IST_RSTn_GPIOHS, pv)
//#define IST_DRDY gpiohs_get_pin(IST_DRDY_GPIOHS)

#define MAG_SEN 0.3f  //转换成 uT

#define IST8310_WHO_AM_I 0x00        // ist8310 who am I 寄存器
#define IST8310_WHO_AM_I_VALUE 0x10  //设备 ID

#define IST8310_WRITE_REG_NUM 4  // IST8310需要设置的寄存器数目

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x00, 0x01},
    {0x41, 0x09, 0x02},
    {0x42, 0xC0, 0x03},
    {0x0A, 0x0B, 0x04}};


// ist8310 middleware
static uint8_t ist8310_IIC_read_single_reg(uint8_t reg) {
	uint8_t res;
	i2c_recv_data(I2C_DEVICE_0, IST_ADDR, &reg, 1, &res, 1);
  	return res;
}
static void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
  	uint8_t buf[2] = {reg, data};
  	i2c_send_data(I2C_DEVICE_0, IST_ADDR, buf, 2);
}
static void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
	i2c_recv_data(I2C_DEVICE_0, IST_ADDR, &reg, 1, buf, len);
}
// end of middleware

int ist8310_init(void) {
	uint8_t res = 0;
	uint8_t writeNum = 0;

	i2c_init(I2C_DEVICE_0, IST_ADDR, 7, IST_I2C_RATE);
	gpiohs_set_drive_mode(IST_RSTn_GPIOHS, GPIO_DM_OUTPUT);
	// gpiohs_set_drive_mode(IST_DRDY_PIN, GPIO_DM_INPUT_PULL_UP);

	IST_NRST(0);
	msleep(50);
	IST_NRST(1);
	msleep(50);

  	// check WHOAMI register
  	res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
  	if (res != IST8310_WHO_AM_I_VALUE) {
  		return IST8310_NO_SENSOR;
  	}
  	msleep(1);
  	// IST8310 Config
  	for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++) {
  		ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0],
  		                             ist8310_write_reg_data_error[writeNum][1]);
  		msleep(1);
  		res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
  		msleep(1);
  		if (res != ist8310_write_reg_data_error[writeNum][1]) {
  			return ist8310_write_reg_data_error[writeNum][2];
  		}
  	}

	return IST8310_NO_ERROR;
}

void IST_read(mag_data_t * mag_data) {
  	uint8_t data[6] = {0};

	ist8310_IIC_read_muli_reg(0x03, data, 6);

    mag_data->mag_raw_X = (BODY_X_IS_IST>0?1:-1) *
#if (BODY_X_IS_IST == 'X'||BODY_X_IS_IST == -'X')
        ((int16_t)((data[1]) << 8) | data[0]);
#elif (BODY_X_IS_IST == 'Y'||BODY_X_IS_IST == -'Y')
        ((int16_t)(data[3] << 8) | data[2]);
#elif (BODY_X_IS_IST == 'Z'||BODY_X_IS_IST == -'Z')
        ((int16_t)(data[5] << 8) | data[4]);
#endif
    mag_data->mag_raw_Y = (BODY_Y_IS_IST>0?1:-1) *
#if (BODY_Y_IS_IST == 'X'||BODY_Y_IS_IST == -'X')
        ((int16_t)(data[1] << 8) | data[0]);
#elif (BODY_Y_IS_IST == 'Y'||BODY_Y_IS_IST == -'Y')
        ((int16_t)(data[3] << 8) | data[2]);
#elif (BODY_Y_IS_IST == 'Z'||BODY_Y_IS_IST == -'Z')
        ((int16_t)(data[5] << 8) | data[4]);
#endif
    mag_data->mag_raw_Z = (BODY_Z_IS_IST>0?1:-1) *
#if (BODY_Z_IS_IST == 'X'||BODY_Z_IS_IST == -'X')
        ((int16_t)(data[1] << 8) | data[0]);
#elif (BODY_Z_IS_IST == 'Y'||BODY_Z_IS_IST == -'Y')
        ((int16_t)(data[3] << 8) | data[2]);
#elif (BODY_Z_IS_IST == 'Z'||BODY_Z_IS_IST == -'Z')
        ((int16_t)(data[5] << 8) | data[4]);
#endif

	mag_data->mag_X = MAG_SEN * mag_data->mag_raw_X;
	mag_data->mag_Y = MAG_SEN * mag_data->mag_raw_Y;
	mag_data->mag_Z = MAG_SEN * mag_data->mag_raw_Z;
	// printf("mag:%f,%f,%f\n", mag_data->mag_X, mag_data->mag_Y, mag_data->mag_Z);
}
