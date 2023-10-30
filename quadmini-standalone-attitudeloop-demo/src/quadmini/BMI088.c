/*
6轴陀螺仪
spi接口

问题1：k210的spi机制和stm不同，先传输cmd，下一个字节等待数据输入


*/
#include <BMI088.h>
#include <BMI088REG.h>
#include <gpiohs.h>
#include <sleep.h>
#include <spi.h>
#include <spi_sw.h>
#include <sysctl.h>


float BMI088_ACCEL_SEN = BMI088_ACCEL_24G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;


// BMI088 middleware
#define BMI088_delay_ms(ms) msleep(ms)
#define BMI088_delay_us(us) usleep(us)

void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
#ifndef USE_SOFTWARE_SPI
	spi_set_clk_rate(BMI088_USE_SPI, BMI_SPI_RATE);
	spi_send_data_standard(BMI088_USE_SPI, CS_ACCEL, &reg, 1, &data, 1);
#else
	spi_sw_send_data_standard(BMI088_USE_SPI, CS_ACCEL, &reg, 1, &data, 1);
#endif
}
//注意：ACCEL在读取寄存器时需要跳过返回的第一个字节
void BMI088_accel_read_single_reg(uint8_t reg, uint8_t *data)
{
	reg |= 0x80;
#ifndef USE_SOFTWARE_SPI
	spi_set_clk_rate(BMI088_USE_SPI, BMI_SPI_RATE);
	spi_receive_data_standard(BMI088_USE_SPI, CS_ACCEL, &reg, 2, data, 1);
#else
	spi_sw_receive_data_standard(BMI088_USE_SPI, CS_ACCEL, &reg, 2, data, 1);
#endif
	// uint8_t data_buf[2];
	// spi_receive_data_standard(BMI088_USE_SPI, CS_ACCEL, &reg, 1, data_buf, 2);
	// printf("readreg:%02X %02X\n", data_buf[0], data_buf[1]);
	// *data = data_buf[1];
}

void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
	reg |= 0x80;
#ifndef USE_SOFTWARE_SPI
	spi_set_clk_rate(BMI088_USE_SPI, BMI_SPI_RATE);
	spi_receive_data_standard(BMI088_USE_SPI, CS_ACCEL, &reg, 2, data, len);
#else
	spi_sw_receive_data_standard(BMI088_USE_SPI, CS_ACCEL, &reg, 2, data, len);
#endif
    
}

void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data)
{
#ifndef USE_SOFTWARE_SPI
	spi_set_clk_rate(BMI088_USE_SPI, BMI_SPI_RATE);
	spi_send_data_standard(BMI088_USE_SPI, CS_GYRO, &reg, 1, &data, 1);
#else
	spi_sw_send_data_standard(BMI088_USE_SPI, CS_GYRO, &reg, 1, &data, 1);
#endif
    
}

void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t *data)
{
	reg |= 0x80;
#ifndef USE_SOFTWARE_SPI
	spi_set_clk_rate(BMI088_USE_SPI, BMI_SPI_RATE);
	spi_receive_data_standard(BMI088_USE_SPI, CS_GYRO, &reg, 1, data, 1);
#else
	spi_sw_receive_data_standard(BMI088_USE_SPI, CS_GYRO, &reg, 1, data, 1);
#endif
    
}

void BMI088_gyro_read_muli_reg(uint8_t reg,uint8_t *data, uint8_t len)
{
	reg |= 0x80;
#ifndef USE_SOFTWARE_SPI
	spi_set_clk_rate(BMI088_USE_SPI, BMI_SPI_RATE);
	spi_receive_data_standard(BMI088_USE_SPI, CS_GYRO, &reg, 1, data, len);
#else
	spi_sw_receive_data_standard(BMI088_USE_SPI, CS_GYRO, &reg, 1, data, len);
#endif
    
}

//end of middleware

static uint8_t
    write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,
         BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
         BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,
         BMI088_ACC_OSR4 | BMI088_ACC_400_HZ | BMI088_ACC_CONF_MUST_Set,
         BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL,
         BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP |
             BMI088_ACC_INT1_GPIO_LOW,
         BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT,
         BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {{BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
     {BMI088_GYRO_BANDWIDTH,
      BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
      BMI088_GYRO_BANDWIDTH_ERROR},
     {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
     {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
     {BMI088_GYRO_INT3_INT4_IO_CONF,
      BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
      BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
     {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
      BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

//使用spi硬件片选引脚
void bmi_spi_init(void) {
#ifndef USE_SOFTWARE_SPI
	spi_init(BMI088_USE_SPI, SPI_WORK_MODE_0, SPI_FF_STANDARD, 8, 0);  // SPI0
	spi_set_clk_rate(BMI088_USE_SPI, 200000);//BMI_SPI_RATE);
#else
    spi_sw_init();
#endif
    msleep(1);
}

uint8_t BMI088_init(void) {
	uint8_t error = BMI088_NO_ERROR;
	// GPIO and SPI  Init
	bmi_spi_init();

	// self test pass and init
	int ret = 0;
	if ((ret = bmi088_accel_self_test()) != BMI088_NO_ERROR) {
		printf("bmi088_accel_self_test return error(%d)", ret);
		error |= BMI088_SELF_TEST_ACCEL_ERROR;
	} else {
		error |= bmi088_accel_init();
	}

	if ((ret = bmi088_gyro_self_test()) != BMI088_NO_ERROR) {
		printf("bmi088_gyro_self_test return error(%d)", ret);
		error |= BMI088_SELF_TEST_GYRO_ERROR;
	} else {
		error |= bmi088_gyro_init();
	}

	return error;
}

int bmi088_accel_init(void) {
	uint8_t res = 0;
	uint8_t write_reg_num = 0;

	// check commiunication
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	// accel software reset
	BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET,
	                              BMI088_ACC_SOFTRESET_VALUE);
	BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

	// check commiunication is normal after reset
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	// check the "who am I"
	if (res != BMI088_ACC_CHIP_ID_VALUE) {
	  	return BMI088_NO_SENSOR;
	}

	// set accel sonsor config and check
	for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
	  	BMI088_accel_write_single_reg(
	    	write_BMI088_accel_reg_data_error[write_reg_num][0],
	    	write_BMI088_accel_reg_data_error[write_reg_num][1]);
	  		BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  	BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], &res);
	  	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  	if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
		{
	    	return write_BMI088_accel_reg_data_error[write_reg_num][2];
	  	}
	}
	return BMI088_NO_ERROR;
}

int bmi088_gyro_init(void) {
	uint8_t write_reg_num = 0;
	uint8_t res = 0;

	// check commiunication
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	// reset the gyro sensor
	BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET,
	                             BMI088_GYRO_SOFTRESET_VALUE);
	BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
	// check commiunication is normal after reset
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	// check the "who am I"
	if (res != BMI088_GYRO_CHIP_ID_VALUE) {
		return BMI088_NO_SENSOR;
	}

	// set gyro sonsor config and check
	for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
	{
	  	BMI088_gyro_write_single_reg(
	      	write_BMI088_gyro_reg_data_error[write_reg_num][0],
	      	write_BMI088_gyro_reg_data_error[write_reg_num][1]);
	  		BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  	BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], &res);
	  	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  	if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
		{
	    	return write_BMI088_gyro_reg_data_error[write_reg_num][2];
	  	}
	}

	return BMI088_NO_ERROR;
}

int bmi088_accel_self_test(void) {
	int16_t self_test_accel[2][3];

	uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
	uint8_t res = 0;

	uint8_t write_reg_num = 0;

	static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] = {
	    {BMI088_ACC_CONF,
	     BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set,
	     BMI088_ACC_CONF_ERROR},
	    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,
	     BMI088_ACC_PWR_CTRL_ERROR},
	    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
	    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
	     BMI088_ACC_PWR_CONF_ERROR},
	    {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL,
	     BMI088_ACC_PWR_CONF_ERROR},
	    {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL,
	     BMI088_ACC_PWR_CONF_ERROR}

	};

	// check commiunication is normal

    // while (1) // debug BMI@asdf
    // {
    //     /* code */
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    // printf("acc:%02X\n", res);
    // }

	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	// reset  bmi088 accel sensor and wait for > 50ms
	BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET,
	                              BMI088_ACC_SOFTRESET_VALUE);
	BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

	// check commiunication is normal
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	if (res != BMI088_ACC_CHIP_ID_VALUE) {
	  return BMI088_NO_SENSOR;
	}

	// set the accel register
	for (write_reg_num = 0; write_reg_num < 4; write_reg_num++) {
	  BMI088_accel_write_single_reg(
	      write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0],
	      write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
	  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], &res);
	  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]) {
	    return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
	  }
	  // accel conf and accel range  . the two register set need wait for > 50ms
	  BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
	}

	// self test include postive and negative
	for (write_reg_num = 0; write_reg_num < 2; write_reg_num++) {
	  BMI088_accel_write_single_reg(
	      write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0],
	      write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
	  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], &res);
	  BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	  if (res !=
	      write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]) {
	    return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
	  }
	  // accel conf and accel range  . the two register set need wait for > 50ms
	  BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

	  // read response accel
	  BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

	  self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
	  self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
	  self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
	}

	// set self test off
	BMI088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_accel_read_single_reg(BMI088_ACC_SELF_TEST, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	if (res != (BMI088_ACC_SELF_TEST_OFF)) {
	  return BMI088_ACC_SELF_TEST_ERROR;
	}

	// reset the accel sensor
	BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET,
	                              BMI088_ACC_SOFTRESET_VALUE);
	BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

	if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) ||
	    (self_test_accel[0][1] - self_test_accel[1][1] < 1365) ||
	    (self_test_accel[0][2] - self_test_accel[1][2] < 680)) {
	  return BMI088_SELF_TEST_ACCEL_ERROR;
	}

	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	return BMI088_NO_ERROR;
}
int bmi088_gyro_self_test(void) {
	uint8_t res = 0;
	uint8_t retry = 0;
	// check commiunication is normal

    // while (1) // debug BMI@asdf
    // {
    //     /* code */
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    // printf("gyro:%02X\n", res);
    // if(res == 0x0F)
    //     break;
    // }

	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	// reset the gyro sensor
	BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET,
	                             BMI088_GYRO_SOFTRESET_VALUE);
	BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
	// check commiunication is normal after reset
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
	BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
	BMI088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
	BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

	do {
		BMI088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, &res);
		BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		retry++;
	} while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

	if (retry == 10) {
	  	return BMI088_SELF_TEST_GYRO_ERROR;
	}

	if (res & BMI088_GYRO_BIST_FAIL) {
	  	return BMI088_SELF_TEST_GYRO_ERROR;
	}

	return BMI088_NO_ERROR;
}

void BMI088_read_gyro_who_am_i(void) {
	uint8_t buf;
	BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &buf);
}

void BMI088_read_accel_who_am_i(void) {
	uint8_t buf;
	BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &buf);
	buf = 0;
}

void BMI088_temperature_read_over(uint8_t *rx_buf, float *temperate) {
	int16_t bmi088_raw_temp;
	bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

	if (bmi088_raw_temp > 1023) {
		bmi088_raw_temp -= 2048;
	}
	*temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088_accel_read_over(uint8_t *rx_buf, float accel[3], float *time) {
	int16_t bmi088_raw_temp;
	uint32_t sensor_time;
	bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
	accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
	bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
	accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
	bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
	accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
	sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
	*time = sensor_time * 39.0625f;
}

void BMI088_gyro_read_over(uint8_t *rx_buf, float gyro[3]) {
	int16_t bmi088_raw_temp;
	bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
	gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
	bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
	gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
	bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
	gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}

void BMI088_read(float gyro[3], float accel[3], float *temperate) {
	uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
	int16_t bmi088_raw_temp;

	BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

	accel[0] = (BODY_X_IS_BMI>0?1:-1) * BMI088_ACCEL_SEN *
#if (BODY_X_IS_BMI == 'X'||BODY_X_IS_BMI == -'X')
        ((int16_t)((buf[1]) << 8) | buf[0]);
#elif (BODY_X_IS_BMI == 'Y'||BODY_X_IS_BMI == -'Y')
        ((int16_t)((buf[3]) << 8) | buf[2]);
#elif (BODY_X_IS_BMI == 'Z'||BODY_X_IS_BMI == -'Z')
        ((int16_t)((buf[5]) << 8) | buf[4]);
#endif
	accel[1] = (BODY_Y_IS_BMI>0?1:-1) * BMI088_ACCEL_SEN *
#if (BODY_Y_IS_BMI == 'X'||BODY_Y_IS_BMI == -'X')
        ((int16_t)((buf[1]) << 8) | buf[0]);
#elif (BODY_Y_IS_BMI == 'Y'||BODY_Y_IS_BMI == -'Y')
        ((int16_t)((buf[3]) << 8) | buf[2]);
#elif (BODY_Y_IS_BMI == 'Z'||BODY_Y_IS_BMI == -'Z')
        ((int16_t)((buf[5]) << 8) | buf[4]);
#endif
	accel[2] = (BODY_Z_IS_BMI>0?1:-1) * BMI088_ACCEL_SEN *
#if (BODY_Z_IS_BMI == 'X'||BODY_Z_IS_BMI == -'X')
        ((int16_t)((buf[1]) << 8) | buf[0]);
#elif (BODY_Z_IS_BMI == 'Y'||BODY_Z_IS_BMI == -'Y')
        ((int16_t)((buf[3]) << 8) | buf[2]);
#elif (BODY_Z_IS_BMI == 'Z'||BODY_Z_IS_BMI == -'Z')
        ((int16_t)((buf[5]) << 8) | buf[4]);
#endif

	BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
	if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
		gyro[0] = (BODY_X_IS_BMI>0?1:-1) * BMI088_GYRO_SEN *
#if (BODY_X_IS_BMI == 'X'||BODY_X_IS_BMI == -'X')
        ((int16_t)((buf[3]) << 8) | buf[2]);
#elif (BODY_X_IS_BMI == 'Y'||BODY_X_IS_BMI == -'Y')
        ((int16_t)((buf[5]) << 8) | buf[4]);
#elif (BODY_X_IS_BMI == 'Z'||BODY_X_IS_BMI == -'Z')
        ((int16_t)((buf[7]) << 8) | buf[6]);
#endif
		gyro[1] = (BODY_Y_IS_BMI>0?1:-1) * BMI088_GYRO_SEN *
#if (BODY_Y_IS_BMI == 'X'||BODY_Y_IS_BMI == -'X')
        ((int16_t)((buf[3]) << 8) | buf[2]);
#elif (BODY_Y_IS_BMI == 'Y'||BODY_Y_IS_BMI == -'Y')
        ((int16_t)((buf[5]) << 8) | buf[4]);
#elif (BODY_Y_IS_BMI == 'Z'||BODY_Y_IS_BMI == -'Z')
        ((int16_t)((buf[7]) << 8) | buf[6]);
#endif
		gyro[2] = (BODY_Z_IS_BMI>0?1:-1) * BMI088_GYRO_SEN *
#if (BODY_Z_IS_BMI == 'X'||BODY_Z_IS_BMI == -'X')
        ((int16_t)((buf[3]) << 8) | buf[2]);
#elif (BODY_Z_IS_BMI == 'Y'||BODY_Z_IS_BMI == -'Y')
        ((int16_t)((buf[5]) << 8) | buf[4]);
#elif (BODY_Z_IS_BMI == 'Z'||BODY_Z_IS_BMI == -'Z')
        ((int16_t)((buf[7]) << 8) | buf[6]);
#endif
	}
	BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

	bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

	if (bmi088_raw_temp > 1023) {
		bmi088_raw_temp -= 2048;
	}

	*temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

uint32_t get_BMI088_sensor_time(void) {
	uint32_t sensor_time = 0;
	uint8_t buf[3];
	BMI088_accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);

	sensor_time = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

	return sensor_time;
}

float get_BMI088_temperate(void) {
	uint8_t buf[2];
	float temperate;
	int16_t temperate_raw_temp;

	BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

	temperate_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

	if (temperate_raw_temp > 1023) {
		temperate_raw_temp -= 2048;
	}

	temperate = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

	return temperate;
}

void get_BMI088_gyro(int16_t gyro[3]) {
	uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyro_raw_temp;

	BMI088_gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);

	gyro_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
	gyro[0] = gyro_raw_temp;
	gyro_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
	gyro[1] = gyro_raw_temp;
	gyro_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
	gyro[2] = gyro_raw_temp;
}

void get_BMI088_accel(float accel[3]) {
  uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
  int16_t accel_raw_temp;

  BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

  accel_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
  accel[0] = accel_raw_temp * BMI088_ACCEL_SEN;
  accel_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
  accel[1] = accel_raw_temp * BMI088_ACCEL_SEN;
  accel_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
  accel[2] = accel_raw_temp * BMI088_ACCEL_SEN;
}
