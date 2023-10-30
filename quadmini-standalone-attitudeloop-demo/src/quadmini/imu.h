#ifndef _IMU_H
#define _IMU_H

#include <BMI088.h>
#include <IST8310.h>

#define ANG_TO_RAD					0.01745329
#define RAD_TO_ANG					57.2957795

#define ACC_RANGE_LOW 9.2
#define ACC_RANGE_HIGH 10.2

#define ACC_COMPEN 1
#define MAG_COMPEN 0

#define REF_ERR_LPF_HZ 1
#define ACCEL_COMPENSATION_KP 2.0
#define ACCEL_COMPENSATION_KI 0.2
#define MAG_COMPENSATION_KP 0.2
#define MC_COMPENSATION_KP 5.0
#define ERR_INTEGRAL_MAX 4.0 * ANG_TO_RAD

#define IMU_Hz 400

#define IMU_USE_Timer TIMER_DEVICE_1
#define IMU_TIM_Channel TIMER_CHANNEL_0

struct quaternions_ty
{
	float q0;
	float q1;
	float q2;
	float q3;
};

struct heading_angle_ty
{
	float yaw;
	float pitch;
	float roll;
};

struct vector_f_3
{
	float x;
	float y;
	float z;
};

struct vector_f_2
{
	float x;
	float y;
};

struct imu_ty
{
	struct vector_f_3 acc;
	struct vector_f_3 gyro;
	struct vector_f_3 gyro_zero;
	struct vector_f_3 mag;
	float temperature;
};

struct matrix_3x3_ty
{
	struct vector_f_3 a;
	struct vector_f_3 b;
	struct vector_f_3 c;
};

struct data_fusion_ty
{
	struct imu_ty imu;
	struct quaternions_ty quaternions;
	struct heading_angle_ty heading_angle;
	struct vector_f_3 e_gyro;
	struct matrix_3x3_ty e_b_transf_mat;
	struct matrix_3x3_ty b_e_transf_mat;
	float start_accel_g;
	float start_yaw;
};

extern struct data_fusion_ty attitude_data;

extern float ob_height;           //融后高度
extern float ob_height_speed;     //融后速度
extern float height_acc_zero;
extern float ob_height_acc;
extern mag_data_t mag_data;

int attitude_init(struct data_fusion_ty * data_fusion);
void attitude_update(struct data_fusion_ty * body_data, float dt);
struct vector_f_3 get_gravity_b(void);

int imu_init(void);
void imu_read(float dt);
void key_calibration(uint8_t exec_now);
void height_fusion_100(float dt);

float getBodySpeed(uint8_t axis);
float getBodyPosition(uint8_t axis);

#endif
