#include <imu.h>
#include <math.h>
#include <printf.h>
#include <stdio.h>
#include <sysctl.h>
#include <motioncap.h>
#include <task.h>

#define TO_180_DEGREES(x) (x > 180 ? (x - 360) : (x < -180 ? (x + 360) : x))

float vec_2_multip_cross(struct vector_f_2 a, struct vector_f_2 b);
struct vector_f_3 vec_3_multip_cross(struct vector_f_3 a, struct vector_f_3 b);
struct vector_f_3 vec_3_transf(struct vector_f_3 vec,
                               const struct matrix_3x3_ty* mat);
float vec_2_normalize(struct vector_f_2* vec);
float vec_3_normalize(struct vector_f_3* vec);
struct vector_f_2 vec_3_project_vec_2(struct vector_f_3 vec);
void quaternions_normalize(struct quaternions_ty* quat);
struct matrix_3x3_ty inverse_matrix_3x3(const struct matrix_3x3_ty* matrix);
void update_quaternions(struct vector_f_3 w, struct quaternions_ty* quat,
                        float dt);
void update_eulerian_angle(struct quaternions_ty quat,
                           struct heading_angle_ty* eulerian_angle);
void update_attitude_matrix(struct quaternions_ty quat,
                            struct matrix_3x3_ty* matrix,
                            struct matrix_3x3_ty* inverse_matrix);
void update_e_gyro(struct heading_angle_ty quat,
					struct vector_f_3 b_gyro,
					struct vector_f_3* e_gyro);
void acc_compensation(struct vector_f_3 acc, struct vector_f_3* gyro,
                      const struct vector_f_3 ref_gravity_vec,
                      const struct matrix_3x3_ty* e_b_mat, float dt);
void mag_compensation(struct vector_f_3 mag, struct vector_f_3* gyro,
                      const struct vector_f_3 ref_gravity_vec,
                      const struct matrix_3x3_ty* b_e_mat, float dt);
void gyro_correction(struct vector_f_3* gyro, struct vector_f_3 acc,
                     struct vector_f_3 mag, const struct matrix_3x3_ty* e_b_mat,
                     const struct matrix_3x3_ty* b_e_mat, float dt);

int attitude_init(struct data_fusion_ty* data_fusion) {
	struct heading_angle_ty heading_angle_init_data = {
	    .pitch = 0.0,
	    .roll = 0.0,
	    .yaw = 0.0,
	};
	struct quaternions_ty quaternions_init_data = {1.0, 0.0, 0.0, 0.0};
	/*变量初始化*/
	data_fusion->heading_angle = heading_angle_init_data;
	data_fusion->quaternions = quaternions_init_data;

	return 0;
}

void attitude_update(struct data_fusion_ty* body_data, float dt)
{
	struct imu_ty* imu = &(body_data->imu);
	struct heading_angle_ty* heading_angle = &(body_data->heading_angle);
	struct vector_f_3 * e_gyro = &(body_data->e_gyro);
	struct vector_f_3 acc = imu->acc;
	struct vector_f_3 gyro = imu->gyro;
	struct vector_f_3 mag = imu->mag;
	struct quaternions_ty* quaternions = &(body_data->quaternions);
	static struct matrix_3x3_ty e_b_transf_matrix = {
	    {1.0, 0.0, 0.0},
	    {0.0, 1.0, 0.0},
	    {0.0, 0.0, 1.0},
	};  // E系到b系的变换矩阵
	static struct matrix_3x3_ty b_e_transf_matrix = {
	    {1.0, 0.0, 0.0},
	    {0.0, 1.0, 0.0},
	    {0.0, 0.0, 1.0},
	};  // b系到E系的变换矩阵

	gyro_correction(&gyro, acc, mag, &e_b_transf_matrix, &b_e_transf_matrix, dt);
	update_quaternions(gyro, quaternions, dt);
	update_attitude_matrix(*quaternions, &b_e_transf_matrix, &e_b_transf_matrix);
	update_eulerian_angle(*quaternions, heading_angle);//更新欧拉角
	update_e_gyro(*heading_angle, imu->gyro, e_gyro);
	// printf("heading:%.6f, %.6f, %.6f\n", heading_angle->roll, heading_angle->pitch, heading_angle->yaw);
	// printf("quat:%10.4f, %10.4f, %10.4f, %10.4f\n\r", quaternions->q0, quaternions->q1, quaternions->q2, quaternions->q3);

	body_data->b_e_transf_mat = b_e_transf_matrix;
	body_data->e_b_transf_mat = e_b_transf_matrix;
}

float vec_2_multip_cross(struct vector_f_2 a, struct vector_f_2 b)
{
  	return a.x * b.y - a.y * b.x;
}

struct vector_f_3 vec_3_multip_cross(struct vector_f_3 a, struct vector_f_3 b)
{
	struct vector_f_3 ret;
	ret.x = a.y * b.z - a.z * b.y;
	ret.y = a.z * b.x - a.x * b.z;
	ret.z = a.x * b.y - a.y * b.x;
	return ret;
}

struct vector_f_3 vec_3_transf(struct vector_f_3 vec, const struct matrix_3x3_ty* mat)
{
	struct vector_f_3 ret;
	ret.x = mat->a.x * vec.x + mat->a.y * vec.y + mat->a.z * vec.z;
	ret.y = mat->b.x * vec.x + mat->b.y * vec.y + mat->b.z * vec.z;
	ret.z = mat->c.x * vec.x + mat->c.y * vec.y + mat->c.z * vec.z;
	return ret;
}

float vec_2_normalize(struct vector_f_2* vec)
{
	float mod = sqrt(vec->x * vec->x + vec->y * vec->y);
	if (fabs(mod) > 1e-6) {
		vec->x /= mod;
		vec->y /= mod;
	} else {
		vec->x = 0;
		vec->y = 0;
	}
	return mod;
}

float vec_3_normalize(struct vector_f_3* vec)
{
	float mod = sqrt(vec->x * vec->x + vec->y * vec->y + vec->z * vec->z);
	if (fabs(mod) > 1e-6) {
		vec->x /= mod;
		vec->y /= mod;
		vec->z /= mod;
	} else {
		vec->x = 0;
		vec->y = 0;
		vec->z = 0;
	}
	return mod;
}

struct vector_f_2 vec_3_project_vec_2(struct vector_f_3 vec) {
	struct vector_f_2 ret;
	ret.x = vec.x;
	ret.y = vec.y;
	return ret;
}

void quaternions_normalize(struct quaternions_ty* quat) {
	float mod = sqrt(quat->q0 * quat->q0 + quat->q1 * quat->q1 + quat->q2 * quat->q2 + quat->q3 * quat->q3);
	if (fabs(mod) > 1e-6) {
		quat->q0 /= mod;
		quat->q1 /= mod;
		quat->q2 /= mod;
		quat->q3 /= mod;
	} else {
		quat->q0 = 1.0;
		quat->q1 = 0.0;
		quat->q2 = 0.0;
		quat->q3 = 0.0;
	}
}

struct matrix_3x3_ty inverse_matrix_3x3(const struct matrix_3x3_ty* matrix) {
	struct matrix_3x3_ty inverse;
	inverse.a.x = matrix->a.x;
	inverse.a.y = matrix->b.x;
	inverse.a.z = matrix->c.x;
	inverse.b.x = matrix->a.y;
	inverse.b.y = matrix->b.y;
	inverse.b.z = matrix->c.y;
	inverse.c.x = matrix->a.z;
	inverse.c.y = matrix->b.z;
	inverse.c.z = matrix->c.z;
	return inverse;
}

void update_quaternions(struct vector_f_3 w, struct quaternions_ty* quat,
                        float dt) {
  struct quaternions_ty last_quat = *quat;
	quat->q0 = last_quat.q0 + (-last_quat.q1 * w.x - last_quat.q2 * w.y - last_quat.q3 * w.z) * dt / 2;
	quat->q1 = last_quat.q1 + (last_quat.q0 * w.x + last_quat.q2 * w.z - last_quat.q3 * w.y) * dt / 2;
	quat->q2 = last_quat.q2 + (last_quat.q0 * w.y - last_quat.q1 * w.z + last_quat.q3 * w.x) * dt / 2;
	quat->q3 = last_quat.q3 + (last_quat.q0 * w.z + last_quat.q1 * w.y - last_quat.q2 * w.x) * dt / 2;
	quaternions_normalize(quat);
}

void update_eulerian_angle(struct quaternions_ty quat,
                           struct heading_angle_ty* eulerian_angle) {
	eulerian_angle->pitch = -asinf(2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2)) * RAD_TO_ANG;
	eulerian_angle->roll = atan2((2.0 * (quat.q0 * quat.q1 + quat.q2 * quat.q3)), (quat.q0 * quat.q0 - quat.q1 * quat.q1 - quat.q2 * quat.q2 + quat.q3 * quat.q3)) * RAD_TO_ANG;
	eulerian_angle->yaw = atan2((2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3)), (quat.q0 * quat.q0 + quat.q1 * quat.q1 - quat.q2 * quat.q2 - quat.q3 * quat.q3)) * RAD_TO_ANG;
}

/*matrix指b系到E系的变换矩阵*/
void update_attitude_matrix(struct quaternions_ty quat,
                            struct matrix_3x3_ty* matrix,
                            struct matrix_3x3_ty* inverse_matrix)
{
	// printf("quat(%10.6f, %10.6f, %10.6f, %10.6f) \n\r", quat.q0, quat.q1,
	// quat.q2, quat.q3);
	matrix->a.x = quat.q0 * quat.q0 + quat.q1 * quat.q1 - quat.q2 * quat.q2 -
	              quat.q3 * quat.q3;
	matrix->a.y = 2 * (quat.q1 * quat.q2 - quat.q0 * quat.q3);
	matrix->a.z = 2 * (quat.q1 * quat.q3 + quat.q0 * quat.q2);
	matrix->b.x = 2 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
	matrix->b.y = quat.q0 * quat.q0 - quat.q1 * quat.q1 + quat.q2 * quat.q2 -
	              quat.q3 * quat.q3;
	matrix->b.z = 2 * (quat.q2 * quat.q3 - quat.q0 * quat.q1);
	matrix->c.x = 2 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
	matrix->c.y = 2 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
	matrix->c.z = quat.q0 * quat.q0 - quat.q1 * quat.q1 - quat.q2 * quat.q2 +
	              quat.q3 * quat.q3;

	*inverse_matrix = inverse_matrix_3x3(matrix);
}

void update_e_gyro(struct heading_angle_ty heading_angle,
					struct vector_f_3 b_gyro,
					struct vector_f_3* e_gyro)
{
	e_gyro->x = (b_gyro.x ) + sin(heading_angle.roll * 3.1415926 / 180.0) * tan(heading_angle.pitch * 3.1415926 / 180.0) * (-b_gyro.y ) + cos(heading_angle.roll * 3.1415926 / 180.0) * tan(heading_angle.pitch * 3.1415926 / 180.0) * (-b_gyro.z );
	e_gyro->y = cos(heading_angle.roll * 3.1415926 / 180.0) * (-b_gyro.y ) - sin(heading_angle.roll * 3.1415926 / 180.0) * (-b_gyro.z );
	e_gyro->z = (sin(heading_angle.roll * 3.1415926 / 180.0) / cos(heading_angle.pitch * 3.1415926 / 180.0)) * (-b_gyro.y ) + (cos(heading_angle.roll * 3.1415926 / 180.0) / cos(heading_angle.pitch * 3.1415926 / 180.0)) * (-b_gyro.z );
}

void acc_compensation(struct vector_f_3 acc, struct vector_f_3* gyro,
                      const struct vector_f_3 ref_gravity_vec,
                      const struct matrix_3x3_ty* e_b_mat, float dt)
{
	struct vector_f_3 ref_tmp;
	static struct vector_f_3 ref_err_lpf = {0.0, 0.0, 0.0};
	struct vector_f_3 ref_err;
	static struct vector_f_3 ref_err_integ = {0.0, 0.0, 0.0};
	float ref_err_lpf_hz = REF_ERR_LPF_HZ * (3.14 * dt);
	float acc_mod = vec_3_normalize(&acc);
	// printf("acc_mod:%f\n\r", acc_mod);

	if (ACC_RANGE_LOW < acc_mod && acc_mod < ACC_RANGE_HIGH) {
		ref_tmp = vec_3_multip_cross(ref_gravity_vec, acc);

		ref_err_lpf.x += ref_err_lpf_hz * (ref_tmp.x - ref_err_lpf.x);
		ref_err_lpf.y += ref_err_lpf_hz * (ref_tmp.y - ref_err_lpf.y);
		ref_err_lpf.z += ref_err_lpf_hz * (ref_tmp.z - ref_err_lpf.z);

		ref_err = ref_err_lpf;
	} else {
		ref_err.x = 0;
		ref_err.y = 0;
		ref_err.z = 0;
	}
	ref_err_integ.x += ref_err.x * dt;
	ref_err_integ.y += ref_err.y * dt;
	ref_err_integ.z += ref_err.z * dt;
#define LIMIT(var, min, max) (var) = (var) < (min) ? (min) : ((var) > (max) ? (max) : (var))
	LIMIT(ref_err_integ.x, -ERR_INTEGRAL_MAX, ERR_INTEGRAL_MAX);
	LIMIT(ref_err_integ.y, -ERR_INTEGRAL_MAX, ERR_INTEGRAL_MAX);
	LIMIT(ref_err_integ.z, -ERR_INTEGRAL_MAX, ERR_INTEGRAL_MAX);

	gyro->x = gyro->x - ACCEL_COMPENSATION_KP * ref_err.x - ACCEL_COMPENSATION_KI * ref_err_integ.x;
	gyro->y = gyro->y - ACCEL_COMPENSATION_KP * ref_err.y - ACCEL_COMPENSATION_KI * ref_err_integ.y;
	gyro->z = gyro->z - ACCEL_COMPENSATION_KP * ref_err.z - ACCEL_COMPENSATION_KI * ref_err_integ.z;
}

void mag_compensation(struct vector_f_3 mag, struct vector_f_3* gyro,
	                    const struct vector_f_3 ref_gravity_vec,
	                    const struct matrix_3x3_ty* b_e_mat, float dt) {
	float mag_mod = vec_3_normalize(&mag);
	struct vector_f_2 standard_mag_vec = {1, 0};
	struct vector_f_3 mag_corr;
	struct vector_f_2 mag_corr_project;
	// float mag_corr_mod;
	float yaw_correct = 0;
	float yaw_err = 0;
	if (mag_mod > 1e-6) {
		mag_corr = vec_3_transf(mag, b_e_mat);
		mag_corr_project = vec_3_project_vec_2(mag_corr);
		vec_2_normalize(&mag_corr_project);
		yaw_err = vec_2_multip_cross(mag_corr_project, standard_mag_vec);
		// printf("magE (%10.5f, %10.5f)", mag_corr_project.x, mag_corr_project.y);
		// printf("ref (%10.5f, %10.5f)\n\r", standard_mag_vec.x,
		// standard_mag_vec.y);

		if(sysctl_get_time_us() < 4000000)
			yaw_correct =
				100 * MAG_COMPENSATION_KP * 1.0 * TO_180_DEGREES(asin(yaw_err) * RAD_TO_ANG);
		else
			yaw_correct =
				MAG_COMPENSATION_KP * 1.0 * TO_180_DEGREES(asin(yaw_err) * RAD_TO_ANG);
	}
	gyro->x = gyro->x - ref_gravity_vec.x * yaw_correct * ANG_TO_RAD;
	gyro->y = gyro->y - ref_gravity_vec.y * yaw_correct * ANG_TO_RAD;
	gyro->z = gyro->z - ref_gravity_vec.z * yaw_correct * ANG_TO_RAD;
}

void mc_compensation(float mc_yaw, struct vector_f_3* gyro,
	                    const struct vector_f_3 ref_gravity_vec,
	                    float hd_yaw) {
	float yaw_correct = 0;
	float yaw_err = 0;

	yaw_err = hd_yaw - mc_yaw;

	yaw_correct = MC_COMPENSATION_KP * TO_180_DEGREES(yaw_err);

	gyro->x = gyro->x - ref_gravity_vec.x * yaw_correct * ANG_TO_RAD;
	gyro->y = gyro->y - ref_gravity_vec.y * yaw_correct * ANG_TO_RAD;
	gyro->z = gyro->z - ref_gravity_vec.z * yaw_correct * ANG_TO_RAD;
}

struct vector_f_3 ref_gravity_vec_b;

void gyro_correction(struct vector_f_3* gyro, struct vector_f_3 acc,
                     struct vector_f_3 mag, const struct matrix_3x3_ty* e_b_mat,
                     const struct matrix_3x3_ty* b_e_mat, float dt)
{
	struct vector_f_3 gravity_vec_e = {0, 0, 1};

	/*根据姿态矩阵计算b系的重力向量*/
	ref_gravity_vec_b = vec_3_transf(gravity_vec_e, e_b_mat);

#if ACC_COMPEN
	acc_compensation(acc, gyro, ref_gravity_vec_b, e_b_mat, dt);
#endif

#if MAG_COMPEN
	mag_compensation(mag, gyro, ref_gravity_vec_b, b_e_mat, dt);
#endif

	if(global_data.flags.use_motioncap_data)
		mc_compensation(motioncap_data.mc_euler[2], gyro, ref_gravity_vec_b, attitude_data.heading_angle.yaw);
}

struct vector_f_3 get_gravity_b(void) {
	return ref_gravity_vec_b;
}

// imu
#include <sysctl.h>
#include <timer.h>
#include <uarths.h>
#include <io_func.h>
#include <filter.h>
#include <task.h>
#include <gpio.h>
#include <led.h>
#include <pmw3901mb.h>
#include <vl53l1x.h>
#include <motioncap.h>
#include <param.h>

bmi088_real_data_t imu_data;
mag_data_t mag_data;
struct data_fusion_ty attitude_data;

float ob_height;
float ob_height_speed;
float height_acc_zero = 0.0;
float ob_height_acc;

void imu_timer_irq(void) {


}

void key_calibration(uint8_t exec_now)
{
	// 用遥控器按键：按下0.2s校准机头、光流清零；按下1s开始校准imu零漂直到按键松开
	// 用指令：校准机头、光流清零，并花1.5s校准零漂
	static uint16_t key = 0;
	static uint32_t tim = 40;
    if(global_data.body_ctrl.flight_status == FLIGHT_LOCKED && global_data.remote_ctrl.cali_btn == 1)
    {
		key++;
	}
    else
    {
		if(key > 20 || tim == 31)
		{
			global_param.gyro_zero = attitude_data.imu.gyro_zero;
			param_write_require(&global_param);
		}
		key = 0;
	}
	if(exec_now) tim = 0;
	tim++;
    if(key == 4 || tim == 1)
    {
		attitude_data.start_accel_g = imu_data.accel[2];
		attitude_data.start_yaw = attitude_data.heading_angle.yaw;
		// OpFlowDataClear();
		ctrl_reset(&global_data.body_ctrl);
    }
	if(key >= 20 || tim <= 30)
	{
		float n = (float)(key - 19);
		if(tim <= 30) n = (float)(tim);
		attitude_data.imu.gyro_zero.x = (attitude_data.imu.gyro_zero.x * (n-1) + imu_data.gyro[0]) / n;
		attitude_data.imu.gyro_zero.y = (attitude_data.imu.gyro_zero.y * (n-1) + imu_data.gyro[1]) / n;
		attitude_data.imu.gyro_zero.z = (attitude_data.imu.gyro_zero.z * (n-1) + imu_data.gyro[2]) / n;
		// printf("cali gyrozero:%f,%f,%f\n",attitude_data.imu.gyro_zero.x,attitude_data.imu.gyro_zero.y,attitude_data.imu.gyro_zero.z);
	}
	if((key >= 4 && key <= 10) || key >= 20 || tim <= 30) led_set(3, 1); else led_set(3, 0);
}

int imu_init(void) {
	uint8_t err;
	err = ist8310_init();
	if (err == IST8310_NO_ERROR)
		printk("IST8310 init success\n");
	else
		printk("IST8310 not found\n");

	err = BMI088_init();
	if (err == BMI088_NO_ERROR)
		printk("BMI088 init success\n");
	else {
		printk("BMI088 init err:%02X\n", err);
        global_data.flags.ready_to_takeoff = -1;
		if (err & BMI088_SELF_TEST_ACCEL_ERROR) printf("BMI Accel Error\n");
		if (err & BMI088_SELF_TEST_GYRO_ERROR) printf("BMI Gyro Error\n");
	}

	attitude_init(&attitude_data);
	// timer_init(IMU_USE_Timer);
	// timer_set_interval(IMU_USE_Timer, IMU_TIM_Channel, 1000000000 / IMU_Hz);
	// timer_set_irq(IMU_USE_Timer, IMU_TIM_Channel, imu_timer_irq, 1);
	// timer_set_enable(IMU_USE_Timer, IMU_TIM_Channel, 1);
	// sysctl_enable_irq();

	return err;
}

void imu_read(float dt) {
	// read data from sensor
	static uint8_t n = 0;
	static float acc_lpf[3] = {0.0,0.0,0.0};
	BMI088_read(imu_data.gyro, imu_data.accel, &imu_data.temp);
	if(n==0)
	{
		acc_lpf[0] = imu_data.accel[0];
		acc_lpf[1] = imu_data.accel[1];
		acc_lpf[2] = imu_data.accel[2];
		attitude_data.start_accel_g = imu_data.accel[2];
	}
	n++;
	if(n==6) {
		// IST_read(&mag_data);
		n=1;
	}

	// write data to attitude struct
	attitude_data.imu.temperature = imu_data.temp;
	attitude_data.imu.gyro.x = imu_data.gyro[0] - attitude_data.imu.gyro_zero.x;//角速度
	attitude_data.imu.gyro.y = imu_data.gyro[1] - attitude_data.imu.gyro_zero.y;
	attitude_data.imu.gyro.z = imu_data.gyro[2] - attitude_data.imu.gyro_zero.z;
	first_order_lpf(&acc_lpf[0], 10, dt, imu_data.accel[0]);
	first_order_lpf(&acc_lpf[1], 10, dt, imu_data.accel[1]);
	first_order_lpf(&acc_lpf[2], 8, dt, imu_data.accel[2]);
	attitude_data.imu.acc.x = acc_lpf[0];//加速度
	attitude_data.imu.acc.y = acc_lpf[1];
	attitude_data.imu.acc.z = acc_lpf[2];

	attitude_data.imu.mag.x = mag_data.mag_X;
	attitude_data.imu.mag.y = mag_data.mag_Y;
	attitude_data.imu.mag.z = mag_data.mag_Z;
	// printf("temp:%f ", attitude_data.imu.temperature);
	// printf("gyro:%f, %f, %f\n", attitude_data.imu.gyro.x, attitude_data.imu.gyro.y, attitude_data.imu.gyro.z);
	// printf("acc:%f, %f, %f\n", attitude_data.imu.acc.x, attitude_data.imu.acc.y, attitude_data.imu.acc.z);
	// printf("mag:%f %f %f\n\r", attitude_data.imu.mag.x, attitude_data.imu.mag.y, attitude_data.imu.mag.z);


	// quaternion calculation
	attitude_update(&attitude_data, 1.0 / IMU_Hz);
}

void height_fusion_100(float dt)
{
	//加速计预估
	float height_acc;
	
	height_acc   	= (attitude_data.imu.acc.x * get_gravity_b().x+attitude_data.imu.acc.y * get_gravity_b().y+attitude_data.imu.acc.z * get_gravity_b().z)-attitude_data.start_accel_g;
	ob_height_acc 	= (height_acc += height_acc_zero);
	ob_height_speed += ob_height_acc*dt;//对z轴加速度进行积分得到高度速度
	ob_height       += ob_height_speed*dt;//对高度速度进行积分得到高度值
}

// 获取机身速度及位置综合动捕数据和传感器数据
float getBodySpeed(uint8_t axis)
{
	if(global_data.flags.use_motioncap_data)
	{
		return getMotioncapSpeed(axis);
	}
	else
	{
		if(axis < 2)
			return getOpFlowSpeed(axis);
		else
			return getObHeightSpeed();
	}
}

float getBodyPosition(uint8_t axis)
{
	if(global_data.flags.use_motioncap_data)
	{
		return getMotioncapPosition(axis);
	}
	else
	{
		if(axis < 2)
			return getOpFlowPosition(axis);
		else
			return getObHeight();
	}
}
