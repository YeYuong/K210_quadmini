/*
 *	无人机姿态与位置控制
 */
#include <stdio.h>
#include <math.h>
#include <gpio.h>
#include "io_func.h"
#include "motor.h"
#include "imu.h"
#include "control.h"
#include "vl53l1x.h"
#include "pmw3901mb.h"
#include "data_exchange_printf.h"
#include "task.h"
#include "commander.h"
#include <sysctl.h>
#include <led.h>
#include <motioncap.h>

#define TO_180_DEGREES(x)	(x > 180  ? (x - 360) : (x < -180 ? (x + 360) : x))
#define RAD_TO_ANG			57.2957795
#define LIMIT(x,min,max) 	((x) = ((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x) )))
#define LIMIT_R(x,min,max) 	((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x)))


struct pid_data_ty pid_data = {
	.angle_pid_param[0] = 		{6.0, 0.1, 0.0},
	.angle_pid_param[1] = 		{6.0, 0.1, 0.0},
	.angle_pid_param[2] = 		{8.0, 0.0, 0.0},

	.angle_speed_pid_param[0] = {0.7, 0.1, 0.007},
	.angle_speed_pid_param[1] = {0.7, 0.1, 0.007},
	.angle_speed_pid_param[2] = {2.0, 2.5, 0.0},

	.height_pid_param = 		{2.5, 0.0, 0.0},
	.height_speed_pid_param = 	{250.0, 30.0, 8.0},

	.x_speed_pid_param =		{25.0, 2.0, 12.0},
	.y_speed_pid_param =		{25.0, 2.0, 12.0},

	.x_posi_pid_param = 		{1.2, 0.0, 0.5},
	.y_posi_pid_param = 		{1.2, 0.0, 0.5},
	.thr_hover = 40
};

int ctrl_init(struct ctrl_data_ty * ctrl_data)
{
	ctrl_data->angle_ctrl.pid_param[0] = pid_data.angle_pid_param[0];
	ctrl_data->angle_ctrl.pid_param[1] = pid_data.angle_pid_param[1];
	ctrl_data->angle_ctrl.pid_param[2] = pid_data.angle_pid_param[2];
	ctrl_data->angle_speed_ctrl.pid_param[0] = pid_data.angle_speed_pid_param[0];
	ctrl_data->angle_speed_ctrl.pid_param[1] = pid_data.angle_speed_pid_param[1];
	ctrl_data->angle_speed_ctrl.pid_param[2] = pid_data.angle_speed_pid_param[2];
	ctrl_data->height_ctrl.pid_param = pid_data.height_pid_param;
	ctrl_data->height_speed_ctrl.pid_param = pid_data.height_speed_pid_param;
	ctrl_data->x_speed_ctrl.pid_param = pid_data.x_speed_pid_param;
	ctrl_data->y_speed_ctrl.pid_param = pid_data.y_speed_pid_param;
	ctrl_data->x_posi_ctrl.pid_param = pid_data.x_posi_pid_param;
	ctrl_data->y_posi_ctrl.pid_param = pid_data.y_posi_pid_param;
	ctrl_data->thr_hover = pid_data.thr_hover;
	ctrl_reset(ctrl_data);

	return 0;
}

void ctrl_reset(struct ctrl_data_ty * ctrl_data)
{
	struct vector_f_3 vector_f_3_zero = {0.0, 0.0, 0.0};
	ctrl_data->angle_ctrl.expect_angle = vector_f_3_zero;
	ctrl_data->angle_ctrl.old_angle_err = vector_f_3_zero;
	ctrl_data->angle_ctrl.angle_err_i = vector_f_3_zero;
	ctrl_data->angle_speed_ctrl.expect_angle_speed = vector_f_3_zero;
	ctrl_data->angle_speed_ctrl.old_angle_speed_err = vector_f_3_zero;
	ctrl_data->angle_speed_ctrl.angle_speed_err_i = vector_f_3_zero;
	ctrl_data->height_ctrl.expect_height = 0.0;
	ctrl_data->height_ctrl.height_err_i = 0.0;
	ctrl_data->height_ctrl.old_height_err = 0.0;
	ctrl_data->height_speed_ctrl.expect_height_speed = 0.0;
	ctrl_data->height_speed_ctrl.height_speed_err_i = 0.0;
	ctrl_data->height_speed_ctrl.old_height_speed_err = 0.0;
	ctrl_data->x_posi_ctrl.expect_x_posi = 0.0;
	ctrl_data->x_posi_ctrl.x_posi_err_i = 0.0;
	ctrl_data->x_posi_ctrl.old_x_posi_err = 0.0;
	ctrl_data->y_posi_ctrl.expect_y_posi = 0.0;
	ctrl_data->y_posi_ctrl.y_posi_err_i = 0.0;
	ctrl_data->y_posi_ctrl.old_y_posi_err = 0.0;
	ctrl_data->x_speed_ctrl.expect_x_speed = 0.0;
	ctrl_data->x_speed_ctrl.x_speed_err_i = 0.0;
	ctrl_data->x_speed_ctrl.old_x_speed_err = 0.0;
	ctrl_data->y_speed_ctrl.expect_y_speed = 0.0;
	ctrl_data->y_speed_ctrl.y_speed_err_i = 0.0;
	ctrl_data->y_speed_ctrl.old_y_speed_err = 0.0;
	ctrl_data->out_roll = 0.0;
	ctrl_data->out_pitch = 0.0;
	ctrl_data->out_yaw = 0.0;
	OpFlowDataClear();
}

void motor_throttle_ctrl(struct ctrl_data_ty * body_ctrl, struct Motors * motor, struct remote_ctrl_ty * remote_ctrl, float dt)
{
    float battery_thr_cmp = 0;
	float thr = body_ctrl->thr + body_ctrl->height_speed_ctrl.out_throttle + battery_thr_cmp;

	float out_roll  = body_ctrl->angle_speed_ctrl.out_motor_speed.x;// * ((thr / (body_ctrl->thr_hover)) > 1.0 ? 1.0 : (thr / (body_ctrl->thr_hover)));/*起飞油门*/
	float out_pitch = body_ctrl->angle_speed_ctrl.out_motor_speed.y;// * ((thr / (body_ctrl->thr_hover)) > 1.0 ? 1.0 : (thr / (body_ctrl->thr_hover)));
	float out_yaw   = body_ctrl->angle_speed_ctrl.out_motor_speed.z;// * ((thr / (body_ctrl->thr_hover)) > 1.0 ? 1.0 : (thr / (body_ctrl->thr_hover)));
	LIMIT(out_roll , -30, 30);
	LIMIT(out_pitch, -30, 30);
	LIMIT(out_yaw  , -40, 40);

	if(UNLOCKED == get_motor_lock())
	{
		//body_ctrl->thr_weight = body_ctrl->height_speed_ctrl.out_throttle / (MAX_THROTTLE - MIN_THROTTLE);
		body_ctrl->out_roll = out_roll;
		body_ctrl->out_pitch = out_pitch;
		body_ctrl->out_yaw = out_yaw;

		motor->motor_throttle.motor1 = thr + out_roll - out_pitch + out_yaw;
		motor->motor_throttle.motor2 = thr + out_roll + out_pitch - out_yaw;
		motor->motor_throttle.motor3 = thr - out_roll + out_pitch + out_yaw;
		motor->motor_throttle.motor4 = thr - out_roll - out_pitch - out_yaw;
	}
	set_motor_throttle(motor);
    // printf("m:%d,%d,%d,%d\n", motor->motor_throttle.motor1, motor->motor_throttle.motor2, motor->motor_throttle.motor3, motor->motor_throttle.motor4);
}

void flight_status_control(struct ctrl_data_ty * body_ctrl, struct remote_ctrl_ty * remote_ctrl, float dt) {
	if (body_ctrl->flight_status == FLIGHT_LOCKED) {	/* 电机锁定 */
		if (remote_ctrl->unlock_btn == 1) {
			unlock_motor();
			body_ctrl->flight_status = FLIGHT_UNLOCKED;
			body_ctrl->thr = 0.0;
			attitude_data.start_yaw = attitude_data.heading_angle.yaw; //解锁瞬间记录当前机头方向
			// gpio_set_pin(LED1_GPIO, GPIO_PV_LOW);
		} else {
			lock_motor();
		}
	} else if (body_ctrl->flight_status == FLIGHT_UNLOCKED) {	/* 电机解锁，低转速 */
		if (remote_ctrl->takeoff_btn == 1) {
			body_ctrl->flight_status = FLIGHT_ACCELERATE;
			body_ctrl->thr_weight = 0.0;
			rgb_set(0, 15, 0);
		} else if (remote_ctrl->lock_btn == 1) {
			body_ctrl->flight_status = FLIGHT_LOCKED;
			lock_motor();
		}
	} else if (body_ctrl->flight_status == FLIGHT_ACCELERATE) {	/* 电机加速至悬停油门 */
		body_ctrl->thr += body_ctrl->thr_hover / (1.0 /*秒*/ / dt);
		rgb_set(0, (uint8_t)(int)body_ctrl->thr, 0);
		if (body_ctrl->thr >= body_ctrl->thr_hover) {
			body_ctrl->thr = body_ctrl->thr_hover;
			body_ctrl->flight_status = FLIGHT_TAKING_OFF;
        	body_ctrl->x_posi_ctrl.expect_x_posi = getBodyPosition(0);
        	body_ctrl->y_posi_ctrl.expect_y_posi = getBodyPosition(1);
			rgb_set(15, 20, 15);
		}
		if (remote_ctrl->lock_btn == 1) {
			body_ctrl->flight_status = FLIGHT_LOCKED;
			lock_motor();
		}
	} else if (body_ctrl->flight_status == FLIGHT_TAKING_OFF) {	/* 起飞：启动角度环、高度速度环，直到高度为10cm */
		// body_ctrl->angle_ctrl.expect_angle.x = 0.0;
		// body_ctrl->angle_ctrl.expect_angle.y = 0.0;
		body_ctrl->height_speed_ctrl.expect_height_speed = 0.16;
		if (ob_height >= 0.2) {
			body_ctrl->height_ctrl.expect_height = 0.25;
			body_ctrl->flight_status = FLIGHT_FLYING;
		}
		if (remote_ctrl->lock_btn == 1) {
			body_ctrl->flight_status = FLIGHT_LOCKED;
			lock_motor();
		}
	} else if (body_ctrl->flight_status == FLIGHT_FLYING) {	/* 飞行：启动所有控制器，响应飞行指令 */
		if (remote_ctrl->land_btn == 1) {
			body_ctrl->flight_status = FLIGHT_LANDING;
			rgb_set(0, 0, 15);
		}
		if (remote_ctrl->lock_btn == 1) {
			body_ctrl->flight_status = FLIGHT_LOCKED;
			lock_motor();
		}
	} else if (body_ctrl->flight_status == FLIGHT_LANDING) {	/* 降落：启动高度速度环缓慢下降，小于5cm落下 */
		body_ctrl->height_speed_ctrl.expect_height_speed = -0.25;
		if ( (remote_ctrl->lock_btn == 1) || (ob_height <= 0.05) || getTOFHeight() <= 0.05) {
			body_ctrl->flight_status = FLIGHT_LOCKED;
			lock_motor();
			ctrl_reset(&global_data.body_ctrl);
			rgb_set(5,10,5);
		}
	}
}

void angle_speed_ctrl(struct ctrl_data_ty * body_ctrl,struct data_fusion_ty * body_data , float dt)
{
	static float angle_speed_z = 0.0;
	struct angle_speed_ctrl_ty * angle_speed_ctrl = &(body_ctrl->angle_speed_ctrl);

	if(body_ctrl->flight_status < FLIGHT_TAKING_OFF)
	{
		angle_speed_ctrl->angle_speed_err_i.x = 0;
		angle_speed_ctrl->angle_speed_err_i.y = 0;
		angle_speed_ctrl->angle_speed_err_i.z = 0;
		body_ctrl->angle_speed_ctrl.out_motor_speed.x = 0;
		body_ctrl->angle_speed_ctrl.out_motor_speed.y = 0;
		body_ctrl->angle_speed_ctrl.out_motor_speed.z = 0;
		return;
	}

	//获取期望角速度
	angle_speed_ctrl->expect_angle_speed.x = body_ctrl->angle_ctrl.out_angle_speed.x;
	angle_speed_ctrl->expect_angle_speed.y = body_ctrl->angle_ctrl.out_angle_speed.y;
	angle_speed_ctrl->expect_angle_speed.z = body_ctrl->angle_ctrl.out_angle_speed.z;
	LIMIT(angle_speed_ctrl->expect_angle_speed.x, -400.0, 400.0);
	LIMIT(angle_speed_ctrl->expect_angle_speed.y, -400.0, 400.0);
	LIMIT(angle_speed_ctrl->expect_angle_speed.z, -400.0, 400.0);

	angle_speed_z += (body_data->imu.gyro.z * RAD_TO_ANG - angle_speed_z) * 0.10f; // 角速度滤波

	//计算角速度误差
	angle_speed_ctrl->angle_speed_err.x = angle_speed_ctrl->expect_angle_speed.x - body_data->imu.gyro.x * RAD_TO_ANG;
	angle_speed_ctrl->angle_speed_err.y = angle_speed_ctrl->expect_angle_speed.y - body_data->imu.gyro.y * RAD_TO_ANG;
	angle_speed_ctrl->angle_speed_err.z = angle_speed_ctrl->expect_angle_speed.z - angle_speed_z;

	//计算角速度误差微分
	angle_speed_ctrl->angle_speed_err_d.x = angle_speed_ctrl->pid_param[0].kd * (angle_speed_ctrl->angle_speed_err.x - angle_speed_ctrl->old_angle_speed_err.x) / dt;
	angle_speed_ctrl->angle_speed_err_d.y = angle_speed_ctrl->pid_param[1].kd * (angle_speed_ctrl->angle_speed_err.y - angle_speed_ctrl->old_angle_speed_err.y) / dt;
	angle_speed_ctrl->angle_speed_err_d.z = angle_speed_ctrl->pid_param[2].kd * (angle_speed_ctrl->angle_speed_err.z - angle_speed_ctrl->old_angle_speed_err.z) / dt;

	//计算角速度误差积分
	angle_speed_ctrl->angle_speed_err_i.x += angle_speed_ctrl->pid_param[0].ki * angle_speed_ctrl->angle_speed_err.x * dt;
	angle_speed_ctrl->angle_speed_err_i.y += angle_speed_ctrl->pid_param[1].ki * angle_speed_ctrl->angle_speed_err.y * dt;
	angle_speed_ctrl->angle_speed_err_i.z += angle_speed_ctrl->pid_param[2].ki * angle_speed_ctrl->angle_speed_err.z * dt;
	LIMIT(angle_speed_ctrl->angle_speed_err_i.x, -150.0, 150.0);
	LIMIT(angle_speed_ctrl->angle_speed_err_i.y, -150.0, 150.0);
	LIMIT(angle_speed_ctrl->angle_speed_err_i.z, -10.0, 10.0);

	angle_speed_ctrl->out_motor_speed.x = angle_speed_ctrl->pid_param[0].kp * (angle_speed_ctrl->angle_speed_err.x + angle_speed_ctrl->angle_speed_err_d.x + angle_speed_ctrl->angle_speed_err_i.x);
	angle_speed_ctrl->out_motor_speed.y = angle_speed_ctrl->pid_param[1].kp * (angle_speed_ctrl->angle_speed_err.y + angle_speed_ctrl->angle_speed_err_d.y + angle_speed_ctrl->angle_speed_err_i.y);
	angle_speed_ctrl->out_motor_speed.z = angle_speed_ctrl->pid_param[2].kp * (angle_speed_ctrl->angle_speed_err.z + angle_speed_ctrl->angle_speed_err_d.z + angle_speed_ctrl->angle_speed_err_i.z);

	angle_speed_ctrl->old_angle_speed_err = angle_speed_ctrl->angle_speed_err;
}

void angle_ctrl(struct ctrl_data_ty * body_ctrl,struct data_fusion_ty * body_data ,struct remote_ctrl_ty * remote_ctrl, float dt)
{
	struct angle_ctrl_ty * angle_ctrl = &(body_ctrl->angle_ctrl);

	if(body_ctrl->flight_status < FLIGHT_TAKING_OFF)
	{
		angle_ctrl->angle_err_i.x = 0;
		angle_ctrl->angle_err_i.y = 0;
		angle_ctrl->angle_err_i.z = 0;
		angle_ctrl->out_angle_speed.x = 0;
		angle_ctrl->out_angle_speed.y = 0;
		angle_ctrl->out_angle_speed.z = 0;
		return;
	}

	//获取角度期望值
	if(body_ctrl->flight_status >= FLIGHT_TAKING_OFF)
	{
		angle_ctrl->expect_angle.x = LIMIT_R(body_ctrl->y_speed_ctrl.out_roll, -20.0, 20.0);
		angle_ctrl->expect_angle.y = LIMIT_R(body_ctrl->x_speed_ctrl.out_pitch, -20.0, 20.0);
	}
	angle_ctrl->expect_angle.z = attitude_data.start_yaw += remote_ctrl->yaw * 100.0 * dt;

	//计算角度误差
	angle_ctrl->angle_err.x = TO_180_DEGREES(angle_ctrl->expect_angle.x - body_data->heading_angle.roll);
	angle_ctrl->angle_err.y = TO_180_DEGREES(angle_ctrl->expect_angle.y - body_data->heading_angle.pitch);
	angle_ctrl->angle_err.z = TO_180_DEGREES(angle_ctrl->expect_angle.z - body_data->heading_angle.yaw);

	//计算角度误差微分
	angle_ctrl->angle_err_d.x = angle_ctrl->pid_param[0].kd * (angle_ctrl->angle_err.x - angle_ctrl->old_angle_err.x) / dt;
	angle_ctrl->angle_err_d.y = angle_ctrl->pid_param[1].kd * (angle_ctrl->angle_err.y - angle_ctrl->old_angle_err.y) / dt;
	angle_ctrl->angle_err_d.z = angle_ctrl->pid_param[2].kd * (angle_ctrl->angle_err.z - angle_ctrl->old_angle_err.z) / dt;

	//计算角度误差积分
	angle_ctrl->angle_err_i.x += angle_ctrl->pid_param[0].ki * angle_ctrl->angle_err.x * dt;
	angle_ctrl->angle_err_i.y += angle_ctrl->pid_param[1].ki * angle_ctrl->angle_err.y * dt;
	angle_ctrl->angle_err_i.z += angle_ctrl->pid_param[2].ki * angle_ctrl->angle_err.z * dt;
	LIMIT(angle_ctrl->angle_err_i.x, -15.0, 15.0);
	LIMIT(angle_ctrl->angle_err_i.y, -15.0, 15.0);
	LIMIT(angle_ctrl->angle_err_i.z, -15.0, 15.0);

	angle_ctrl->old_angle_err = angle_ctrl->angle_err;

	//误差限幅
	LIMIT(angle_ctrl->angle_err.x, -90, 90);
	LIMIT(angle_ctrl->angle_err.y, -90, 90);
	LIMIT(angle_ctrl->angle_err.z, -180, 180);

	//计算输出角速度
	angle_ctrl->out_angle_speed.x = angle_ctrl->pid_param[0].kp * (angle_ctrl->angle_err.x + angle_ctrl->angle_err_d.x + angle_ctrl->angle_err_i.x);
	angle_ctrl->out_angle_speed.y = angle_ctrl->pid_param[1].kp * (angle_ctrl->angle_err.y + angle_ctrl->angle_err_d.y + angle_ctrl->angle_err_i.y);
	angle_ctrl->out_angle_speed.z = angle_ctrl->pid_param[2].kp * (angle_ctrl->angle_err.z + angle_ctrl->angle_err_d.z + angle_ctrl->angle_err_i.z);
}


void height_ctrl(struct ctrl_data_ty * body_ctrl,struct data_fusion_ty * body_data, struct remote_ctrl_ty * remote_ctrl, float dt)
{
	float _height_est;
	struct height_ctrl_ty * height_ctrl = &(body_ctrl->height_ctrl);
	_height_est = getBodyPosition(2);

	if(body_ctrl->flight_status != FLIGHT_FLYING)
	{
		height_ctrl->height_err_i = 0;
		return;
	}

	//从遥控器获取期望高度
	float exp_height_speed = remote_ctrl->throttle * 0.4;
	height_ctrl->expect_height += exp_height_speed * dt;

	if(global_data.flags.use_motioncap_data)
		LIMIT(height_ctrl->expect_height, 0.1, 1.8);
	else
		LIMIT(height_ctrl->expect_height, 0.1, 1.0);

	//计算高度误差
	height_ctrl->height_err = height_ctrl->expect_height - _height_est;

	//计算高度误差微分
	height_ctrl->height_err_d = height_ctrl->pid_param.kd * (0.4 * (height_ctrl->height_err - height_ctrl->old_height_err) / dt + 0.6 * (-getObHeightSpeed()));

	//计算高度误差积分
	height_ctrl->height_err_i += height_ctrl->pid_param.ki * height_ctrl->height_err * dt;
	LIMIT(height_ctrl->height_err_i, (-300), (300));

	height_ctrl->old_height_err = height_ctrl->height_err;

	//计算输出高度速度
	height_ctrl->out_height_speed = height_ctrl->pid_param.kp * height_ctrl->height_err + height_ctrl->height_err_d + height_ctrl->height_err_i;
	height_ctrl->out_height_speed = LIMIT_R(height_ctrl->out_height_speed, -0.5, 0.6);
}

float _height_speed_est;

void height_speed_ctrl(struct ctrl_data_ty * body_ctrl, struct data_fusion_ty * body_data, struct remote_ctrl_ty * remote_ctrl, float dt)
{
	struct height_speed_ctrl_ty * height_speed_ctrl = &(body_ctrl->height_speed_ctrl);
	_height_speed_est = getBodySpeed(MC_Z);

	if(body_ctrl->flight_status < FLIGHT_TAKING_OFF)
	{
		height_speed_ctrl->height_speed_err_i = 0;
		height_speed_ctrl->out_throttle = 0;
		return;
	}

	//由油门通道计算高度速度控制量
	float out_thr = 0;

	if(body_ctrl->flight_status == FLIGHT_FLYING)
		height_speed_ctrl->expect_height_speed = body_ctrl->height_ctrl.out_height_speed;
	// height_speed_ctrl->expect_height_speed = remote_ctrl->throttle * 0.4;

	//计算高度速度误差
	height_speed_ctrl->height_speed_err = height_speed_ctrl->expect_height_speed - _height_speed_est;

	//计算高度速度误差微分
	height_speed_ctrl->height_speed_err_d = height_speed_ctrl->pid_param.kd * (height_speed_ctrl->height_speed_err - height_speed_ctrl->old_height_speed_err) / dt;

	//计算高度速度误差积分
	height_speed_ctrl->height_speed_err_i += height_speed_ctrl->pid_param.ki * height_speed_ctrl->height_speed_err * dt;

	LIMIT(height_speed_ctrl->height_speed_err_i, -50, 50);

	height_speed_ctrl->old_height_speed_err = height_speed_ctrl->height_speed_err;

	//计算输出油门
	out_thr = height_speed_ctrl->pid_param.kp * height_speed_ctrl->height_speed_err + height_speed_ctrl->height_speed_err_d + height_speed_ctrl->height_speed_err_i;
	out_thr = 1.0 * LIMIT_R(out_thr, -40, 50);
	height_speed_ctrl->out_throttle = out_thr;
}

void xy_speed_ctrl(struct ctrl_data_ty * body_ctrl, struct data_fusion_ty * body_data, struct remote_ctrl_ty * remote_ctrl, float dt)
{
	struct x_speed_ctrl_ty * x_speed_ctrl = &(body_ctrl->x_speed_ctrl);
	struct y_speed_ctrl_ty * y_speed_ctrl = &(body_ctrl->y_speed_ctrl);

	if(body_ctrl->flight_status < FLIGHT_TAKING_OFF)
	{
		x_speed_ctrl->x_speed_err_i = 0;
		y_speed_ctrl->y_speed_err_i = 0;
		x_speed_ctrl->out_pitch = 0.0;
		y_speed_ctrl->out_roll = 0.0;
		return;
	}

	x_speed_ctrl->expect_x_speed = body_ctrl->x_posi_ctrl.out_x_sp;
	y_speed_ctrl->expect_y_speed = body_ctrl->y_posi_ctrl.out_y_sp;

	//计算速度误差
	x_speed_ctrl->x_speed_err = x_speed_ctrl->expect_x_speed - getBodySpeed(MC_X);
	y_speed_ctrl->y_speed_err = y_speed_ctrl->expect_y_speed - getBodySpeed(MC_Y);
    // if(y_speed_ctrl->y_speed_err < 0.0)
    // {
    //     y_speed_ctrl->y_speed_err *= 0.8;
    // }

    static float x_err_lpf = 0.0;
    static float y_err_lpf = 0.0;
    x_err_lpf += 0.4*(x_speed_ctrl->x_speed_err - x_err_lpf);
    y_err_lpf += 0.4*(y_speed_ctrl->y_speed_err - y_err_lpf);
    global_data.debug_data = y_err_lpf;
    global_data.motor_current = y_speed_ctrl->y_speed_err;

	//计算速度误差微分
	x_speed_ctrl->x_speed_err_d = x_speed_ctrl->pid_param.kd * (x_err_lpf - x_speed_ctrl->old_x_speed_err);
	y_speed_ctrl->y_speed_err_d = y_speed_ctrl->pid_param.kd * (y_err_lpf - y_speed_ctrl->old_y_speed_err);

	//计算速度误差积分
	x_speed_ctrl->x_speed_err_i += x_speed_ctrl->pid_param.ki * x_speed_ctrl->x_speed_err * dt;
	y_speed_ctrl->y_speed_err_i += y_speed_ctrl->pid_param.ki * y_speed_ctrl->y_speed_err * dt;
	LIMIT(x_speed_ctrl->x_speed_err_i, -10.0, 10.0);
	LIMIT(y_speed_ctrl->y_speed_err_i, -10.0, 10.0);

	x_speed_ctrl->old_x_speed_err = x_err_lpf;//x_speed_ctrl->x_speed_err;
	y_speed_ctrl->old_y_speed_err = y_err_lpf;//y_speed_ctrl->y_speed_err;

	//计算输出角度
	x_speed_ctrl->out_pitch = x_speed_ctrl->pid_param.kp * x_speed_ctrl->x_speed_err + x_speed_ctrl->x_speed_err_d + x_speed_ctrl->x_speed_err_i;
	y_speed_ctrl->out_roll = y_speed_ctrl->pid_param.kp * y_speed_ctrl->y_speed_err + y_speed_ctrl->y_speed_err_d + y_speed_ctrl->y_speed_err_i;
	x_speed_ctrl->out_pitch = LIMIT_R(x_speed_ctrl->out_pitch, -20.0, 20.0);
	y_speed_ctrl->out_roll = -LIMIT_R(y_speed_ctrl->out_roll, -20.0, 20.0);
}

void xy_position_ctrl(struct ctrl_data_ty * body_ctrl, struct data_fusion_ty * body_data, struct remote_ctrl_ty * remote_ctrl, float dt)
{
	struct x_posi_ctrl_ty * x_posi_ctrl = &(body_ctrl->x_posi_ctrl);
	struct y_posi_ctrl_ty * y_posi_ctrl = &(body_ctrl->y_posi_ctrl);

	if(body_ctrl->flight_status < FLIGHT_TAKING_OFF)
	{
		x_posi_ctrl->x_posi_err_i = 0;
		y_posi_ctrl->y_posi_err_i = 0;
		x_posi_ctrl->out_x_sp = 0;
		y_posi_ctrl->out_y_sp = 0;
		return;
	}

	//由RC通道计算xy速度控制量
	float exp_x_speed = remote_ctrl->pitch * 0.5;
	float exp_y_speed = -remote_ctrl->roll * 0.5;
	x_posi_ctrl->expect_x_posi += exp_x_speed * dt;
	y_posi_ctrl->expect_y_posi += exp_y_speed * dt;

	//计算位置误差
	x_posi_ctrl->x_posi_err = x_posi_ctrl->expect_x_posi - getBodyPosition(MC_X);
	y_posi_ctrl->y_posi_err = y_posi_ctrl->expect_y_posi - getBodyPosition(MC_Y);

    static float x_err_lpf = 0.0;
    static float y_err_lpf = 0.0;
    x_err_lpf += 0.3*(x_posi_ctrl->x_posi_err - x_err_lpf);
    y_err_lpf += 0.3*(y_posi_ctrl->y_posi_err - y_err_lpf);
    
	//计算位置误差微分
	x_posi_ctrl->x_posi_err_d = x_posi_ctrl->pid_param.kd * (x_err_lpf - x_posi_ctrl->old_x_posi_err);
	y_posi_ctrl->y_posi_err_d = y_posi_ctrl->pid_param.kd * (y_err_lpf - y_posi_ctrl->old_y_posi_err);

	//计算位置误差积分
	x_posi_ctrl->x_posi_err_i += x_posi_ctrl->pid_param.ki * x_posi_ctrl->x_posi_err * dt;
	y_posi_ctrl->y_posi_err_i += y_posi_ctrl->pid_param.ki * y_posi_ctrl->y_posi_err * dt;
	LIMIT(x_posi_ctrl->x_posi_err_i, -1.0, 1.0);
	LIMIT(y_posi_ctrl->y_posi_err_i, -1.0, 1.0);

	x_posi_ctrl->old_x_posi_err = x_err_lpf;//x_posi_ctrl->x_posi_err;
	y_posi_ctrl->old_y_posi_err = y_err_lpf;//y_posi_ctrl->y_posi_err;

	//计算输出速度
	x_posi_ctrl->out_x_sp = x_posi_ctrl->pid_param.kp * x_posi_ctrl->x_posi_err + x_posi_ctrl->x_posi_err_d + x_posi_ctrl->x_posi_err_i;
	y_posi_ctrl->out_y_sp = y_posi_ctrl->pid_param.kp * y_posi_ctrl->y_posi_err + y_posi_ctrl->y_posi_err_d + y_posi_ctrl->y_posi_err_i;
	LIMIT(x_posi_ctrl->out_x_sp, -0.8, 0.8);
	LIMIT(y_posi_ctrl->out_y_sp, -0.8, 0.8);
}

// void test_control(float dt)
// {
// 	static float time = 0;
// 	time += dt;
// 	if(time <= dt + 0.0001)
// 	{
// 		unlock_motor();
// 		global_data.body_ctrl.flight_status = FLIGHT_TAKING_OFF;
// 	}
// 	if(global_data.body_ctrl.flight_status == FLIGHT_FLYING)
// 	{
// 		global_data.body_ctrl.angle_speed_ctrl.expect_angle_speed.z = 60;
// 	}
// 	if(time >= 5.0)
// 	{
// 		global_data.body_ctrl.angle_speed_ctrl.expect_angle_speed.z = 0;
//     	global_data.body_ctrl.flight_status = FLIGHT_LANDING;
// 	}
// }
