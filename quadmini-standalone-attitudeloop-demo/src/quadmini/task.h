#ifndef _TASK_H
#define _TASK_H

#include <imu.h>
#include <control.h>
#include <commander.h>
#include <motor.h>

struct global_flags_ty
{
	uint8_t use_motioncap_data;
	uint8_t use_commander;
	uint8_t serial_connected;
    int8_t ready_to_takeoff;
};

struct global_data_ty
{
	struct ctrl_data_ty			body_ctrl;
	struct remote_ctrl_ty		remote_ctrl;
	struct Motors				motor;
	struct global_flags_ty		flags;
	float batt_voltage;
	float motor_current;
	float debug_data;
};

extern struct global_data_ty global_data;

int task_loop(struct global_data_ty * global_data);

#endif
