#include <motioncap.h>
#include <imu.h>
#include <task.h>
#include <sysctl.h>
#include <string.h>
#include <led.h>
#include <pmw3901mb.h>
#include <commander.h>

#define ob_1  0.9 //融合滤波系数
#define ob_2  0.35
#define ob_3  0.3

float mc_x_speed = 0.0;
float mc_y_speed = 0.0;
float mc_height;
float mc_height_speed;

uint8_t motioncap_update = 0;

motioncap_t motioncap_data;

float getMotioncapSpeed(mc_axis_enum axis)
{
    if(axis < 2)
        return motioncap_data.mc_vel[axis];
    else
        return ob_height_speed;
}

float getMotioncapPosition(mc_axis_enum axis)
{
    if(axis < 2)
        return motioncap_data.mc_pos[axis];
    else
        return ob_height;
}

void speed_fusion_mc(float dt)
{
	float spd[2];
	static float last_mc_x = 0.0;
	static float last_mc_y = 0.0;

	spd[MC_X] = (getMotioncapPosition(MC_X)-last_mc_x)/dt;
	spd[MC_Y] = (getMotioncapPosition(MC_Y)-last_mc_y)/dt;
	mc_x_speed += 0.2*(spd[MC_X]-mc_x_speed);
	mc_y_speed += 0.2*(spd[MC_Y]-mc_y_speed);
	motioncap_data.mc_vel_calc[MC_X] = mc_x_speed;
	motioncap_data.mc_vel_calc[MC_Y] = mc_y_speed;

	last_mc_x = getMotioncapPosition(MC_X);
	last_mc_y = getMotioncapPosition(MC_Y);
}

#define LIMIT_R(x,min,max) 	((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x)))

void height_fusion_20_mc(float dt)
{
	float new_height_MC, err1, err2;
	static float old_height_MC=0;
	new_height_MC = motioncap_data.mc_pos[MC_Z];
	// mc_height_speed = (new_height_MC-old_height_MC)/dt;
    mc_height_speed = motioncap_data.mc_vel[MC_Z];//

	err1 = (new_height_MC-ob_height);
	err2 = (mc_height_speed - ob_height_speed);
	
	ob_height       += ob_1 * err1;
	ob_height_speed += ob_2 * (err1 + err2);
	height_acc_zero = -LIMIT_R(global_data.body_ctrl.thr - 45, 0, 20)*0.13/20.0 + ob_3 * (err1 + err2);
	old_height_MC = new_height_MC;

	// motioncap_cradle(dt);
}

void motioncap_on_pack_received(void *data)
{
	led_set(2, 1);
	motioncap_update = 1;
	memcpy(&motioncap_data, data, 36);
	global_data.flags.use_motioncap_data = 1;
}

void motioncap_cradle(float dt)
{
	// 动捕摇篮系统
	static float interv_time = 0.0;
    if(global_data.flags.use_motioncap_data)
    {
        interv_time += dt;
        global_data.debug_data = interv_time;
        if(motioncap_update)
        {
            interv_time = 0.0;
            motioncap_update = 0;
        }
        if(interv_time >= 0.3)
        {
            global_data.flags.use_motioncap_data = 0;
            opFlow.posSum[OP_X] = 100.0 * motioncap_data.mc_pos[MC_X];
            opFlow.posSum[OP_Y] = 100.0 * motioncap_data.mc_pos[MC_Y];
            led_set(2, 0);
            if(global_data.flags.use_commander == 1)
            {
                commander_land();
            }
            printf("Motioncap offline, switch back to imu!\n");
        }
    }
}