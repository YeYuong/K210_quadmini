#include <commander.h>
#include <sysctl.h>
#include <task.h>
#include <imu.h>
#include <led.h>
#include <math.h>
#include <data_exchange_printf.h>

uint8_t cmdr_update = 0;

void commander_takeoff(void)
{
    // 起飞
    if(global_data.body_ctrl.flight_status == FLIGHT_LOCKED || global_data.body_ctrl.flight_status == FLIGHT_UNLOCKED)
    {
        unlock_motor();
        attitude_data.start_yaw = 0;
        global_data.body_ctrl.flight_status = FLIGHT_ACCELERATE;
        global_data.body_ctrl.thr = 0.0;
        global_data.body_ctrl.thr_weight = 0.0;
        global_data.body_ctrl.x_posi_ctrl.expect_x_posi = getBodyPosition(0);
        global_data.body_ctrl.y_posi_ctrl.expect_y_posi = getBodyPosition(1);
        
        rgb_set(0, 15, 0);
    }
    global_data.flags.use_commander = 1;
    cmdr_update = 1;
}

void commander_goto(float *axis)
{
    // 设置目标点
    if(global_data.body_ctrl.flight_status == FLIGHT_FLYING)
    {
        global_data.body_ctrl.x_posi_ctrl.expect_x_posi = axis[0];
        global_data.body_ctrl.y_posi_ctrl.expect_y_posi = axis[1];
        global_data.body_ctrl.height_ctrl.expect_height = axis[2];
        rgb_set(10, 10, 0);
    }
    cmdr_update = 1;
}
void commander_move(float *axis)
{
    // 设置目标点
    if(global_data.body_ctrl.flight_status == FLIGHT_FLYING)
    {
        global_data.body_ctrl.x_posi_ctrl.expect_x_posi = getBodyPosition(0) + axis[0];
        global_data.body_ctrl.y_posi_ctrl.expect_y_posi = getBodyPosition(1) + axis[1];
        global_data.body_ctrl.height_ctrl.expect_height = getBodyPosition(2) + axis[2];
        rgb_set(10, 20, 0);
        cmdr_update = 1;
    }
}


void commander_maintain(void)
{
    if(global_data.body_ctrl.flight_status == FLIGHT_FLYING)
    {
        if(fabs(getBodyPosition(0)-global_data.body_ctrl.x_posi_ctrl.expect_x_posi)<=0.1
        && fabs(getBodyPosition(1)-global_data.body_ctrl.y_posi_ctrl.expect_y_posi)<=0.1
        && fabs(getBodyPosition(2)-global_data.body_ctrl.height_ctrl.expect_height)<=0.1)
            rgb_set(15, 20, 15);
    }
    cmdr_update = 1;
}

void commander_land(void)
{
    // 设置为降落状态，并将高度下降到0.1m
    global_data.flags.use_commander = 0;
    global_data.body_ctrl.flight_status = FLIGHT_LANDING;
    // global_data.body_ctrl.x_posi_ctrl.expect_x_posi = getBodyPosition(0);
    // global_data.body_ctrl.y_posi_ctrl.expect_y_posi = getBodyPosition(1);
    // global_data.body_ctrl.height_ctrl.expect_height = 0.1;
    rgb_set(0, 0, 15);
}

void command_executor(commander_t *commander)
{
    // printf("cmd:%d, coord:%f,%f,%f\n", commander->cmds, commander->coordinate[0], commander->coordinate[1], commander->coordinate[2]);
    switch (commander->cmds)
    {
    case CMD_MAINTAIN:
        commander_maintain();
        break;
    case TAKE_OFF:
        commander_takeoff();
        break;
    case GOTO:
        commander_goto(commander->coordinate);
        break;
    case MOVE:
        commander_move(commander->coordinate);
        break;
    case LAND:
        commander_land();
        break;
    case RESET:
        sysctl_reset(SYSCTL_RESET_SOC);
        break;
    case CALIBRATE:
        key_calibration(1);
        break;
    
    default:
        break;
    }
}
void commander_cradle(float dt)
{
	// 指令摇篮系统
    // 0.5秒无指令退出指令模式并强制降落，指令间需持续发送MAINTAIN指令或重复指令
	static float interv_time = 0.0;
    if(global_data.flags.use_commander)
    {
        interv_time += dt;
        if(cmdr_update)
        {
            interv_time = 0.0;
            cmdr_update = 0;
        }
        if(interv_time >= 0.5)
        {
            commander_land();
            printf("No Commander Signal! Landing!\n");
        }
    }
}

// void commander_cradle(float dt) // start timing after goto command starts @asdf
// {
// 	// 指令摇篮系统
//     // 0.5秒无指令退出指令模式并强制降落，指令间需持续发送MAINTAIN指令
// 	static float interv_time = 0.0;
//     static uint8_t goto_cmd_start = 0;
//     if(global_data.flags.use_commander)
//     {
//         if(goto_cmd_start)
//             interv_time += dt;
//         if(cmdr_update)
//         {
//             goto_cmd_start = 1;
//             interv_time = 0.0;
//             cmdr_update = 0;
//         }
//         if(interv_time >= 0.5)
//         {
//             goto_cmd_start = 0;
//             interv_time = 0.0;
//             commander_land();
//             printf("No Commander Signal! Landing!\n");
//         }
//     } else { goto_cmd_start = 0;}
// }
