#include <data_exchange.h>
#include <data_exchange_printf.h>
#include <pmw3901mb.h>
#include <vl53l1x.h>
#include <ads1015.h>
#include <imu.h>
#include <ota.h>
#include <printf.h>
#include <stdio.h>
#include <rotors.h>
#include <sysctl.h>
#include <task.h>
#include <uarths.h>
#include <uart.h>
#include <motor.h>
#include <control.h>
#include <led.h>
#include <motioncap.h>
#include <ov7740.h>
#include <sleep.h>

#include <math.h>


#define TASK_LOOP_FREQ 400
#define TASK_LOOP_TIME_US (1000000 / TASK_LOOP_FREQ)

void task_400hz(struct global_data_ty * p_global_data) {
    static uint64_t start_time;
    float dt = start_time == 0 ? 0.0025 : (sysctl_get_time_us() - start_time) / 1000000.0;
    start_time = sysctl_get_time_us();
    imu_read(dt);
    data_exchange_task();
    flight_status_control(&(p_global_data->body_ctrl), &(p_global_data->remote_ctrl), dt);

    angle_speed_ctrl(&(p_global_data->body_ctrl), &attitude_data, dt);
    motor_throttle_ctrl(&(p_global_data->body_ctrl), &(p_global_data->motor), &(p_global_data->remote_ctrl), dt);
    
}

void task_200hz(struct global_data_ty * p_global_data) {
    static uint64_t start_time;
    float dt = start_time == 0 ? 0.005 : (sysctl_get_time_us() - start_time) / 1000000.0;
    start_time = sysctl_get_time_us();

    angle_ctrl(&(p_global_data->body_ctrl), &attitude_data, &(p_global_data->remote_ctrl), dt);
    
    ota_task();

}

void task_100hz(struct global_data_ty * p_global_data) {
    static uint64_t start_time;
    float dt = start_time == 0 ? 0.01 : (sysctl_get_time_us() - start_time) / 1000000.0;
    start_time = sysctl_get_time_us();

    height_fusion_100(dt);

    opticalFlowTask(dt);    /*光流任务*/
    if(global_data.flags.use_motioncap_data)
    {
        speed_fusion_mc(dt);
    }
    
	motioncap_cradle(dt);
	commander_cradle(dt);
}

void task_50hz(struct global_data_ty * p_global_data) {
    static uint64_t start_time;
    float dt = start_time == 0 ? 0.02 : (sysctl_get_time_us() - start_time) / 1000000.0;
    start_time = sysctl_get_time_us();

    height_speed_ctrl(&(p_global_data->body_ctrl), &attitude_data, &(p_global_data->remote_ctrl), dt);
    xy_speed_ctrl(&(p_global_data->body_ctrl), &attitude_data, &(p_global_data->remote_ctrl), dt);
   
}

void task_20hz(struct global_data_ty * p_global_data) {
    static uint64_t start_time;
    float dt = start_time == 0 ? 0.05 : (sysctl_get_time_us() - start_time) / 1000000.0;
    start_time = sysctl_get_time_us();

    VL53L1X_Read();    /*激光测距任务*/
    if(global_data.flags.use_motioncap_data)
        height_fusion_20_mc(dt);
    else
        height_fusion_20_vl(dt);
    height_ctrl(&(p_global_data->body_ctrl), &attitude_data, &(p_global_data->remote_ctrl), dt);
    xy_position_ctrl(&(p_global_data->body_ctrl), &attitude_data, &(p_global_data->remote_ctrl), dt);

    key_calibration(0);
}

void task_10hz(struct global_data_ty * p_global_data) {
    static uint64_t start_time;
    float dt = start_time == 0 ? 0.1 : (sysctl_get_time_us() - start_time) / 1000000.0;
    start_time = sysctl_get_time_us();
    // xy_position_ctrl(&(p_global_data->body_ctrl), &attitude_data, &(p_global_data->remote_ctrl), dt);
    key_reboot();
    adc_task();
    // cam_task();

}

int task_loop(struct global_data_ty * global_data) {
    uint64_t loop_cnt = 0;
    uint64_t start_time = 0;
    

    while (1) {
        start_time = sysctl_get_time_us();
        loop_cnt++;


        
        if (loop_cnt % 1 == 0) task_400hz(global_data);
        if (loop_cnt % 2 == 0) task_200hz(global_data);
        if (loop_cnt % 4 == 0) task_100hz(global_data);
        if (loop_cnt % 8 == 0) task_50hz(global_data);
        if (loop_cnt % 20 == 0) task_20hz(global_data);
        if (loop_cnt % 40 == 0) task_10hz(global_data);

        // printw("Spend%fs\n\r", (sysctl_get_time_us() - start_time) / 1000000.0);
        // printf("Spend%fs\n", (sysctl_get_time_us()/* - start_time*/) / 1000000.0);

        while ((sysctl_get_time_us() - start_time) < TASK_LOOP_TIME_US)
          ;
    }

    return 0;
}
