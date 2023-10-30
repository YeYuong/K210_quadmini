#include <i2c.h>
#include <imu.h>
#include <sleep.h>
#include <math.h>
#include <task.h>
#include <control.h>
#include "vl53l1x.h"
#include "VL53L1X_api.h"
#define ob_1  0.2 //融合滤波系数
#define ob_2  0.35
#define ob_3  0.3

static uint8_t isInitvl53l1x = 0;

// static uint8_t count = 0;
// static uint8_t validCnt = 0;
// static uint8_t inValidCnt = 0;

uint16_t tof_distance;
float quality = 1.0f;
float tof_height;

float vl_height;
float vl_height_speed;

VL53L1_Dev_t tof_dev;

#define LIMIT_R(x,min,max) 	((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x)))

void VL53L1X_Init(void)
{
	i2c_init(I2C_DEVICE_0, VL53L1X_ADDR, 7, VL53L1X_I2C_RATE);
	msleep(10);
	
	/*vl53l1x 初始化*/

    tof_dev.i2c_num = I2C_DEVICE_0;
    tof_dev.I2cDevAddr = VL53L1X_ADDR;
    uint16_t id = 0;
    
    if(VL53L1X_GetSensorId(tof_dev, &id) || id != VL53L1X_ID)
	{
        global_data.flags.ready_to_takeoff = global_data.flags.ready_to_takeoff>0?0:global_data.flags.ready_to_takeoff;
		printf("VL53L1X not found\n");
		return;
	}
    else
    {
        printf("VL53L1X init success\n");
    }
    
	isInitvl53l1x = 1;
	VL53L1X_SensorInit(tof_dev);
	VL53L1X_SetDistanceMode(tof_dev, 2);
	VL53L1X_SetTimingBudgetInMs(tof_dev, 50);
	VL53L1X_SetInterMeasurementInMs(tof_dev, 50);
	VL53L1X_StartRanging(tof_dev);
}

void VL53L1X_Read(void)
{
    int ret = VL53L1X_GetDistance(tof_dev, &tof_distance);
    if(ret < 0)
    {
        global_data.flags.ready_to_takeoff = 0;
    }else{
        global_data.flags.ready_to_takeoff &= 1;
    }
	tof_height = tof_distance * 0.001f;
	// printf("ret%d tof:%f\n", ret, tof_height);
}


void height_fusion_20_vl(float dt)
{
	float new_height_VL, err1, err2;
	static float old_height_VL=0;
	vl_height = tof_height * get_gravity_b().z;
	new_height_VL=vl_height;
	vl_height_speed=(new_height_VL-old_height_VL)/dt;

	err1 = (new_height_VL-ob_height);
	err2 = (vl_height_speed - ob_height_speed);
	
	ob_height       += ob_1 * err1;
	ob_height_speed += ob_2 * (err1 + err2);
	height_acc_zero = -LIMIT_R(global_data.body_ctrl.thr - 45, 0, 20)*0.13/20.0 + ob_3 * (err1 + err2);
	old_height_VL=new_height_VL;
}

float getTOFHeight(void) //m
{
	return tof_height;
}

float getObHeight(void) //m
{
	return ob_height;
}

float getObHeightSpeed(void) //m
{
	return ob_height_speed;
}

// void vl53l1x_Task(void)
// {
// 	int status;
// 	uint8_t isDataReady = 0;
// 	status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
				
// 	if(isDataReady)
// 	{
// 		status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
// 		if(status==0)
// 		{
// 			range_last = rangingData.RangeMilliMeter * 0.1f;	/*单位cm*/				
// 		}
// 		status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
// 	}	
	
// 	if(range_last < VL53L1X_MAX_RANGE)			
// 		validCnt++;			
// 	else 			
// 		inValidCnt++;			
	
// 	if(inValidCnt + validCnt == 10)
// 	{
// 		quality += (validCnt/10.f - quality) * 0.1f;	/*低通*/
// 		validCnt = 0;
// 		inValidCnt = 0;
// 	}
	
// }

// uint8_t vl53lxxReadRange(zRange_t* zrange)
// {
// 	zrange->quality = quality;		//可信度
// 	vl53lxx.quality = quality;
	
// 	if (range_last != 0 && range_last < VL53L1X_MAX_RANGE) 
// 	{			
// 		zrange->distance = (float)range_last;	//单位[cm]	
// 		vl53lxx.distance = 	zrange->distance;
// 		return 1;
// 	}

// 	return 0;
// }


