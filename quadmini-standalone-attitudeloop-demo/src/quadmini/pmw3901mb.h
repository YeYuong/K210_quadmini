#ifndef PMW3901MB_H
#define PMW3901MB_H

#include "imu.h"

#define PMW_USE_SPI SPI_DEVICE_0
#define PMW_SPI_RATE 2000000

typedef struct opFlow_s 
{
	float pixSum[2];		/*累积像素*/
	float pixComp[2];		/*像素补偿*/
	float pixValid[2];		/*有效像素*/
	float pixValidLast[2];	/*上一次有效像素*/
	
	float deltaPos[2];		/*2帧之间的位移 单位cm*/
	float deltaVel[2];		/*速度 单位cm/s*/
	float posSum[2];		/*累积位移 单位cm*/
	float velLpf[2];		/*速度低通 单位cm/s*/
	
	uint8_t isOpFlowOk;		/*光流状态*/
	uint8_t isDataValid;	/*数据有效*/
    uint8_t OpFlowExist;    /*光流芯片通信正常*/

} opFlow_t;

typedef enum {
    OP_X = 0,
    OP_Y
} op_axis_enum;

extern opFlow_t opFlow;

void opticalFlowTask(float dt);
uint8_t getOpFlowData(struct data_fusion_ty * attitude_data_p, float dt);	//读取光流数据
void opticalFlow_Init(void);		/*初始化光流模块*/
uint8_t getOpDataState(void);		/*光流数据状态*/
void OpFlowDataClear(void);
float getOpFlowSpeed(op_axis_enum axis);
float getOpFlowPosition(op_axis_enum axis);

#endif
