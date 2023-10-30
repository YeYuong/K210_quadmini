#include "pmw3901mb.h"
#include "vl53l1x.h"
#include "imu.h"
#include "io_func.h"
#include <stdio.h>
#include <stdlib.h>
#include <sleep.h>
#include <gpiohs.h>
#include <spi.h>
#include <spi_sw.h>
#include <sysctl.h>
#include <math.h>
#include <task.h>


#define DEG2RAD		0.017453293f
#define RAD2DEG		57.29578f

#define PMW_NCS_SET(val)	gpiohs_set_pin(PMW_NCS_GPIOHS, val)

#define RESOLUTION			(0.17643691f)/*1m高度下 1个像素对应的位移，单位cm*/
#define OULIER_LIMIT 		(100)		/*光流像素输出限幅*/
#define VEL_LIMIT			(150.f)		/*光流速度限幅*/

#define VEL_LPF_FILTER			/*低通滤波*/
//#define AVERAGE_FILTER		/*均值滤波*/

#define LIMIT(val, min, max) (val)<(min)?(min):(val)>(max)?(max):(val)

static uint8_t outlierCount = 0;			/*数据不可用计数*/

opFlow_t opFlow;	/*光流*/

// TaskHandle_t opFlowTaskHandle = NULL;

#define RGB_DO(pv) gpiohs_set_pin(RGB_GPIO_PIN, pv)
#define RGB_DO_0 *(uint32_t*)(0x38001000u + 0x0cu) &= ~(0x1u<<RGB_GPIO_PIN)
#define RGB_DO_1 *(uint32_t*)(0x38001000u + 0x0cu) |=  (0x1u<<RGB_GPIO_PIN)

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

typedef struct motionBurst_s 
{
	union 
	{
		uint8_t motion;
		struct 
		{
			uint8_t frameFrom0    : 1;
			uint8_t runMode       : 2;
			uint8_t reserved1     : 1;
			uint8_t rawFrom0      : 1;
			uint8_t reserved2     : 2;
			uint8_t motionOccured : 1;
		};
	};

	uint8_t observation;
	int16_t deltaX;
	int16_t deltaY;

	uint8_t squal;

	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
} motionBurst_t;

motionBurst_t currentMotion;

static void InitRegisters(void);

static void registerWrite(uint8_t reg, uint8_t value)
{
	// 最高位为1 写寄存器
	reg |= 0x80u;
	
	spi_set_clk_rate(PMW_USE_SPI, PMW_SPI_RATE);

	PMW_NCS_SET(0);
	
	usleep(50);
#ifndef USE_SOFTWARE_SPI
	spi_send_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, &reg, 1, &value, 1);
#else
    spi_sw_send_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, &reg, 1, &value, 1);
#endif
	usleep(50);

	PMW_NCS_SET(1);
	
	usleep(200);
}

static uint8_t registerRead(uint8_t reg)
{
	uint8_t data = 0;

	// 最高位为0 读寄存器
	reg &= ~0x80u;

	spi_set_clk_rate(PMW_USE_SPI, PMW_SPI_RATE);

	PMW_NCS_SET(0);
	
	usleep(50);
#ifndef USE_SOFTWARE_SPI
	spi_send_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, &reg, 1, NULL, 0);
	usleep(50);
	spi_receive_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, NULL, 0, &data, 1);	
#else
	spi_sw_send_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, &reg, 1, NULL, 0);
	usleep(50);
	spi_sw_receive_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, NULL, 0, &data, 1);	
#endif
	usleep(50);
	
	PMW_NCS_SET(1);
	
	// usleep(200);

	return data;
}

static void readMotion(motionBurst_t * motion)
{
	uint8_t address = 0x16;
	
	spi_set_clk_rate(PMW_USE_SPI, PMW_SPI_RATE);

	PMW_NCS_SET(0);
	
	usleep(50);  
#ifndef USE_SOFTWARE_SPI
	spi_send_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, &address, 1, NULL, 0);
	usleep(50);
	spi_receive_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, NULL, 0, (uint8_t*)motion, sizeof(motionBurst_t));
#else
	spi_sw_send_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, &address, 1, NULL, 0);
	usleep(50);
	spi_sw_receive_data_standard(PMW_USE_SPI, SPI_CHIP_SELECT_2, NULL, 0, (uint8_t*)motion, sizeof(motionBurst_t));
#endif
	usleep(50);
	
	PMW_NCS_SET(1);

	uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
	realShutter |= (motion->shutter & 0x0ff) << 8;
	motion->shutter = realShutter;
}

static void InitRegisters(void)
{	
	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5B, 0x32);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x65, 0x20);
	registerWrite(0x6A, 0x18);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x5F, 0x40);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x1F);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xFF);
	registerWrite(0x65, 0x1F);
	registerWrite(0x7F, 0x14);
	registerWrite(0x65, 0x67);
	registerWrite(0x66, 0x08);
	registerWrite(0x63, 0x70);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x02);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);
	registerWrite(0x70, 0x00);

	msleep(10); // delay 10ms

	registerWrite(0x32, 0x44);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x62, 0xF0);
	registerWrite(0x63, 0x00);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	
//	/*初始化LED_N*/
//	registerWrite(0x7F, 0x0E);
//	registerWrite(0x72, 0x0F);
//	registerWrite(0x7F, 0x00);
}
/*复位光流数据*/
static void resetOpFlowData(void)
{
	for(uint8_t i=0; i<2; i++)
	{
		opFlow.pixSum[i] = 0;
		opFlow.pixComp[i] = 0;
		opFlow.pixValid[i] = 0;
		opFlow.pixValidLast[i] = 0;
	}
}

void OpFlowDataClear(void)
{
	for(uint8_t i=0; i<2; i++)
	{
		opFlow.pixSum[i] = 0;
		opFlow.pixComp[i] = 0;
		opFlow.pixValid[i] = 0;
		opFlow.pixValidLast[i] = 0;
		opFlow.velLpf[i] = 0;
		opFlow.posSum[i] = 0;
	}
}

/*光流任务函数*/
void opticalFlowTask(float dt)
{	
	static uint16_t count = 0;
    if(opFlow.OpFlowExist == 0){
        return;
    }
	opFlow.isOpFlowOk = 1;
	
	readMotion(&currentMotion);

	if(currentMotion.minRawData == 0 && currentMotion.maxRawData == 0)
	{
		if(count++ > 100 && opFlow.isOpFlowOk == 1)
		{
			count = 0;
			opFlow.isOpFlowOk = 0;		/*光流出错*/
			return;
		}		
	}
	else { count = 0; }

	/*连续2帧之间的像素变化，根据实际安装方向调整 (pitch:x)  (roll:y)*/
	int16_t pixelDx = currentMotion.deltaY;
	int16_t pixelDy = currentMotion.deltaX;

	if (abs(pixelDx) < OULIER_LIMIT && abs(pixelDy) < OULIER_LIMIT) 
	{
		opFlow.pixSum[X] += -pixelDx;
		opFlow.pixSum[Y] += -pixelDy;
	}else 
	{
		outlierCount++;
	}

	// printf("opFlow:%d, %d\n", currentMotion.deltaX, currentMotion.deltaY);

	getOpFlowData(&attitude_data,dt);

	//printf("opFlow:%f, %f\n", opFlow.posSum[X], opFlow.posSum[Y]);
}

float getOpFlowSpeed(op_axis_enum axis) //m
{
	return 0.01 * opFlow.velLpf[axis];
}

float getOpFlowPosition(op_axis_enum axis) //m
{
	return 0.01 * opFlow.posSum[axis];
}


#ifdef AVERAGE_FILTER

#define GROUP		2
#define FILTER_NUM	3

/*限幅均值滤波法*/
void velFilter(float* in, float* out)
{	
	static uint8_t i=0;
	static float filter_buf[GROUP][FILTER_NUM] = {0.0};
	double filter_sum[GROUP] = {0.0};		
	
	for(uint8_t j=0;j<GROUP;j++)
	{
		if(filter_buf[j][i] == 0.0f)
		{
			filter_buf[j][i] = in[j];
			out[j] = in[j];			
		} else 
		{			
			if(fabs(in[j]) < VEL_LIMIT)
			{
				filter_buf[j][i] = in[j];
			}
			for(uint8_t cnt=0;cnt<FILTER_NUM;cnt++)
			{
				filter_sum[j] += filter_buf[j][cnt];
			}
			out[j]=filter_sum[j] /FILTER_NUM;
		}
	}
	if(++i >= FILTER_NUM)	i = 0;
}
#endif

uint8_t getOpFlowData(struct data_fusion_ty * attitude_data_p, float dt) //计算光流信息，保存到结构体
{
	static uint8_t cnt = 0;
	float height = getObHeight();/*读取高度信息 单位m*/
	height = height > 0.035 ? height : 0.0;//高度低于3.5cm无效
	
	if(opFlow.isOpFlowOk && height<4.0f)	/*4m范围内，光流可用*/
	{
		cnt= 0;
		opFlow.isDataValid = 1;
		
		float tanRoll = -tan(attitude_data_p->heading_angle.roll * DEG2RAD);
		float tanPitch = tan(attitude_data_p->heading_angle.pitch * DEG2RAD);
		
		opFlow.pixComp[X] += (480.f * tanPitch - opFlow.pixComp[X]) * 0.2;	/*像素补偿，负方向*/
		opFlow.pixComp[Y] += (480.f * tanRoll - opFlow.pixComp[Y]) * 0.2;
		opFlow.pixValid[X] = (opFlow.pixSum[X] + opFlow.pixComp[X]);	/*实际输出像素*/
		opFlow.pixValid[Y] = (opFlow.pixSum[Y] + opFlow.pixComp[Y]);		
		float velComp[2] = {0.0,0.0};
		// velComp[X] = -3.8 * attitude_data_p->imu.gyro.y;
		// velComp[Y] = -3.8 * attitude_data_p->imu.gyro.x;
		opFlow.deltaPos[X] = RESOLUTION * height * (opFlow.pixValid[X] - opFlow.pixValidLast[X] + velComp[X]);	/*2帧之间位移变化量，单位cm*/
		opFlow.deltaPos[Y] = RESOLUTION * height * (opFlow.pixValid[Y] - opFlow.pixValidLast[Y] + velComp[Y]);	
		opFlow.pixValidLast[X] = opFlow.pixValid[X];	/*上一次实际输出像素*/
		opFlow.pixValidLast[Y] = opFlow.pixValid[Y];
		opFlow.deltaVel[X] = opFlow.deltaPos[X] / dt;	/*速度 cm/s*/
		opFlow.deltaVel[Y] = opFlow.deltaPos[Y] / dt;
		
#ifdef AVERAGE_FILTER
		velFilter(opFlow.deltaVel, opFlow.velLpf);		/*限幅均值滤波法*/
#else
		opFlow.velLpf[X] += (opFlow.deltaVel[X] - opFlow.velLpf[X]) * 0.15f;	/*速度低通 cm/s*/
		opFlow.velLpf[Y] += (opFlow.deltaVel[Y] - opFlow.velLpf[Y]) * 0.15f;	/*速度低通 cm/s*/
#endif			
		opFlow.velLpf[X] = LIMIT(opFlow.velLpf[X], -VEL_LIMIT, VEL_LIMIT);	/*速度限幅 cm/s*/
		opFlow.velLpf[Y] = LIMIT(opFlow.velLpf[Y], -VEL_LIMIT, VEL_LIMIT);	/*速度限幅 cm/s*/
	
		opFlow.posSum[X] += opFlow.deltaPos[X];	/*累积位移 cm*/
		opFlow.posSum[Y] += opFlow.deltaPos[Y];	/*累积位移 cm*/
	}
	else if(opFlow.isDataValid == 1)
	{
		if(cnt++ > 100)	/*超过定点高度，切换为定高模式*/
		{
			cnt = 0;
			opFlow.isDataValid = 0;
		}	
		resetOpFlowData();	
	}
	
	return opFlow.isOpFlowOk;	/*返回光流状态*/
}
/*初始化光流模块
  光流使用软件片选，必须首先初始化，否则会影响其它spi器件通信*/
void opticalFlow_Init(void)
{
    gpiohs_set_drive_mode(PMW_NCS_GPIOHS, GPIO_DM_OUTPUT);
    resetOpFlowData();
    opFlow.isOpFlowOk = 1;
	
	msleep(50);
	
	PMW_NCS_SET(1);
    
#ifndef USE_SOFTWARE_SPI
	spi_init(PMW_USE_SPI, SPI_WORK_MODE_0, SPI_FF_STANDARD, 8, 0);  // SPI0
	spi_set_clk_rate(PMW_USE_SPI, PMW_SPI_RATE);
#else
    spi_sw_init();  //software SPI
#endif
	msleep(40);

	uint8_t chipId = registerRead(0);
	uint8_t invChipId = registerRead(0x5f);
	// printf("Motion chip is: 0x%x\n", chipId);
	// printf("si pihc noitoM: 0x%x\n", invChipId);
    if(chipId == 0x49 && invChipId == 0xb6)
    {
        printf("PMW3901 init success\n");
        opFlow.OpFlowExist = 1;
    }
    else
    {
        printf("PMW3901 not found\n");
        opFlow.OpFlowExist = 0;
        global_data.flags.ready_to_takeoff = global_data.flags.ready_to_takeoff>0?0:global_data.flags.ready_to_takeoff;
    }

	// 上电复位
	registerWrite(0x3a, 0x5a);
	msleep(5);

	InitRegisters();
	msleep(5);

	//vl53lxxInit();	/*初始化vl53lxx*/
}

/*获取光流数据状态*/
uint8_t getOpDataState(void)
{
	return opFlow.isDataValid;
}
