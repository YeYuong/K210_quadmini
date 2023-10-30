#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdio.h>
#include <stdint.h>

#define VL53L1X_MAX_RANGE			410		//410cm
#define VL53L1X_ADDR 				0x29
#define VL53L1X_I2C_RATE 			400000
#define VL53L1X_ID			0xEACC

extern uint16_t vl53lxxId;	/*vl53оƬID*/
extern uint8_t isEnableVl53lxx;
extern float tof_height;

extern float vl_height;           //VL参考高度
extern float vl_height_speed;     //VL参考速度

void VL53L1X_Init(void);
void VL53L1X_Read(void);

float getTOFHeight(void);
float getObHeight(void);
float getObHeightSpeed(void);
void height_fusion_20_vl(float dt);

#endif
