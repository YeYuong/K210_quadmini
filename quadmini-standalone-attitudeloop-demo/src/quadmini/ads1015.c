/*
ADS驱动
yyl
*/
#include <stdio.h>
#include <ads1015.h>
#include <i2c.h>
#include <sleep.h>

uint16_t ADS1015_CONFIG;//定义ADS1015配置变量
uint8_t ads_exist = 0;

uint16_t ADS1015_ReadOneByte(uint8_t i2cAddress,uint16_t ReadAddr)
{
    uint16_t temp=0;
    int8_t  buf[2]={0, 0};

    i2c_recv_data(I2C_DEVICE_0, i2cAddress, &ReadAddr, 1, buf, 2);
	// printf("1:%x\n",buf[0]);
	// printf("2:%x\n",buf[1]);
    temp=(buf[0]<<8) + buf[1];
    return temp;
}

void ADS1015_WriteOneByte(uint8_t i2cAddress,uint8_t WriteAddr,uint16_t DataToWrite)
{
    uint8_t buf[3]={0, 0, 0};
	int temp;
	// buf[0]=WriteAddr>>8;
	buf[0]=WriteAddr;
	// printf("buf0:%x\n",buf[0]);
	buf[1]=DataToWrite>>8;
	// printf("buf0:%x\n",buf[1]);
	buf[2]=DataToWrite&0xff;
	// printf("buf0:%x\n",buf[2]);
	temp = i2c_send_data(I2C_DEVICE_0, i2cAddress, buf, 3);
	// printf("写是否成功：%d\n",temp);
}
	
//ADS1015配置函数
//channel:模数转换通道
void ADS1015_Config(uint8_t i2cAddress,uint8_t channel)
{
  // Start with default values
  ADS1015_CONFIG  = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_3300SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
	
  // Set PGA/voltage range
  ADS1015_CONFIG |= ADS1015_REG_CONFIG_PGA_6_144V ;
  switch (channel)
  {
    case (0):
      ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }
  // Set 'start single-conversion' bit
  ADS1015_CONFIG |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  ADS1015_WriteOneByte(i2cAddress, ADS1015_REG_POINTER_CONFIG, ADS1015_CONFIG);
}

//ADS1015初始化函数
void ADS1015_Init()
{
	uint8_t id=0;
	uint16_t ADS_ID_ADDRESS=0x01;

	// IIC_Init();
    i2c_init(I2C_DEVICE_0, ADS1015_ADDRESS, 7, ADS1015_I2C_RATE);
	i2c_recv_data(I2C_DEVICE_0, ADS1015_ADDRESS, &(ADS_ID_ADDRESS), 1, &id, 1);
	if(id > 0)
		ads_exist = 1;
	else
		ads_exist = 0;
	printf("ADS1015 init success\n");
}

//ADS1015读数据
//返回值：获取的AD值
//Channel：通道数，范围0-3
uint16_t ADS1015_Read(uint8_t channel)
{
	int temp;
	 // Start with default values
	  ADS1015_CONFIG  = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
						ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
						ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
						ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
						ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
						ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
		
	  // Set PGA/voltage range
	 ADS1015_CONFIG |= ADS1015_REG_CONFIG_PGA_6_144V;
	if(channel<=3) 
	{
		switch (channel)//采用单端模式,AD值范围为0~2048
		{
			case (0):
				ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
			break;
			case (1):
				ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
			break;
			case (2):
				ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
			break;
			case (3):
				ADS1015_CONFIG |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
			break;
		}	
		  // Set 'start single-conversion' bit
		ADS1015_CONFIG |= ADS1015_REG_CONFIG_OS_SINGLE;
		// printf("%x\n",ADS1015_CONFIG);
		ADS1015_WriteOneByte(ADS1015_ADDRESS, ADS1015_REG_POINTER_CONFIG, ADS1015_CONFIG);
		
		msleep(1);
		//delay_us(500);
		// delay_ms(1);
		temp=ADS1015_ReadOneByte(ADS1015_ADDRESS, ADS1015_REG_POINTER_CONVERT);
		// printf("返回AD值:%x\n",temp);
		temp=temp>>4;
		//最终返回AD值
		// printf("返回AD值:%d\n",temp);
		//目前选取了电压量程6.144v,将AD值转化为电压
		// raw_voltage = (float)temp / 2048.0 * 6.144 * 2;
		// printf("%f\n",raw_voltage);

		return temp;
	}
	return 0;
}

#include <task.h>
#include <led.h>
#define VOLT_CHANNEL 0
#define CURR_CHANNEL 2
#define ALLERT_VOLT 3.6
#define LAND_VOLT	3.2
float batt_voltage = 4.0;
float motor_current = 0.0;

float read_voltage(void)
{
    if(ads_exist)
    {
        uint16_t AD_value = ADS1015_Read(VOLT_CHANNEL);
        //目前选取了电压量程6.144v,将AD值转化为电压
        return (float)AD_value / 2048.0 * 6.144 * 2;
    }
    else
    {
        return 4.2;
    }
}

float read_current(void)
{
	uint16_t AD_value = ADS1015_Read(CURR_CHANNEL);
	//目前选取了电压量程6.144v,将AD值转化为电压量程
	//使用的是20放大倍数，15mΩ
	return ((float)AD_value / 2048.0 * 6.1404) / (20 * 0.015);
}

void adc_task(void)
{
	static uint16_t cnt = 0;
	if(ads_exist)
	{
		batt_voltage += 0.6 * (read_voltage() - batt_voltage);
		// read_current();
		global_data.batt_voltage = batt_voltage;
		if(global_data.body_ctrl.flight_status > FLIGHT_UNLOCKED)
		{
			if(batt_voltage < LAND_VOLT)
			{
				cnt++;
				if(cnt >= 10)
				{
					rgb_set(40, 0, 0);
					// global_data.flags.use_commander = 0;
					// global_data.body_ctrl.flight_status = FLIGHT_LANDING;
				}
			} else { cnt = 0; }
		}
		else
		{
			if(batt_voltage < ALLERT_VOLT)
			{
				cnt++;
				if(cnt >= 10)
				{
					cnt = 0;
					lock_motor();
					// global_data.body_ctrl.flight_status = FLIGHT_LOCKED;
					rgb_set(40, 0, 0);
				}
			} else { cnt = 0; }
		}
	}
}


//检测AD1015是否存在
//参数：
//i2cAddress：芯片地址
//返回值：0存在 1不存在
// uint8_t AD1015_Check(uint8_t i2cAddress)
// {
// 	uint8_t x=22;
// 	IIC_Start();  
// 	IIC_Send_Byte(i2cAddress<<1);  
// 	x=IIC_Wait_Ack();
// 	return x;
// }


