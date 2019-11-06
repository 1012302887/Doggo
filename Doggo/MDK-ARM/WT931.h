#ifndef __WT931_H__
#define __WT931_H__
#include "main.h"
typedef struct Imu_Data_s{
	float a_x;
	float a_y;
	float a_z;
	float W_x;
	float W_y;
	float W_z;
	float Roll;
	float Pitch;
	float Yaw;
	float Temperature;
}Imu_Data_s;
void Imu_Data_Analysis(uint8_t num);
void Imu_Data_Calc();
extern Imu_Data_s Imu_Data;
#endif
