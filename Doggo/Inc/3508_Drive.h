#ifndef _3508_Drive_H
#define _3508_Drive_H
#include "pid.h"
typedef struct
{
	PidTypeDef Moto_Angle_Pid[8];//id_1-8
	PidTypeDef Moto_Speed_Pid[8];
	fp32 Target_Angle[8];
	fp32 current_Angle[8];
	fp32 Target_V[8];
	fp32 current_V[8];
}Leg_Move_t;
void Leg_Moto_Data_Update(fp32 Angle_Gain[3],fp32 Speed_Gain[3]);
void Leg_Moto_Data_Init(void);
void Leg_Control_Loop(fp32 Target_Angle_f[8]);
extern Leg_Move_t Leg_Move;
#endif
