#include "3508_Drive.h"
#include "pid.h"
#include "can_receive.h"
#include "arm_math.h"
#include "NI_MING.h"
Leg_Move_t Leg_Move;
void Leg_Moto_Data_Init(void)
{
	
	const static fp32 Moto_Angle_Pid[3] = {10, 0, 100};
	const static fp32 Moto_Speed_Pid[3] = {80, 0, 120};
	for(int i=0;i<8;i++)
	{
		PID_Init(&Leg_Move.Moto_Angle_Pid[i],PID_POSITION,Moto_Angle_Pid,500,0);
		PID_Init(&Leg_Move.Moto_Speed_Pid[i],PID_POSITION,Moto_Speed_Pid,16000,0);
	}
}
void Leg_Moto_Data_Update(fp32 Angle_Gain[3],fp32 Speed_Gain[3])
{
	for(int i=0;i<8;i++)
	{
		PID_Chang_Params(&Leg_Move.Moto_Angle_Pid[i],Angle_Gain);
		PID_Chang_Params(&Leg_Move.Moto_Speed_Pid[i],Speed_Gain);
		Leg_Move.current_Angle[i]=get_Leg_moto_Measure_Point(i)->total_angle;
		Leg_Move.current_V[i]=get_Leg_moto_Measure_Point(i)->speed_rpm*PI/30.0f;
	}
}
uint8_t send_moto_flag;
void Leg_Control_Loop(fp32 Target_Angle_f[8])
{
	for(int i=0;i<8;i++)
	{
		Leg_Move.Target_Angle[i]=Target_Angle_f[i];
		PID_Calc(&Leg_Move.Moto_Angle_Pid[i],Leg_Move.current_Angle[i],Leg_Move.Target_Angle[i]);
		PID_Calc(&Leg_Move.Moto_Speed_Pid[i],Leg_Move.current_V[i],Leg_Move.Moto_Angle_Pid[i].out);
	}
		Send_3508_Id1_Id4(Leg_Move.Moto_Speed_Pid[0].out,Leg_Move.Moto_Speed_Pid[1].out,Leg_Move.Moto_Speed_Pid[2].out,Leg_Move.Moto_Speed_Pid[3].out);
		Send_3508_Id5_Id8(Leg_Move.Moto_Speed_Pid[4].out,Leg_Move.Moto_Speed_Pid[5].out,Leg_Move.Moto_Speed_Pid[6].out,Leg_Move.Moto_Speed_Pid[7].out);

//		serial_plot(4,Leg_Move.Moto_Angle_Pid[0].set,Leg_Move.Moto_Angle_Pid[0].fdb,\
		get_Leg_moto_Measure_Point(0)->given_current*6.0f/16384,get_Leg_moto_Measure_Point(1)->given_current*6.0f/16384);
}
