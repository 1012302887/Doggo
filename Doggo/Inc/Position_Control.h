#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H
#include "main.h"
void PositionControlThread(void *pvParameters);
typedef enum States {
	STOP = 0,
	TROT = 1,
	BOUND = 2,
	WALK = 3,
	PRONK = 4,
	JUMP = 5,
	DANCE = 6,
	HOP = 7,
	TEST = 8,
	ROTATE = 9,
	FLIP = 10,
	TURN_TROT = 11,
	RESTART = 12,
	INIT=13,
	neiba=14
}States_e;
typedef struct 
{
	float stance_height; // Desired height of body from ground during walking (m)
	float down_amp; // Peak amplitude below stanceHeight in sinusoidal trajectory (m)
	float up_amp; // Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
	float flight_percent; // Portion of the gait time should be doing the down portion of trajectory
	float step_length; // Length of entire step (m)
	float freq; // Frequency of one gait cycle (Hz)
	float step_diff; //difference between left and right leg step length
	float theta;
	float gamma;
	float x;
	float y;
	float L;
}GaitParams_t;
extern void SinTrajectory (float t, GaitParams_t *params, float gaitOffset);
extern void CartesianToThetaGamma(GaitParams_t *params,float leg_direction);
extern void gait(GaitParams_t *params,float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,\
	int8_t leg0_direction, int8_t leg1_direction,int8_t leg2_direction, int8_t leg3_direction);
extern void CartesianToLegParams(float x, float y, float leg_direction, float *L, float* theta);
extern void Theta_Gamma_To_moto_angle(GaitParams_t *params,uint8_t leg_num);
extern void hop(GaitParams_t *params);
extern void Send_To_Moto(float Ref_Angle_1,float Ref_Angle_2,float Ref_Angle_3,float Ref_Angle_4,float Ref_Angle_5,float Ref_Angle_6,float Ref_Angle_7,float Ref_Angle_8);
extern void CommandAllLegs(float theta,float gamma);
extern uint8_t IsValidGaitParams(GaitParams_t *params);
extern States_e state;extern States_e last_state;
extern fp32 Angle_Gain[3];extern fp32 Speed_Gain[3];
extern GaitParams_t gait_params;
void State_Trans(void);
#endif
