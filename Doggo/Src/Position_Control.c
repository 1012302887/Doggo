#include "math.h"
#include "can_receive.h"
#include "Position_Control.h"
#include "jump.h"
#include  "REST.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "NI_MING.h"
#include "FreeRTOS.h"
#include "task.h"
#include "3508_drive.h"
#include "remote_control.h"
#include "detect_task.h"
#include "user_lib.h"
#include "ADC1.h"
#include "WT931.h"
#include "Hop.h"
#include "jacobian.h"
#include "dwt_stm32_delay.h"
extern ramp_t state_change_ramp;
#define L1 0.11f // upper leg length (m)
#define L2 0.19f // lower leg length (m)
//void MOTO_ANGLE_TO_X_Y(float theta1,float theta2);
fp32 Angle_Gain[3]={100,0,100};fp32 Speed_Gain[3]={200,0,200};
float leg_0_theta_1,leg_0_theta_2,leg_1_theta_1,leg_1_theta_2,leg_2_theta_1,leg_2_theta_2,leg_3_theta_1,leg_3_theta_2;
States_e state;
States_e last_state;
GaitParams_t gait_params ;
// {stance_height, down_AMP, up_AMP, flight_percent (proportion), step_length, FREQ}
GaitParams_t state_gait_params[] = 
{
   //{s.h, d.a., u.a., f.p., s.l., fr., s.d.}
    {0.22, 0.0,   0.0, 0.35, 0.0, 1.0, 0.0}, // STOP
    {0.24, 0.026f, 0.08, 0.35, 0.08, 1.8f, 0.0}, // TROT
    {0.30, 0.04, 0.06, 0.35, 0.0, 2.0f, 0.0}, // BOUND
    {0.24, 0.026f, 0.07, 0.35f, 0.10, 1.5f, 0.0}, // WALK
    {0.20, 0.05, 0.0, 0.75, 0.0, 1.0, 0.0}, // PRONK
    {0, 0, 0, 0, 0, 0, 0}, // JUMP
    {0.25, 0.05, 0.05, 0.35, 0.0, 1.5, 0.0}, // DANCE
    {0.25, 0.04, 0.05, 0.2, 0.05, 1.0, 0.0}, // HOP
    {0, 0, 0, 0, 0, 1.0, 0}, // TEST
    {0.20, 0.02, 0.08, 0.35f, 0, 1.0f, 0.0}, // ROTATE
    {0.25, 0.07, 0.06, 0.2, 0.0, 1.0, 0.0}, // FLIP
    {0.25, 0.04, 0.06, 0.35, 0.1, 2.0, 0.06}, // TURN_TROT
    {0, 0, 0, 0, 0.13, 0, 0}, // RESET
		{0, 0, 0, 0, 0, 0, 0}, // INIT
		{0.22f, 0.025, 0.08f, 0.35f, 0.0, 1.5f, 0} // Stepping
};
extern void Trans_Jacobian(float Theta_1,float Theta_2,float current_1,float current_2);
void CoupledMoveLeg(uint8_t leg_num,float t, GaitParams_t *params,float gait_offset, float leg_direction) {
    SinTrajectory(t,params,gait_offset);
    CartesianToThetaGamma(params,leg_direction);//坐标转换角度
    Theta_Gamma_To_moto_angle(params,leg_num);//转换为水平与上关节角度
}
void static_CoupledMoveLeg(uint8_t leg_num,float t, GaitParams_t *params,float gait_offset, float leg_direction,float x, float y) {
		params->x=x;params->y=y;
    CartesianToThetaGamma(params,leg_direction);//坐标转换角度
    Theta_Gamma_To_moto_angle(params,leg_num);//转换为水平与上关节角度
}
void PositionControlThread(void *pvParameters)
{
	uint32_t time_add;
	last_state=state = INIT;
	gait_params = state_gait_params[state];
	Leg_Moto_Data_Init();
	Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
	for(;;)
	{
//		Trans_Jacobian(get_Leg_moto_Measure_Point(1)->total_angle,get_Leg_moto_Measure_Point(0)->total_angle,\
//		get_Leg_moto_Measure_Point(0)->given_current/1000.0f,get_Leg_moto_Measure_Point(1)->given_current/1000.0f);
		time_add++;
		if(time_add%10==0){
//				printf("--%d--",HAL_GetTick());
//			Ni_Ming(0xf1,get_Leg_moto_Measure_Point(6)->given_current/1000.0f,get_Leg_moto_Measure_Point(7)->given_current/1000.0f,\
//			get_Leg_moto_Measure_Point(0)->given_current/1000.0f,get_Leg_moto_Measure_Point(1)->given_current/1000.0f);
//				Ni_Ming(0xf1,Leg_Move.Moto_Angle_Pid[6].fdb,get_Leg_moto_Measure_Point(7)->round_cnt,\
//				Leg_Move.Moto_Angle_Pid[7].set,Leg_Move.Moto_Angle_Pid[7].fdb);
//			Ni_Ming(0xf1,Detect_Judeg(1),Detect_Judeg(2),0,0);
		}
		State_Trans();//状态转换
		if(last_state!=state)
		{
			gait_params = state_gait_params[state];//更新步态参数
//			ramp_init(&state_change_ramp,state_change_ramp_value);
		}
		last_state=state;
//		Angle_Gain[0]=P_1;Angle_Gain[2]=D_1;Speed_Gain[0]=P_2;Speed_Gain[2]=D_2;
	  switch(state) {
			case INIT:{
				NULL;
			}break;
			case STOP:{
				Angle_Gain[0]=350;Angle_Gain[1]=0.0f;Angle_Gain[2]=250;Speed_Gain[0]=450;Speed_Gain[1]=0 ;Speed_Gain[2]=300;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				gait(&gait_params, 0, 0, 0, 0,1,1,1,1);
			}break;
			case RESTART:{
				ExecuteRest();
			}break;
			case DANCE:{
//				gait(&gait_params, 0.0, 0.5, 0.0, 0.5);
			}break;
			case BOUND:{
//				gait(&gait_params, 0.0, 0.5, 0.5, 0.0);
			}break;
			case TROT:{
				Angle_Gain[0]=200;Angle_Gain[1]=0;Angle_Gain[2]=250;Speed_Gain[0]=500;Speed_Gain[1]=0;Speed_Gain[2]=300;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				gait(&gait_params, 0.0, 0.5, 0.5, 0,\
				sign_int16(get_remote_control_point()->rc.ch[4]),sign_int16(get_remote_control_point()->rc.ch[4]),\
				sign_int16(get_remote_control_point()->rc.ch[4]),sign_int16(get_remote_control_point()->rc.ch[4]));
			}break;
			case TURN_TROT:{
//				gait(&gait_params, 0.0, 0.5, 0.0, 0.5);
			}break;
			case WALK:{
				Angle_Gain[0]=260;Angle_Gain[2]=250;Speed_Gain[0]=400;Speed_Gain[2]=130;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				gait(&gait_params, 0.0, 0.5, 0.25, 0.75,\
				sign_int16(get_remote_control_point()->rc.ch[4]),sign_int16(get_remote_control_point()->rc.ch[4]),\
				sign_int16(get_remote_control_point()->rc.ch[4]),sign_int16(get_remote_control_point()->rc.ch[4]));
			}break;
			case PRONK:{
//				gait(&gait_params, 0.0, 0.0, 0.0, 0.0);
			}break;
			case JUMP:{
				ExecuteJump();
			}break;
			case ROTATE:{
				Angle_Gain[0]=120;Angle_Gain[2]=100;Speed_Gain[0]=220;Speed_Gain[2]=130;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				if(get_remote_control_point()->rc.ch[0]>0){//右转
				gait(&gait_params, 0.75, 0.5, 0.0, 0.25,\
					-sign_int16(get_remote_control_point()->rc.ch[0]),-sign_int16(get_remote_control_point()->rc.ch[0]),\
					sign_int16(get_remote_control_point()->rc.ch[0]),sign_int16(get_remote_control_point()->rc.ch[0]));
				}
				else{//左转
				gait(&gait_params, 0.0, 0.25, 0.75, 0.5,\
				-sign_int16(get_remote_control_point()->rc.ch[0]),-sign_int16(get_remote_control_point()->rc.ch[0]),\
				sign_int16(get_remote_control_point()->rc.ch[0]),sign_int16(get_remote_control_point()->rc.ch[0]));
				}
			}break;
			case HOP:{
				hop(&gait_params);
			}break;
			case Stepping:{
				Angle_Gain[0]=150;Angle_Gain[1]=0;Angle_Gain[2]=200;Speed_Gain[0]=250;Speed_Gain[1]=0;Speed_Gain[2]=100;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				gait(&gait_params, 0.0, 0.5, 0.5, 0,1,1,1,1);
			}break;
		}
		vTaskDelay(2);
	}
}
void gait(GaitParams_t *params,float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,\
	int8_t leg0_direction, int8_t leg1_direction,int8_t leg2_direction, int8_t leg3_direction) { 
		double t; 
		GaitParams_t paramsR = *params;GaitParams_t paramsL = *params;
		GaitParams_t params1 = *params;GaitParams_t params2 = *params;GaitParams_t params3 = *params;GaitParams_t params4 = *params;
    paramsR.step_length -= params->step_diff;
    paramsL.step_length += params->step_diff;

    if (!IsValidGaitParams(&paramsR)||!IsValidGaitParams(&paramsL)) 
		{
        return;
    }
		
    t = xTaskGetTickCount()/1000.0f;
//    const float leg0_direction = -1.0;//方向
		params1.stance_height+=0.010f;
    CoupledMoveLeg(0,t, &params1, leg0_offset, leg0_direction);
//    const float leg1_direction = -1.0;
    CoupledMoveLeg(1,t, &params2, leg1_offset, leg1_direction);
//    const float leg2_direction = -1.0;
		params3.stance_height+=0.010f;
    CoupledMoveLeg(2,t, &params3, leg2_offset, leg2_direction);
//    const float leg3_direction = -1.0;
		params4.stance_height+=0.010f;
    CoupledMoveLeg(3,t, &params4, leg3_offset, leg3_direction);
		
		 Send_To_Moto(-(PI/2.0f-leg_0_theta_1),-(PI/2.0f-leg_0_theta_2),(PI/2.0f-leg_1_theta_1),(PI/2.0f-leg_1_theta_2),\
		(PI/2.0f-leg_2_theta_1),(PI/2.0f-leg_2_theta_2),-(PI/2.0f-leg_3_theta_1),-(PI/2.0f-leg_3_theta_2));	
}
void static_gait(GaitParams_t *params,float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,\
	int8_t leg0_direction, int8_t leg1_direction,int8_t leg2_direction, int8_t leg3_direction,float *x,float *y) { 
		double t; 
//		GaitParams_t paramsR = *params;GaitParams_t paramsL = *params;
		GaitParams_t params1 = *params;GaitParams_t params2 = *params;GaitParams_t params3 = *params;GaitParams_t params4 = *params;

//    if (!IsValidGaitParams(&paramsR)||!IsValidGaitParams(&paramsL)) 
//		{
//        return;
//    }
		
		y[0]+=0.010f;;
    static_CoupledMoveLeg(0,t, &params1, leg0_offset, leg0_direction,x[0],y[0]);
    static_CoupledMoveLeg(1,t, &params2, leg1_offset, leg1_direction,x[1],y[1]);
    static_CoupledMoveLeg(2,t, &params3, leg2_offset, leg2_direction,x[2],y[2]);
//		params4.stance_height+=0.010f;
    static_CoupledMoveLeg(3,t, &params4, leg3_offset, leg3_direction,x[3],y[3]);
		
		 Send_To_Moto(-(PI/2.0f-leg_0_theta_1),-(PI/2.0f-leg_0_theta_2),(PI/2.0f-leg_1_theta_1),(PI/2.0f-leg_1_theta_2),\
		(PI/2.0f-leg_2_theta_1),(PI/2.0f-leg_2_theta_2),-(PI/2.0f-leg_3_theta_1),-(PI/2.0f-leg_3_theta_2));	
}
/**
* Takes the leg parameters and returns the gamma angle (rad) of the legs
*/
void GetGamma(float *L,float *gamma) 
{
	float cos_param = (pow(L1,2.0f) + pow(*L,2.0f) - pow(L2,2.0f)) / (2.0f*L1*(*L));
	if (cos_param < -1.0f) {
			*gamma = PI;
			#ifdef DEBUG_HIGH
			printf("ERROR: L is too small to find valid alpha and beta!");
			#endif
		} 
	else if (cos_param > 1.0f) {
			*gamma = 0;
			#ifdef DEBUG_HIGH
			printf("ERROR: L is too large to find valid alpha and beta!");
			#endif
		} 
	else {
			*gamma = acos(cos_param);
		}
}
/**
* Converts the leg params L, gamma to cartesian coordinates x, y (in m)
* Set x_direction to 1.0 or -1.0 to change which direction the leg walks
*/
void LegParamsToCartesian(float L, float theta, float leg_direction, float* x, float* y) {
    *x = leg_direction * L * arm_cos_f32(theta);
    *y = L * arm_sin_f32(theta);
}
/**
* Converts the cartesian coords x, y (m) to leg params L (m), theta (rad)
*/
void CartesianToLegParams(float x, float y, float leg_direction, float *L, float* theta) {
    *L = pow((pow(x,2.0f) + pow(y,2.0f)), 0.5f);
    *theta = atan2((leg_direction * x),y);
}

/**
* sinusoidal trajectory generator function with flexibility from parameters described below. Can do 4-beat, 2-beat, trotting, etc with this.
*/
void SinTrajectory (float t, GaitParams_t *params, float gaitOffset) {
		static float p ;
		static double prev_t;//上次时间

		float stanceHeight = params->stance_height;
		float downAMP = params->down_amp;
		float upAMP = params->up_amp;
		float flightPercent = params->flight_percent;
		float stepLength = params->step_length;
		float FREQ = params->freq;

		p += FREQ * (t - prev_t < 0.5f ? t - prev_t : 0); // should reduce the lurching when starting a new gait
		prev_t = t;
		float gp = fmod((p+gaitOffset),1.0f); // mod(a,m) returns remainder division of a by m//求余
		if (gp <= flightPercent) {
				params->x = (gp/flightPercent)*stepLength - stepLength/2.0f;
				params->y = -upAMP*arm_sin_f32(PI*gp/flightPercent) + stanceHeight;
		}
		else {
				float percentBack = (gp-flightPercent)/(1.0f-flightPercent);
				params->x = -percentBack*stepLength + stepLength/2.0f;
				params->y = downAMP*arm_sin_f32(PI*percentBack) + stanceHeight;
		}
}
void CartesianToThetaGamma(GaitParams_t *params,float leg_direction){//坐标转换theta;gamma
    CartesianToLegParams(params->x, params->y, leg_direction, &(params->L), &(params->theta));//计算出L和theta
    GetGamma(&params->L,&params->gamma);//计算出gamma
}
void Theta_Gamma_To_moto_angle(GaitParams_t *params,uint8_t leg_num){//转换为水平与上关节角度
	switch(leg_num)
	{ 
		case 0:
		{	leg_0_theta_2=PI/2.0f-(params->theta)-(params->gamma);leg_0_theta_1=PI-2.0f*(params->gamma)-leg_0_theta_2;}break;
		case 1:
		{	leg_1_theta_2=PI/2.0f-(params->theta)-(params->gamma);leg_1_theta_1=PI-2.0f*(params->gamma)-leg_1_theta_2;}break;
		case 2:
		{	leg_2_theta_2=PI/2.0f-(params->theta)-(params->gamma);leg_2_theta_1=PI-2.0f*(params->gamma)-leg_2_theta_2;}break;
		case 3:
		{	leg_3_theta_2=PI/2.0f-(params->theta)-(params->gamma);leg_3_theta_1=PI-2.0f*(params->gamma)-leg_3_theta_2;}break;
	}
}
uint8_t IsValidGaitParams(GaitParams_t *params){//检验步态参数是否合理
    const float maxL = 0.30f;
    const float minL = 0.11f;

    float stanceHeight = params->stance_height;
    float downAMP = params->down_amp;
    float upAMP = params->up_amp;
    float flightPercent = params->flight_percent;
    float stepLength = params->step_length;
    float FREQ = params->freq;

    if (stanceHeight + downAMP > maxL || sqrt(pow(stanceHeight, 2) + pow(stepLength / 2.0f, 2)) > maxL) {
        printf("Gait overextends leg");
        return 0;
    }
    if (stanceHeight - upAMP < minL) {
        printf("Gait underextends leg");
        return 0;
    }

    if (flightPercent < 0 || flightPercent > 1.0f) {
				printf("Flight percent is invalid");
        return 0;
    }

    if (FREQ < 0) {
        printf("Frequency cannot be negative");
        return 0;
    }

    if (FREQ > 10.0f) {
        printf("Frequency is too high (>10)");
        return 0;
    }
    return 1;
}

void Send_To_Moto(float Ref_Angle_1,float Ref_Angle_2,float Ref_Angle_3,float Ref_Angle_4,float Ref_Angle_5,float Ref_Angle_6,float Ref_Angle_7,float Ref_Angle_8){
	float angle_ref[8]={Ref_Angle_1,Ref_Angle_2,Ref_Angle_3,Ref_Angle_4,Ref_Angle_5,Ref_Angle_6,Ref_Angle_7,Ref_Angle_8};
	Leg_Control_Loop(angle_ref);
}
void CommandAllLegs(float theta,float gamma){
	static float moto_theta_1,moto_theta_2,All_MOTO_ANGLE[8];
	moto_theta_2=PI/2.0f-(theta)-(gamma);
	moto_theta_1=PI-2*(gamma)-moto_theta_2;
	
	All_MOTO_ANGLE[0]=(-(PI/2.0f-moto_theta_1));All_MOTO_ANGLE[1]=(-(PI/2.0f-moto_theta_2));
	All_MOTO_ANGLE[2]=((PI/2.0f-moto_theta_1)); All_MOTO_ANGLE[3]=((PI/2.0f-moto_theta_2));
	All_MOTO_ANGLE[4]=((PI/2.0f-moto_theta_1)); All_MOTO_ANGLE[5]=((PI/2.0f-moto_theta_2));
	All_MOTO_ANGLE[6]=(-(PI/2.0f-moto_theta_1));All_MOTO_ANGLE[7]=(-(PI/2.0f-moto_theta_2));
	Leg_Control_Loop(All_MOTO_ANGLE);
}	
void State_Trans(void){
	if(state==JUMP||state==RESTART){
		return;
	}
	if(state==HOP){
		if(!(switch_is_down(get_remote_control_point()->rc.s[1]))){
			state=STOP;
		}
		return ;
	}
	D_TIME JUMP_FLAG_TIME;
	if(state!=INIT&&get_remote_control_point()->rc.ch[3]>630&&xTaskGetTickCount()-JUMP_FLAG_TIME.press_time>2000){
		JUMP_FLAG_TIME.press_time=xTaskGetTickCount();
		StartJump(xTaskGetTickCount()/1000.0f);
		return;
	}
	if(switch_is_up(get_remote_control_point()->rc.s[1])||Detect_Judeg(0)){
		state=INIT;
	}
	else if(switch_is_down(get_remote_control_point()->rc.s[0])){
		state=STOP;
	}
	else if(switch_is_mid(get_remote_control_point()->rc.s[0])){
		if(fabs((float)get_remote_control_point()->rc.ch[4])>300){
			state=WALK;//WALK
		}
//		else if(fabs((float)get_remote_control_point()->rc.ch[0])>2000){
//			state=ROTATE;
//		}
		else{
			state=Stepping;
		}
	}
	else if(switch_is_up(get_remote_control_point()->rc.s[0]))
	{
		if(fabs((float)get_remote_control_point()->rc.ch[4])>300){
			state=TROT;//TROT
		}
//		else if(fabs((float)get_remote_control_point()->rc.ch[0])>200){
//			state=ROTATE;
//		}
		else{
			state=Stepping;
		}
	}
	if(last_state==STOP&&state==INIT){//
		StartRest(xTaskGetTickCount()/1000.0f);
		return;
	}
	if(state!=HOP&&switch_is_down(get_remote_control_point()->rc.s[1])){
		StartHop(xTaskGetTickCount()/1000.0f);
	}
}

