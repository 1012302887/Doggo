#include "math.h"
#include "can_receive.h"
#include "Position_Control.h"
#include "jump.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "NI_MING.h"
#include "FreeRTOS.h"
#include "task.h"
#include "3508_drive.h"
#include "remote_control.h"
#include "detect_task.h"
#include "user_lib.h"
#include "HX711.h"
#define L1 0.11f // upper leg length (m)
#define L2 0.19f // lower leg length (m)
float real_x,real_y;float weight;
void MOTO_ANGLE_TO_X_Y(float theta1,float theta2);
fp32 Angle_Gain[3]={10,0,100};fp32 Speed_Gain[3]={80,0,120};
float leg_0_theta_1,leg_0_theta_2,leg_1_theta_1,leg_1_theta_2,leg_2_theta_1,leg_2_theta_2,leg_3_theta_1,leg_3_theta_2;
States_e state;
States_e last_state;
GaitParams_t gait_params ;
// {stance_height, down_AMP, up_AMP, flight_percent (proportion), step_length, FREQ}
GaitParams_t state_gait_params[] = 
{
   //{s.h, d.a., u.a., f.p., s.l., fr., s.d.}
    {0, 0, 0, 0, 0, 0, 0}, // STOP
    {0.25, 0.04, 0.06, 0.35, 0.15, 2.0, 0.0}, // TROT
    {0.30, 0.04, 0.06, 0.35, 0.0, 2.0, 0.0}, // BOUND
    {0.29, 0.00, 0.15, 1.0, 0.0, 1.5, 0.0}, // WALK
    {0.20, 0.05, 0.0, 0.75, 0.0, 1.0, 0.0}, // PRONK
    {0, 0, 0, 0, 0, 0, 0}, // JUMP
    {0.25, 0.05, 0.05, 0.35, 0.0, 1.5, 0.0}, // DANCE
    {0.25, 0.04, 0.05, 0.2, 0.0, 1.0, 0.0}, // HOP
    {0, 0, 0, 0, 0, 1.0, 0}, // TEST
    {0, 0, 0, 0, 0, 0, 0}, // ROTATE
    {0.25, 0.07, 0.06, 0.2, 0.0, 1.0, 0.0}, // FLIP
    {0.25, 0.04, 0.06, 0.35, 0.1, 2.0, 0.06}, // TURN_TROT
    {0, 0, 0, 0, 0, 0, 0} // RESET
};
extern void Trans_Jacobian(float Theta_1,float Theta_2,float current_1,float current_2);
void CoupledMoveLeg(uint8_t leg_num,float t, GaitParams_t *params,float gait_offset, float leg_direction) 
{
    SinTrajectory(t,params,gait_offset);
    CartesianToThetaGamma(params,leg_direction);//坐标转换角度
    Theta_Gamma_To_moto_angle(params,leg_num);//转换为水平与上关节角度
//		Trans_Jacobian(leg_0_theta_1,2*params->gamma+leg_0_theta_1,fabs(get_Leg_moto_Measure_Point(0)->given_current*0.3/1000),\
//		fabs(get_Leg_moto_Measure_Point(1)->given_current*0.3/1000));
//		serial_plot(4,params->x,params->y,real_x,real_y);
}
extern float jacobian_out_[2];float P_1,P_2,D_1,D_2;
float jacobian_gamma,jacobian_theta;
void PositionControlThread(void *pvParameters)
{
	uint32_t time_add;
	Get_Maopi();//获取毛皮质量
	state = INIT;
	gait_params = state_gait_params[state];
	Leg_Moto_Data_Init();
	Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
	Get_Maopi();//获取毛皮质量
	for(;;)
	{
		time_add++;
//	  jacobian_gamma=fabs(get_Leg_moto_Measure_Point(0)->total_angle+get_Leg_moto_Measure_Point(1)->total_angle)/2.0f;
//		jacobian_theta=fabs(get_Leg_moto_Measure_Point(1)->total_angle-get_Leg_moto_Measure_Point(0)->total_angle);
//		Trans_Jacobian(fabs(90.0f-jacobian_gamma-jacobian_theta),2*jacobian_gamma+fabs(90.0f-jacobian_gamma-jacobian_theta),\
//		get_Leg_moto_Measure_Point(0)->given_current*3.0f/10000.0f,get_Leg_moto_Measure_Point(1)->given_current*3.0f/10000.0f);
		if(time_add%6==0)
		{
//		Ni_Ming(0xf1,Leg_Move.Moto_Angle_Pid[0].set,Leg_Move.Moto_Angle_Pid[0].fdb,\
//		get_Leg_moto_Measure_Point(0)->given_current/1000,get_Leg_moto_Measure_Point(1)->given_current/1000);
//			serial_plot(4,Leg_Move.Moto_Angle_Pid[0].set,Leg_Move.Moto_Angle_Pid[0].fdb,\
//			Leg_Move.Moto_Angle_Pid[1].set,Leg_Move.Moto_Angle_Pid[1].fdb);
		}
		if(last_state!=state)
		{
			gait_params = state_gait_params[state];//更新步态参数
		}
		last_state=state;
    State_Trans();//状态转换
	  switch(state) 
		{
			case INIT:{
				NULL;
			}break;
			case STOP:{
				Angle_Gain[0]=7;Angle_Gain[2]=100;Speed_Gain[0]=80;Speed_Gain[2]=120;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				gait_params.x=0;gait_params.y=0.26;//指定坐标
				CartesianToThetaGamma(&gait_params,1);
				Theta_Gamma_To_moto_angle(&gait_params,0);//转换为水平与上关节角度
				CommandAllLegs(gait_params.theta,gait_params.gamma);
			}break;
			case DANCE:{
				gait(&gait_params, 0.0, 0.5, 0.0, 0.5);
			}break;
			case BOUND:{
				gait(&gait_params, 0.0, 0.5, 0.5, 0.0);
			}break;
			case TROT:{
				Angle_Gain[0]=12;Angle_Gain[2]=100;Speed_Gain[0]=100;Speed_Gain[2]=100;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				gait(&gait_params, 0.0, 0.5, 0.0, 0.5);
			}break;
			case TURN_TROT:{
				gait(&gait_params, 0.0, 0.5, 0.0, 0.5);
			}break;
			case WALK:{
				Angle_Gain[0]=35;Angle_Gain[2]=100;Speed_Gain[0]=100;Speed_Gain[2]=120;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				gait(&gait_params, 0.0, 0.25, 0.75, 0.5);
			}break;
			case PRONK:{
				gait(&gait_params, 0.0, 0.0, 0.0, 0.0);
			}break;
			case JUMP:{
				ExecuteJump();
			}break;
			case ROTATE:{
						long rotate_start = 0; // milliseconds when rotate was commanded
						gait_params.x=0;gait_params.y=0.24f;
						CartesianToThetaGamma(&gait_params,1.0);
						float freq = 0.1;
						float phase = freq * (HAL_GetTick() - rotate_start)/1000.0f;
						gait_params.theta = (-cos(2*PI * phase) + 1.0f) * 0.5 * 2 * PI;
						Theta_Gamma_To_moto_angle(&gait_params,0);//转换为电机目标角度
						//每条腿步态相同
						Send_To_Moto(-(90.0f-leg_0_theta_1),-(90.0f-leg_0_theta_2),-(90.0f-leg_0_theta_1),-(90.0f-leg_0_theta_2),\
						-(90.0f-leg_0_theta_1),-(90.0f-leg_0_theta_2),-(90.0f-leg_0_theta_1),-(90.0f-leg_0_theta_2));
			}break;
			case HOP:{
				hop(&gait_params);
			}break;
//			case FLIP:
//					ExecuteFlip(gait_params);
//					break;
//			case RESET:
//					reset();
//					break;
//			case TEST:
//					test();
//					break;
		}
		vTaskDelay(1);
	}
}
void gait(GaitParams_t *params,float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset) 
{
		double t; 
		GaitParams_t paramsR = *params;GaitParams_t paramsL = *params;
    paramsR.step_length -= params->step_diff;
    paramsL.step_length += params->step_diff;

    if (!IsValidGaitParams(&paramsR)||!IsValidGaitParams(&paramsL)) 
		{
        return;
    }
		
    t = xTaskGetTickCount()/1000.0f;
    const float leg0_direction = -1.0;//方向
    CoupledMoveLeg(0,t, &paramsL, leg0_offset, leg0_direction);
    const float leg1_direction = -1.0;
    CoupledMoveLeg(1,t, &paramsL, leg1_offset, leg1_direction);
    const float leg2_direction = 1.0;
    CoupledMoveLeg(2,t, &paramsR, leg2_offset, leg2_direction);
    const float leg3_direction = 1.0;
    CoupledMoveLeg(3,t, &paramsR, leg3_offset, leg3_direction);
		
		Send_To_Moto(-(90.0f-leg_0_theta_1),-(90.0f-leg_0_theta_2),-(90.0f-leg_1_theta_1),-(90.0f-leg_1_theta_2),\
		-(90.0f-leg_2_theta_1),-(90.0f-leg_2_theta_2),-(90.0f-leg_3_theta_1),-(90.0f-leg_3_theta_2));
}
/**
* Takes the leg parameters and returns the gamma angle (rad) of the legs
*/
void GetGamma(float *L,float *gamma) 
{
	float cos_param = (pow(L1,2.0f) + pow(*L,2.0f) - pow(L2,2.0f)) / (2.0f*L1*(*L));
	if (cos_param < -1.0f) 
		{
			*gamma = PI;
			#ifdef DEBUG_HIGH
			printf("ERROR: L is too small to find valid alpha and beta!");
			#endif
		} 
	else if (cos_param > 1.0f) 
		{
			*gamma = 0;
			#ifdef DEBUG_HIGH
			printf("ERROR: L is too large to find valid alpha and beta!");
			#endif
		} 
	else 
		{
			*gamma = acos(cos_param);
		}
}
/**
* Converts the leg params L, gamma to cartesian coordinates x, y (in m)
* Set x_direction to 1.0 or -1.0 to change which direction the leg walks
*/
void LegParamsToCartesian(float L, float theta, float leg_direction, float* x, float* y) 
{
    *x = leg_direction * L * arm_cos_f32(theta);
    *y = L * arm_sin_f32(theta);
}
/**
* Converts the cartesian coords x, y (m) to leg params L (m), theta (rad)
*/
void CartesianToLegParams(float x, float y, float leg_direction, float *L, float* theta) 
{
    *L = pow((pow(x,2.0f) + pow(y,2.0f)), 0.5f);
    *theta = atan2((leg_direction * x),y);
}

/**
* sinusoidal trajectory generator function with flexibility from parameters described below. Can do 4-beat, 2-beat, trotting, etc with this.
*/
void SinTrajectory (float t, GaitParams_t *params, float gaitOffset) 
{
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
		if (gp <= flightPercent) 
		{
				params->x = (gp/flightPercent)*stepLength - stepLength/2.0f;
				params->y = -upAMP*arm_sin_f32(PI*gp/flightPercent) + stanceHeight;
		}
		else 
		{
				float percentBack = (gp-flightPercent)/(1.0f-flightPercent);
				params->x = -percentBack*stepLength + stepLength/2.0f;
				params->y = downAMP*arm_sin_f32(PI*percentBack) + stanceHeight;
		}
}
void CartesianToThetaGamma(GaitParams_t *params,float leg_direction)//坐标转换theta;gamma
{
    CartesianToLegParams(params->x, params->y, leg_direction, &params->L, &params->theta);//计算出L和theta
    GetGamma(&params->L,&params->gamma);//计算出gamma
}
void MOTO_ANGLE_TO_X_Y(float theta1,float theta2)
{
	float gamma=(180.0f-fabs(theta1)-fabs(theta2))/2.0f;float theta=90.0f-fabs(theta2)-gamma;
//	float L=2*0.11*0.19*arm_cos_f32(gamma)-pow(0.11,2)-pow(0.19,2);
//	real_x=L*arm_sin_f32(theta);real_y=L*arm_cos_f32(theta);
}
void Theta_Gamma_To_moto_angle(GaitParams_t *params,uint8_t leg_num)//转换为水平与上关节角度
{
	switch(leg_num)
	{ 
		case 0:
		{	leg_0_theta_2=90.0f-(params->theta*180/PI)-(params->gamma*180/PI);leg_0_theta_1=180.0f-2*(params->gamma*180/PI)-leg_0_theta_2;}break;
		case 1:
		{	leg_1_theta_2=90.0f-(params->theta*180/PI)-(params->gamma*180/PI);leg_1_theta_1=180.0f-2*(params->gamma*180/PI)-leg_1_theta_2;}break;
		case 2:
		{	leg_2_theta_2=90.0f-(params->theta*180/PI)-(params->gamma*180/PI);leg_2_theta_1=180.0f-2*(params->gamma*180/PI)-leg_2_theta_2;}break;
		case 3:
		{	leg_3_theta_2=90.0f-(params->theta*180/PI)-(params->gamma*180/PI);leg_3_theta_1=180.0f-2*(params->gamma*180/PI)-leg_3_theta_2;}break;
	}
}
uint8_t IsValidGaitParams(GaitParams_t *params) {//检验步态参数是否合理
    const float maxL = 0.31f;
    const float minL = 0.12f;

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

    if (flightPercent <= 0 || flightPercent > 1.0f) {
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
void hop(GaitParams_t *params) 
{
	CartesianToThetaGamma(params,1);//腿上升部分
	
	CommandAllLegs(params->theta,params->gamma);//电机转指定角度。腿上升部分
	vTaskDelay(1000000*0.2f/params->freq);
	
	CartesianToThetaGamma(params,1);

	CommandAllLegs(params->theta,params->gamma);//电机转指定角度。腿下降部分
	vTaskDelay(1000000*0.2f/params->freq);
	
	CartesianToThetaGamma(params,1);
	
	CommandAllLegs(params->theta,params->gamma);//电机转指定角度。腿恢复部分
	vTaskDelay(1000000*(0.8f-params->flight_percent)/params->freq);
}
void Send_To_Moto(float Ref_Angle_1,float Ref_Angle_2,float Ref_Angle_3,float Ref_Angle_4,float Ref_Angle_5,float Ref_Angle_6,float Ref_Angle_7,float Ref_Angle_8)
{
		float angle_ref[8]={Ref_Angle_1,Ref_Angle_2,Ref_Angle_3,Ref_Angle_4,Ref_Angle_5,Ref_Angle_6,Ref_Angle_7,Ref_Angle_8};
		Leg_Control_Loop(angle_ref);
}
void CommandAllLegs(float theta,float gamma)
{
	static float moto_angle_1,moto_angle_2,All_MOTO_ANGLE[8];
	moto_angle_2=90.0f-(theta*180/PI)-(gamma*180/PI);
	moto_angle_1=180.0f-2*(gamma*180/PI)-moto_angle_2;
	
	All_MOTO_ANGLE[0]=(-(90.0f-moto_angle_1));All_MOTO_ANGLE[1]=(-(90.0f-moto_angle_2));
	All_MOTO_ANGLE[2]=(-(90.0f-moto_angle_1));All_MOTO_ANGLE[3]=(-(90.0f-moto_angle_2));
	All_MOTO_ANGLE[4]=(-(90.0f-moto_angle_1));All_MOTO_ANGLE[5]=(-(90.0f-moto_angle_2));
	All_MOTO_ANGLE[6]=(-(90.0f-moto_angle_1));All_MOTO_ANGLE[7]=(-(90.0f-moto_angle_2));
	
	Leg_Control_Loop(All_MOTO_ANGLE);
}	
void State_Trans(void)
{
	if(state==JUMP)
	{
		return;
	}
	D_TIME JUMP_FLAG_TIME;
	if(state!=INIT&&state!=STOP&&get_remote_control_point()->rc.ch[1]>630&&xTaskGetTickCount()-JUMP_FLAG_TIME.press_time>1000)
	{
		JUMP_FLAG_TIME.press_time=xTaskGetTickCount();
		StartJump(xTaskGetTickCount()/1000.0f);
		return;
	}
	if(switch_is_up(get_remote_control_point()->rc.s[1])||Detect_Judeg(0))
	{
		state=INIT;
	}
	else if(switch_is_down(get_remote_control_point()->rc.s[0]))
	{
		state=STOP;
	}
	else if(switch_is_mid(get_remote_control_point()->rc.s[0]))
	{
		state=WALK;
	}
	else if(switch_is_up(get_remote_control_point()->rc.s[0]))
	{
		state=TROT;
	}
}

