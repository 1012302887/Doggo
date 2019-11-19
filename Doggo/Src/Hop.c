#include "Hop.h"
#include "position_control.h"
#include "3508_DRIVE.h"
#include "task.h"
#include "user_lib.h"
static float hop_start_time_ = 0.0f;
static ramp_t hop_Up_Ramp;
void StartHop(float start_time_s) {
    hop_start_time_ = start_time_s;
    state = HOP;
		ramp_init(&hop_Up_Ramp,1000);
}
void hop(GaitParams_t *params) {
    // min radius = 0.8
    // max radius = 0.25
    const float prep_time = 1.0f; 
    const float launch_time = 1.0f ; 

    const float stance_height = 0.16f; // Desired leg extension before the jump [m]
		const float Jump_height = 0.27f; 
    float t = xTaskGetTickCount()/1000.0f - hop_start_time_; // Seconds since jump was commanded

    if (t < prep_time) {
//        gait_params.x = 0;
//        gait_params.y = stance_height;
//        CartesianToThetaGamma(&gait_params,1.0);

				Angle_Gain[0]=200;Angle_Gain[2]=100;Speed_Gain[0]=300;Speed_Gain[2]=100;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
			
				float x[4]={0,0,\
				0,0};
				float y[4]={stance_height,stance_height,\
				stance_height,stance_height};
				static_gait(&gait_params, 0, 0, 0, 0,-1,-1,-1,-1,x,y);
//				/*Ð±ÆÂÊä³ö£¬¹·ÂýËÙÏÂ½µ*/
//				gait_params.theta=gait_params.theta*ramp_calc(&hop_Up_Ramp);
//				gait_params.gamma=gait_params.gamma*ramp_calc(&hop_Up_Ramp);
//
//        CommandAllLegs(gait_params.theta,gait_params.gamma);
    } else if (t >= prep_time && t < prep_time + launch_time) {
				Angle_Gain[0]=200;Angle_Gain[1]=0;Angle_Gain[2]=100;Speed_Gain[0]=300;Speed_Gain[1]=0 ;Speed_Gain[2]=100;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				float x[4]={gait_params.step_length,gait_params.step_length,\
				gait_params.step_length,gait_params.step_length};
				float y[4]={Jump_height,Jump_height,Jump_height,Jump_height};
				static_gait(&gait_params, 0, 0, 0, 0,-1,-1,-1,-1,x,y);
    }
		else{
			hop_start_time_=(xTaskGetTickCount()/1000.0f);
			ramp_init(&hop_Up_Ramp,1000);
		}	
}
