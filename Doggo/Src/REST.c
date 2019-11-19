#include "REST.h"
#include "position_control.h"
#include "3508_DRIVE.h"
#include "task.h"
#include "user_lib.h"
static float rest_start_time_ = 0.0f;
static ramp_t Rest_Up_Ramp,Rest_neiba_Ramp;
void StartRest(float start_time_s) {
    rest_start_time_ = start_time_s;
    state = RESTART;
		ramp_init(&Rest_Up_Ramp,2000);
		ramp_init(&Rest_neiba_Ramp,2000);
}
void ExecuteRest(void) {
    // min radius = 0.8
    // max radius = 0.25
    const float prep_time = 1.0f; 
    const float launch_time = 1.0f ; 

    const float stance_height = 0.12f; // Desired leg extension before the jump [m]
	
    float t = xTaskGetTickCount()/1000.0f - rest_start_time_; // Seconds since jump was commanded

    if (t < prep_time) {
        gait_params.x = 0;
        //gait_params.y = stance_height;
        CartesianToThetaGamma(&gait_params,1.0);

				Angle_Gain[0]=100;Angle_Gain[2]=100;Speed_Gain[0]=150;Speed_Gain[2]=100;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
			
				/*Ð±ÆÂÊä³ö£¬¹·ÂýËÙÏÂ½µ*/
				gait_params.theta=gait_params.theta*ramp_calc(&Rest_Up_Ramp);
				gait_params.gamma=gait_params.gamma*ramp_calc(&Rest_Up_Ramp);
			
        CommandAllLegs(gait_params.theta,gait_params.gamma);
    } else if (t >= prep_time && t < prep_time + launch_time) {
				Angle_Gain[0]=100;Angle_Gain[1]=0;Angle_Gain[2]=100;Speed_Gain[0]=300;Speed_Gain[1]=0 ;Speed_Gain[2]=0;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				float x[4]={gait_params.step_length*ramp_calc(&Rest_neiba_Ramp),gait_params.step_length*ramp_calc(&Rest_neiba_Ramp),\
				gait_params.step_length*ramp_calc(&Rest_neiba_Ramp),gait_params.step_length*ramp_calc(&Rest_neiba_Ramp)};
				float y[4]={stance_height,stance_height,stance_height,stance_height};
				static_gait(&gait_params, 0, 0, 0, 0,1,-1,1,-1,x,y);
    }else {
        state = INIT;
    }
}
