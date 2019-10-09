#include "jump.h"
#include "position_control.h"
#include "3508_DRIVE.h"
#include "task.h"
#include "user_lib.h"
float start_time_ = 0.0f;
ramp_t Jump_Up_Ramp,Jump_Fall_Ramp;
/**
 * Tell the position control thread to do the jump
 * @param start_time_s The timestamp of when the jump command was sent
 */
void StartJump(float start_time_s) {
    start_time_ = start_time_s;
    state = JUMP;
		ramp_init(&Jump_Up_Ramp,1000);
//		ramp_init(&Jump_Fall_Ramp,800);
}

/**
* Linear increase in height for jump.
*/
//void TrajectoryJump(float t, float launchTime, float stanceHeight,
//    float downAMP, float& x, float& y) {
//    //Need to check if n works
//    float n = t/launchTime;
//    x = 0;
//    y = downAMP*n + stanceHeight;
//    //y = downAMP*sin(PI/4 + PI/4*n) + stanceHeight;
//}

/**
* Drives the moto in an open-loop, position-control sinTrajectory.
*/
void ExecuteJump() {
    // min radius = 0.8
    // max radius = 0.25
    const float prep_time = 2.0f; // Duration before jumping [s]
    const float launch_time = 1.0f ; // Duration before retracting the leg [s]
    const float fall_time = 1.5f; //Duration after retracting leg to go back to normal behavior [s]

    const float stance_height = 0.11f; // Desired leg extension before the jump [m]
    const float jump_extension = 0.28f; // Maximum leg extension in [m]
    const float fall_extension = 0.25f; // Desired leg extension during fall [m]

    float t = xTaskGetTickCount()/1000.0f - start_time_; // Seconds since jump was commanded

    if (t < prep_time) {
        gait_params.x = 0;
        gait_params.y = stance_height;
        CartesianToThetaGamma(&gait_params,1.0);

        // Use gains with small stiffness and lots of damping
//				static float a,b,c,d;
				Angle_Gain[0]=6;Angle_Gain[2]=80;Speed_Gain[0]=70;Speed_Gain[2]=100;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
			
				/*Ð±ÆÂÊä³ö£¬ÂýËÙÉÏÉý*/
				gait_params.theta=gait_params.theta*ramp_calc(&Jump_Up_Ramp);
				gait_params.gamma=gait_params.gamma*ramp_calc(&Jump_Up_Ramp);
			
        CommandAllLegs(gait_params.theta,gait_params.gamma);
    } else if (t >= prep_time && t < prep_time + launch_time) {
        gait_params.x = 0;
        gait_params.y = jump_extension;
        CartesianToThetaGamma(&gait_params,1.0);

        // Use high stiffness and low damping to execute the jump
//        static float a,b,c,d;
				Angle_Gain[0]=10;Angle_Gain[2]=100;Speed_Gain[0]=80;Speed_Gain[2]=100;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				CommandAllLegs(gait_params.theta,gait_params.gamma);
    } else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) {
        gait_params.x = 0;
        gait_params.y = fall_extension;
        CartesianToThetaGamma(&gait_params,1.0);

        // Use low stiffness and lots of damping to handle the fall
//				static float a,b,c,d;
				Angle_Gain[0]=6;Angle_Gain[2]=100;Speed_Gain[0]=80;Speed_Gain[2]=120;
				Leg_Moto_Data_Update(Angle_Gain,Speed_Gain);
				/*Ð±ÆÂÊä³ö£¬ÂýËÙ»Ö¸´*/
//				gait_params.theta=gait_params.theta*ramp_calc(&Jump_Fall_Ramp);
//				gait_params.gamma=gait_params.gamma*ramp_calc(&Jump_Fall_Ramp);
        CommandAllLegs(gait_params.theta,gait_params.gamma);
    } else {
        state = STOP;
    }
}

