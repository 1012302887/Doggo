#include "arm_math.h"
#include "NI_MING.h"
arm_matrix_instance_f32 jacobian,torque,jacobian_inv,jacobian_tran,jacobian_out;
float J[4],torque_[2],J_NEG[4],J_tran[4],jacobian_out_[2];
float Beta,Cos_Beta,Sin_Beta,Cos_Delta,Sin_Delta,A,D;
void Trans_Jacobian(float Theta_1,float Theta_2,float current_1,float current_2){
	current_1=fabs(current_1);current_2=fabs(current_2);
	Theta_2=fabs(Theta_2);Theta_1=fabs(Theta_1);
	Beta=Theta_2-Theta_1;
	Cos_Beta=arm_cos_f32(Beta);Sin_Beta=arm_sin_f32(Beta);
	A=1.0f/(2.0f*Sin_Beta);
	D=0.11f/(2.0f*0.19f*__sqrtf(2.0f))/__sqrtf(1.0f-Cos_Beta)/__sqrtf(1-(0.11f/0.19f)*(0.5f-Cos_Beta/2.0f));
	Cos_Delta=-arm_cos_f32(Theta_1-acos(__sqrtf((1.0f-Cos_Beta)/2.0f))-acos((0.11f/0.19f)*__sqrtf((1.0f-Cos_Beta)/2.0f)));
	Sin_Delta=-arm_sin_f32(Theta_1-acos(__sqrtf((1.0f-Cos_Beta)/2.0f))-acos((0.11f/0.19f)*__sqrtf((1.0f-Cos_Beta)/2.0f)));
	arm_mat_init_f32(&jacobian,2,2,J);arm_mat_init_f32(&torque,2,1,torque_);arm_mat_init_f32(&jacobian_inv,2,2,J_NEG);
	arm_mat_init_f32(&jacobian_tran,2,2,J_tran);arm_mat_init_f32(&jacobian_out,2,1,jacobian_out_);
	J[0]=-0.11f*arm_sin_f32(Theta_1)-0.19f*(1-(A+D))*Sin_Delta;
	J[1]=-0.19f*(A+D)*Sin_Delta;
	J[2]=0.11f*arm_cos_f32(Theta_1)+0.19f*(1-(A+D))*Cos_Delta;
	J[3]=0.19f*(A+D)*Cos_Delta;
	torque_[0]=current_1;torque_[1]=current_2;
	arm_mat_trans_f32(&jacobian,&jacobian_tran);arm_mat_inverse_f32(&jacobian_tran,&jacobian_inv);
	arm_mat_mult_f32(&jacobian_inv,&torque,&jacobian_out);
}
