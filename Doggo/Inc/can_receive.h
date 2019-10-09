#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"
/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
		CAN_3508_M5_ID = 0x201,
    CAN_3508_M6_ID = 0x202,
    CAN_3508_M7_ID = 0x203,
    CAN_3508_M8_ID = 0x204,

} can_msg_id_e;
//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
		uint16_t send_ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
		fp32    total_angle;
		uint16_t offset_ecd;
		int32_t round_cnt;
} motor_measure_t;
//陀螺仪数据结构体
typedef struct
{
	float v_x;
	float v_z;
	double pit;
	double yaw;
	float acc_x;
	float acc_z;
	float acc_y;
	double yaw_init_offset;
	int16_t raw_pit;
	int16_t raw_yaw;
	
}Gyro_info_t;

void Send_3508_Id1_Id4(int16_t a, int16_t b, int16_t c, int16_t d);
void Send_3508_Id5_Id8(int16_t a, int16_t b, int16_t c, int16_t d);
void Send_Gyro_jiaozhun(uint8_t mode ,uint16_t time);//
extern const motor_measure_t *get_Leg_moto_Measure_Point(uint8_t i);
#endif
