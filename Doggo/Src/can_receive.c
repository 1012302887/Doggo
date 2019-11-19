#include "CAN_Receive.h"
#include "user_lib.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "detect_task.h"
#include "remote_control.h"
#include "position_control.h"
#include "arm_math.h"
Gyro_info_t Gyro_info;
extern States_e state;
//声明电机变量
static motor_measure_t Leg_motor[8];
//底盘电机数据读取
void get_motor_measure(motor_measure_t* ptr, uint8_t Data[])                                                     
{
    static	int32_t total_ecd;   
    if(state==INIT)
    {
       ptr->offset_ecd=(uint16_t)(Data[0] << 8 | Data[1]);
       ptr->round_cnt=0;
    }      
//	(ptr)->last_ecd = (ptr)->ecd;                                                          
//	(ptr)->ecd = (uint16_t)(Data[0] << 8 | Data[1]);          
    (ptr)->speed_rpm = (uint16_t)(Data[2] << 8 | Data[3]);
    (ptr)->speed_rpm/=15.4f;
		(ptr)->given_current = (uint16_t)(Data[4] << 8 | Data[5]);
		(ptr)->temperate = Data[6];                               
    ptr->last_ecd = ptr->ecd;
    ptr->ecd      = (uint16_t)(Data[0] << 8 | Data[1]);
    if (ptr->ecd - ptr->last_ecd > 6666)
    {
        ptr->round_cnt--;
    }
    else if (ptr->ecd - ptr->last_ecd < -6666)
    {
        ptr->round_cnt++;
    }
    total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
    /* total angle, unit is rad */
//    ptr->total_angle = loop_fp32_constrain(total_ecd/(8192.0f/(2*PI))/15.4f,-PI,PI);
		if(ptr==&Leg_motor[0]){
				ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)-0.92f,-PI,PI);
		}
		else if(ptr==&Leg_motor[1]){
				ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)+2.7f,-PI,PI);
				if(ptr->total_angle>2.0f){
					ptr->total_angle=ptr->total_angle-2*PI;
				}
		}
		else if(ptr==&Leg_motor[4]){
				ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)+0.84f,-PI,PI);
		}
		else if(ptr==&Leg_motor[5]){
				ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)-2.80f,-PI,PI);
				if(ptr->total_angle<-2.0f){
					ptr->total_angle=ptr->total_angle+2*PI;
			}
		}
		else if(ptr==&Leg_motor[2]){
			ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)-2.69f,-PI,PI);
				if(ptr->total_angle<-2.0f){
					ptr->total_angle=ptr->total_angle+2*PI;
			}
		}
		else if(ptr==&Leg_motor[3]){
			ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)+1.0f,-PI,PI);
		}
		else if(ptr==&Leg_motor[6]){
			ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)+2.8f,-PI,PI);
				if(ptr->total_angle>2.0f){
					ptr->total_angle=ptr->total_angle-2*PI;
			}
		}
		else if(ptr==&Leg_motor[7]){
			ptr->total_angle = loop_fp32_constrain((total_ecd/(8192.0f/(2*PI))/15.4f)-0.93f,-PI,PI);
		}
}
//返回陀螺仪变量地址，通过指针方式获取原始数据
const Gyro_info_t *get_GYRO_Measure_Point(void)
{
	return &Gyro_info;
}
//返回腿电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Leg_moto_Measure_Point(uint8_t i)
{
	return &Leg_motor[i];
}
void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t DATA[8];//储存CAN1接收到数据
	static uint8_t Data[8];//储存CAN2接收到数据
	if (hcan == &hcan1)
	{ 
		//CAN1接收
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rx1Message,DATA);
		switch (Rx1Message.StdId)
		{
      static uint8_t i = 0;
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:{
 				i = Rx1Message.StdId - CAN_3508_M1_ID;
				get_motor_measure(&Leg_motor[i], DATA);
			}break;
			default:
			{
			}break;
		}	
	}	
	else if (hcan == &hcan2)
	{
		//CAN2接收
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&Rx2Message,Data);
		switch (Rx2Message.StdId)
		{
			case 101:
				{
					DetectHook(1);//注册时间，判断是否陀螺仪掉线
					static int32_t yaw_connt=0;
					static int16_t raw_v_x, raw_v_z;
					static float pitch_angle,yaw_angle,last_yaw_angle=0;
					raw_v_x = Data[0]<<8 | Data[1];
					raw_v_z = Data[2]<<8 | Data[3];
					Gyro_info.raw_pit = Data[4]<<8 | Data[5];
					Gyro_info.raw_yaw   = Data[6]<<8 | Data[7];
					//陀螺仪原始数据是弧度，把弧度转换为角度
					Gyro_info.v_x = -(float)raw_v_x * 0.057295f;//pit_Speed
					Gyro_info.v_z = (float)raw_v_z * 0.057295f;//YAW_Speed
					
					//陀螺仪原始数据被乘了100倍
					pitch_angle = (float)Gyro_info.raw_pit/100.0f;
					yaw_angle  = (float)Gyro_info.raw_yaw/100.0f;
					Gyro_info.pit = pitch_angle;
					if(Gyro_info.pit<-300)
					{
						Gyro_info.pit+=360;
					}
					else if(Gyro_info.pit>300)
					{
						Gyro_info.pit-=360;
					}
					
					if((yaw_angle -last_yaw_angle) > 330)
					{
						yaw_connt--;
					}
					else if((yaw_angle - last_yaw_angle) < -330)
					{
						yaw_connt++;
					}
						Gyro_info.yaw =yaw_angle + yaw_connt * 360;
						last_yaw_angle = yaw_angle;
//					printf("-%d-",HAL_GetTick());
				}break;
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			case CAN_3508_M8_ID:{
			static uint8_t ii = 0;
			ii = Rx2Message.StdId - CAN_3508_M1_ID;
			get_motor_measure(&Leg_motor[ii], Data);
			}break;
			default:
			{
			}break;
		}
	}
}
void Send_3508_Id1_Id4(int16_t a, int16_t b, int16_t c, int16_t d)
{ 
	uint8_t i[8];
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.StdId = 0x200;
  Tx1Message.DLC = 0x08;
	
	i[0]= a >> 8;i[1]= a;i[2]= b >> 8;i[3]= b;
	i[4]= c >> 8;i[5]= c;i[6]= d >> 8;i[7]= d;
	HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,i,(uint32_t*)CAN_TX_MAILBOX0);
}
void Send_3508_Id5_Id8(int16_t a, int16_t b, int16_t c, int16_t d)
{
  uint8_t i[8];
  Tx2Message.RTR = CAN_RTR_DATA;
  Tx2Message.IDE = CAN_ID_STD;
  Tx2Message.StdId = 0x1ff;
  Tx2Message.DLC = 0x08;
	i[0]= a >> 8;i[1]= a;i[2]= b >> 8;i[3]= b;
	i[4]= c >> 8;i[5]= c;i[6]= d >> 8;i[7]= d;
	HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,i,(uint32_t*)CAN_TX_MAILBOX0);
}
void Send_Gyro_jiaozhun(uint8_t mode ,uint16_t time)//
{
	static uint8_t Data[8];
	Tx2Message.StdId   = 100;
	Tx2Message.IDE     = CAN_ID_STD;
	Tx2Message.RTR     = CAN_RTR_DATA;
	Tx2Message.DLC     = 3;
	Data[0] = mode;
	Data[1] = time>>8;
	Data[2] = time;
	HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,Data,(uint32_t*)CAN_TX_MAILBOX0);
}
