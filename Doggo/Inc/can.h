#ifndef CAN_H
#define CAN_H
#include "main.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_TxHeaderTypeDef    Tx1Message;      
extern CAN_RxHeaderTypeDef    Rx1Message;  
extern CAN_TxHeaderTypeDef    Tx2Message;      
extern CAN_RxHeaderTypeDef    Rx2Message;
extern void MX_CAN1_Init(void);
extern void MX_CAN2_Init(void);
#endif
