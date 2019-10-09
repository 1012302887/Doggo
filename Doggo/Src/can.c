#include "can.h"
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_FilterTypeDef FILTER1;
CAN_FilterTypeDef FILTER2;
CAN_TxHeaderTypeDef    Tx1Message;      
CAN_RxHeaderTypeDef    Rx1Message;  
CAN_TxHeaderTypeDef    Tx2Message;      
CAN_RxHeaderTypeDef    Rx2Message;

void MX_CAN1_Init(void)
{
//printf("CAN1__INIT");
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
	FILTER1.FilterIdHigh=0X0000;     //32位ID
  FILTER1.FilterIdLow=0X0000;
  FILTER1.FilterMaskIdHigh=0X0000; //32位MASK
  FILTER1.FilterMaskIdLow=0X0000;  
  FILTER1.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  FILTER1.FilterBank=0;          //过滤器0
  FILTER1.FilterMode=CAN_FILTERMODE_IDMASK;
  FILTER1.FilterScale=CAN_FILTERSCALE_32BIT;
  FILTER1.FilterActivation=ENABLE; //激活滤波器0
  FILTER1.SlaveStartFilterBank=14;
  HAL_CAN_ConfigFilter(&hcan1,&FILTER1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* CAN2 init function */
void MX_CAN2_Init(void)
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  FILTER2.FilterIdHigh=0X0000;     //32位ID
  FILTER2.FilterIdLow=0X0000;
  FILTER2.FilterMaskIdHigh=0X0000; //32位MASK
  FILTER2.FilterMaskIdLow=0X0000;  
  FILTER2.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  FILTER2.FilterBank=14;
  FILTER2.FilterMode=CAN_FILTERMODE_IDMASK;
  FILTER2.FilterScale=CAN_FILTERSCALE_32BIT;
  FILTER2.FilterActivation=ENABLE; 
  FILTER2.SlaveStartFilterBank=14;
  HAL_CAN_ConfigFilter(&hcan2,&FILTER2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}
