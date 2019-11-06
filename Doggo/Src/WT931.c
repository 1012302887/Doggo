#include "WT931.h"
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
#define ImuBufferLength 64
static uint8_t ImuDataBuffer[ImuBufferLength];
Imu_Data_s Imu_Data;
void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
		
	SET_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)ImuDataBuffer, ImuBufferLength);
}
void USART3_IRQHandler(void)
{
	static uint32_t	temp;
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE))
    {
      uint16_t tmp = huart3.Instance->DR;
      tmp = huart3.Instance->SR;
      tmp--;
			CLEAR_BIT(huart3.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart3.hdmarx);
	
     	temp = huart3.hdmarx->Instance->NDTR;//NDTR寄存器中设置传输的数据总数，每个外设的事件或每次burst传输之后，这个值是将会递减。
        
			if((ImuBufferLength - temp) >=11 )
			{
				 Imu_Data_Calc();
//				printf("-%d-",HAL_GetTick());
			}
	 }
			SET_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE);
      DMA1->LIFCR = DMA_FLAG_DMEIF0_4  | DMA_FLAG_FEIF0_4  | DMA_FLAG_HTIF0_4  | DMA_FLAG_TCIF0_4  | DMA_FLAG_TEIF0_4 ;
      __HAL_DMA_SET_COUNTER(huart3.hdmarx, ImuBufferLength);
      __HAL_DMA_ENABLE(huart3.hdmarx);
} 
void Imu_Data_Calc(){
	if (ImuDataBuffer[0]==0x55){
		Imu_Data_Analysis(0);
	}
	if(ImuDataBuffer[11]==0x55){
		Imu_Data_Analysis(11);
	}
	if(ImuDataBuffer[22]==0x55){
		Imu_Data_Analysis(22);
	}
}
void Imu_Data_Analysis(uint8_t num){
	switch (ImuDataBuffer[num+1]){
		case 0x51:
		{Imu_Data.a_x=(float)((short)(ImuDataBuffer[num+3]<<8)|ImuDataBuffer[num+2])/32768*16.0f;
		 Imu_Data.a_y=(float)((short)(ImuDataBuffer[num+5]<<8)|ImuDataBuffer[num+4])/32768*16.0f;
		 Imu_Data.a_z=(float)((short)(ImuDataBuffer[num+7]<<8)|ImuDataBuffer[num+6])/32768*16.0f;
		 Imu_Data.Temperature=(float)((ImuDataBuffer[num+9]<<8)|ImuDataBuffer[num+8])/(100.0f);}break;
		case 0x52:{
		 Imu_Data.W_x=(float)((short)(ImuDataBuffer[num+3]<<8)|ImuDataBuffer[num+2])/32768*2000.f;
		 Imu_Data.W_y=(float)((short)(ImuDataBuffer[num+5]<<8)|ImuDataBuffer[num+4])/32768*2000.f;
		 Imu_Data.W_z=(float)((short)(ImuDataBuffer[num+7]<<8)|ImuDataBuffer[num+6])/32768*2000.f;}break;
		 case 0x53:{
		 Imu_Data.Roll=(float)((short)(ImuDataBuffer[num+3]<<8)|ImuDataBuffer[num+2])/32768*180.f;
		 Imu_Data.Pitch=(float)(((short)ImuDataBuffer[num+5]<<8)|ImuDataBuffer[num+4])/32768*180.f;
		 Imu_Data.Yaw=(float)((short)(ImuDataBuffer[num+7]<<8)|ImuDataBuffer[num+6])/32768*180.f;}break;
	}
}
