#include "rc.h"
#include "stm32f4xx.h"
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
void RC_Init(uint8_t *rx1_buf,uint16_t dma_buf_num)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);//¿ªÆô´®¿Ú¿ÕÏÐÖÐ¶Ï.
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx1_buf, dma_buf_num);
  /* USER CODE END USART2_Init 2 */
}
