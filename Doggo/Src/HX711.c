/************************************************************************************
							�������ṩ�����µ��̣�
								Ilovemcu.taobao.com
								epic-mcu.taobao.com
							ʵ�������Χ��չģ����������ϵ���
							���ߣ����زر���							
*************************************************************************************/
#include "HX711.h"
#include "main.h"
// ����ƽ���˲������ֳƻ���ƽ���˲�����
#define FILTER_N 12
float filter_buf[FILTER_N + 1];

uint32_t HX711_Buffer;
uint32_t Weight_Maopi;
uint32_t Weight_Shiwu;
uint8_t Flag_Error = 0;

//У׼����
//��Ϊ��ͬ�Ĵ������������߲��Ǻ�һ�£���ˣ�ÿһ����������Ҫ�������������������ʹ����ֵ��׼ȷ��
//�����ֲ��Գ���������ƫ��ʱ�����Ӹ���ֵ��
//������Գ���������ƫСʱ����С����ֵ��
//��ֵ����ΪС��
#define GapValue 370


void Init_HX711pin(void){
		//HX711_SCK
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		//HX711_DOUT
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
}
//****************************************************
//��ȡHX711
//****************************************************
static unsigned long count; 
static unsigned char i; 
extern void delay_us(uint16_t us);
uint32_t HX711_Read(void)        //����128
{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
          count=0; 
           delay_us(1);
          while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)); 
          for(i=0;i<24;i++)
        { 
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
						count=count<<1; 
						delay_us(1);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET); 
						if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7))
							{count++;} 
						delay_us(1);
        } 
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
				count=count^0x800000;//��25�������½�����ʱ��ת������
				delay_us(1);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
        return(count);
}

//****************************************************
//��ȡëƤ����
//****************************************************
void Get_Maopi(void)
{
	Weight_Maopi = HX711_Read();	
} 

//****************************************************
//����
//****************************************************
uint32_t Get_Weight(void)
{
	HX711_Buffer = HX711_Read();
	if((HX711_Buffer-Weight_Maopi)/GapValue>5000)
	{
		return 0;
	}		
	if(HX711_Buffer > Weight_Maopi)			
	{
		Weight_Shiwu = HX711_Buffer;
		Weight_Shiwu = Weight_Shiwu - Weight_Maopi;				//��ȡʵ���AD������ֵ��
	
		Weight_Shiwu = (int32_t)(Weight_Shiwu/GapValue); 	//����ʵ���ʵ������
																		//��Ϊ��ͬ�Ĵ������������߲�һ������ˣ�ÿһ����������Ҫ���������GapValue���������
																		//�����ֲ��Գ���������ƫ��ʱ�����Ӹ���ֵ��
																		//������Գ���������ƫСʱ����С����ֵ��
		return Weight_Shiwu;
	}
	return 0;
}
float HX711_Filter(void) 
{
	int i;
	int filter_sum = 0;
	filter_buf[FILTER_N] = Get_Weight(); //ADת����ֵ�����������һ��ֵ
	for(i = 0; i < FILTER_N; i++) 
	{
		filter_buf[i] = filter_buf[i + 1]; // �����������ƣ���λ����
		filter_sum += filter_buf[i];
	}
	return (float)(filter_sum / FILTER_N);
} 
