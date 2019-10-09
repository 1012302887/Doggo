/************************************************************************************
							本例程提供自以下店铺：
								Ilovemcu.taobao.com
								epic-mcu.taobao.com
							实验相关外围扩展模块均来自以上店铺
							作者：神秘藏宝室							
*************************************************************************************/
#include "HX711.h"
#include "main.h"
// 递推平均滤波法（又称滑动平均滤波法）
#define FILTER_N 12
float filter_buf[FILTER_N + 1];

uint32_t HX711_Buffer;
uint32_t Weight_Maopi;
uint32_t Weight_Shiwu;
uint8_t Flag_Error = 0;

//校准参数
//因为不同的传感器特性曲线不是很一致，因此，每一个传感器需要矫正这里这个参数才能使测量值很准确。
//当发现测试出来的重量偏大时，增加该数值。
//如果测试出来的重量偏小时，减小改数值。
//该值可以为小数
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
//读取HX711
//****************************************************
static unsigned long count; 
static unsigned char i; 
extern void delay_us(uint16_t us);
uint32_t HX711_Read(void)        //增益128
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
				count=count^0x800000;//第25个脉冲下降沿来时，转换数据
				delay_us(1);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
        return(count);
}

//****************************************************
//获取毛皮重量
//****************************************************
void Get_Maopi(void)
{
	Weight_Maopi = HX711_Read();	
} 

//****************************************************
//称重
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
		Weight_Shiwu = Weight_Shiwu - Weight_Maopi;				//获取实物的AD采样数值。
	
		Weight_Shiwu = (int32_t)(Weight_Shiwu/GapValue); 	//计算实物的实际重量
																		//因为不同的传感器特性曲线不一样，因此，每一个传感器需要矫正这里的GapValue这个除数。
																		//当发现测试出来的重量偏大时，增加该数值。
																		//如果测试出来的重量偏小时，减小改数值。
		return Weight_Shiwu;
	}
	return 0;
}
float HX711_Filter(void) 
{
	int i;
	int filter_sum = 0;
	filter_buf[FILTER_N] = Get_Weight(); //AD转换的值赋给数组最后一个值
	for(i = 0; i < FILTER_N; i++) 
	{
		filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位丢掉
		filter_sum += filter_buf[i];
	}
	return (float)(filter_sum / FILTER_N);
} 
