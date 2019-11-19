/*
quu..__
 $$$b  `---.__
	"$$b        `--.                          ___.---uuudP
	 `$$b           `.__.------.__     __.---'      $$$$"              .
		 "$b          -'            `-.-'            $$$"              .'|
			 ".                                       d$"             _.'  |
				 `.   /                              ..."             .'     |
					 `./                           ..::-'            _.'       |
						/                         .:::-'            .-'         .'
					 :                          ::''\          _.'            |
					.' .-.             .-.           `.      .'               |
					: /'$$|           .@"$\           `.   .'              _.-'
				 .'|$u$$|          |$$,$$|           |  <            _.-'
				 | `:$$:'          :$$$$$:           `.  `.       .-'
				 :                  `"--'             |    `-.     \
				:##.       ==             .###.       `.      `.    `\
				|##:                      :###:        |        >     >
				|#'     `..'`..'          `###'        x:      /     /
				 \                                   xXX|     /    ./
					\                                xXXX'|    /   ./
					/`-.                                  `.  /   /
				 :    `-  ...........,                   | /  .'
				 |         ``:::::::'       .            |<    `.
				 |             ```          |           x| \ `.:``.
				 |                         .'    /'   xXX|  `:`M`M':.
				 |    |                    ;    /:' xXXX'|  -'MMMMM:'
				 `.  .'                   :    /:'       |-'MMMM.-'
					|  |                   .'   /'        .'MMM.-'
					`'`'                   :  ,'          |MMM<
						|                     `'            |tbap\
						 \                                  :MM.-'
							\                 |              .''
							 \.               `.            /
								/     .:::::::.. :           /
							 |     .:::::::::::`.         /
							 |   .:::------------\       /
							/   .''               >::'  /
							`',:                 :    .'
																	 `:.:'
 
*/
#include "FreeRTOS.h"
#include "task.h"
#include "Detect_Task.h"
#include "can_receive.h"
#include "arm_math.h"

#define errorListLength 6
static error_t errorList[errorListLength+1];
static uint32_t time_add_;
static void led_flicker(void);
//掉线判断任务
void DetectTask(void *pvParameters)
{
	//空闲一段时间
	vTaskDelay(1800);
	while (1)
	{
		time_add_++;
		led_flicker();
		for(uint8_t i=0;i<errorListLength;i++)
		{
			//防止在此任务运行时，正好注册时间，出现偶然性当前时间小于注册时间。出现负数，导致判断出错
			if(xTaskGetTickCount()<errorList[i].regTime)
			{
				errorList[i].regTime=xTaskGetTickCount()-1;
			}
			errorList[i].Losttime=(int)(xTaskGetTickCount()-errorList[i].regTime);//当前时间减去注册时间
			if(errorList[i].Losttime>errorList[i].overtime)//时间差超过设定超时时间
			{
					errorList[i].OFF_TIME++;
					if(errorList[i].OFF_TIME>500)
					{
						errorList[i].OFF_TIME=0;
						errorList[i].lost_flag=1;
					}
			}
			else
			{
				errorList[i].OFF_TIME=0;
				errorList[i].lost_flag=0;
			}
		}
		vTaskDelay(100);
	}
}
//设备接收数据钩子函数
void DetectHook(uint8_t toe)//调用函数，更行注册时间
{
	errorList[toe].regTime= xTaskGetTickCount();
}
void DetectInit(uint16_t i,uint32_t over_time)//over_time超时时间
{
		errorList[i].regTime  = xTaskGetTickCount();
		errorList[i].overtime = over_time;
}
//判断是否掉线，返回1掉线
uint8_t Detect_Judeg(uint8_t toe){
	if(errorList[toe].lost_flag==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//流水灯，判断代码是否跑飞
static void led_flicker(void){
	if((time_add_%10 == 0)  && (time_add_%20 != 0))
	{
		TIM9->CCR1 = 1000;
		TIM9->CCR2 = 0;
	}
	else if(time_add_%20 == 0)
	{
		TIM9->CCR1 = 0;
		TIM9->CCR2 = 1000;
	}
}
