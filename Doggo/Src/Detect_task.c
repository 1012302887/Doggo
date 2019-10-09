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
/* 0:串口6断线判断  */
/* 1:陀螺仪断线判断  */
/* 2:接收机断线判断  */
/* 3:YAW轴断线判断  */
static error_t errorList[errorListLength+1];
//掉线判断任务
void DetectTask(void *pvParameters)
{
	static uint32_t time_add_;
	//空闲一段时间
	vTaskDelay(1800);
	while (1)
	{
		time_add_++;
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
		vTaskDelay(2);
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
uint8_t Detect_Judeg(uint8_t toe)
{
	if(errorList[toe].lost_flag==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
