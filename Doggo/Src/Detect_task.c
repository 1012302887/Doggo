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
//�����ж�����
void DetectTask(void *pvParameters)
{
	//����һ��ʱ��
	vTaskDelay(1800);
	while (1)
	{
		time_add_++;
		led_flicker();
		for(uint8_t i=0;i<errorListLength;i++)
		{
			//��ֹ�ڴ���������ʱ������ע��ʱ�䣬����żȻ�Ե�ǰʱ��С��ע��ʱ�䡣���ָ����������жϳ���
			if(xTaskGetTickCount()<errorList[i].regTime)
			{
				errorList[i].regTime=xTaskGetTickCount()-1;
			}
			errorList[i].Losttime=(int)(xTaskGetTickCount()-errorList[i].regTime);//��ǰʱ���ȥע��ʱ��
			if(errorList[i].Losttime>errorList[i].overtime)//ʱ�����趨��ʱʱ��
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
//�豸�������ݹ��Ӻ���
void DetectHook(uint8_t toe)//���ú���������ע��ʱ��
{
	errorList[toe].regTime= xTaskGetTickCount();
}
void DetectInit(uint16_t i,uint32_t over_time)//over_time��ʱʱ��
{
		errorList[i].regTime  = xTaskGetTickCount();
		errorList[i].overtime = over_time;
}
//�ж��Ƿ���ߣ�����1����
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
//��ˮ�ƣ��жϴ����Ƿ��ܷ�
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
