#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Start_Task.h"
#include "Position_Control.h"
#include "Detect_Task.h"
#define PositionControl_PRIO 17
#define PositionControl_SIZE 512

#define START_TASK_PRIO 1
#define START_STK_SIZE 256

#define Detect_TASK_PRIO 10
#define Detect_STK_SIZE 256
static TaskHandle_t PositionControlHandle;
static TaskHandle_t StartTask_Handler;
static TaskHandle_t DetectTask_Handler;
void Start_Task(void *pvParameters);
void Start_Task(void *pvParameters)
{
    taskENTER_CRITICAL();
		xTaskCreate((TaskFunction_t)PositionControlThread,
                (const char *)"PositionControlThread",
                (uint16_t)PositionControl_SIZE,
                (void *)NULL,
                (UBaseType_t)PositionControl_PRIO,
                (TaskHandle_t *)&PositionControlHandle);
    xTaskCreate((TaskFunction_t)DetectTask,
                (const char *)"DetectTask",
                (uint16_t)Detect_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Detect_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);
		vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
void start_task(void)
{
    xTaskCreate((TaskFunction_t)Start_Task,          //������
                (const char *)"Start_Task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}
