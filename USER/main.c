#include "sys.h"
#include "delay.h"
#include "usart.h"
//#include "spi.h"
//#include "timer.h"
//#include "moto.h"
//#include "pwm.h"
//#include "ad7799.h"
//#include "DRV8825.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t Start_Task_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define HMI_TASK_PRIO		8
//任务堆栈大小	
#define HMI_STK_SIZE 		256  
//任务句柄
TaskHandle_t HMI_Task_Handler;
//任务函数
void hmi_task(void *pvParameters);

//任务优先级
#define COM_TASK_PRIO		9
//任务堆栈大小	
#define COM_STK_SIZE 		256  
//任务句柄
TaskHandle_t Com_Task_Handler;
//任务函数
void com_task(void *pvParameters);

#define COM_Q_NUM   4   	//发送数据的消息队列的数量 
#define HMI_Q_NUM   4   	//发送数据的消息队列的数量 
QueueHandle_t Com_Queue;	//串口信息队列句柄
QueueHandle_t Hmi_Queue;	//HMI信息队列句柄

//------------------MAIN 开始------------------
int main(void)
{
	//所有硬件初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	
	delay_init(168);  //初始化延时函数
	UART1_Init(115200);	//串口初始化波特率为115200
	USART3_Init(115200);//9600
	
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&Start_Task_Handler);   //任务句柄              
    vTaskStartScheduler();    
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	//创建消息队列
    Com_Queue=xQueueCreate(COM_Q_NUM,USART_REC_LEN); //创建消息Com_Queue,队列项长度是串口接收缓冲区长度
	Hmi_Queue=xQueueCreate(HMI_Q_NUM,USART_REC_LEN); //创建消息Hmi_Queue,队列项长度是串口接收缓冲区长度
	
    //创建hmi_task任务
    xTaskCreate((TaskFunction_t )hmi_task,     	
                (const char*    )"hmi_task",   	
                (uint16_t       )HMI_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )HMI_TASK_PRIO,	
                (TaskHandle_t*  )&HMI_Task_Handler);   
    //创建LED1任务
    xTaskCreate((TaskFunction_t )com_task,     
                (const char*    )"com_task",   
                (uint16_t       )COM_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )COM_TASK_PRIO,
                (TaskHandle_t*  )&Com_Task_Handler);          
    vTaskDelete(Start_Task_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//hmi_task任务函数 
void hmi_task(void *pvParameters)
{//uart3通讯
	u8 buffer[USART_REC_LEN];
	BaseType_t err;
    while(1)
    {
        if(Hmi_Queue!=NULL)
        {
			memset(buffer,0,USART_REC_LEN);	//清除缓冲区
			
			err=xQueueReceive(Hmi_Queue,buffer,portMAX_DELAY);
			if(err==pdTRUE)
			{//HMI命令解析
				//printf("0%s0",buffer);
            }
        }

		//vTaskDelay(10);
	}
}   

//com_task任务函数
void com_task(void *pvParameters)
{//uart1通讯
	u8 buffer[USART_REC_LEN];
	BaseType_t err;
    while(1)
    {
        if(Com_Queue!=NULL)
        {
			memset(buffer,0,USART_REC_LEN);	//清除缓冲区
			
			err=xQueueReceive(Com_Queue,buffer,portMAX_DELAY);
			if(err==pdTRUE)
			{//串口命令解析
				printf("Received string:%s\r\n",buffer);
            }
        }

		//vTaskDelay(10);
    }
}

/*------------------MAIN 结束------------------*/
