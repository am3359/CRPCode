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

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t Start_Task_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define HMI_TASK_PRIO		8
//�����ջ��С	
#define HMI_STK_SIZE 		256  
//������
TaskHandle_t HMI_Task_Handler;
//������
void hmi_task(void *pvParameters);

//�������ȼ�
#define COM_TASK_PRIO		9
//�����ջ��С	
#define COM_STK_SIZE 		256  
//������
TaskHandle_t Com_Task_Handler;
//������
void com_task(void *pvParameters);

#define COM_Q_NUM   4   	//�������ݵ���Ϣ���е����� 
#define HMI_Q_NUM   4   	//�������ݵ���Ϣ���е����� 
QueueHandle_t Com_Queue;	//������Ϣ���о��
QueueHandle_t Hmi_Queue;	//HMI��Ϣ���о��

//------------------MAIN ��ʼ------------------
int main(void)
{
	//����Ӳ����ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	
	delay_init(168);  //��ʼ����ʱ����
	UART1_Init(115200);	//���ڳ�ʼ��������Ϊ115200
	USART3_Init(115200);//9600
	
	//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&Start_Task_Handler);   //������              
    vTaskStartScheduler();    
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	
	//������Ϣ����
    Com_Queue=xQueueCreate(COM_Q_NUM,USART_REC_LEN); //������ϢCom_Queue,��������Ǵ��ڽ��ջ���������
	Hmi_Queue=xQueueCreate(HMI_Q_NUM,USART_REC_LEN); //������ϢHmi_Queue,��������Ǵ��ڽ��ջ���������
	
    //����hmi_task����
    xTaskCreate((TaskFunction_t )hmi_task,     	
                (const char*    )"hmi_task",   	
                (uint16_t       )HMI_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )HMI_TASK_PRIO,	
                (TaskHandle_t*  )&HMI_Task_Handler);   
    //����LED1����
    xTaskCreate((TaskFunction_t )com_task,     
                (const char*    )"com_task",   
                (uint16_t       )COM_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )COM_TASK_PRIO,
                (TaskHandle_t*  )&Com_Task_Handler);          
    vTaskDelete(Start_Task_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

//hmi_task������ 
void hmi_task(void *pvParameters)
{//uart3ͨѶ
	u8 buffer[USART_REC_LEN];
	BaseType_t err;
    while(1)
    {
        if(Hmi_Queue!=NULL)
        {
			memset(buffer,0,USART_REC_LEN);	//���������
			
			err=xQueueReceive(Hmi_Queue,buffer,portMAX_DELAY);
			if(err==pdTRUE)
			{//HMI�������
				//printf("0%s0",buffer);
            }
        }

		//vTaskDelay(10);
	}
}   

//com_task������
void com_task(void *pvParameters)
{//uart1ͨѶ
	u8 buffer[USART_REC_LEN];
	BaseType_t err;
    while(1)
    {
        if(Com_Queue!=NULL)
        {
			memset(buffer,0,USART_REC_LEN);	//���������
			
			err=xQueueReceive(Com_Queue,buffer,portMAX_DELAY);
			if(err==pdTRUE)
			{//�����������
				printf("Received string:%s\r\n",buffer);
            }
        }

		//vTaskDelay(10);
    }
}

/*------------------MAIN ����------------------*/
