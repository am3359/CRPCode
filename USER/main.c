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

// State machine helper macros.
#define STATE_ENTRY_ACTION                if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
#define STATE_TRANSITION_TEST             } if ( NextState == CurrentState ) {
#define STATE_TRANSITION_TEST_EXCLUSIVE   } else if ( NextState == CurrentState ) {
#define STATE_EXIT_ACTION                 } if ( NextState != CurrentState ) { CurrentState = NextState;
#define STATE_EXIT_ACTION_EXCLUSIVE       } else if ( NextState != CurrentState ) { CurrentState = NextState;
#define STATE_END                         } break;

//�������ȼ�
#define COM_TASK_PRIO        10
//�����ջ��С    
#define COM_STK_SIZE         256  
//������
TaskHandle_t Com_Task_Handler;
//������
void com_task(void *pvParameters);

//�������ȼ�
#define HMI_TASK_PRIO        9
//�����ջ��С    
#define HMI_STK_SIZE         256  
//������
TaskHandle_t HMI_Task_Handler;
//������
void hmi_task(void *pvParameters);

//�������ȼ�
#define START_TASK_PRIO        1
//�����ջ��С    
#define START_STK_SIZE         128  
//������
TaskHandle_t Start_Task_Handler;
//������
void start_task(void *pvParameters);

#define COM_Q_NUM   4       //�������ݵ���Ϣ���е����� 
#define HMI_Q_NUM   4       //�������ݵ���Ϣ���е����� 
QueueHandle_t Com_Queue;    //������Ϣ���о��
QueueHandle_t Hmi_Queue;    //HMI��Ϣ���о��

#define COM_WAIT   1       //�ȴ�com����
#define COM_INST   2       //com����(instruct)����

#define HMI_WAIT   1       //�ȴ�hmi����

//------------------MAIN ��ʼ------------------
int main(void)
{
    //����Ӳ����ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
    
    delay_init(168);  //��ʼ����ʱ����
    UART1_Init(115200);    //���ڳ�ʼ��������Ϊ115200
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
    Com_Queue=xQueueCreate(COM_Q_NUM,COM_REC_LEN); //������ϢCom_Queue,��������Ǵ��ڽ��ջ���������
    Hmi_Queue=xQueueCreate(HMI_Q_NUM,HMI_REC_LEN); //������ϢHmi_Queue,��������Ǵ��ڽ��ջ���������
    
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

//com_task������
void com_task(void *pvParameters)
{//uart1ͨѶ
    //u32 PreviousState = 0;
    //u32 CurrentState  = 0;
    //u32 NextState     = 0;
    
    u8 buffer[COM_REC_LEN];
    BaseType_t err;

    
    while(1)
    {
        if(Com_Queue!=NULL)
        {
            memset(buffer,0,COM_REC_LEN);    //���������

            err=xQueueReceive(Com_Queue,buffer,10);//���÷�����ʽ  portMAX_DELAY
            if(err==pdTRUE)
            {//�����������

    //��       Vnnb��4�ֽڣ�                 V��1�ֽڣ�
    //������� Snncpppp��8�ֽڣ�             S��1�ֽڣ�
    //�䶯��   Pnnktttt��8�ֽڣ�             P��1�ֽڣ�
    //��ת��   Rnnctttt��8�ֽڣ�             R��1�ֽڣ�
    //�¶�     Tnnb��4�ֽڣ�                 T��1�ֽڣ�
    //��ʱ     Wxxxxxxx��8�ֽڣ�             W��1�ֽڣ�
    //����     D0YYMMDD��8�ֽڣ�             D��1�ֽڣ�
    //ʱ��     N0HHMMSS��8�ֽڣ�             N��1�ֽڣ�

            //1234[V010;S01cpppp;P01ktttt;R01ctttt;T01b;D0171207;N0143500]
            //(P0130020)

            }
        }
        


        //vTaskDelay(10);
    }
}

//hmi_task������ 
void hmi_task(void *pvParameters)
{//uart3ͨѶ
    u8 buffer[HMI_REC_LEN];
    BaseType_t err;
    while(1)
    {
        if(Hmi_Queue!=NULL)
        {
            memset(buffer,0,HMI_REC_LEN);    //���������
            
            err=xQueueReceive(Hmi_Queue,buffer,portMAX_DELAY);
            if(err==pdTRUE)
            {//HMI�������
                printf("0%s0",buffer);
            }
        }

        //vTaskDelay(10);
    }
}



/*------------------MAIN ����------------------*/
