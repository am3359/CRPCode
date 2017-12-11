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
#define HMI_Q_NUM   16      //�������ݵ���Ϣ���е����� 
QueueHandle_t Com_Queue;    //������Ϣ���о��
QueueHandle_t Hmi_Queue;    //HMI��Ϣ���о��

#define COM_WAIT   1       //�ȴ�com����
#define COM_INST   2       //com����(instruct)����

#define HMI_WAIT   1       //�ȴ�hmi����

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);

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
    
    u8 c[8][2];//��λ�úͳ���
    u8 num;//���������
    u8 cmdtype;//�������ͣ�0��Ч���ݣ�1��������2������
    
    u8 i;//,j,valid;

    
    while(1)
    {
        if(Com_Queue!=NULL)
        {
            memset(buffer,0,COM_REC_LEN);    //���������

            err=xQueueReceive(Com_Queue,buffer,10);//���÷�����ʽ  portMAX_DELAY
            if(err == pdTRUE)
            {//�����������
            //1234[V010;S01cpppp;P01ktttt;R01ctttt;T01b;D0171207;N0143500]
            //(P0130020)
//                printf("%s\r\n",buffer);
//                printf("�ַ�����:%d\r\n",str_len(buffer));
//                memset(c,0,8*2);
//                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);
//                printf("������:%d\r\n",num);
//                printf("��������:%d\r\n",cmdtype);
//                printf("����1λ��,����:%d,%d\r\n",c[0][0],c[0][1]);
//                printf("����2λ��,����:%d,%d\r\n",c[1][0],c[1][1]);
//                printf("����3λ��,����:%d,%d\r\n",c[2][0],c[2][1]);
//                printf("����4λ��,����:%d,%d\r\n",c[3][0],c[3][1]);
//                printf("����5λ��,����:%d,%d\r\n",c[4][0],c[4][1]);
//                printf("����6λ��,����:%d,%d\r\n",c[5][0],c[5][1]);
//                printf("����7λ��,����:%d,%d\r\n",c[6][0],c[6][1]);
//                printf("����8λ��,����:%d,%d\r\n",c[7][0],c[7][1]);
                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);
                if (cmdtype == 1)
                {//����������
                    for(i=0;i<num;i++)
                    {
                        switch(buffer[c[i][0]])
                        {
                            case 'd':
                            case 'D':
                                if (c[i][1] == 1)
                                {//��ѯ����
                                }
                                else if (c[i][1] == 8)
                                {//��������
                                }
                                break;
                            case 'n':
                            case 'N':
                                if (c[i][1] == 1)
                                {//��ѯʱ��
                                }
                                else if (c[i][1] == 8)
                                {//����ʱ��
                                }
                                break;
                            case 't':
                            case 'T':
                                if (c[i][1] == 1)
                                {//��ѯ�¶�
                                }
                                else if (c[i][1] == 4)
                                {//�����¶�
                                }
                                break;
                            case 'v':
                            case 'V':
                                if (c[i][1] == 1)
                                {//��ѯ��״̬
                                    //���ص�ǰ���з�״̬
                                }
                                else if (c[i][1] == 4)
                                {//���÷�����//Vnnb
                                    //for(j=1;j<3;i++)
                                    //if(c[i+j][0])
                                }
                                break;
                            case 's':
                            case 'S':
                                if (c[i][1] == 1)
                                {//��ѯ�������״̬
                                }
                                else if (c[i][1] == 8)
                                {//���ò������
                                }
                                break;
                            case 'p':
                            case 'P':
                                if (c[i][1] == 1)
                                {//��ѯ�䶯��״̬
                                }
                                else if (c[i][1] == 8)
                                {//�����䶯��
                                }
                                break;
                            case 'r':
                            case 'R':
                                if (c[i][1] == 1)
                                {//��ѯ��ת��״̬
                                }
                                else if (c[i][1] == 8)
                                {//������ת��
                                }
                                break;
                            default:
                                break;
                        }
                    }
                }
                else if (cmdtype == 2)
                {//��������
                    for(i=0;i<num;i++)
                    {
                        switch(buffer[c[i][0]])
                        {
                            case 's':
                            case 'S':
                                if (c[i][1] == 1)
                                {//��ѯ�������״̬
                                }
                                else if (c[i][1] == 8)
                                {//���ò������
                                }
                                break;
                            case 'p':
                            case 'P':
                                if (c[i][1] == 1)
                                {//��ѯ�䶯��״̬
                                }
                                else if (c[i][1] == 8)
                                {//�����䶯��
                                }
                                break;
                            case 'r':
                            case 'R':
                                if (c[i][1] == 1)
                                {//��ѯ��ת��״̬
                                }
                                else if (c[i][1] == 8)
                                {//������ת��
                                }
                                break;
                            case 'w':
                            case 'W':
                                if (c[i][1] == 1)
                                {//��ѯ��ʱ״̬
                                }
                                else if (c[i][1] == 8)
                                {//������ʱ
                                }
                                break;
                            default:
                                break;
                        }
                    }
                }
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
            if(err == pdTRUE)
            {//HMI�������
                //printf("0%s0",buffer);
            }
        }

        //vTaskDelay(10);
    }
}

/*------------------MAIN ����------------------*/
u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n)
{
    //u8 c[8][8],l[8];
    u32 result;
    u8 i,j,k,steps;
    u8 valid,end;
    //�������ݣ�
    //1 2 3 4[V010;S01cpppp;P01ktttt;R01ctttt;T01b;D0yymmdd;N0hhmmss]
    //1 2 3 4(P0130020;;;W0171207;;;;;;;abcde)
    result=0;//Ĭ��Ϊ��Ч�ַ���
    steps=0;//0���һ����ʼλ��1������ͽ���λ
    end=0;
    for(i=0;i<len;i++)
    {
        if(steps == 0)
        {
            if((buf[i] == ']') || (buf[i] == ')'))
            {//���ҵ��˽���λֱ���˳�
                //result=0;//Ĭ��Ϊ��Ч�ַ���
                //end=0;
                steps=2;
            }
            else if(buf[i] == '[')
            {//����ʼλ[
                end=']';//���������
            }
            else if(buf[i] == '(')
            {//����ʼλ(
                end=')';//���������
            }
            if(end)
            {
                j=0;//�������
                k=0;//���ٸ�����
                c[0][0]=i+1;//����λ��
                c[0][1]=0;//�����
                valid=1;//Ĭ�ϵ�ǰ������Ч
                steps=1;
            }
        }
        else if(steps == 1)
        {
            //�ҷָ���;�������]
            if(buf[i] == ';')
            {
                if(k < 8)
                {//<=8��������Ӧ;���������;
                    if((j)&&(valid))//����;���С����Сָ���1�Ͷ���,��Чָ���
                    {
                        c[k][1]=j;
                        k++;//һ����k������
                    }
                    c[k][0]=i+1;
                    c[k][1]=0;
                    j=0;//�������
                    valid=1;//Ĭ�ϵ�ǰ������Ч
                }
            }
            else if(buf[i] == end)
            {//??�ھŸ���������н�����־��ô����?? [V010;V020;V030;V040;V050;V060;V070;V080;V090;]
                if((j)&&(valid))//����;���С����Сָ���1�Ͷ���,��Чָ���
                {
                    c[k][1]=j;
                    k++;//һ����k������
                }
                if(k)
                {//�ҵ���Ӧ����λ����������һ����Ч����
                    if(end == ']')
                    {
                        result=1;
                    }
                    else if(end == ')')
                    {
                        result=2;
                    }
                }
                *n=k;
                steps=2;
            }
            else
            {
                if(buf[i] == 'D'|| buf[i] == 'd'|| buf[i] == 'N'|| buf[i] == 'n'|| \
                   buf[i] == 'P'|| buf[i] == 'p'|| buf[i] == 'R'|| buf[i] == 'r'|| \
                   buf[i] == 'S'|| buf[i] == 's'|| buf[i] == 'T'|| buf[i] == 't'|| \
                   buf[i] == 'V'|| buf[i] == 'v'|| buf[i] == 'W'|| buf[i] == 'w')
                {
                    if(j)
                    {//���������֣�������ĸ��Ч
                        valid=0;
                    }
                }
                else if(buf[i] >= '0' && buf[i] <= '9')
                {
                    if(j==0)
                    {//�������ض���ĸ��������Ч
                        valid=0;
                    }
                }
                else
                {//������ĸ������֮����ַ�����ʹ��ǰ������Ч
                    valid=0;
                }
                j++;
            }
        }
        else break;
    }
    return result;
}

u8 str_len(u8 *str)
{//�����ַ�������
    u8 i = 0;      
    while ( str[i++] != '\0')
    {
        if (i>60) break;
    }
    return i;
}
