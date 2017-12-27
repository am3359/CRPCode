#include "sys.h"
#include "delay.h"
#include "usart.h"
//#include "spi.h"
//#include "timer.h"
//#include "ad7799.h"
#include "HC595.h"
#include "stepmoto.h"

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
#define VALVE_TASK_PRIO      8
//�����ջ��С    
#define VALVE_STK_SIZE       256  
//������
TaskHandle_t Valve_Task_Handler;
//������
void valve_task(void *pvParameters);

//�������ȼ�
#define STEP_TASK_PRIO      7
//�����ջ��С    
#define STEP_STK_SIZE       256  
//������
TaskHandle_t Step_Task_Handler;
//������
void step_task(void *pvParameters);

//�������ȼ�
#define START_TASK_PRIO        1
//�����ջ��С    
#define START_STK_SIZE         128  
//������
TaskHandle_t Start_Task_Handler;
//������
void start_task(void *pvParameters);

#define COM_Q_NUM      4    //�������ݵ���Ϣ���е����� 
#define HMI_Q_NUM     16    //�������ݵ���Ϣ���е����� 

#define CMD_LEN       12    //�����������  //����Ϊ4�ֽڻ�8�ֽڻ��н���λ1�ֽ��ܳ���5��9�ֽڣ��ճ�4��������Ϊ12�ֽ�

#define VALVE_Q_NUM    8    //���ͷ������������Ϣ���е����� 
#define STEP_Q_NUM     8    //���Ͳ�����������������Ϣ���е����� 
#define PUMP_Q_NUM     8    //���ͱÿ����������Ϣ���е����� 

QueueHandle_t Com_Queue;    //������Ϣ���о��
QueueHandle_t Hmi_Queue;    //HMI��Ϣ���о��
QueueHandle_t Valve_Queue;  //��������Ϣ���о��
QueueHandle_t Step_Queue;   //�������������Ϣ���о��
QueueHandle_t Pump_Queue;   //�ÿ�����Ϣ���о��

//#define COM_WAIT   1       //�ȴ�com����
//#define COM_INST   2       //com����(instruct)����

//#define HMI_WAIT   1       //�ȴ�hmi����

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);
u32 GetPWMCnt;

//------------------MAIN ��ʼ------------------
int main(void)
{
    //����Ӳ����ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
    
    delay_init(168);  //��ʼ����ʱ����
    UART1_Init(115200);    //���ڳ�ʼ��������Ϊ115200
    USART3_Init(115200);//9600
    
//  TIM3_Init();
    
    HC595Init();
    StepMotoInit();


    
//  TIM_Cmd(TIM3, DISABLE);
//  GetPWMCnt  = TIM_GetCounter(TIM3);
//  TIM_SetCounter(TIM3, 0); 
//  TIM_Cmd(TIM3, ENABLE);
    
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
    Com_Queue=xQueueCreate(COM_Q_NUM,COM_REC_LEN); //������ϢCom_Queue
    Hmi_Queue=xQueueCreate(HMI_Q_NUM,HMI_REC_LEN); //������ϢHmi_Queue
    
    Valve_Queue=xQueueCreate(VALVE_Q_NUM,CMD_LEN); //������ϢValve_Queue
    Step_Queue=xQueueCreate(STEP_Q_NUM,CMD_LEN); //������ϢValve_Queue
    //Pump_Queue=xQueueCreate(PUMP_Q_NUM,CMD_LEN); //������ϢValve_Queue

    //����com_task����
    xTaskCreate((TaskFunction_t )com_task,
                (const char*    )"com_task",
                (uint16_t       )COM_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )COM_TASK_PRIO,
                (TaskHandle_t*  )&Com_Task_Handler);
    //����hmi_task����
    xTaskCreate((TaskFunction_t )hmi_task,
                (const char*    )"hmi_task",
                (uint16_t       )HMI_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )HMI_TASK_PRIO,
                (TaskHandle_t*  )&HMI_Task_Handler);
    //����valve_task����
    xTaskCreate((TaskFunction_t )valve_task,
                (const char*    )"valve_task",
                (uint16_t       )VALVE_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )VALVE_TASK_PRIO,
                (TaskHandle_t*  )&Valve_Task_Handler);
    //����step_task����
    xTaskCreate((TaskFunction_t )step_task,
                (const char*    )"step_task",
                (uint16_t       )STEP_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )STEP_TASK_PRIO,
                (TaskHandle_t*  )&Step_Task_Handler);
               
    vTaskDelete(Start_Task_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
//u8 x;
//com_task������
void com_task(void *pvParameters)
{//uart1ͨѶ

    u8 buffer[COM_REC_LEN];
    BaseType_t err;
    u8 command[12];//����Ϊ4�ֽڻ�8�ֽڻ��н���λ1�ֽ��ܳ���5��9�ֽڣ��ճ�4��������Ϊ12�ֽ�
    
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
                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);//�����ַ������жϳ���Ч����λ�ó��Ⱥ͸�������������cmdtype��0��Ч���ݣ�1��������2������
                if (cmdtype == 1)
                {//����������
                    for(i=0;i<num;i++)
                    {
                        switch(buffer[c[i][0]])
                        {
                            case 'D':
                                if (c[i][1] == 1)
                                {//��ѯ����
                                }
                                else if (c[i][1] == 8)
                                {//��������
                                }
                                break;
                            case 'N':
                                if (c[i][1] == 1)
                                {//��ѯʱ��
                                }
                                else if (c[i][1] == 8)
                                {//����ʱ��
                                }
                                break;
                            case 'T':
                                if (c[i][1] == 1)
                                {//��ѯ�¶�
                                }
                                else if (c[i][1] == 4)
                                {//�����¶�
                                }
                                break;
                            case 'V':
                                if ((c[i][1] == 1) || (c[i][1] == 4))
                                {//��ѯ��״̬  //���÷�����//Vnnb
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='1';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Valve_Queue,command,10);//��Valve_Queue�����з�������
                                }
                                break;
                            case 'S':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//��ѯ�������״̬  //���ò������
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='1';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Step_Queue,command,10);//��Step_Queue�����з�������
                                }
                                break;
                            case 'P':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//��ѯ�䶯��״̬  //�����䶯��

                                }
                                break;
                            case 'R':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//��ѯ��ת��״̬  //������ת��
                                    
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
                            case 'S':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//��ѯ�������״̬  //���ò������
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='2';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Step_Queue,command,10);//��Step_Queue�����з�������
                                }
                                break;
                            case 'P':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//��ѯ�䶯��״̬  //�����䶯��

                                }
                                break;
                            case 'R':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//��ѯ��ת��״̬  //������ת��
                                    
                                }
                                break;
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

//valve_task������ 
void valve_task(void *pvParameters)
{
    u8 buffer[CMD_LEN];
    BaseType_t err;
    static u32 valve_state;
    u8 bits;
    
    u32 PreviousState = 0;
    u32 CurrentState  = 0;
    u32 NextState     = 0;
    
    while(1)
    {
        switch(CurrentState)
        {
//            case 0:
//                CurrentState = COM_WAIT;
//                NextState    = COM_WAIT;
//                break;
            case 1:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(Valve_Queue!=NULL)
                    {
                        memset(buffer,0,CMD_LEN);    //���������

                        err=xQueueReceive(Valve_Queue,buffer,10);//���÷�����ʽ  portMAX_DELAY
                        if(err == pdTRUE)
                        {//ִ�з���������
                            printf("����������:%s\r\n",buffer);
                            //V090
                            bits=(buffer[1]-'0')*10+buffer[2]-'0';
                            if (bits)//bits>0��Ӧ�ķ�����
                            {
                                if(buffer[3]=='0')
                                    valve_state &= ~(1<<(bits-1));
                                else
                                    valve_state |= 1<<(bits-1);
                            }
                            printf("��״̬:%#x\r\n",valve_state);
                            SwitchOut(valve_state);
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
//            case 2:
//                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

//                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

//                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

//                STATE_END                                                       //} break;
            default:
                CurrentState = 1;
                NextState    = 1;
                valve_state = 0;
                printf("��״̬:%#x\r\n",valve_state);
                SwitchOut(valve_state);
                break;

        }
        
    }
}

//step_task������ 
void step_task(void *pvParameters)
{
    u8 buffer[CMD_LEN];
    BaseType_t err;
    s32 steps;
    u8 num;

    while(1)
    {
        if(Step_Queue!=NULL)
        {
            memset(buffer,0,CMD_LEN);    //���������

            err=xQueueReceive(Step_Queue,buffer,10);//���÷�����ʽ  portMAX_DELAY
            if(err == pdTRUE)
            {//ִ�в��������������
                printf("���������������:%s\r\n",buffer);
                num=(buffer[1]-'0')*10+buffer[2]-'0';
                switch(num)
                {
                    case 1:
                        steps = (buffer[5]-'0')*100+(buffer[6]-'0')*10+buffer[7]-'0';
                        if((buffer[3]-'0') >= 4) steps=-steps;
                        StepMoto1Move(steps);
//  vTaskDelay(2000);
//  TIM_Cmd(TIM3, DISABLE);
//  GetPWMCnt  = TIM_GetCounter(TIM3);
//  TIM_SetCounter(TIM3, 0); 
//  TIM_Cmd(TIM3, ENABLE);
//  printf("ʵ�ʲ���:%d\r\n",GetPWMCnt);
                        break;
                    default:
                        break;
                }
                
            }
        }
    }
}

/*------------------MAIN ����------------------*/
//�����ַ����е���Ч���������ĸ��ɴ�д
//buf[]�������ַ�����len���ַ�������
//c[8][2]�������Ч������ʼλ�úͳ��ȣ�n�������Ч������
//c[8][2]һά���鳤��>=8��������
//����ֵ��0��Ч���ݣ�1��������2������
u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n)
{
    u32 result;
    u8 i,j,k,steps;
    u8 valid,end;
    result=0;//Ĭ��Ϊ��Ч�ַ���
    steps=0;//0���һ����ʼλ��1������ͽ���λ
    end=0;
    k=0;//���ٸ�����
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
            {//<=8��������Ӧ;�������;
                if(k < 7)
                {//���滹����������
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
                else if(k == 7)
                {//�����������
                    if((j)&&(valid))//����;���С����Сָ���1�Ͷ���,��Чָ���
                    {
                        c[k][1]=j;
                        k++;//һ����k������
                    }
                }
            }
            else if(buf[i] == end)
            {
                if(k <= 7)
                {//<=8��������Ӧ;�������;
                    if((j)&&(valid))//����;���С����Сָ���1�Ͷ���,��Чָ���
                    {
                        c[k][1]=j;
                        k++;//һ����k������
                    }
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
                //*n=k;
                steps=2;
            }
            else
            {
                if(buf[i] >= 'a' && buf[i] <= 'z')
                {//ת�ɴ�д
                    buf[i]+='A'-'a';
                }
                if(buf[i] == 'D'|| buf[i] == 'N'|| buf[i] == 'P'|| buf[i] == 'R'||
                   buf[i] == 'S'|| buf[i] == 'T'|| buf[i] == 'V'|| buf[i] == 'W')
                {
                    if(j)
                    {//������������֣�������ĸ��Ч
                        valid=0;
                    }
                }
                else if(buf[i] >= '0' && buf[i] <= '9')
                {
                    if(j==0)
                    {//��ͷ�������ض���ĸ��������Ч
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
        else
        {
            break;
        }
    }
    *n=k;
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
//    while(str[i])
//    {
//        ++i;
//        if (i>60) break;
//    }

//    return i;
}
