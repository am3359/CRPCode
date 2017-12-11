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

//任务优先级
#define COM_TASK_PRIO        10
//任务堆栈大小    
#define COM_STK_SIZE         256  
//任务句柄
TaskHandle_t Com_Task_Handler;
//任务函数
void com_task(void *pvParameters);

//任务优先级
#define HMI_TASK_PRIO        9
//任务堆栈大小    
#define HMI_STK_SIZE         256  
//任务句柄
TaskHandle_t HMI_Task_Handler;
//任务函数
void hmi_task(void *pvParameters);

//任务优先级
#define START_TASK_PRIO        1
//任务堆栈大小    
#define START_STK_SIZE         128  
//任务句柄
TaskHandle_t Start_Task_Handler;
//任务函数
void start_task(void *pvParameters);

#define COM_Q_NUM   4       //发送数据的消息队列的数量 
#define HMI_Q_NUM   16      //发送数据的消息队列的数量 
QueueHandle_t Com_Queue;    //串口信息队列句柄
QueueHandle_t Hmi_Queue;    //HMI信息队列句柄

#define COM_WAIT   1       //等待com命令
#define COM_INST   2       //com命令(instruct)解析

#define HMI_WAIT   1       //等待hmi命令

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);

//------------------MAIN 开始------------------
int main(void)
{
    //所有硬件初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
    
    delay_init(168);  //初始化延时函数
    UART1_Init(115200);    //串口初始化波特率为115200
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
    Com_Queue=xQueueCreate(COM_Q_NUM,COM_REC_LEN); //创建消息Com_Queue,队列项长度是串口接收缓冲区长度
    Hmi_Queue=xQueueCreate(HMI_Q_NUM,HMI_REC_LEN); //创建消息Hmi_Queue,队列项长度是串口接收缓冲区长度
    
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

//com_task任务函数
void com_task(void *pvParameters)
{//uart1通讯
    //u32 PreviousState = 0;
    //u32 CurrentState  = 0;
    //u32 NextState     = 0;
    
    u8 buffer[COM_REC_LEN];
    BaseType_t err;
    
    u8 c[8][2];//存位置和长度
    u8 num;//存命令个数
    u8 cmdtype;//命令类型，0无效数据，1非阻塞，2阻塞型
    
    u8 i;//,j,valid;

    
    while(1)
    {
        if(Com_Queue!=NULL)
        {
            memset(buffer,0,COM_REC_LEN);    //清除缓冲区

            err=xQueueReceive(Com_Queue,buffer,10);//采用非阻塞式  portMAX_DELAY
            if(err == pdTRUE)
            {//串口命令解析
            //1234[V010;S01cpppp;P01ktttt;R01ctttt;T01b;D0171207;N0143500]
            //(P0130020)
//                printf("%s\r\n",buffer);
//                printf("字符长度:%d\r\n",str_len(buffer));
//                memset(c,0,8*2);
//                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);
//                printf("命令数:%d\r\n",num);
//                printf("命令类型:%d\r\n",cmdtype);
//                printf("命令1位置,长度:%d,%d\r\n",c[0][0],c[0][1]);
//                printf("命令2位置,长度:%d,%d\r\n",c[1][0],c[1][1]);
//                printf("命令3位置,长度:%d,%d\r\n",c[2][0],c[2][1]);
//                printf("命令4位置,长度:%d,%d\r\n",c[3][0],c[3][1]);
//                printf("命令5位置,长度:%d,%d\r\n",c[4][0],c[4][1]);
//                printf("命令6位置,长度:%d,%d\r\n",c[5][0],c[5][1]);
//                printf("命令7位置,长度:%d,%d\r\n",c[6][0],c[6][1]);
//                printf("命令8位置,长度:%d,%d\r\n",c[7][0],c[7][1]);
                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);
                if (cmdtype == 1)
                {//非阻塞命令
                    for(i=0;i<num;i++)
                    {
                        switch(buffer[c[i][0]])
                        {
                            case 'd':
                            case 'D':
                                if (c[i][1] == 1)
                                {//查询日期
                                }
                                else if (c[i][1] == 8)
                                {//设置日期
                                }
                                break;
                            case 'n':
                            case 'N':
                                if (c[i][1] == 1)
                                {//查询时间
                                }
                                else if (c[i][1] == 8)
                                {//设置时间
                                }
                                break;
                            case 't':
                            case 'T':
                                if (c[i][1] == 1)
                                {//查询温度
                                }
                                else if (c[i][1] == 4)
                                {//设置温度
                                }
                                break;
                            case 'v':
                            case 'V':
                                if (c[i][1] == 1)
                                {//查询阀状态
                                    //返回当前所有阀状态
                                }
                                else if (c[i][1] == 4)
                                {//设置阀开关//Vnnb
                                    //for(j=1;j<3;i++)
                                    //if(c[i+j][0])
                                }
                                break;
                            case 's':
                            case 'S':
                                if (c[i][1] == 1)
                                {//查询步进电机状态
                                }
                                else if (c[i][1] == 8)
                                {//设置步进电机
                                }
                                break;
                            case 'p':
                            case 'P':
                                if (c[i][1] == 1)
                                {//查询蠕动泵状态
                                }
                                else if (c[i][1] == 8)
                                {//设置蠕动泵
                                }
                                break;
                            case 'r':
                            case 'R':
                                if (c[i][1] == 1)
                                {//查询旋转泵状态
                                }
                                else if (c[i][1] == 8)
                                {//设置旋转泵
                                }
                                break;
                            default:
                                break;
                        }
                    }
                }
                else if (cmdtype == 2)
                {//阻塞命令
                    for(i=0;i<num;i++)
                    {
                        switch(buffer[c[i][0]])
                        {
                            case 's':
                            case 'S':
                                if (c[i][1] == 1)
                                {//查询步进电机状态
                                }
                                else if (c[i][1] == 8)
                                {//设置步进电机
                                }
                                break;
                            case 'p':
                            case 'P':
                                if (c[i][1] == 1)
                                {//查询蠕动泵状态
                                }
                                else if (c[i][1] == 8)
                                {//设置蠕动泵
                                }
                                break;
                            case 'r':
                            case 'R':
                                if (c[i][1] == 1)
                                {//查询旋转泵状态
                                }
                                else if (c[i][1] == 8)
                                {//设置旋转泵
                                }
                                break;
                            case 'w':
                            case 'W':
                                if (c[i][1] == 1)
                                {//查询延时状态
                                }
                                else if (c[i][1] == 8)
                                {//设置延时
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

//hmi_task任务函数 
void hmi_task(void *pvParameters)
{//uart3通讯
    u8 buffer[HMI_REC_LEN];
    BaseType_t err;
    while(1)
    {
        if(Hmi_Queue!=NULL)
        {
            memset(buffer,0,HMI_REC_LEN);    //清除缓冲区
            
            err=xQueueReceive(Hmi_Queue,buffer,portMAX_DELAY);
            if(err == pdTRUE)
            {//HMI命令解析
                //printf("0%s0",buffer);
            }
        }

        //vTaskDelay(10);
    }
}

/*------------------MAIN 结束------------------*/
u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n)
{
    //u8 c[8][8],l[8];
    u32 result;
    u8 i,j,k,steps;
    u8 valid,end;
    //测试数据：
    //1 2 3 4[V010;S01cpppp;P01ktttt;R01ctttt;T01b;D0yymmdd;N0hhmmss]
    //1 2 3 4(P0130020;;;W0171207;;;;;;;abcde)
    result=0;//默认为无效字符串
    steps=0;//0查第一个起始位，1查命令和结束位
    end=0;
    for(i=0;i<len;i++)
    {
        if(steps == 0)
        {
            if((buf[i] == ']') || (buf[i] == ')'))
            {//先找到了结束位直接退出
                //result=0;//默认为无效字符串
                //end=0;
                steps=2;
            }
            else if(buf[i] == '[')
            {//找起始位[
                end=']';//保存结束符
            }
            else if(buf[i] == '(')
            {//找起始位(
                end=')';//保存结束符
            }
            if(end)
            {
                j=0;//单个命令长
                k=0;//多少个命令
                c[0][0]=i+1;//命令位置
                c[0][1]=0;//命令长度
                valid=1;//默认当前命令有效
                steps=1;
            }
        }
        else if(steps == 1)
        {
            //找分隔符;或结束符]
            if(buf[i] == ';')
            {
                if(k < 8)
                {//<=8个命令响应;，否则忽略;
                    if((j)&&(valid))//两个;间隔小于最小指令长度1就丢弃,无效指令丢弃
                    {
                        c[k][1]=j;
                        k++;//一共有k个命令
                    }
                    c[k][0]=i+1;
                    c[k][1]=0;
                    j=0;//单个命令长
                    valid=1;//默认当前命令有效
                }
            }
            else if(buf[i] == end)
            {//??第九个命令后面有结束标志怎么处理?? [V010;V020;V030;V040;V050;V060;V070;V080;V090;]
                if((j)&&(valid))//两个;间隔小于最小指令长度1就丢弃,无效指令丢弃
                {
                    c[k][1]=j;
                    k++;//一共有k个命令
                }
                if(k)
                {//找到对应结束位并且至少有一条有效命令
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
                    {//必须是数字，特殊字母无效
                        valid=0;
                    }
                }
                else if(buf[i] >= '0' && buf[i] <= '9')
                {
                    if(j==0)
                    {//必须是特定字母，数字无效
                        valid=0;
                    }
                }
                else
                {//出现字母和数字之外的字符都会使当前命令无效
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
{//计算字符串长度
    u8 i = 0;      
    while ( str[i++] != '\0')
    {
        if (i>60) break;
    }
    return i;
}
