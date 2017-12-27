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
#define VALVE_TASK_PRIO      8
//任务堆栈大小    
#define VALVE_STK_SIZE       256  
//任务句柄
TaskHandle_t Valve_Task_Handler;
//任务函数
void valve_task(void *pvParameters);

//任务优先级
#define STEP_TASK_PRIO      7
//任务堆栈大小    
#define STEP_STK_SIZE       256  
//任务句柄
TaskHandle_t Step_Task_Handler;
//任务函数
void step_task(void *pvParameters);

//任务优先级
#define START_TASK_PRIO        1
//任务堆栈大小    
#define START_STK_SIZE         128  
//任务句柄
TaskHandle_t Start_Task_Handler;
//任务函数
void start_task(void *pvParameters);

#define COM_Q_NUM      4    //发送数据的消息队列的数量 
#define HMI_Q_NUM     16    //发送数据的消息队列的数量 

#define CMD_LEN       12    //阀控制命令长度  //命令为4字节或8字节还有结束位1字节总长度5或9字节，凑成4的整数倍为12字节

#define VALVE_Q_NUM    8    //发送阀控制命令的消息队列的数量 
#define STEP_Q_NUM     8    //发送步进电机控制命令的消息队列的数量 
#define PUMP_Q_NUM     8    //发送泵控制命令的消息队列的数量 

QueueHandle_t Com_Queue;    //串口消息队列句柄
QueueHandle_t Hmi_Queue;    //HMI消息队列句柄
QueueHandle_t Valve_Queue;  //阀控制消息队列句柄
QueueHandle_t Step_Queue;   //步进电机控制消息队列句柄
QueueHandle_t Pump_Queue;   //泵控制消息队列句柄

//#define COM_WAIT   1       //等待com命令
//#define COM_INST   2       //com命令(instruct)解析

//#define HMI_WAIT   1       //等待hmi命令

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);
u32 GetPWMCnt;

//------------------MAIN 开始------------------
int main(void)
{
    //所有硬件初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
    
    delay_init(168);  //初始化延时函数
    UART1_Init(115200);    //串口初始化波特率为115200
    USART3_Init(115200);//9600
    
//  TIM3_Init();
    
    HC595Init();
    StepMotoInit();


    
//  TIM_Cmd(TIM3, DISABLE);
//  GetPWMCnt  = TIM_GetCounter(TIM3);
//  TIM_SetCounter(TIM3, 0); 
//  TIM_Cmd(TIM3, ENABLE);
    
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
    Com_Queue=xQueueCreate(COM_Q_NUM,COM_REC_LEN); //创建消息Com_Queue
    Hmi_Queue=xQueueCreate(HMI_Q_NUM,HMI_REC_LEN); //创建消息Hmi_Queue
    
    Valve_Queue=xQueueCreate(VALVE_Q_NUM,CMD_LEN); //创建消息Valve_Queue
    Step_Queue=xQueueCreate(STEP_Q_NUM,CMD_LEN); //创建消息Valve_Queue
    //Pump_Queue=xQueueCreate(PUMP_Q_NUM,CMD_LEN); //创建消息Valve_Queue

    //创建com_task任务
    xTaskCreate((TaskFunction_t )com_task,
                (const char*    )"com_task",
                (uint16_t       )COM_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )COM_TASK_PRIO,
                (TaskHandle_t*  )&Com_Task_Handler);
    //创建hmi_task任务
    xTaskCreate((TaskFunction_t )hmi_task,
                (const char*    )"hmi_task",
                (uint16_t       )HMI_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )HMI_TASK_PRIO,
                (TaskHandle_t*  )&HMI_Task_Handler);
    //创建valve_task任务
    xTaskCreate((TaskFunction_t )valve_task,
                (const char*    )"valve_task",
                (uint16_t       )VALVE_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )VALVE_TASK_PRIO,
                (TaskHandle_t*  )&Valve_Task_Handler);
    //创建step_task任务
    xTaskCreate((TaskFunction_t )step_task,
                (const char*    )"step_task",
                (uint16_t       )STEP_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )STEP_TASK_PRIO,
                (TaskHandle_t*  )&Step_Task_Handler);
               
    vTaskDelete(Start_Task_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}
//u8 x;
//com_task任务函数
void com_task(void *pvParameters)
{//uart1通讯

    u8 buffer[COM_REC_LEN];
    BaseType_t err;
    u8 command[12];//命令为4字节或8字节还有结束位1字节总长度5或9字节，凑成4的整数倍为12字节
    
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
                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);//输入字符串，判断出有效命令位置长度和个数，返回类型cmdtype：0无效数据，1非阻塞，2阻塞型
                if (cmdtype == 1)
                {//非阻塞命令
                    for(i=0;i<num;i++)
                    {
                        switch(buffer[c[i][0]])
                        {
                            case 'D':
                                if (c[i][1] == 1)
                                {//查询日期
                                }
                                else if (c[i][1] == 8)
                                {//设置日期
                                }
                                break;
                            case 'N':
                                if (c[i][1] == 1)
                                {//查询时间
                                }
                                else if (c[i][1] == 8)
                                {//设置时间
                                }
                                break;
                            case 'T':
                                if (c[i][1] == 1)
                                {//查询温度
                                }
                                else if (c[i][1] == 4)
                                {//设置温度
                                }
                                break;
                            case 'V':
                                if ((c[i][1] == 1) || (c[i][1] == 4))
                                {//查询阀状态  //设置阀开关//Vnnb
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='1';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Valve_Queue,command,10);//向Valve_Queue队列中发送数据
                                }
                                break;
                            case 'S':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//查询步进电机状态  //设置步进电机
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='1';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Step_Queue,command,10);//向Step_Queue队列中发送数据
                                }
                                break;
                            case 'P':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//查询蠕动泵状态  //设置蠕动泵

                                }
                                break;
                            case 'R':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//查询旋转泵状态  //设置旋转泵
                                    
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
                            case 'S':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//查询步进电机状态  //设置步进电机
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='2';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Step_Queue,command,10);//向Step_Queue队列中发送数据
                                }
                                break;
                            case 'P':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//查询蠕动泵状态  //设置蠕动泵

                                }
                                break;
                            case 'R':
                                if ((c[i][1] == 1) || (c[i][1] == 8))
                                {//查询旋转泵状态  //设置旋转泵
                                    
                                }
                                break;
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

//valve_task任务函数 
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
                        memset(buffer,0,CMD_LEN);    //清除缓冲区

                        err=xQueueReceive(Valve_Queue,buffer,10);//采用非阻塞式  portMAX_DELAY
                        if(err == pdTRUE)
                        {//执行阀控制命令
                            printf("阀控制命令:%s\r\n",buffer);
                            //V090
                            bits=(buffer[1]-'0')*10+buffer[2]-'0';
                            if (bits)//bits>0相应的阀动作
                            {
                                if(buffer[3]=='0')
                                    valve_state &= ~(1<<(bits-1));
                                else
                                    valve_state |= 1<<(bits-1);
                            }
                            printf("阀状态:%#x\r\n",valve_state);
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
                printf("阀状态:%#x\r\n",valve_state);
                SwitchOut(valve_state);
                break;

        }
        
    }
}

//step_task任务函数 
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
            memset(buffer,0,CMD_LEN);    //清除缓冲区

            err=xQueueReceive(Step_Queue,buffer,10);//采用非阻塞式  portMAX_DELAY
            if(err == pdTRUE)
            {//执行步进电机控制命令
                printf("步进电机控制命令:%s\r\n",buffer);
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
//  printf("实际步数:%d\r\n",GetPWMCnt);
                        break;
                    default:
                        break;
                }
                
            }
        }
    }
}

/*------------------MAIN 结束------------------*/
//查找字符串中的有效命令，并把字母变成大写
//buf[]：命令字符串，len：字符串长度
//c[8][2]：存放有效命令起始位置和长度，n：存放有效命令数
//c[8][2]一维数组长度>=8否则会出错
//返回值：0无效数据，1非阻塞，2阻塞型
u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n)
{
    u32 result;
    u8 i,j,k,steps;
    u8 valid,end;
    result=0;//默认为无效字符串
    steps=0;//0查第一个起始位，1查命令和结束位
    end=0;
    k=0;//多少个命令
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
            {//<=8个命令响应;否则忽略;
                if(k < 7)
                {//后面还可以有命令
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
                else if(k == 7)
                {//后面命令忽略
                    if((j)&&(valid))//两个;间隔小于最小指令长度1就丢弃,无效指令丢弃
                    {
                        c[k][1]=j;
                        k++;//一共有k个命令
                    }
                }
            }
            else if(buf[i] == end)
            {
                if(k <= 7)
                {//<=8个命令响应;否则忽略;
                    if((j)&&(valid))//两个;间隔小于最小指令长度1就丢弃,无效指令丢弃
                    {
                        c[k][1]=j;
                        k++;//一共有k个命令
                    }
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
                //*n=k;
                steps=2;
            }
            else
            {
                if(buf[i] >= 'a' && buf[i] <= 'z')
                {//转成大写
                    buf[i]+='A'-'a';
                }
                if(buf[i] == 'D'|| buf[i] == 'N'|| buf[i] == 'P'|| buf[i] == 'R'||
                   buf[i] == 'S'|| buf[i] == 'T'|| buf[i] == 'V'|| buf[i] == 'W')
                {
                    if(j)
                    {//后面必须是数字，特殊字母无效
                        valid=0;
                    }
                }
                else if(buf[i] >= '0' && buf[i] <= '9')
                {
                    if(j==0)
                    {//开头必须是特定字母，数字无效
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
        else
        {
            break;
        }
    }
    *n=k;
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
//    while(str[i])
//    {
//        ++i;
//        if (i>60) break;
//    }

//    return i;
}
