#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "timer.h"
#include "ad7799.h"
#include "HC595.h"
#include "stepmoto.h"
#include "w25qxx.h"
#include "exti.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "string.h"

#include "ff.h"
#include "rtc.h"
// State machine helper macros.
#define STATE_ENTRY_ACTION                if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
#define STATE_TRANSITION_TEST             } if ( NextState == CurrentState ) {
#define STATE_TRANSITION_TEST_EXCLUSIVE   } else if ( NextState == CurrentState ) {
#define STATE_EXIT_ACTION                 } if ( NextState != CurrentState ) { CurrentState = NextState;
#define STATE_EXIT_ACTION_EXCLUSIVE       } else if ( NextState != CurrentState ) { CurrentState = NextState;
#define STATE_END                         } break;

//??堆栈大小需要测试,优先级需要确定

//!数值越大优先级越高
//任务优先级
#define COM_TASK_PRIO        10
//任务堆栈大小    
#define COM_STK_SIZE         512  
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
#define MOTION_TASK_PRIO      8
//任务堆栈大小    
#define MOTION_STK_SIZE       256  
//任务句柄
TaskHandle_t Motion_Task_Handler;
//任务函数
void motion_task(void *pvParameters);

////任务优先级
//#define STEP_TASK_PRIO      7
////任务堆栈大小    
//#define STEP_STK_SIZE       256  
////任务句柄
//TaskHandle_t Step_Task_Handler;
////任务函数
//void step_task(void *pvParameters);

////任务优先级
//#define CRP_TASK_PRIO      6
////任务堆栈大小    
//#define CRP_STK_SIZE       256  
////任务句柄
//TaskHandle_t CRP_Task_Handler;
////任务函数
//void crp_task(void *pvParameters);

//任务优先级
#define START_TASK_PRIO        1
//任务堆栈大小    
#define START_STK_SIZE         128  
//任务句柄
TaskHandle_t Start_Task_Handler;
//任务函数
void start_task(void *pvParameters);

//??HMI需要重写，不是以页面为状态，是以收到的命令为状态
#define HMI_OTHER      0
#define HMI_START      1
#define HMI_CHECK      2
#define HMI_ANALY      3
#define HMI_QUERY      4
#define HMI_QC         5
#define HMI_SET        6
#define HMI_SYS        7
#define HMI_OFF        8

#define COM_Q_NUM      4                           //发送数据的消息队列的数量 
#define HMI_Q_NUM     16                           //发送数据的消息队列的数量 

#define CMD_LEN       12                           //阀控制命令长度  //命令为4字节或8字节还有结束位1字节总长度5或9字节，凑成4的整数倍为12字节

#define MOTION_Q_NUM  16                           //运动控制命令的消息队列的数量 

//#define VALVE_Q_NUM   16                           //发送阀控制命令的消息队列的数量 
//#define STEP_Q_NUM    16                           //发送步进电机控制命令的消息队列的数量 
//#define PUMP_Q_NUM     8                           //发送泵控制命令的消息队列的数量 

QueueHandle_t Com_Queue;                           //串口消息队列句柄
QueueHandle_t Hmi_Queue;                           //HMI消息队列句柄
QueueHandle_t Motion_Queue;                        //运动控制消息队列句柄
//QueueHandle_t Valve_Queue;                         //阀控制消息队列句柄
//QueueHandle_t Step_Queue;                          //步进电机控制消息队列句柄
//QueueHandle_t Pump_Queue;                          //泵控制消息队列句柄   ??蠕动泵当做步进电机用，旋转泵当做阀用

struct motion_state{
    u8 motion_type;//运动类型：0，无运动；1，非阻塞运动；2，阻塞运动
    u8 current_motion;//当前运动目标：'A'，复位运动；'S'，步进电机；'V'，阀；'R'，蠕动泵；'P'，旋转泵
    u16 step_flag;//每一位代表一个步进电机开1或关0，只能一个是1??   蠕动泵归到步进电机中
    u16 valve_flag;//每一位代表一个阀开1或关0，可以多个1           旋转泵归到阀中
}motion_state1;//??运动状态变量不锁

//互斥信号量句柄
SemaphoreHandle_t MutexSemMotion;	//运动控制互斥信号量??只锁运动控制模块
//??printf应在一个任务中，否则要加互斥信号量

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);
u8 exf_getfree(u8 *drv,u32 *total,u32 *free);
u8 getDir(u8 * buf1,u8 * buf2,u8 pos);

FATFS fs;
FIL file;                                          //文件
//UINT br,bw;                                        //读写变量
FILINFO fileinfo;                                  //文件信息
DIR dir;                                           //目录

#define LEVEL             8                        //LEVEL设置大小代表遍历的深度，8就代表8层，内存足够的话可以设置更大些
#define WORKPATH          "0:/TEST"                //默认创建的工作目录0:/CRP
#define WORKFILE          "setting"                //默认当前的文件
#define MAXNAMELEN        32                       //文件名最长为32字节

u8 filename[MAXNAMELEN];//32??
FRESULT fres;
u8 curDir[64]=WORKPATH;//??
u8 curFilename[MAXNAMELEN]=WORKFILE;//??
u32 curFileFlag=0;//0，没有打开；1，文件以读的方式打开；2，文件以写的方式打开
//------------------MAIN 开始------------------
int main(void)
{

    //所有硬件初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4，表示支持0-15级抢占优先级，不支持子优先级。
    
    delay_init(168);                               //初始化延时函数
    
    EXTIX_Init();
    
    UART1_Init(115200);                            //串口1初始化波特率为115200，用于串口通讯
    USART3_Init(115200);                           //串口3初始化波特率为115200，用于HMI 通讯
    
    TIM2_PWM_Init();                               //产生LED脉冲信号1120Hz PB11
    AD7799_Init(ADC_CON_GAIN1);                    //初始化AD7799,包含SPI2初始化//??频率选择，时序等..
    
    HC595Init();                                   //??检查时序是否可靠，变化时是否进入临界区
    StepMotoInit();                                //??检查时序是否可靠，变化时是否进入临界区
//StepMotoCal(1100,392,50);
//StepMotoCal(2000,1000,50);

    W25QXX_Init();                                 //??检查失败需要返回一个标志，在开机自检时显示出来  if(W25QXX_Init() != W25Q128) error
    
    My_RTC_Init();                                 //设置RTC
    delay_us(2000000);                             //??为了仿真时延时，正式时删除

    fres=f_mount(&fs,"0:",1);                      //立即挂载FLASH.
    if(fres == FR_NO_FILESYSTEM) {                 //FLASH磁盘,FAT文件系统错误,重新格式化FLASH
        printf("建立文件系统...\r\n");             //格式化FLASH
        fres=f_mkfs("0:",1,4096);                  //格式化FLASH,1,盘符0,不需要引导区,8个扇区为1个簇
        if(fres == FR_OK) {
            //f_setlabel((const TCHAR *)"0:CRP");    //设置Flash磁盘的名字为：CRP
            printf("格式化完成\r\n");               //格式化完成
        } else printf("格式化失败...\r\n");         //格式化失败
        delay_us(1000000);
    }
    fres=f_mkdir((const TCHAR *)curDir);           //创建文件夹
    if(fres == FR_OK) printf("新建目录%s\r\n",curDir);//创建文件夹完成
    else if(fres == FR_EXIST) printf("文件夹%s已经存在\r\n",curDir);
    else printf("创建目录%s失败\r\n",curDir);       //创建文件夹失败

    fileinfo.lfname = (TCHAR *)filename;           //为长文件名分配空间
    fileinfo.lfsize = MAXNAMELEN;                  //文件名长度不能超过32字节

    //创建开始任务
    xTaskCreate((TaskFunction_t )start_task,             //任务函数
                (const char*    )"start_task",           //任务名称
                (uint16_t       )START_STK_SIZE,         //任务堆栈大小
                (void*          )NULL,                   //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,        //任务优先级
                (TaskHandle_t*  )&Start_Task_Handler);   //任务句柄
    vTaskStartScheduler();    
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();                          //进入临界区
    
    //创建消息队列
    Com_Queue=xQueueCreate(COM_Q_NUM,COM_REC_LEN); //创建消息Com_Queue，接收串口命令，命令数4，命令长度63
    Hmi_Queue=xQueueCreate(HMI_Q_NUM,HMI_REC_LEN); //创建消息Hmi_Queue，接收HMI 命令，命令数16，命令长度15
    
    Motion_Queue=xQueueCreate(MOTION_Q_NUM,CMD_LEN);   //创建消息Valve_Queue，接收步进电机命令，命令数16，命令长度12
    //Valve_Queue=xQueueCreate(VALVE_Q_NUM,CMD_LEN); //创建消息Valve_Queue，接收阀命令，命令数16，命令长度12
    //Step_Queue=xQueueCreate(STEP_Q_NUM,CMD_LEN);   //创建消息Valve_Queue，接收步进电机命令，命令数16，命令长度12
    //Pump_Queue=xQueueCreate(PUMP_Q_NUM,CMD_LEN); //创建消息Valve_Queue

   	//创建互斥信号量
	MutexSemMotion=xSemaphoreCreateMutex();
    
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
    //创建motion_task任务
    xTaskCreate((TaskFunction_t )motion_task,
                (const char*    )"motion_task",
                (uint16_t       )MOTION_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )MOTION_TASK_PRIO,
                (TaskHandle_t*  )&Motion_Task_Handler);
//    //创建valve_task任务
//    xTaskCreate((TaskFunction_t )valve_task,
//                (const char*    )"valve_task",
//                (uint16_t       )VALVE_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )VALVE_TASK_PRIO,
//                (TaskHandle_t*  )&Valve_Task_Handler);
//    //创建step_task任务
//    xTaskCreate((TaskFunction_t )step_task,
//                (const char*    )"step_task",
//                (uint16_t       )STEP_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )STEP_TASK_PRIO,
//                (TaskHandle_t*  )&Step_Task_Handler);
//    //创建crp_task任务
//    xTaskCreate((TaskFunction_t )crp_task,
//                (const char*    )"crp_task",
//                (uint16_t       )CRP_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )CRP_TASK_PRIO,
//                (TaskHandle_t*  )&CRP_Task_Handler);
                
    vTaskDelete(Start_Task_Handler);               //删除开始任务
    taskEXIT_CRITICAL();                           //退出临界区
}

u32 sum;
u16 c0,c1,a;
void com_task(void *pvParameters)
{//uart1通讯
    u8 tbuf[64];                                   //??__align(4)  //??注意tbuf的大小要能放得下最深的文件名绝对路径
    u8 year,month,date,week;
    u8 hour,min,sec,ampm;
    
    u8 buffer[COM_REC_LEN];                        //??__align(4) 
    BaseType_t err;
    u8 command[12];                                //命令为4字节或8字节还有结束位1字节总长度5或9字节，凑成4的整数倍为12字节
    
    u8 c[8][2];                                    //存位置和长度
    u8 num;                                        //存命令个数
    u8 cmdtype;                                    //命令类型，0无效数据，1非阻塞，2阻塞型
    
    u8 i,j,m;//,j,valid;
    u32 total,free;
    
    u8 l[LEVEL];                                   //l[]保存每层文件长度，返回上级目录时用
    DIR dir_a[LEVEL];                              //FATFS使用的目录结构，只有这个比较占内存需要LEVEL*36字节
    
    
    u8 bits;
    //u16 c0,c1,a;
    s32 steps;
    
    while(1) {
        if(Com_Queue!=NULL) {
            memset(buffer,0,COM_REC_LEN);          //清除缓冲区

            err=xQueueReceive(Com_Queue,buffer,10);//接收串口命令，采用非阻塞式  portMAX_DELAY
            if(err == pdTRUE) {//串口命令解析
            //1234[V010;S01cpppp;P01ktttt;R01b;T01b;D0171207;N0143500]
            //(P0130020)
                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);//输入字符串，判断出有效命令位置长度和个数，返回类型cmdtype：0无效数据，1非阻塞，2阻塞型
                if (cmdtype == 1) {//非阻塞命令[]，直接处理
                    for(i=0;i<num;i++) {
                        switch(buffer[c[i][0]]) {
                            case 'D':
                                //??获取时间之前，需要先向HMI获取时间
                                RTC_Get_Time(&hour,&min,&sec,&ampm);//得到时间    
                                RTC_Get_Date(&year,&month,&date,&week);
                                if (c[i][1] == 8) {//设置日期
                                    year=(buffer[c[i][0]+2]-'0')*10+buffer[c[i][0]+3]-'0';
                                    month=(buffer[c[i][0]+4]-'0')*10+buffer[c[i][0]+5]-'0';
                                    date=(buffer[c[i][0]+6]-'0')*10+buffer[c[i][0]+7]-'0';
                                    year=CorrectYear(year);
                                    CorrectDate(year,&month,&date);
                                    CorrectTime(&hour,&min,&sec);
                                    week=RTC_Get_Week(2000 + year,month,date);
                                    if(hour < 12)//??
                                        RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //设置时间
                                    else
                                        RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //设置时间
                                    RTC_Set_Date(year,month,date,week);    //设置日期
                                    //????还要设置HMI的时间
                                    
                                    
                                }
                                printf("D20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                break;
                            case 'F':
                                if (c[i][1] == 1) {//查询磁盘信息、文件列表
                                    exf_getfree((u8*)"0:",&total,&free);
                                    printf("0磁盘总容量：%dKB，剩余：%dKB\r\n", total,free);

                                    strcpy((char *)tbuf, (const char *)curDir);
                                    
                                    m = 0;
                                    j = 1;
                                    printf("当前目录:\r\n%s->\r\n", tbuf);
                                    while(1) {
                                        if ( j > m ) {                                         //只有搜索子目录时才执行
                                            f_opendir(&dir_a[j-1], (TCHAR *)tbuf);
                                            l[j-1] = strlen((char *)tbuf);
                                        }
                                        m = j;
                                        f_readdir(&dir_a[j-1], &fileinfo);                     //读取目录下的一个文件
                                        if (fileinfo.fname[0] == 0) {                          //到末尾了,退出 //错误了/fres != FR_OK || 
                                            if (j>1) j--;                                      //下个循环进入父目录
                                            else break;
                                            tbuf[l[j-1]] = '\0';                               //存储的路径返回上级目录
                                        }else{
                                            sprintf((char *)tbuf,"%s/%s",tbuf,*fileinfo.lfname ? fileinfo.lfname : fileinfo.fname);//搜索到的文件或文件名连接成完整的路径
                                            if (fileinfo.fattrib & AM_DIR) {                   //是目录
                                                printf("%s [%dD]\r\n", tbuf,j);                //打印目录
                                                if (j<8) j++;                                  //下个循环进入子目录
                                            }else {
                                                printf("%s [%dF]\r\n", tbuf,j);                //打印文件
                                                tbuf[l[j-1]] = '\0';                           //存储的路径返回目录
                                            }
                                        }
                                    }
                                } else if (c[i][1] == 2) {//F0,F1,F2,F3,F4
                                    if (buffer[c[i][0]+1]=='0'){//读
                                        if(curFileFlag==0) {
                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("当前文件名:%s,标志:%d\r\n", tbuf,curFileFlag);//打印文件名 
                                            fres = f_open(&file, (const TCHAR*)tbuf, FA_OPEN_EXISTING | FA_READ);
                                            if(fres == FR_OK) {//FR_OK 打开文件成功  ??文件不存在呢
                                                printf("文件内容:->\r\n");
                                                do{
                                                    //fres = f_read(&file, tbuf, 60, &br);//读取60个字节，br返回读取的字节数
                                                    if(f_gets((TCHAR *)tbuf,60,&file) != 0)//读取一行 ??\n会变成\0吗 \r怎么办??
                                                        printf("%s", tbuf);
                                                }while ((!f_eof(&file))&&(!f_error(&file)));//??没有结束且没有发生错误
                                                fres=f_close(&file);
                                            } else if (fres == FR_NO_FILE){
                                                printf("文件不存在\r\n");
                                            }
                                        }
                                    } else if (buffer[c[i][0]+1]=='1'){//写
                                        if(curFileFlag==2) {
                                            curFileFlag=0;
                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("当前文件名:%s,标志:%d\r\n", tbuf,curFileFlag);//打印文件名 
                                            fres=f_close(&file);
                                        }
                                    } else if (buffer[c[i][0]+1]=='2'){//删??
                                        if(curFileFlag==0) {
                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("当前文件名:%s,标志:%d\r\n", tbuf,curFileFlag);//打印文件名 
                                            fres = f_unlink((const TCHAR*)tbuf);
                                            if (fres == FR_OK)
                                                printf("删除成功\r\n");//删除成功
                                            else if (fres == FR_NO_FILE)
                                                printf("文件不存在\r\n");//删除成功
                                            else
                                                printf("删除失败\r\n");//删除失败
                                        }
                                    } else if (buffer[c[i][0]+1]=='3'){//返回当前文件夹
                                        printf("当前文件夹:%s,标志:%d\r\n", curDir,curFileFlag);//打印文件夹
                                    } else if (buffer[c[i][0]+1]=='4'){//格式化磁盘
                                        printf("建立文件系统...\r\n");        //格式化FLASH
                                        fres=f_mkfs("0:",1,4096);             //格式化FLASH,1,盘符0,不需要引导区,8个扇区为1个簇
                                        if(fres == FR_OK) {
                                            //f_setlabel((const TCHAR *)"0:CRP");    //设置Flash磁盘的名字为：CRP
                                            printf("格式化完成\r\n");         //格式化完成
                                        } else printf("格式化失败...\r\n");    //格式化失败
                                        vTaskDelay(pdMS_TO_TICKS(1000));
                                        
                                        strcpy((char *)curDir,WORKPATH);
                                        strcpy((char *)curFilename,WORKFILE);
                                        curFileFlag=0;
                                        fres=f_mkdir((const TCHAR *)curDir);        //创建文件夹//??已经存在会不会冲突
                                        if(fres == FR_OK) printf("创建了文件夹%s\r\n",curDir);         //创建文件夹完成
                                        else if(fres == FR_EXIST) printf("文件夹%s已经存在\r\n",curDir);
                                        else printf("创建文件夹%s失败\r\n",curDir);         //创建文件夹失败
                                        printf("当前文件名%s/%s,标志:%d\r\n", curDir,curFilename,curFileFlag);                                        
                                    }
                                } else if (c[i][1] > 3) {//读、写、删除文件 
                                    if(buffer[c[i][0]+1]=='0'){//读 [F0,A00010001U000000]  ??,判断
                                        if(curFileFlag==0) {
                                            if(c[i][1]-3 < 32) {//长度小于32
                                                memcpy(curFilename, buffer+c[i][0]+3, c[i][1]-3);
                                                curFilename[c[i][1]-3]='\0';//??最后一位不能漏掉
                                            } else {//长度大于等于31，后面丢弃
                                                memcpy(curFilename, buffer+c[i][0]+3, 31);
                                                curFilename[31]='\0';//
                                            }

                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("当前文件名:%s,标志:%d\r\n", tbuf,curFileFlag);//打印文件名 
                                            fres = f_open(&file, (const TCHAR*)tbuf, FA_OPEN_EXISTING | FA_READ);
                                            if(fres == FR_OK) {//FR_OK 打开文件成功  ??文件不存在呢
                                                printf("文件内容:->\r\n");
                                                do{
                                                    //fres = f_read(&file, tbuf, 60, &br);//读取60个字节，br返回读取的字节数
                                                    if(f_gets((TCHAR *)tbuf,60,&file) != 0)//读取一行 ??\n会变成\0吗 \r怎么办??
                                                        printf("%s", tbuf);
                                                }while ((!f_eof(&file))&&(!f_error(&file)));//??没有结束且没有发生错误
                                                fres=f_close(&file);
                                            } else if (fres == FR_NO_FILE){
                                                printf("文件不存在\r\n");
                                            }
                                        }
                                    } else if(buffer[c[i][0]+1]=='1'){//写?? [F1,ABCDEFG]
                                        sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                        printf("当前文件名:%s,标志:%d\r\n", tbuf,curFileFlag);//打印文件名 
                                        if(curFileFlag==0) {//第一次写先打开文件
                                            fres = f_open(&file, (const TCHAR*)tbuf, FA_CREATE_ALWAYS | FA_WRITE);//??重名的会覆盖，没有的文件名直接创建
                                            if(fres == FR_OK) {
                                                curFileFlag=2;
                                                memcpy(tbuf, buffer+c[i][0]+3, c[i][1]-3);//??
                                                tbuf[c[i][1]-3]='\r';
                                                tbuf[c[i][1]-2]='\n';
                                                tbuf[c[i][1]-1]='\0';
                                                //fres=f_write(&file,buffer+c[i][0]+3,c[i][0]-3,&bw);
                                                f_puts((TCHAR *)tbuf,&file);//??
                                                printf("写入数据:%s\r\n",tbuf);
                                            }
                                        } else if(curFileFlag==2) {//第n次写不用打开文件 
                                                memcpy(tbuf, buffer+c[i][0]+3, c[i][1]-3);//??
                                                tbuf[c[i][1]-3]='\r';
                                                tbuf[c[i][1]-2]='\n';
                                                tbuf[c[i][1]-1]='\0';
                                            //fres=f_write(&file,buffer+c[i][0]+3,c[i][0]-3,&bw);
                                            f_puts((TCHAR *)tbuf,&file);//??
                                            printf("写入数据:%s\r\n",tbuf);
                                        }
                                    } else if(buffer[c[i][0]+1]=='3'){//设置当前文件夹 [F3,0:/CRP]
                                        if(curFileFlag==0) {
                                            if(c[i][1]-3 < 64) {//长度小于64
                                                memcpy(curDir, buffer+c[i][0]+3, c[i][1]-3);
                                                curDir[c[i][1]-3]='\0';//??最后一位不能漏掉
                                            } else {//长度大于等于63，后面丢弃
                                                memcpy(curDir, buffer+c[i][0]+3, 63);
                                                curDir[63]='\0';//
                                            }

                                            if(strlen((const TCHAR *)curDir)<=3) {
                                                strcpy ((char *)curDir, "0:");
                                                printf("当前是根目录%s\r\n",curDir);
                                            } else {
                                                m=1;
                                                do{
                                                    j=getDir(curDir,tbuf,m);
                                                    fres=f_mkdir((const TCHAR *)tbuf);        //创建文件夹//??已经存在会不会冲突//??一次可以创建多层文件夹吗？
                                                    if(fres == FR_OK) printf("创建文件夹%s\r\n",tbuf);         //创建文件夹完成
                                                    else if(fres == FR_EXIST) printf("文件夹%s已经存在\r\n",tbuf);
                                                    else printf("创建文件夹%s失败\r\n",tbuf);         //创建文件夹失败
                                                    m++;
                                                }while(j);
                                                strcpy((char *)curDir,(const char *)tbuf);
                                                printf("当前目录:%s,标志:%d\r\n", curDir,curFileFlag);//打印目录
                                            }
                                        }
                                    }
                                }
                                break;
                            case 'N':
                                //??获取时间之前，需要先向HMI获取时间
                                RTC_Get_Time(&hour,&min,&sec,&ampm);//得到时间    
                                RTC_Get_Date(&year,&month,&date,&week);
                                if (c[i][1] == 8) {//设置时间
                                    hour=(buffer[c[i][0]+2]-'0')*10+buffer[c[i][0]+3]-'0';
                                    min=(buffer[c[i][0]+4]-'0')*10+buffer[c[i][0]+5]-'0';
                                    sec=(buffer[c[i][0]+6]-'0')*10+buffer[c[i][0]+7]-'0';
                                    year=CorrectYear(year);
                                    CorrectDate(year,&month,&date);
                                    CorrectTime(&hour,&min,&sec);
                                    week=RTC_Get_Week(2000 + year,month,date);
                                    if(hour < 12)//??
                                        RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //设置时间
                                    else
                                        RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //设置时间
                                    RTC_Set_Date(year,month,date,week);    //设置日期
                                    //????还要设置HMI的时间
                                    
                                    
                                }
                                printf("N20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                break;
                            case 'T':
                                if (c[i][1] == 1) {//查询温度
                                }
                                else if (c[i][1] == 4) {//设置温度
                                }
                                break;
                            case 'M':
                                //查询状态
                                printf("运动类型：%d\r\n",motion_state1.motion_type);
                                if(motion_state1.current_motion >= 'A' && motion_state1.current_motion <= 'Z')
                                    printf("当前运动目标：%c\r\n",motion_state1.current_motion);
                                else
                                    printf("当前没有运动目标\r\n");
                                printf("步进电机状态：%#x\r\n",motion_state1.step_flag);
                                printf("阀状态：%#x\r\n",motion_state1.valve_flag);
                                break;
                            case 'V':
                                if (c[i][1] == 4) {//查询阀状态  //设置阀开关//Vnnb
                                    //在阻塞队列Motion_Queue为空时直接处理??如何避免冲突 处理时不允许向阻塞队列Motion_Queue添加任务
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //已经获得信号量并且现在可以访问运动模块
                                        //运动模块互斥访问
                                        
                                        motion_state1.motion_type=1;//非阻塞运动
                                        motion_state1.current_motion='V';
                                        
                                        printf("阀控制命令:%s\r\n",buffer);  //??(buffer[c[i][0]+2]-'0')*10+buffer[c[i][0]+3]-'0';
                                        //[V011]
                                        bits=(buffer[c[i][0]+1]-'0')*10+buffer[c[i][0]+2]-'0';
                                        if (bits) {//bits>0相应的阀动作
                                            if(buffer[c[i][0]+3]=='0')
                                                motion_state1.valve_flag &= ~(1<<(bits-1));
                                            else
                                                motion_state1.valve_flag |= 1<<(bits-1);
                                        }
                                        printf("阀状态:%#x\r\n",motion_state1.valve_flag);
                                        SwitchOut(motion_state1.valve_flag);
                                        
                                        motion_state1.motion_type=0;//无运动 
                                        motion_state1.current_motion=0;//无运动目标
                                        
                                        xSemaphoreGive( MutexSemMotion );//释放互斥信号量
                                    } else printf("com_task[V]没有获得互斥信号量\r\n");
                                }
                                break;
                            case 'S':
                                if (c[i][1] == 8) {//查询步进电机状态  //设置步进电机
                                    //在阻塞队列Motion_Queue为空时直接处理??如何避免冲突 处理时不允许向阻塞队列Motion_Queue添加任务
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //已经获得信号量并且现在可以访问运动模块
                                        //运动模块互斥访问
                                        
                                        motion_state1.motion_type=1;//非阻塞运动
                                        motion_state1.current_motion='S';
                                        
                                        printf("步进电机控制命令:%s\r\n",buffer);
                                        num=(buffer[c[i][0]+1]-'0')*10+buffer[c[i][0]+2]-'0';
                                    
                                        if(num) {//??当num=5时，就是蠕动泵
                                            steps = (buffer[c[i][0]+4]-'0')*1000+(buffer[c[i][0]+5]-'0')*100+(buffer[c[i][0]+6]-'0')*10+buffer[c[i][0]+7]-'0';
                                            
                                            bits = buffer[c[i][0]+3]-'0';
                                            if(( bits== 3) || ( bits >= 7)) {//3,7~255
                                                bits =0;
                                            } else if( bits >= 4)  {//4,5,6
                                                steps=-steps;
                                                bits -= 4;
                                            }//0,1,2
                                            
                                            StepMotoMove(num-1,0,steps,bits);
                                        }
                                        
                                        motion_state1.motion_type=0;//无运动 
                                        motion_state1.current_motion=0;//无运动目标
                                        
                                        xSemaphoreGive( MutexSemMotion );//释放互斥信号量
                                    } else printf("com_task[S]没有获得互斥信号量\r\n");
                                }
                                break;
                            case 'P':
                                if (c[i][1] == 8) {//查询蠕动泵状态  //设置蠕动泵
                                    //在阻塞队列Motion_Queue为空时直接处理??如何避免冲突 处理时不允许向阻塞队列Motion_Queue添加任务
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //已经获得信号量并且现在可以访问运动模块
                                        //运动模块互斥访问
                                        
                                        motion_state1.motion_type=1;//非阻塞运动
                                        motion_state1.current_motion='P';
                                        
                                        printf("蠕动泵控制命令:%s\r\n",buffer);
                                        num=(buffer[c[i][0]+1]-'0')*10+buffer[c[i][0]+2]-'0';
                                        
                                        if(num==1) {
                                            steps = (buffer[c[i][0]+4]-'0')*1000+(buffer[c[i][0]+5]-'0')*100+(buffer[c[i][0]+6]-'0')*10+buffer[c[i][0]+7]-'0';
                                            
                                            bits = buffer[c[i][0]+3]-'0';
                                            if(( bits== 3) || ( bits >= 7)) {//3,7~255
                                                bits =0;
                                            } else if( bits >= 4)  {//4,5,6
                                                steps=-steps;
                                                bits -= 4;
                                            }//0,1,2
                                            
                                            StepMotoMove(4,0,steps,bits);
                                        }
                                        
                                        motion_state1.motion_type=0;//无运动 
                                        motion_state1.current_motion=0;//无运动目标
                                        
                                        xSemaphoreGive( MutexSemMotion );//释放互斥信号量
                                    } else printf("com_task[P]没有获得互斥信号量\r\n");
                                }
                                break;
                            case 'R':
                                if (c[i][1] == 4) {//查询旋转泵状态  //设置旋转泵
                                    //在阻塞队列Motion_Queue为空时直接处理??如何避免冲突 处理时不允许向阻塞队列Motion_Queue添加任务
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //已经获得信号量并且现在可以访问运动模块
                                        //运动模块互斥访问
                                        
                                        motion_state1.motion_type=1;//非阻塞运动
                                        motion_state1.current_motion='R';
                                        
                                        printf("旋转泵控制命令:%s\r\n",buffer);//??c[i][0]+
                                        //[R011]
                                        bits=(buffer[c[i][0]+1]-'0')*10+buffer[c[i][0]+2]-'0';
                                        if (bits==1) {//bits>0相应的阀动作
                                            if(buffer[c[i][0]+3]=='0')
                                                motion_state1.valve_flag &= ~(1<<(15-1));
                                            else
                                                motion_state1.valve_flag |= 1<<(15-1);
                                        }
                                        printf("旋转泵状态:%#x\r\n",motion_state1.valve_flag);
                                        SwitchOut(motion_state1.valve_flag);
                                        
                                        motion_state1.motion_type=0;//无运动 
                                        motion_state1.current_motion=0;//无运动目标
                                        
                                        xSemaphoreGive( MutexSemMotion );//释放互斥信号量
                                    } else printf("com_task[R]没有获得互斥信号量\r\n");
                                }
                                break;
                            case 'B':
                                if (c[i][1] == 13) {
                                    c0=(buffer[c[i][0]+1]-'0')*1000+(buffer[c[i][0]+2]-'0')*100+(buffer[c[i][0]+3]-'0')*10+(buffer[c[i][0]+4]-'0');
                                    c1=(buffer[c[i][0]+5]-'0')*1000+(buffer[c[i][0]+6]-'0')*100+(buffer[c[i][0]+7]-'0')*10+(buffer[c[i][0]+8]-'0');
                                    a=(buffer[c[i][0]+9]-'0')*1000+(buffer[c[i][0]+10]-'0')*100+(buffer[c[i][0]+11]-'0')*10+(buffer[c[i][0]+12]-'0');
                                    printf("c0:%d,c1:%d,a:%d,\r\n", c0,c1,a); 
                                    sum=StepMotoCal(c0,c1,a);

                                }
                                break;
                            default:
                                break;
                        }
                    }
                } else if (cmdtype == 2) {//阻塞命令()，发送到motion_task任务去处理
                    for(i=0;i<num;i++) {
                        switch(buffer[c[i][0]]) {
                            case 'M':
                                //查询状态
                                printf("运动类型：%d\r\n",motion_state1.motion_type);
                                if(motion_state1.current_motion >= 'A' && motion_state1.current_motion <= 'Z')
                                    printf("当前运动目标：%c\r\n",motion_state1.current_motion);
                                else
                                    printf("当前没有运动目标\r\n");
                                printf("步进电机状态：%#x\r\n",motion_state1.step_flag);
                                printf("阀状态：%#x\r\n",motion_state1.valve_flag);
                                break;
                            case 'V':
                                if (c[i][1] == 4) {//查询阀状态  //设置阀开关//Vnnb
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='V';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//向Motion_Queue队列中发送数据
                                }
                                break;
                            case 'S':
                                if (c[i][1] == 8) {//查询步进电机状态  //设置步进电机
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='S';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//向Motion_Queue队列中发送数据
                                }
                                break;
                            case 'P':
                                if (c[i][1] == 8) {//查询蠕动泵状态  //设置蠕动泵
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='P';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//向Motion_Queue队列中发送数据
                                }
                                break;
                            case 'R':
                                if (c[i][1] == 4) {//查询旋转泵状态  //设置旋转泵
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='R';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//向Motion_Queue队列中发送数据
                                }
                                break;
                            case 'W':
                                if (c[i][1] == 8) {//设置延时
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='W';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//向Motion_Queue队列中发送数据
                                }
                                break;
                            case 'A'://复位运动
                                memcpy(command,buffer+c[i][0], c[i][1]);
                                //command[0]='A';
                                command[1]='\0';
                                xQueueSend(Motion_Queue,command,10);//向Motion_Queue队列中发送数据
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

//motion_task任务函数 只处理阻塞式任务()
void motion_task(void *pvParameters)
{
    u8 buffer[CMD_LEN];
    BaseType_t err;
    u8 bits;
    //u32 valve_state;
    s32 steps;
    u8 num;
    //??有一个初始状态，把所有值归零，所有步进电机到起始位置
    while(1) {
        if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //已经获得信号量并且现在可以访问运动模块
            //运动模块互斥访问
            

            if(Motion_Queue!=NULL) {
                memset(buffer,0,CMD_LEN);    //清除缓冲区

                err=xQueueReceive(Motion_Queue,buffer,10);//采用非阻塞式  portMAX_DELAY
                if(err == pdTRUE) {
                    switch(buffer[0]){
                        case 'A'://复位命令
                            motion_state1.motion_type=2;//阻塞运动()
                            motion_state1.current_motion='A';
                            printf("复位命令:%s\r\n",buffer);
                            //所有步进电机到起始位置，所有阀断电，柱塞泵要和阀配合，并加入必要的延时
                        
                        
                            break;
                        case 'V'://执行阀控制命令
                            motion_state1.motion_type=2;//阻塞运动()
                            motion_state1.current_motion='V';
                            printf("阀控制命令:%s\r\n",buffer);
                            //(V011)
                            bits=(buffer[1]-'0')*10+buffer[2]-'0';
                            if (bits) {//bits>0相应的阀动作
                                if(buffer[3]=='0')
                                    motion_state1.valve_flag &= ~(1<<(bits-1));
                                else
                                    motion_state1.valve_flag |= 1<<(bits-1);
                            }
                            printf("阀状态:%#x\r\n",motion_state1.valve_flag);
                            SwitchOut(motion_state1.valve_flag);
                            break;
                        case 'S'://执行步进电机控制命令
                            motion_state1.motion_type=2;//阻塞运动()
                            motion_state1.current_motion='S';
                            printf("步进电机控制命令:%s\r\n",buffer);
                            num=(buffer[1]-'0')*10+buffer[2]-'0';
                        
                            if((num>=1) && (num<=5)){//??当num=5时，就是蠕动泵

                                steps = (buffer[4]-'0')*1000+(buffer[5]-'0')*100+(buffer[6]-'0')*10+buffer[7]-'0';

                                bits = buffer[3]-'0';
                                if(( bits== 3) || ( bits >= 7)) {//3,7~255
                                    bits =0;
                                } else if( bits >= 4)  {//4,5,6
                                    steps=-steps;
                                    bits -= 4;
                                }//0,1,2
                                
                                sum=StepMotoMove(num-1,0,steps,bits);
                                

                                vTaskDelay(pdMS_TO_TICKS((sum*2+StepMotor[num-1].Size[1]*(*StepMotor[num-1].Buf[1]))/1000));
//printf("sum:%d;steps:%d;c1:%d;time:%d\r\n",sum,StepMotor[num-1].Size[1],*StepMotor[num-1].Buf[1],(sum*2+StepMotor[num-1].Size[1]*(*StepMotor[num-1].Buf[1]))/1000);
                            }

                            break;
                        case 'P'://执行蠕动泵控制命令  归到步进电机控制
                            motion_state1.motion_type=2;//阻塞运动()
                            motion_state1.current_motion='P';
                            printf("蠕动泵控制命令:%s\r\n",buffer);
                            num=(buffer[1]-'0')*10+buffer[2]-'0';
                            if(num==1) {

                                steps = (buffer[4]-'0')*1000+(buffer[5]-'0')*100+(buffer[6]-'0')*10+buffer[7]-'0';
                                
                                bits = buffer[3]-'0';
                                if(( bits== 3) || ( bits >= 7)) {//3,7~255
                                    bits =0;
                                } else if( bits >= 4)  {//4,5,6
                                    steps=-steps;
                                    bits -= 4;
                                }//0,1,2
                                
                                sum=StepMotoMove(4,0,steps,bits);
                                
                                vTaskDelay(pdMS_TO_TICKS((sum*2+StepMotor[4].Size[1]*(*StepMotor[4].Buf[1]))/1000));

                            }
                            break;
                        case 'R'://执行旋转泵控制命令  归到阀控制
                            motion_state1.motion_type=2;//阻塞运动()
                            motion_state1.current_motion='R';
                            printf("旋转泵控制命令:%s\r\n",buffer);
                            //(R011)
                            bits=(buffer[1]-'0')*10+buffer[2]-'0';
                            if (bits==1) {//bits>0相应的阀动作
                                if(buffer[3]=='0')
                                    motion_state1.valve_flag &= ~(1<<(15-1));
                                else
                                    motion_state1.valve_flag |= 1<<(15-1);
                            }
                            printf("旋转泵状态:%#x\r\n",motion_state1.valve_flag);
                            SwitchOut(motion_state1.valve_flag);
                            break;
                        case 'W'://执行延时控制命令
                            motion_state1.motion_type=2;//阻塞运动()
                            motion_state1.current_motion='W';
                            printf("延时命令:%s\r\n",buffer);
                        
                            steps = (buffer[1]-'0')*1000000+(buffer[2]-'0')*100000+(buffer[3]-'0')*10000+
                                    (buffer[4]-'0')*1000+(buffer[5]-'0')*100+(buffer[6]-'0')*10+buffer[7]-'0';
                            vTaskDelay(pdMS_TO_TICKS(steps));
                            printf("延时结束\r\n");
                            break;
                    }
                }
                motion_state1.motion_type=0;//无运动 
                motion_state1.current_motion=0;//无运动目标
            }
            
            
            xSemaphoreGive( MutexSemMotion );//释放互斥信号量
        } else printf("motion_task没有获得互斥信号量\r\n");
    }
}

//hmi_task任务函数 
void hmi_task(void *pvParameters)
{//uart3通讯
    u8 buffer[HMI_REC_LEN];
    BaseType_t res;
    u8 year,month,date,week;
    u8 hour,min,sec,start_cnt;//time_valid;//,ampm;
    
//    u8 command[12];
    
    u32 PreviousState = HMI_OTHER;
    u32 CurrentState  = HMI_OTHER;
    u32 NextState     = HMI_OTHER;
    
    while(1) {
        if(Hmi_Queue!=NULL) {//获取HMI传来的信息
            memset(buffer,0,HMI_REC_LEN);    //清除缓冲区
            res=xQueueReceive(Hmi_Queue,buffer,10);//portMAX_DELAY ??延时0还是其它
            if(res == pdTRUE) {//HMI命令解析
                printf("%s",buffer);
                //Uart3_PutString("page start", sizeof("page start")-1);
            }
        }
        
        switch(CurrentState) {
            case HMI_START:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page start", sizeof("page start")-1);//Uart3_HMICmd("cls RED", sizeof("cls RED")-1);//刷屏
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100  //??打印pdMS_TO_TICKS看看是不是实际值
//                    memcpy(command,"1021", 4);
//                    command[4]='\0';
//                    xQueueSend(Valve_Queue,command,10);//向Valve_Queue队列中发送数据
//                    memcpy(command,"10140400", 8);
//                    command[8]='\0';
//                    xQueueSend(Step_Queue,command,10);//向Valve_Queue队列中发送数据
                    start_cnt=0;
                    //time_valid=0;
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    //需要完成：从HMI读取当前时间赋值给RTC，步进电机运动，光电信号（光隔和红光）测试，阀，泵等检测
                    //1：从HMI读取当前时间赋值给RTC
                    if(start_cnt == 0){
                        //Uart3_HMICmd("get rtc0", sizeof("get rtc0")-1);
                        Uart3_PutString("\x5A\xA5\x03\x81\x20\x07", 6);//5A A5 03 81 20 07 读取RTC
                        memset(buffer,0,HMI_REC_LEN);
                        start_cnt++;
                    } else if(start_cnt < 10){
                        if((res == pdTRUE) && (buffer[1] == 0x20)){//收到有效数据
                            
                            year  = ((buffer[3] >> 4 ) & 0x0f)* 10 + (buffer[3] & 0x0f);
                            month = ((buffer[4] >> 4 ) & 0x0f)* 10 + (buffer[4] & 0x0f);
                            date  = ((buffer[5] >> 4 ) & 0x0f)* 10 + (buffer[5] & 0x0f);
                            week  =                                  (buffer[6] & 0x0f);
                            hour  = ((buffer[7] >> 4 ) & 0x0f)* 10 + (buffer[7] & 0x0f);
                            min   = ((buffer[8] >> 4 ) & 0x0f)* 10 + (buffer[8] & 0x0f);
                            sec   = ((buffer[9] >> 4 ) & 0x0f)* 10 + (buffer[9] & 0x0f);

                            //printf("time_valid:%d\r\n",time_valid);
                            printf("\r\n时间有效20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);//??怎么到了这
                            //year=CorrectYear(year);
                            //CorrectDate(year,&month,&date);
                            //CorrectTime(&hour,&min,&sec);
                            //week=RTC_Get_Week(2000 + year,month,date);
//                            if(hour < 12)//?? 
//                                RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //设置时间
//                            else
//                                RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //设置时间
//                            RTC_Set_Date(year,month,date,week);    //设置日期
                            //time_valid = 1;
                            //start_cnt=10;
                            NextState = HMI_CHECK;
                        } else {//无数据
                            start_cnt++;
                        }
                    } else {
                        //显示读取时间失败
                        printf("\r\n没有读到时间\r\n");

                        NextState = HMI_CHECK;
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));//至少保证20 的HMI反馈时间
                    //printf("start_cnt:%d\r\n",start_cnt);
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;
                    vTaskDelay(pdMS_TO_TICKS(2000));//至少保证10
                STATE_END                                                       //} break;
            case HMI_CHECK:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page check", sizeof("page check")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    //需要完成：清洗等
                    
                    NextState = HMI_ANALY;
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;
                    vTaskDelay(pdMS_TO_TICKS(2000));//至少保证10
                STATE_END                                                       //} break;
           case HMI_ANALY:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page analyse_0", sizeof("page analyse_0")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(buffer[0]==0x65) {
                        if(buffer[1]==0x02) {
                            switch(buffer[2]) {
                                case 0x03:
                                    NextState = HMI_QUERY;
                                    break;
                                case 0x04:
                                    NextState = HMI_QC;
                                    break;
                                case 0x05:
                                    NextState = HMI_SET;
                                    break;
                                case 0x06:
                                    NextState = HMI_SYS;
                                    break;
                                case 0x07:
                                    NextState = HMI_OFF;
                                    break;
                                default:
                                    break;
                            }
                        } else {
                            //Uart3_HMICmd("page analyse_0", sizeof("page analyse_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_QUERY:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page query_0", sizeof("page query_0")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(buffer[0]==0x65) {
                        if(buffer[1]==0x03 ) {
                            switch(buffer[2]) {
                                case 0x0B:
                                    NextState = HMI_ANALY;
                                    break;
                                case 0x0C:
                                    NextState = HMI_QC;
                                    break;
                                case 0x0D:
                                    NextState = HMI_SET;
                                    break;
                                case 0x0E:
                                    NextState = HMI_SYS;
                                    break;
                                case 0x0F:
                                    NextState = HMI_OFF;
                                    break;
                                default:
                                    break;
                            }
                        } else {
                            //Uart3_HMICmd("page query_0", sizeof("page query_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_QC:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page qc_0", sizeof("page qc_0")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(buffer[0]==0x65) {
                        if(buffer[1]==0x04 ) {
                            switch(buffer[2]) {
                                case 0x07:
                                    NextState = HMI_ANALY;
                                    break;
                                case 0x08:
                                    NextState = HMI_QUERY;
                                    break;
                                case 0x09:
                                    NextState = HMI_SET;
                                    break;
                                case 0x0A:
                                    NextState = HMI_SYS;
                                    break;
                                case 0x0B:
                                    NextState = HMI_OFF;
                                    break;
                                default:
                                    break;
                            }
                        } else {
                            //Uart3_HMICmd("page qc_0", sizeof("page qc_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_SET:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page set_0", sizeof("page set_0")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(buffer[0]==0x65) {
                        if(buffer[1]==0x05 ) {
                            switch(buffer[2]) {
                                case 0x01:
                                    NextState = HMI_ANALY;
                                    break;
                                case 0x02:
                                    NextState = HMI_QUERY;
                                    break;
                                case 0x03:
                                    NextState = HMI_QC;
                                    break;
                                case 0x04:
                                    NextState = HMI_SYS;
                                    break;
                                case 0x05:
                                    NextState = HMI_OFF;
                                    break;
                                default:
                                    break;
                            }
                        } else {
                            //Uart3_HMICmd("page set_0", sizeof("page set_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_SYS:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page system_0", sizeof("page system_0")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(buffer[0]==0x65) {
                        if(buffer[1]==0x06 ) {
                            switch(buffer[2]) {
                                case 0x07:
                                    NextState = HMI_ANALY;
                                    break;
                                case 0x08:
                                    NextState = HMI_QUERY;
                                    break;
                                case 0x09:
                                    NextState = HMI_QC;
                                    break;
                                case 0x0A:
                                    NextState = HMI_SET;
                                    break;
                                case 0x0B:
                                    NextState = HMI_OFF;
                                    break;
                                default:
                                    break;
                            }
                        } else {
                            //Uart3_HMICmd("page system_0", sizeof("page system_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_OFF:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page poweroff_0", sizeof("page poweroff_0")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(buffer[0]==0x65) {
                        if(buffer[1]==0x07 ) {
                            switch(buffer[2]) {
                                case 0x02:
                                    NextState = HMI_ANALY;
                                    break;
                                case 0x03:
                                    NextState = HMI_QUERY;
                                    break;
                                case 0x04:
                                    NextState = HMI_QC;
                                    break;
                                case 0x05:
                                    NextState = HMI_SET;
                                    break;
                                case 0x06:
                                    NextState = HMI_SYS;
                                    break;
                                default:
                                    break;
                            }
                        } else {
                            //Uart3_HMICmd("page poweroff_0", sizeof("page poweroff_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_OTHER:
           default:
                vTaskDelay(pdMS_TO_TICKS(2000));//??
                CurrentState = HMI_START;
                NextState    = HMI_START;
                
                break;
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
    for(i=0;i<len;i++) {
        if(steps == 0) {
            if((buf[i] == ']') || (buf[i] == ')')) {//先找到了结束位直接退出
                //result=0;//默认为无效字符串
                //end=0;
                steps=2;
            } else if(buf[i] == '[') {//找起始位[
                end=']';//保存结束符
            } else if(buf[i] == '(') {//找起始位(
                end=')';//保存结束符
            }
            if(end) {
                j=0;//单个命令长
                c[0][0]=i+1;//命令位置
                c[0][1]=0;//命令长度
                valid=1;//默认当前命令有效
                steps=1;
            }
        } else if(steps == 1) {
            //找分隔符;或结束符]
            if(buf[i] == ';') {//<=8个命令响应;否则忽略;
                if(k < 7) {//后面还可以有命令
                    if((j)&&(valid)) {//两个;间隔小于最小指令长度1就丢弃,无效指令丢弃
                        c[k][1]=j;
                        k++;//一共有k个命令
                    }
                    c[k][0]=i+1;
                    c[k][1]=0;
                    j=0;//单个命令长
                    valid=1;//默认当前命令有效
                }
                else if(k == 7) {//后面命令忽略
                    if((j)&&(valid)) {//两个;间隔小于最小指令长度1就丢弃,无效指令丢弃
                        c[k][1]=j;
                        k++;//一共有k个命令
                    }
                }
            } else if(buf[i] == end) {
                if(k <= 7) {//<=8个命令响应;否则忽略;
                    if((j)&&(valid)) {//两个;间隔小于最小指令长度1就丢弃,无效指令丢弃
                        c[k][1]=j;
                        k++;//一共有k个命令
                    }
                }
                if(k) {//找到对应结束位并且至少有一条有效命令
                    if(end == ']') {
                        result=1;
                    } else if(end == ')') {
                        result=2;
                    }
                }
                //*n=k;
                steps=2;
            } else {
                if(j==0) {//只判断第一个字母有效性，内容不判断
                    if(buf[i] >= 'a' && buf[i] <= 'z') {//首字母转成大写
                        buf[i] -= 'a' - 'A';//??
                    }
                    if(buf[i] == 'D'|| buf[i] == 'N'|| buf[i] == 'P'|| buf[i] == 'R'||
                       buf[i] == 'S'|| buf[i] == 'T'|| buf[i] == 'V'|| buf[i] == 'W'||
                       buf[i] == 'F'|| buf[i] == 'M'|| buf[i] == 'A'|| buf[i] == 'B'){//第一个字母有效

                    } else {//第一个字母无效
                        valid=0;
                    }
                }
                /*{//??F需要重新写
                    if(j) {//后面必须是数字，特殊字母无效
                        valid=0;
                    }
                } else if(buf[i] >= '0' && buf[i] <= '9') {
                    if(j==0) {//开头必须是特定字母，数字无效
                        valid=0;
                    }
                } else {//出现字母和数字之外的字符都会使当前命令无效
                    valid=0;
                }*/
                j++;
            }
        } else {
            break;
        }
    }
    *n=k;
    return result;
}

u8 str_len(u8 *str)
{//计算字符串长度
    u8 i = 0;      
    while ( str[i++] != '\0') {
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

u8 getDir(u8 * buf1,u8 * buf2,u8 num)
{//字符串第一个字符不能是'/'，应该以"0:/"形式开头
    u8 i,j,k,l,m;
    l = strlen((const char *)buf1);
    if (l > 64) l = 64;//限制长度避免溢出
    i = 0;//指向buf1中的位置
    j = 0;//指向buf2中的位置，是有效字符串的长度
    k = 0;//记录目录层数
    while(i < l){//取出buf1中每一个字符进行判断，i数量小于长度l，
        //"0://test//20180101///"
        if (*(buf1 + i) == '/') {
            m = 0;//计数斜杠个数
            while(*(buf1 + i + m + 1) == '/') m++;
            i += m;
            k++;
            if(k >= (num+1)){
                break;
            } else {
                *(buf2 + j) = '/';
                j++;
            }
        } else {
            *(buf2 + j) = *(buf1 + i);
            j++;
        }
        i++;
    }
    
    *(buf2 + j) = '\0';
    
    if (i == (l - 1)){
        if (*(buf1+i) == '/') return 0;
    }
    else if (i < l) return 1;
    
    return 0;
}

//得到磁盘剩余容量
//drv:磁盘编号("0:"/"1:")
//total:总容量     （单位KB）
//free:剩余容量     （单位KB）
//返回值:0,正常.其他,错误代码
u8 exf_getfree(u8 *drv,u32 *total,u32 *free)
{
    FATFS *fs1;
    u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //得到磁盘信息及空闲簇数量
    res =(u32)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
    if(res==0) {                                               
        tot_sect=(fs1->n_fatent-2)*fs1->csize;    //得到总扇区数
        fre_sect=fre_clust*fs1->csize;            //得到空闲扇区数       
#if _MAX_SS!=512                                  //扇区大小不是512字节,则转换为512字节
        tot_sect*=fs1->ssize/512;
        fre_sect*=fs1->ssize/512;
#endif      
        *total=tot_sect>>1;    //单位为KB
        *free=fre_sect>>1;    //单位为KB 
    }
    return res;
}

//void LED_Init(void)
//{    	 
//    GPIO_InitTypeDef  GPIO_InitStructure;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟

//    //GPIOC6,F7初始化设置
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//LED0和LED1对应IO口
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC
//	
//	GPIO_SetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7);//GPIOC6,F7设置高

//}

