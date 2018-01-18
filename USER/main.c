#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
//#include "timer.h"
//#include "ad7799.h"
#include "HC595.h"
#include "stepmoto.h"
#include "w25qxx.h"

#include "FreeRTOS.h"
#include "task.h"
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
#define CRP_TASK_PRIO      6
//任务堆栈大小    
#define CRP_STK_SIZE       256  
//任务句柄
TaskHandle_t CRP_Task_Handler;
//任务函数
void crp_task(void *pvParameters);

//任务优先级
#define START_TASK_PRIO        1
//任务堆栈大小    
#define START_STK_SIZE         128  
//任务句柄
TaskHandle_t Start_Task_Handler;
//任务函数
void start_task(void *pvParameters);

#define HMI_OTHER      0
#define HMI_START      1
#define HMI_CHECK      2
#define HMI_ANALY      3
#define HMI_QUERY      4
#define HMI_QC         5
#define HMI_SET        6
#define HMI_SYS        7
#define HMI_OFF        8

#define CRP_OTHER      0
#define CRP_START      1
#define CRP_CHECK      2
#define CRP_ANALY      3
#define CRP_QC         4
#define CRP_SET        5
#define CRP_SYS        6
#define CRP_OFF        7

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

#define LEVEL          8    //LEVEL设置大小代表遍历的深度，8就代表8层，内存足够的话可以设置更大些

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);
u8 exf_getfree(u8 *drv,u32 *total,u32 *free);
u8 getDir(u8 * buf1,u8 * buf2,u8 pos);
FRESULT scan_files (char* path);
u32 GetPWMCnt;
FATFS fs;
FIL file;                        //文件1
//FIL ftemp;                       //文件2.
UINT br,bw;                      //读写变量
FILINFO fileinfo;                //文件信息
DIR dir;                         //目录

u8 filename[32];//64??
FRESULT fres;
u8 curDir[64]="0:/TEST";//??
u8 curFilename[32]="0001";//??
u32 curFileFlag=0;//0，没有打开；1，文件以读的方式打开；2，文件以写的方式打开
//------------------MAIN 开始------------------
int main(void)
{
    //FRESULT fres;
    //所有硬件初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4，表示支持0-15级抢占优先级，不支持子优先级。
    
    delay_init(168);  //初始化延时函数
    UART1_Init(115200);    //串口初始化波特率为115200
    USART3_Init(9600);//115200
    
//  TIM3_Init();
    
    HC595Init();//??检查时序是否可靠，变化时是否进入临界区
    StepMotoInit();//??检查时序是否可靠，变化时是否进入临界区

    W25QXX_Init();//??检查失败需要返回一个标志，在开机自检时显示出来  if(W25QXX_Init() != W25Q128) error
    
    My_RTC_Init();//设置RTC
    delay_us(2000000);
    fres=f_mount(&fs,"0:",1);                 //立即挂载FLASH.
    if(fres == FR_NO_FILESYSTEM) {            //FLASH磁盘,FAT文件系统错误,重新格式化FLASH
        //??在HMI里设置可以清除所有记录的功能，直接格式化flash
        printf("建立文件系统...\r\n");        //格式化FLASH
        fres=f_mkfs("0:",1,4096);             //格式化FLASH,1,盘符0,不需要引导区,8个扇区为1个簇
        if(fres == FR_OK) {
            //f_setlabel((const TCHAR *)"0:CRP");    //设置Flash磁盘的名字为：CRP
            printf("格式化完成\r\n");         //格式化完成
        } else printf("格式化失败...\r\n");    //格式化失败
        delay_us(1000000);
    }
    fres=f_mkdir((const TCHAR *)curDir);        //创建文件夹//??已经存在会不会冲突
    if(fres == FR_OK) printf("创建了文件夹%s\r\n",curDir);         //创建文件夹完成
    else if(fres == FR_EXIST) printf("文件夹%s已经存在\r\n",curDir);
    else printf("创建文件夹%s失败\r\n",curDir);         //创建文件夹失败
  //printf("DIR大小:%d",sizeof(DIR)*LEVEL);
#if _USE_LFN  //1
    fileinfo.lfname = (TCHAR *)filename;
    fileinfo.lfsize = 32;//文件名长度不能超过32字节
//    fileinfo1.lfname = (TCHAR *)filename1;
//    fileinfo1.lfsize = 32;//文件名长度不能超过32字节
#endif
//    printf("Power On\r\n");
//    printf("数字【%d】表示为【%d】\r\n",1000,pdMS_TO_TICKS(1000));
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
    //创建crp_task任务
    xTaskCreate((TaskFunction_t )crp_task,
                (const char*    )"crp_task",
                (uint16_t       )CRP_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )CRP_TASK_PRIO,
                (TaskHandle_t*  )&CRP_Task_Handler);
                
    vTaskDelete(Start_Task_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}
//u8 x;
//com_task任务函数
void com_task(void *pvParameters)
{//uart1通讯
    u8 tbuf[64];//??__align(4)  //??注意tbuf的大小要能放得下最深的文件名绝对路径
    u8 year,month,date,week;
    u8 hour,min,sec,ampm;
    
    u8 buffer[COM_REC_LEN];//??__align(4) 
    BaseType_t err;
    u8 command[12];//命令为4字节或8字节还有结束位1字节总长度5或9字节，凑成4的整数倍为12字节
    
    u8 c[8][2];//存位置和长度
    u8 num;//存命令个数
    u8 cmdtype;//命令类型，0无效数据，1非阻塞，2阻塞型
    
    u8 i,j,m;//,j,valid;
    //FRESULT fres;
    u32 total,free;
    
    
    u8 l[LEVEL];        //l[]保存每层文件长度，返回上级目录时用
    DIR dir_a[LEVEL];   //FATFS使用的目录结构，只有这个比较占内存需要LEVEL*36字节
    while(1) {
        if(Com_Queue!=NULL) {
            memset(buffer,0,COM_REC_LEN);    //清除缓冲区

            err=xQueueReceive(Com_Queue,buffer,10);//采用非阻塞式  portMAX_DELAY
            if(err == pdTRUE) {//串口命令解析
            //1234[V010;S01cpppp;P01ktttt;R01ctttt;T01b;D0171207;N0143500]
            //(P0130020)
                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);//输入字符串，判断出有效命令位置长度和个数，返回类型cmdtype：0无效数据，1非阻塞，2阻塞型
                if (cmdtype == 1) {//非阻塞命令
                    for(i=0;i<num;i++) {
                        switch(buffer[c[i][0]]) {
                            case 'D':
                                RTC_Get_Time(&hour,&min,&sec,&ampm);//得到时间    
                                RTC_Get_Date(&year,&month,&date,&week);
                                /*if (c[i][1] == 1) {//查询日期
                                    printf("20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                } else */if (c[i][1] == 8) {//设置日期
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
                                }
                                printf("D20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                break;
                            case 'F':
                                if (c[i][1] == 1) {//查询磁盘信息、文件列表  ??可以显示整个磁盘的所有文件夹和文件
                                    exf_getfree((u8*)"0:",&total,&free);
                                    printf("0磁盘总容量：%dKB，剩余：%dKB\r\n", total,free);
                                    //http://www.openedv.com/posts/list/7572.htm
                                    //?? 参考u8 mf_scan_files(u8 * path) 
                                    //??fileinfo.fattrib=AM_DIR AM_ARC
                                    

                                    strcpy((char *)tbuf, (const char *)curDir);
                                    
                                    m = 0;
                                    j = 1;
                                    printf("当前目录:\r\n%s->\r\n", tbuf);
                                    while(1) {                                                 //只有向下一层文件夹搜索时才打开执行
                                        if ( j > m ) {
                                            f_opendir(&dir_a[j-1], (TCHAR *)tbuf);
                                            l[j-1] = strlen((char *)tbuf);
                                        }
                                        m = j;
                                        f_readdir(&dir_a[j-1], &fileinfo);                     //读取目录下的一个文件
                                        if (fileinfo.fname[0] == 0) {                          //到末尾了,退出 //错误了/fres != FR_OK || 
                                            if (j>1) j--;
                                            else break;
                                            tbuf[l[j-1]] = '\0';
                                        }else{
                                            sprintf((char *)tbuf,"%s/%s",tbuf,*fileinfo.lfname ? fileinfo.lfname : fileinfo.fname);//搜索到的文件或文件名连接成完整的路径
                                            if (fileinfo.fattrib & AM_DIR) {                   //是目录
                                                printf("%s [%dD]\r\n", tbuf,j);                //打印目录
                                                if (j<8) j++;
                                            }else {
                                                printf("%s [%dF]\r\n", tbuf,j);                //打印文件
                                                tbuf[l[j-1]] = '\0';
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
                                        
                                        strcpy((char *)curDir,"0:/TEST");
                                        strcpy((char *)curFilename,"0001");
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
                                //vTaskDelay(pdMS_TO_TICKS(20));
                                RTC_Get_Time(&hour,&min,&sec,&ampm);//得到时间    
                                RTC_Get_Date(&year,&month,&date,&week);
                                /*if (c[i][1] == 1) {//查询时间
                                    printf("20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                } else */if (c[i][1] == 8) {//设置时间
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
                                }
                                printf("N20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                break;
                            case 'T':
                                if (c[i][1] == 1) {//查询温度
                                }
                                else if (c[i][1] == 4) {//设置温度
                                }
                                break;
                            case 'V':
                                if ((c[i][1] == 1) || (c[i][1] == 4)) {//查询阀状态  //设置阀开关//Vnnb
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='1';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Valve_Queue,command,10);//向Valve_Queue队列中发送数据
                                }
                                break;
                            case 'S':
                                if ((c[i][1] == 1) || (c[i][1] == 8)) {//查询步进电机状态  //设置步进电机
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='1';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Step_Queue,command,10);//向Step_Queue队列中发送数据
                                }
                                break;
                            case 'P':
                                if ((c[i][1] == 1) || (c[i][1] == 8)) {//查询蠕动泵状态  //设置蠕动泵

                                }
                                break;
                            case 'R':
                                if ((c[i][1] == 1) || (c[i][1] == 8)) {//查询旋转泵状态  //设置旋转泵
                                    
                                }
                                break;
                            default:
                                break;
                        }
                    }
                } else if (cmdtype == 2) {//阻塞命令
                    for(i=0;i<num;i++) {
                        switch(buffer[c[i][0]]) {
                            case 'S':
                                if ((c[i][1] == 1) || (c[i][1] == 8)) {//查询步进电机状态  //设置步进电机
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    command[0]='2';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Step_Queue,command,10);//向Step_Queue队列中发送数据
                                }
                                break;
                            case 'P':
                                if ((c[i][1] == 1) || (c[i][1] == 8)) {//查询蠕动泵状态  //设置蠕动泵

                                }
                                break;
                            case 'R':
                                if ((c[i][1] == 1) || (c[i][1] == 8)) {//查询旋转泵状态  //设置旋转泵
                                    
                                }
                                break;
                            case 'W':
                                if (c[i][1] == 1) {//查询延时状态
                                }
                                else if (c[i][1] == 8) {//设置延时
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
    BaseType_t res;
    u8 year,month,date,week;
    u8 hour,min,sec,time_valid;//,ampm;
    u32 start_task;
    
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
                    Uart3_HMICmd("page start", sizeof("page start")-1);//Uart3_HMICmd("cls RED", sizeof("cls RED")-1);//刷屏
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100  //??打印pdMS_TO_TICKS看看是不是实际值
//                    memcpy(command,"1021", 4);
//                    command[4]='\0';
//                    xQueueSend(Valve_Queue,command,10);//向Valve_Queue队列中发送数据
//                    memcpy(command,"10140400", 8);
//                    command[8]='\0';
//                    xQueueSend(Step_Queue,command,10);//向Valve_Queue队列中发送数据
                    start_task=0;
                    time_valid=0;
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    //需要完成：从HMI读取当前时间赋值给RTC，步进电机运动，光电信号（光隔和红光）测试，阀，泵等检测
                    //1：从HMI读取当前时间赋值给RTC
                    if(start_task == 0){
                        Uart3_HMICmd("get rtc0", sizeof("get rtc0")-1);
                        start_task++;
                    } else if(start_task < 10){
                        if((res == pdTRUE) && (buffer[0] == 0x71)){//收到有效数据
                            if((buffer[1]+buffer[2]*256)>=2000) year=(buffer[1]+buffer[2]*256)-2000;
                            else year = 0;
                            time_valid |= (1<<0);
                            start_task=10;
                        } else {//无数据
                            start_task++;
                        }
                    } else if(start_task == 10){
                        Uart3_HMICmd("get rtc1", sizeof("get rtc1")-1);
                        start_task++;
                    } else if(start_task < 20){
                        if((res == pdTRUE) && (buffer[0] == 0x71)){//收到有效数据
                            month=buffer[1];
                            time_valid |= (1<<1);
                            start_task=20;
                        } else {//无数据
                            start_task++;
                        }
                    } else if(start_task == 20){
                        Uart3_HMICmd("get rtc2", sizeof("get rtc2")-1);
                        start_task++;
                    } else if(start_task < 30){
                        if((res == pdTRUE) && (buffer[0] == 0x71)){//收到有效数据
                            date=buffer[1];
                            time_valid |= (1<<2);
                            start_task=30;
                        } else {//无数据
                            start_task++;
                        }
                    } else if(start_task == 30){
                        Uart3_HMICmd("get rtc3", sizeof("get rtc3")-1);
                        start_task++;
                    } else if(start_task < 40){
                        if((res == pdTRUE) && (buffer[0] == 0x71)){//收到有效数据
                            hour=buffer[1];
                            time_valid |= (1<<3);
                            start_task=40;
                        } else {//无数据
                            start_task++;
                        }
                    } else if(start_task == 40){
                        Uart3_HMICmd("get rtc4", sizeof("get rtc4")-1);
                        start_task++;
                    } else if(start_task < 50){
                        if((res == pdTRUE) && (buffer[0] == 0x71)){//收到有效数据
                            min=buffer[1];
                            time_valid |= (1<<4);
                            start_task=50;
                        } else {//无数据
                            start_task++;
                        }
                    } else if(start_task == 50){
                        Uart3_HMICmd("get rtc5", sizeof("get rtc5")-1);
                        start_task++;
                    } else if(start_task < 60){
                        if((res == pdTRUE) && (buffer[0] == 0x71)){//收到有效数据
                            sec=buffer[1];
                            time_valid |= (1<<5);
                            start_task=60;
                        } else {//无数据
                            start_task++;
                        }
                    } else if(start_task == 60){
                        Uart3_HMICmd("get rtc6", sizeof("get rtc6")-1);
                        start_task++;
                    } else if(start_task < 70){
                        if((res == pdTRUE) && (buffer[0] == 0x71)){//收到有效数据
                            week=buffer[1];//??星期日是7还是0
                            time_valid |= (1<<6);
                            start_task=70;
                        } else {//无数据
                            start_task++;
                        }
                    } else {
                        if(time_valid == 0x7F){//??需要判断time_valid为127才能表示时间数据有效
                            //printf("time_valid:%d\r\n",time_valid);
                            printf("\r\n时间有效20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                            year=CorrectYear(year);
                            CorrectDate(year,&month,&date);
                            CorrectTime(&hour,&min,&sec);
                            week=RTC_Get_Week(2000 + year,month,date);
                            if(hour < 12)//?? 
                                RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //设置时间
                            else
                                RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //设置时间
                            RTC_Set_Date(year,month,date,week);    //设置日期
                        }//否则显示读取时间失败
                        NextState = HMI_CHECK;
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));//至少保证20 的HMI反馈时间
                    //printf("start_task:%d\r\n",start_task);
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;
                    vTaskDelay(pdMS_TO_TICKS(2000));//至少保证10
                STATE_END                                                       //} break;
            case HMI_CHECK:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    Uart3_HMICmd("page check", sizeof("page check")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//至少保证100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    //需要完成：清洗等
                    
                    NextState = HMI_ANALY;
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;
                    vTaskDelay(pdMS_TO_TICKS(2000));//至少保证10
                STATE_END                                                       //} break;
           case HMI_ANALY:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    Uart3_HMICmd("page analyse_0", sizeof("page analyse_0")-1);
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
                            Uart3_HMICmd("page analyse_0", sizeof("page analyse_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_QUERY:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    Uart3_HMICmd("page query_0", sizeof("page query_0")-1);
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
                            Uart3_HMICmd("page query_0", sizeof("page query_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_QC:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    Uart3_HMICmd("page qc_0", sizeof("page qc_0")-1);
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
                            Uart3_HMICmd("page qc_0", sizeof("page qc_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_SET:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    Uart3_HMICmd("page set_0", sizeof("page set_0")-1);
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
                            Uart3_HMICmd("page set_0", sizeof("page set_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_SYS:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    Uart3_HMICmd("page system_0", sizeof("page system_0")-1);
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
                            Uart3_HMICmd("page system_0", sizeof("page system_0")-1);
                            NextState = HMI_ANALY;
                        }
                    }
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case HMI_OFF:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    Uart3_HMICmd("page poweroff_0", sizeof("page poweroff_0")-1);
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
                            Uart3_HMICmd("page poweroff_0", sizeof("page poweroff_0")-1);
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
    
    while(1) {
        switch(CurrentState) {
//            case 0:
//                CurrentState = COM_WAIT;
//                NextState    = COM_WAIT;
//                break;
            case 1:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    if(Valve_Queue!=NULL) {
                        memset(buffer,0,CMD_LEN);    //清除缓冲区

                        err=xQueueReceive(Valve_Queue,buffer,10);//采用非阻塞式  portMAX_DELAY
                        if(err == pdTRUE) {//执行阀控制命令
                            printf("阀控制命令:%s\r\n",buffer);
                            //V090
                            bits=(buffer[1]-'0')*10+buffer[2]-'0';
                            if (bits) {//bits>0相应的阀动作
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
                vTaskDelay(pdMS_TO_TICKS(500));
                valve_state = 0;//所有阀关闭
                printf("阀状态:%#x\r\n",valve_state);
                SwitchOut(valve_state);
                vTaskDelay(pdMS_TO_TICKS(500));
                CurrentState = 1;
                NextState    = 1;
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

    while(1) {
        if(Step_Queue!=NULL) {
            memset(buffer,0,CMD_LEN);    //清除缓冲区

            err=xQueueReceive(Step_Queue,buffer,10);//采用非阻塞式  portMAX_DELAY
            if(err == pdTRUE) {//执行步进电机控制命令
                printf("步进电机控制命令:%s\r\n",buffer);
                num=(buffer[1]-'0')*10+buffer[2]-'0';
                switch(num) {
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


void crp_task(void *pvParameters)
{
//    u8 year,month,date,week;
//    u8 hour,min,sec,ampm;
//    u32 PreviousState = CRP_START;
//    u32 CurrentState  = CRP_START;
//    u32 NextState     = CRP_START;
    while(1) {
//        vTaskDelay(5000);
//        RTC_Get_Time(&hour,&min,&sec,&ampm);//得到时间            
//        RTC_Get_Date(&year,&month,&date,&week);
//        printf("20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);

        /*
        switch(CurrentState)
        {
            case CRP_START:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
            case CRP_CHECK:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case CRP_ANALY:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case CRP_QC:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case CRP_SET:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case CRP_SYS:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case CRP_OFF:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;

                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {

                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;

                STATE_END                                                       //} break;
           case CRP_OTHER:
           default:
                CurrentState = 1;
                NextState    = 1;
                break;

        }*/
        
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
                       buf[i] == 'F'){//第一个字母有效

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
{
    u8 i,j,k,l,m;
    l=strlen((const char *)buf1);
    i=0;
    j=0;
    k=0;
    while(i<l){//i数量小于长度l
        //"0://test//20180101///"
        if (*(buf1+i) == '/') {
            m=0;
            while(*(buf1+i+m+1) == '/') m++;
            i+=m;
            k++;
            if(k >= (num+1)){
                break;
            } else {
                *(buf2+j)='/';
                j++;
            }
        }
        else {
            *(buf2+j)=*(buf1+i);
            j++;
        }
        i++;
    }
    
    *(buf2+j)='\0';
    
    if (i == (l-1)){
        if (*(buf1+i) == '/') return 0;
    }
    else if (i<l) return 1;
    
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

//FRESULT scan_files (char* path)
//{
//    FRESULT res;
//    u32 i[8],j;//最多找8层
//    char fn[64],tmppath[64];
//    strcpy((char *)tmppath,(char *)path);
//    for(j=0;j<8;j++) {
//        res = f_opendir(&dir, (TCHAR*)tmppath);
//        if (res == FR_OK) {
//            i[j] = strlen(tmppath);
//            {
//                res = f_readdir(&dir, &fileinfo);
//                if (res != FR_OK || fileinfo.fname[0] == 0) break;
//                if (fileinfo.fname[0] == '.') continue;
//#if _USE_LFN
//                strcpy((char *)fn,*fileinfo.lfname ? fileinfo.lfname : fileinfo.fname);//strcat((char *)fn, fileinfo.lfname);
//#else          
//                strcpy((char *)fn,fileinfo.fname);//strcat((char *)fn, fileinfo.fname);//短文件名：TT2018~1.TXT
//#endif
//                if (fileinfo.fattrib & AM_DIR) {//是目录
//                    strcat(tmppath,"/");
//                    strcat(tmppath,fn);
//                    //sprintf(&tmppath[i], "/%s", fn);
//                    printf("%s\n", tmppath);
//                    res = scan_files(tmppath);
//                    if (res != FR_OK) break;
//                    tmppath[i[j]] = 0;

//                } else {
//                    printf("%s/%s\n", tmppath, fn);
//                }
//            }
//        }
//    }

//   return res;
//}
