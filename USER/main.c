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

//??��ջ��С��Ҫ����,���ȼ���Ҫȷ��

//!��ֵԽ�����ȼ�Խ��
//�������ȼ�
#define COM_TASK_PRIO        10
//�����ջ��С    
#define COM_STK_SIZE         512  
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
#define MOTION_TASK_PRIO      8
//�����ջ��С    
#define MOTION_STK_SIZE       256  
//������
TaskHandle_t Motion_Task_Handler;
//������
void motion_task(void *pvParameters);

////�������ȼ�
//#define STEP_TASK_PRIO      7
////�����ջ��С    
//#define STEP_STK_SIZE       256  
////������
//TaskHandle_t Step_Task_Handler;
////������
//void step_task(void *pvParameters);

////�������ȼ�
//#define CRP_TASK_PRIO      6
////�����ջ��С    
//#define CRP_STK_SIZE       256  
////������
//TaskHandle_t CRP_Task_Handler;
////������
//void crp_task(void *pvParameters);

//�������ȼ�
#define START_TASK_PRIO        1
//�����ջ��С    
#define START_STK_SIZE         128  
//������
TaskHandle_t Start_Task_Handler;
//������
void start_task(void *pvParameters);

//??HMI��Ҫ��д��������ҳ��Ϊ״̬�������յ�������Ϊ״̬
#define HMI_OTHER      0
#define HMI_INIT       1
#define HMI_CHECK      2
#define HMI_TEST       3
#define HMI_QUERY      4
#define HMI_QC         5
#define HMI_SET        6
#define HMI_SYS        7
#define HMI_OFF        8

#define READ_RES         0X81
#define RTC_NOW          0x20

#define READ_MEM         0x83
#define MENU1            0x0160
#define MENU1_ANALY      0x2930
#define MENU1_QUERY      0x2131
#define MENU1_QC         0x4032
#define MENU1_SET        0x2333
#define MENU1_SYS        0x2434
#define MENU1_OFF        0x2535

#define COM_Q_NUM      4                           //�������ݵ���Ϣ���е����� 
#define HMI_Q_NUM     16                           //�������ݵ���Ϣ���е����� 

#define CMD_LEN       12                           //�����������  //����Ϊ4�ֽڻ�8�ֽڻ��н���λ1�ֽ��ܳ���5��9�ֽڣ��ճ�4��������Ϊ12�ֽ�

#define MOTION_Q_NUM  16                           //�˶������������Ϣ���е����� 

//#define VALVE_Q_NUM   16                           //���ͷ������������Ϣ���е����� 
//#define STEP_Q_NUM    16                           //���Ͳ�����������������Ϣ���е����� 
//#define PUMP_Q_NUM     8                           //���ͱÿ����������Ϣ���е����� 

#define ReadRTCLen 6
#define ToPageNLen 7
char ReadRTC[8]={0x5A,0xA5,0x03,0x81,0x20,0x07,0x00,0x00};
char ToPageN[8]={0x5A,0xA5,0x04,0x80,0x03,0x00,0x01,0x00};

//char RECORDS[]="A0001����CRP0001[  180119093625   ]U000000";
#define RECADDR   0x0360
char RECORDS[]="\x5A\xA5\x31\x82\x01\x60 A0001 ����CRP0001[   180119093625   ] U000000";

QueueHandle_t Com_Queue;                           //������Ϣ���о��
QueueHandle_t Hmi_Queue;                           //HMI��Ϣ���о��
QueueHandle_t Motion_Queue;                        //�˶�������Ϣ���о��
//QueueHandle_t Valve_Queue;                         //��������Ϣ���о��
//QueueHandle_t Step_Queue;                          //�������������Ϣ���о��
//QueueHandle_t Pump_Queue;                          //�ÿ�����Ϣ���о��   ??�䶯�õ�����������ã���ת�õ�������

struct motion_state{
    u8 motion_type;//�˶����ͣ�0�����˶���1���������˶���2�������˶���3����λ��4����ͣ��ȡ��
    u8 current_motion;//��ǰ�˶�Ŀ�꣺'A'����λ�˶���'S'�����������'V'������'R'���䶯�ã�'P'����ת��
    //u8 suspend;//����
    u16 step_flag;//ÿһλ����һ�����������1���0��ֻ��һ����1??   �䶯�ù鵽���������
    u16 valve_flag;//ÿһλ����һ������1���0�����Զ��1           ��ת�ù鵽����
}motion_state1;//??�˶�״̬��������

struct record_inf{
    u8 time[24];//ʱ��
    u8 serial[8];//���
    u8 number[8];//���
    u8 result[8];//���
}record_inf1;

//�����ź������
SemaphoreHandle_t MutexSemMotion;	//�˶����ƻ����ź���??ֻ���˶�����ģ��
//??printfӦ��һ�������У�����Ҫ�ӻ����ź���

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);
u8 exf_getfree(u8 *drv,u32 *total,u32 *free);
u8 getDir(u8 * buf1,u8 * buf2,u8 pos);
void KEY_Init(void);
void MotionReset(void);
void string_cpy(char * buf1,char * buf2,u8 len);
u8 ReadRecords(char * tbuf,DIR * dir_a,u8 * l,u8 *m);

FATFS fs;
FIL file;                                          //�ļ�
//UINT br,bw;                                        //��д����
FILINFO fileinfo;                                  //�ļ���Ϣ
DIR dir;                                           //Ŀ¼

#define LEVEL             2                        //LEVEL���ô�С�����������ȣ�8�ʹ���8�㣬�ڴ��㹻�Ļ��������ø���Щ
#define WORKPATH          "0:/TEST"                //Ĭ�ϴ����Ĺ���Ŀ¼0:/CRP
#define WORKFILE          "setting"                //Ĭ�ϵ�ǰ���ļ�
#define MAXNAMELEN        32                       //�ļ����Ϊ32�ֽ�

u8 filename[MAXNAMELEN];//32??
FRESULT fres;
u8 curDir[64]=WORKPATH;//??
u8 curFilename[MAXNAMELEN]=WORKFILE;//??
u32 curFileFlag=0;//0��û�д򿪣�1���ļ��Զ��ķ�ʽ�򿪣�2���ļ���д�ķ�ʽ��

u32 HMICurMenu=HMI_OTHER;
u32 HMICurSubMenu;
//------------------MAIN ��ʼ------------------
int main(void)
{

    //����Ӳ����ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4����ʾ֧��0-15����ռ���ȼ�����֧�������ȼ���
    
    delay_init(168);                               //��ʼ����ʱ����
    
    EXTIX_Init();
    
    UART1_Init(115200);                            //����1��ʼ��������Ϊ115200�����ڴ���ͨѶ
    USART3_Init(115200);                           //����3��ʼ��������Ϊ115200������HMI ͨѶ
    
    KEY_Init();
    
    TIM2_PWM_Init();                               //����LED�����ź�1120Hz PB11
    AD7799_Init(ADC_CON_GAIN1);                    //��ʼ��AD7799,����SPI2��ʼ��//??Ƶ��ѡ��ʱ���..
    
    HC595Init();                                   //??���ʱ���Ƿ�ɿ����仯ʱ�Ƿ�����ٽ���
    SwitchOut(0);
    motion_state1.valve_flag=0;
    StepMotoInit();                                //??���ʱ���Ƿ�ɿ����仯ʱ�Ƿ�����ٽ���
//StepMotoCal(1100,392,50);
//StepMotoCal(2000,1000,50);

    W25QXX_Init();                                 //??���ʧ����Ҫ����һ����־���ڿ����Լ�ʱ��ʾ����  if(W25QXX_Init() != W25Q128) error
    
    My_RTC_Init();                                 //����RTC
    delay_us(2000000);                             //??Ϊ�˷���ʱ��ʱ����ʽʱɾ��

    fres=f_mount(&fs,"0:",1);                      //��������FLASH.
    if(fres == FR_NO_FILESYSTEM) {                 //FLASH����,FAT�ļ�ϵͳ����,���¸�ʽ��FLASH
        printf("�����ļ�ϵͳ...\r\n");             //��ʽ��FLASH
        fres=f_mkfs("0:",1,4096);                  //��ʽ��FLASH,1,�̷�0,����Ҫ������,8������Ϊ1����
        if(fres == FR_OK) {
            //f_setlabel((const TCHAR *)"0:CRP");    //����Flash���̵�����Ϊ��CRP
            printf("��ʽ�����\r\n");               //��ʽ�����
        } else printf("��ʽ��ʧ��...\r\n");         //��ʽ��ʧ��
        delay_us(1000000);
    }
    fres=f_mkdir((const TCHAR *)curDir);           //�����ļ���
    if(fres == FR_OK) printf("�½�Ŀ¼%s\r\n",curDir);//�����ļ������
    else if(fres == FR_EXIST) printf("�ļ���%s�Ѿ�����\r\n",curDir);
    else printf("����Ŀ¼%sʧ��\r\n",curDir);       //�����ļ���ʧ��

    fileinfo.lfname = (TCHAR *)filename;           //Ϊ���ļ�������ռ�
    fileinfo.lfsize = MAXNAMELEN;                  //�ļ������Ȳ��ܳ���32�ֽ�

//char tbuf[64];
//u8 m[2],l[2],res;
//DIR dir_a[2];
//strcpy(tbuf, WORKPATH);
//m[0] = 0;
//m[1] = 1;
//do{
//    res=ReadRecords(tbuf,dir_a,l,m);
//} while((res==1)||(res==0));
    
    //������ʼ����
    xTaskCreate((TaskFunction_t )start_task,             //������
                (const char*    )"start_task",           //��������
                (uint16_t       )START_STK_SIZE,         //�����ջ��С
                (void*          )NULL,                   //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t*  )&Start_Task_Handler);   //������
    vTaskStartScheduler();    
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();                          //�����ٽ���
    
    //������Ϣ����
    Com_Queue=xQueueCreate(COM_Q_NUM,COM_REC_LEN); //������ϢCom_Queue�����մ������������4�������63
    Hmi_Queue=xQueueCreate(HMI_Q_NUM,HMI_REC_LEN); //������ϢHmi_Queue������HMI ���������16�������15
    
    Motion_Queue=xQueueCreate(MOTION_Q_NUM,CMD_LEN);   //������ϢMotion_Queue�����ղ���������������16�������12
    //Valve_Queue=xQueueCreate(VALVE_Q_NUM,CMD_LEN); //������ϢValve_Queue�����շ����������16�������12
    //Step_Queue=xQueueCreate(STEP_Q_NUM,CMD_LEN);   //������ϢValve_Queue�����ղ���������������16�������12
    //Pump_Queue=xQueueCreate(PUMP_Q_NUM,CMD_LEN); //������ϢValve_Queue

   	//���������ź���
	MutexSemMotion=xSemaphoreCreateMutex();
    
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
    //����motion_task����
    xTaskCreate((TaskFunction_t )motion_task,
                (const char*    )"motion_task",
                (uint16_t       )MOTION_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )MOTION_TASK_PRIO,
                (TaskHandle_t*  )&Motion_Task_Handler);
//    //����valve_task����
//    xTaskCreate((TaskFunction_t )valve_task,
//                (const char*    )"valve_task",
//                (uint16_t       )VALVE_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )VALVE_TASK_PRIO,
//                (TaskHandle_t*  )&Valve_Task_Handler);
//    //����step_task����
//    xTaskCreate((TaskFunction_t )step_task,
//                (const char*    )"step_task",
//                (uint16_t       )STEP_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )STEP_TASK_PRIO,
//                (TaskHandle_t*  )&Step_Task_Handler);
//    //����crp_task����
//    xTaskCreate((TaskFunction_t )crp_task,
//                (const char*    )"crp_task",
//                (uint16_t       )CRP_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )CRP_TASK_PRIO,
//                (TaskHandle_t*  )&CRP_Task_Handler);

    vTaskDelete(Start_Task_Handler);               //ɾ����ʼ����
    taskEXIT_CRITICAL();                           //�˳��ٽ���
}


void com_task(void *pvParameters)
{//uart1ͨѶ
    u8 tbuf[64];                                   //??__align(4)  //??ע��tbuf�Ĵ�СҪ�ܷŵ���������ļ�������·��
    u8 year,month,date,week;
    u8 hour,min,sec,ampm;
    
    u8 buffer[COM_REC_LEN];                        //??__align(4) 
    BaseType_t err;
    u8 command[12];                                //����Ϊ4�ֽڻ�8�ֽڻ��н���λ1�ֽ��ܳ���5��9�ֽڣ��ճ�4��������Ϊ12�ֽ�
    
    u8 c[8][2];                                    //��λ�úͳ���
    u8 num;                                        //���������
    u8 cmdtype;                                    //�������ͣ�0��Ч���ݣ�1��������2������
    
    u8 i,j,m;//,j,valid;
    u32 total,free;
    
    u8 l[LEVEL];                                   //l[]����ÿ���ļ����ȣ������ϼ�Ŀ¼ʱ��
    DIR dir_a[LEVEL];                              //FATFSʹ�õ�Ŀ¼�ṹ��ֻ������Ƚ�ռ�ڴ���ҪLEVEL*36�ֽ�
    
    
    u8 bits;

    u16 c0,c1,a;
    s32 steps;
    
    while(1) {
        if(Com_Queue!=NULL) {
            memset(buffer,0,COM_REC_LEN);          //���������

            err=xQueueReceive(Com_Queue,buffer,10);//���մ���������÷�����ʽ  portMAX_DELAY
            if(err == pdTRUE) {//�����������
            //1234[V010;S01cpppp;P01ktttt;R01b;T01b;D0171207;N0143500]
            //(P0130020)
                cmdtype=decodeCmd(buffer,str_len(buffer),c,&num);//�����ַ������жϳ���Ч����λ�ó��Ⱥ͸�������������cmdtype��0��Ч���ݣ�1��������2������
                if (cmdtype == 1) {//����������[]��ֱ�Ӵ���
                    for(i=0;i<num;i++) {
                        switch(buffer[c[i][0]]) {
                            case 'D':
                                //??��ȡʱ��֮ǰ����Ҫ����HMI��ȡʱ��
                                RTC_Get_Time(&hour,&min,&sec,&ampm);//�õ�ʱ��    
                                RTC_Get_Date(&year,&month,&date,&week);
                                if (c[i][1] == 8) {//��������
                                    year=(buffer[c[i][0]+2]-'0')*10+buffer[c[i][0]+3]-'0';
                                    month=(buffer[c[i][0]+4]-'0')*10+buffer[c[i][0]+5]-'0';
                                    date=(buffer[c[i][0]+6]-'0')*10+buffer[c[i][0]+7]-'0';
                                    year=CorrectYear(year);
                                    CorrectDate(year,&month,&date);
                                    CorrectTime(&hour,&min,&sec);
                                    week=RTC_Get_Week(2000 + year,month,date);
                                    if(hour < 12)//??
                                        RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //����ʱ��
                                    else
                                        RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //����ʱ��
                                    RTC_Set_Date(year,month,date,week);    //��������
                                    //????��Ҫ����HMI��ʱ��
                                    
                                    
                                }
                                printf("D20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                break;
                            case 'F':
                                if (c[i][1] == 1) {//��ѯ������Ϣ���ļ��б�
                                    exf_getfree((u8*)"0:",&total,&free);
                                    printf("0������������%dKB��ʣ�ࣺ%dKB\r\n", total,free);

                                    strcpy((char *)tbuf, (const char *)curDir);
                                    
                                    m = 0;
                                    j = 1;
                                    printf("��ǰĿ¼:\r\n%s->\r\n", tbuf);
                                    while(1) {
                                        if ( j > m ) {                                         //ֻ��������Ŀ¼ʱ��ִ��
                                            f_opendir(&dir_a[j-1], (TCHAR *)tbuf);
                                            l[j-1] = strlen((char *)tbuf);
                                        }
                                        m = j;
                                        f_readdir(&dir_a[j-1], &fileinfo);                     //��ȡĿ¼�µ�һ���ļ�
                                        if (fileinfo.fname[0] == 0) {                          //��ĩβ��,�˳� //������/fres != FR_OK || 
                                            if (j>1) j--;                                      //�¸�ѭ�����븸Ŀ¼
                                            else break;
                                            tbuf[l[j-1]] = '\0';                               //�洢��·�������ϼ�Ŀ¼
                                        }else{
                                            sprintf((char *)tbuf,"%s/%s",tbuf,*fileinfo.lfname ? fileinfo.lfname : fileinfo.fname);//���������ļ����ļ������ӳ�������·��
                                            if (fileinfo.fattrib & AM_DIR) {                   //��Ŀ¼
                                                printf("%s [%dD]\r\n", tbuf,j);                //��ӡĿ¼
                                                if (j<LEVEL) j++;                              //�¸�ѭ��������Ŀ¼
                                            }else {
                                                printf("%s [%dF]\r\n", tbuf,j);                //��ӡ�ļ�
                                                tbuf[l[j-1]] = '\0';                           //�洢��·������Ŀ¼
                                            }
                                        }
                                    }
                                } else if (c[i][1] == 2) {//F0,F1,F2,F3,F4
                                    if (buffer[c[i][0]+1]=='0'){//��
                                        if(curFileFlag==0) {
                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("��ǰ�ļ���:%s,��־:%d\r\n", tbuf,curFileFlag);//��ӡ�ļ��� 
                                            fres = f_open(&file, (const TCHAR*)tbuf, FA_OPEN_EXISTING | FA_READ);
                                            if(fres == FR_OK) {//FR_OK ���ļ��ɹ�  ??�ļ���������
                                                printf("�ļ�����:->\r\n");
                                                do{
                                                    //fres = f_read(&file, tbuf, 60, &br);//��ȡ60���ֽڣ�br���ض�ȡ���ֽ���
                                                    if(f_gets((TCHAR *)tbuf,60,&file) != 0)//��ȡһ�� ??\n����\0�� \r��ô��??
                                                        printf("%s", tbuf);
                                                }while ((!f_eof(&file))&&(!f_error(&file)));//??û�н�����û�з�������
                                                fres=f_close(&file);
                                            } else if (fres == FR_NO_FILE){
                                                printf("�ļ�������\r\n");
                                            }
                                        }
                                    } else if (buffer[c[i][0]+1]=='1'){//д
                                        if(curFileFlag==2) {
                                            curFileFlag=0;
                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("��ǰ�ļ���:%s,��־:%d\r\n", tbuf,curFileFlag);//��ӡ�ļ��� 
                                            fres=f_close(&file);
                                        }
                                    } else if (buffer[c[i][0]+1]=='2'){//ɾ??
                                        if(curFileFlag==0) {
                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("��ǰ�ļ���:%s,��־:%d\r\n", tbuf,curFileFlag);//��ӡ�ļ��� 
                                            fres = f_unlink((const TCHAR*)tbuf);
                                            if (fres == FR_OK)
                                                printf("ɾ���ɹ�\r\n");//ɾ���ɹ�
                                            else if (fres == FR_NO_FILE)
                                                printf("�ļ�������\r\n");//ɾ���ɹ�
                                            else
                                                printf("ɾ��ʧ��\r\n");//ɾ��ʧ��
                                        }
                                    } else if (buffer[c[i][0]+1]=='3'){//���ص�ǰ�ļ���
                                        printf("��ǰ�ļ���:%s,��־:%d\r\n", curDir,curFileFlag);//��ӡ�ļ���
                                    } else if (buffer[c[i][0]+1]=='4'){//��ʽ������
                                        printf("�����ļ�ϵͳ...\r\n");        //��ʽ��FLASH
                                        fres=f_mkfs("0:",1,4096);             //��ʽ��FLASH,1,�̷�0,����Ҫ������,8������Ϊ1����
                                        if(fres == FR_OK) {
                                            //f_setlabel((const TCHAR *)"0:CRP");    //����Flash���̵�����Ϊ��CRP
                                            printf("��ʽ�����\r\n");         //��ʽ�����
                                        } else printf("��ʽ��ʧ��...\r\n");    //��ʽ��ʧ��
                                        vTaskDelay(pdMS_TO_TICKS(1000));
                                        
                                        strcpy((char *)curDir,WORKPATH);
                                        strcpy((char *)curFilename,WORKFILE);
                                        curFileFlag=0;
                                        fres=f_mkdir((const TCHAR *)curDir);        //�����ļ���//??�Ѿ����ڻ᲻���ͻ
                                        if(fres == FR_OK) printf("�������ļ���%s\r\n",curDir);         //�����ļ������
                                        else if(fres == FR_EXIST) printf("�ļ���%s�Ѿ�����\r\n",curDir);
                                        else printf("�����ļ���%sʧ��\r\n",curDir);         //�����ļ���ʧ��
                                        printf("��ǰ�ļ���%s/%s,��־:%d\r\n", curDir,curFilename,curFileFlag);                                        
                                    }
                                } else if (c[i][1] > 3) {//����д��ɾ���ļ� 
                                    if(buffer[c[i][0]+1]=='0'){//�� [F0,A00010001U000000]  ??,�ж�
                                        if(curFileFlag==0) {
                                            if(c[i][1]-3 < 32) {//����С��32
                                                memcpy(curFilename, buffer+c[i][0]+3, c[i][1]-3);
                                                curFilename[c[i][1]-3]='\0';//??���һλ����©��
                                            } else {//���ȴ��ڵ���31�����涪��
                                                memcpy(curFilename, buffer+c[i][0]+3, 31);
                                                curFilename[31]='\0';//
                                            }

                                            sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                            printf("��ǰ�ļ���:%s,��־:%d\r\n", tbuf,curFileFlag);//��ӡ�ļ��� 
                                            fres = f_open(&file, (const TCHAR*)tbuf, FA_OPEN_EXISTING | FA_READ);
                                            if(fres == FR_OK) {//FR_OK ���ļ��ɹ�  ??�ļ���������
                                                printf("�ļ�����:->\r\n");
                                                do{
                                                    //fres = f_read(&file, tbuf, 60, &br);//��ȡ60���ֽڣ�br���ض�ȡ���ֽ���
                                                    if(f_gets((TCHAR *)tbuf,60,&file) != 0)//��ȡһ�� ??\n����\0�� \r��ô��??
                                                        printf("%s", tbuf);
                                                }while ((!f_eof(&file))&&(!f_error(&file)));//??û�н�����û�з�������
                                                fres=f_close(&file);
                                            } else if (fres == FR_NO_FILE){
                                                printf("�ļ�������\r\n");
                                            }
                                        }
                                    } else if(buffer[c[i][0]+1]=='1'){//д?? [F1,ABCDEFG]
                                        sprintf((char *)tbuf,"%s/%s", curDir,curFilename);
                                        printf("��ǰ�ļ���:%s,��־:%d\r\n", tbuf,curFileFlag);//��ӡ�ļ��� 
                                        if(curFileFlag==0) {//��һ��д�ȴ��ļ�
                                            fres = f_open(&file, (const TCHAR*)tbuf, FA_CREATE_ALWAYS | FA_WRITE);//??�����ĻḲ�ǣ�û�е��ļ���ֱ�Ӵ���
                                            if(fres == FR_OK) {
                                                curFileFlag=2;
                                                memcpy(tbuf, buffer+c[i][0]+3, c[i][1]-3);//??
                                                tbuf[c[i][1]-3]='\r';
                                                tbuf[c[i][1]-2]='\n';
                                                tbuf[c[i][1]-1]='\0';
                                                //fres=f_write(&file,buffer+c[i][0]+3,c[i][0]-3,&bw);
                                                f_puts((TCHAR *)tbuf,&file);//??
                                                printf("д������:%s\r\n",tbuf);
                                            }
                                        } else if(curFileFlag==2) {//��n��д���ô��ļ� 
                                                memcpy(tbuf, buffer+c[i][0]+3, c[i][1]-3);//??
                                                tbuf[c[i][1]-3]='\r';
                                                tbuf[c[i][1]-2]='\n';
                                                tbuf[c[i][1]-1]='\0';
                                            //fres=f_write(&file,buffer+c[i][0]+3,c[i][0]-3,&bw);
                                            f_puts((TCHAR *)tbuf,&file);//??
                                            printf("д������:%s\r\n",tbuf);
                                        }
                                    } else if(buffer[c[i][0]+1]=='3'){//���õ�ǰ�ļ��� [F3,0:/CRP]
                                        if(curFileFlag==0) {
                                            if(c[i][1]-3 < 64) {//����С��64
                                                memcpy(curDir, buffer+c[i][0]+3, c[i][1]-3);
                                                curDir[c[i][1]-3]='\0';//??���һλ����©��
                                            } else {//���ȴ��ڵ���63�����涪��
                                                memcpy(curDir, buffer+c[i][0]+3, 63);
                                                curDir[63]='\0';//
                                            }

                                            if(strlen((const TCHAR *)curDir)<=3) {
                                                strcpy ((char *)curDir, "0:");
                                                printf("��ǰ�Ǹ�Ŀ¼%s\r\n",curDir);
                                            } else {
                                                m=1;
                                                do{
                                                    j=getDir(curDir,tbuf,m);
                                                    fres=f_mkdir((const TCHAR *)tbuf);        //�����ļ���//??�Ѿ����ڻ᲻���ͻ//??һ�ο��Դ�������ļ�����
                                                    if(fres == FR_OK) printf("�����ļ���%s\r\n",tbuf);         //�����ļ������
                                                    else if(fres == FR_EXIST) printf("�ļ���%s�Ѿ�����\r\n",tbuf);
                                                    else printf("�����ļ���%sʧ��\r\n",tbuf);         //�����ļ���ʧ��
                                                    m++;
                                                }while(j);
                                                strcpy((char *)curDir,(const char *)tbuf);
                                                printf("��ǰĿ¼:%s,��־:%d\r\n", curDir,curFileFlag);//��ӡĿ¼
                                            }
                                        }
                                    }
                                }
                                break;
                            case 'N':
                                //??��ȡʱ��֮ǰ����Ҫ����HMI��ȡʱ��
                                RTC_Get_Time(&hour,&min,&sec,&ampm);//�õ�ʱ��    
                                RTC_Get_Date(&year,&month,&date,&week);
                                if (c[i][1] == 8) {//����ʱ��
                                    hour=(buffer[c[i][0]+2]-'0')*10+buffer[c[i][0]+3]-'0';
                                    min=(buffer[c[i][0]+4]-'0')*10+buffer[c[i][0]+5]-'0';
                                    sec=(buffer[c[i][0]+6]-'0')*10+buffer[c[i][0]+7]-'0';
                                    year=CorrectYear(year);
                                    CorrectDate(year,&month,&date);
                                    CorrectTime(&hour,&min,&sec);
                                    week=RTC_Get_Week(2000 + year,month,date);
                                    if(hour < 12)//??
                                        RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //����ʱ��
                                    else
                                        RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //����ʱ��
                                    RTC_Set_Date(year,month,date,week);    //��������
                                    //????��Ҫ����HMI��ʱ��
                                    
                                    
                                }
                                printf("N20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);
                                break;
                            case 'T':
                                if (c[i][1] == 1) {//��ѯ�¶�
                                }
                                else if (c[i][1] == 4) {//�����¶�
                                }
                                break;
                            case 'M':
                                //��ѯ״̬
                                printf("�˶����ͣ�%d\r\n",motion_state1.motion_type);
                                if(motion_state1.current_motion >= 'A' && motion_state1.current_motion <= 'Z')
                                    printf("��ǰ�˶�Ŀ�꣺%c\r\n",motion_state1.current_motion);
                                else
                                    printf("��ǰû���˶�Ŀ��\r\n");
                                printf("�������״̬��%#x\r\n",motion_state1.step_flag);
                                printf("��״̬��%#x\r\n",motion_state1.valve_flag);

                                switch(HMICurMenu){
                                    case HMI_OTHER:
                                        printf("HMI״̬��HMI_OTHER\r\n");
                                        break;
                                    case HMI_INIT:
                                        printf("HMI״̬��HMI_INIT\r\n");
                                        break;
                                    case HMI_CHECK:
                                        printf("HMI״̬��HMI_CHECK\r\n");
                                        break;
                                    case HMI_TEST:
                                        printf("HMI״̬��HMI_TEST\r\n");
                                        break;
                                    case HMI_QUERY:
                                        printf("HMI״̬��HMI_QUERY\r\n");
                                        break;
                                    case HMI_QC:
                                        printf("HMI״̬��HMI_QC\r\n");
                                        break;
                                    case HMI_SET:
                                        printf("HMI״̬��HMI_SET\r\n");
                                        break;
                                    case HMI_SYS:
                                        printf("HMI״̬��HMI_SYS\r\n");
                                        break;
                                    case HMI_OFF:
                                        printf("HMI״̬��HMI_OFF\r\n");
                                        break;
                                    default:
                                        printf("HMI״̬����\r\n");
                                        break;
                                }

                                break;
                            case 'V':
                                if (c[i][1] == 4) {//��ѯ��״̬  //���÷�����//Vnnb
                                    //����������Motion_QueueΪ��ʱֱ�Ӵ���??��α����ͻ ����ʱ����������������Motion_Queue�������
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //�Ѿ�����ź����������ڿ��Է����˶�ģ��
                                        //�˶�ģ�黥�����
                                        
                                        motion_state1.motion_type=1;//�������˶�
                                        motion_state1.current_motion='V';
                                        
                                        printf("����������:%s\r\n",buffer);  //??(buffer[c[i][0]+2]-'0')*10+buffer[c[i][0]+3]-'0';
                                        //[V011]
                                        bits=(buffer[c[i][0]+1]-'0')*10+buffer[c[i][0]+2]-'0';
                                        if (bits) {//bits>0��Ӧ�ķ�����
                                            if(buffer[c[i][0]+3]=='0')
                                                motion_state1.valve_flag &= ~(1<<(bits-1));
                                            else
                                                motion_state1.valve_flag |= 1<<(bits-1);
                                        }
                                        printf("��״̬:%#x\r\n",motion_state1.valve_flag);
                                        SwitchOut(motion_state1.valve_flag);
                                        
                                        motion_state1.motion_type=0;//���˶� 
                                        motion_state1.current_motion=0;//���˶�Ŀ��
                                        
                                        xSemaphoreGive( MutexSemMotion );//�ͷŻ����ź���
                                    } else printf("com_task[V]û�л�û����ź���\r\n");
                                }
                                break;
                            case 'S':
                                if (c[i][1] == 8) {//��ѯ�������״̬  //���ò������
                                    //����������Motion_QueueΪ��ʱֱ�Ӵ���??��α����ͻ ����ʱ����������������Motion_Queue�������
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //�Ѿ�����ź����������ڿ��Է����˶�ģ��
                                        //�˶�ģ�黥�����
                                        
                                        motion_state1.motion_type=1;//�������˶�
                                        motion_state1.current_motion='S';
                                        
                                        printf("���������������:%s\r\n",buffer);
                                        num=(buffer[c[i][0]+1]-'0')*10+buffer[c[i][0]+2]-'0';
                                    
                                        if(num) {//??��num=5ʱ�������䶯��
                                            motion_state1.step_flag |= (1<<(num-1));
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
                                        
                                        motion_state1.motion_type=0;//���˶� 
                                        motion_state1.current_motion=0;//���˶�Ŀ��
                                        
                                        xSemaphoreGive( MutexSemMotion );//�ͷŻ����ź���
                                    } else printf("com_task[S]û�л�û����ź���\r\n");
                                }
                                break;
                            case 'P':
                                if (c[i][1] == 8) {//��ѯ�䶯��״̬  //�����䶯��
                                    //����������Motion_QueueΪ��ʱֱ�Ӵ���??��α����ͻ ����ʱ����������������Motion_Queue�������
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //�Ѿ�����ź����������ڿ��Է����˶�ģ��
                                        //�˶�ģ�黥�����
                                        
                                        motion_state1.motion_type=1;//�������˶�
                                        motion_state1.current_motion='P';
                                        
                                        printf("�䶯�ÿ�������:%s\r\n",buffer);
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
                                        
                                        motion_state1.motion_type=0;//���˶� 
                                        motion_state1.current_motion=0;//���˶�Ŀ��
                                        
                                        xSemaphoreGive( MutexSemMotion );//�ͷŻ����ź���
                                    } else printf("com_task[P]û�л�û����ź���\r\n");
                                }
                                break;
                            case 'R':
                                if (c[i][1] == 4) {//��ѯ��ת��״̬  //������ת��
                                    //����������Motion_QueueΪ��ʱֱ�Ӵ���??��α����ͻ ����ʱ����������������Motion_Queue�������
                                    if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //�Ѿ�����ź����������ڿ��Է����˶�ģ��
                                        //�˶�ģ�黥�����
                                        
                                        motion_state1.motion_type=1;//�������˶�
                                        motion_state1.current_motion='R';
                                        
                                        printf("��ת�ÿ�������:%s\r\n",buffer);//??c[i][0]+
                                        //[R011]
                                        bits=(buffer[c[i][0]+1]-'0')*10+buffer[c[i][0]+2]-'0';
                                        if (bits==1) {//bits>0��Ӧ�ķ�����
                                            if(buffer[c[i][0]+3]=='0')
                                                motion_state1.valve_flag &= ~(1<<(15-1));
                                            else
                                                motion_state1.valve_flag |= 1<<(15-1);
                                        }
                                        printf("��ת��״̬:%#x\r\n",motion_state1.valve_flag);
                                        SwitchOut(motion_state1.valve_flag);
                                        
                                        motion_state1.motion_type=0;//���˶� 
                                        motion_state1.current_motion=0;//���˶�Ŀ��
                                        
                                        xSemaphoreGive( MutexSemMotion );//�ͷŻ����ź���
                                    } else printf("com_task[R]û�л�û����ź���\r\n");
                                }
                                break;
                            case 'B':
                                if (c[i][1] == 13) {
                                    c0=(buffer[c[i][0]+1]-'0')*1000+(buffer[c[i][0]+2]-'0')*100+(buffer[c[i][0]+3]-'0')*10+(buffer[c[i][0]+4]-'0');
                                    c1=(buffer[c[i][0]+5]-'0')*1000+(buffer[c[i][0]+6]-'0')*100+(buffer[c[i][0]+7]-'0')*10+(buffer[c[i][0]+8]-'0');
                                    a=(buffer[c[i][0]+9]-'0')*1000+(buffer[c[i][0]+10]-'0')*100+(buffer[c[i][0]+11]-'0')*10+(buffer[c[i][0]+12]-'0');
                                    printf("c0:%d,c1:%d,a:%d,\r\n", c0,c1,a); 
                                    StepMotoCal(c0,c1,a);
                                }
                                break;
                            default:
                                break;
                        }
                    }
                } else if (cmdtype == 2) {//��������()�����͵�motion_task����ȥ����
                    for(i=0;i<num;i++) {
                        switch(buffer[c[i][0]]) {
                            case 'M':
                                //��ѯ״̬
                                printf("�˶����ͣ�%d\r\n",motion_state1.motion_type);
                                if(motion_state1.current_motion >= 'A' && motion_state1.current_motion <= 'Z')
                                    printf("��ǰ�˶�Ŀ�꣺%c\r\n",motion_state1.current_motion);
                                else
                                    printf("��ǰû���˶�Ŀ��\r\n");
                                printf("�������״̬��%#x\r\n",motion_state1.step_flag);
                                printf("��״̬��%#x\r\n",motion_state1.valve_flag);
                                
                                switch(HMICurMenu){
                                    case HMI_OTHER:
                                        printf("HMI״̬��HMI_OTHER\r\n");
                                        break;
                                    case HMI_INIT:
                                        printf("HMI״̬��HMI_INIT\r\n");
                                        break;
                                    case HMI_CHECK:
                                        printf("HMI״̬��HMI_CHECK\r\n");
                                        break;
                                    case HMI_TEST:
                                        printf("HMI״̬��HMI_TEST\r\n");
                                        break;
                                    case HMI_QUERY:
                                        printf("HMI״̬��HMI_QUERY\r\n");
                                        break;
                                    case HMI_QC:
                                        printf("HMI״̬��HMI_QC\r\n");
                                        break;
                                    case HMI_SET:
                                        printf("HMI״̬��HMI_SET\r\n");
                                        break;
                                    case HMI_SYS:
                                        printf("HMI״̬��HMI_SYS\r\n");
                                        break;
                                    case HMI_OFF:
                                        printf("HMI״̬��HMI_OFF\r\n");
                                        break;
                                    default:
                                        printf("HMI״̬����\r\n");
                                        break;
                                }
                                
                                break;
                            case 'V':
                                if (c[i][1] == 4) {//��ѯ��״̬  //���÷�����//Vnnb
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='V';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//��Motion_Queue�����з�������
                                }
                                break;
                            case 'S':
                                if (c[i][1] == 8) {//��ѯ�������״̬  //���ò������
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='S';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//��Motion_Queue�����з�������
                                }
                                break;
                            case 'P':
                                if (c[i][1] == 8) {//��ѯ�䶯��״̬  //�����䶯��
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='P';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//��Motion_Queue�����з�������
                                }
                                break;
                            case 'R':
                                if (c[i][1] == 4) {//��ѯ��ת��״̬  //������ת��
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='R';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//��Motion_Queue�����з�������
                                }
                                break;
                            case 'W':
                                if (c[i][1] == 8) {//������ʱ
                                    memcpy(command,buffer+c[i][0], c[i][1]);
                                    //command[0]='W';
                                    command[c[i][1]]='\0';
                                    xQueueSend(Motion_Queue,command,10);//��Motion_Queue�����з�������
                                }
                                break;
                            case 'A'://��λ�˶�
                                memcpy(command,buffer+c[i][0], c[i][1]);
                                //command[0]='A';
                                command[c[i][1]]='\0';
                                xQueueSend(Motion_Queue,command,10);//��Motion_Queue�����з�������
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

//motion_task������ ֻ��������ʽ����()
void motion_task(void *pvParameters)
{
    u8 buffer[CMD_LEN];
    BaseType_t err;
    u8 bits;
    //u32 valve_state;
    s32 steps;
    u32 sum;
    u8 num;
    //??��һ����ʼ״̬��������ֵ���㣬���в����������ʼλ��
    while(1) {
        if(Motion_Queue!=NULL) {
            memset(buffer,0,CMD_LEN);    //���������
            err=xQueueReceive(Motion_Queue,buffer,10);//���÷�����ʽ  portMAX_DELAY
            if(err == pdTRUE) {
                if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //�Ѿ�����ź����������ڿ��Է����˶�ģ��
                //�˶�ģ�黥�����
                
                    switch(buffer[0]){
                        case 'A'://��λ����
                            motion_state1.motion_type=2;//�����˶�()
                            motion_state1.current_motion='A';
                        
                            if(buffer[1]=='0') {
                                printf("��λ����:%s\r\n",buffer);
                                //���в����������ʼλ�ã����з��ϵ磬������Ҫ�ͷ���ϣ��������Ҫ����ʱ
                                MotionReset();
                            }
                        
     //for(num=0;num<5;num++) printf("StepMotor[%d].stage=%d\r\n",num,StepMotor[num].stage);
     //for(num=0;num<5;num++) printf("StepMotor[%d].pot=%d\r\n",num,StepMotor[num].pot);
     //for(num=0;num<5;num++) printf("StepMotor[%d].dist=%d\r\n",num,StepMotor[num].dist);
                        
                            break;
                        case 'V'://ִ�з���������
                            motion_state1.motion_type=2;//�����˶�()
                            motion_state1.current_motion='V';
                            printf("����������:%s\r\n",buffer);
                            //(V011)
                            bits=(buffer[1]-'0')*10+buffer[2]-'0';
                            if (bits) {//bits>0��Ӧ�ķ�����
                                if(buffer[3]=='0')
                                    motion_state1.valve_flag &= ~(1<<(bits-1));
                                else
                                    motion_state1.valve_flag |= 1<<(bits-1);
                            }
                            printf("��״̬:%#x\r\n",motion_state1.valve_flag);
                            SwitchOut(motion_state1.valve_flag);
                            break;
                        case 'S'://ִ�в��������������
                            motion_state1.motion_type=2;//�����˶�()
                            motion_state1.current_motion='S';
                            printf("���������������:%s\r\n",buffer);
                            num=(buffer[1]-'0')*10+buffer[2]-'0';
                        
                            if((num>=1) && (num<=5)){//??��num=5ʱ�������䶯��
                                motion_state1.step_flag |= (1<<(num-1));
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
                        case 'P'://ִ���䶯�ÿ�������  �鵽�����������
                            motion_state1.motion_type=2;//�����˶�()
                            motion_state1.current_motion='P';
                            printf("�䶯�ÿ�������:%s\r\n",buffer);
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
                        case 'R'://ִ����ת�ÿ�������  �鵽������
                            motion_state1.motion_type=2;//�����˶�()
                            motion_state1.current_motion='R';
                            printf("��ת�ÿ�������:%s\r\n",buffer);
                            //(R011)
                            bits=(buffer[1]-'0')*10+buffer[2]-'0';
                            if (bits==1) {//bits>0��Ӧ�ķ�����
                                if(buffer[3]=='0')
                                    motion_state1.valve_flag &= ~(1<<(15-1));
                                else
                                    motion_state1.valve_flag |= 1<<(15-1);
                            }
                            printf("��ת��״̬:%#x\r\n",motion_state1.valve_flag);
                            SwitchOut(motion_state1.valve_flag);
                            break;
                        case 'W'://ִ����ʱ��������
                            motion_state1.motion_type=2;//�����˶�()
                            motion_state1.current_motion='W';
                            printf("��ʱ����:%s\r\n",buffer);
                        
                            steps = (buffer[1]-'0')*1000000+(buffer[2]-'0')*100000+(buffer[3]-'0')*10000+
                                    (buffer[4]-'0')*1000+(buffer[5]-'0')*100+(buffer[6]-'0')*10+buffer[7]-'0';
                            vTaskDelay(pdMS_TO_TICKS(steps));
                            printf("��ʱ����\r\n");
                            break;
                    }
                    
                    motion_state1.motion_type=0;//���˶� 
                    motion_state1.current_motion=0;//���˶�Ŀ��
                    xSemaphoreGive( MutexSemMotion );//�ͷŻ����ź���
                } else printf("motion_taskû�л�û����ź���\r\n");
            }
        }
    }
}

//hmi_task������ ��Ҫ������ӦHMI��������г�ʱ��ȴ�(������ʼ�ĸ�λ)������������
void hmi_task(void *pvParameters)
{//uart3ͨѶ
    u8 buffer[HMI_REC_LEN];
    BaseType_t res;
    u8 year,month,date,week;
    u8 hour,min,sec;//,i;//time_valid;//,ampm;
    //u32 tempState;
    
    u16 mem_addr,mem_data;
    
    u8 query_cnt=0;//��ѯ���ļ�¼����
    u8 query_mod=0;//��ѯ��ʽ��0��ȫ����1��������г���2����ʱ���г�
    u8 query_ref=0;//0��ʾ��û��ˢ��,1�����Ѿ�ˢ��

    char tbuf[32];
    //char hbuf[32];
    u16 addr;
    u8 m[2],l[2],re;
    DIR dir_a[2];

    //vTaskDelay(pdMS_TO_TICKS(100));//??���ٱ�֤100  //??��ӡpdMS_TO_TICKS�����ǲ���ʵ��ֵ
    
    while(1) {
        if(Hmi_Queue!=NULL) {//��ȡHMI��������Ϣ
            memset(buffer,0,HMI_REC_LEN);    //���������
            res=xQueueReceive(Hmi_Queue,buffer,10);//portMAX_DELAY ??��ʱ0��������
            if(res == pdTRUE) {//HMI�������
                //printf("%s",buffer);

                switch(buffer[0]) {
                    case READ_RES://��ȡ�Ĵ�������
                        if(buffer[1] == RTC_NOW) {
                            
                            year  = ((buffer[3] >> 4 ) & 0x0f)* 10 + (buffer[3] & 0x0f);
                            month = ((buffer[4] >> 4 ) & 0x0f)* 10 + (buffer[4] & 0x0f);
                            date  = ((buffer[5] >> 4 ) & 0x0f)* 10 + (buffer[5] & 0x0f);
                            week  =                                  (buffer[6] & 0x0f);
                            hour  = ((buffer[7] >> 4 ) & 0x0f)* 10 + (buffer[7] & 0x0f);
                            min   = ((buffer[8] >> 4 ) & 0x0f)* 10 + (buffer[8] & 0x0f);
                            sec   = ((buffer[9] >> 4 ) & 0x0f)* 10 + (buffer[9] & 0x0f);
                            //printf("time_valid:%d\r\n",time_valid);
                            printf("\r\nʱ����Ч20%02d/%02d/%02d Week:%d %02d:%02d:%02d\r\n",year,month,date,week,hour,min,sec);//??��ô������
                            //year=CorrectYear(year);
                            //CorrectDate(year,&month,&date);
                            //CorrectTime(&hour,&min,&sec);
                            //week=RTC_Get_Week(2000 + year,month,date);
                            if(hour < 12)//?? 
                                RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //����ʱ��
                            else
                                RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //����ʱ��
                            RTC_Set_Date(year,month,date,week);    //��������
                        }
                        break;
                    case READ_MEM://��ȡ�����洢������
                        mem_addr = buffer[1]*256+buffer[2];
                        mem_data = buffer[4]*256+buffer[5];
                        switch(mem_addr) {
                            case MENU1://�ı�˵�
                                switch(mem_data) {
                                    case MENU1_ANALY:
                                        HMICurMenu = HMI_TEST;
                                        break;
                                    case MENU1_QUERY:
                                        HMICurMenu = HMI_QUERY;
                                        break;
                                    case MENU1_QC:
                                        HMICurMenu = HMI_QC;
                                        break;
                                    case MENU1_SET:
                                        HMICurMenu = HMI_SET;
                                        break;
                                    case MENU1_SYS:
                                        HMICurMenu = HMI_SYS;
                                        break;
                                    case MENU1_OFF:
                                        HMICurMenu = HMI_OFF;
                                        break;
                                }
                                break;
                        }
                        break;
                }
            }
        }
        
        switch(HMICurMenu) {
            case HMI_OTHER:
                Uart3_PutString(ReadRTC,ReadRTCLen);//5A A5 03 81 20 07 ��ȡRTC Ŀ���Ǹ��¿������ڲ�ʱ��
                HMICurMenu=HMI_INIT;
                break;
            case HMI_INIT://�˴�����������ʽִ�и�λ����
                //��ɵ���������õĸ�λ�ͳ�ʼ������ȡ ������źţ�����ͺ�⣩����    ??������??
                //��λ�˶�    ??������??
                if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //�Ѿ�����ź����������ڿ��Է����˶�ģ��
                //�˶�ģ�黥�����
                                        
                    motion_state1.motion_type=2;//�����˶�
                    motion_state1.current_motion='A';
                                        
                    MotionReset();

                    motion_state1.motion_type=0;//���˶� 
                    motion_state1.current_motion=0;//���˶�Ŀ��
                        
                    xSemaphoreGive( MutexSemMotion );//�ͷŻ����ź���
                } else printf("HMI_taskû�л�û����ź���\r\n");

                Uart3_PutString(ToPageN,ToPageNLen);//5A A5 04 80 03 00 01  ҳ���л��е�1��ҳ��
                HMICurMenu=HMI_TEST;
                break;
            case HMI_CHECK:
                break;
            case HMI_TEST:
                if(motion_state1.motion_type==0) {//���˶� 
                    if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)==Bit_RESET) {//�����ź�Ϊ�Ϳ�ʼ����
                        vTaskDelay(pdMS_TO_TICKS(10));//���ٱ�֤10
                        if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)==Bit_RESET) {
                            printf("��������\r\n");
                            Uart3_PutString("\x5A\xA5\x0D\x82\x01\x60\x31\x32\x33\x34\x35\x36\x37\x38\x30\x31",16);//�ı���
                            Uart3_PutString("\x5A\xA5\x0E\x84\x01\x00\x64\x00\x32\x01\x00\x00\x50\x02\x00\x00\x64",17);//������
                            buffer[0]='A';
                            buffer[1]='1';
                            buffer[2]='\0';
                            xQueueSend(Motion_Queue,buffer,10);//��Motion_Queue�����з�������
                        }
                    }
                }
                break;
            case HMI_QUERY:
                //��ȡ�ļ�
                if (query_ref==0) {//��ûˢ��
                    query_cnt = 0;
                    switch(query_mod) {
                        case 0://ȫ����ʾ
                            strcpy(tbuf, WORKPATH);
                            m[0] = 0;
                            m[1] = 1;
                            //addr = RECADDR;
                            do{
                                re=ReadRecords(tbuf,dir_a,l,m);
                                if(re==0) {//char RECORDS[]="\x5A\xA5\x31\x82\x01\x60 A0001 ����CRP0001[   180119093625   ] U000000"
                                    addr=RECADDR+query_cnt*46;
                                    RECORDS[4]=*((char *)(&addr)+1);
                                    RECORDS[5]=*((char *)(&addr)+0);
                                    string_cpy(RECORDS+7,(char *)record_inf1.serial,5);
                                    string_cpy(RECORDS+20,(char *)record_inf1.number,4);
                                    string_cpy(RECORDS+28,(char *)record_inf1.time,12);
                                    string_cpy(RECORDS+45,(char *)record_inf1.result,7);
                                    Uart3_PutString(RECORDS,sizeof(RECORDS)-1);
                                    query_cnt++;
                                    if(query_cnt>=9) break;
                                }
                            } while((re==1)||(re==0));
                            break;
                    }
                    query_ref=1;
                }
                break;
            case HMI_QC:
                break;
            case HMI_SET:
                break;
            case HMI_SYS:
                break;
            case HMI_OFF:
                break;
            
        }


//                    Uart3_PutString("\x5A\xA5\x04\x80\x03\x00\x01", 7);//5A A5 04 80 03 00 01  ҳ���л��е�1��ҳ��


        vTaskDelay(pdMS_TO_TICKS(10));
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
    for(i=0;i<len;i++) {
        if(steps == 0) {
            if((buf[i] == ']') || (buf[i] == ')')) {//���ҵ��˽���λֱ���˳�
                //result=0;//Ĭ��Ϊ��Ч�ַ���
                //end=0;
                steps=2;
            } else if(buf[i] == '[') {//����ʼλ[
                end=']';//���������
            } else if(buf[i] == '(') {//����ʼλ(
                end=')';//���������
            }
            if(end) {
                j=0;//�������
                c[0][0]=i+1;//����λ��
                c[0][1]=0;//�����
                valid=1;//Ĭ�ϵ�ǰ������Ч
                steps=1;
            }
        } else if(steps == 1) {
            //�ҷָ���;�������]
            if(buf[i] == ';') {//<=8��������Ӧ;�������;
                if(k < 7) {//���滹����������
                    if((j)&&(valid)) {//����;���С����Сָ���1�Ͷ���,��Чָ���
                        c[k][1]=j;
                        k++;//һ����k������
                    }
                    c[k][0]=i+1;
                    c[k][1]=0;
                    j=0;//�������
                    valid=1;//Ĭ�ϵ�ǰ������Ч
                }
                else if(k == 7) {//�����������
                    if((j)&&(valid)) {//����;���С����Сָ���1�Ͷ���,��Чָ���
                        c[k][1]=j;
                        k++;//һ����k������
                    }
                }
            } else if(buf[i] == end) {
                if(k <= 7) {//<=8��������Ӧ;�������;
                    if((j)&&(valid)) {//����;���С����Сָ���1�Ͷ���,��Чָ���
                        c[k][1]=j;
                        k++;//һ����k������
                    }
                }
                if(k) {//�ҵ���Ӧ����λ����������һ����Ч����
                    if(end == ']') {
                        result=1;
                    } else if(end == ')') {
                        result=2;
                    }
                }
                //*n=k;
                steps=2;
            } else {
                if(j==0) {//ֻ�жϵ�һ����ĸ��Ч�ԣ����ݲ��ж�
                    if(buf[i] >= 'a' && buf[i] <= 'z') {//����ĸת�ɴ�д
                        buf[i] -= 'a' - 'A';//??
                    }
                    if(buf[i] == 'D'|| buf[i] == 'N'|| buf[i] == 'P'|| buf[i] == 'R'||
                       buf[i] == 'S'|| buf[i] == 'T'|| buf[i] == 'V'|| buf[i] == 'W'||
                       buf[i] == 'F'|| buf[i] == 'M'|| buf[i] == 'A'|| buf[i] == 'B'){//��һ����ĸ��Ч

                    } else {//��һ����ĸ��Ч
                        valid=0;
                    }
                }
                /*{//??F��Ҫ����д
                    if(j) {//������������֣�������ĸ��Ч
                        valid=0;
                    }
                } else if(buf[i] >= '0' && buf[i] <= '9') {
                    if(j==0) {//��ͷ�������ض���ĸ��������Ч
                        valid=0;
                    }
                } else {//������ĸ������֮����ַ�����ʹ��ǰ������Ч
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
{//�����ַ�������
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
{//�ַ�����һ���ַ�������'/'��Ӧ����"0:/"��ʽ��ͷ
    u8 i,j,k,l,m;
    l = strlen((const char *)buf1);
    if (l > 64) l = 64;//���Ƴ��ȱ������
    i = 0;//ָ��buf1�е�λ��
    j = 0;//ָ��buf2�е�λ�ã�����Ч�ַ����ĳ���
    k = 0;//��¼Ŀ¼����
    while(i < l){//ȡ��buf1��ÿһ���ַ������жϣ�i����С�ڳ���l��
        //"0://test//20180101///"
        if (*(buf1 + i) == '/') {
            m = 0;//����б�ܸ���
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

//�õ�����ʣ������
//drv:���̱��("0:"/"1:")
//total:������     ����λKB��
//free:ʣ������     ����λKB��
//����ֵ:0,����.����,�������
u8 exf_getfree(u8 *drv,u32 *total,u32 *free)
{
    FATFS *fs1;
    u8 res;
    u32 fre_clust=0, fre_sect=0, tot_sect=0;
    //�õ�������Ϣ�����д�����
    res =(u32)f_getfree((const TCHAR*)drv, (DWORD*)&fre_clust, &fs1);
    if(res==0) {                                               
        tot_sect=(fs1->n_fatent-2)*fs1->csize;    //�õ���������
        fre_sect=fre_clust*fs1->csize;            //�õ�����������       
#if _MAX_SS!=512                                  //������С����512�ֽ�,��ת��Ϊ512�ֽ�
        tot_sect*=fs1->ssize/512;
        fre_sect*=fs1->ssize/512;
#endif      
        *total=tot_sect>>1;    //��λΪKB
        *free=fre_sect>>1;    //��λΪKB 
    }
    return res;
}

void KEY_Init(void)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOEʱ��

    //GPIOE11��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE
	
}


void MotionReset(void)
{
    u32 sum,i;

    printf("�򿪷�#5\r\n");//Ϊ��ʹע����2�Ƽ���ͨ·
    motion_state1.valve_flag |= 1<<(5-1);
    printf("��״̬:%#x\r\n",motion_state1.valve_flag);
    SwitchOut(motion_state1.valve_flag);
                    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    motion_state1.current_motion='S';
    
    for(i=0;i<5;i++) {
        StepMotor[i].stage=3;//���е���˶�״̬Ϊ3ֹͣ
    }
    
    motion_state1.step_flag = (1<<0);
    sum=StepMotoMove(0,0,1600,0);//??����λλ��4000������
    vTaskDelay(pdMS_TO_TICKS((sum*2+StepMotor[0].Size[1]*(*StepMotor[0].Buf[1]))/1000));
    vTaskDelay(pdMS_TO_TICKS(200));
    if (IsLmt(0)==0) printf("�������0����\r\n");
    
    motion_state1.step_flag = (1<<1);
    sum=StepMotoMove(1,0,-2400,0);//??����λλ��4000������
    vTaskDelay(pdMS_TO_TICKS((sum*2+StepMotor[1].Size[1]*(*StepMotor[1].Buf[1]))/1000));
    vTaskDelay(pdMS_TO_TICKS(200));
    if (IsLmt(1)==0) printf("�������1����\r\n");
    
    motion_state1.step_flag = (1<<2);
    sum=StepMotoMove(2,0,-2400,0);//??����λλ��4000������
    vTaskDelay(pdMS_TO_TICKS((sum*2+StepMotor[2].Size[1]*(*StepMotor[2].Buf[1]))/1000));
    vTaskDelay(pdMS_TO_TICKS(200));
    if (IsLmt(2)==0) printf("�������2����\r\n");
    
    motion_state1.step_flag = (1<<3);
    sum=StepMotoMove(3,0,-2400,0);//??����λλ��4000������
    vTaskDelay(pdMS_TO_TICKS((sum*2+StepMotor[3].Size[1]*(*StepMotor[3].Buf[1]))/1000));
    vTaskDelay(pdMS_TO_TICKS(200));
    if (IsLmt(3)==0) printf("�������3����\r\n");
    
    motion_state1.step_flag = 0;
    
AllSleep();//����
    vTaskDelay(pdMS_TO_TICKS(200));

    printf("�ر����з�\r\n");
    motion_state1.valve_flag=0;
    printf("��״̬:%#x\r\n",motion_state1.valve_flag);
    SwitchOut(motion_state1.valve_flag);
}

u8 string_cmp(char * buf1,char * buf2,u8 len)
{
    u8 i;
    for(i=0;i<len;i++) {
        if(buf1[i] != buf2[i]) return 0;
    }
    return 1;
}

void string_cpy(char * buf1,char * buf2,u8 len)
{
    u8 i;
    for(i=0;i<len;i++) buf1[i] = buf2[i];
}

u8 ReadRecords(char * tbuf,DIR * dir_a,u8 * l,u8 *m)
{//res=0���ɹ���1�������У�2��������3������
    char namebuf[16];
    char sbuf[32];//64
    u8 flag;
    u8 res = 1;
    if((m[1]!=1)&&(m[1]!=2)) return 3;//ʧ��
    if ( m[1] > m[0] ) {                                        //ֻ��������Ŀ¼ʱ��ִ��
        f_opendir(&dir_a[m[1]-1], tbuf);
        l[m[1]-1] = strlen(tbuf);
    }
    m[0] = m[1];
    f_readdir(&dir_a[m[1]-1], &fileinfo);                       //��ȡĿ¼�µ�һ���ļ�
    if (fileinfo.fname[0] == 0) {                               //��ĩβ��,�˳� //������/fres != FR_OK || 
        if (m[1]==2){                                           //����Ŀ¼�¸�ѭ�����빤��Ŀ¼
            m[1]=1; 
            tbuf[l[0]] = '\0';                                  //�洢��·�����ع���Ŀ¼
            res=1;
        }
        else res=2;
    } else {
        sprintf(tbuf,"%s/%s",tbuf,*fileinfo.lfname ? fileinfo.lfname : fileinfo.fname);//���������ļ����ļ������ӳ�������·��
        if (fileinfo.fattrib & AM_DIR) {                        //��Ŀ¼
            if(m[1]==1) {
                strcpy(namebuf,*fileinfo.lfname ? fileinfo.lfname : fileinfo.fname);//�õ�Ŀ¼
                if(strlen(namebuf) == 8) m[1]=2;                //Ŀ¼������8�ҵ�ǰ�ǹ���Ŀ¼���¸�ѭ��������Ŀ¼
            }
        }else {
            //printf("%s [%dF]\r\n", tbuf,m[1]);                //��ӡ�ļ�
            if(m[1]==2) {                                       //��ǰ��ĳ��ʱ��Ŀ¼��
                fres = f_open(&file, (const TCHAR*)tbuf, FA_OPEN_EXISTING | FA_READ);
                if(fres == FR_OK) {//FR_OK ���ļ��ɹ�  ??�ļ���������
                    //printf("�ļ�����:->\r\n");
                    flag=0;
                    do{
                        if(f_gets(sbuf,32,&file) != 0) {//��ȡһ�� ??\n����\0�� \r��ô��??
                            //printf("%s", tbuf);
                            switch(sbuf[0]) {
                                case 'A'://���
                                    if(string_cmp(&sbuf[2],&tbuf[l[0]+1+8+1],5)) {
                                        string_cpy((char *)record_inf1.serial,&sbuf[2],5);
                                        record_inf1.serial[5] = '\0';
                                        flag |= 1;
                                    }
                                    break;
                                case 'B'://���
                                    if(string_cmp(&sbuf[2],&tbuf[l[0]+1+8+1+5],4)) {
                                        string_cpy((char *)record_inf1.number,&sbuf[2],4);
                                        record_inf1.number[4] = '\0';
                                        flag |= 2;
                                    }
                                    break;
                                case 'I'://ʱ��
                                    if(string_cmp(&sbuf[2],&tbuf[l[0]+1+2],6)) {
                                        string_cpy((char *)record_inf1.time,&sbuf[2],12);
                                        record_inf1.time[12] = '\0';
                                        flag |= 4;
                                    }
                                    break;
                            }
                        }
                        if(flag==7) {
                            string_cpy((char *)record_inf1.result,&tbuf[l[0]+1+8+1+5+4],7);
                            record_inf1.result[7] = '\0';
                            res=0;
                            break;//������Ҫ���˳�
                        }
                    }while ((!f_eof(&file))&&(!f_error(&file)));//??û�н�����û�з�������
                    fres=f_close(&file);
                } else if (fres == FR_NO_FILE){
                    printf("�ļ�������\r\n");
                }
                tbuf[l[1]] = '\0';                              //�洢��·������Ŀ¼
                //res = 0;
            }
        }
    }
    return res;
}
