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
#define HMI_START      1
#define HMI_CHECK      2
#define HMI_ANALY      3
#define HMI_QUERY      4
#define HMI_QC         5
#define HMI_SET        6
#define HMI_SYS        7
#define HMI_OFF        8

#define COM_Q_NUM      4                           //�������ݵ���Ϣ���е����� 
#define HMI_Q_NUM     16                           //�������ݵ���Ϣ���е����� 

#define CMD_LEN       12                           //�����������  //����Ϊ4�ֽڻ�8�ֽڻ��н���λ1�ֽ��ܳ���5��9�ֽڣ��ճ�4��������Ϊ12�ֽ�

#define MOTION_Q_NUM  16                           //�˶������������Ϣ���е����� 

//#define VALVE_Q_NUM   16                           //���ͷ������������Ϣ���е����� 
//#define STEP_Q_NUM    16                           //���Ͳ�����������������Ϣ���е����� 
//#define PUMP_Q_NUM     8                           //���ͱÿ����������Ϣ���е����� 

QueueHandle_t Com_Queue;                           //������Ϣ���о��
QueueHandle_t Hmi_Queue;                           //HMI��Ϣ���о��
QueueHandle_t Motion_Queue;                        //�˶�������Ϣ���о��
//QueueHandle_t Valve_Queue;                         //��������Ϣ���о��
//QueueHandle_t Step_Queue;                          //�������������Ϣ���о��
//QueueHandle_t Pump_Queue;                          //�ÿ�����Ϣ���о��   ??�䶯�õ�����������ã���ת�õ�������

struct motion_state{
    u8 motion_type;//�˶����ͣ�0�����˶���1���������˶���2�������˶�
    u8 current_motion;//��ǰ�˶�Ŀ�꣺'A'����λ�˶���'S'�����������'V'������'R'���䶯�ã�'P'����ת��
    u16 step_flag;//ÿһλ����һ�����������1���0��ֻ��һ����1??   �䶯�ù鵽���������
    u16 valve_flag;//ÿһλ����һ������1���0�����Զ��1           ��ת�ù鵽����
}motion_state1;//??�˶�״̬��������

//�����ź������
SemaphoreHandle_t MutexSemMotion;	//�˶����ƻ����ź���??ֻ���˶�����ģ��
//??printfӦ��һ�������У�����Ҫ�ӻ����ź���

u32 decodeCmd(u8 buf[],u8 len,u8 c[ ][2],u8 *n);
u8 str_len(u8 *str);
u8 exf_getfree(u8 *drv,u32 *total,u32 *free);
u8 getDir(u8 * buf1,u8 * buf2,u8 pos);

FATFS fs;
FIL file;                                          //�ļ�
//UINT br,bw;                                        //��д����
FILINFO fileinfo;                                  //�ļ���Ϣ
DIR dir;                                           //Ŀ¼

#define LEVEL             8                        //LEVEL���ô�С�����������ȣ�8�ʹ���8�㣬�ڴ��㹻�Ļ��������ø���Щ
#define WORKPATH          "0:/TEST"                //Ĭ�ϴ����Ĺ���Ŀ¼0:/CRP
#define WORKFILE          "setting"                //Ĭ�ϵ�ǰ���ļ�
#define MAXNAMELEN        32                       //�ļ����Ϊ32�ֽ�

u8 filename[MAXNAMELEN];//32??
FRESULT fres;
u8 curDir[64]=WORKPATH;//??
u8 curFilename[MAXNAMELEN]=WORKFILE;//??
u32 curFileFlag=0;//0��û�д򿪣�1���ļ��Զ��ķ�ʽ�򿪣�2���ļ���д�ķ�ʽ��
//------------------MAIN ��ʼ------------------
int main(void)
{

    //����Ӳ����ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4����ʾ֧��0-15����ռ���ȼ�����֧�������ȼ���
    
    delay_init(168);                               //��ʼ����ʱ����
    
    EXTIX_Init();
    
    UART1_Init(115200);                            //����1��ʼ��������Ϊ115200�����ڴ���ͨѶ
    USART3_Init(115200);                           //����3��ʼ��������Ϊ115200������HMI ͨѶ
    
    TIM2_PWM_Init();                               //����LED�����ź�1120Hz PB11
    AD7799_Init(ADC_CON_GAIN1);                    //��ʼ��AD7799,����SPI2��ʼ��//??Ƶ��ѡ��ʱ���..
    
    HC595Init();                                   //??���ʱ���Ƿ�ɿ����仯ʱ�Ƿ�����ٽ���
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
    
    Motion_Queue=xQueueCreate(MOTION_Q_NUM,CMD_LEN);   //������ϢValve_Queue�����ղ���������������16�������12
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

u32 sum;
u16 c0,c1,a;
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
    //u16 c0,c1,a;
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
                                                if (j<8) j++;                                  //�¸�ѭ��������Ŀ¼
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
                                    sum=StepMotoCal(c0,c1,a);

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
                                command[1]='\0';
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
    u8 num;
    //??��һ����ʼ״̬��������ֵ���㣬���в����������ʼλ��
    while(1) {
        if( xSemaphoreTake( MutexSemMotion, pdMS_TO_TICKS(10) ) == pdTRUE ) { //�Ѿ�����ź����������ڿ��Է����˶�ģ��
            //�˶�ģ�黥�����
            

            if(Motion_Queue!=NULL) {
                memset(buffer,0,CMD_LEN);    //���������

                err=xQueueReceive(Motion_Queue,buffer,10);//���÷�����ʽ  portMAX_DELAY
                if(err == pdTRUE) {
                    switch(buffer[0]){
                        case 'A'://��λ����
                            motion_state1.motion_type=2;//�����˶�()
                            motion_state1.current_motion='A';
                            printf("��λ����:%s\r\n",buffer);
                            //���в����������ʼλ�ã����з��ϵ磬������Ҫ�ͷ���ϣ��������Ҫ����ʱ
                        
                        
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
                }
                motion_state1.motion_type=0;//���˶� 
                motion_state1.current_motion=0;//���˶�Ŀ��
            }
            
            
            xSemaphoreGive( MutexSemMotion );//�ͷŻ����ź���
        } else printf("motion_taskû�л�û����ź���\r\n");
    }
}

//hmi_task������ 
void hmi_task(void *pvParameters)
{//uart3ͨѶ
    u8 buffer[HMI_REC_LEN];
    BaseType_t res;
    u8 year,month,date,week;
    u8 hour,min,sec,start_cnt;//time_valid;//,ampm;
    
//    u8 command[12];
    
    u32 PreviousState = HMI_OTHER;
    u32 CurrentState  = HMI_OTHER;
    u32 NextState     = HMI_OTHER;
    
    while(1) {
        if(Hmi_Queue!=NULL) {//��ȡHMI��������Ϣ
            memset(buffer,0,HMI_REC_LEN);    //���������
            res=xQueueReceive(Hmi_Queue,buffer,10);//portMAX_DELAY ??��ʱ0��������
            if(res == pdTRUE) {//HMI�������
                printf("%s",buffer);
                //Uart3_PutString("page start", sizeof("page start")-1);
            }
        }
        
        switch(CurrentState) {
            case HMI_START:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page start", sizeof("page start")-1);//Uart3_HMICmd("cls RED", sizeof("cls RED")-1);//ˢ��
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100  //??��ӡpdMS_TO_TICKS�����ǲ���ʵ��ֵ
//                    memcpy(command,"1021", 4);
//                    command[4]='\0';
//                    xQueueSend(Valve_Queue,command,10);//��Valve_Queue�����з�������
//                    memcpy(command,"10140400", 8);
//                    command[8]='\0';
//                    xQueueSend(Step_Queue,command,10);//��Valve_Queue�����з�������
                    start_cnt=0;
                    //time_valid=0;
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    //��Ҫ��ɣ���HMI��ȡ��ǰʱ�丳ֵ��RTC����������˶�������źţ�����ͺ�⣩���ԣ������õȼ��
                    //1����HMI��ȡ��ǰʱ�丳ֵ��RTC
                    if(start_cnt == 0){
                        //Uart3_HMICmd("get rtc0", sizeof("get rtc0")-1);
                        Uart3_PutString("\x5A\xA5\x03\x81\x20\x07", 6);//5A A5 03 81 20 07 ��ȡRTC
                        memset(buffer,0,HMI_REC_LEN);
                        start_cnt++;
                    } else if(start_cnt < 10){
                        if((res == pdTRUE) && (buffer[1] == 0x20)){//�յ���Ч����
                            
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
//                            if(hour < 12)//?? 
//                                RTC_Set_Time(hour,min,sec,RTC_H12_AM);    //����ʱ��
//                            else
//                                RTC_Set_Time(hour,min,sec,RTC_H12_PM);    //����ʱ��
//                            RTC_Set_Date(year,month,date,week);    //��������
                            //time_valid = 1;
                            //start_cnt=10;
                            NextState = HMI_CHECK;
                        } else {//������
                            start_cnt++;
                        }
                    } else {
                        //��ʾ��ȡʱ��ʧ��
                        printf("\r\nû�ж���ʱ��\r\n");

                        NextState = HMI_CHECK;
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));//���ٱ�֤20 ��HMI����ʱ��
                    //printf("start_cnt:%d\r\n",start_cnt);
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;
                    vTaskDelay(pdMS_TO_TICKS(2000));//���ٱ�֤10
                STATE_END                                                       //} break;
            case HMI_CHECK:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page check", sizeof("page check")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100
                STATE_TRANSITION_TEST                                           //} if ( NextState == CurrentState ) {
                    //��Ҫ��ɣ���ϴ��
                    
                    NextState = HMI_ANALY;
                STATE_EXIT_ACTION                                               //} if ( NextState != CurrentState ) { CurrentState = NextState;
                    vTaskDelay(pdMS_TO_TICKS(2000));//���ٱ�֤10
                STATE_END                                                       //} break;
           case HMI_ANALY:
                STATE_ENTRY_ACTION                                              //if ( CurrentState != PreviousState ) { PreviousState = CurrentState;
                    //Uart3_HMICmd("page analyse_0", sizeof("page analyse_0")-1);
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100
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
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100
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
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100
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
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100
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
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100
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
                    vTaskDelay(pdMS_TO_TICKS(100));//���ٱ�֤100
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

//void LED_Init(void)
//{    	 
//    GPIO_InitTypeDef  GPIO_InitStructure;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��

//    //GPIOC6,F7��ʼ������
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//LED0��LED1��ӦIO��
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC
//	
//	GPIO_SetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7);//GPIOC6,F7���ø�

//}

