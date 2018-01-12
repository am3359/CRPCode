#include "rtc.h"
//#include "led.h"
#include "delay.h"
#include "usart.h" 
//#include "calendar.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//RTC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/5
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
//********************************************************************************
//修改说明
//V1.1 20140726
//新增:RTC_Get_Week函数,用于根据年月日信息,得到星期信息.
////////////////////////////////////////////////////////////////////////////////// 


NVIC_InitTypeDef   NVIC_InitStructure;

//RTC时间设置
//hour,min,sec:小时,分钟,秒钟
//ampm:@RTC_AM_PM_Definitions  :RTC_H12_AM/RTC_H12_PM
 
ErrorStatus RTC_Set_Time(u8 hour,u8 min,u8 sec,u8 ampm)
{
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	
	RTC_TimeTypeInitStructure.RTC_Hours=hour;
	RTC_TimeTypeInitStructure.RTC_Minutes=min;
	RTC_TimeTypeInitStructure.RTC_Seconds=sec;
	RTC_TimeTypeInitStructure.RTC_H12=ampm;
	
	return RTC_SetTime(RTC_Format_BIN,&RTC_TimeTypeInitStructure);
	
}
//RTC日期设置
//year,month,date:年(0~99),月(1~12),日(0~31)
//week:星期(1~7,0,非法!)
//返回值:0,成功
//       1,进入初始化模式失败 
ErrorStatus RTC_Set_Date(u8 year,u8 month,u8 date,u8 week)
{
	
	RTC_DateTypeDef RTC_DateTypeInitStructure;
	RTC_DateTypeInitStructure.RTC_Date=date;
	RTC_DateTypeInitStructure.RTC_Month=month;
	RTC_DateTypeInitStructure.RTC_WeekDay=week;
	RTC_DateTypeInitStructure.RTC_Year=year;
	return RTC_SetDate(RTC_Format_BIN,&RTC_DateTypeInitStructure);
}
//获取RTC时间
//*hour,*min,*sec:小时,分钟,秒钟 
//*ampm:@RTC_AM_PM_Definitions  :RTC_H12_AM/RTC_H12_PM.
void RTC_Get_Time(u8 *hour,u8 *min,u8 *sec,u8 *ampm)
{
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	
	RTC_GetTime(RTC_Format_BIN,&RTC_TimeTypeInitStructure);
	
	*hour=RTC_TimeTypeInitStructure.RTC_Hours;
	*min=RTC_TimeTypeInitStructure.RTC_Minutes;
	*sec=RTC_TimeTypeInitStructure.RTC_Seconds;
	*ampm=RTC_TimeTypeInitStructure.RTC_H12;
}
//获取RTC日期
//*year,*mon,*date:年,月,日
//*week:星期
void RTC_Get_Date(u8 *year,u8 *month,u8 *date,u8 *week)
{
	RTC_DateTypeDef  RTC_DateStruct;
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

	*year=RTC_DateStruct.RTC_Year;
	*month=RTC_DateStruct.RTC_Month;
	*date=RTC_DateStruct.RTC_Date;
	*week=RTC_DateStruct.RTC_WeekDay; 
}
//RTC初始化
//返回值:0,初始化成功;
//       1,LSE开启失败;
//       2,进入初始化模式失败;
u8 My_RTC_Init(void)
{
    RTC_InitTypeDef RTC_InitStructure;
    u16 retry=0X1FFF; 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//使能PWR时钟
    PWR_BackupAccessCmd(ENABLE);    //使能后备寄存器访问 
    
    if(RTC_ReadBackupRegister(RTC_BKP_DR0)!=0x5050)        //是否第一次配置?
    {
        RCC_LSEConfig(RCC_LSE_ON);//LSE 开启    
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)    //检查指定的RCC标志位设置与否,等待低速晶振就绪
        {
            retry++;
            delay_ms(10);
        }
        if(retry==0)return 1;        //LSE 开启失败. 
            
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);        //设置RTC时钟(RTCCLK),选择LSE作为RTC时钟    
        RCC_RTCCLKCmd(ENABLE);    //使能RTC时钟 

        RTC_InitStructure.RTC_AsynchPrediv = 0x7F;//RTC异步分频系数(1~0X7F)
        RTC_InitStructure.RTC_SynchPrediv  = 0xFF;//RTC同步分频系数(0~7FFF)
        RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;//RTC设置为,24小时格式
        RTC_Init(&RTC_InitStructure);
 
        RTC_Set_Time(11,59,56,RTC_H12_AM);    //设置时间
        RTC_Set_Date(18,1,10,3);        //设置日期
     
        RTC_WriteBackupRegister(RTC_BKP_DR0,0x5050);    //标记已经初始化过了
    } 
 
    return 0;
}

////设置闹钟时间(按星期闹铃,24小时制)
////week:星期几(1~7)
////hour,min,sec:小时,分钟,秒钟
//void RTC_Set_AlarmA(u8 week,u8 hour,u8 min,u8 sec)
//{ 
//	EXTI_InitTypeDef   EXTI_InitStructure;
//	RTC_AlarmTypeDef RTC_AlarmTypeInitStructure;
//	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
//	
//	RTC_AlarmCmd(RTC_Alarm_A,DISABLE);//关闭闹钟A 
//	
//  RTC_TimeTypeInitStructure.RTC_Hours=hour;//小时
//	RTC_TimeTypeInitStructure.RTC_Minutes=min;//分钟
//	RTC_TimeTypeInitStructure.RTC_Seconds=sec;//秒
//	RTC_TimeTypeInitStructure.RTC_H12=RTC_H12_AM;
//  
//	RTC_AlarmTypeInitStructure.RTC_AlarmDateWeekDay=week;//星期
//	RTC_AlarmTypeInitStructure.RTC_AlarmDateWeekDaySel=RTC_AlarmDateWeekDaySel_WeekDay;//按星期闹
//	RTC_AlarmTypeInitStructure.RTC_AlarmMask=RTC_AlarmMask_None;//精确匹配星期，时分秒
//	RTC_AlarmTypeInitStructure.RTC_AlarmTime=RTC_TimeTypeInitStructure;
//  RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_AlarmTypeInitStructure);
// 
//	
//	RTC_ClearITPendingBit(RTC_IT_ALRA);//清除RTC闹钟A的标志
//  EXTI_ClearITPendingBit(EXTI_Line17);//清除LINE17上的中断标志位 
//	
//	RTC_ITConfig(RTC_IT_ALRA,ENABLE);//开启闹钟A中断
//	RTC_AlarmCmd(RTC_Alarm_A,ENABLE);//开启闹钟A 
//	
//	EXTI_InitStructure.EXTI_Line = EXTI_Line17;//LINE17
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE17
//  EXTI_Init(&EXTI_InitStructure);//配置

//	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn; 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级1
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级2
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
//  NVIC_Init(&NVIC_InitStructure);//配置

//	
//}
////周期性唤醒定时器设置
///*wksel:
//#define RTC_WakeUpClock_RTCCLK_Div16        ((uint32_t)0x00000000)
//#define RTC_WakeUpClock_RTCCLK_Div8         ((uint32_t)0x00000001)
//#define RTC_WakeUpClock_RTCCLK_Div4         ((uint32_t)0x00000002)
//#define RTC_WakeUpClock_RTCCLK_Div2         ((uint32_t)0x00000003)
//#define RTC_WakeUpClock_CK_SPRE_16bits      ((uint32_t)0x00000004)
//#define RTC_WakeUpClock_CK_SPRE_17bits      ((uint32_t)0x00000006)
//*/
////cnt:自动重装载值.减到0,产生中断.
//void RTC_Set_WakeUp(u32 wksel,u16 cnt)
//{ 
//	EXTI_InitTypeDef   EXTI_InitStructure;
//	
//	RTC_WakeUpCmd(DISABLE);//关闭WAKE UP
//	
//	RTC_WakeUpClockConfig(wksel);//唤醒时钟选择
//	
//	RTC_SetWakeUpCounter(cnt);//设置WAKE UP自动重装载寄存器
//	
//	
//	RTC_ClearITPendingBit(RTC_IT_WUT); //清除RTC WAKE UP的标志
//  EXTI_ClearITPendingBit(EXTI_Line22);//清除LINE22上的中断标志位 
//	 
//	RTC_ITConfig(RTC_IT_WUT,ENABLE);//开启WAKE UP 定时器中断
//	RTC_WakeUpCmd( ENABLE);//开启WAKE UP 定时器　
//	
//	EXTI_InitStructure.EXTI_Line = EXTI_Line22;//LINE22
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE22
//  EXTI_Init(&EXTI_InitStructure);//配置
// 
// 
//	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn; 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//抢占优先级1
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;//子优先级2
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
//  NVIC_Init(&NVIC_InitStructure);//配置
//}

////RTC闹钟中断服务函数
//void RTC_Alarm_IRQHandler(void)
//{ 
//	OSIntEnter(); 
//	if(RTC_GetFlagStatus(RTC_FLAG_ALRAF)==SET)//ALARM A中断?
//	{
//		RTC_ClearFlag(RTC_FLAG_ALRAF);//清除中断标志
//		printf("ALARM A!\r\n");
//	}   
//	EXTI_ClearITPendingBit(EXTI_Line17);	//清除中断线17的中断标志 	
// OSIntExit();  	
//}

////RTC WAKE UP中断服务函数
//void RTC_WKUP_IRQHandler(void)
//{  
//	OSIntEnter(); 
//	if(RTC_GetFlagStatus(RTC_FLAG_WUTF)==SET)//WK_UP中断?
//	{ 
//		RTC_ClearFlag(RTC_FLAG_WUTF);	//清除中断标志
//		LED1=!LED1; 
//	}   
//	EXTI_ClearITPendingBit(EXTI_Line22);//清除中断线22的中断标志 			
//  OSIntExit();  	
//}
u8 const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表	  
//获得现在是星期几
//功能描述:输入公历日期得到星期(只允许1901-2099年)
//year,month,day：公历年月日 
//返回值：星期号(1~7,代表周1~周日)																						 
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{	
	u16 temp2;
	u8 yearH,yearL;
	yearH=year/100;	yearL=year%100; 
	// 如果为21世纪,年份数加100  
	if (yearH>19)yearL+=100;
	// 所过闰年数只算1900年之后的  
	temp2=yearL+yearL/4;
	temp2=temp2%7; 
	temp2=temp2+day+table_week[month-1];
	if (yearL%4==0&&month<3)temp2--;
	temp2%=7;
	if(temp2==0)temp2=7;
	return temp2;
}	


//-------------------------------------------------------------------
//函数名称：IsLeapYear
//功能描述：判断某年是否为闰年
//调用清单：
//被调清单：
//参数说明：year,年
//输出说明：1是闰年，0不是闰年
//返回  值：1,0
//其    他：
//-------------------------------------------------------------------
u8 IsLeapYear(u16 year)
{
    if(((year%4 == 0) && (year%100 != 0)) || (year%400 == 0))
        return 1;
    return 0;
}

u8 CorrectYear(u8 year)
{
    if (year>99) year = 99;
    return year;
}

//-------------------------------------------------------------------
//函数名称：IsLegal
//功能描述：判断年月日是否合法
//调用清单：IsLeapYear
//被调清单：
//参数说明：year,mon,day
//输出说明：合法，非法
//返回  值：1,0
//其    他：
//-------------------------------------------------------------------
u8 IsLegalDate(u8 year,u8 mon,u8 date)
{
    u16 w_year;
    u8 month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    w_year=2000 + year;
    if (IsLeapYear(w_year)) month[1] = 29; 
    return  mon <= 12 && date <= month[mon - 1]; 
}

void CorrectDate(u8 year,u8 *mon,u8 *date)
{
    u16 w_year;
    u8 month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    w_year=2000 + year;
    if (IsLeapYear(w_year)) month[1] = 29; 
    if(*mon > 12) *mon = 1;
    if(*date > month[*mon - 1]) *date = 1;
}

//-------------------------------------------------------------------
//函数名称：IsLegal
//功能描述：判断年月日是否合法
//调用清单：IsLeapYear
//被调清单：
//参数说明：year,mon,day
//输出说明：合法，非法
//返回  值：1,0
//其    他：
//-------------------------------------------------------------------
u8 IsLegalTime(u8 hour,u8 min,u8 sec)
{
    if(hour < 24 || min < 60 || sec < 60) return 1;
    return 0;
}

void CorrectTime(u8 *hour,u8 *min,u8 *sec)
{
    if(*hour > 23 ) *hour = 0;
    if(*min > 59 ) *min = 0;
    if(*sec > 59) *sec = 0;
}











