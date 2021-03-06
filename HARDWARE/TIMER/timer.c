#include "timer.h"
//#include "led.h"
//#include "led.h"
//#include "malloc.h"
//#include "usart.h"
//#include "string.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//FreeRTOS时间统计所用的节拍计数器
volatile unsigned long long FreeRTOSRunTimeTicks;

//初始化TIM3使其为FreeRTOS的时间统计提供时基
//void ConfigureTimeForRunTimeStats(void)
//{
//	//定时器3初始化，定时器时钟为84M，分频系数为84-1，所以定时器3的频率
//	//为84M/84=1M，自动重装载为50-1，那么定时器周期就是50us
//	FreeRTOSRunTimeTicks=0;
//	TIM3_Int_Init(50-1,84-1);	//初始化TIM3
//}

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
//void TIM3_Int_Init(u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
//	
//	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
//	
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//	TIM_Cmd(TIM3,ENABLE); //使能定时器3
//	
//	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级0
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//}

//void TIM3_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//    /*输入管脚PD2*/
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//    /*计数器*/
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//    TIM_DeInit(TIM3); 
//    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
//    TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
//    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure); // Time base configuration   
//    //TIM_TIxExternalClockConfig(TIM3, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);//TIM3 的chanl2 做外部时钟的输入管脚。
//    /* TIM3 的ETR管脚做外部时钟的输入管脚用这种模式*/
//    //TIM_ETRClockMode1Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted,0);
//    TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
//    TIM_SetCounter(TIM3, 0);   // 清零计数器CNT
//}

//定时器3中断服务函数
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//	{
//		FreeRTOSRunTimeTicks++;
//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
//}

void TIM2_PWM_Init(void)
{//产生LED脉冲信号1120Hz
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//GPIO_StructInit(&GPIO_InitStructure);
	
	//配置GPIOB_Pin_11，作为TIM_Channel2 PWM输出*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //指定第11引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //模式必须为复用！
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //频率为快速
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉与否对PWM产生无影响
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);//复用GPIOB_Pin_11为TIM2_CH4
	
	//TIM_DeInit(TIM2);//初始化TIM2寄存器
	TIM_TimeBaseStructure.TIM_Period = 300-1; //84000000/(300*250)=1120Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 250-1;//84-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;//0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/*配置输出比较，产生占空比为20%的PWM方波*/
	//TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1为正常占空比模式，PWM2为反极性模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//输出反相 TIM_OCNPolarity_Low;//输出同相，
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	TIM_OCInitStructure.TIM_Pulse = 150;//输入CCR（占空比数值）50% TIM_Period
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	//TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);//CCR自动装载默认也是打开的
	//TIM_ARRPreloadConfig(TIM2, ENABLE); //ARR自动装载默认是打开的，可以不设置
	//TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	//TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE); //使能TIM2定时器
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
}


void TIM4_PWM_Config(void)
{
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    TIM_OCInitTypeDef    TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 1000;//50
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    TIM_OCStructInit(&TIM_OCInitStructure);

    //初始化TIM4 Channel1 PWM模式	
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;////
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse =50;//100;//10????
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    
    //初始化TIM4 Channel2 PWM模式	 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_Pulse =50;//100;//10????
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2

    //初始化TIM4 Channel3 PWM模式	 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_Pulse =50;//100;//10????
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC3

    TIM_DMACmd(TIM4,TIM_DMA_CC1,DISABLE);
    TIM_DMACmd(TIM4,TIM_DMA_CC2,DISABLE);
    TIM_DMACmd(TIM4,TIM_DMA_CC3,DISABLE);
    TIM4 -> CCER &= ~(1<<0); //关闭TME4 PWM输出
    TIM4 -> CCER &= ~(1<<4); //关闭TME4 PWM输出
    TIM4 -> CCER &= ~(1<<8); //关闭TME4 PWM输出
    TIM_Cmd(TIM4,DISABLE);
}

void TIM3_PWM_Config(void)
{
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    TIM_OCInitTypeDef    TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 1000;//50
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    TIM_OCStructInit(&TIM_OCInitStructure);
    //初始化TIM3 Channel1 PWM模式	 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_Pulse =50;//100;//10????
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3
    
    //初始化TIM3 Channel2 PWM模式	 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_Pulse =50;//100;//10????
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC4
    
    TIM_DMACmd(TIM3,TIM_DMA_CC1,DISABLE);
    TIM_DMACmd(TIM3,TIM_DMA_CC2,DISABLE);

    
    TIM3 -> CCER &= ~(1<<0); //关闭TME4 PWM输出
    TIM3 -> CCER &= ~(1<<4); //关闭TME4 PWM输出
    
    TIM_Cmd(TIM3,DISABLE);
}
