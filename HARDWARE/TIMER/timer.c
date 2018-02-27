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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

//FreeRTOSʱ��ͳ�����õĽ��ļ�����
volatile unsigned long long FreeRTOSRunTimeTicks;

//��ʼ��TIM3ʹ��ΪFreeRTOS��ʱ��ͳ���ṩʱ��
//void ConfigureTimeForRunTimeStats(void)
//{
//	//��ʱ��3��ʼ������ʱ��ʱ��Ϊ84M����Ƶϵ��Ϊ84-1�����Զ�ʱ��3��Ƶ��
//	//Ϊ84M/84=1M���Զ���װ��Ϊ50-1����ô��ʱ�����ھ���50us
//	FreeRTOSRunTimeTicks=0;
//	TIM3_Int_Init(50-1,84-1);	//��ʼ��TIM3
//}

//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
//void TIM3_Int_Init(u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
//	
//	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
//	
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
//	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
//	
//	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�0
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//}

//void TIM3_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//    /*����ܽ�PD2*/
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//    /*������*/
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//    TIM_DeInit(TIM3); 
//    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
//    TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
//    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure); // Time base configuration   
//    //TIM_TIxExternalClockConfig(TIM3, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);//TIM3 ��chanl2 ���ⲿʱ�ӵ�����ܽš�
//    /* TIM3 ��ETR�ܽ����ⲿʱ�ӵ�����ܽ�������ģʽ*/
//    //TIM_ETRClockMode1Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted,0);
//    TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
//    TIM_SetCounter(TIM3, 0);   // ���������CNT
//}

//��ʱ��3�жϷ�����
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
//	{
//		FreeRTOSRunTimeTicks++;
//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
//}

void TIM3_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    
    //����GPIOC_Pin_6����ΪTIM_Channel1 PWM���*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //ָ����������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //ģʽ����Ϊ���ã�
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //Ƶ��Ϊ����
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //��������PWM������Ӱ��
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);//����GPIOC_Pin_6ΪTIM3_CH4
    
    //TIM_DeInit(TIM3);//��ʼ��TIM3�Ĵ���//Ƶ��84000000/(TIM_Prescaler*TIM_Period)
    TIM_TimeBaseStructure.TIM_Period = 300-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 250-1; //84000000/(300*250)=1120Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
    //TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;//0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    /*��������Ƚϣ�����ռ�ձ�Ϊ20%��PWM����*/
    //TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//������� TIM_OCNPolarity_Low;//���ͬ�࣬
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    TIM_OCInitStructure.TIM_Pulse = 150;//����CCR��ռ�ձ���ֵ��50% TIM_Period
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_Cmd(TIM3, ENABLE); //ʹ��TIM3��ʱ��
    TIM_CtrlPWMOutputs(TIM3,ENABLE);
}

void TIM4_PWM_Config(void)
{
    //GPIO_InitTypeDef    GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    TIM_OCInitTypeDef    TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);////
//------------------------------------------------------------------------------------    
//    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
//    
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);
//------------------------------------------------------------------------------------    
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 1000;//50
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    TIM_OCStructInit(&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;////
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse =50;//100;//10????
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
//------------------------------------------------------------------------------------    
    //TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
    //TIM_ARRPreloadConfig(TIM4,ENABLE);
    //TIM_CtrlPWMOutputs(TIM4,ENABLE); 
//    TIM_DMAConfig(TIM4,TIM_DMABase_ARR,TIM_DMABurstLength_3Transfers);
    TIM_DMACmd(TIM4,TIM_DMA_CC1,DISABLE);
    TIM4 -> CCER &= ~(1<<0); //�ر�TME4 PWM���
    TIM_Cmd(TIM4,DISABLE);
}
