#include "exti.h"
#include "delay.h" 
//#include "sys.h"
//#include "led.h" 
//#include "key.h"
//#include "beep.h"
#include "stepmoto.h"

void PWM_OFF(u8 no)
{
    DMA_Cmd(StepMotor[no].DMA_Stream,DISABLE);
    TIM_DMACmd(StepMotor[no].timer,StepMotor[no].TIM_DMASource,DISABLE);
    (StepMotor[no].timer) -> CCER &= ~StepMotor[no].CCER; //关闭TIM PWM输出
    TIM_Cmd((StepMotor[no].timer),DISABLE);
}

//外部中断5~9服务程序
void EXTI9_5_IRQHandler(void)
{

    if(EXTI_GetITStatus(EXTI_Line7) != RESET) {
        
        if(!GPIO_ReadOutputDataBit (GPIOD,GPIO_Pin_5)) {
            PWM_OFF(0);
//PDout(0)=1;
        }
        EXTI_ClearITPendingBit(EXTI_Line7);    // Clear the EXTI line 7 pending bit  
    }
    if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
        
        if(GPIO_ReadOutputDataBit (GPIOD,GPIO_Pin_4)) {
            PWM_OFF(1);
//PDout(0)=1;
        }
        EXTI_ClearITPendingBit(EXTI_Line8);    // Clear the EXTI line 8 pending bit  
    }
    if(EXTI_GetITStatus(EXTI_Line9) != RESET) {
        
        if(GPIO_ReadOutputDataBit (GPIOD,GPIO_Pin_6)) {
            PWM_OFF(2);
//PDout(0)=1;
        }
        EXTI_ClearITPendingBit(EXTI_Line9);    // Clear the EXTI line 9 pending bit  
    }
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line10) != RESET) {
        
        if(GPIO_ReadOutputDataBit (GPIOD,GPIO_Pin_2)) {
            PWM_OFF(3);
//PDout(0)=1;
        }
        EXTI_ClearITPendingBit(EXTI_Line10);    // Clear the EXTI line 10 pending bit  
    }
}

//外部中断初始化程序
//初始化PE7~10为中断输入.
void EXTIX_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化    
    
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource7);//PE7 连接到中断线7
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);//PE8 连接到中断线8
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);//PE9 连接到中断线9
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource10);//PE10 连接到中断线10
	
	/* 配置EXTI_Line7,8,9,10 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发//EXTI_Trigger_Rising; //上降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
    EXTI_Init(&EXTI_InitStructure);//配置
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断5~9
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断10~15
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//抢占优先级4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
	   
}












