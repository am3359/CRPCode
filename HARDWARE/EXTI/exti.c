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
    (StepMotor[no].timer) -> CCER &= ~StepMotor[no].CCER; //�ر�TIM PWM���
    TIM_Cmd((StepMotor[no].timer),DISABLE);
}

//�ⲿ�ж�5~9�������
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

//�ⲿ�жϳ�ʼ������
//��ʼ��PE7~10Ϊ�ж�����.
void EXTIX_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��    
    
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource7);//PE7 ���ӵ��ж���7
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource8);//PE8 ���ӵ��ж���8
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);//PE9 ���ӵ��ж���9
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource10);//PE10 ���ӵ��ж���10
	
	/* ����EXTI_Line7,8,9,10 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���//EXTI_Trigger_Rising; //�Ͻ��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
    EXTI_Init(&EXTI_InitStructure);//����
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�5~9
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//�ⲿ�ж�10~15
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;//��ռ���ȼ�4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);//����
	   
}












