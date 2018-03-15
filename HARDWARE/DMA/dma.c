#include "dma.h"

/************************************************************************************
�������ƣ�
���������
���������
����������
************************************************************************************/
void TIM_PWM_DMA_Config(u8 no)
{
    DMA_InitTypeDef  DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(StepMotor[no].RCC_AHB1Periph, ENABLE);

    DMA_DeInit(StepMotor[no].DMA_Stream); 
    DMA_StructInit(&DMA_InitStructure);
    
    DMA_InitStructure.DMA_Channel = StepMotor[no].DMA_Channel;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(StepMotor[no].timer)->ARR;  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)StepMotor[no].Buf[StepMotor[no].stage];
    DMA_InitStructure.DMA_BufferSize = StepMotor[no].Size[StepMotor[no].stage];
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = StepMotor[no].IncEn[StepMotor[no].stage];
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(StepMotor[no].DMA_Stream,&DMA_InitStructure);
    
    DMA_ITConfig(StepMotor[no].DMA_Stream,DMA_IT_TC,ENABLE);
    
    DMA_Cmd(StepMotor[no].DMA_Stream,ENABLE);
    TIM_DMACmd(StepMotor[no].timer,StepMotor[no].TIM_DMASource,ENABLE);
    (StepMotor[no].timer)->CCER |= StepMotor[no].CCER; //��TIM PWM���
    TIM_Cmd((StepMotor[no].timer),ENABLE);
}

/************************************************************************************
�������ƣ�
���������
���������
����������
************************************************************************************/
void DMA1_Init(void)
{
    NVIC_InitTypeDef    NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;  //DMA1�ж���0 �����˶�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;//��ռ���ȼ�9  ����??���ȼ��ߵ���ôѡ��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;  //DMA1�ж���3 �����˶�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;//��ռ���ȼ�10  ����??���ȼ��ߵ���ôѡ��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;  //DMA1�ж���4 ע����1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;//��ռ���ȼ�11  ����??���ȼ��ߵ���ôѡ��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;  //DMA1�ж���5 ע����2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;//��ռ���ȼ�12  ����??���ȼ��ߵ���ôѡ��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;  //DMA1�ж���7 �䶯����ת
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;//��ռ���ȼ�13  ����??���ȼ��ߵ���ôѡ��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);
}
