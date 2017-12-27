#include "dma.h"
/************************************************************************************
函数名称：
输入参数：
输出参数：
功能描述：
************************************************************************************/
void TIM4_PWMDMA_Config(u16 *DataBuf,u32 BufSize,u32 MemoryInc)
{
    DMA_InitTypeDef  DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Stream4); 
    DMA_StructInit(&DMA_InitStructure);
    
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->ARR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DataBuf;
    DMA_InitStructure.DMA_BufferSize = BufSize;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = MemoryInc;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream0,&DMA_InitStructure);
    
    DMA_ITConfig(DMA1_Stream0,DMA_IT_TC,ENABLE);
    //DMA_Cmd(DMA1_Stream0,DISABLE);
}
/************************************************************************************
函数名称：
输入参数：
输出参数：
功能描述：
************************************************************************************/
//void DMA1_Stream0_CH2_Cmd(void (*Fuc)(uint16_t *,int32_t,int32_t),uint16_t *DataBuf,int32_t BufSize,int32_t MemoryInc)
//{
//    //flag_DMA1_Stream0_CH2 = 1;
//    Fuc(DataBuf,BufSize,MemoryInc);
//    DMA_Cmd(DMA1_Stream0,ENABLE);
//}
/************************************************************************************
函数名称：
输入参数：
输出参数：
功能描述：
************************************************************************************/
void DMA1_Init(void)
{
    NVIC_InitTypeDef    NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;  //DMA1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;//抢占优先级9  或者6？？
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);
}


