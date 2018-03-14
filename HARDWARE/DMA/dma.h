#ifndef __DMA_H
#define __DMA_H
#include "sys.h"
#include "stepmoto.h"

extern TStepMotor StepMotor[5];

void TIM_PWM_DMA_Config(u8 no);

//void TIM4_PWMDMA_Config0(u16 *DataBuf,u32 BufSize,u32 MemoryInc);
//void TIM4_PWMDMA_Config1(u16 *DataBuf,u32 BufSize,u32 MemoryInc);
//void TIM3_PWMDMA_Config0(u16 *DataBuf,u32 BufSize,u32 MemoryInc);
//void TIM3_PWMDMA_Config1(u16 *DataBuf,u32 BufSize,u32 MemoryInc);
//void TIM4_PWMDMA_Config4(u16 *DataBuf,u32 BufSize,u32 MemoryInc);
void DMA1_Init(void);
#endif
