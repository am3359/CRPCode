#ifndef __DMA_H
#define __DMA_H
#include "sys.h"

void TIM4_PWMDMA_Config(u16 *DataBuf,u32 BufSize,u32 MemoryInc);
//void DMA1_Stream0_CH2_Cmd(void (*Fuc)(uint16_t *,int32_t,int32_t),uint16_t *DataBuf,int32_t BufSize,int32_t MemoryInc);
void DMA1_Init(void);
#endif
