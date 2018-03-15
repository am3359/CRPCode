#ifndef __DMA_H
#define __DMA_H
#include "sys.h"
#include "stepmoto.h"

//extern TStepMotor StepMotor[5];

void TIM_PWM_DMA_Config(u8 no);

void DMA1_Init(void);
#endif
