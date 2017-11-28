#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

void SPI1_Init(void);			            //初始化SPI1口
void SPI1_SetSpeed(u8 SpeedSet);            //设置SPI1速度   
u8 SPI1_ReadWriteByte(u8 TxData);           //SPI1总线读写一个字节

void SPI2_Init(void);
u8 SPI2_ReadWriteByte(u8 TxData);

void SPI2_DMA_Config(u16 num);
void PWM_Init(void);

void Sort_Ad(u16 *ad,u16 cnt);
#endif

