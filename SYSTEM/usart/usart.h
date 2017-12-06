#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define COM_REC_LEN             64  	//定义com通讯最大接收字节数 64
#define HMI_REC_LEN             64  	//定义hmi通讯最大接收字节数 64
#define EN_USART_RX 			1		//使能（1）/禁止（0）串口接收
	  	
extern u8  COM_RX_BUF[COM_REC_LEN]; //接收缓冲,最大COM_REC_LEN个字节.末字节为换行符 
extern u16 COM_RX_STA;         		//接收状态标记	
extern u8  HMI_RX_BUF[HMI_REC_LEN]; //接收缓冲,最大HMI_REC_LEN个字节.末字节为换行符 
extern u16 HMI_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void UART1_Init(u32 bound);
void USART3_Init(u32 bound);
void Uart3_PutString(char * buf , u8 len);
#endif


