#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define COM_REC_LEN             64  	//����comͨѶ�������ֽ��� 64
#define HMI_REC_LEN             64  	//����hmiͨѶ�������ֽ��� 64
#define EN_USART_RX 			1		//ʹ�ܣ�1��/��ֹ��0�����ڽ���
	  	
extern u8  COM_RX_BUF[COM_REC_LEN]; //���ջ���,���COM_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 COM_RX_STA;         		//����״̬���	
extern u8  HMI_RX_BUF[HMI_REC_LEN]; //���ջ���,���HMI_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 HMI_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void UART1_Init(u32 bound);
void USART3_Init(u32 bound);
void Uart3_PutString(char * buf , u8 len);
#endif


