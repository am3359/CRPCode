#include "sys.h"
#include "usart.h"	
#include "string.h"
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用
#include "task.h"
#include "queue.h"
#endif
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 COM_RX_BUF[COM_REC_LEN];     //接收缓冲,最大COM_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 COM_RX_STA=0;       //接收状态标记	

u8 HMI_RX_BUF[HMI_REC_LEN];     //接收缓冲,最大HMI_REC_LEN个字节.
u16 HMI_RX_STA=0;       //接收状态标记	


//初始化IO 串口1 
//bound:波特率
void UART1_Init(u32 bound){
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
		
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;//抢占优先级7
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
}

u8 Uart1_PutChar(u8 ch)
{
  /* Write a character to the USART */
  USART_SendData(USART1, (u8) ch);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {
  }
  return ch;
}

void Uart1_PutString(u8 * buf , u8 len)
{
    u32 i;
    for(i=0;i<len;i++)
    {
        Uart1_PutChar(*buf++);
    }
}

extern QueueHandle_t Com_Queue;	//信息队列句柄，消息队列长度为COM_Q_NUM = 4

void USART1_IRQHandler(void)                	//串口1中断服务程序
{//以0x0D 0x0A 结尾
	u8 Res;
	BaseType_t xHigherPriorityTaskWoken;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0D 0x0A结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((COM_RX_STA & 0x8000)==0)//接收未完成
		{
			if(COM_RX_STA & 0x4000)//接收到了0x0D
			{
				if(Res==0x0a)
				{//接收完成了 
					COM_RX_STA|=0x8000;
				}
				else
				{//接收错误,重新开始
					COM_RX_STA=0;	
					memset(COM_RX_BUF,0,COM_REC_LEN);//清除数据接收缓冲区COM_RX_BUF,用于下一次数据接收
				}
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)
				{//收到0X0D
					COM_RX_STA|=0x4000;
				}
				else
				{//不是0X0D保持数据
					COM_RX_BUF[COM_RX_STA & 0X3FFF]=Res ;
					COM_RX_STA++;
					if((COM_RX_STA & 0X3FFF)>(COM_REC_LEN-1))//if(COM_RX_STA>(COM_REC_LEN-1))
					{//接收数据错误,重新开始接收
						COM_RX_STA=0;	
						memset(COM_RX_BUF,0,COM_REC_LEN);//清除数据接收缓冲区COM_RX_BUF,用于下一次数据接收
					}
				}
			}
		}

		if((COM_RX_STA & 0x8000)&&(Com_Queue!=NULL))
		{//向队列发送接收到的数据
			if((COM_RX_STA & 0X3FFF)>0)
			{//长度超过1个字节才发送
				xQueueSendFromISR(Com_Queue,COM_RX_BUF,&xHigherPriorityTaskWoken);//向队列中发送数据
				
				COM_RX_STA=0;	
				memset(COM_RX_BUF,0,COM_REC_LEN);//清除数据接收缓冲区COM_RX_BUF,用于下一次数据接收
					
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
			}
			else
			{
				COM_RX_STA=0;	
				memset(COM_RX_BUF,0,COM_REC_LEN);//清除数据接收缓冲区COM_RX_BUF,用于下一次数据接收
			}
		}
	}
} 


void USART3_Init(u32 bound){
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3); //GPIOC10复用为USART3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); //GPIOC11复用为USART3

	//USART3端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10与GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10,PC11

   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_GetFlagStatus(USART3, USART_FLAG_TC);//首字节丢失问题
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//USART3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8;//抢占优先级8
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

extern QueueHandle_t Hmi_Queue;	//信息队列句柄，消息队列长度为HMI_Q_NUM = 16

void USART3_IRQHandler(void)                	//串口3中断服务程序
{//以0xff 0xff 0xff结尾
	u8 Res;
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据
		
		if((COM_RX_STA & 0xc000)!=0xc000)//接收未完成
		{
			HMI_RX_BUF[HMI_RX_STA & 0X3FFF]=Res ;
			HMI_RX_STA++;

			if(Res == 0xff) HMI_RX_STA += 0x4000;
			else HMI_RX_STA &= 0X3FFF;

			if((HMI_RX_STA & 0X3FFF)>(HMI_REC_LEN-1))
			{
				HMI_RX_STA=0;//接收数据错误,重新开始接收
				memset(HMI_RX_BUF,0,HMI_REC_LEN);
			}
		}
		
		if(((HMI_RX_STA & 0xc000)==0xc000)&&(Hmi_Queue!=NULL))
		{//向队列发送接收到的数据
			if((HMI_RX_STA & 0X3FFF)>3)
			{//除了0xff 0xff 0xff长度超过1个字节才发送
				HMI_RX_BUF[(HMI_RX_STA & 0X3FFF)-3]=0;
				//HMI_RX_BUF[(HMI_RX_STA & 0X3FFF)-2]=0;
				//HMI_RX_BUF[(HMI_RX_STA & 0X3FFF)-1]=0;
				xQueueSendFromISR(Hmi_Queue,HMI_RX_BUF,&xHigherPriorityTaskWoken);//向队列中发送数据
				
				HMI_RX_STA=0;	
				memset(HMI_RX_BUF,0,HMI_REC_LEN);//清除数据接收缓冲区HMI_RX_BUF,用于下一次数据接收
			
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
			}
			else
			{
				HMI_RX_STA=0;	
				memset(HMI_RX_BUF,0,HMI_REC_LEN);//清除数据接收缓冲区HMI_RX_BUF,用于下一次数据接收
			}
		}
		
	}
//65 01 01 01 FF FF FF 
//65 01 02 01 FF FF FF 
//65 01 03 01 FF FF FF 
//65 01 04 01 FF FF FF 
//70 31 32 33 34 FF FF FF 

}

u8 Uart3_PutChar(u8 ch)
{
  /* Write a character to the USART */
  USART_SendData(USART3, (u8) ch);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {
  }
  return ch;
}

void Uart3_PutString(char * buf , u8 len)
{
    char i;
    for(i=0;i<len;i++)
    {
        Uart3_PutChar(*buf++);
    }
}

#endif
