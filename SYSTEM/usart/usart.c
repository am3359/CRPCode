#include "sys.h"
#include "usart.h"	
#include "string.h"
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��
#include "task.h"
#include "queue.h"
#endif
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 COM_RX_BUF[COM_REC_LEN];     //���ջ���,���COM_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 COM_RX_STA=0;       //����״̬���	

u8 HMI_RX_BUF[HMI_REC_LEN];     //���ջ���,���HMI_REC_LEN���ֽ�.
u16 HMI_RX_STA=0;       //����״̬���	


//��ʼ��IO ����1 
//bound:������
void UART1_Init(u32 bound){
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
		
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;//��ռ���ȼ�7
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
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

extern QueueHandle_t Com_Queue;	//��Ϣ���о������Ϣ���г���ΪCOM_Q_NUM = 4

void USART1_IRQHandler(void)                	//����1�жϷ������
{//��0x0D 0x0A ��β
	u8 Res;
	BaseType_t xHigherPriorityTaskWoken;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0D 0x0A��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((COM_RX_STA & 0x8000)==0)//����δ���
		{
			if(COM_RX_STA & 0x4000)//���յ���0x0D
			{
				if(Res==0x0a)
				{//��������� 
					COM_RX_STA|=0x8000;
				}
				else
				{//���մ���,���¿�ʼ
					COM_RX_STA=0;	
					memset(COM_RX_BUF,0,COM_REC_LEN);//������ݽ��ջ�����COM_RX_BUF,������һ�����ݽ���
				}
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)
				{//�յ�0X0D
					COM_RX_STA|=0x4000;
				}
				else
				{//����0X0D��������
					COM_RX_BUF[COM_RX_STA & 0X3FFF]=Res ;
					COM_RX_STA++;
					if((COM_RX_STA & 0X3FFF)>(COM_REC_LEN-1))//if(COM_RX_STA>(COM_REC_LEN-1))
					{//�������ݴ���,���¿�ʼ����
						COM_RX_STA=0;	
						memset(COM_RX_BUF,0,COM_REC_LEN);//������ݽ��ջ�����COM_RX_BUF,������һ�����ݽ���
					}
				}
			}
		}

		if((COM_RX_STA & 0x8000)&&(Com_Queue!=NULL))
		{//����з��ͽ��յ�������
			if((COM_RX_STA & 0X3FFF)>0)
			{//���ȳ���1���ֽڲŷ���
				xQueueSendFromISR(Com_Queue,COM_RX_BUF,&xHigherPriorityTaskWoken);//������з�������
				
				COM_RX_STA=0;	
				memset(COM_RX_BUF,0,COM_REC_LEN);//������ݽ��ջ�����COM_RX_BUF,������һ�����ݽ���
					
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//�����Ҫ�Ļ�����һ�������л�
			}
			else
			{
				COM_RX_STA=0;	
				memset(COM_RX_BUF,0,COM_REC_LEN);//������ݽ��ջ�����COM_RX_BUF,������һ�����ݽ���
			}
		}
	}
} 


void USART3_Init(u32 bound){
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3); //GPIOC10����ΪUSART3
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); //GPIOC11����ΪUSART3

	//USART3�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10��GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10,PC11

   //USART3 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_GetFlagStatus(USART3, USART_FLAG_TC);//���ֽڶ�ʧ����
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

	//USART3 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8;//��ռ���ȼ�8
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

extern QueueHandle_t Hmi_Queue;	//��Ϣ���о������Ϣ���г���ΪHMI_Q_NUM = 16

void USART3_IRQHandler(void)                	//����3�жϷ������
{//��0xff 0xff 0xff��β
	u8 Res;
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//��ȡ���յ�������
		
		if((COM_RX_STA & 0xc000)!=0xc000)//����δ���
		{
			HMI_RX_BUF[HMI_RX_STA & 0X3FFF]=Res ;
			HMI_RX_STA++;

			if(Res == 0xff) HMI_RX_STA += 0x4000;
			else HMI_RX_STA &= 0X3FFF;

			if((HMI_RX_STA & 0X3FFF)>(HMI_REC_LEN-1))
			{
				HMI_RX_STA=0;//�������ݴ���,���¿�ʼ����
				memset(HMI_RX_BUF,0,HMI_REC_LEN);
			}
		}
		
		if(((HMI_RX_STA & 0xc000)==0xc000)&&(Hmi_Queue!=NULL))
		{//����з��ͽ��յ�������
			if((HMI_RX_STA & 0X3FFF)>3)
			{//����0xff 0xff 0xff���ȳ���1���ֽڲŷ���
				HMI_RX_BUF[(HMI_RX_STA & 0X3FFF)-3]=0;
				//HMI_RX_BUF[(HMI_RX_STA & 0X3FFF)-2]=0;
				//HMI_RX_BUF[(HMI_RX_STA & 0X3FFF)-1]=0;
				xQueueSendFromISR(Hmi_Queue,HMI_RX_BUF,&xHigherPriorityTaskWoken);//������з�������
				
				HMI_RX_STA=0;	
				memset(HMI_RX_BUF,0,HMI_REC_LEN);//������ݽ��ջ�����HMI_RX_BUF,������һ�����ݽ���
			
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//�����Ҫ�Ļ�����һ�������л�
			}
			else
			{
				HMI_RX_STA=0;	
				memset(HMI_RX_BUF,0,HMI_REC_LEN);//������ݽ��ջ�����HMI_RX_BUF,������һ�����ݽ���
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
