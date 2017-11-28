#include "spi.h"

extern u16 SPI2_Rx_Buff[1024];   //DMAָ���ڴ��ַ
extern u16 SPI2_Tx_Cnt;
extern u16 SPI2_Tx_Byte;

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI1�ĳ�ʼ��
void SPI1_Init(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//ʹ��SPI1ʱ��
 
	//PA5,6,7��ʼ������
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA5~7���ù������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//����  
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);//PA5����Ϊ SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);//PA6����Ϊ SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);//PA7����Ϊ SPI1

	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//��λSPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//ֹͣ��λSPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ32
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����

	//SPI1_ReadWriteByte(0xff);//��������
}   
//SPI1�ٶ����ú���
//SPI�ٶ�=fAPB2/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2ʱ��һ��Ϊ84Mhz��
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
	SPI1->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
	SPI1->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ� 
	SPI_Cmd(SPI1,ENABLE); //ʹ��SPI1
} 
//SPI1 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ��byte  ����
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte  
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����	
}

void SPI2_Init(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);//ʹ��SPI2ʱ��
	
	//PB13~15��ʼ������
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//PB13~15���ù������	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//����  
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);  //�����ŵĸ��ù���
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//��λSPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//ֹͣ��λSPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ4
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����

	//SPI2_ReadWriteByte(0xff);//��������
}

//SPI2 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ������
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����	
}

/*
void SPI2_DMA_Config(u16 num)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	TIM_Cmd(TIM5,DISABLE);
	// TIM5 Update DMA Request disable
	TIM_DMACmd(TIM5, TIM_DMA_CC2 | TIM_DMA_CC4, DISABLE);//

	// DMA disable
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	
	DMA_DeInit(DMA1_Stream3);
	DMA_DeInit(DMA1_Stream4);
	 
	//   SPI2 RX DMA ����  Stream3 
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;         //ָ��DMA���������ַΪSPI2�����ݵ�ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI2_Rx_Buff;         //ָ��DMA���ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //DMA���䷽��Ϊ������ д���ڴ�
	DMA_InitStructure.DMA_BufferSize = num ;//��2���͵�9�����������⣬����+8����һ��ADֵ��Ч����+1//100;//DataSize;                            //��������(0-65535������Ϊ0)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //ʧ�������ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //ʹ���ڴ��ַ���� ��ȥFORѭ��
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//PSIZE=16bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;	//MSIZE=16bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//       //DMAģʽΪ��ѭ��ģʽ,��ѭ��ģʽֻ���е��δ��䡣
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     //����ȨΪ��
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //ʧ��FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;       //FIFO�ķ�ֵΪ����
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;	            //�ڴ�ͻ������Ϊ��һ
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //����ͻ������Ϊ��һ
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);                             //��ʼ��DMA1_Stream3
	
	//   SPI2 TX DMA ����   Stream4
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;         //ָ��DMA���������ַΪSPI2�����ݵ�ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&SPI2_Tx_Byte;         //ָ��DMA���ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //DMA���䷽��Ϊ���ڴ棬д����
	DMA_InitStructure.DMA_BufferSize = num ;//DataSize;                            //��������(0-65535������Ϊ0)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //ʧ�������ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_PeripheralInc_Disable;                 //ʧ���ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//PSIZE=16bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;	//MSIZE=16bit	   
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//       //DMAģʽΪ��ѭ��ģʽ,��ѭ��ģʽֻ���е��δ��䡣
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium ;                  //����ȨΪ�е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //ʧ��FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;       //FIFO�ķ�ֵΪ����
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;	            //�ڴ�ͻ������Ϊ��һ
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //����ͻ������Ϊ��һ
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);                             //��ʼ��DMA1_Stream4

	TIM_DMACmd(TIM5, TIM_DMA_CC2 | TIM_DMA_CC4, ENABLE);//
	// DMA enable
	DMA_Cmd(DMA1_Stream3, ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);
	
	SPI2_Tx_Byte = 0;
	SPI2_Tx_Cnt = 0;

	TIM_ClearFlag(TIM5,TIM_FLAG_CC2 | TIM_FLAG_CC4);//TIM_FLAG_CC2 | TIM_FLAG_CC4 //0x1eff

	TIM_Cmd(TIM5,ENABLE);
}
	
void PWM_Init(void)
{//���8MHz
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	
	//����PA1����ΪTIM5_Ch1 PWM���
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;//ָ����һ����//GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_12 |GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//ģʽ����Ϊ���ã�
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//Ƶ��Ϊ����
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//GPIO_PuPd_NOPULL;//��������PWM������Ӱ��
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	
	//TIM_DeInit(TIM5);//��ʼ��TIM5�Ĵ���
	//ʱ����ʼ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //���������á�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���
	TIM_TimeBaseStructure.TIM_Prescaler = 6 - 1;   //��Ϊ���APB1��Ƶ��Ϊ1,��ʱʱ��*2,ʱ��Ϊ168M/6*2=14MHz,
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;//0;
	TIM_TimeBaseStructure.TIM_Period = 35 - 1;  //Period = (Timer clock / TIM PWM clock) - 1 = 14M / 400K = 35
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	//��������Ƚϣ�����PWM���� 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//������� TIM_OCNPolarity_Low;//���ͬ�࣬
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = 23;//15;//ccr1;
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 21;//15;//ccr2;
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	
	//TIM_OCInitStructure.TIM_Pulse = 14;//14;//ccr3;
	//TIM_OC3Init(TIM5,&TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 21;//14;//ccr4;
	TIM_OC4Init(TIM5,&TIM_OCInitStructure);
	//TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);//CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	//TIM_ARRPreloadConfig(TIM5,ENABLE);//ARR�Զ�װ��Ĭ���Ǵ򿪵ģ����Բ�����
	//TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_CC4,ENABLE);

	TIM_Cmd(TIM5,ENABLE);
	TIM_CtrlPWMOutputs(TIM5,ENABLE);
	
	// TIM5 Update DMA Request enable 
	//TIM_DMACmd(TIM5, TIM_DMA_CC2 | TIM_DMA_CC4, ENABLE);//

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;//spi2 rx;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Sort_Ad(u16 *ad,u16 cnt) 
{
  u16 temp;
  u16 i,j;
  for(i=0;i<cnt;i++) 
  {
    for(j=0;j<cnt-i-1;j++) 
    {
      if(ad[j]<ad[j+1]) 
      {
        temp =ad[j+1];
        ad[j+1]=ad[j];
        ad[j] = temp;
      }
    }
  }
}

void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_CC4)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC4);
		if(SPI2_Tx_Cnt < 4)
		{
			SPI2_Tx_Byte = 0xffdc;//д�Ĵ���,�����ԶԵ�,��IN0ɨ�赽IN7,ȫ����,�ⲿ��׼��ѹԴ���¶ȴ���������,���ض�CFG
			SPI2_Tx_Cnt++;
		}
		else if(SPI2_Tx_Cnt == 4)
		{
			SPI2_Tx_Byte = 0x7fdc;//0x7ffc;
			SPI2_Tx_Cnt++;
		}
	}
}*/
