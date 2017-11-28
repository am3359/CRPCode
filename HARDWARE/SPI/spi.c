#include "spi.h"

extern u16 SPI2_Rx_Buff[1024];   //DMA指向内存地址
extern u16 SPI2_Tx_Cnt;
extern u16 SPI2_Tx_Byte;

//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI口初始化
//这里针是对SPI1的初始化
void SPI1_Init(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//使能SPI1时钟
 
	//PA5,6,7初始化设置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA5~7复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//上拉  
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);//PA5复用为 SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);//PA6复用为 SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);//PA7复用为 SPI1

	//这里只针对SPI口初始化
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//定义波特率预分频的值:波特率预分频值为32
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设

	//SPI1_ReadWriteByte(0xff);//启动传输
}   
//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
} 
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		 			 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个byte  数据
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
}

void SPI2_Init(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);//使能SPI2时钟
	
	//PB13~15初始化设置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//PB13~15复用功能输出	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//上拉  
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);  //打开引脚的复用功能
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

	//这里只针对SPI口初始化
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//定义波特率预分频的值:波特率预分频值为4
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设

	//SPI2_ReadWriteByte(0xff);//启动传输
}

//SPI2 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个数据
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据	
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
	 
	//   SPI2 RX DMA 配置  Stream3 
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;         //指定DMA的外设基地址为SPI2的数据地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI2_Rx_Buff;         //指定DMA的内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //DMA传输方向为读外设 写到内存
	DMA_InitStructure.DMA_BufferSize = num ;//第2个和第9个经常出问题，所以+8，第一个AD值无效所以+1//100;//DataSize;                            //传输数量(0-65535，不能为0)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //失能外设地址增长
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //使能内存地址增长 免去FOR循环
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//PSIZE=16bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;	//MSIZE=16bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//       //DMA模式为非循环模式,非循环模式只进行单次传输。
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     //优先权为高
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //失能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;       //FIFO的阀值为半满
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;	            //内存突发传输为单一
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //外设突发传输为单一
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);                             //初始化DMA1_Stream3
	
	//   SPI2 TX DMA 配置   Stream4
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;         //指定DMA的外设基地址为SPI2的数据地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&SPI2_Tx_Byte;         //指定DMA的内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //DMA传输方向为读内存，写外设
	DMA_InitStructure.DMA_BufferSize = num ;//DataSize;                            //传输数量(0-65535，不能为0)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //失能外设地址增长
	DMA_InitStructure.DMA_MemoryInc = DMA_PeripheralInc_Disable;                 //失能内存地址增长
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//PSIZE=16bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;	//MSIZE=16bit	   
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//       //DMA模式为非循环模式,非循环模式只进行单次传输。
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium ;                  //优先权为中等
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //失能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;       //FIFO的阀值为半满
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;	            //内存突发传输为单一
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //外设突发传输为单一
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);                             //初始化DMA1_Stream4

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
{//最快8MHz
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	
	//配置PA1，作为TIM5_Ch1 PWM输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;//指定第一引脚//GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_12 |GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//模式必须为复用！
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//频率为快速
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//GPIO_PuPd_NOPULL;//上拉与否对PWM产生无影响
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	
	//TIM_DeInit(TIM5);//初始化TIM5寄存器
	//时基初始化
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //死区控制用。
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
	TIM_TimeBaseStructure.TIM_Prescaler = 6 - 1;   //因为如果APB1分频不为1,则定时时钟*2,时钟为168M/6*2=14MHz,
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;//0;
	TIM_TimeBaseStructure.TIM_Period = 35 - 1;  //Period = (Timer clock / TIM PWM clock) - 1 = 14M / 400K = 35
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	//配置输出比较，产生PWM方波 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1为正常占空比模式，PWM2为反极性模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//输出反相 TIM_OCNPolarity_Low;//输出同相，
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
	//TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);//CCR自动装载默认也是打开的
	//TIM_ARRPreloadConfig(TIM5,ENABLE);//ARR自动装载默认是打开的，可以不设置
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
			SPI2_Tx_Byte = 0xffdc;//写寄存器,单极性对地,从IN0扫描到IN7,全带宽,外部基准电压源，温度传感器禁用,不回读CFG
			SPI2_Tx_Cnt++;
		}
		else if(SPI2_Tx_Cnt == 4)
		{
			SPI2_Tx_Byte = 0x7fdc;//0x7ffc;
			SPI2_Tx_Cnt++;
		}
	}
}*/
