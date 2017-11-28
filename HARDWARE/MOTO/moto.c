#include "moto.h"
#include "spi.h"

const u8 TxData[][8]={
	{0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	{0xEC,0x00,0x04,0x00,0xC5,0x00,0x00,0x00}, // CHOPCONF: TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle) +10mA
	{0x90,0x00,0x06,0x0C,0x0C,0x00,0x00,0x00}, // IHOLD_IRUN: IHOLD=12, IRUN=12 (max. current), IHOLDDELAY=6    +40mA
	{0x91,0x00,0x00,0x00,0x0A,0x00,0x00,0x00}, // TZEROWAIT=10: Delay before power down in stand still
	{0x80,0x00,0x00,0x00,0x04,0x00,0x00,0x00}, // EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)     +60mA
	{0x93,0x00,0x00,0x01,0xF4,0x00,0x00,0x00}, // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
	{0xF0,0x00,0x04,0x01,0xC8,0x00,0x00,0x00}, // PWM_CONF: AUTO=1, 1/1024 Fclk, Switch amplitude limit=200, Grad=1 -60mA
	{0xA4,0x00,0x00,0x03,0xE8,0x00,0x00,0x00}, // A1 = 1 000 First acceleration
	{0xA5,0x00,0x00,0xC3,0x50,0x00,0x00,0x00}, // V1 = 50 000 Acceleration threshold velocity V1
	{0xA6,0x00,0x00,0x27,0x10,0x00,0x00,0x00}, // AMAX = 10000 Acceleration above V1
	{0xA7,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // VMAX = 0///50000
	{0xA8,0x00,0x00,0x75,0x30,0x00,0x00,0x00}, // DMAX = 30000 Deceleration above V1
	{0xAA,0x00,0x00,0x05,0x78,0x00,0x00,0x00}, // D1 = 1400 Deceleration below V1
	{0xAB,0x00,0x00,0x00,0x0A,0x00,0x00,0x00}, // VSTOP = 10 Stop velocity (Near to zero)
	{0xA0,0x00,0x00,0x00,0x01,0x00,0x00,0x00}, // RAMPMODE = 1 (Target Positive Velocity move)
	{0xAD,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // XTARGET = 0
	{0xB4,0x00,0x00,0x00,0x0F,0x00,0x00,0x00}, // SW_MODE - Switch mode configuration
	{0x35,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
	};

void MOTO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	u32 i;
	u8 RxData[8];
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟


	TMC5130CS=1;

	//PB12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//PB12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	SPI2_Init();		   			//初始化SPI2
	//SPI2_SetSpeed(SPI_BaudRatePrescaler_16);		//设置为84M/16时钟
	
	//查看返回值为11表示识别正确
	MOTO_CmdWrRd((u8 *)TxData[0],RxData);//

	for(i=0;i<18;i++)
	{
		MOTO_CmdWrRd((u8 *)TxData[i],RxData);
	}
	
	//MOTO_Cmd(IHOLDIRUN|WRITE,0x00060C0C);
}

void MOTO_CmdWrRd(u8 *TxData,u8 *RxData)
{		 			 
	u32 i;
	TMC5130CS=0;
	for(i=0;i<5;i++)
	{
		RxData[i] = SPI2_ReadWriteByte(TxData[i]);
	}
	TMC5130CS=1;
}

u32 MOTO_Cmd(u8 command,u32 data)
{
	u8 TxD[8],RxD[8];
	TxD[0]= command;
	TxD[1]= *(((u8 *)&data)+3);
	TxD[2]= *(((u8 *)&data)+2);
	TxD[3]= *(((u8 *)&data)+1);
	TxD[4]= *(((u8 *)&data)+0);	
	MOTO_CmdWrRd(TxD,RxD);
	*(((u8 *)&data)+3)=RxD[1];
	*(((u8 *)&data)+2)=RxD[2];
	*(((u8 *)&data)+1)=RxD[3];
	*(((u8 *)&data)+0)=RxD[4];
	return data;
}
