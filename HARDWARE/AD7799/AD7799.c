#include "ad7799.h"
#include "spi.h"
#include "delay.h"

/*--------------------------------------------------------- 
 Func: AD7799写入数据 
 Time: 2016-2-29 
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/
void ADC_WriteBytes(u8 *Cmd,u8 Length)
{
	u8 i;
	for(i=0;i<Length;i++) SPI2_ReadWriteByte(*(Cmd+i));
}

/*--------------------------------------------------------- 
 Func: AD7799读取数据 
 Time: 2016-2-29 
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/
void ADC_ReadBytes(u8 *Buffer,u8 Length)
{
	u8 i;
	for(i=0;i<Length;i++) *(Buffer+i)=SPI2_ReadWriteByte(0x00);
}

void RDY_ON(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
 
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟
 
	//PB14设置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//复用功能
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//上拉  
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
}

void RDY_OFF(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
 
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟
 
	//PB14设置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//上拉  
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
}

/*--------------------------------------------------------- 
 Func: AD7799忙判断 
 Time: 2016-2-29 
 Ver.: V1.0 
 Note: 0/OK >0/ERROR,timeout 
---------------------------------------------------------*/  
u8 AD7799_WaitBusy(void)  
{  
    u32 i;  
	RDY_ON();
    AD7799CS=0;
    i=0;  
    while(ADC_RDY_DAT==1)
	{  
		i++;
		if(i>100) return 1; 		
		delay_ms(1);
    }  
    AD7799CS=1; 
	RDY_OFF();
    return 0;  
} 

/*--------------------------------------------------------- 
 Func: AD7799复位 
 Time: 2016-2-29 
 Ver.: V1.0 
 Note: DIN保持高连续发送32个CLK
---------------------------------------------------------*/  
void AD7799_Reset(void)  
{  
	u8 Cmd[4]={0xFF,0xFF,0xFF,0xFF};  
	AD7799CS=0;
	ADC_WriteBytes(Cmd,4);  
	AD7799CS=1;  
}

/*--------------------------------------------------------- 
 Func: AD7799读取寄存器数据 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/  
void AD7799_ReadReg(u8 RegAddr,u8 *Buffer,u8 Length)  
{  
	AD7799CS=0;
	RegAddr|=ADC_OP_READ;  
	ADC_WriteBytes(&RegAddr,1);  
	ADC_ReadBytes(Buffer,Length);  
	AD7799CS=1; 
}

/*--------------------------------------------------------- 
 Func: AD7799写入寄存器数据 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/  
void AD7799_WriteReg(u8 RegAddr,u8 *Buffer,u8 Length)  
{
	AD7799CS=0;
    RegAddr|=ADC_OP_WRITE;  
	ADC_WriteBytes(&RegAddr,1);  
	ADC_WriteBytes(Buffer,Length);  
	AD7799CS=1;  
}

/*--------------------------------------------------------- 
 Func: AD7799通道内部校准 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 0/OK >0/Error 
---------------------------------------------------------*/  
u8 AD7799_Calibrate(u8 CHx,u8 Gain)  
{  
    u8 R,Cmd[2];   
    Cmd[0]=UNIPOLAR|Gain;  
    Cmd[1]=CHx;  
    AD7799_WriteReg(ADC_REG_CONFIG,Cmd,2);  //设置配置寄存器  
    Cmd[0]=ADC_MODE_IZEROS;  
    Cmd[1]=0x0F;  
    AD7799_WriteReg(ADC_REG_MODE,Cmd,2);    //进行内部零度校准  
	delay_ms(1);
    R|=AD7799_WaitBusy();                   //等待校准完成  
    Cmd[0]=ADC_MODE_IFULLS;  
    Cmd[1]=0x0F;                          
    AD7799_WriteReg(ADC_REG_MODE,Cmd,2);    //进行内部零度校准  
	delay_ms(1);
    R|=AD7799_WaitBusy();                   //等待校准完成  
    return R;  
} 

/*--------------------------------------------------------- 
 Func: AD7799片选脚初始化 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/ 
void AD7799CS_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//PG2手动输出
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化
}

/*--------------------------------------------------------- 
 Func: AD7799初始化 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 0/OK >0/Error 
---------------------------------------------------------*/ 
u8 AD7799_Init(u8 Gain)
{
	u8 Cmd[2],ID;
	AD7799CS_Init();
	AD7799CS=1;                                     //片选信号置高
	SPI2_Init();                                    //SPI口初始化
	AD7799_Reset();                                 //复位
	delay_ms(10);                                   //延时
	AD7799_ReadReg(ADC_REG_ID,&ID,1);               //读取器件ID  
	//if((ID==0xFF)||(ID==0x00))return 1;  
	if(((ID & 0x0F)!=0x09)&&((ID & 0x0F)!=0x08))return 1;
	AD7799_Calibrate(ADC_CON_CH1,Gain);             //通道1校准  
	//AD7799_Calibrate(ADC_CON_CH2,Gain);             //通道2校准  
	Cmd[0]=0x70;//0x70;//0x40;                        //设置IO口
	AD7799_WriteReg(ADC_REG_IO,Cmd,1);
	return 0; 
}

/*--------------------------------------------------------- 
 Func: AD7799开始转换 
 Time: 2016-2-29 
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/  
void AD7799_Start(u8 CovChx,u8 CovGain,u8 CovMode,u8 CovRate)  
{  
    u8 Cmd[2];     
    Cmd[0]=UNIPOLAR|CovGain;  
    Cmd[1]=CovChx;  
    AD7799_WriteReg(ADC_REG_CONFIG,Cmd,2);   
    Cmd[0]=CovMode;  
    Cmd[1]=CovRate;  
    AD7799_WriteReg(ADC_REG_MODE,Cmd,2);  
}

/*--------------------------------------------------------- 
 Func: AD7799读取转换结果 
 Time: 2016-2-29 
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/  
u32 AD7799_Read(void)  
{  
    u8 Cmd[4];  
    u32 D=0;  
    Cmd[0]=0;  
    AD7799_ReadReg(ADC_REG_DATA,&Cmd[1],3);  
    *((u8 *)(&D)+2)=Cmd[1];  
	*((u8 *)(&D)+1)=Cmd[2];
	*((u8 *)(&D)+0)=Cmd[3];

    return D;  
} 
