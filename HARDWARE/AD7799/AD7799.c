#include "ad7799.h"
#include "spi.h"
#include "delay.h"

/*--------------------------------------------------------- 
 Func: AD7799д������ 
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
 Func: AD7799��ȡ���� 
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
 
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//ʹ��GPIOBʱ��
 
	//PB14����
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//���ù���
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//����  
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
}

void RDY_OFF(void)
{	 
	GPIO_InitTypeDef  GPIO_InitStructure;
 
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//ʹ��GPIOBʱ��
 
	//PB14����
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//����  
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
}

/*--------------------------------------------------------- 
 Func: AD7799æ�ж� 
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
 Func: AD7799��λ 
 Time: 2016-2-29 
 Ver.: V1.0 
 Note: DIN���ָ���������32��CLK
---------------------------------------------------------*/  
void AD7799_Reset(void)  
{  
	u8 Cmd[4]={0xFF,0xFF,0xFF,0xFF};  
	AD7799CS=0;
	ADC_WriteBytes(Cmd,4);  
	AD7799CS=1;  
}

/*--------------------------------------------------------- 
 Func: AD7799��ȡ�Ĵ������� 
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
 Func: AD7799д��Ĵ������� 
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
 Func: AD7799ͨ���ڲ�У׼ 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 0/OK >0/Error 
---------------------------------------------------------*/  
u8 AD7799_Calibrate(u8 CHx,u8 Gain)  
{  
    u8 R,Cmd[2];   
    Cmd[0]=UNIPOLAR|Gain;  
    Cmd[1]=CHx;  
    AD7799_WriteReg(ADC_REG_CONFIG,Cmd,2);  //�������üĴ���  
    Cmd[0]=ADC_MODE_IZEROS;  
    Cmd[1]=0x0F;  
    AD7799_WriteReg(ADC_REG_MODE,Cmd,2);    //�����ڲ����У׼  
	delay_ms(1);
    R|=AD7799_WaitBusy();                   //�ȴ�У׼���  
    Cmd[0]=ADC_MODE_IFULLS;  
    Cmd[1]=0x0F;                          
    AD7799_WriteReg(ADC_REG_MODE,Cmd,2);    //�����ڲ����У׼  
	delay_ms(1);
    R|=AD7799_WaitBusy();                   //�ȴ�У׼���  
    return R;  
} 

/*--------------------------------------------------------- 
 Func: AD7799Ƭѡ�ų�ʼ�� 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 
---------------------------------------------------------*/ 
void AD7799CS_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//PG2�ֶ����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��
}

/*--------------------------------------------------------- 
 Func: AD7799��ʼ�� 
 Time: 2016-2-29
 Ver.: V1.0 
 Note: 0/OK >0/Error 
---------------------------------------------------------*/ 
u8 AD7799_Init(u8 Gain)
{
	u8 Cmd[2],ID;
	AD7799CS_Init();
	AD7799CS=1;                                     //Ƭѡ�ź��ø�
	SPI2_Init();                                    //SPI�ڳ�ʼ��
	AD7799_Reset();                                 //��λ
	delay_ms(10);                                   //��ʱ
	AD7799_ReadReg(ADC_REG_ID,&ID,1);               //��ȡ����ID  
	//if((ID==0xFF)||(ID==0x00))return 1;  
	if(((ID & 0x0F)!=0x09)&&((ID & 0x0F)!=0x08))return 1;
	AD7799_Calibrate(ADC_CON_CH1,Gain);             //ͨ��1У׼  
	//AD7799_Calibrate(ADC_CON_CH2,Gain);             //ͨ��2У׼  
	Cmd[0]=0x70;//0x70;//0x40;                        //����IO��
	AD7799_WriteReg(ADC_REG_IO,Cmd,1);
	return 0; 
}

/*--------------------------------------------------------- 
 Func: AD7799��ʼת�� 
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
 Func: AD7799��ȡת����� 
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
