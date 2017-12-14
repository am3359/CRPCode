#include "HC595.h"

#include "delay.h"
/* 延时模块82615468 sp-320-12
*/
//static void delay(u32 t)
//{
//	u32 i;
//	while(t--)
//		for (i = 0; i < 1; i++);
//}


void HC595Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  // 开启GPIO时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;          //下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	HC595_CK = 1;
	HC595_DA = 1;
	HC595_CS = 1;
	HC595_OE = 0;
	
}
void HC595Send(u8 data)
{
    u8 j;
    for (j = 8; j > 0; j--)
    {
        if(data & 0x80)
            HC595_DA = 1;
        else
            HC595_DA = 0;
        HC595_CK = 0;              //上升沿发生移位
        //delay(1);
        data <<= 1;
        HC595_CK = 1;
		//delay(1);
  }
	//HC595Load();
}

void HC595Load(void)
{
    HC595_CS = 0;
    HC595_CS = 1;
}

void SwitchOut(u16 data)
{
    HC595Send(data >> 8);
    HC595Send(data >> 0);
    HC595Load();
}

//end of file
