#include "PWM.h"

void TIM2_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//GPIO_StructInit(&GPIO_InitStructure);
	
	//配置GPIOA_Pin_3，作为TIM_Channel2 PWM输出*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //指定第六引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //模式必须为复用！
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //频率为快速
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉与否对PWM产生无影响
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);//复用GPIOA_Pin_3为TIM2_CH4
	
	//TIM_DeInit(TIM2);//初始化TIM2寄存器
	TIM_TimeBaseStructure.TIM_Period = 200-1; //查数据手册可知，TIM2与TIM5为32位自动装载
	TIM_TimeBaseStructure.TIM_Prescaler = 420-1;//84-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;//0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/*配置输出比较，产生占空比为20%的PWM方波*/
	//TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1为正常占空比模式，PWM2为反极性模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//输出反相 TIM_OCNPolarity_Low;//输出同相，
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	TIM_OCInitStructure.TIM_Pulse = 100;//输入CCR（占空比数值）50% TIM_Period
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	//TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);//CCR自动装载默认也是打开的
	//TIM_ARRPreloadConfig(TIM2, ENABLE); //ARR自动装载默认是打开的，可以不设置
	//TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	//TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE); //使能TIM2定时器
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
}
