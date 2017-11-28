#include "PWM.h"

void TIM2_PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	//GPIO_StructInit(&GPIO_InitStructure);
	
	//����GPIOA_Pin_3����ΪTIM_Channel2 PWM���*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //ָ����������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //ģʽ����Ϊ���ã�
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //Ƶ��Ϊ����
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //��������PWM������Ӱ��
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);//����GPIOA_Pin_3ΪTIM2_CH4
	
	//TIM_DeInit(TIM2);//��ʼ��TIM2�Ĵ���
	TIM_TimeBaseStructure.TIM_Period = 200-1; //�������ֲ��֪��TIM2��TIM5Ϊ32λ�Զ�װ��
	TIM_TimeBaseStructure.TIM_Prescaler = 420-1;//84-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	//TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;//0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/*��������Ƚϣ�����ռ�ձ�Ϊ20%��PWM����*/
	//TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//������� TIM_OCNPolarity_Low;//���ͬ�࣬
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	
	TIM_OCInitStructure.TIM_Pulse = 100;//����CCR��ռ�ձ���ֵ��50% TIM_Period
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	//TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);//CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	//TIM_ARRPreloadConfig(TIM2, ENABLE); //ARR�Զ�װ��Ĭ���Ǵ򿪵ģ����Բ�����
	//TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	//TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2, ENABLE); //ʹ��TIM2��ʱ��
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
}
