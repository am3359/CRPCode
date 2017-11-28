#include "DRV8825.h"
//#include "delay.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"

speedRampData srd;
u8 MotionStatus = 0;

u32 savetime;

u32 tim3_ms(void)
{
	return 0;//这里需要修改
}

static unsigned long sqrt(unsigned long x)
{
  register unsigned long xr;  // result register
  register unsigned long q2;  // scan-bit register
  register unsigned char f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
  }
}

void DRV8825_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	//GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	//配置PA3，作为TIM2_Ch4 PWM输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//模式必须为复用！
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;//频率为快速
	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//GPIO_PuPd_NOPULL;//上拉与否对PWM产生无影响
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

	//PA0手动输出 DIR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	//PA4手动输出 SLP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	//PB7输入信号
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

	//PF11手动输出 EN
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//时基初始化
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //死区控制用。
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseStructure.TIM_Period = 1000;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	//TIM_PrescalerConfig(TIM2, 84-1, TIM_PSCReloadMode_Immediate);
	
	//配置输出比较，产生PWM方波 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//TIM_OCMode_PWM1;//PWM1为正常占空比模式，PWM2为反极性模式//TIM_OCMode_Timing;//
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	//TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;//
	//TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;//输出反相 TIM_OCNPolarity_Low;//输出同相，
	//TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = 10;//10;//ccr4;
	TIM_OC4Init(TIM2,&TIM_OCInitStructure);

	TIM2->CCER &= ~(1<<12); //禁止输出
	TIM_Cmd(TIM2, DISABLE); SLP=0;
	//TIM_Cmd(TIM2,ENABLE);
	//TIM_CtrlPWMOutputs(TIM2,ENABLE);

	//TIM2中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//嵌套中断通道为TIM2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能中断
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void AxisMove(s32 step, u32 accel, u32 decel, u32 speed)
{
	//达到最大速度之前的步数。
	u32 max_s_lim;
	//必须启动减速时的步数（如果不能达到最大速度）。
	u32 accel_lim;

	if(step < 0)
	{//反转
		srd.dir = CCW;
		DIR=1;
		step =-step;
	}
	else
	{//正转
		srd.dir = CW;
		DIR=0;
	}

	//如果只移动一步
	if(step == 1)
	{
		// 只移动一步
		srd.accel_count = -1;
		// ...in DECEL state.
		srd.run_state = DECEL;
		// Just a short delay so main() can act on 'running'.
		srd.step_delay = 1000;//??
		
		MotionStatus = 1;
		
		TIM2->CCR4=10;//??????
		TIM2->ARR=20;//??
    
		//TIM2->CCER |= 1<<12; //使能输出 ??多了一个脉冲
		TIM_Cmd(TIM2, ENABLE); SLP=1;
		savetime=tim3_ms();
	}
	// 只有在步数不为0时才移动
	else if(step != 0)
	{
		// Refer to documentation for detailed information about these calculations.
		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha / tt)/ w
		srd.min_delay = A_T_x100 / speed;//T1_FREQ/speed/2;

		// Set accelration by calc the first (c0) step delay .
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
		srd.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100;//srd.step_delay = ((long)T1_FREQ*0.676* sqrt(2000000 / accel))/1000/2;
		// Find out after how many steps does the speed hit the max speed limit.
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);//max_s_lim = speed*speed/(2*accel);
		// If we hit max speed limit before 0,5 step it will round to 0.
		// But in practice we need to move atleast 1 step to get any speed at all.
		if(max_s_lim == 0)
		{
			max_s_lim = 1;
		}

		// Find out after how many steps we must start deceleration.
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim = ((long)step*decel) / (accel+decel);
		// We must accelrate at least 1 step before we can start deceleration.
		if(accel_lim == 0)
		{
			accel_lim = 1;
		}

		// Use the limit we hit first to calc decel.
		if(accel_lim <= max_s_lim)
		{
			srd.decel_val = accel_lim - step;
		}
		else
		{
			srd.decel_val =-(long)(max_s_lim*accel/decel);
		}
		// We must decelrate at least 1 step to stop.
		if(srd.decel_val == 0)
		{
			srd.decel_val = -1;
		}

		// Find step to start decleration.
		srd.decel_start = step + srd.decel_val;

		// If the maximum speed is so low that we dont need to go via accelration state.
		if(srd.step_delay <= srd.min_delay)
		{
			srd.step_delay = srd.min_delay;
			srd.run_state = RUN;
		}
		else
		{
			srd.run_state = ACCEL;
		}

		// Reset counter.
		srd.accel_count = 0;
		MotionStatus = 1;
		//OCR1A = 10;
		TIM2->CCR4=10;//??
		TIM2->ARR=20;
	
		TIM_Cmd(TIM2, ENABLE); SLP=1;
		savetime=tim3_ms();
	}
}

u16 Int2Str(u32 x, char * str)//每个字符后面添加了";\r\n"
{
	u16 ret,i=1;
	u32 tmp=10;
	while(x/tmp!=0)
	{
		i++;
		tmp*=10;
	}
	str[i]=';';
	str[i+1]='\r';
	str[i+2]='\n';
	str[i+3]='\0';
	ret=i+3;
	tmp=x;
	while(i>1)
	{
		str[--i]='0'+(tmp%10);
		tmp/=10;
	}
	str[0]=tmp+'0';
	return ret;
}

void AxisMove1(s32 step, u32 accel, u32 decel, u32 speed)
{
	//达到最大速度之前的步数。
	u32 max_s_lim;
	//必须启动减速时的步数（如果不能达到最大速度）。
	u32 accel_lim;

	// Holds next delay period.
	u16 new_step_delay;
	// Remember the last step delay used when accelrating.
	u16 last_accel_delay;
	// Counting steps when moving.
	u32 step_count = 0;
	// Keep track of remainder from new_step-delay calculation to incrase accurancy
	s32 rest = 0;
	//static u8 i=0;
	u32 len;
	char ss[16];
	
	if(step < 0)
	{//反转
		srd.dir = CCW;
		step =-step;
	}
	else
	{//正转
		srd.dir = CW;
	}

	//如果只移动一步
	if(step == 1)
	{
		// 只移动一步
		srd.accel_count = -1;
		// ...in DECEL state.
		srd.run_state = DECEL;
		// Just a short delay so main() can act on 'running'.
		srd.step_delay = 1000;//??
		
		MotionStatus = 1;
		
	}
	// 只有在步数不为0时才移动
	else if(step != 0)
	{
		// Refer to documentation for detailed information about these calculations.
		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha / tt)/ w
		srd.min_delay = A_T_x100 / speed;//T1_FREQ/speed/2;

		// Set accelration by calc the first (c0) step delay .
		// step_delay = 1/tt * sqrt(2*alpha/accel)
		// step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
		srd.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100;//srd.step_delay = ((long)T1_FREQ*0.676* sqrt(2000000 / accel))/1000/2;
		// Find out after how many steps does the speed hit the max speed limit.
		// max_s_lim = speed^2 / (2*alpha*accel)
		max_s_lim = (long)speed*speed/(long)(((long)A_x20000*accel)/100);//max_s_lim = speed*speed/(2*accel);
		// If we hit max speed limit before 0,5 step it will round to 0.
		// But in practice we need to move atleast 1 step to get any speed at all.
		if(max_s_lim == 0)
		{
			max_s_lim = 1;
		}

		// Find out after how many steps we must start deceleration.
		// n1 = (n1+n2)decel / (accel + decel)
		accel_lim = ((u32)step*decel) / (accel+decel);
		// We must accelrate at least 1 step before we can start deceleration.
		if(accel_lim == 0)
		{
			accel_lim = 1;
		}

		// Use the limit we hit first to calc decel.
		if(accel_lim <= max_s_lim)
		{
			srd.decel_val = accel_lim - step;
		}
		else
		{
			srd.decel_val =-(s32)(max_s_lim*accel/decel);
		}
		// We must decelrate at least 1 step to stop.
		if(srd.decel_val == 0)
		{
			srd.decel_val = -1;
		}

		// Find step to start decleration.
		srd.decel_start = step + srd.decel_val;

		// If the maximum speed is so low that we dont need to go via accelration state.
		if(srd.step_delay <= srd.min_delay)
		{
			srd.step_delay = srd.min_delay;
			srd.run_state = RUN;
		}
		else
		{
			srd.run_state = ACCEL;
		}

		// Reset counter.
		srd.accel_count = 0;
		
		MotionStatus = 1;
	}
	

	while(MotionStatus)
	{
		delay_ms(10);
		len=Int2Str(srd.step_delay, ss);

		switch(srd.run_state) 
		{
		case STOP:
			step_count = 0;
			rest = 0;
	
			MotionStatus = 0;
			break;
		case ACCEL:
			Uart3_PutString(ss , len);
			step_count++;
			srd.accel_count++;
			new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
			if(step_count >= srd.decel_start)
			{
				srd.accel_count = srd.decel_val;
				srd.run_state = DECEL;
			}
			else if(new_step_delay <= srd.min_delay)
			{
				last_accel_delay = new_step_delay;
				new_step_delay = srd.min_delay;
				rest = 0;
				srd.run_state = RUN;
			}
			break;
		case RUN:
			Uart3_PutString(ss , len);
			step_count++;
			new_step_delay = srd.min_delay;
			if(step_count >= srd.decel_start )
			{
				srd.accel_count = srd.decel_val;
				// Start decelration with same delay as accel ended with.
				new_step_delay = last_accel_delay;
				srd.run_state = DECEL;
			}
			break;
		case DECEL:
			Uart3_PutString(ss , len);
			step_count++;
			srd.accel_count++;
			new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
			if(srd.accel_count >= 0)
			{
				srd.run_state = STOP;
			}
			break;
		}
		srd.step_delay = new_step_delay;
	}

}

void TIM2_IRQHandler(void)
{ 
	// Holds next delay period.
	u32 new_step_delay;
	// Remember the last step delay used when accelrating.
	static u32 last_accel_delay;
	// Counting steps when moving.
	static u32 step_count = 0;
	// Keep track of remainder from new_step-delay calculation to incrase accurancy
	static s32 rest = 0;
	//static u8 i=0;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		/* Clear TIM2 Capture Compare1 interrupt pending bit*/
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		TIM2->CCR4=srd.step_delay >> 1;//!!srd.step_delay 必须大于10
		TIM2->ARR=srd.step_delay;
		// OCR1A = srd.step_delay;	//		   

		switch(srd.run_state) 
		{
		case STOP:
			step_count = 0;
			rest = 0;
			TIM2->CCER &= ~(1<<12); //禁止输出
			TIM_Cmd(TIM2, DISABLE);	SLP=0;		
		savetime=tim3_ms()-savetime;
			MotionStatus = 0;
			break;
		case ACCEL:
			TIM2->CCER |= 1<<12; //使能输出
			step_count++;
			srd.accel_count++;
			new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
			if(step_count >= srd.decel_start)
			{
				srd.accel_count = srd.decel_val;
				srd.run_state = DECEL;
			}
			else if(new_step_delay <= srd.min_delay)
			{
				last_accel_delay = new_step_delay;
				new_step_delay = srd.min_delay;
				rest = 0;
				srd.run_state = RUN;
			}
			break;
		case RUN:
			TIM2->CCER |= 1<<12; //使能输出
			step_count++;
			new_step_delay = srd.min_delay;
			if(step_count >= srd.decel_start )
			{
				srd.accel_count = srd.decel_val;
				// Start decelration with same delay as accel ended with.
				new_step_delay = last_accel_delay;
				srd.run_state = DECEL;
			}
			break;
		case DECEL:
			TIM2->CCER |= 1<<12; //使能输出
			step_count++;
			srd.accel_count++;
			new_step_delay = srd.step_delay - (((2 * (long)srd.step_delay) + rest)/(4 * srd.accel_count + 1));
			rest = ((2 * (long)srd.step_delay)+rest)%(4 * srd.accel_count + 1);
			if(srd.accel_count >= 0)
			{
				srd.run_state = STOP;
			}
			break;
		}
		srd.step_delay = new_step_delay;
	}
}
