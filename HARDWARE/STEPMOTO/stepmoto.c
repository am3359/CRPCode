#include "stepmoto.h"
#include "dma.h"
#include "timer.h"

const u16 SteppMotoBuf1[StepBufLen1]={
    0x7AB6,0x7AB6,0x7AB6,0x7AB6,
    0x7AB6,0x7AB5,0x7AB4,0x7AB3,
    0x7AB0,0x7AAC,0x7AA6,0x7A9B,
    0x7A8A,0x7A6D,0x7A3D,0x79EF,
    0x796F,0x789E,0x774C,0x752F,
    0x71DF,0x6CD1,0x6572,0x5B5C,
    0x4EB3,0x406C,0x3225,0x257C,
    0x1B66,0x1407,0x0EF9,0x0BA9,
    0x098C,0x083A,0x0769,0x06E9,
    0x069B,0x066B,0x064E,0x063D,
    0x0632,0x062C,0x0628,0x0625,
    0x0624,0x0623,0x0622,0x0622,};
u16 SteppAccelBuf[StepBufLen1];
u16 SteppDecelBuf[StepBufLen1];
s32 Stepper_Mid_Steps;
u16 Stepper_Run=20000;
s8 Stepper_Period=0;

void StepGPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    //配置PB6，作为TIM4_Ch1 PWM输出STEP
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_100MHz;//GPIO_Speed_2MHz;//
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
    //PD0->SLP PD1->EN PD5->DIR
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化    
}

void StepMotoInit(void)
{
    StepGPIO_Init();
	DMA1_Init();
	TIM4_PWM_Config();
}

/************************************************************************************
函数名称：步进电机1控制函数
输入参数：
输出参数：
功能描述：S曲线计算，根据输入参数更新加速度缓冲区，加速度和减速度相同，启动DMA PWM
************************************************************************************/
void StepMoto1Move(s32 steps)
{
    //s32 Start_Steps = 0;
    //u32 steps;
//-----------------------------------------------------------------------------------
	u16 i;														 								 //加速曲线表格初始化
	for(i = 0;i<StepBufLen1;i++)
		SteppAccelBuf[i] = SteppMotoBuf1[i];
	
	for(i = 0;i<StepBufLen1;i++)
		SteppDecelBuf[i] = SteppAccelBuf[StepBufLen1-1-i];
//-----------------------------------------------------------------------------------	
    if(steps < 0)
    {//反转
        DIR1=1;
        steps =-steps;
    }
    else
    {//正转
        DIR1=0;
        //steps =steps;
    }
    
    Stepper_Run = SteppAccelBuf[StepBufLen1-1];         //匀速时的运行频率
    Stepper_Mid_Steps = steps - StepBufLen1*2; //匀速阶段步数
    
    Stepper_Period=3;

    //DMA1_Stream0_CH2_Cmd(&TIM4_PWMDMA_Config,SteppAccelBuf,StepBufLen1,DMA_MemoryInc_Enable);
    TIM4_PWMDMA_Config(SteppAccelBuf,StepBufLen1,DMA_MemoryInc_Enable);
    DMA_Cmd(DMA1_Stream0,ENABLE);
    TIM_DMACmd(TIM4,TIM_DMA_CC1,ENABLE);
    TIM4->CCER |= 1<<0; //开TME4 PWM输出
    TIM_Cmd(TIM4,ENABLE);
}

void DMA1_Stream0_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_TCIF0)==SET)
    {
		DMA_ClearFlag(DMA1_Stream0,DMA_IT_TCIF0);
		
		if(--Stepper_Period>0)//根据当前调速阶段重新配置DMA
		{
			if(Stepper_Mid_Steps<0)
                {//不可用???
				Stepper_Mid_Steps = -Stepper_Mid_Steps;
				//DMA1_Stream0_CH2_Cmd(&TIM4_PWMDMA_Config,&SteppDecelBuf[StepBufLen1+Stepper_Mid_Steps],Stepper_Mid_Steps,DMA_MemoryInc_Enable);
                TIM4_PWMDMA_Config(&SteppDecelBuf[StepBufLen1-Stepper_Mid_Steps],Stepper_Mid_Steps,DMA_MemoryInc_Enable);
                DMA_Cmd(DMA1_Stream0,ENABLE);
				Stepper_Period = 1;
			}
			else
			{
				if(Stepper_Period == 2)
                {
					//DMA1_Stream0_CH2_Cmd(&TIM4_PWMDMA_Config,&Stepper_Run,Stepper_Mid_Steps,DMA_MemoryInc_Disable);
                    TIM4_PWMDMA_Config(&Stepper_Run,Stepper_Mid_Steps,DMA_MemoryInc_Disable);
                    DMA_Cmd(DMA1_Stream0,ENABLE);
                }
				if(Stepper_Period == 1)
                {
					//DMA1_Stream0_CH2_Cmd(&TIM4_PWMDMA_Config,SteppDecelBuf,StepBufLen1,DMA_MemoryInc_Enable);
                    TIM4_PWMDMA_Config(SteppDecelBuf,StepBufLen1,DMA_MemoryInc_Enable);
                    DMA_Cmd(DMA1_Stream0,ENABLE);
                }
			}
		}
		else
		{
			DMA_Cmd(DMA1_Stream0,DISABLE);
			TIM_DMACmd(TIM4,TIM_DMA_CC1,DISABLE);
			TIM4 -> CCER &= ~(1<<0); //关闭TME4 PWM输出
			//flag_DMA1_Stream0_CH2 = 0;
		}
    }
}
