#include "stepmoto.h"
#include "dma.h"
#include "timer.h"
#include "delay.h"
#include "math.h"
#include "usart.h"

u16 SteppAccelBuf[2][StepBufLen];
u16 SteppDecelBuf[2][StepBufLen];
u16 SteppRun[2];
//u16 Stepper_Run=2000;//c1

TStepMotor StepMotor[5]={
    {//横向运动
        GPIOD,GPIO_Pin_0,GPIOD,GPIO_Pin_1,GPIOD,GPIO_Pin_5,GPIOE,GPIO_Pin_7,
        TIM4,TIM_DMA_CC1,(1<<0),
        RCC_AHB1Periph_DMA1,DMA1_Stream0,DMA_Channel_2,
        0,0,0,0,3,1,Bit_RESET,
        {SteppAccelBuf[0],&SteppRun[0],SteppDecelBuf[0]},
        {DMA_MemoryInc_Enable,DMA_MemoryInc_Disable,DMA_MemoryInc_Enable},
        {StepBufLen,StepBufLen,StepBufLen},
        {
            {2000,1600,50},//L慢 [B200016000050] (S0141600;S0101600) acc pulse:28 acc time:50600 us
            {2000,1200,50},//M中 [B200012000050] (S0141600;S0101600) acc pulse:30 acc time:48400 us
            {2000, 800,50} //H高 [B200008000050] (S0141600;S0101600) acc pulse:32 acc time:45400 us
        }
    },
    {//上下运动
        GPIOD,GPIO_Pin_0,GPIOD,GPIO_Pin_1,GPIOD,GPIO_Pin_4,GPIOE,GPIO_Pin_8,
        TIM4,TIM_DMA_CC2,(1<<4),
        RCC_AHB1Periph_DMA1,DMA1_Stream3,DMA_Channel_2,
        0,0,0,0,3,1,Bit_SET,
        {SteppAccelBuf[0],&SteppRun[0],SteppDecelBuf[0]},
        {DMA_MemoryInc_Enable,DMA_MemoryInc_Disable,DMA_MemoryInc_Enable},
        {StepBufLen,StepBufLen,StepBufLen},
        {
            {1200,1000,50},//L慢 [B120010000050] acc pulse:24 acc time:26500 us
            {1100, 750,40},//M中 [B110007500040] acc pulse:34 acc time:31625 us
            {1000, 500,30} //H高 [B100005000030] acc pulse:48 acc time:36250 us
        }
    },
    {//注射器1
        GPIOD,GPIO_Pin_0,GPIOD,GPIO_Pin_1,GPIOD,GPIO_Pin_6,GPIOE,GPIO_Pin_9,
        TIM3,TIM_DMA_CC1,(1<<0),
        RCC_AHB1Periph_DMA1,DMA1_Stream4,DMA_Channel_5,
        0,0,0,0,3,1,Bit_SET,
        {SteppAccelBuf[1],&SteppRun[1],SteppDecelBuf[1]},
        {DMA_MemoryInc_Enable,DMA_MemoryInc_Disable,DMA_MemoryInc_Enable},
        {StepBufLen,StepBufLen,StepBufLen},
        {
            {1600,1500,50},//L慢 [B160015000050] acc pulse:22 acc time:34150 us
            {1600,1000,50},//M中 [B160010000050] acc pulse:30 acc time:39300 us
            {1600, 500,50} //H高 [B160005000050] acc pulse:32 acc time:34150 us
        }
    },
    {//注射器2
        GPIOD,GPIO_Pin_0,GPIOD,GPIO_Pin_1,GPIOD,GPIO_Pin_2,GPIOE,GPIO_Pin_10,
        TIM3,TIM_DMA_CC2,(1<<4),
        RCC_AHB1Periph_DMA1,DMA1_Stream5,DMA_Channel_5,
        0,0,0,0,3,1,Bit_SET,
        {SteppAccelBuf[1],&SteppRun[1],SteppDecelBuf[1]},
        {DMA_MemoryInc_Enable,DMA_MemoryInc_Disable,DMA_MemoryInc_Enable},
        {StepBufLen,StepBufLen,StepBufLen},
        {
            {1600,1500,50},//L慢 [B160015000050] acc pulse:22 acc time:34150 us
            {1600,1000,50},//M中 [B160010000050] acc pulse:30 acc time:39300 us
            {1600, 500,50} //H高 [B160005000050] acc pulse:32 acc time:34150 us
        }
    },
    {//蠕动泵1
        GPIOD,GPIO_Pin_0,GPIOD,GPIO_Pin_1,GPIOD,GPIO_Pin_3,GPIOE,GPIO_Pin_7,
        TIM4,TIM_DMA_CC3,(1<<8),
        RCC_AHB1Periph_DMA1,DMA1_Stream7,DMA_Channel_2,
        0,0,0,0,3,0,Bit_SET,
        {SteppAccelBuf[0],&SteppRun[0],SteppDecelBuf[0]},
        {DMA_MemoryInc_Enable,DMA_MemoryInc_Disable,DMA_MemoryInc_Enable},
        {StepBufLen,StepBufLen,StepBufLen},
        {
            {3500,3000,50},//L慢 [B350030000050] acc pulse:28 acc time:91250 us
            {3500,2000,50},//M中 [B350020000050] acc pulse:34 acc time:94250 us
            {3500,1000,50} //H高 [B350010000050] acc pulse:36 acc time:82250 us
        }
    },
};


//u16 SteppAccelBuf[StepBufLen];
//u16 SteppDecelBuf[StepBufLen];
//u32 Stepper_Mid_Steps;//匀速步数

//s8 Stepper_Period;//运行阶段
//u8 Stepper_Mode;//运行模式
//u8 Stepper_Change;//加减速步数  Stepper_Change+Stepper_Change+Stepper_Mid_Steps=总步数
//u8 Stepper_Change1;//保存 Stepper_Change+Stepper_Change>Steps时的值

void StepGPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    //配置PB6，作为TIM4_Ch1 PWM输出STEP
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
    //配置PB7，作为TIM4_Ch2 PWM输出STEP
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
    //配置PB8，作为TIM4_Ch3 PWM输出STEP
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
    
    //配置PC6，作为TIM3_Ch1 PWM输出STEP
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
    //配置PC7，作为TIM3_Ch2 PWM输出STEP
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM3);

    //步进电机脉冲：PB6横向运动，PB7上下运动，PB8蠕动泵旋转
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;// | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_100MHz;//GPIO_Speed_2MHz;//
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
    //步进电机脉冲：PC6注射器1(步进电机3)，PC7注射器2(步进电机4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;// | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_100MHz;//GPIO_Speed_2MHz;//
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    
    //PD0->SLP,PD1->EN,PD4->横向运动DIR,PD5->上下运动DIR,PD6->注射器1DIR,PD2->注射器2DIR PD3->蠕动泵DIR 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化    
}

void StepMotoInit(void)
{
    StepGPIO_Init();
    SLP1=1;//SLP1=0;
//    EN1=0;//EN1=1;
//    //DIR1=0;//DIR1=1;
//    //STP1=1;//STP1=0;
//    delay_us(2000);
//    SLP1=0;//SLP1=1;
//    delay_us(2000);
    EN1=1;//EN1=0;
    //while(1);
    delay_us(10000);
//    SLP1=1;//SLP1=0;
	DMA1_Init();
	TIM4_PWM_Config();
    TIM3_PWM_Config();
}

/************************************************************************************
函数名称：步进电机加减速S曲线计算函数，保证加速或减速时间在100ms~1000ms
输入参数：初始计数值c0,目标速度计数值c1,a斜率输入参数扩大了100倍，需要除以100,计数缓冲区pBuf[]
输出参数：返回加减速步数
功能描述：S曲线计算，根据输入参数更新加速度缓冲区，加速度和减速度相同，启动DMA PWM
************************************************************************************/
u16 tmpS[StepBufLen];//计算S曲线用
//#define PRT 1
u32 StepMotoCal(u16 c0,u16 c1,u16 a)
{//??没有考虑加减速步数相加超过总步数
    //u16 tmpS[StepBufLen];//计算S曲线用
    u8 i,j;
    u32 sum=0;

    for(i=0;i<StepBufLen;i++)
        tmpS[i]=(u16)(((double)c1-(double)c0)/(1.0+exp(-(double)a*((double)i-StepBufLen/2)/100.0))+(double)c0+0.5);

    for(i=0;i<StepBufLen/2-1;i++){
        j=i;
        if(tmpS[i]>tmpS[i+1]) break;
    }

    for(i=0;i<StepBufLen-j*2;i++){//减速是加速的反转
        sum=sum+tmpS[StepBufLen-j-1-i];
        printf("A%d\t%d\r\n",i+1,tmpS[i+j]);delay_us(1000);
    }
    for(i=0;i<StepBufLen-j*2;i++){//减速是加速的反转
        printf("D%d\t%d\r\n",i+1,tmpS[StepBufLen-j-1-i]);delay_us(1000);
    }

    printf("acc pulse:%d acc time:%d us\r\n",StepBufLen-j*2,sum);

    return sum;
}

//no步进电机编号0~4，level速度模式
//void StepMotoCal1(u8 no,u8 level)
//{//??没有考虑加减速步数相加超过总步数
//    //u16 tmpS[StepBufLen];//计算S曲线用
//    u16 i,j;
//    u16 c0,c1,a;
//    StepMotor[no].level=level;
//    c0=StepMotor[no].SetStepMotor[level].c0;
//    c1=StepMotor[no].SetStepMotor[level].c1;
//    a=StepMotor[no].SetStepMotor[level].a;

//    for(i=0;i<StepBufLen;i++)
//        tmpS[i]=(u16)(((double)c1-(double)c0)/(1.0+exp(-(double)a*((double)i-StepBufLen/2)/100.0))+(double)c0+0.5);
//    
//    for(i=0;i<StepBufLen/2-1;i++){
//        j=i;
//        if(tmpS[i]>tmpS[i+1]) break;
//    }

//    for(i=0;i<StepBufLen-j*2;i++){//加减速数据保存
//        StepMotor[no].AccelBuf[i]=tmpS[j+i];
//        StepMotor[no].DecelBuf[i]=tmpS[StepBufLen-j-1-i];
//    }

//    StepMotor[no].ADSize=StepBufLen-j*2;
//}

void PWM_OFF(u8 no)
{
    DMA_Cmd(StepMotor[no].DMA_Stream,DISABLE);
    TIM_DMACmd(StepMotor[no].timer,StepMotor[no].TIM_DMASource,DISABLE);
    (StepMotor[no].timer) -> CCER &= ~StepMotor[no].CCER; //关闭TIM PWM输出
    TIM_Cmd((StepMotor[no].timer),DISABLE);
}

u8 IsLmt(u8 no)
{
    u8 res=0;

    if(GPIO_ReadInputDataBit(StepMotor[no].Lmt_GPIO, StepMotor[no].Lmt_GPIO_Pin)==Bit_RESET) {//限位信号为低代表在原点
        StepMotor[no].dist = 0;
        StepMotor[no].pot = 0;
        res=1;
    } else {
        res=0;
    }
                
    return res;
}

/************************************************************************************
函数名称：步进电机1控制函数
输入参数：no步进电机编号，mode步进电机运行模式，steps步数,偶数步，目标速度计数值c1
         模式0：加速，匀速，减速；
         模式1：加速，匀速，忽略步数；
         模式2：匀速，忽略加减速和步数；
         模式3：匀速，减速；
         level 默认速度 0：慢，1：中，2：快
输出参数：
功能描述：加速度和减速度相同，启动DMA PWM
************************************************************************************/
u32 StepMotoMove(u8 no,u8 mode,s32 steps,u8 level)//??载入S曲线时可以修改占空比，使高脉冲是最快速度时周期的一半
{
    u32 sum=0;
    
    u16 i,j;
    u16 c0,c1,a;
    
    if(steps < 0)
    {//反转
        //DIR1=1;
        GPIO_SetBits(StepMotor[no].Dir_GPIO, StepMotor[no].Dir_GPIO_Pin);//GPIO_ResetBits
        steps =-steps;
    }
    else
    {//正转
        //DIR1=0;
        GPIO_ResetBits(StepMotor[no].Dir_GPIO, StepMotor[no].Dir_GPIO_Pin);//GPIO_SetBits
        //steps =steps;
    }

//如果向限位运动并且已经在限位
if(StepMotor[no].LmtEn) {//有限位要求
    if(GPIO_ReadOutputDataBit (StepMotor[no].Dir_GPIO,StepMotor[no].Dir_GPIO_Pin)==StepMotor[no].LmtBit) {//方向确认
        if(IsLmt(no)) return 0;
    }
}
   
    //先计算加减速参数
    StepMotor[no].mode = mode;

    StepMotor[no].level=level;
    c0=StepMotor[no].SetStepMotor[level].c0;
    c1=StepMotor[no].SetStepMotor[level].c1;
    a=StepMotor[no].SetStepMotor[level].a;

    for(i=0;i<StepBufLen;i++)
        tmpS[i]=(u16)(((double)c1-(double)c0)/(1.0+exp(-(double)a*((double)i-StepBufLen/2)/100.0))+(double)c0+0.5);
    
    for(i=0;i<StepBufLen/2-1;i++){
        j=i;
        if(tmpS[i]>tmpS[i+1]) break;
    }

    for(i=0;i<StepBufLen-j*2;i++){//加减速数据保存
        sum=sum+tmpS[j+i];
        *(StepMotor[no].Buf[0]+i)=tmpS[j+i];
        *(StepMotor[no].Buf[2]+i)=tmpS[StepBufLen-j-1-i];
    }
    *(StepMotor[no].Buf[1]+0)=c1;//匀速数据

    StepMotor[no].Size[0]=StepBufLen-j*2;
    StepMotor[no].Size[2]=StepBufLen-j*2;
//printf("acc pulse:%d acc time:%d us\r\n",StepBufLen-j*2,sum);
//    for(i=0;i<StepBufLen-j*2;i++){//减速是加速的反转
//        printf("A%d\t%d\r\n",i+1,*(StepMotor[no].Buf[0]+i));delay_us(1000);
//    }
//    for(i=0;i<StepBufLen-j*2;i++){//减速是加速的反转
//        printf("D%d\t%d\r\n",i+1,*(StepMotor[no].Buf[2]+i));delay_us(1000);
//    }
    
//SLP1=0;//SLP1=1;
GPIO_ResetBits(StepMotor[no].Slp_GPIO, StepMotor[no].Slp_GPIO_Pin);//GPIO_SetBits//退出休眠
    switch(mode)
    {
        case 0://模式0：加速，匀速，减速；
            if(steps > (StepMotor[no].Size[0]+StepMotor[no].Size[2])) {
                StepMotor[no].Size[1] = steps - (StepMotor[no].Size[0]+StepMotor[no].Size[2]); //匀速阶段步数            ??还需要考虑短行程，速度模式等情况??
            } else {
                StepMotor[no].Size[0] = steps/2;//??加减速有长短是有问题
                StepMotor[no].Size[1] = 0;
                StepMotor[no].Size[2] = steps/2;
            }
            StepMotor[no].stage=0;//加速  0：加速，1：匀速，2：减速，3或其它：停止
            TIM_PWM_DMA_Config(no);
            break;
        case 1://模式1：加速，匀速，忽略步数；
            StepMotor[no].Size[1] = 200;
            StepMotor[no].stage=0;//加速  0：加速，1：匀速，2：减速，3或其它：停止
            TIM_PWM_DMA_Config(no);
            break;
        case 2://模式2：匀速，忽略加减速和步数；
            StepMotor[no].Size[1] = 200;
            StepMotor[no].stage=1;//匀速  0：加速，1：匀速，2：减速，3或其它：停止
            TIM_PWM_DMA_Config(no);
            break;
        case 3://模式3：匀速，减速；
            if(steps > StepMotor[no].Size[2]) {
                StepMotor[no].Size[1] = steps - StepMotor[no].Size[2]; //匀速阶段步数            ??还需要考虑短行程，速度模式等情况??
                StepMotor[no].stage=1;//匀速  0：加速，1：匀速，2：减速，3或其它：停止
            } else {
                StepMotor[no].Size[1] = 0;
                StepMotor[no].Size[2] = steps;
                StepMotor[no].stage=2;//减速  0：加速，1：匀速，2：减速，3或其它：停止
            }
            TIM_PWM_DMA_Config(no);
            break;
    }
    return sum;
}

void StepMotorHandler(u8 no)
{
    switch(StepMotor[0].mode)
    {
        case 0://模式0：加速，匀速，减速；
            if(StepMotor[no].stage==0) {
                if(StepMotor[no].Size[1]>0) {
                    StepMotor[no].stage=1;//匀速  0：加速，1：匀速，2：减速，3或其它：停止
//printf("匀速\r\n");
                } else {
                    StepMotor[no].stage=2;//减速  0：加速，1：匀速，2：减速，3或其它：停止
//printf("减速1\r\n");
                }
                TIM_PWM_DMA_Config(no);
            } else if(StepMotor[no].stage==1) {
//printf("减速2\r\n");
                StepMotor[no].stage=2;//减速  0：加速，1：匀速，2：减速，3或其它：停止
                TIM_PWM_DMA_Config(no);
            } else {
//printf("结束\r\n");
                StepMotor[no].stage=3;
                PWM_OFF(no);
//SLP1=1;//SLP1=0;
//所有电机不工作时才休眠
if((StepMotor[0].stage>2) && (StepMotor[1].stage>2) && (StepMotor[2].stage>2)
     && (StepMotor[3].stage>2) && (StepMotor[4].stage>2))
    GPIO_SetBits(StepMotor[no].Slp_GPIO, StepMotor[no].Slp_GPIO_Pin);//GPIO_ResetBits//休眠
            }
            break;
        case 1://模式1：加速，匀速，忽略步数；
        case 2://模式2：匀速，忽略加减速和步数；
            if(StepMotor[no].stage==0) {
                StepMotor[no].stage=1;//匀速  0：加速，1：匀速，2：减速，3或其它：停止
                TIM_PWM_DMA_Config(no);
            } else if(StepMotor[no].stage==1) {
                //StepMotor[no].stage=1;//减速  0：加速，1：匀速，2：减速，3或其它：停止
                TIM_PWM_DMA_Config(no);
            } else {
                StepMotor[no].stage=3;
                PWM_OFF(no);
//SLP1=1;//SLP1=0;
//所有电机不工作时才休眠
if((StepMotor[0].stage>2) && (StepMotor[1].stage>2) && (StepMotor[2].stage>2)
     && (StepMotor[3].stage>2) && (StepMotor[4].stage>2))
    GPIO_SetBits(StepMotor[no].Slp_GPIO, StepMotor[no].Slp_GPIO_Pin);//GPIO_ResetBits//休眠
            }
            break;
        case 3://模式3：匀速，减速；
            if(StepMotor[no].stage==1) {
                StepMotor[no].stage=2;//减速  0：加速，1：匀速，2：减速，3或其它：停止
                TIM_PWM_DMA_Config(no);
            } else {
                StepMotor[no].stage=3;
                PWM_OFF(no);
//SLP1=1;//SLP1=0;
//所有电机不工作时才休眠
if((StepMotor[0].stage>2) && (StepMotor[1].stage>2) && (StepMotor[2].stage>2)
     && (StepMotor[3].stage>2) && (StepMotor[4].stage>2))
    GPIO_SetBits(StepMotor[no].Slp_GPIO, StepMotor[no].Slp_GPIO_Pin);//GPIO_ResetBits//休眠
            }
            break;
        default:
            PWM_OFF(no);
//SLP1=1;//SLP1=0;
//所有电机不工作时才休眠
if((StepMotor[0].stage>2) && (StepMotor[1].stage>2) && (StepMotor[2].stage>2)
     && (StepMotor[3].stage>2) && (StepMotor[4].stage>2))
    GPIO_SetBits(StepMotor[no].Slp_GPIO, StepMotor[no].Slp_GPIO_Pin);//GPIO_ResetBits//休眠
            break;
    }
}

void DMA1_Stream0_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_TCIF0)==SET)
    {
        DMA_ClearFlag(DMA1_Stream0,DMA_IT_TCIF0);

        StepMotorHandler(0);
    }
}

void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)==SET)
    {
		DMA_ClearFlag(DMA1_Stream3,DMA_IT_TCIF3);

        StepMotorHandler(1);
    }
}

void DMA1_Stream7_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF7)==SET)
    {
		DMA_ClearFlag(DMA1_Stream7,DMA_IT_TCIF7);

        StepMotorHandler(4);
    }
}

void DMA1_Stream4_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4)==SET)
    {
        DMA_ClearFlag(DMA1_Stream4,DMA_IT_TCIF4);

        StepMotorHandler(2);
    }
}

void DMA1_Stream5_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5)==SET)
    {
        DMA_ClearFlag(DMA1_Stream5,DMA_IT_TCIF5);

        StepMotorHandler(3);
    }
}

