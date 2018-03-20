#ifndef __STEPMOTO_H__ 
#define __STEPMOTO_H__ 
#include "sys.h"

//#define DIR1  PDout(5)  //横向运动
#define STP1  PBout(6)
#define SLP1  PDout(0)
#define EN1   PDout(1)

//#define DIR2  PDout(4)  //上下运动
//#define DIR5  PDout(3)  //蠕动泵旋转方向
// 
//#define DIR3  PDout(6)  //注射器1
//#define DIR4  PDout(2)  //注射器2

#define StepBufLen     50
//#define StepBufLen1     80

typedef struct
{
    u16 c0;                          //初值   c0
    u16 c1;                          //匀速值 c1
    u16 a;                           //斜率   (扩大了100被的值)
}TSetStepMotor;

//typedef struct
//{
//    u32 dist;                        //到起始位置距离
//    u16 pot;                         //0：起始位置；n：位置n。开机后到起始位置
//    u16 level;                       //默认速度 0 1 2
//    u16 mode;                        //工作模式 模式0：加速，匀速，减速；模式1：加速，匀速，忽略步数；模式2：匀速，忽略加减速和步数；模式3：匀速，减速；
//}TAttr;

typedef struct
{
    GPIO_TypeDef* Slp_GPIO;
    u16 Slp_GPIO_Pin;
    GPIO_TypeDef* En_GPIO;
    u16 En_GPIO_Pin;
    GPIO_TypeDef* Dir_GPIO;
    u16 Dir_GPIO_Pin;
    GPIO_TypeDef* Lmt_GPIO;
    u16 Lmt_GPIO_Pin;
    
    TIM_TypeDef* timer;              //TIM4;
    u16 TIM_DMASource;               //TIM_DMA_CC1
    u16 CCER;                        //(1<<0)
    
    u32 RCC_AHB1Periph;              //RCC_AHB1Periph_DMA1
    DMA_Stream_TypeDef* DMA_Stream;  //DMA1_Stream0
    u32 DMA_Channel;                 //DMA_Channel_2

    u32 dist;                       //到原点(限位)位置距离
    u8 pot;                         //0：原点(限位)位置；n：位置n。开机后到原点(限位)位置
    u8 level;                       //默认速度 0：慢，1：中，2：快
    u8 mode;                        //工作模式 模式0：加速，匀速，减速；模式1：加速，匀速，忽略步数；模式2：匀速，忽略加减速和步数；模式3：匀速，减速；
    u8 stage;                       //运行阶段 0：加速，1：匀速，2：减速，3或其它：停止
    u8 LmtEn;                       //0，无限位；1，有限位
    u8 LmtBit;                      //限位位0有效还是1有效 Bit_SET,Bit_RESET

    u16 *Buf[3];                    //加速匀速减速数组
    u32 IncEn[3];                   //加速匀速减速，递增或不递增
    u16 Size[3];                    //加速匀速减速数据长度

    TSetStepMotor SetStepMotor[3];

}TStepMotor;//??dist,pot在开机后不一定与实际位置相符，需要主动移到原点位


void StepMotoInit(void);
u16 get_c1(void);
void StepMoto1Move(s32 step);
u32 StepMotoCal(u16 c0,u16 c1,u16 a);
void PWM_OFF(u8 no);
u8 IsLmt(u8 no);
void AllSleep(void);
u32 StepMotoMove(u8 no,u8 mode,s32 steps,u8 level);

extern TStepMotor StepMotor[5];

#endif
