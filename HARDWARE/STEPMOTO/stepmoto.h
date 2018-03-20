#ifndef __STEPMOTO_H__ 
#define __STEPMOTO_H__ 
#include "sys.h"

//#define DIR1  PDout(5)  //�����˶�
#define STP1  PBout(6)
#define SLP1  PDout(0)
#define EN1   PDout(1)

//#define DIR2  PDout(4)  //�����˶�
//#define DIR5  PDout(3)  //�䶯����ת����
// 
//#define DIR3  PDout(6)  //ע����1
//#define DIR4  PDout(2)  //ע����2

#define StepBufLen     50
//#define StepBufLen1     80

typedef struct
{
    u16 c0;                          //��ֵ   c0
    u16 c1;                          //����ֵ c1
    u16 a;                           //б��   (������100����ֵ)
}TSetStepMotor;

//typedef struct
//{
//    u32 dist;                        //����ʼλ�þ���
//    u16 pot;                         //0����ʼλ�ã�n��λ��n����������ʼλ��
//    u16 level;                       //Ĭ���ٶ� 0 1 2
//    u16 mode;                        //����ģʽ ģʽ0�����٣����٣����٣�ģʽ1�����٣����٣����Բ�����ģʽ2�����٣����ԼӼ��ٺͲ�����ģʽ3�����٣����٣�
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

    u32 dist;                       //��ԭ��(��λ)λ�þ���
    u8 pot;                         //0��ԭ��(��λ)λ�ã�n��λ��n��������ԭ��(��λ)λ��
    u8 level;                       //Ĭ���ٶ� 0������1���У�2����
    u8 mode;                        //����ģʽ ģʽ0�����٣����٣����٣�ģʽ1�����٣����٣����Բ�����ģʽ2�����٣����ԼӼ��ٺͲ�����ģʽ3�����٣����٣�
    u8 stage;                       //���н׶� 0�����٣�1�����٣�2�����٣�3��������ֹͣ
    u8 LmtEn;                       //0������λ��1������λ
    u8 LmtBit;                      //��λλ0��Ч����1��Ч Bit_SET,Bit_RESET

    u16 *Buf[3];                    //�������ټ�������
    u32 IncEn[3];                   //�������ټ��٣������򲻵���
    u16 Size[3];                    //�������ټ������ݳ���

    TSetStepMotor SetStepMotor[3];

}TStepMotor;//??dist,pot�ڿ�����һ����ʵ��λ���������Ҫ�����Ƶ�ԭ��λ


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
