#ifndef _AD7799_H
#define _AD7799_H
#include "sys.h"

#define ADC_OP_READ (1<<6)
#define ADC_OP_WRITE 0

#define ADC_REG_COM     (0<<3)  //写
#define ADC_REG_STAT    (0<<3)  //读
#define ADC_REG_MODE    (1<<3)
#define ADC_REG_CONFIG  (2<<3)
#define ADC_REG_DATA    (3<<3)
#define ADC_REG_ID      (4<<3)
#define ADC_REG_IO      (5<<3)
#define ADC_REG_OFFSET  (6<<3)
#define ADC_REG_FS      (7<<3)

#define ADC_CON_CH1     0
#define ADC_CON_CH2     1
#define ADC_CON_CH3     2

#define ADC_CON_GAIN1   0
#define ADC_CON_GAIN2   1
#define ADC_CON_GAIN4   2
#define ADC_CON_GAIN8   3
#define ADC_CON_GAIN16  4
#define ADC_CON_GAIN32  5
#define ADC_CON_GAIN64  6
#define ADC_CON_GAIN128 7

#define ADC_MODE_CONTINUOUS (0<<5)
#define ADC_MODE_SINGLE     (1<<5)
#define ADC_MODE_IDLE       (2<<5)
#define ADC_MODE_POWERDOWN  (3<<5)
#define ADC_MODE_IZEROS     (4<<5) //内部0刻度校准
#define ADC_MODE_IFULLS     (5<<5) //内部满刻度校准

#define ADC_CREAD 4

#define UNIPOLAR 0x10
#define BIPOLAR  0x00

#define AD7799CS PGout(2)
#define ADC_RDY_DAT PBin(14)

void AD7799_WriteReg(u8 RegAddr,u8 *Buffer,u8 Length);
u8 AD7799_WaitBusy(void);
u8 AD7799_Init(u8 Gain);
void AD7799_Start(u8 CovChx,u8 CovGain,u8 CovMode,u8 CovRate);
u32 AD7799_Read(void);

#endif
