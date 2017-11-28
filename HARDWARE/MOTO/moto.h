#ifndef __MOTO_H
#define __MOTO_H	 
#include "sys.h" 

#define	TMC5130CS 		PBout(12)  		//TMC5130AµÄÆ¬Ñ¡ÐÅºÅ

#define T_POS 		PAin(1)
//#define HOLL 		PDin(8)//GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8)    //PD8
//#define FILTER 		PDin(9)//GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)    //PD9

#define GCONF               0x00
#define GSTAT               0x01
#define IFCNT               0x02
#define SLAVECONF           0x03
#define IOIN                0x04
#define XCOMPARE            0x05

#define IHOLDIRUN           0x10
#define TZEROWAIT           0x11
#define TSTEP               0x12
#define TPWMTHRS            0x13
#define TCOOLTHRS           0x14
#define THIGH               0x15

#define RAMPMODE            0x20
#define XACTUAL             0x21
#define VACTUAL             0x22
#define VSTART              0x23
#define A1                  0x24
#define V1                  0x25
#define AMAX                0x26
#define VMAX                0x27
#define DMAX                0x28
#define D1                  0x2A
#define VSTOP               0x2B
#define TZEROCROSS          0x2C
#define XTARGET             0x2D

#define VDCMIN              0x33
#define SWMODE              0x34
#define RAMPSTAT            0x35
#define XLATCH              0x36

#define WRITE               0x80
#define READ                0x00

#define RIGHT0              0x00000001
#define LEFT0               0x00000002
#define RIGHT1              0x00000002
#define LEFT1               0x00000001

void MOTO_Init(void);
void MOTO_CmdWrRd(u8 *TxData,u8 *RxData);
u32 MOTO_Cmd(u8 command,u32 data);
#endif
