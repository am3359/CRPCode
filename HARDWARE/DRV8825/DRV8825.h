#ifndef _DRV8825_H
#define _DRV8825_H
#include "sys.h"

#define DIR  PAout(0)
#define STP  PAout(3)
#define SLP  PAout(4)
#define EN  PFout(11)

#define CW  0
#define CCW 1

#define T1_FREQ 1000000
#define FSPR 200
#define SPR (FSPR*2)//2Ï¸·Ö //(FSPR*32)
// Maths constants. To simplify maths when calculating in AxisMove().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*100000*100000)         // 
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000
// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

typedef struct {
	//! What part of the speed ramp we are in.
	u32 run_state ;
	//! Direction stepper motor should move.
	u32 dir ;
	//! Peroid of next timer delay. At start this value set the accelration rate.
	u32 step_delay;
	//! What step_pos to start decelaration
	u32 decel_start;
	//! Sets deceleration rate.
	s32 decel_val;
	//! Minimum time delay (max speed)
	s32 min_delay;
	//! Counter used when accelerateing/decelerateing to calculate step_delay.
	s32 accel_count;
} speedRampData;

void DRV8825_Init(void);
void AxisMove(s32 step, u32 accel, u32 decel, u32 speed);
void AxisMove1(s32 step, u32 accel, u32 decel, u32 speed);
#endif
