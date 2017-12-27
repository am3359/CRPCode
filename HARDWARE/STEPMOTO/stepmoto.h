#ifndef __STEPMOTO_H__ 
#define __STEPMOTO_H__ 
#include "sys.h"

#define DIR1  PDout(5)
#define STP1  PBout(6)
#define SLP1  PDout(0)
#define EN1   PDout(1)

#define StepBufLen1     48

void StepMotoInit(void);
void StepMoto1Move(s32 step);

#endif
