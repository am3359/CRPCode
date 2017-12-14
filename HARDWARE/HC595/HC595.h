#ifndef __HC595_H__ 
#define __HC595_H__ 
#include "sys.h"


#define HC595_CK               PEout(2)
#define HC595_CS               PEout(3)
#define HC595_OE               PEout(4)
#define HC595_DA               PEout(5)

void HC595Send(u8 data); 
void HC595Init(void); 
void HC595Load(void); 
void SwitchOut(u16 Data);

#endif
