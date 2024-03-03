#ifndef __Key_H
#define __Key_H

#include "common.h"

#define KEY0_PRES	1		//KEY0  
#define KEY1_PRES	2		//KEY1 

#define KEY2_PRES	3		//KEY2  
#define KEY3_PRES	4		//KEY3 

extern char Mode;

void Key_Init(void);
uint8 GetKey_Value(uint8 mode);
void Adjust_Mode(void);

#endif