#ifndef __ADC_H
#define __ADC_H

extern float ADC_proc[];

extern float Ratio;

extern float Ratio_Mid;

void ADC_InitAll(void);
void ADC_GetValue(void);
void Get_Ratio(void);
#endif
