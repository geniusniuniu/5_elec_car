#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "zf_adc.h"
#include "Motor.h"

float ADC_Array_Original[5][3];
float ADC_proc[5];

void ADC_InitAll(void)
{
	adc_init(ADC_P10, ADC_SYSclk_DIV_2);
	adc_init(ADC_P00, ADC_SYSclk_DIV_2);
	
	adc_init(ADC_P16, ADC_SYSclk_DIV_2);

	adc_init(ADC_P05, ADC_SYSclk_DIV_2);
	adc_init(ADC_P06, ADC_SYSclk_DIV_2);
}

void ADC_GetValue(void)
{
	float temp;
	char i,j,k;
	for(i=0;i<3;i++)
	{
		ADC_Array_Original[0][i] = adc_once(ADC_P10, ADC_8BIT);		//��һ·���
		ADC_Array_Original[1][i] = adc_once(ADC_P00, ADC_8BIT);		//���·���

		ADC_Array_Original[2][i] = adc_once(ADC_P16, ADC_8BIT);		//�м����

		ADC_Array_Original[3][i] = adc_once(ADC_P05, ADC_8BIT);		//����·���
		ADC_Array_Original[4][i] = adc_once(ADC_P06, ADC_8BIT);		//����·���
	}
	for(k=0;k<5;k++) //���ֵ ����ȡ����ƽ��ֵ
	{
		temp = 0;
		for(j=0;j<3;j++)
		{
			temp += ADC_Array_Original[k][j];
		}
		ADC_proc[k] = temp/3;
		if(ADC_proc[k] >= 200)			ADC_proc[k] = 200;	//�Ե��ֵ�����޷�����
		else if(ADC_proc[k] <= 2)		ADC_proc[k] = 2;	//�Ե��ֵ�����޷�����

		ADC_proc[k] = 100*(ADC_proc[k]/200);		//��һ��,�����ֵ������0~100֮��		
		if(ADC_proc[0] >= 55 )
			ADC_proc[0] = 55;
		if(ADC_proc[4] >= 55)
			ADC_proc[4] = 55;
	}
}

