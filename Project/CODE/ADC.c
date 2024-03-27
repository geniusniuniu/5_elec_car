#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "zf_adc.h"
#include "Motor.h"
#include "Element.h"
#include "math.h"

#define EDGE_PROTECT 35

float ADC_Array_Original[5][3];
float ADC_proc[5];

float sum_L,sum_R;
float Diff,Plus;
float Ratio = 0;
float Diff_Mid,Plus_Mid;
float Ratio_Mid = 0;

void ADC_InitAll(void)
{
	adc_init(ADC_P10, ADC_SYSclk_DIV_2);
	adc_init(ADC_P00, ADC_SYSclk_DIV_2);
	
	adc_init(ADC_P16, ADC_SYSclk_DIV_2);

	adc_init(ADC_P05, ADC_SYSclk_DIV_2);
	adc_init(ADC_P06, ADC_SYSclk_DIV_2);
}

//��ȡ����ADC��ͨ��ֵ����ȡƽ��ֵ
void ADC_GetValue(void)
{
	float temp;
	char i,j;
	for(i=0;i<3;i++)
	{
		ADC_Array_Original[0][i] = adc_once(ADC_P10, ADC_8BIT);		//��һ·���
		ADC_Array_Original[1][i] = adc_once(ADC_P00, ADC_8BIT);		//���·���	

		ADC_Array_Original[2][i] = adc_once(ADC_P05, ADC_8BIT);		//����·���
		
		ADC_Array_Original[3][i] = adc_once(ADC_P06, ADC_8BIT);		//�м���� 
		ADC_Array_Original[4][i] = adc_once(ADC_P16, ADC_8BIT);		//����·���
	}
	//ȡ���ε��ֵƽ��ֵ
	for(i=0;i<5;i++)
	{
		temp = 0;
		for(j=0;j<3;j++)
		{
			temp += ADC_Array_Original[i][j];
		}
		ADC_proc[i] = temp/3;
		//�Ե��ֵ�޷�
		if(ADC_proc[i] >= 200)			ADC_proc[i] = 200;	
		else if(ADC_proc[i] <= 2)		ADC_proc[i] = 2;
		//��һ��,�����ֵ������0~100֮��
		ADC_proc[i] = 100*(ADC_proc[i]/200);	

	}
}


//��ADCֵ���д���õ���Ⱥ�
void Get_Ratio(void)
{
	#if TRACE_METHOD2
	//������
		sum_L = sqrt((ADC_proc[0]*ADC_proc[0]+ADC_proc[1]*ADC_proc[1]));
		sum_R = sqrt((ADC_proc[4]*ADC_proc[4]+ADC_proc[3]*ADC_proc[3]));
		Diff = sum_L - sum_R;
		Plus = sum_L + sum_R;
	    if((ADC_proc[0]+ADC_proc[1]+ADC_proc[3]+ADC_proc[4] > EDGE_PROTECT))  //���С��EDGE_PROTECT
				Ratio = Diff/Plus;											//�������ߣ��´�ƫ��ֵ
//		else																//���ϴλ������ٴμӣ�����
//		{
//			if(Ratio >= 0 && Barrier_Executed == 1)
//				Ratio += 0.2;
//			else
//				Ratio -= 0.2;
//		}
	
	#elif TRACE_METHOD1 //����Ѳ��
		Diff = ADC_proc[0] - ADC_proc[4];
		Plus = ADC_proc[0] + ADC_proc[4];
		
		Diff_Mid = ADC_proc[1] - ADC_proc[3];
		Plus_Mid = ADC_proc[1] + ADC_proc[3];

		Ratio = Diff/Plus;
		Ratio_Mid = Diff_Mid/Plus_Mid;
		if((Plus_Mid > 36 && Plus_Mid < 75)|| Plus <45)
			Ratio = Ratio_Mid;

	#endif
		Limit_Out(&Ratio,-0.9,0.9);   //�޷�
}
