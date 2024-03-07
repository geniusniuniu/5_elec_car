#include "zf_tim.h"
#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "Motor.h"
#include "ZF_PWM.h"
#include "zf_gpio.h"
#include "PID.h"
#include <STC32Gxx.H>
#include <math.h>

PID_InitTypeDef Left_Wheel_PID;
PID_InitTypeDef Right_Wheel_PID;
PID_InitTypeDef Turn_PID;

float Speed_L,Speed_R;

void Limit_Out(float *Output,float Limit_Min,float Limit_Max)
{
    if (*Output <= Limit_Min) *Output = Limit_Min;
    else if (*Output >= Limit_Max) *Output = Limit_Max;
}

float Num2Abs(float x)
{
	if(x < 0)
		return -x;
	return x;
}

void Motor_Init(void)
{
	pwm_init(PWMA_CH1P_P60, 15000, 0);		//����-
	pwm_init(PWMA_CH3P_P64, 15000, 0);		//�ҵ��-
	pwm_init(PWMA_CH2P_P62, 15000, 0);		//����--PWM
	pwm_init(PWMA_CH4P_P66, 15000, 0);		//�ҵ��-PWM
}


void Left_SetSpeed(float speed)
{
	static char Special_NumL = 0;  //�쳣�������
	
	if(abs(speed) >= (SPEED_MAX-100))  //����50�ζ��ƽ�����ٶȣ��쳣���ֱ������
		Special_NumL++;
	else
		Special_NumL = 0;
	if(Special_NumL == 50)   //�����쳣
	{
		pwm_duty(PWMA_CH2P_P62, 0);
        pwm_duty(PWMA_CH1P_P60, 0);
//		x10_ms = 10;
		return ;
	}
    else				 //û�г����쳣
	{
		if(speed >= SPEED_MAX)			speed = SPEED_MAX; 
		else if(speed <= -SPEED_MAX)	speed = -SPEED_MAX;
		if(speed > 0)
		{
			pwm_duty(PWMA_CH2P_P62, (int)speed);
			pwm_duty(PWMA_CH1P_P60, 0);
		} 
		else if(speed < 0)
		{
			pwm_duty(PWMA_CH2P_P62, 0);
			pwm_duty(PWMA_CH1P_P60, -(int)speed);
		} 
		else 
		{
			pwm_duty(PWMA_CH2P_P62, 0);
			pwm_duty(PWMA_CH1P_P60, 0);
		}
	}
	
}

void Right_SetSpeed(float speed)	
{
	static char  Special_NumR = 0; //�쳣�������
	
    if(speed >= SPEED_MAX) 			speed = SPEED_MAX;  
	else if(speed <= -SPEED_MAX)	speed = -SPEED_MAX;
	
	if(abs(speed) >= (SPEED_MAX-100))  //����50�ζ��ƽ�����ٶȣ��쳣���ֱ������
		Special_NumR++;
	else
		Special_NumR = 0;
	if(Special_NumR == 50)
	{
		pwm_duty(PWMA_CH3P_P64, 0);
        pwm_duty(PWMA_CH4P_P66, 0);
		
		return ;
	}
    else
	{
		if(speed > 0)
		{
			pwm_duty(PWMA_CH3P_P64, (int)speed);
			pwm_duty(PWMA_CH4P_P66, 0);
		} 
		else if(speed < 0)
		{
			pwm_duty(PWMA_CH3P_P64, 0);
			pwm_duty(PWMA_CH4P_P66, -(int)speed);
		} 
		else 
		{
			pwm_duty(PWMA_CH3P_P64, 0);
			pwm_duty(PWMA_CH4P_P66, 0);
		}
	
	}
}

void Get_Speed(void)	//��ȡ�ٶ�
{
	if(P53 == 0)  		 Speed_L = ctimer_count_read(CTIM3_P04);	////���ַ����ȡ��ֵ
	else  				{Speed_L = ctimer_count_read(CTIM3_P04); Speed_L = -Speed_L;}
	ctimer_count_clean(CTIM3_P04);									//�������ֵ����ʼ��һ�ּ���

	
	if(P35 == 1)		 Speed_R = ctimer_count_read(CTIM0_P34);		//���ַ����ȡ��ֵ
	else  				{Speed_R = ctimer_count_read(CTIM0_P34); Speed_R = -Speed_R;}
	ctimer_count_clean(CTIM0_P34);									//�������ֵ����ʼ��һ�ּ���
}

