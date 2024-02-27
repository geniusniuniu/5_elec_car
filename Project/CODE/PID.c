#include "zf_tim.h"
#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "Motor.h"
#include "ZF_PWM.h"
#include "PID.h"

void PID_Init(PID_InitTypeDef *PID_Struct, float Kp, float Ki, float Kd,float Out_Limit, float Integral_Limit)		//PID��ʼ��
{
	PID_Struct->Kp = Kp;
	PID_Struct->Ki = Ki;
	PID_Struct->Kd = Kd;
	
	PID_Struct->Err = 0;
	PID_Struct->Err_last = 0;

	PID_Struct->PID_Out = 0;
	PID_Struct->Out_Limit = Out_Limit;
	PID_Struct->Integral_Limit = Integral_Limit;
}

void PID_Calculate(PID_InitTypeDef *PID_Struct, float Exp_Val, float Act_Val)		//PID����
{
	PID_Struct->Err = Exp_Val-Act_Val;		//errֵΪ����ƫ���뵱ǰƫ��Ĳ�ֵ	
	PID_Struct->Integral += PID_Struct->Err;		//���ֵ�ۼ�	
	Limit_Out(&PID_Struct->Integral, -PID_Struct->Integral_Limit,PID_Struct->Integral_Limit);									//�����޷�
	PID_Struct->PID_Out = PID_Struct->Err * PID_Struct->Kp + 
								PID_Struct->Integral * PID_Struct->Ki +	//�����������
									(PID_Struct->Err - PID_Struct->Err_last)*(PID_Struct->Kd);
	
	Limit_Out(&PID_Struct->PID_Out,-PID_Struct->Out_Limit,PID_Struct->Out_Limit);							//����޷�
	PID_Struct->Err_last = PID_Struct->Err;		//������һ��err
}




