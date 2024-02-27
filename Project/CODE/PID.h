#ifndef __PID_H
#define __PID_H

typedef struct
{
	float Err;			  //����ƫ��ֵ
	float Err_last;		  //������һ��ƫ��ֵ

	float Kp,Ki,Kd;		  //������������֡�΢��
	float Integral;       // ������
	float PID_Out;		  //�����������
	
	float Out_Limit;      // ����޷�
	float Integral_Limit;  // �����޷� 
}PID_InitTypeDef;


void PID_Init(PID_InitTypeDef *PID_Struct, float Kp, float Ki, float Kd, float Out_limit, float Integral_limit);		//PID��ʼ��
void PID_Calculate(PID_InitTypeDef *PID_Struct, float Exp_Val, float Act_Val);		//PID����

#endif