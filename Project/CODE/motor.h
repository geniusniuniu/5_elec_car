#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_tim.h"
#include "PID.h"

#define SPEED_MAX 			8700
#define Special_Speed 		850


extern float Speed_L;
extern float Speed_R;


float Num2Abs(float x);
void Limit_Out(float *Output,float Limit_Min,float Limit_Max);

void Motor_Init(void);
void Right_SetSpeed(float speed);
void Left_SetSpeed(float speed);
void Get_Speed(void);	//��ȡ�ٶ�
void Motor_Test(float Speed);  //�������Ե������
#endif
