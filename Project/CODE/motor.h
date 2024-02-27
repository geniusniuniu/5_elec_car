#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_tim.h"
#include "PID.h"

#define Speed_Max 9000

extern float Speed_L;
extern float Speed_R;

extern PID_InitTypeDef Left_Wheel_PID;
extern PID_InitTypeDef Right_Wheel_PID;
extern PID_InitTypeDef Turn_PID;

float Num2Abs(float x);
void Limit_Out(float *Output,float Limit_Min,float Limit_Max);

void Motor_Init(void);
void Right_SetSpeed(float speed);
void Left_SetSpeed(float speed);
void Get_Speed(void);	//获取速度

#endif
