#ifndef __MOTOR_H
#define __MOTOR_H

#include "zf_tim.h"
#include "PID.h"

#define SPEED_MAX 8700

extern float Speed_L;
extern float Speed_R;

extern char A;
extern char A1;



float Num2Abs(float x);
void Limit_Out(float *Output,float Limit_Min,float Limit_Max);

void Motor_Init(void);
void Right_SetSpeed(float speed);
void Left_SetSpeed(float speed);
void Get_Speed(void);	//获取速度
void Motor_Test(float Speed);  //用来测试电机接线
#endif
