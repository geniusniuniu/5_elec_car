#ifndef __PID_H
#define __PID_H

typedef struct
{
	float Err;			  //定义偏差值
	float Err_last;		  //定义上一次偏差值

	float Kp,Ki,Kd;		  //定义比例、积分、微分
	float Integral;       // 积分项
	float PID_Out;		  //定义总输出量
	
	float Out_Limit;      // 输出限幅
	float Integral_Limit;  // 积分限幅 
}PID_InitTypeDef;


void PID_Init(PID_InitTypeDef *PID_Struct, float Kp, float Ki, float Kd, float Out_limit, float Integral_limit);		//PID初始化
void PID_Calculate(PID_InitTypeDef *PID_Struct, float Exp_Val, float Act_Val);		//PID计算

#endif