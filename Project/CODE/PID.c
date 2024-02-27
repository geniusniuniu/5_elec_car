#include "zf_tim.h"
#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "Motor.h"
#include "ZF_PWM.h"
#include "PID.h"

void PID_Init(PID_InitTypeDef *PID_Struct, float Kp, float Ki, float Kd,float Out_Limit, float Integral_Limit)		//PID初始化
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

void PID_Calculate(PID_InitTypeDef *PID_Struct, float Exp_Val, float Act_Val)		//PID计算
{
	PID_Struct->Err = Exp_Val-Act_Val;		//err值为期望偏差与当前偏差的差值	
	PID_Struct->Integral += PID_Struct->Err;		//误差值累加	
	Limit_Out(&PID_Struct->Integral, -PID_Struct->Integral_Limit,PID_Struct->Integral_Limit);									//积分限幅
	PID_Struct->PID_Out = PID_Struct->Err * PID_Struct->Kp + 
								PID_Struct->Integral * PID_Struct->Ki +	//计算总输出量
									(PID_Struct->Err - PID_Struct->Err_last)*(PID_Struct->Kd);
	
	Limit_Out(&PID_Struct->PID_Out,-PID_Struct->Out_Limit,PID_Struct->Out_Limit);							//输出限幅
	PID_Struct->Err_last = PID_Struct->Err;		//更新上一次err
}




