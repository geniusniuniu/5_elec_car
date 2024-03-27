#include "zf_tim.h"
#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "Motor.h"
#include "ZF_PWM.h"
#include "PID.h"


//***********************************************************位置式PID************************************************************//

PID_InitTypeDef Left_Wheel_PID;
PID_InitTypeDef Right_Wheel_PID;
PID_InitTypeDef Turn_PID;

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
		//积分限幅
	if(PID_Struct->Integral > PID_Struct->Integral_Limit){
		PID_Struct->Integral = PID_Struct->Integral_Limit;
	} else if(PID_Struct->Integral < -PID_Struct->Integral_Limit){
		PID_Struct->Integral = -PID_Struct->Integral_Limit;
	}
	PID_Struct->PID_Out = PID_Struct->Err * PID_Struct->Kp + 
								PID_Struct->Integral * PID_Struct->Ki +	//计算总输出量
									(PID_Struct->Err - PID_Struct->Err_last)*(PID_Struct->Kd);
	
	//输出限幅
	if(PID_Struct->PID_Out > PID_Struct->Out_Limit){
		PID_Struct->PID_Out = PID_Struct->Out_Limit;
	} else if(PID_Struct->PID_Out < -PID_Struct->Out_Limit){
		PID_Struct->PID_Out = -PID_Struct->Out_Limit;
	}
	PID_Struct->Err_last = PID_Struct->Err;		//更新上一次err
}




//***********************************************************增量式PID**************************************************//


PID_Incremental Turn;
PID_Incremental Left_Wheel;
PID_Incremental Right_Wheel;

PID_Incremental PID_Incremental_Init(float Kp, float Ki, float Kd,float Out_Limit)
{
	PID_Incremental pid;
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;
	
    pid.error = 0;
    pid.last_error = 0;
    pid.last_last_error = 0;
    pid.last_out = 0;
    pid.out = 0;
    pid.outmax = Out_Limit;
    pid.outmin = -Out_Limit;
	return pid;
}


float PID_Incremental_Calc(PID_Incremental *pid, float setpoint, float input_value)
{
	float derivative;
	float output_increment;
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = setpoint - input_value;
    derivative = (pid->error - 2 * pid->last_last_error + pid->last_last_error);
    output_increment = pid->Kp * (pid->error - pid->last_error) + pid->Ki * pid->error + pid->Kd * derivative;

    pid->out += output_increment;

    if(pid->out > pid->outmax){
		pid->out = pid->outmax;
	} else if(pid->out < pid->outmin){
		pid->out = pid->outmin;
	}


    pid->last_out = pid->out;

    return pid->out;
}


