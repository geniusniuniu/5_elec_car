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


extern PID_InitTypeDef Left_Wheel_PID;
extern PID_InitTypeDef Right_Wheel_PID;
extern PID_InitTypeDef Turn_PID;

void PID_Init(PID_InitTypeDef *PID_Struct, float Kp, float Ki, float Kd, float Out_limit, float Integral_limit);		//PID初始化
void PID_Calculate(PID_InitTypeDef *PID_Struct, float Exp_Val, float Act_Val);		//PID计算



//***********************************************************增量式PID**************************************************//
typedef struct{
    float Kp;
    float Ki;
    float Kd;
	
    float error;
    float last_error;
    float last_last_error;
    float last_out;
	
    float out;
    float outmax;
    float outmin;
	
	
} PID_Incremental;


extern PID_Incremental Turn;
extern PID_Incremental Left_Wheel;
extern PID_Incremental Right_Wheel;




PID_Incremental PID_Incremental_Init(float Kp, float Ki, float Kd,float Out_Limit);
float PID_Incremental_Calc(PID_Incremental *pid, float setpoint, float input_value);






#endif