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


extern PID_InitTypeDef Left_Wheel_PID;
extern PID_InitTypeDef Right_Wheel_PID;
extern PID_InitTypeDef Turn_PID;

void PID_Init(PID_InitTypeDef *PID_Struct, float Kp, float Ki, float Kd, float Out_limit, float Integral_limit);		//PID��ʼ��
void PID_Calculate(PID_InitTypeDef *PID_Struct, float Exp_Val, float Act_Val);		//PID����



//***********************************************************����ʽPID**************************************************//
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