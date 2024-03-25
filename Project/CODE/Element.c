#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "Element.h"
#include "Motor.h"
#include "PID.h"
#include "Buzzer.h"
#include "MPU6050.h"

extern float Exp_Speed;
extern float Ratio;
extern float ADC_proc[5];

//上下坡标志位
char Up_Down_Flag = 0;
char Circle_Delay2 = 0;

//环岛标志位
char Circle_Flag1 = 0;  // 左右环岛标志位
char Circle_Flag2 = 0;
char Circle_Flag3 = 0; 



void Elem_Up_Down(float Angle,float Gyro)  //上下坡
{

	if(Angle > -4 && Gyro < -400)
		Exp_Speed = 380;
	else if(Angle < -18)
		Exp_Speed = 30;
}




//障碍物识别
char Barrier_Flag1=0;
char Barrier_Flag2=0;
char Barrier_Flag3=0;
char Barrier_Executed = 0;
char Barrier_Delay = 0;
char Avoid_ON = 0;
float Sum_Angle=0;

void Elem_Barrier(float Gyro_Z)
{
	Gyro_Z = (Gyro_Z*2000)/32768;	
	#if BARRIER_DIR == 0		//向右避障
		if(Barrier_Flag1==1)
		{
			Ratio = -0.48 ;			//直接更改期望值
			Sum_Angle += Gyro_Z*0.005;	
		}
		if(Sum_Angle < -20)   		//右拐避障
		{
			Barrier_Flag1 = 0;   	//出赛道角度停止积分
			Barrier_Flag2 = 1;
			Sum_Angle = 0;			//积分清零
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < 24)  	//左拐回正
				Ratio = 0.42; 		
			else
				Barrier_Flag3 = 1;  //回正标志位
		}
		
		if(Barrier_Flag3==1)		//回正后标志位清零
		{	
			Barrier_Flag1 = 0;
			Barrier_Flag2 = 0;
			Barrier_Flag3 = 0;
			Sum_Angle = 0;
			Barrier_Executed = 1;
			Avoid_ON = 0;
			#if TRACE_METHOD2
				Barrier_Delay = 30;
			#endif
		}
		
		
	#elif BARRIER_DIR == 1  //向左避障	
		if(Barrier_Flag1==1)
		{
			Ratio = 0.48;			//直接更改期望值
			Sum_Angle += Gyro_Z*0.005;
			
		}
		if(Sum_Angle > 20)   	//左拐避障
		{
			Barrier_Flag1 = 0;   //出赛道角度停止积分
			Barrier_Flag2 = 1;
			Sum_Angle = 0;		//积分清零
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < -25)  //右拐回正

				Ratio = -0.48; 		
			else
				Barrier_Flag3 = 1;  //回正
		}
		
		if(Barrier_Flag3==1)		//回正后标志位清零
		{	
			Barrier_Flag1 = 0;
			Barrier_Flag2 = 0;
			Barrier_Flag3 = 0;
			Sum_Angle = 0;
			Barrier_Executed = 1;
			Avoid_ON = 0;
			#if TRACE_METHOD2
				Barrier_Delay = 30;
			#endif
		}

	#endif			
}


float Sum_Dis1 = 0;
float Sum_Dis2 = 0;
float Sum_Angle_C1 = 0;
void Elem_Circle(float Speed,float Gyro_Z)
{
//	static float Sum_Dis1 = 0;
//	static float Sum_Dis2 = 0;
//	static float Sum_Angle_C1 = 0;
	static float Circle_Delay2 = 0;			//出环延时
	
	if(Circle_Delay2 > 0)					//出环，清除标志位
	{
		Circle_Flag1 = 0;
		Circle_Flag2 = 0;
		if(Circle_Flag3 == LEFT_CIRCLE)		//用来记录出环时的方向
			Ratio -=0.2;
		else if(Circle_Flag3 == RIGHT_CIRCLE)
			Ratio +=0.2;
		Circle_Delay2--;
		return ;        					//发生误判，退出函数
	}

	
	if(Circle_Flag1)						//识别到环岛
	{
		Gyro_Z = (Gyro_Z*2000)/32768;
		if(Sum_Dis1 > DIS_ROUND_IN)			//路程积分，积满进环
		{
			Sum_Angle_C1 += Gyro_Z*0.005;
			if(Circle_Flag2 == 0 && (ADC_proc[0]+ADC_proc[1] > ADC_proc[3]+ADC_proc[4]))		//标志位未清零时只置位一次
				Circle_Flag2 = LEFT_CIRCLE;
			else if(Circle_Flag2 == 0 && (ADC_proc[0]+ADC_proc[1] < ADC_proc[3]+ADC_proc[4]))
				Circle_Flag2 = RIGHT_CIRCLE;
			
			Circle_Flag3 = Circle_Flag2;														//变量记录方向
			
			if(Sum_Angle_C1 < 35  && Circle_Flag2 == LEFT_CIRCLE)								//角度积满，入环成功，正常循迹
				Ratio = 0.52;
			if(Sum_Angle_C1 > -35 && Circle_Flag2 == RIGHT_CIRCLE)
				Ratio = -0.52;
		}
		else
			Sum_Dis1+=Speed;
		
		if(Sum_Angle_C1 > ROUND_L || Sum_Angle_C1 < ROUND_R )									//出环条件之一，角度积分够大
		{
			if(ADC_proc[2] > 64 || ADC_proc[0] > 59 || ADC_proc[4] > 59)   						//预出环 防止误判再次入环
			{
				Sum_Dis2 += Speed;
				if(Sum_Dis2 > DIS_ROUND_OUT)													//路程积满出环
				{
					Sum_Dis1 = 0;
					Sum_Dis2 = 0;
					Sum_Angle_C1 = 0;
					Circle_Flag1 = 0;
					Circle_Flag2 = 0;
				}
				Circle_Delay2 = 80;   //延时800ms
			}
		}
	}
	else
	{
		Sum_Dis1 = 0;
		Sum_Dis2 = 0;
		Sum_Angle_C1 = 0;
		Circle_Flag2 = 0;
	}
}
