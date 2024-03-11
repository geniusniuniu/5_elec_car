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
char delay_10ms = 0;

//环岛标志位
char circle_flag_L = 0;  // 左右环岛识别建立不同标志位
char circle_flag_R = 0;
char circle_In_Flag = 0;
char circle_Out_Flag = 0;
char circle_Force_Flag = 0;  //强迫出环标志位

//避障相关标志位
char Barrier_Flag1=0;
char Barrier_Flag2=0;
char Barrier_Flag3=0;
char Barrier_Executed = 0;
char Barrier_Flag4=0;
char Avoid_ON = 0;
float Sum_Angle=0;


void Elem_Up_Down(float Angle,float Gyro)  //上下坡
{

	if(Angle > -2 && Gyro > -400)
		Exp_Speed = 300;
	else if(Angle < -15)
		Exp_Speed = 30;
}


//障碍物识别	
void Elem_Barrier(float Gyro_Z)
{
	Gyro_Z = (Gyro_Z*2000)/32768;	
	if(Barrier_Flag1==1)
	{
		Ratio = -0.48 ;			//直接更改期望值
		Sum_Angle += Gyro_Z*0.005;
		
	}
	if(Sum_Angle < -20)   	//右拐避障
	{
		//Ratio = 0.2;
		Barrier_Flag1 = 0;   //出赛道角度停止积分
		Barrier_Flag2 = 1;
		Sum_Angle = 0;		//积分清零
	}
	if(Barrier_Flag2==1)
	{
		Sum_Angle += Gyro_Z*0.005;   
		if(Sum_Angle < 27)  //左拐回正
		{
			Ratio = 0.52; 		
			Barrier_Flag3 = 0;  //尚未回正
		}
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
		
//	#if TRACE_METHOD2
//		Barrier_Flag4 == 30;
//	#endif
						
	}
}

//进行右环岛识别并进出右环岛
void Elem_Circle_R(float Speed,float Gyro_Z)
{
	static float Sum_Dis1 = 0;
	static float Sum_Dis2 = 0;
	static float Sum_Angle_C1 = 0;
	static float Sum_Angle_C2 = 0;
	if(circle_flag_R == 1)  //识别圆环标志位
	{
		Sum_Dis1 += Speed ;
		if(Sum_Dis1 > 4000) //路程积满入环，开始角度积分
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			if(Sum_Angle_C1 < -15) // 入环结束,正常循迹
			{
				Ratio = -0.45;
				Sum_Angle_C1 += Gyro_Z*0.005;
			}
			else
			{
				Sum_Angle_C1 = -21;
				circle_In_Flag = 1;
			}
					
		}
		if(circle_In_Flag == 1) //如果已经进环，判断出环条件，角度积满出环
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			
			if(Sum_Angle_C2 > ROUND_R) //右转角度积分是负值
				Sum_Angle_C2 += Gyro_Z*0.005;
			else	
				circle_Out_Flag = 1;
		}
		if(circle_Out_Flag == 1 && ADC_proc[2] > 70)
		{
			Sum_Dis2 += Speed; //出环路程积分
			if(Sum_Dis2 < 5000)
				Ratio += 0.2;
			else			//出环结束，标志位清零
			{
				circle_flag_R = 0;
				Sum_Dis1 = 0;
				Sum_Dis2 = 0;
				Sum_Angle_C1 = 0;
				Sum_Angle_C2 = 0;
				circle_In_Flag = 0;
				circle_Out_Flag = 0;
				
				//实验室右环岛是最后一个特殊元素
				//如果有多个重复元素，再置一个记数标志位
				//Element_Num 
				//if(Element_Num == x) //重复元素全部走完
//				Avoid_ON == 1	
				
			}
		}
	}
	
}


//进行左环岛识别并进出左环岛
void Elem_Circle_L(float Speed,float Gyro_Z)
{
	static float Sum_Dis1 = 0;
	static float Sum_Dis2 = 0;
	static float Sum_Angle_C1 = 0;
	static float Sum_Angle_C2 = 0;
	if(circle_flag_L == 1)  //识别圆环标志位
	{		
		Sum_Dis1 += Speed ;
		if(Sum_Dis1 > 4000) //路程积满入环，开始角度积分
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			if(Sum_Angle_C1 < 15) // 入环结束,正常循迹
			{
				Ratio = 0.45;
				Sum_Angle_C1 += Gyro_Z*0.005;
			}
			else
			{
				Sum_Angle_C1 = 21;
				circle_In_Flag = 1;	
			}
				
		}
		if(circle_In_Flag == 1) //如果已经进环，正常循迹
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			
			if(Sum_Angle_C2 < ROUND_L) //判断出环条件，角度积满出环 左转角度积分是正值 
				Sum_Angle_C2 += Gyro_Z*0.005;
			else
				circle_Out_Flag = 1;
		}
		if(circle_Out_Flag == 1 && ADC_proc[2] > 70)
		{
			Sum_Dis2 += Speed; //出环路程积分
			if(Sum_Dis2 < 5000)
			{
				Ratio -= 0.2;
			}
			else			//出环结束，标志位清零
			{
				circle_flag_L = 0;
				Sum_Dis1 = 0;
				Sum_Dis2 = 0;
				Sum_Angle_C1 = 0;
				Sum_Angle_C2 = 0;
				circle_In_Flag = 0;
				circle_Out_Flag = 0;
			}
		}
	}
//	//如果误入环岛，也需要正常出去	***************************************************************************/
}
