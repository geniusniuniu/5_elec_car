#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "Element.h"
#include "Motor.h"
#include "PID.h"
#include "Buzzer.h"

#define ROUND_R -120  //车转一圈陀螺仪角度积分
#define ROUND_L 120  

extern float Exp_Speed;
extern float Ratio;
extern float ADC_proc[5];

//上下坡标志位
char Up_Down_Flag;

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
float Sum_Angle=0;

void Elem_Up_Down(float Angle,float Gyro)  //上下坡
{
	if(Num2Abs(Gyro)>400)
	{
		if(Angle > 7) 		  	{Exp_Speed = 290; x10_ms = 13; }
	}
	if(Angle < -4)     {Exp_Speed = 20; x10_ms = 13; }

}

//障碍物识别	
void Elem_Barrier(float Gyro_Z)
{
//	if (Barrier_Flag4 == 1) 
//	{
//        return ;
//    }
	Gyro_Z = (Gyro_Z*2000)/32768;	
	if(Barrier_Flag1==1)
	{
		Ratio = -0.46;			//直接更改期望值
		Sum_Angle += Gyro_Z*0.005;
		
	}
	if(Sum_Angle < -27)   	//右拐避障
	{
		Barrier_Flag1 = 0;   //出赛道角度停止积分
		Barrier_Flag2 = 1;
		Sum_Angle = 0;		//积分清零
	}
    if(Barrier_Flag2==1)
    {
		Sum_Angle += Gyro_Z*0.005;   
        if(Sum_Angle < 30.5)  //左拐回正
        {
			Ratio = 0.51; 		
			Barrier_Flag3 = 0;  //尚未回正
        }
		else
			Barrier_Flag3 = 1;  //回正
	 }
	if(Barrier_Flag3==1)		//回正后标志位清零
	{	
		Barrier_Flag4 = 1;
		Barrier_Flag1 = 0;
		Barrier_Flag2 = 0;
		Barrier_Flag3 = 0;
		Sum_Angle = 0;
	}
}

//进行右环岛识别并进出右环岛
void Elem_Circle_R(float Speed,float Gyro_Z)
{
	static float Sum_Dis1 = 0;
	static float Sum_Dis2 = 0;
	static float Sum_Angle_C = 0;
	if(circle_flag_R == 1)  //识别圆环标志位
	{
		Sum_Dis1 += Speed ;
		if(Sum_Dis1 > 4000) //路程积满入环，开始角度积分
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			Sum_Angle_C += Gyro_Z*0.005;
			
			if(Sum_Angle_C < -20) // 入环结束,正常循迹
			{
				Ratio = 0;
				circle_In_Flag = 1;
			}
			else
			Ratio = 0.4;
		}
		if(circle_In_Flag == 1) //如果已经进环，判断出环条件，角度积满出环
		{
				Gyro_Z = (Gyro_Z*2000)/32768;
				Sum_Angle_C += Gyro_Z*0.005;
				if(Sum_Angle_C < ROUND_R) //右转角度积分是负值
					circle_Out_Flag = 1;
		}
		if(circle_Out_Flag == 1 || ADC_proc[2] > 80)
		{
			Sum_Dis2 += Speed; //出环路程积分
			if(Sum_Dis2 < 6000)
				Ratio = -0.3;	
			else			//出环结束，标志位清零
			{
				Ratio = 0;
				circle_flag_R = 0;
				Sum_Dis1 = 0;
				Sum_Dis2 = 0;
				Sum_Angle_C = 0;
				circle_In_Flag = 0;
				circle_Out_Flag = 0;
			}
		}
	}
}





//进行左环岛识别并进出左环岛
void Elem_Circle_L(float Speed,float Gyro_Z)
{
	static float Sum_Dis1 = 0;
	static float Sum_Dis2 = 0;
	static float Sum_Angle_C = 0;
	if(circle_flag_L == 1)  //识别圆环标志位
	{
		Sum_Dis1 += Speed ;
		if(Sum_Dis1 > 4000) //路程积满入环，开始角度积分
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			Sum_Angle_C += Gyro_Z*0.005;
			
			if(Sum_Angle_C > 20) // 入环结束,正常循迹
			{
				Ratio = 0;
				circle_In_Flag = 1;
			}
			else
			Ratio = -0.4;
		}
		if(circle_In_Flag == 1) //如果已经进环，判断出环条件，角度积满出环
		{
				Gyro_Z = (Gyro_Z*2000)/32768;
				Sum_Angle_C += Gyro_Z*0.005;
				if(Sum_Angle_C > ROUND_L) //左转角度积分是正值
					circle_Out_Flag = 1;
		}
		if(circle_Out_Flag == 1 || ADC_proc[2] > 80)
		{
			Sum_Dis2 += Speed; //出环路程积分
			if(Sum_Dis2 < 6000)
				Ratio = 0.3;	
			else			//出环结束，标志位清零
			{
				Ratio = 0;
				circle_flag_L = 0;
				Sum_Dis1 = 0;
				Sum_Dis2 = 0;
				Sum_Angle_C = 0;
				circle_In_Flag = 0;
				circle_Out_Flag = 0;
			}
		}
	}
}
