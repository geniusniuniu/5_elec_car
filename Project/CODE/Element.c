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
float Circle_Flag = 0;  // 左右环岛标志位
char Circle_Flag2 = 0;

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

	if(Angle > -4 && Gyro < -400)
		Exp_Speed = 310;
	else if(Angle < -18)
		Exp_Speed = 30;
}


//障碍物识别	 
void Elem_Barrier(float Gyro_Z)
{
	#if BARRIER_DIR == 0
		//向右避障
		Gyro_Z = (Gyro_Z*2000)/32768;	
		if(Barrier_Flag1==1)
		{
			Ratio = -0.48 ;			//直接更改期望值
			Sum_Angle += Gyro_Z*0.005;
			
		}
		if(Sum_Angle < -20)   	//右拐避障
		{
			Ratio = -0.01;
			Barrier_Flag1 = 0;   //出赛道角度停止积分
			Barrier_Flag2 = 1;
			Sum_Angle = 0;		//积分清零
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < 22)  //左拐回正
			{
				Ratio = 0.48; 		
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
		}
	//	#if TRACE_METHOD2
	//		Barrier_Flag4 == 30;
	//	#endif
	#elif BARRIER_DIR == 1  //向左避障
		Gyro_Z = (Gyro_Z*2000)/32768;	
		if(Barrier_Flag1==1)
		{
			Ratio = 0.48;			//直接更改期望值
			Sum_Angle += Gyro_Z*0.005;
			
		}
		if(Sum_Angle > 20)   	//左拐避障
		{
			Ratio = 0.01;
			Barrier_Flag1 = 0;   //出赛道角度停止积分
			Barrier_Flag2 = 1;
			Sum_Angle = 0;		//积分清零
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < -22)  //右拐回正

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
	static float Delay_10Ms = 0;
	if(Delay_10Ms > 0)
	{
		Circle_Flag = 0;
		Circle_Flag2 = 0;
		Ratio -= 0.2;
		Delay_10Ms--;
		return ;        //发生误判，退出函数
	}
	if(Circle_Flag2)
	{
		x10_ms = 13;
		Gyro_Z = (Gyro_Z*2000)/32768;
		if(Sum_Dis1>DIS_ROUND_IN)
		{
			Sum_Angle_C1 += Gyro_Z*0.005;
			if((Circle_Flag == 0 && ADC_proc[0]+ADC_proc[1] > ADC_proc[3]+ADC_proc[4]))
				Circle_Flag = LEFT_CIRCLE;
			else if((Circle_Flag == 0 && ADC_proc[0]+ADC_proc[1] < ADC_proc[3]+ADC_proc[4]) )
				Circle_Flag = RIGHT_CIRCLE;
			
			if(Sum_Angle_C1 < 30 && Circle_Flag == LEFT_CIRCLE)
				Ratio = 0.42;
			if(Sum_Angle_C1 > -30 && Circle_Flag == RIGHT_CIRCLE)
				Ratio = -0.42;
		}
		else
			Sum_Dis1+=Speed;
		
		if(Sum_Angle_C1 > ROUND_L || Sum_Angle_C1 < ROUND_R )
		{
			if(ADC_proc[2] > 64)   //预出环 防止误判入环
			{
				Sum_Dis2 += Speed;
				if(Sum_Dis2 > DIS_ROUND_OUT)
				{
					Sum_Dis1 = 0;
					Sum_Dis2 = 0;
					Sum_Angle_C1 = 0;
					Circle_Flag = 0;
					Circle_Flag2 = 0;
				}
				Delay_10Ms = 250;   //延时2500ms
			}
		}
	}
}



////进行左环岛识别并进出左环岛
//void Elem_Circle_L(float Speed,float Gyro_Z)
//{
//	static float Sum_Dis1 = 0;
//	static float Sum_Dis2 = 0;
//	static float Sum_Angle_C1 = 0;
//	static char Delay_10Ms = 0;
//	if(Delay_10Ms > 0)
//	{
//		circle_flag_L = 0;
//		circle_flag_R = 0;
//		Ratio -= 0.2;
//		Delay_10Ms --;
//		return;        //发生误判，强制退出
//	}

//	if(circle_flag_L == 1)  //识别圆环标志位
//	{	
//		Gyro_Z = (Gyro_Z*2000)/32768;
//		if(Sum_Dis2>DIS_ROUND_IN)
//		{
//			Sum_Angle_C1 += Gyro_Z*0.005;
//			if(Sum_Angle_C1 < 80)
//				Ratio = 0.52;
//		}
//		else
//			Sum_Dis2+=Speed;
//		if(Sum_Angle_C1 > ROUND_L)
//		{
//			if(ADC_proc[2] > 66)   //预出环 防止再次误判
//			{
//				Sum_Dis1 += Speed;
//				if(Sum_Dis1 > DIS_ROUND_OUT)
//				{
//					Sum_Dis1=0;
//					Sum_Dis2=0;
//					Sum_Angle_C1=0;
//					circle_flag_R=0;
//				}
//				Delay_10Ms = 100;   //延时1000ms
//			}
//		}
//		
//				//实验室右环岛是最后一个特殊元素
//				//如果有多个重复元素，再置一个记数标志位
//				//Element_Num 
//				//if(Element_Num == x) //重复元素全部走完
////				Avoid_ON == 1	
//		
////如果误入环岛，也需要正常出去	***************************************************************************/
//	}
//}



//float Sum_Dis1 = 0;
//float Sum_Dis2 = 0;
//float Sum_Angle_C1 = 0;
////进行右环岛识别并进出右环岛
//void Elem_Circle_R(float Speed,float Gyro_Z)
//{
////	static float Sum_Dis1 = 0;
////	static float Sum_Dis2 = 0;
////	static float Sum_Angle_C1 = 0;
//	static char Delay_10Ms = 0;
//	if(Delay_10Ms > 0)
//	{
//		circle_flag_R = 0;
//		circle_flag_L = 0;
//		Ratio += 0.2;
//		Delay_10Ms--;
//		return;
//	}
//	if(circle_flag_R == 1)  //识别圆环标志位
//	{	
//		Gyro_Z = (Gyro_Z*2000)/32768;
//		if(Sum_Dis2>DIS_ROUND_IN)
//		{
//			Sum_Angle_C1 += Gyro_Z*0.005;
//			if(Sum_Angle_C1 > -80)
//				Ratio = -0.52;
//		}
//		else
//			Sum_Dis2 += Speed;
//		if(Sum_Angle_C1 < ROUND_R)
	//	{
	//		if(ADC_proc[2] > 66)   //预出环 防止再次误判
	//		{
	//			Sum_Dis1 += Speed;
	//			if(Sum_Dis1 > DIS_ROUND_OUT)
	//			{
	//				Sum_Dis1=0;
	//				Sum_Dis2=0;
	//				Sum_Angle_C1=0;
	//				circle_flag_R=0;
	//			}
	//			Delay_10Ms = 100;   //延时1000ms
	//		}
//		}
//	}
//}


