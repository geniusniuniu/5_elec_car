#ifndef __Element_H
#define __Element_H
#include "PID.h"

#define TRACE_METHOD1 		0  		//单向巡线
#define TRACE_METHOD2 		1  		//向量法巡线


#define RIGHT_CIRCLE		2
#define LEFT_CIRCLE			1

#define ROUND_R 			-120  	//车转一圈陀螺仪角度积分
#define ROUND_L 		 	120  

#define DIS_ROUND_IN        3800
#define DIS_ROUND_OUT       6000

#define BARRIER_FIELD_STATUS	0	// 可以根据赛道状态改变，1表示赛道周围空旷，可以常开避障
									// 0 表示赛道周围多干扰，只有经过障碍前
									// 最后一个特殊元素在开启避障
#define BARRIER_DIR 		1 		//0表示向左避障，1表示向右避障
									
									
extern char Up_Down_Flag;
extern char Barrier_Flag1;
extern char Barrier_Flag2;
extern char Barrier_Flag3;
extern char Barrier_Flag4;

extern char Avoid_ON;
extern char Barrier_Executed;
extern float Sum_Angle;

extern char Up_Down_Flag;

extern float Sum_Dis1;
extern float Sum_Dis2;
extern float Sum_Angle_C1;
extern char Circle_Flag2;

extern float Circle_Flag;
//extern char circle_flag_R;


void Elem_Up_Down(float Angle,float Gyro);  
//void Elem_Circle_R(float Speed,float Gyro_Z);
void Elem_Circle(float Speed,float Gyro_Z);
void Elem_Barrier(float Gyro_Z);

#endif