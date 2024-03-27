#ifndef __Element_H
#define __Element_H
#include "PID.h"
#define TRACE_METHOD1 		0  		//单向巡线
#define TRACE_METHOD2 		1  		//向量法巡线


#define RIGHT_CIRCLE		2
#define LEFT_CIRCLE			1

#define ROUND_R 			-110  	//车转一圈陀螺仪角度积分
#define ROUND_L 		 	110  

#define DIS_ROUND_IN        6100
#define DIS_ROUND_OUT       6000

#define BARRIER_FIELD_STATUS	0	// 可以根据赛道状态改变，1表示赛道周围空旷，可以常开避障
									// 0 表示赛道周围多干扰，只有经过障碍前
									// 最后一个特殊元素在开启避障
									
#define BARRIER_DIR 		0 		//0表示向右避障，1表示向左避障
									
									
extern char Up_Down_Flag;
extern char Barrier_Flag1;
extern char Barrier_Flag2;
extern char Barrier_Flag3;
extern char Barrier_Delay;

extern char Avoid_ON;
extern char Barrier_Executed;
extern float Sum_Angle;

extern char Up_Down_Flag;


extern float Circle_Flag1;
extern float Circle_Delay1;	


void Elem_Up_Down(float Angle,float Gyro);  
//void Elem_Circle_R(float Speed,float Gyro_Z);
void Elem_Circle(float Speed,float Gyro_Z);
void Elem_Barrier(float Gyro_Z);

#endif