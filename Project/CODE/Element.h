#ifndef __Element_H
#define __Element_H
#include "PID.h"

extern char Up_Down_Flag;
extern char Barrier_Flag1;
extern char Barrier_Flag2;
extern char Barrier_Flag3;
extern char Barrier_Flag4;
extern char Barrier_Executed;
extern float Sum_Angle;

extern char circle_flag_L;
extern char circle_flag_R;
extern char circle_In_Flag;
extern char circle_Out_Flag;
extern char circle_Force_Flag;

void Elem_Up_Down(float Angle,float Gyro);  //������
void Elem_Circle_R(float Speed,float Gyro_Z);
void Elem_Circle_L(float Speed,float Gyro_Z);
void Elem_Barrier(float Gyro_Z);

#endif