#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "Element.h"
#include "Motor.h"
#include "PID.h"
#include "Buzzer.h"

#define ROUND_R -120  //��תһȦ�����ǽǶȻ���
#define ROUND_L 120  

extern float Exp_Speed;
extern float Ratio;
extern float ADC_proc[5];

//�����±�־λ
char Up_Down_Flag;

//������־λ
char circle_flag_L = 0;  // ���һ���ʶ������ͬ��־λ
char circle_flag_R = 0;
char circle_In_Flag = 0;
char circle_Out_Flag = 0;
char circle_Force_Flag = 0;  //ǿ�ȳ�����־λ


//������ر�־λ
char Barrier_Flag1=0;
char Barrier_Flag2=0;
char Barrier_Flag3=0;
char Barrier_Executed = 0;
char Barrier_Flag4=0;
float Sum_Angle=0;

void Elem_Up_Down(float Angle,float Gyro)  //������
{
	if(Num2Abs(Gyro)>400)
	{
		if(Angle > 7) 		  	{Exp_Speed = 290; x10_ms = 13; }
	}
	if(Angle < -4)     {Exp_Speed = 20; x10_ms = 13; }

}

//�ϰ���ʶ��	
void Elem_Barrier(float Gyro_Z)
{
//	if (Barrier_Flag4 == 1) 
//	{
//        return ;
//    }
	Gyro_Z = (Gyro_Z*2000)/32768;	
	if(Barrier_Flag1==1)
	{
		Ratio = -0.46;			//ֱ�Ӹ�������ֵ
		Sum_Angle += Gyro_Z*0.005;
		
	}
	if(Sum_Angle < -27)   	//�ҹձ���
	{
		Barrier_Flag1 = 0;   //�������Ƕ�ֹͣ����
		Barrier_Flag2 = 1;
		Sum_Angle = 0;		//��������
	}
    if(Barrier_Flag2==1)
    {
		Sum_Angle += Gyro_Z*0.005;   
        if(Sum_Angle < 30.5)  //��ջ���
        {
			Ratio = 0.51; 		
			Barrier_Flag3 = 0;  //��δ����
        }
		else
			Barrier_Flag3 = 1;  //����
	 }
	if(Barrier_Flag3==1)		//�������־λ����
	{	
		Barrier_Flag4 = 1;
		Barrier_Flag1 = 0;
		Barrier_Flag2 = 0;
		Barrier_Flag3 = 0;
		Sum_Angle = 0;
	}
}

//�����һ���ʶ�𲢽����һ���
void Elem_Circle_R(float Speed,float Gyro_Z)
{
	static float Sum_Dis1 = 0;
	static float Sum_Dis2 = 0;
	static float Sum_Angle_C = 0;
	if(circle_flag_R == 1)  //ʶ��Բ����־λ
	{
		Sum_Dis1 += Speed ;
		if(Sum_Dis1 > 4000) //·�̻����뻷����ʼ�ǶȻ���
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			Sum_Angle_C += Gyro_Z*0.005;
			
			if(Sum_Angle_C < -20) // �뻷����,����ѭ��
			{
				Ratio = 0;
				circle_In_Flag = 1;
			}
			else
			Ratio = 0.4;
		}
		if(circle_In_Flag == 1) //����Ѿ��������жϳ����������ǶȻ�������
		{
				Gyro_Z = (Gyro_Z*2000)/32768;
				Sum_Angle_C += Gyro_Z*0.005;
				if(Sum_Angle_C < ROUND_R) //��ת�ǶȻ����Ǹ�ֵ
					circle_Out_Flag = 1;
		}
		if(circle_Out_Flag == 1 || ADC_proc[2] > 80)
		{
			Sum_Dis2 += Speed; //����·�̻���
			if(Sum_Dis2 < 6000)
				Ratio = -0.3;	
			else			//������������־λ����
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





//�����󻷵�ʶ�𲢽����󻷵�
void Elem_Circle_L(float Speed,float Gyro_Z)
{
	static float Sum_Dis1 = 0;
	static float Sum_Dis2 = 0;
	static float Sum_Angle_C = 0;
	if(circle_flag_L == 1)  //ʶ��Բ����־λ
	{
		Sum_Dis1 += Speed ;
		if(Sum_Dis1 > 4000) //·�̻����뻷����ʼ�ǶȻ���
		{
			Gyro_Z = (Gyro_Z*2000)/32768;
			Sum_Angle_C += Gyro_Z*0.005;
			
			if(Sum_Angle_C > 20) // �뻷����,����ѭ��
			{
				Ratio = 0;
				circle_In_Flag = 1;
			}
			else
			Ratio = -0.4;
		}
		if(circle_In_Flag == 1) //����Ѿ��������жϳ����������ǶȻ�������
		{
				Gyro_Z = (Gyro_Z*2000)/32768;
				Sum_Angle_C += Gyro_Z*0.005;
				if(Sum_Angle_C > ROUND_L) //��ת�ǶȻ�������ֵ
					circle_Out_Flag = 1;
		}
		if(circle_Out_Flag == 1 || ADC_proc[2] > 80)
		{
			Sum_Dis2 += Speed; //����·�̻���
			if(Sum_Dis2 < 6000)
				Ratio = 0.3;	
			else			//������������־λ����
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
