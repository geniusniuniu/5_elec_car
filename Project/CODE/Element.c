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

//�����±�־λ
char Up_Down_Flag = 0;
char Circle_Delay2 = 0;

//������־λ
char Circle_Flag1 = 0;  // ���һ�����־λ
char Circle_Flag2 = 0;
char Circle_Flag3 = 0; 



void Elem_Up_Down(float Angle,float Gyro)  //������
{

	if(Angle > -4 && Gyro < -400)
		Exp_Speed = 380;
	else if(Angle < -18)
		Exp_Speed = 30;
}




//�ϰ���ʶ��
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
	#if BARRIER_DIR == 0		//���ұ���
		if(Barrier_Flag1==1)
		{
			Ratio = -0.48 ;			//ֱ�Ӹ�������ֵ
			Sum_Angle += Gyro_Z*0.005;	
		}
		if(Sum_Angle < -20)   		//�ҹձ���
		{
			Barrier_Flag1 = 0;   	//�������Ƕ�ֹͣ����
			Barrier_Flag2 = 1;
			Sum_Angle = 0;			//��������
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < 24)  	//��ջ���
				Ratio = 0.42; 		
			else
				Barrier_Flag3 = 1;  //������־λ
		}
		
		if(Barrier_Flag3==1)		//�������־λ����
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
		
		
	#elif BARRIER_DIR == 1  //�������	
		if(Barrier_Flag1==1)
		{
			Ratio = 0.48;			//ֱ�Ӹ�������ֵ
			Sum_Angle += Gyro_Z*0.005;
			
		}
		if(Sum_Angle > 20)   	//��ձ���
		{
			Barrier_Flag1 = 0;   //�������Ƕ�ֹͣ����
			Barrier_Flag2 = 1;
			Sum_Angle = 0;		//��������
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < -25)  //�ҹջ���

				Ratio = -0.48; 		
			else
				Barrier_Flag3 = 1;  //����
		}
		
		if(Barrier_Flag3==1)		//�������־λ����
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
	static float Circle_Delay2 = 0;			//������ʱ
	
	if(Circle_Delay2 > 0)					//�����������־λ
	{
		Circle_Flag1 = 0;
		Circle_Flag2 = 0;
		if(Circle_Flag3 == LEFT_CIRCLE)		//������¼����ʱ�ķ���
			Ratio -=0.2;
		else if(Circle_Flag3 == RIGHT_CIRCLE)
			Ratio +=0.2;
		Circle_Delay2--;
		return ;        					//�������У��˳�����
	}

	
	if(Circle_Flag1)						//ʶ�𵽻���
	{
		Gyro_Z = (Gyro_Z*2000)/32768;
		if(Sum_Dis1 > DIS_ROUND_IN)			//·�̻��֣���������
		{
			Sum_Angle_C1 += Gyro_Z*0.005;
			if(Circle_Flag2 == 0 && (ADC_proc[0]+ADC_proc[1] > ADC_proc[3]+ADC_proc[4]))		//��־λδ����ʱֻ��λһ��
				Circle_Flag2 = LEFT_CIRCLE;
			else if(Circle_Flag2 == 0 && (ADC_proc[0]+ADC_proc[1] < ADC_proc[3]+ADC_proc[4]))
				Circle_Flag2 = RIGHT_CIRCLE;
			
			Circle_Flag3 = Circle_Flag2;														//������¼����
			
			if(Sum_Angle_C1 < 35  && Circle_Flag2 == LEFT_CIRCLE)								//�ǶȻ������뻷�ɹ�������ѭ��
				Ratio = 0.52;
			if(Sum_Angle_C1 > -35 && Circle_Flag2 == RIGHT_CIRCLE)
				Ratio = -0.52;
		}
		else
			Sum_Dis1+=Speed;
		
		if(Sum_Angle_C1 > ROUND_L || Sum_Angle_C1 < ROUND_R )									//��������֮һ���ǶȻ��ֹ���
		{
			if(ADC_proc[2] > 64 || ADC_proc[0] > 59 || ADC_proc[4] > 59)   						//Ԥ���� ��ֹ�����ٴ��뻷
			{
				Sum_Dis2 += Speed;
				if(Sum_Dis2 > DIS_ROUND_OUT)													//·�̻�������
				{
					Sum_Dis1 = 0;
					Sum_Dis2 = 0;
					Sum_Angle_C1 = 0;
					Circle_Flag1 = 0;
					Circle_Flag2 = 0;
				}
				Circle_Delay2 = 80;   //��ʱ800ms
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
