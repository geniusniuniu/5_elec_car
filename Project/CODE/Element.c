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
char delay_10ms = 0;

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
char Avoid_ON = 0;
float Sum_Angle=0;


void Elem_Up_Down(float Angle,float Gyro)  //������
{

	if(Angle > -4 && Gyro < -400)
		Exp_Speed = 310;
	else if(Angle < -18)
		Exp_Speed = 30;
}


//�ϰ���ʶ��	 
void Elem_Barrier(float Gyro_Z)
{
	#if BARRIER_DIR == 0
		//���ұ���
		Gyro_Z = (Gyro_Z*2000)/32768;	
		if(Barrier_Flag1==1)
		{
			Ratio = -0.48 ;			//ֱ�Ӹ�������ֵ
			Sum_Angle += Gyro_Z*0.005;
			
		}
		if(Sum_Angle < -20)   	//�ҹձ���
		{
			Ratio = -0.01;
			Barrier_Flag1 = 0;   //�������Ƕ�ֹͣ����
			Barrier_Flag2 = 1;
			Sum_Angle = 0;		//��������
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < 22)  //��ջ���
			{
				Ratio = 0.48; 		
			}
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
		}
	//	#if TRACE_METHOD2
	//		Barrier_Flag4 == 30;
	//	#endif
	#elif BARRIER_DIR == 1  //�������
		Gyro_Z = (Gyro_Z*2000)/32768;	
		if(Barrier_Flag1==1)
		{
			Ratio = 0.48 ;			//ֱ�Ӹ�������ֵ
			Sum_Angle += Gyro_Z*0.005;
			
		}
		if(Sum_Angle > 20)   	//��ձ���
		{
			Ratio = 0.01;
			Barrier_Flag1 = 0;   //�������Ƕ�ֹͣ����
			Barrier_Flag2 = 1;
			Sum_Angle = 0;		//��������
		}
		if(Barrier_Flag2==1)
		{
			Sum_Angle += Gyro_Z*0.005;   
			if(Sum_Angle < -22)  //�ҹջ���

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
		}
	#endif				
}



//�����󻷵�ʶ�𲢽����󻷵�
void Elem_Circle_L(float Speed,float Gyro_Z)
{
	static float Sum_Dis1 = 0;
	static float Sum_Dis2 = 0;
	static float Sum_Angle_C1 = 0;
	
	if(Delay_10Ms > 0)
	{
		circle_flag_L = 0;
		Delay_10Ms--;
		return;        //�������У�ǿ���˳�
	}
	if(circle_flag_L == 1)  //ʶ��Բ����־λ
	{	
		Gyro_Z = (Gyro_Z*2000)/32768;
		if(Sum_Dis2>3000)
		{
			Sum_Angle_C1 += Gyro_Z*0.005;
			if(Sum_Angle_C1 < 60)
			{
				Ratio = 0.52;
			}
		}
		else
		{
			Sum_Dis2+=Speed;
		}
		if(Sum_Angle_C1 > ROUND_L)
		{
			Sum_Dis1+=Speed;
			if(Sum_Dis1 > 5500 && Sum_Dis1 <= 10000)
				Ratio = -0.2;
			else if(Sum_Dis1 > 10000)
			{
				Sum_Dis1=0;
				Sum_Dis2=0;
				Sum_Angle_C1=0;
				circle_flag_L=0;
			}
			if(ADC_proc[2] > 65)   //Ԥ���� ��ֹ�ٴ�����
			{
				Delay_10Ms = 50;   //��ʱ500ms
			}
		}
		
				//ʵ�����һ��������һ������Ԫ��
				//����ж���ظ�Ԫ�أ�����һ��������־λ
				//Element_Num 
				//if(Element_Num == x) //�ظ�Ԫ��ȫ������
//				Avoid_ON == 1	
		
//������뻷����Ҳ��Ҫ������ȥ	***************************************************************************/
	}
}



float Sum_Dis1 = 0;
float Sum_Dis2 = 0;
float Sum_Angle_C1 = 0;
//�����һ���ʶ�𲢽����һ���
void Elem_Circle_R(float Speed,float Gyro_Z)
{
//	static float Sum_Dis1 = 0;
//	static float Sum_Dis2 = 0;
//	static float Sum_Angle_C1 = 0;
	static char Delay_10Ms;
	if(Delay_10Ms > 0)
	{
		circle_flag_R = 0;
		Delay_10Ms--;
		return;
	}
	if(circle_flag_R == 1)  //ʶ��Բ����־λ
	{	
		Gyro_Z = (Gyro_Z*2000)/32768;
		if(Sum_Dis2>3000)
		{
			Sum_Angle_C1 += Gyro_Z*0.005;
			if(Sum_Angle_C1 > -40)
			{
				Ratio = -0.47;
			}
		}
		else
			Sum_Dis2 += Speed;
		if(Sum_Angle_C1 < ROUND_R)   //Ԥ����
		{
			Sum_Dis1 += Speed;
			if(Sum_Dis1 > 5500 && Sum_Dis1 <= 12500)
				Ratio = 0.2;
			else if(Sum_Dis1 > 12500)
			{
				Sum_Dis1=0;
				Sum_Dis2=0;
				Sum_Angle_C1=0;
				circle_flag_R=0;
			}
			if(ADC_proc[2] > 65)   //Ԥ���� ��ֹ�ٴ�����
			{
				Delay_10Ms = 50;   //��ʱ500ms
			}
		}
	}
}


