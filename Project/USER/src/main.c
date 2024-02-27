#include "headfile.h"
#include "Motor.h"
#include "ADC.h"
#include "MPU6050.h"
#include "Element.h"
#include "ADC.h"
#include "math.h"
#include "TOF.h"

short gx, gy, gz;
char Isr_flag_20 = 0; //�ж�2�δ���
char Isr_flag_10 = 0; 

float Diff,Plus; // �� / ��
float Ratio;
float Ratio_Last;
float sum_L,sum_R,sum;


float Exp_Speed_L = 0;
float Exp_Speed_R = 0;
float Exp_Speed = 200;
extern uint8 vl53l0x_finsh_flag;
extern uint16 vl53l0x_distance_mm;

void Init_all(void)
{
	WTST = 0;						//���ó������ȴ���������ֵΪ0�ɽ�CPUִ�г�����ٶ�����Ϊ���  
	DisableGlobalIRQ();				//�ر����ж�
	sys_clk = 35000000;				//����ϵͳƵ��Ϊ35MHz

//��ʼ���Ĵ���
	board_init();					

////编码器方向引脚初始化
	gpio_mode(P5_3, GPIO);          
	gpio_mode(P3_5, GPIO);		

////测距模块初始化
	//gpio_mode(P3_2, GPIO);
	vl53l0x_init();
	
////OLED初始化
	oled_init();					
//	
////MPU6050初始化
	MPU6050_DMP_Init();	
//	
////定时器初始化
	pit_timer_ms(TIM_4, 10);		//10ms定时器
//	
////编码器初始化
	ctimer_count_init(CTIM0_P34);	//编码器1计数
	ctimer_count_init(CTIM3_P04);	//编码器2计数
//	
////串口初始化
//	uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, 115200, TIM_2);
//	
////电机初始化
	Motor_Init();
////蜂鸣器初始化
	pwm_init(PWMB_CH4_P77，100，1000);
////初始化所有AD引脚
	ADC_InitAll(); 
	
////pid初始化  PID_Init(结构体, KP, KI, KD, 输出限幅，积分限幅)
	PID_Init(&Left_Wheel_PID , 20, 0.5, 0, 10000, 2000);
	PID_Init(&Right_Wheel_PID, 20, 0.5, 0, 10000, 2000);
	PID_Init(&Turn_PID , -2, 0, 0 ,10000, 0);
} 

//对ADC值进行处理得到差比和
void Get_Ratio(void)
{
	sum = ADC_proc[0]+ ADC_proc[1]+ADC_proc[2]+ADC_proc[3]+ADC_proc[4];
	sum_L = sqrt((ADC_proc[0]*ADC_proc[0]+ADC_proc[1]*ADC_proc[1]));
	sum_R = sqrt((ADC_proc[4]*ADC_proc[4]+ADC_proc[3]*ADC_proc[3]));
	Diff = sum_L - sum_R;
	Plus = sum_L + sum_R;
	Ratio = Diff/Plus;
	Ratio = 0.7*Ratio +0.3*Ratio_Last;
	Ratio_Last = Ratio;
}

void main(void)	
{
	Init_all();
	EnableGlobalIRQ();	//���ж������
	while(1)
	{
		oled_printf_float(0,0,vl53l0x_distance_mm,5,2);
		oled_printf_float(0,2,ADC_proc[1],5,2);
		oled_printf_float(0,4,Ratio,5,2);
		oled_printf_float(0,6,Pitch,5,2);
//		printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Exp_Speed_L,Exp_Speed_R,Speed_L,Speed_R,Turn_PID.PID_Out*0.09);
//		printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",ADC_proc[0],ADC_proc[1],Sum_Angle,sum_L,sum_R,Ratio);
		if(Isr_flag_10 == 1)
		{
			ADC_GetValue();
			MPU6050_Refresh_DMP();
			MPU_Get_Gyroscope(&gx, &gy, &gz);
			Get_Ratio();
			
			///////////////////////////////// 直道弯道判别 ///////////////////////////////////////
			if(Ratio >= -0.1 && Ratio <= 0.1) //直线
			{
				Turn_PID.Kp = -15;
				Turn_PID.Kd = -2.6;
				Left_Wheel_PID.Kp = Right_Wheel_PID.Kp = 20;
				Left_Wheel_PID.Ki = Right_Wheel_PID.Ki = 0.5;
				Exp_Speed = 260;
			}
			else   // 拐弯
			{
				Turn_PID.Kp = -138;
				Turn_PID.Kd = -24.3;
				Left_Wheel_PID.Kp = Right_Wheel_PID.Kp = 36;
				Left_Wheel_PID.Ki = Right_Wheel_PID.Ki = 0.9; //i太大会出现矫正滞后，导致车反方向飘逸
				Exp_Speed = 200;
			}
			
			Get_Speed();  //获取车速
			
			///////////////////////////////// 避开路障 //////////////////////////////////////////
			vl53l0x_get_distance(); 
			if (vl53l0x_distance_mm < 650)		//	路障检测
				Barrier_Flag1 = 1;
			Elem_Barrier(gz);
			if(Barrier_Flag4 > 0)
			{
				Ratio = -0.4;
				Barrier_Flag4 -= 1;
			}
			
			////////////////////////////////// 圆环判别 ///////////////////////////////////////////////
			if(ADC_proc[2] > 80 && (ADC_proc[3] > 10 || ADC_proc[1] > 10))	//识别圆环
			{
				if(ADC_proc[3] > ADC_proc[1])  //判断左右
				{
					circle_flag_R = 1;
					Elem_Circle_R((Speed_L + Speed_R)/2,gz);	
				}
				else if(ADC_proc[3] < ADC_proc[1]) 
				{
					circle_flag_L = 1;
					Elem_Circle_L((Speed_L + Speed_R)/2,gz);
				}
			}
			
			///////////////////////////// 转向环计算 //////////////////////////////////////////////////				
			PID_Calculate(&Turn_PID,Ratio*100,gz/100); 
			Limit_Out(&Turn_PID.PID_Out,-2000,5000);
				
			////////////////////////////////上下坡道 ////////////////////////////////////////////////////
			Elem_Up_Down(Pitch,gy);		
				
			///////////////////////////// 特殊元素降速 ///////////////////////////////////////////////////
			if( circle_flag_L == 1 || circle_flag_R == 1 || Barrier_Flag2 == 1 || Barrier_Flag1 == 1)  
				Exp_Speed = 160;
			Exp_Speed_L = Exp_Speed + Turn_PID.PID_Out*0.09;
			Exp_Speed_R = Exp_Speed - Turn_PID.PID_Out*0.09;
			

			PID_Calculate(&Left_Wheel_PID,Exp_Speed_L,Speed_L);//速度环PID计算
			PID_Calculate(&Right_Wheel_PID,Exp_Speed_R,Speed_R);
			
			//////////////////////// // 驶离赛道，停车 /////////////////////////////////////////////////
			if(ADC_proc[2]<5 && Barrier_Executed == 1) 
			{
				Left_Wheel_PID.PID_Out = 0;
				Right_Wheel_PID.PID_Out = 0;
			}
			
//			Left_SetSpeed(Left_Wheel_PID.PID_Out);
//			Right_SetSpeed(Right_Wheel_PID.PID_Out);
			
			Left_SetSpeed(2000);
			Right_SetSpeed(-2000);
			Isr_flag_10 = 0;
		} 
	}
}



