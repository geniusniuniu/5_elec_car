#include "headfile.h"
#include "Motor.h"
#include "ADC.h"
#include "MPU6050.h"
#include "Element.h"
#include "ADC.h"
#include "math.h"
#include "TOF.h"
#include "Buzzer.h"
#include "Key.h"
#include "ui.h"

extern uint16 vl53l0x_distance_mm;
extern uint8 vl53l0x_finsh_flag;

short gx, gy, gz;
char Isr_flag_10 = 0; 
char KeyValue = 0;	
char Speed_Delay = 100;
char flag_stop = 0;

float Exp_Speed_L = 0;
float Exp_Speed_R = 0;
float Exp_Speed = 0;
float Adjust_Val = -200;
float x = 0;

void Init_all(void);
void Get_Ratio(void);

void main(void)	
{
	Init_all();
	EnableGlobalIRQ();	
	while(1)
	{		
	    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Speed_L,Speed_R,Left_Wheel.out,Right_Wheel.out,Exp_Speed_L,Exp_Speed_R,Turn_PID.PID_Out);
//		printf("%d,%.2f\r\n",vl53l0x_distance_mm,0.0);
/******************************************** 按键读值**********************************************************************/ 	
		ui_show();
		KeyValue = GetKey_Value(0);
		if 		(KeyValue == KEY2_PRES) 	{page++; if(page >= 3)  page = 3;oled_all_static_state();}		
		else if (KeyValue == KEY3_PRES) 	{page--; if(page <= 0)  page = 0;oled_all_static_state();}			
		else if (KeyValue == KEY0_PRES) 	Adjust_Val += 1;
		else if (KeyValue == KEY1_PRES) 	Adjust_Val -= 1;
	
///******************************************** 类似中断服务处理 **************************************************************/ 
		if(Isr_flag_10 == 1)  
		{
			ADC_GetValue();						//获取电感值
			MPU6050_Refresh_DMP();				//读取角度值
			MPU_Get_Gyroscope(&gx, &gy, &gz);	//读取角速度
			Get_Ratio();						//计算偏差值
			vl53l0x_get_distance();				//测距传感器读值
			
		/************************************************ 直道弯道判别 ********************************************/ 
			
//			#if TRACE_METHOD2  //向量法
//				if(Ratio >= -0.1 && Ratio <= 0.1) //直线
//				{
//					Turn_PID.Kp = -15;
//					Turn_PID.Kd = -2.6;
//					Left_Wheel_PID.Kp = Right_Wheel_PID.Kp = 20;
//					Left_Wheel_PID.Ki = Right_Wheel_PID.Ki = 0.55;
//					Exp_Speed = 300;
//				}
//				else   // 拐弯    
//				{
//					Turn_PID.Kp = -200;  // -180
//					Turn_PID.Kd = -35;  // -45
//					Left_Wheel_PID.Kp = 45;  // 36
//					Left_Wheel_PID.Ki = 0.9; //0.8
//					Right_Wheel_PID.Kp = 45;
//					Right_Wheel_PID.Ki = 0.9;
//					Exp_Speed = 260;
//				}
				Exp_Speed = 240;
				
//			#elif TRACE_METHOD1  //单向巡线
//				if(Ratio > -0.16 && Ratio < 0.16) //直线
//				{
//					Turn_PID.Kp = -20;
//					Turn_PID.Kd = -3.5;
//					Left_Wheel_PID.Kp  = 20;
//					Left_Wheel_PID.Ki  = 0.6;
//					
//					Right_Wheel_PID.Kp = 20;
//					Right_Wheel_PID.Ki = 0.6; 
//					Exp_Speed = 240;
//				} 
//				else   // 拐弯
//				{
//					Turn_PID.Kp = -180 ;
//					Turn_PID.Kd = -32;
//					Left_Wheel_PID.Kp  = 26;
//					Left_Wheel_PID.Ki  = 1.26;
//					
//					Right_Wheel_PID.Kp = 26;
//					Right_Wheel_PID.Ki = 1.26; //i太大会出现矫正滞后，导致车反方向飘逸
//					Exp_Speed = 200;
//				}	
					
//			#endif	
				
		/************************************************ 避开路障 ***********************************************/ 	
				
//			if(Barrier_Executed == 0)
//			{	
//				if (vl53l0x_finsh_flag == 1 && vl53l0x_distance_mm < 800)		//	检测到路障
//				{ 
//					x10_ms = 13; 
//					Barrier_Flag1 = 1;
//				}
//				Elem_Barrier(gz);
//			}
//			#if TRACE_METHOD2  //弥补向量法检测缺陷导致车身反偏
//				if(Barrier_Delay > 0)
//				{
//					Ratio = -0.2;
//					Barrier_Delay -= 1;
//				}	
//			#endif

//		/************************************************ 圆环判别 ***********************************************/ 
			
			if(ADC_proc[2] > 66 || ADC_proc[0] > 64 || ADC_proc[4] > 64) 
			{
				Circle_Flag1 = 1;   									//识别到圆环标志位
				x10_ms = 13;
			}
			if(vl53l0x_finsh_flag == 1 && vl53l0x_distance_mm < 400)	//一次测距完成，区分坡道
				Circle_Delay1 = 150;		
			if(Circle_Delay1 > 0)										//检测到坡道，清零环岛标志位，并延时1.5秒
			{
				Circle_Flag1 = 0;
				Circle_Delay1--;
			}
			//Elem_Circle((Speed_L+Speed_R)/2,gz);	
			
		/************************************************ 转向环计算 **********************************************/ 	
			
			 
			Limit_Out(&Turn_PID.PID_Out,-5000,5000);
			
		/************************************************ 上下坡道 ************************************************/ 
			Elem_Up_Down(Pitch,gy);		
		
		/************************************************ 特殊元素降速 ********************************************/ 
//			if(Barrier_Flag2 == 1 || Barrier_Flag1 == 1)  
//				Exp_Speed = 280;
			
			
			if(Ratio > 0)	
			{
				Exp_Speed_L = Exp_Speed + Turn_PID.PID_Out*0.10;
				Exp_Speed_R = Exp_Speed;
			}
			else
			{
				Exp_Speed_L = Exp_Speed;
				Exp_Speed_R = Exp_Speed - Turn_PID.PID_Out*0.10;
			}
			
			
	   /********************************************* 驶离赛道，撞到障碍，停车 *********************************************/

			if((--Speed_Delay) == 0 && abs(Speed_L) < 50 && abs(Speed_R) < 50)
				x = 1;
			if(vl53l0x_distance_mm < 190 || x == 1) 
			{
				flag_stop = 1;
			}
	   /********************************************* 设置左右PWM ************************************************/ 	
			
//			Motor_Test(2000);
			Isr_flag_10 = 0;
		} 
	}
}














void Init_all(void)
{
	WTST = 0;						 
	DisableGlobalIRQ();				
	sys_clk = 35000000;				
	board_init();					

////编码器方向引脚初始化
	gpio_mode(P5_3, GPIO);          
	gpio_mode(P3_5, GPIO);	
	
////按键引脚初始化
	Key_Init();
	
////测距模块初始化
	//gpio_mode(P3_2, GPIO);
   vl53l0x_init();
	
////OLED初始化
	ui_init();					
	
////MPU6050初始化
	MPU6050_DMP_Init();	
//	
////定时器初始化
	pit_timer_ms(TIM_4, 10);		//10ms定时器
//	
////编码器初始化
	ctimer_count_init(CTIM0_P34);	//编码器1计数
	ctimer_count_init(CTIM3_P04);	//编码器2计数
	
////电机初始化
	Motor_Init();
	
////蜂鸣器初始化
	 Buzzer_Init();
	
////初始化所有AD引脚
	ADC_InitAll(); 
	
////pid初始化  PID_Init(结构体, KP, KI, KD, 输出限幅，积分限幅)

//	PID_Init(&Left_Wheel , 20, 0.5, 0, 9000, 2000);
//	PID_Init(&Right_Wheel, 20, 0.5, 0, 9000, 2000);
	Left_Wheel = PID_Incremental_Init(25, 8, 2,9000);
	Right_Wheel = PID_Incremental_Init(25, 8, 2,9000);

	PID_Init(&Turn_PID , -100, 0, -50 ,10000, 300);
	
} 

