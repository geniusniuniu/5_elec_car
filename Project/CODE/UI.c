#include "ui.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "headfile.h"
#include "ADC.h"
#include "Motor.h"
#include "Element.h"
#include "MPU6050.h"

extern uint16 vl53l0x_distance_mm;
extern float Ratio;
extern float Ratio_Mid;
extern float Exp_Speed;
extern float sum;

uint8 page=0;
float value[4][7];
char value_name[4][7][7];
///////////////////////////////////////////////////����ʾ����




void value_name_init(void)//��name��Ϊ��Ҫ�Ĳ����������� ���е��ܳ��Ȳ�Ҫ��
{
	//page=0
	strcpy(value_name[0][0],"ADC0_1"); //NAME_1��ʾ��ʾ���ǵ�һҳ
	strcpy(value_name[0][1],"ADC1  ");
	strcpy(value_name[0][2],"ADC2  ");
	strcpy(value_name[0][3],"ADC3  ");


	//page=1	
	strcpy(value_name[1][0],"ADC4_2");
	strcpy(value_name[1][1],"T_Kp  ");
	strcpy(value_name[1][2],"T_Kd  ");
	strcpy(value_name[1][3],"Exp_Sp");

	
	//page=2
	strcpy(value_name[2][0],"Sp_Kp3");
	strcpy(value_name[2][1],"Sp_Ki ");
	strcpy(value_name[2][2],"Lpwm  ");
	strcpy(value_name[2][3],"Rpwm  ");
	
	//page=3
	strcpy(value_name[3][0],"Vl_Dis");
	strcpy(value_name[3][1],"Ratio ");
	strcpy(value_name[3][2],"Ra_Mid");
	strcpy(value_name[3][3],"Ra_Sum");

}

void refresh_value()//value=���������
{
	//page=0
    value[0][0]= ADC_proc[0];
	value[0][1]= ADC_proc[1];
	value[0][2]= ADC_proc[2];
	value[0][3]= ADC_proc[3];
	
	//page=1
	value[1][0]= ADC_proc[4];
	value[1][1]= Turn_PID.Kp;
	value[1][2]= Turn_PID.Kd;
	value[1][3]= Exp_Speed;
	
	//page=2
	value[2][0]= Left_Wheel_PID.Kp;
	value[2][1]= Left_Wheel_PID.Ki;
	value[2][2]= Left_Wheel_PID.PID_Out;
	value[2][3]= Right_Wheel_PID.PID_Out;
	
	//page=3
	value[3][0]= vl53l0x_distance_mm;
	value[3][1]= Ratio;
	value[3][2]= Ratio_Mid;
	value[3][3]= sum;
		
}
//                         oled��ʾ����
///////////////////////////////////////////////////////////////////////

void oled_change_value(int pages,int y,float value_new)
{
    value[pages][y]=value_new;
    oled_printf_float(55,y,value[pages][y],4,4);
}

void oled_all_static_state()  //��̬��ʾ����
{
    uint8 i,j;
//    oled_p8x16str(0, 0,"page");
//    oled_p8x16str(40, 0,"=");
//    oled_int16(60,0,Mode_Num);

//    oled_p8x16str(70, 0,"mode");
//    oled_int16(120, 0,page);
	for(j=0;j<=3;j++)
	{
		i = j*2;
		oled_p8x16str(0 ,i,value_name[page][j]);
		oled_p8x16str(50,i,"=");
	}
}


void oled_show(void)
{
    uint8 i,j;
	refresh_value();
	for(j=0;j<=3;j++)
	{
		i = j*2;
		oled_printf_float(55,i,value[page][j],5,2);
	}
}

//////////////////////////////////////////////////////////////
void ui_init(void)
{
	oled_init();
	value_name_init();
	oled_all_static_state();
}

void ui_show(void)
{
	refresh_value();
	oled_show();
}



