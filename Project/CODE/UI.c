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
extern float Exp_Speed;
extern float Exp_Speed_L;
extern float Exp_Speed_R;

uint8 page=0;

float value[4][7];
char value_name[4][7][7];
///////////////////////////////////////////////////待显示参数




void value_name_init(void)//将name改为想要的参数名，但“ ”中的总长度不要变
{
	//page=0
	strcpy(value_name[0][0],"ADC0  "); //NAME_1表示显示的是第一页
	strcpy(value_name[0][1],"ADC1  ");
	strcpy(value_name[0][2],"ADC2  ");
	strcpy(value_name[0][3],"ADC3  ");


	//page=1	
	strcpy(value_name[1][0],"ADC4  ");
	strcpy(value_name[1][1],"Ratio ");
	strcpy(value_name[1][2],"L_OUT ");
	strcpy(value_name[1][3],"R_OUT ");

	
	//page=2
	strcpy(value_name[2][0],"Exp_L ");
	strcpy(value_name[2][1],"Exp_R ");
	strcpy(value_name[2][2],"Bar_Ex");
	strcpy(value_name[2][3],"Vl_Dis");
	
	//page=3
	strcpy(value_name[3][0],"Sum_An");
	strcpy(value_name[3][1],"Bar1  ");
	strcpy(value_name[3][2],"Bar2  ");
	strcpy(value_name[3][3],"Bar3  ");

}

void refresh_value()//value=填入变量名
{
	//page=0
    value[0][0]= ADC_proc[0];//Diff;//ADC_proc[0];
	value[0][1]= ADC_proc[1];
	value[0][2]= ADC_proc[3];
	value[0][3]= ADC_proc[4];
	
	//page=1
	value[1][0]= ADC_proc[2];
	value[1][1]= Ratio;
	value[1][2]= Left_Wheel.out;
	value[1][3]= Right_Wheel.out;
	
	//page=2
	value[2][0]= Exp_Speed_L;
	value[2][1]= Exp_Speed_R;
	value[2][2]= Barrier_Executed;
	value[2][3]= vl53l0x_distance_mm;
	
	//page=3
	value[3][0]= Sum_Angle;
	value[3][1]= Barrier_Flag1;//Sum_Dis1;
	value[3][2]= Barrier_Flag2;//Sum_Dis2;
	value[3][3]= Barrier_Flag3;//Sum_Angle_C1;
		
}
//                         oled显示函数
///////////////////////////////////////////////////////////////////////

void oled_change_value(int pages,int y,float value_new)
{
    value[pages][y]=value_new;
    oled_printf_float(55,y,value[pages][y],4,4);
}

void oled_all_static_state()  //静态显示部分
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



