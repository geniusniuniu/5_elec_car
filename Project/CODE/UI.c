#include "ui.h"
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "headfile.h"
#include "ADC.h"
#include "Motor.h"

float value[4][7];
char value_name[4][7][6];

uint8 page=0;

///////////////////////////////////////////////////待显示参数




void value_name_init(void)//将name改为想要的参数名，但“ ”中的总长度不要变
{
	//page=0
	strcpy(value_name[0][0],"ADC0 ");
	strcpy(value_name[0][1],"ADC1 ");
	strcpy(value_name[0][2],"ADC2 ");
	strcpy(value_name[0][3],"ADC3 ");


	//page=1	
	strcpy(value_name[1][0],"nam4 ");
	strcpy(value_name[1][1],"nam5 ");
	strcpy(value_name[1][2],"nam6 ");
	strcpy(value_name[1][3],"nam7 ");

	
	//page=2
	strcpy(value_name[2][0],"nam8 ");
	strcpy(value_name[2][1],"nam9 ");
	strcpy(value_name[2][2],"na10 ");
	strcpy(value_name[2][3],"na11 ");

}

void refresh_value()//value=填入变量名
{
	//page=0
    value[0][0]= 0;
	value[0][1]= 1;
	value[0][2]= 2;
	value[0][3]= 3;
	
	//page=1
	value[1][0]= 4;
	value[1][1]= 5;
	value[1][2]= 6;
	value[1][3]= 7;
	
	//page=2
	value[2][0]= 8;
	value[2][1]= 9;
	value[2][2]= 0;
	value[2][3]= 1;
		
}
//                         oled显示函数
///////////////////////////////////////////////////////////////////////

void oled_change_value(int pages,int y,float value_new)
{
    value[pages][y]=value_new;
    oled_printf_float(40,y,value[pages][y],4,4);
}

void oled_all_static_state()  //静态显示部分
{
    uint8 j;
//    oled_p8x16str(0, 0,"page");
//    oled_p8x16str(40, 0,"=");
//    oled_int16(60,0,Mode_Num);

//    oled_p8x16str(70, 0,"mode");
//    oled_int16(120, 0,page);
	for(j=0;j<=6;j+=2)
	{
		oled_p8x16str(0 ,j,value_name[page][j/2]);
		oled_p8x16str(40,j,"=");
	}
}


void oled_show(void)
{
    uint8 j;
	refresh_value();
	for(j=0;j<=6;j+=2)
	{
		oled_printf_float(45,j,value[page][j/2],5,3);
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



