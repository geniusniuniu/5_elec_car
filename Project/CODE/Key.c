#include <string.h>
#include <stdio.h>
#include "common.h"
#include "zf_gpio.h"
#include <STC32Gxx.H>
#include "Key.h"
#include "zf_delay.h"

void Key_Init(void)
{
	//������ʼ��
	gpio_mode(P7_0, GPIO); //KEY1
	gpio_mode(P7_1, GPIO); //KEY2
}

//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//����ֵ��
//0��û���κΰ�������
//KEY0_PRES��KEY0����
//KEY1_PRES��KEY1����
//WKUP_PRES��WK_UP���� 
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1!!
uint8 GetKey_Val(uint8 mode)
{ 
	static uint8 key_up=1;//�����ɿ���־
	if(mode)
		key_up=1;  //֧������		  
	if(key_up&&(P70==0||P71==0))  //ע��˺�������Ӧ���ȼ�,KEY0>KEY1!!
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(P70==0)
			return KEY0_PRES; //1
		else if(P71==0)
			return KEY1_PRES; //2
	}
	else if(P70==1&&P71==1)
		key_up=1; 	     
	return 0;// �ް�������
}
