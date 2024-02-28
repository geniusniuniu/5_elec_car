/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		����ʾ����Э��
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		    
 ********************************************************************************************************************/

#ifndef _SEEKFREE_AT24C02_H
#define _SEEKFREE_AT24C02_H

#include "common.h"



									//AT24C02ģ��֧��256��8λ���ݴ洢
									//��ģ��ʹ�õ���IIC���߿���
#define AT24C02_DEV_ADDR 0xA0>>1	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
									//AT24C02�ĵ�ַ�ǿɱ��ַ��A0 = 0 A1 = 0 A2 = 0��ʱ��,
									//�豸��ַλA0�������Ҫ�޸��豸��ַ����鿴AT24C02�ֲ�����޸�

void at24c02_write_byte(uint8 byte_reg, uint8 dat);
uint8 at24c02_read_byte(uint8 byte_reg);
void at24c02_write_bytes(uint8 byte_reg, uint8 *dat_add, uint8 num);
void at24c02_read_bytes(uint8 byte_reg, uint8 *dat_add, uint8 num);

#endif 