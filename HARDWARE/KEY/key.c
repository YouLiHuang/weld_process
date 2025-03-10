/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-18 19:08:13
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-03-10 10:26:56
 * @Description: 
 * 
 * Copyright (c) 2025 by huangyouli, All Rights Reserved. 
 */
#include "key.h"
#include "delay.h"

void KEYin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void OUT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
								  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

u8 new_key_scan()
{
	if (KEY_PC0 == 0 || KEY_PC1 == 0 || KEY_PC2 == 0 || KEY_PC3 == 0 || KEY_In4 == 0)
	{
		delay_ms(10);
		if (KEY_PC0 == 0)
			return 1;
		else if (KEY_PC1 == 0)
			return 2;
		else if (KEY_PC2 == 0)
			return 3;
		else if (KEY_PC3 == 0)
			return 4;
		else if (KEY_In3 == 0)
			return 7;
		else if (KEY_In4 == 0)
			return 8;
		else
			return 0;
	}
	return 0;
}


