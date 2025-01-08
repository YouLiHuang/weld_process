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

// mode=0
// u8 KEY_Scan(u8 mode)
// {
// 	static u8 key_up = 1;
// 	if (mode)
// 		key_up = 1;
// 	if (key_up && (KEY_PC0 == 0 || KEY_PC1 == 0 || KEY_PC2 == 0 || KEY_PC3 == 0 || KEY_In1 == 0 || KEY_In2 == 0 || KEY_In4 == 0))
// 	{
// 		delay_ms(10);
// 		key_up = 0;
// 		if (KEY_PC0 == 0)
// 			return 1;
// 		else if (KEY_PC1 == 0)
// 			return 2;

// 		else if (KEY_PC2 == 0)
// 			return 3;
// 		else if (KEY_PC3 == 0)
// 			return 4;
// 		else if (KEY_In1 == 0)
// 			return 5;
// 		else if (KEY_In2 == 0)
// 			return 6;
// 		else if (KEY_In3 == 0)
// 			return 7;
// 		else if (KEY_In4 == 0)
// 			return 8;
// 	}
// 	else if (KEY_PC0 == 1 && KEY_PC1 == 1 && KEY_PC2 == 1 && KEY_PC3 == 1 &&
// 			 KEY_In1 == 1 && KEY_In2 == 1 && KEY_In4 == 1)
// 		key_up = 1;
// 	return 0;
// }

// u8 KEY_Scan2(u8 mode)
// {
// 	OS_ERR err;
// 	static u8 key_up2 = 1;
// 	if (mode)
// 		key_up2 = 1;
// 	if (key_up2 && (KEY_Res == 0))
// 	{
// 		OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_PERIODIC, &err);
// 		if (KEY_Res == 1)
// 			return 9;
// 	}
// 	else if (KEY_Res == 0)
// 		key_up2 = 1;
// 	return 0;
// }
