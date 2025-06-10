/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-18 19:08:13
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-10 11:42:02
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "io_ctrl.h"
#include "delay.h"
#include "includes.h"
#include "user_config.h"

extern OS_SEM WELD_START_SEM;
extern OS_Q key_msg;
START_TYPE start_type;

/**
 * @description: PC0-PC3:START SIGNAL
 * @return {*}
 */
void START_IO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
#if START_IO_ENABLE
	GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_2 | GPIO_Pin_3;
#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
#if START_IO_ENABLE
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);
#endif

	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
#if START_IO_ENABLE
	EXTI_InitStructure.EXTI_Line |= EXTI_Line2 | EXTI_Line3;
#endif
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
#if START_IO_ENABLE
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

void Start_signal_irq(void)
{
	OS_ERR err;
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif
	/*key1 key2*/
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		/*software delay*/
		for (int i = 0; i < 1000; i++)
			;
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == RESET)
		{
			/*notify main task to start weld*/
			start_type = KEY0;
			OSQPost(&key_msg, &start_type, sizeof(start_type), OS_OPT_POST_ALL | OS_OPT_POST_FIFO, &err);
		}
	}
	else if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
		/*software delay*/
		for (int i = 0; i < 1000; i++)
			;
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == RESET)
		{
			/*notify main task to start weld*/
			start_type = KEY1;
			OSQPost(&key_msg, &start_type, sizeof(start_type), OS_OPT_POST_ALL | OS_OPT_POST_FIFO, &err);
		}
	}
#if START_IO_ENABLE
	/*reserve io*/
	else if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);
		/*software delay*/
		for (int i = 0; i < 1000; i++)
			;
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == RESET)
			OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
	}
	else if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
		/*software delay*/
		for (int i = 0; i < 1000; i++)
			;
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) == RESET)
			OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
	}
#endif

#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

/**
 * @description:  PA3 PC7:reserve PC8：reset PC9:TEMP1 PC10:TEMP2 PC11:water
 * @return {*}
 */
void IO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 * @description:
 * @return {*}
 */
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

/**
 * @description:
 * @return {*}
 */
uint8_t new_key_scan()
{
	if (KEY_PC0 == 0 || KEY_PC1 == 0 || KEY_PC2 == 0 || KEY_PC3 == 0 || KEY_In4 == 0)
	{
		delay_ms(10);
		if (KEY_PC0 == 0)
			return KEY_PC0_PRES;
		else if (KEY_PC1 == 0)
			return KEY_PC1_PRES;
		else if (KEY_PC2 == 0)
			return KEY_PC2_PRES;
		else if (KEY_PC3 == 0)
			return KEY_PC2_PRES;
		else if (KEY_In3 == 0)
			return KEY_In3_PRES;
		else if (KEY_In4 == 0)
			return KEY_In4_PRES;
		else if (KEY_Res)
			return KEY_Res_PRES;
		else
			return 0;
	}
	return 0;
}
