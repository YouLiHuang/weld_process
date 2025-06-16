/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-18 19:08:13
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-16 15:26:16
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "io_ctrl.h"
#include "delay.h"
#include "timer.h"
#include "includes.h"
#include "user_config.h"
#include "protect.h"

START_TYPE start_type = START_IDEAL;
extern OS_SEM WELD_START_SEM;


void TIM6_INIT(uint16_t time_outms)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	uint16_t preriod = time_outms * 1000;
	if (preriod > 0xff)
		preriod = 0xff;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = preriod;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM6, ENABLE);

	/*NVIC*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, DISABLE);
}

/**
 * @description: 5ms time out irq
 * @return {*}
 */
void TIM6_irq(void)
{

	OS_ERR err;
#if SYSTEM_SUPPORT_OS
	OSIntEnter();
#endif

	if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TIM_Cmd(TIM6, DISABLE);

		/*check which key is pressed*/
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == RESET)
		{
			/*notify main task to start weld*/
			start_type = KEY0;
			OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
		}
		if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == RESET)
		{
			/*notify main task to start weld*/
			start_type = KEY1;
			OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
		}
	}

#if SYSTEM_SUPPORT_OS
	OSIntExit();
#endif
}

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

/**
 * @description: start signal exit irq
 * @return {*}
 */
void Start_signal_irq(void)
{

#if SYSTEM_SUPPORT_OS
	OSIntEnter();
#endif
	/*key1*/
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		/*software delay*/
		TIM_Cmd(TIM6, ENABLE);
	}
	/*key2*/
	else if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
		/*software delay*/
		TIM_Cmd(TIM6, ENABLE);
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
void INPUT_IO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*defalut high level/low level trigger*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = CURRENT_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(CURRENT_OVERLOAD_GPIO, &GPIO_InitStructure);
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
uint8_t RLY_INPUT_SCAN()
{
	OS_ERR err;
	if (RLY_START0 == 0 || RLY_START1 == 0 || RLY_START2 == 0 || RLY_START3 == 0)
	{
		OSTimeDly(20, OS_OPT_TIME_DLY, &err);
		if (RLY_START0 == 0)
			return RLY_START0_ACTIVE;
		else if (RLY_START1 == 0)
			return RLY_START1_ACTIVE;
		else if (RLY_START2 == 0)
			return RLY_START2_ACTIVE;
		else if (RLY_START3 == 0)
			return RLY_START3_ACTIVE;
		else if (RLY_RESET == 0)
			return RLY_RESET_ACTIVE;
		else if (RLY_TEMP == 0)
			return RLY_TEMP_ACTIVE;
		else if (RLY_RADIATOR == 0)
			return RLY_RADIATOR_ACTIVE;
		else if (RLY_WATER == 0)
			return RLY_WATER_ACTIVE;
	}
	return RLY_NOINPUT;
}
