/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-13 10:22:31
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
#include "protect.h"
#include "crc16.h"
#include "includes.h"
#include "PID.h"
#include "timer.h"
#include "delay.h"
#include "touchscreen.h"

#define PROTECT_CHECK 0
extern Page_Param *page_param;
extern Error_ctrl *err_ctrl;
/**
 * @description: create new error handle
 * @param {char} *msg
 * @param {char} *pic
 * @param {err_callback} callback
 * @return {*}
 */
error_handle *new_error_handle(mERROR_TYPE type, const char *pic, err_callback err_callback, err_reset_callback reset_callback)
{
	error_handle *handle = (error_handle *)malloc(sizeof(error_handle));
	if (handle != NULL)
	{
		char *name = (char *)malloc(sizeof(char) * 20);
		strcpy(name, pic);
		handle->pic_name = name;
		handle->state = false;
		handle->err_type = type;
		handle->error_callback = err_callback;
		handle->reset_callback = reset_callback;
		return handle;
	}
	else
		return NULL;
}

/**
 * @description: create new error list controller
 * @return {*}
 */
Error_ctrl *new_error_ctrl(void)
{

	Error_ctrl *ctrl = (Error_ctrl *)malloc(sizeof(Error_ctrl));
	if (ctrl != NULL)
	{
		ctrl->max_len = ERR_LIST_MAX_LEN;
		ctrl->err_list = (error_handle **)malloc(ERR_LIST_MAX_LEN * sizeof(error_handle *)); // 允许最大注册16种错误
		ctrl->error_cnt = 0;
		ctrl->index = 0;
		ctrl->sensor_err_threshold = SENSOR_ERR_THRESHOLD;
		ctrl->temp_over_threshold = OVER_TEMP_THRESHOLD;
		ctrl->temp_low_threshold = LOW_TEMP_THRESHOLD;
		ctrl->sensor_err_cnt = 0;
		ctrl->temp_over_cnt = 0;
		ctrl->temp_low_cnt = 0;
		return ctrl;
	}
	return NULL;
}

/**
 * @description: register error handle to the controller
 * @param {Error_ctrl} *ctrl
 * @param {error_handle} *err_handle
 * @return {*}
 */
bool register_error(Error_ctrl *ctrl, error_handle *err_handle)
{
	if (ctrl == NULL || err_handle == NULL)
		return false;

	if (ctrl->index < ctrl->max_len)
	{
		ctrl->err_list[ctrl->index] = err_handle;
		ctrl->index++;
		ctrl->error_cnt++;
		return true;
	}
	else
		return false;
}

/**
 * @description: Get the corresponding handle based on the specified error type
 * @param {Error_ctrl} *ctrl
 * @param {mERROR_TYPE} type
 * @return {*}
 */
error_handle *err_get_type(Error_ctrl *ctrl, mERROR_TYPE type)
{
	if (ctrl == NULL)
		return NULL;

	for (u8 i = 0; i < ctrl->error_cnt; i++)
	{
		if (ctrl->err_list[i]->err_type == type)
			return ctrl->err_list[i];
	}
	return NULL;
}

bool err_occur(Error_ctrl *ctrl)
{
	if (ctrl == NULL)
		return false;

	for (u8 i = 0; i < ctrl->error_cnt; i++)
	{
		if (true == ctrl->err_list[i]->state)
			return true;
	}
	return false;
}

void err_clear(Error_ctrl *ctrl)
{
	for (u8 i = 0; i < ctrl->error_cnt; i++)
	{
		ctrl->err_list[i]->state = false;
	}
}

void err_cnt_clear(Error_ctrl *ctrl)
{
	ctrl->sensor_err_cnt = 0;
	ctrl->temp_over_cnt = 0;
	ctrl->temp_low_cnt = 0;
}

/**
 * @description: current check IO config
 * @return {*}
 */
void TestCurrent_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// 设置中断优先级分组2
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // 使能 SYSCFG 时钟

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 默认是高电平，上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0); // PB0 连接线 0

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 上升沿还是下降沿到时候需要验证修改
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @description:
 * @return {*}
 */
static void TIM_PROTECT_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 20 - 1; // 20-10us
	TIM_TimeBaseStructure.TIM_Period = 84 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	// 使能定时器7
	TIM_Cmd(TIM7, DISABLE);

	// 开启定时器7中断
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}

/**
 * @description: sensor protect config
 * @return {*}
 */
static void EXTI_PROTECT_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // 使能 SYSCFG 时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	   // 普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	   // 下拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// 连线
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource10);

	// 配置 EXTI_Line9，10
	EXTI_InitStructure.EXTI_Line = EXTI_Line9 | EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // 低转高电平触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

static void PROTECT_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	// IO10-15中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// IO5-9中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TIM7
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void PROTECT_Init(void)
{
	TIM_PROTECT_Mode_Config();
	EXTI_PROTECT_Config();
	PROTECT_NVIC_Config();
}

extern OS_SEM ERROR_HANDLE_SEM; // 错误信号

/**
 * @description: Interrupt callback of timer 7
 * @return {*}
 */
void TIM_P_Protect_IT(void)
{
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif

	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) // 溢出中断
	{
		__NOP();
	}

	TIM_ClearITPendingBit(TIM7, TIM_IT_Update); // 清除中断标志位
#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

/**
 * @description: External interrupt triggered by runaway current
 * @return {*}
 */
void EXTI0_Currunt_Protect_IT(void)
{
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		err_get_type(err_ctrl, CURRENT_OUT_OT_CTRL)->state = true;
		OS_ERR err;
		/*唤醒错误处理线程*/
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

/**
 * @description: External interrupt triggered by over temperature
 * @return {*}
 */
void EXTI9_Temperature_Protect_IT(void)
{
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif
	if (EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		err_get_type(err_ctrl, MCU_OVER_HEAT)->state = true;
		OS_ERR err;
		/*唤醒错误处理线程*/
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
	EXTI_ClearITPendingBit(EXTI_Line9);
#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

/**
 * @description: Transformer overheating interrupt
 * @return {*}
 */
void EXTI10_Temperature_Protect_IT(void)
{
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif
	if (EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		err_get_type(err_ctrl, TRANSFORMER_OVER_HEAT)->state = true;
		OS_ERR err;
		/*唤醒错误处理线程*/
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
	EXTI_ClearITPendingBit(EXTI_Line10);
#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}
