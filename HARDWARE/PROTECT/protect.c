/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-10 10:00:03
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
#include "protect.h"

extern Error_ctrl *err_ctrl;
extern OS_SEM ERROR_HANDLE_SEM;
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

		/*err cnt*/
		ctrl->sensor_err_threshold = SENSOR_ERR_THRESHOLD;
		ctrl->temp_over_threshold = OVER_TEMP_THRESHOLD;
		ctrl->temp_low_threshold = LOW_TEMP_THRESHOLD;
		ctrl->Reverse_connection_threshold = REVERSE_ERR_THRESHOLD;

		ctrl->sensor_err_cnt = 0;
		ctrl->temp_over_cnt = 0;
		ctrl->temp_low_cnt = 0;
		ctrl->Reverse_connection_cnt = 0;
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

/*--------------------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------bsp init--------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------------------------*/

// PC9-整流管温度
// PC10-散热器温度
// PB0-过流保护

/**
 * @description: current check IO config
 * @return {*}
 */
void Current_Check_IO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// 默认高电平，低电平则触发过流保护
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = CURRENT_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(CURRENT_OVERLOAD_GPIO, &GPIO_InitStructure);
}

void Temp_Protect_IO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// 默认高电平，低电平则触发过温保护
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = RECTIFICATION_PIN | RADIATOR_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(TEMP_OVERLOAD_GPIO, &GPIO_InitStructure);
}



/**
 * @description: External interrupt triggered by over temperature
 * @return {*}
 */
void EXTI9_Temperature_Protect_IT(void)
{
#if SYSTEM_SUPPORT_OS
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
	OSIntExit();
#endif
}

/**
 * @description: Transformer overheating interrupt
 * @return {*}
 */
void EXTI10_Temperature_Protect_IT(void)
{
#if SYSTEM_SUPPORT_OS
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
	OSIntExit();
#endif
}
