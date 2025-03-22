/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-15 19:17:48
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-03-22 09:52:56
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "timer.h"

#include "welding_process.h"
#include "stm32f4xx.h"
#include "includes.h"
#include "adc.h"
#include "PID.h"
#include "usart.h"
#include "protect.h"
#include "Kalman.h"
#include "crc16.h"
#include "tempcomp.h"
#include "touchscreen.h"

#define TIME_CHECH 0 // 超时监测
#define PROTECT_ON 1 // 保护开关

/*实时控制*/
extern weld_ctrl *weld_controller;
double fitting_curves[3] = {0.0002, -0.23, 76}; // 二次曲线拟合系数

/*温度补偿部分*/
extern Kalman kfp;
extern dynamical_comp dynam_comp;
last_temp_sotre lasttemp;
uint16_t kalman_comp_temp = 0;

/*错误处理*/
extern Error_ctrl *err_ctrl;	// 错误注册表
extern OS_SEM ERROR_HANDLE_SEM; // 错误信号

/*绘图专用数据*/
extern Temp_draw_ctrl *temp_draw_ctrl;
uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN] = {0}; // Temperature preservation buffer

/*热电偶*/
extern Thermocouple *current_Thermocouple;

/**
 * @description: create an new controller
 * @param {uint16_t} *buf
 * @param {uint16_t} len1
 * @param {uint16_t} len2
 * @param {uint16_t} len3
 * @return {*}
 */
Temp_draw_ctrl *new_temp_draw_ctrl(uint16_t *buf, uint16_t len1, uint16_t len2, uint16_t len3)
{
	if (len1 + len2 + len3 > TEMP_BUF_MAX_LEN)
		return NULL;
	Temp_draw_ctrl *ctrl = (Temp_draw_ctrl *)malloc(sizeof(Temp_draw_ctrl));
	if (ctrl == NULL || buf == NULL)
		return NULL;

	ctrl->sample_freq = 1;
	ctrl->current_index = 0;
	ctrl->buf_len_max = TEMP_BUF_MAX_LEN;
	ctrl->temp_buf = buf;

	ctrl->first_step_index_start = 0;
	ctrl->first_step_index_end = 0;

	ctrl->second_step_index_start = 0;
	ctrl->second_step_index_end = 0;

	ctrl->third_step_index_start = 0;
	ctrl->third_step_index_end = 0;
	ctrl->tick_record = 0;

	return ctrl;
}

/**
 * @description: reset a controller
 * @param {Temp_draw_ctrl} *ctrl
 * @return {*}
 */
void reset_temp_draw_ctrl(Temp_draw_ctrl *ctrl, const uint16_t welding_time[])
{
	uint16_t total_time_length = welding_time[0] + welding_time[1] + welding_time[2] + welding_time[3] + welding_time[4];
	if (total_time_length > ctrl->buf_len_max)
		ctrl->sample_freq = (uint8_t)(total_time_length / ctrl->buf_len_max) + 1;
	else
		ctrl->sample_freq = 1;

	ctrl->current_index = 0;
	ctrl->first_step_index_start = 0;
	ctrl->first_step_index_end = 0;

	ctrl->second_step_index_start = 0;
	ctrl->second_step_index_end = 0;

	ctrl->third_step_index_start = 0;
	ctrl->third_step_index_end = 0;

	ctrl->second_step_stable_index = 0;

	ctrl->tick_record = 0;
	for (uint16_t i = 0; i < ctrl->buf_len_max; i++)
	{
		ctrl->temp_buf[i] = 0;
	}
}

/**
 * @description: Dynamically adjust pid parameters according to set values
 * @param {void} *controller
 * @param {double} *fitting_curves
 * @param {uint16_t} setting
 * @return {*}
 */
void pid_param_dynamic_reload(void *controller, double *fitting_curves, uint16_t setting)
{

#if PID_DEBUG != 1
	weld_ctrl *ctrl = (weld_ctrl *)controller;
	ctrl->pid_ctrl->stable_flag = false;
	uint8_t new_kd = 0;

	switch (ctrl->state)
	{
	case FIRST_STATE:
		new_kd = fitting_curves[0] * setting * setting +
				 fitting_curves[1] * setting +
				 fitting_curves[2];

		if (new_kd > 25)
			new_kd = 25;
		if (new_kd <= 12)
			new_kd = 12;

		ctrl->pid_ctrl->kp = 15;
		ctrl->pid_ctrl->ki = 0.028;
		ctrl->pid_ctrl->kd = new_kd;
		/*稳态标志复位*/
		weld_controller->pid_ctrl->stable_flag = false;

		break;

	case SECOND_STATE:

		if (ctrl->weld_time[1] == 0)
		{
			new_kd = fitting_curves[0] * setting * setting +
					 fitting_curves[1] * setting +
					 fitting_curves[2];
		}
		else
		{
			uint16_t delta_temp = ctrl->weld_temp[1] - ctrl->second_step_start_temp;

			new_kd = fitting_curves[0] * delta_temp * delta_temp +
					 fitting_curves[1] * delta_temp +
					 fitting_curves[2];
		}
		if (new_kd > 25)
			new_kd = 25;
		if (new_kd <= 12)
			new_kd = 12;

		ctrl->pid_ctrl->kp = 15;
		ctrl->pid_ctrl->ki = 0.028;
		ctrl->pid_ctrl->kd = new_kd;
		/*稳态标志复位*/
		weld_controller->pid_ctrl->stable_flag = false;

		break;
	}

#endif
}

/**
 * @description: Timer 3 configuration, welding cycle time recording, 1ms interrupt
 * @return {*}
 */
void TIM3_Int_Init(void)
{
	uint16_t arr = 2000 - 1;
	uint16_t psc = 84 - 1;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	TIM_Cmd(TIM3, DISABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @description:  Timer initialization, clock frequency 168MHz, tim3 is PID control timer, record welding process temperature (1ms interrupt)
 * @return {*}
 */
void TIM5_Int_Init(void)
{
	uint16_t arr = 2000 - 1;
	uint16_t psc = 84 - 1;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

	TIM_Cmd(TIM5, DISABLE);
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static volatile uint16_t current_temp_comp = 0;

/**
 * @description: Timer 5 interrupt function - real-time closed-loop control of welding
 * @return {*}
 */
void TIM5_IRQHandler(void)
{

#if SYSTEM_SUPPORT_OS
	OSIntEnter();
#endif

	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		/*feedback*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple); // 原始温度数据实时温度
#if KALMAN_FILTER == 1
		uint16_t kalman_filter_temp = KalmanFilter(&kfp, weld_controller->realtime_temp); // 卡尔曼滤波
#if COMPENSATION == 1
		slid_windows(&lasttemp, kalman_filter_temp);				// 滑动窗口
		kalman_comp_temp = dynamic_temp_comp(lasttemp, dynam_comp); // 动态补偿
		current_temp_comp = kalman_comp_temp;						// 获取当前温度估计值
#else
		current_temp_comp = kalman_filter_temp;
#endif
#else
		kalman_comp_temp = weld_controller->realtime_temp;
		current_temp_comp = weld_controller->realtime_temp;
#endif

		/*real-time control*/
		switch (weld_controller->state)
		{
		case PRE_STATE:
			weld_controller->step_time_tick++;
			break;
			/*--------------------------------------------------------------------first step----------------------------------------------------------------------*/
		case FIRST_STATE:
			/*Time updates*/
			weld_controller->step_time_tick++;

			/*Detect whether the target temperature is approaching*/
			if (current_temp_comp >= STABLE_THRESHOLD * weld_controller->weld_temp[0])
			{
				weld_controller->Switch_Control = true;
			}
			/*open loop control*/
			if (weld_controller->Switch_Control == false)
			{
				weld_controller->Duty_Cycle += BASIC_STEP * (1 + weld_controller->temp_gain1);
			}
			/*Closed-loop control*/
			else
			{

				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[0] + STABLE_ERR,
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}
#if 0
			if (current_temp_comp >= weld_controller->first_step_turn && weld_controller->pid_ctrl->stable_flag == false)
			{
				/*到达刹车点，转阶段*/
				weld_controller->pid_ctrl->stable_flag = true;
			}

			if (weld_controller->pid_ctrl->stable_flag == false)
			{
				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->first_step_set + STABLE_ERR,
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}
			else
			{
				// 第二个设定点
				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[0] + STABLE_ERR,
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}
#endif

			/*2、执行*/
			TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
			TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

			break;

			/*--------------------------------------------------------------------second step----------------------------------------------------------------------*/
		case SECOND_STATE:
			/*Time updates*/
			weld_controller->step_time_tick++;

			/*Detect whether the target temperature is approaching*/
			if (current_temp_comp >= STABLE_THRESHOLD * weld_controller->weld_temp[1])
			{
				weld_controller->Switch_Control = true;
			}
			/*open loop control*/
			if (weld_controller->Switch_Control == false)
			{
				weld_controller->Duty_Cycle += BASIC_STEP * (1 + weld_controller->temp_gain2);
			}
			/*Closed-loop control*/
			else
			{

				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[1] + STABLE_ERR,
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}

#if 0
			if (current_temp_comp >= weld_controller->second_step_turn && weld_controller->pid_ctrl->stable_flag == false)
			{
				/*到达刹车点，转阶段*/
				weld_controller->pid_ctrl->stable_flag = true;
			}

			if (weld_controller->pid_ctrl->stable_flag == false)
			{
				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->second_step_set + STABLE_ERR,
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}
			else
			{
				// 第二个设定点
				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[1] + STABLE_ERR,
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}
#endif

			/*execute*/
			TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
			TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

			break;

			/*--------------------------------------------------------------------third step----------------------------------------------------------------------*/
			/*
			In the cooling process, the temperature control requirements are not high,
			and it should be possible to give a descending curve with a specified slope,
			and the starting value of the PDC refers to the end value of the previous stage
			*/
		case THIRD_STATE:
			/*Time updates*/
			weld_controller->step_time_tick++;
			/*Algorithm control - slow down, temporarily do not implement, directly close the output*/
			/*...*/
			/*execute*/
			TIM_SetCompare1(TIM1, 1);
			TIM_SetCompare1(TIM4, 1);

			break;
		}

		/*Data visualization collection, Reduce the sampling rate*/
		if (weld_controller->weld_time_tick % temp_draw_ctrl->sample_freq == 0)
		{
			if (temp_draw_ctrl->current_index < temp_draw_ctrl->buf_len_max)
				temp_draw_ctrl->temp_buf[temp_draw_ctrl->current_index++] = current_temp_comp;
		}
	}

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

#if SYSTEM_SUPPORT_OS
	OSIntExit();
#endif
}

/**
 * @description: Timer 3 interrupts, and the total duration of the welding cycle is recorded
 * @return {*}
 */
void TIM3_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS
	OSIntEnter();
#endif

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		weld_controller->weld_time_tick += 1;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
#if SYSTEM_SUPPORT_OS
	OSIntExit();
#endif
}
