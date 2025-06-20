/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-15 19:17:48
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-20 10:00:40
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "user_config.h"
#include "timer.h"

#include "welding_process.h"
#include "stm32f4xx.h"
#include "includes.h"
#include "adc.h"
#include "PID.h"
#include "usart.h"
#include "protect.h"

#include "touchscreen.h"
#include "dynamic_correct.h"
#if KALMAN_FILTER
#include "Kalman.h"
extern Kalman kfp;
uint16_t kalman_comp_temp = 0;
#endif

/*real-time control*/
extern weld_ctrl *weld_controller;

/*error handle*/
extern Error_ctrl *err_ctrl;
extern OS_SEM ERROR_HANDLE_SEM;

/*plot*/
extern Temp_draw_ctrl *temp_draw_ctrl;
uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN] = {0};

/*sensor*/
extern Thermocouple *current_Thermocouple;

/*dynamic algorithm*/
extern uint16_t PWM_Record[500];
extern uint16_t record_index;
extern uint16_t Stable_Threshold_cnt;
extern uint16_t record_cnt;

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

	for (uint8_t i = 0; i < sizeof(ctrl->display_temp) / sizeof(ctrl->display_temp[0]); i++)
	{
		ctrl->display_temp[i] = 0;
	}

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
 * @description: APH1 defalut clock=168M/4=42M pre(=4)!=1 so APB1_TIM3_CLK=42M*2=84M
 * @return {*}
 */
void TIM3_INIT(void)
{
	uint16_t arr = 1000 - 1;
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
 * @description:  APH1 defalut clock=168M/4=42M pre(=4)!=1 so APB1_TIM5_CLK=42M*2=84M
 * @return {*}
 */
void TIM5_INIT(void)
{
	uint16_t arr = 1000 - 1;
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
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
#if KALMAN_FILTER == 1
		uint16_t kalman_filter_temp = KalmanFilter(&kfp, weld_controller->realtime_temp);
#endif

		/*Time updates*/
		weld_controller->step_time_tick++;
		/*real-time control*/
		switch (weld_controller->state)
		{
		case PRE_STATE:
			break;
			/*--------------------------------------------------------------------first step----------------------------------------------------------------------*/
		case FIRST_STATE:
			weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[0] + 2 * weld_controller->temp_comp,
														 weld_controller->realtime_temp,
														 weld_controller->Duty_Cycle,
														 weld_controller->pid_ctrl);

			break;

			/*--------------------------------------------------------------------second step----------------------------------------------------------------------*/
		case SECOND_STATE:

#if PID_DEBUG == 0
			if (weld_controller->enter_transition_flag == false)
			{
				/*no first step*/
				if (weld_controller->weld_time[1] == 0)
				{
					weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[1] * COMPENSATION_THRESHOLD,
																 weld_controller->realtime_temp,
																 weld_controller->Duty_Cycle,
																 weld_controller->pid_ctrl);
				}
				else
				{
					weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[1] + weld_controller->temp_comp,
																 weld_controller->realtime_temp,
																 weld_controller->Duty_Cycle,
																 weld_controller->pid_ctrl);
				}
			}
			else
			{
				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[1] + weld_controller->temp_comp,
															 weld_controller->realtime_temp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}

#else
			weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[1],
														 weld_controller->realtime_temp,
														 weld_controller->Duty_Cycle,
														 weld_controller->pid_ctrl);

#endif

#if PWM_SAMPLE
			if (weld_controller->realtime_temp < weld_controller->weld_temp[1] + TEMP_STABLE_ERR &&
				weld_controller->realtime_temp > weld_controller->weld_temp[1] - TEMP_STABLE_ERR)
			{
				/*Make sure the temperature is stable before sample*/
				if (Stable_Threshold_cnt > STABLE_THRESHOLD)
				{
					/*Ring Collection*/
					if (record_index < sizeof(PWM_Record) / sizeof(uint16_t))
						PWM_Record[record_index++] = weld_controller->Duty_Cycle;
					else
						record_index = 0;
					/*total count*/
					record_cnt++;
				}
				else
					Stable_Threshold_cnt++;
			}
#endif

			break;

			/*--------------------------------------------------------------------third step----------------------------------------------------------------------*/
		case THIRD_STATE:
			break;
		}

		/*Data visualization collection, Reduce the sampling rate*/
		if (weld_controller->weld_time_tick % temp_draw_ctrl->sample_freq == 0)
		{
			if (temp_draw_ctrl->current_index < temp_draw_ctrl->buf_len_max)
				temp_draw_ctrl->temp_buf[temp_draw_ctrl->current_index++] = weld_controller->realtime_temp;
		}
	}

	/*restrict output*/
	switch (current_Thermocouple->type)
	{

	case E_TYPE:
		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		break;

	case J_TYPE:
	case K_TYPE:
		if (weld_controller->realtime_temp < weld_controller->restrict_temp)
		{
			if (weld_controller->Duty_Cycle > weld_controller->restrict_duty)
				weld_controller->Duty_Cycle = weld_controller->restrict_duty;
		}
		break;
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
		weld_controller->weld_time_tick++;
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

#if SYSTEM_SUPPORT_OS
	OSIntExit();
#endif
}
