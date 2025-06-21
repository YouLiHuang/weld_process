/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-13 09:22:31
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-20 14:12:53
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "user_config.h"
#include "welding_process.h"
#include "includes.h"

#include "pwm.h"
#include "pid.h"
#include "adc.h"
#include "timer.h"
#include "protect.h"
#include "spi.h"
#include "usart.h"
#include "touchscreen.h"
#include "dynamic_correct.h"
#include "modbus_app.h"
#include "touch_screen_app.h"

#if PID_DEBUG
extern pid_feedforword_ctrl *pid_ctrl_debug;
extern pid_feedforword_ctrl *pid_ctrl;
#endif

/*---------------------------------------------------Real-time control---------------------------------------------------------*/
static volatile WELD_MODE welding_flag = IDEAL_MODE; // Welding different stage markers
extern weld_ctrl *weld_controller;					 // Welding controllers
Correction_factor corrct_factor = {0.1, 1.25};		 // Steady-state fitting curve correction coefficient
/*------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------Compatible touchscreen data interface------------------------------------------*/
extern uint8_t cur_GP;
extern RDY_SCH_STATE cur_key1;
extern ION_OFF_STATE cur_key2;
extern SGW_CTW_STATE cur_key3;
extern SWITCH_STATE switch_mode;
/*------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------False alarms--------------------------------------------------------------*/
extern OS_SEM ERROR_HANDLE_SEM;
extern Error_ctrl *err_ctrl;
/*-------------------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------Control algorithms-----------------------------------------------------------*/

#if KALMAN_FILTER
#include "Kalman.h"
extern uint16_t kalman_comp_temp; // Temperature compensation value
Kalman kfp;
#endif

/*-----------------------------------------------------temp plot-----------------------------------------------------------------*/
// user
extern Temp_draw_ctrl *temp_draw_ctrl;				 // Drawing controllers
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN]; // Temperature preservation buffer
// os
extern OS_MUTEX PLOT_Mux;
extern OS_SEM PLOT_SEM;
/*--------------------------------------------------------------------------------------------------------------------------------*/
// The current thermocouple object
extern Thermocouple *current_Thermocouple;

/*-------------------------------------------------------------- USB---------------------------------------------------------------*/
extern OS_SEM DATA_SAVE_SEM;
/*--------------------------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------- MODBUS ---------------------------------------------------------------*/
extern OS_MUTEX ModBus_Mux;
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint16_t usRegInputBuf[REG_INPUT_NREGS];
extern uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8];
extern uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];

/*---------------------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------*/
/*													 Data Object API                                                  */
/*--------------------------------------------------------------------------------------------------------------------*/
static void stop_weld(void);
WELD_MODE get_weld_flag()
{
	return welding_flag;
}

/**
 * @description: create a new controller
 * @param {pid_feedforword_ctrl} *pid_ctrl
 * @return {*}
 */
weld_ctrl *new_weld_ctrl(pid_feedforword_ctrl *pid_ctrl)
{
	weld_ctrl *ctrl = (weld_ctrl *)malloc(sizeof(weld_ctrl));
	if (ctrl != NULL)
	{
		/*control parameter*/
		ctrl->Duty_Cycle = PD_MIN;
		ctrl->pid_ctrl = pid_ctrl;
		ctrl->enter_transition_flag = false;

		/*realtime parameter*/
		ctrl->first_step_start_temp = 0;
		ctrl->second_step_start_temp = 0;
		ctrl->third_step_start_temp = 0;
		ctrl->state = IDEAL_STATE;
		ctrl->Count_Dir = UP;

		/*time tick*/
		ctrl->step_time_tick = 0;
		ctrl->weld_time_tick = 0;

		/*count*/
		ctrl->weld_count = 0;

		/*user parameter*/
		for (uint8_t i = 0; i < sizeof(ctrl->weld_time) / sizeof(ctrl->weld_time[0]); i++)
		{
			ctrl->weld_time[i] = 0;
		}
		for (uint8_t i = 0; i < sizeof(ctrl->weld_temp) / sizeof(ctrl->weld_temp[0]); i++)
		{
			ctrl->weld_temp[i] = 0;
		}
		for (uint8_t i = 0; i < sizeof(ctrl->alarm_temp) / sizeof(ctrl->alarm_temp[0]); i++)
		{
			ctrl->alarm_temp[i] = 0;
		}
		ctrl->temp_gain1 = DEFAULT_GAIN1;
		ctrl->temp_gain2 = DEFAULT_GAIN2;
		ctrl->hold_temp = 0;
		ctrl->final_duty = 0;
		ctrl->temp_comp = STABLE_ERR_E;
		ctrl->restrict_temp = ctrl->weld_temp[0] * RESTRICT_TEMP_COFF;
		ctrl->ss_coefficient.slope = DEFAULT_SLOPE;
		ctrl->ss_coefficient.intercept = DEFAULT_INTERCEPT;

		ctrl->user_hook_callback = stop_weld;

		return ctrl;
	}
	return NULL;
}

/**
 * @description: controller reset
 * @param {weld_ctrl} *ctrl
 * @return {*}
 */
bool weld_ctrl_reset(weld_ctrl *ctrl)
{
	if (ctrl != NULL)
	{
		/*control parameter*/
		ctrl->Duty_Cycle = PD_MIN;
		/*realtime parameter*/
		ctrl->first_step_start_temp = 0;
		ctrl->second_step_start_temp = 0;
		ctrl->third_step_start_temp = 0;
		ctrl->state = IDEAL_STATE;

		/*time tick*/
		ctrl->step_time_tick = 0;
		ctrl->weld_time_tick = 0;

		/*count*/
		ctrl->weld_count = 0;

		ctrl->temp_gain1 = DEFAULT_GAIN1;
		ctrl->temp_gain2 = DEFAULT_GAIN2;
		/*user parameter*/
		for (uint8_t i = 0; i < sizeof(ctrl->weld_time) / sizeof(ctrl->weld_time[0]); i++)
		{
			ctrl->weld_time[i] = 0;
		}
		for (uint8_t i = 0; i < sizeof(ctrl->weld_temp) / sizeof(ctrl->weld_temp[0]); i++)
		{
			ctrl->weld_temp[i] = 0;
		}
		for (uint8_t i = 0; i < sizeof(ctrl->alarm_temp) / sizeof(ctrl->alarm_temp[0]); i++)
		{
			ctrl->alarm_temp[i] = 0;
		}

		/*pid controller reset*/
		reset_forword_ctrl(ctrl->pid_ctrl);
#if PID_DEBUG
		reset_forword_ctrl(pid_ctrl_debug);
#endif

		return true;
	}
	return false;
}

/**
 * @description: number convert to string
 * @param {char} *buffer
 * @param {uint8_t} buf_len
 * @param {uint16_t} value
 * @return {*}
 */
void user_value_convert_to_string(char *buffer, const uint8_t buf_len, const uint16_t value)
{

	/*data length calculate*/
	uint8_t length = 1;
	uint16_t cal_val = value;
	while (cal_val > 0)
	{
		cal_val /= 10;
		if (cal_val > 0)
			length++;
	}

	/*value convert to string*/
	uint8_t per_bit = 0;
	uint8_t cnt;
	if (length <= buf_len) // len:4 /1000%10 /100%10 /10%10 %10
	{
		for (int i = length - 1; i >= 0; i--)
		{
			cal_val = value;
			cnt = i;
			if (i != 0)
			{
				while (cnt-- != 0)
					cal_val /= 10;

				cal_val %= 10;
				per_bit = cal_val;
			}
			else
				per_bit = value % 10;

			buffer[length - i - 1] = per_bit + '0';
		}
	}
}

/**
 * @description: Dynamically adjust pid parameters according to different step
 * @return {*}
 */
static void pid_dynamic_reload(void)
{
	switch (current_Thermocouple->type)
	{
	case J_TYPE:
	case K_TYPE:
		break;
	case E_TYPE:
		/*two step*/
		if (weld_controller->weld_time[1] > 100 && weld_controller->weld_time[3] > 0)
		{
			switch (weld_controller->state)
			{
			case FIRST_STATE:
				weld_controller->pid_ctrl->kp = FIRST_STEP_KP;
				weld_controller->pid_ctrl->ki = FIRST_STEP_KI;
				weld_controller->pid_ctrl->kd = FIRST_STEP_KD;
				break;
			case SECOND_STATE:
				weld_controller->pid_ctrl->kp = SECOND_KP;
				weld_controller->pid_ctrl->ki = SECOND_KI;
				weld_controller->pid_ctrl->kd = SECOND_KD;
				break;

			default:
				break;
			}
		}
		break;
	}
}

/*--------------------------------------------------------------------------------------------------------------------*/
/*													 Process Control API                                              */
/*--------------------------------------------------------------------------------------------------------------------*/
static void First_Step_ECB(void);
static void First_Step_JCB(void);
static void First_Step_KCB(void);

static void Second_Step_ECB(void);
static void Second_Step_JCB(void);
static void Second_Step_KCB(void);

static void down_temp_line()
{
	OS_ERR err;
	uint16_t index = 0;
	uint16_t weld_win_width = 0;
	uint16_t win_width = 0;
	uint16_t total_time = 0;
	uint16_t temp = 0;
	uint8_t temp_display = 0;
	uint8_t key = 0;

	for (uint8_t i = 0; i < 5; i++)
		total_time += weld_controller->weld_time[i];

	weld_win_width = total_time / (temp_draw_ctrl->delta_tick - 1); // 转换计算(参见读写线程的坐标绘制)：焊接周期绘图区域大小
	win_width = WIN_WIDTH - weld_win_width - DRAW_RESERVE;			// 温降曲线绘图区域大小（留一个余量）
	if (win_width >= WIN_WIDTH / 2)
		win_width = WIN_WIDTH / 2;

	OSMutexPend(&PLOT_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
	while (index < win_width)
	{
		if (get_weld_flag() == BUSY_MODE)
			break;

		/*stop draw start another weld*/
		if (cur_key3 == SGW)
		{
			key = RLY_INPUT_SCAN();
			if (key == RLY_START1_ACTIVE || key == RLY_START0_ACTIVE)
				break;
		}

		temp = temp_convert(current_Thermocouple); // temp sample
		if (temp > MAX_TEMP_DISPLAY)			   // limit
			temp = MAX_TEMP_DISPLAY;

		temp_display = temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY;						// Coordinate scaling
		draw_point(temp_display);														// draw
		command_set_comp_val("step3", "val", temp);										// display realtime temp
		OSTimeDlyHMSM(0, 0, 0, temp_draw_ctrl->delta_tick, OS_OPT_TIME_PERIODIC, &err); // Sampling interval
		index++;
	}
	OSMutexPost(&PLOT_Mux, OS_OPT_POST_NONE, &err);
}

static void display_temp_cal(void)
{
	uint32_t sum = 0;

	/*step 1*/
	temp_draw_ctrl->display_temp[0] = weld_controller->second_step_start_temp;
	/*step 2*/
	for (uint16_t i = temp_draw_ctrl->second_step_stable_index; i < temp_draw_ctrl->second_step_index_end; i++)
		sum += temp_draw_ctrl->temp_buf[i];
	temp_draw_ctrl->display_temp[1] = sum / (temp_draw_ctrl->second_step_index_end - temp_draw_ctrl->second_step_stable_index + 1);
	/*step 3*/
	temp_draw_ctrl->display_temp[2] = weld_controller->weld_temp[2];
}

/**
 * @description: PWM deinit
 * @return {*}
 */
static void PMW_TIMER_DEINIT(void)
{
	/*TIMER DEINIT*/
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
	TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);

	TIM_Cmd(TIM3, DISABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_Cmd(TIM5, DISABLE);
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}

/**
 * @description:
 * @param {START_TYPE} type
 * @return {*}
 */
static void Load_Data(START_TYPE type)
{

	switch (type)
	{
	case KEY0:
		if (cur_GP % 2 != 0)
		{
			cur_GP--;
			Load_param(weld_controller, cur_GP);
			Load_param_alarm(weld_controller, cur_GP);
			command_set_comp_val("GP", "val", cur_GP);
		}

		break;
	case KEY1:
		if (cur_GP < 19 && (cur_GP % 2 == 0))
		{
			cur_GP++;
			Load_param(weld_controller, cur_GP);
			Load_param_alarm(weld_controller, cur_GP);
			command_set_comp_val("GP", "val", cur_GP);
		}

		break;
	}
}

/**
 * @description: Preparation before welding
 * @return {*}
 */
static void Weld_Preparation()
{

	OS_ERR err;
	/*表示进入焊接过程*/
	welding_flag = BUSY_MODE;
	/*----------------------------------------时间刻度复位----------------------------------------*/
	weld_controller->step_time_tick = 0;
	weld_controller->weld_time_tick = 0;
	/*------------------------------------------参数限制------------------------------------------*/
	/*稳态温度误差补偿*/
	switch (current_Thermocouple->type)
	{
	case E_TYPE:
		weld_controller->temp_comp = STABLE_ERR_E;
		break;
	case J_TYPE:
		weld_controller->temp_comp = STABLE_ERR_J;
		break;
	case K_TYPE:
		weld_controller->temp_comp = STABLE_ERR_K;
		break;
	}

	if (weld_controller->weld_time[1] > 999) // first step
		weld_controller->weld_time[1] = 999;
	if (weld_controller->weld_time[2] > 200) // hold time
		weld_controller->weld_time[2] = 200;
	if (weld_controller->weld_time[3] > 9999) // second time
		weld_controller->weld_time[3] = 9999;
	if (weld_controller->weld_time[4] > 999) // third time
		weld_controller->weld_time[4] = 999;

	if (weld_controller->weld_temp[0] < USER_SET_MIN_TEMP)
		weld_controller->weld_temp[0] = USER_SET_MIN_TEMP;

	if (weld_controller->weld_temp[1] < USER_SET_MIN_TEMP)
		weld_controller->weld_temp[1] = USER_SET_MIN_TEMP;

	if (weld_controller->weld_temp[0] > USER_FIRST_SET_MAX)
		weld_controller->weld_temp[0] = USER_FIRST_SET_MAX;

	if (weld_controller->weld_temp[1] > USER_SET_MAX_TEMP)
		weld_controller->weld_temp[1] = USER_SET_MAX_TEMP;

	if (weld_controller->weld_temp[0] > weld_controller->weld_temp[1])
		weld_controller->weld_temp[0] = weld_controller->weld_temp[0];

	if (weld_controller->temp_gain1 >= 1)
		weld_controller->temp_gain1 = 1;
	if (weld_controller->temp_gain1 <= 0)
		weld_controller->temp_gain1 = 0;

	if (weld_controller->temp_gain2 >= 1)
		weld_controller->temp_gain2 = 1;
	if (weld_controller->temp_gain2 <= 0)
		weld_controller->temp_gain2 = 0;

/*------------------------------------------算法部分------------------------------------------*/
#if PID_DEBUG
	reset_forword_ctrl(pid_ctrl_debug);
#endif
	reset_forword_ctrl(weld_controller->pid_ctrl); // pid控制器初始化

#if COMPENSATION
	dynamic_init(&dynam_comp, 100); // 动态补偿初始化
	window_init(&lasttemp);			// 滑动窗口初始化
#endif

#if KALMAN_FILTER
	Kalman_Init(&kfp);	  // 卡尔曼滤波器初始化，初始值归零
	kalman_comp_temp = 0; // 卡尔曼估计值初始化
#endif

	/*------------------------------------------plot --------------------------------------------*/
	reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);
	/*------------------------------------------IO ctrl------------------------------------------*/

	/*MODBUS updata*/
	OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
	RLY_AIR0 = 1; // 气阀1启动
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_0;
	RLY_AIR1 = 1; // 气阀2启动
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_1;
	RLY_AIR2 = 1; // 气阀3启动
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_2;
	RLY_OVER = 0; // 1为焊接结束信号
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_3);
	RLY_CNT = 0; // 1为计数，0清除计数信号
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_5);
	OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);

	/*------------------------------------------pwm deinit-------------------------------------------*/
	uint16_t tmp_ccmr1 = 0;
	tmp_ccmr1 = TIM1->CCMR1;
	tmp_ccmr1 &= (uint16_t)~TIM_CCMR1_OC1M;
	tmp_ccmr1 |= ((uint16_t)0x0060);
	TIM1->CCMR1 = tmp_ccmr1;

	tmp_ccmr1 = TIM4->CCMR1;
	tmp_ccmr1 &= (uint16_t)~TIM_CCMR1_OC1M;
	tmp_ccmr1 |= ((uint16_t)0x0060);
	TIM4->CCMR1 = tmp_ccmr1;
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}

/**
 * @description: 中途停止焊接————后续可作为急停警报
 * @return {*}
 */
static void stop_weld(void)
{
	/*确保PWM关闭*/
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
	TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
	TIM_Cmd(TIM4, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	TIM_Cmd(TIM3, DISABLE);						// 焊接周期计数关闭
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志位
	TIM_Cmd(TIM5, DISABLE);						// 实时控制器关闭
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); // 清除中断5标志位
	TIM3->CNT = 0;
	TIM5->CNT = 0;
}

/**
 * @description: start of weld
 * @return {*}
 */
static void start_of_weld(void)
{
	/*Start recording welding cycle time*/
	weld_controller->weld_time_tick = 0;
	TIM_Cmd(TIM3, ENABLE);
}

/**
 * @description: Pre-stress before welding
 * @return {*}
 */
static void Preload()
{

	/*焊接周期第一段，预压*/
	weld_controller->state = PRE_STATE;
	weld_controller->step_time_tick = 0;
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[0] + 1)
		;
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	weld_controller->state = IDEAL_STATE;
	weld_controller->step_time_tick = 0;
}

/**
 * @description: The first stage of welding, the temperature rises
 * @return {*}
 */
static void First_Step()
{
	switch (current_Thermocouple->type)
	{
	case E_TYPE:
		First_Step_ECB();
		break;
	case J_TYPE:
		First_Step_JCB();
		break;
	case K_TYPE:
		First_Step_KCB();
		break;
	}
}

/**
 * @description: In the second stage of welding, the temperature is maintained
 * @return {*}
 */
static void Second_Step()
{

	switch (current_Thermocouple->type)
	{
	case E_TYPE:
		Second_Step_ECB();
		break;
	case J_TYPE:
		Second_Step_JCB();
		break;
	case K_TYPE:
		Second_Step_KCB();
		break;
	}
}
/**
 * @description: The third stage of welding, the temperature drops
 * @return {*}
 */
static void Third_Step()
{
	/*enter third step*/
	weld_controller->state = THIRD_STATE;

	/*1、三阶段采样开始，坐标记录*/
	temp_draw_ctrl->third_step_index_start = temp_draw_ctrl->current_index;
	weld_controller->step_time_tick = 0;
	/*2、实时控制部分*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[4])
	{
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);

		/*Temperature out of control*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[4])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}

#if REALTIME_TEMP_DISPLAY == 1
		if (weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			switch (request_PGManger()->id)
			{
			case WAVE_PAGE:
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
				command_set_comp_val("step3", "val", weld_controller->realtime_temp);
				break;
			case PARAM_PAGE:
				command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
				break;
			}
		}
#endif

		/*limit output execute*/
		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, 0);
		TIM_SetCompare1(TIM4, 0);
	}

	/*end of step*/
	TIM_Cmd(TIM5, DISABLE);													  // 关闭实时控制器
	TIM5->CNT = 0;															  // 计数值复位
	weld_controller->state = IDEAL_STATE;									  // 焊接状态复位
	temp_draw_ctrl->third_step_index_end = temp_draw_ctrl->current_index - 1; // 记录阶段结束绘图坐标
}

/**
 * @description: End welding and reset parameters
 * @return {*}
 */
static void End_of_Weld()
{
	OS_ERR err;

	TIM_Cmd(TIM3, DISABLE); // 关闭焊接周期计数
	TIM_Cmd(TIM5, DISABLE); // 关闭实时控制器
	TIM3->CNT = 0;
	TIM5->CNT = 0;

	/*记录焊接时间*/
	temp_draw_ctrl->tick_record = weld_controller->weld_time_tick;
	/*焊接终了，时间记录复位*/
	weld_controller->weld_time_tick = 0;
	weld_controller->step_time_tick = 0;
	/*焊接结束*/
	welding_flag = IDEAL_MODE;
	weld_controller->Duty_Cycle = 0;
	weld_controller->state = IDEAL_STATE;

	/*MODBUS updata*/
	OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
	RLY_AIR0 = 0; // 气阀1关闭
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_0);
	RLY_AIR1 = 0; // 气阀2关闭
	ucRegCoilsBuf[0] &= (0x01 << COIL_ADDR_1);
	RLY_AIR2 = 0; // 气阀3关闭
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_2);
	RLY_OVER = 1; // 1为焊接结束信号
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_3;
	RLY_CNT = 1; // 1为计数，0清除计数信号
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_5;
	/*weld conut updata*/
	if (weld_controller->Count_Dir == UP)
	{
		weld_controller->weld_count++;
		usRegHoldingBuf[HOLD_ADDR_16] = weld_controller->weld_count;
	}
	else if (weld_controller->Count_Dir == DOWN && weld_controller->weld_count > 0)
	{
		weld_controller->weld_count--;
		usRegHoldingBuf[HOLD_ADDR_16] = weld_controller->weld_count;
	}
	command_set_comp_val("count", "val", weld_controller->weld_count);
	OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);
}

/**
 * @description: Conduct a simulation of welding with no actual current output
 * @return {*}
 */
static void simulate_weld()
{
	OS_ERR err;

	/*MODBUS updata*/
	OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
	RLY_AIR0 = 1; // 气阀1启动
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_0;
	RLY_AIR1 = 1; // 气阀2启动
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_1;
	RLY_AIR2 = 1; // 气阀3启动
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_2;
	RLY_OVER = 0; // 1为焊接结束信号
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_3);
	RLY_CNT = 0; // 1为计数，0清除计数信号
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_5);
	OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);

	/*controller reset*/
	weld_controller->step_time_tick = 0;
	weld_controller->weld_time_tick = 0;
	TIM_Cmd(TIM3, ENABLE);
	while (weld_controller->weld_time_tick < weld_controller->weld_time[0] + weld_controller->weld_time[1] + weld_controller->weld_time[3] + weld_controller->weld_time[4])
		;
	TIM_Cmd(TIM3, DISABLE);
	weld_controller->weld_time_tick = 0;

	/*MODBUS updata*/
	OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
	RLY_AIR0 = 0; // 气阀1关闭
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_0);
	RLY_AIR1 = 0; // 气阀2关闭
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_1);
	RLY_AIR2 = 0; // 气阀3关闭
	ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_2);
	RLY_OVER = 1; // 1为焊接结束信号
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_3;
	RLY_CNT = 1; // 1为计数，0清除计数信号
	ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_5;
	/*weld conut updata*/
	if (weld_controller->Count_Dir == UP)
	{
		weld_controller->weld_count++;
		usRegHoldingBuf[HOLD_ADDR_16] = weld_controller->weld_count;
	}
	else if (weld_controller->Count_Dir == DOWN && weld_controller->weld_count > 0)
	{
		weld_controller->weld_count--;
		usRegHoldingBuf[HOLD_ADDR_16] = weld_controller->weld_count;
	}
	OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);
}

/**
 * @description: Real-time welding control.
 * @return {*}
 */
static void weld_real_time_ctrl()
{

	OS_ERR err;

	/*变量初始化以及相关初始化工作：控制器复位、初始化、缓存擦除、参数限幅*/
	Weld_Preparation();
	start_of_weld();
	Preload();

	/*first step*/
	if (weld_controller->weld_time[1] != 0)
	{
		weld_controller->first_step_start_temp = temp_convert(current_Thermocouple);
		First_Step();
		if (true == err_occur(err_ctrl))
			goto STOP_LABEL;
	}

	/*second step*/
	if (weld_controller->weld_time[3] != 0)
	{
		reset_forword_ctrl(weld_controller->pid_ctrl);
#if PID_DEBUG
		reset_forword_ctrl(pid_ctrl_debug);
		reset_forword_ctrl(weld_controller->pid_ctrl);
#endif
		weld_controller->second_step_start_temp = temp_convert(current_Thermocouple);
		Second_Step();
		if (true == err_occur(err_ctrl))
			goto STOP_LABEL;
	}

	/*third step*/
	if (weld_controller->weld_time[3] != 0)
	{
		weld_controller->third_step_start_temp = temp_convert(current_Thermocouple);
		Third_Step();
		if (true == err_occur(err_ctrl))
			goto STOP_LABEL;
	}

	/*end*/
	End_of_Weld();

	/*dynamic algorithm*/
	dynamic_pwm_adjust();		  // final pwm dynamic
	if (switch_mode == AUTO_MODE) // gain dynamic
		dynamic_param_adjust();

STOP_LABEL:
	if (true == err_occur(err_ctrl))
	{
		stop_weld();
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
}

/**
 * @description:
 * @param {START_TYPE} type
 * @return {*}
 */
void welding_process(START_TYPE type)
{
	OS_ERR err;
	uint8_t key = 0;

	/*which key is pressed*/
	key = (type == START1) ? RLY_START0_ACTIVE : RLY_START1_ACTIVE;

	/*load different parameter sets based on the specific button pressed.*/
	Load_Data(type);

	/*The welding is stopped when the countdown timer reaches the end*/
	if (weld_controller->weld_count == 0 && weld_controller->Count_Dir == DOWN)
		return;

	/*PWM deinit*/
	PMW_TIMER_DEINIT();

	/*----------------------------------------------- real-time control ---------------------------------------------------*/

	/*-------------------------------------------------------CTW-------------------------------------------------------*/
	if (cur_key3 == CTW && cur_key2 == ION && cur_key1 == RDY)
	{

		key = RLY_INPUT_SCAN();
		/*weld until release key*/
		while (key == RLY_START0_ACTIVE || key == RLY_START1_ACTIVE)
		{

			weld_controller->realtime_temp = temp_convert(current_Thermocouple);
			/*emter weld*/
			if (weld_controller->realtime_temp < weld_controller->weld_temp[2])
			{
				if (true == err_occur(err_ctrl))
					break;

				/*clear touch screen*/
				if (request_PGManger()->id == WAVE_PAGE)
				{
					command_send("cle wave_line.id,0");
					OSTimeDly(5, OS_OPT_TIME_DLY, &err);
				}

				/*weld real-time control*/
				weld_real_time_ctrl();

				/*calculate three temperature to display*/
				display_temp_cal();

				/*plot temp line(down)*/
				if (request_PGManger()->id == WAVE_PAGE)
				{
					down_temp_line();
				}

				/*save data to disk*/
				OSSemPost(&DATA_SAVE_SEM, OS_OPT_POST_ALL, &err);

				/*weld interval*/
				OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
			}

			/*check if key release*/
			key = RLY_INPUT_SCAN();
		}
	}
	/*simulate weld*/
	else if (cur_key3 == CTW && cur_key2 == IOFF && cur_key1 == RDY)
	{
		key = RLY_INPUT_SCAN();
		while (key == RLY_START0_ACTIVE || key == RLY_START1_ACTIVE)
		{
			simulate_weld();
			/*weld interval*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
			key = RLY_INPUT_SCAN();
			if (!(key == RLY_START0_ACTIVE || key == RLY_START1_ACTIVE))
				break;
		}
	}

	/*------------------------------------------------------SGW-------------------------------------------------------*/
	if (cur_key3 == SGW && cur_key2 == ION && cur_key1 == RDY)
	{
		key = RLY_INPUT_SCAN();
		if (key != RLY_START0_ACTIVE && key != RLY_START1_ACTIVE)
			return;

		/*clear screen*/
		if (request_PGManger()->id == WAVE_PAGE)
		{
			command_send("cle wave_line.id,0");
			OSTimeDly(5, OS_OPT_TIME_DLY, &err);
		}
		/*enter weld*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		if (weld_controller->realtime_temp > weld_controller->weld_temp[2])
			return;

		/*weld real-time control*/
		weld_real_time_ctrl();

		/*calculate three temperature to display*/
		display_temp_cal();

		/*plot temp line(down)*/
		if (request_PGManger()->id == WAVE_PAGE)
			down_temp_line();
		else /*not in wave page, notify plot task to plot temp line*/
		{
			/*avoid send one more time, which will plot one more line*/
			OSSemSet(&PLOT_SEM, 0, &err);
			OSSemPost(&PLOT_SEM, OS_OPT_POST_ALL, &err);
		}

		/*save data to disk*/
		OSSemPost(&DATA_SAVE_SEM, OS_OPT_POST_ALL, &err);

		/*weld interval*/
		OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);

		/*wait until release key*/
		while (1)
		{
			key = RLY_INPUT_SCAN();
			if (key != RLY_START0_ACTIVE && key != RLY_START1_ACTIVE)
				break;

			// main task pend
			OSTimeDly(10, OS_OPT_TIME_DLY, &err);
		}
	}
	/*weld simulate*/
	else if (cur_key3 == SGW && cur_key2 == IOFF && cur_key1 == RDY)
	{
		simulate_weld();
		/*weld interval*/
		OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
		/*wait until release key*/
		while (1)
		{
			key = RLY_INPUT_SCAN();
			if (key != RLY_START0_ACTIVE && key != RLY_START1_ACTIVE)
				break;

			// main task pend
			OSTimeDly(10, OS_OPT_TIME_DLY, &err);
		}
	}
}

/*-----------------------------------------callback real time temp control----------------------------------------------*/
static void First_Step_ECB(void)
{
	Page_ID cur_Page_id = request_PGManger()->id;

	/*enter first step*/
	weld_controller->state = FIRST_STATE;
	/*start sample*/
	temp_draw_ctrl->first_step_index_start = 0;
	/*reset cotroller*/
	weld_controller->Duty_Cycle = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;

	/*restrict duty avoid too fast rise*/
	/*restrict param*/
	weld_controller->restrict_temp = RESTRICT_TEMP_COFF * weld_controller->weld_temp[0];
	weld_controller->restrict_duty = PD_MAX * (0.1 + 0.9 * weld_controller->temp_gain1);

#if PID_DEBUG
	weld_controller->pid_ctrl->kp = FIRST_STEP_KP;
	weld_controller->pid_ctrl->ki = FIRST_STEP_KI;
	weld_controller->pid_ctrl->kd = FIRST_STEP_KD;
#else
	pid_dynamic_reload();
#endif

	/*real-time control*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[1])
	{
		/*temp sample*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		/*alarm*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[0])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}
		// low temp
		//		if (weld_controller->realtime_temp < weld_controller->first_step_start_temp + 10 &&
		//			weld_controller->realtime_temp > weld_controller->first_step_start_temp - 10 &&
		//			weld_controller->step_time_tick > 100)
		//		{
		//			stop_weld();
		//			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		//			OS_ERR err;
		//			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		//			break;
		//		}

		/*restrict output*/
		if (weld_controller->realtime_temp < weld_controller->restrict_temp)
		{
			if (weld_controller->Duty_Cycle > weld_controller->restrict_duty)
				weld_controller->Duty_Cycle = weld_controller->restrict_duty;
		}

		/*limit output*/
		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			switch (cur_Page_id)
			{
			case WAVE_PAGE:
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
				command_set_comp_val("step3", "val", weld_controller->realtime_temp);
				break;
			case PARAM_PAGE:
				command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
				break;
			}
		}
#endif
	}

	/*end of first step*/
	/*timer reset*/
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	/*controller reset*/
	weld_controller->state = IDEAL_STATE;
	/*draw controller reset*/
	temp_draw_ctrl->first_step_index_end = temp_draw_ctrl->current_index - 1;
}

static void First_Step_JCB(void)
{

	Page_ID cur_Page_id = request_PGManger()->id;

	/*enter first step*/
	weld_controller->state = FIRST_STATE;
	/*start sample*/
	temp_draw_ctrl->first_step_index_start = 0;
	/*reset cotroller*/
	weld_controller->Duty_Cycle = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;

	/*restrict duty avoid too fast rise*/
	/*restrict param*/
	weld_controller->restrict_temp = RESTRICT_TEMP_COFF * weld_controller->weld_temp[0];
	weld_controller->restrict_duty = PD_MAX * (0.1 + 0.9 * weld_controller->temp_gain1);

#if PID_DEBUG
	weld_controller->pid_ctrl->kp = FIRST_STEP_KP;
	weld_controller->pid_ctrl->ki = FIRST_STEP_KI;
	weld_controller->pid_ctrl->kd = FIRST_STEP_KD;
#else
	pid_dynamic_reload();
#endif

	/*real-time control*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[1])
	{
		/*temp sample*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		/*alarm*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[0])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}
		// low temp
		//		if (weld_controller->realtime_temp < weld_controller->first_step_start_temp + 10 &&
		//			weld_controller->realtime_temp > weld_controller->first_step_start_temp - 10 &&
		//			weld_controller->step_time_tick > 100)
		//		{
		//			stop_weld();
		//			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		//			OS_ERR err;
		//			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		//			break;
		//		}

		/*restrict output*/
		if (weld_controller->realtime_temp < weld_controller->restrict_temp)
		{
			if (weld_controller->Duty_Cycle > weld_controller->restrict_duty)
				weld_controller->Duty_Cycle = weld_controller->restrict_duty;
		}

		/*limit output*/
		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			switch (cur_Page_id)
			{
			case WAVE_PAGE:
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
				command_set_comp_val("step3", "val", weld_controller->realtime_temp);
				break;
			case PARAM_PAGE:
				command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
				break;
			}
		}
#endif
	}

	/*end of first step*/
	/*timer reset*/
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	/*controller reset*/
	weld_controller->state = IDEAL_STATE;
	/*draw controller reset*/
	temp_draw_ctrl->first_step_index_end = temp_draw_ctrl->current_index - 1;
}

static void First_Step_KCB(void)
{
	Page_ID cur_Page_id = request_PGManger()->id;
	/*restrict duty avoid too fast rise*/

	/*enter first step*/
	weld_controller->state = FIRST_STATE;
	/*start sample*/
	temp_draw_ctrl->first_step_index_start = 0;
	/*reset cotroller*/
	weld_controller->Duty_Cycle = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;

	/*restrict param*/
	weld_controller->restrict_temp = RESTRICT_TEMP_COFF * weld_controller->weld_temp[0];
	weld_controller->restrict_duty = PD_MAX * (0.1 + 0.9 * weld_controller->temp_gain1);

#if PID_DEBUG
	weld_controller->pid_ctrl->kp = FIRST_STEP_KP;
	weld_controller->pid_ctrl->ki = FIRST_STEP_KI;
	weld_controller->pid_ctrl->kd = FIRST_STEP_KD;
#else
	pid_dynamic_reload();
#endif

	/*real-time control*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[1])
	{
		/*temp sample*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		/*alarm*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[0])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}
		// low temp
		//		if (weld_controller->realtime_temp < weld_controller->first_step_start_temp + 10 &&
		//			weld_controller->realtime_temp > weld_controller->first_step_start_temp - 10 &&
		//			weld_controller->step_time_tick > 100)
		//		{
		//			stop_weld();
		//			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		//			OS_ERR err;
		//			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		//			break;
		//		}

		/*restrict output*/
		if (weld_controller->realtime_temp < weld_controller->restrict_temp)
		{
			if (weld_controller->Duty_Cycle > weld_controller->restrict_duty)
				weld_controller->Duty_Cycle = weld_controller->restrict_duty;
		}

		/*limit output*/
		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			switch (cur_Page_id)
			{
			case WAVE_PAGE:
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
				command_set_comp_val("step3", "val", weld_controller->realtime_temp);
				break;
			case PARAM_PAGE:
				command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
				break;
			}
		}
#endif
	}

	/*end of first step*/
	/*timer reset*/
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	/*controller reset*/
	weld_controller->state = IDEAL_STATE;
	/*draw controller reset*/
	temp_draw_ctrl->first_step_index_end = temp_draw_ctrl->current_index - 1;
}

static void Second_Step_ECB(void)
{
	Page_ID cur_Pgae_id = request_PGManger()->id;
	Steady_state_coefficient ss = weld_controller->ss_coefficient;
	uint16_t second_temp_record_start = weld_controller->weld_temp[1] * TEMP_AVG_SAMPLE_START;
	uint16_t current_hold_time = 0;

	/*enter second step*/
	weld_controller->state = SECOND_STATE;
	/*start sample*/
	temp_draw_ctrl->second_step_index_start = temp_draw_ctrl->current_index;
	temp_draw_ctrl->second_step_stable_index = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;
	/*heat compensation reset*/
	weld_controller->enter_transition_flag = false;
	weld_controller->enter_transition_time = 0;
	/*restrict param*/
	weld_controller->restrict_temp = weld_controller->weld_temp[1] * RESTRICT_TEMP_COFF;
	weld_controller->restrict_duty = PD_MAX * (0.1 + 0.9 * weld_controller->temp_gain2);
	/*hold temp*/
	weld_controller->hold_temp = weld_controller->weld_temp[1] * COMPENSATION_THRESHOLD;
	/*Steady-state estimation output (coefficients can be added here for correction)*/
	weld_controller->final_duty = ss.slope * weld_controller->weld_temp[1] + ss.intercept;
	if (weld_controller->final_duty > FINAL_DUTY_LIMIT)
		weld_controller->final_duty = FINAL_DUTY_LIMIT;

#if PID_DEBUG
	weld_controller->pid_ctrl->kp = pid_ctrl_debug->kp;
	weld_controller->pid_ctrl->ki = pid_ctrl_debug->ki;
	weld_controller->pid_ctrl->kd = pid_ctrl_debug->kd;
#else
	/*pid reload*/
	pid_dynamic_reload();
#endif

	/*real-time control*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[3])
	{
		/*temp sample*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		/*alarm*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[2])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}

		/*low temp alarm*/
		//		if (weld_controller->realtime_temp < weld_controller->second_step_start_temp + 10 &&
		//			weld_controller->realtime_temp > weld_controller->second_step_start_temp - 10 &&
		//			weld_controller->step_time_tick > 100)
		//		{
		//			stop_weld();
		//			err_get_type(err_ctrl, TEMP_DOWN)->state = true;
		//			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		//			OS_ERR err;
		//			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		//			break;
		//		}

		/*second step temp avg record*/
		if (weld_controller->realtime_temp >= second_temp_record_start &&
			temp_draw_ctrl->second_step_stable_index == 0)
		{
			if (weld_controller->pid_ctrl->stable_threshold_cnt >= weld_controller->pid_ctrl->stable_threshold)
				temp_draw_ctrl->second_step_stable_index = temp_draw_ctrl->current_index;
			else
				weld_controller->pid_ctrl->stable_threshold_cnt++;
		}

		/*temp rise step ---> restrict output*/
		if (weld_controller->realtime_temp < weld_controller->restrict_temp)
		{
			if (weld_controller->Duty_Cycle > weld_controller->restrict_duty)
				weld_controller->Duty_Cycle = weld_controller->restrict_duty;
		}

		/*enter transition area (heat compensation)*/
		if (weld_controller->realtime_temp > weld_controller->hold_temp)
		{
			/*only execute the code below one time*/
			if (weld_controller->enter_transition_flag == false &&
				weld_controller->enter_transition_time == 0)
			{
				weld_controller->enter_transition_flag = true;
				weld_controller->enter_transition_time = weld_controller->step_time_tick;
			}
		}

		/*in transition area hold the temp (heat compensation)*/
		if (weld_controller->enter_transition_flag == true)
		{
			current_hold_time = weld_controller->step_time_tick - weld_controller->enter_transition_time;
			if (current_hold_time < weld_controller->weld_time[2])
			{
				if (weld_controller->Duty_Cycle < weld_controller->final_duty)
					weld_controller->Duty_Cycle = weld_controller->final_duty;
			}
		}

		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			switch (cur_Pgae_id)
			{
			case WAVE_PAGE:
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
				command_set_comp_val("step3", "val", weld_controller->realtime_temp);
				break;
			case PARAM_PAGE:
				command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
				break;
			}
		}
#endif
	}

	/*timer reset*/
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	/*PWM off*/
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	/*end of step*/
	weld_controller->state = IDEAL_STATE;
	/*Record end coordinates*/
	temp_draw_ctrl->second_step_index_end = temp_draw_ctrl->current_index - 1;
}

static void Second_Step_JCB(void)
{
	Page_ID cur_Pgae_id = request_PGManger()->id;
	Steady_state_coefficient ss = weld_controller->ss_coefficient;
	uint16_t second_temp_record_start = weld_controller->weld_temp[1] * TEMP_AVG_SAMPLE_START;
	uint16_t current_hold_time = 0;

#if FAST_RISE_ENABLE
	uint16_t fast_rise_duty = 0;
	fast_rise_duty = ss.slope * weld_controller->weld_temp[1] + ss.intercept;
#endif

	/*enter second step*/
	weld_controller->state = SECOND_STATE;
	/*start sample*/
	temp_draw_ctrl->second_step_index_start = temp_draw_ctrl->current_index;
	temp_draw_ctrl->second_step_stable_index = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;
	/*heat compensation reset*/
	weld_controller->enter_transition_flag = false;
	weld_controller->enter_transition_time = 0;
	/*restrict param*/
	weld_controller->restrict_temp = weld_controller->weld_temp[1] * RESTRICT_TEMP_COFF;
	weld_controller->restrict_duty = PD_MAX * (0.1 + 0.9 * weld_controller->temp_gain2);
	/*hold temp*/
	weld_controller->hold_temp = weld_controller->weld_temp[1] * COMPENSATION_THRESHOLD;
	/*Steady-state estimation output (coefficients can be added here for correction)*/
	weld_controller->final_duty = ss.slope * weld_controller->weld_temp[1] + ss.intercept;
	if (weld_controller->final_duty > FINAL_DUTY_LIMIT)
		weld_controller->final_duty = FINAL_DUTY_LIMIT;

#if PID_DEBUG
	weld_controller->pid_ctrl->kp = pid_ctrl_debug->kp;
	weld_controller->pid_ctrl->ki = pid_ctrl_debug->ki;
	weld_controller->pid_ctrl->kd = pid_ctrl_debug->kd;
#else
	/*pid reload*/
	pid_dynamic_reload();
#endif

	/*real-time control*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[3])
	{
		/*temp sample*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		/*alarm*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[2])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}

		/*low temp alarm*/
		//		if (weld_controller->realtime_temp < weld_controller->second_step_start_temp + 10 &&
		//			weld_controller->realtime_temp > weld_controller->second_step_start_temp - 10 &&
		//			weld_controller->step_time_tick > 100)
		//		{
		//			stop_weld();
		//			err_get_type(err_ctrl, TEMP_DOWN)->state = true;
		//			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		//			OS_ERR err;
		//			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		//			break;
		//		}

		/*second step temp avg record*/
		if (weld_controller->realtime_temp >= second_temp_record_start &&
			temp_draw_ctrl->second_step_stable_index == 0)
		{
			if (weld_controller->pid_ctrl->stable_threshold_cnt >= weld_controller->pid_ctrl->stable_threshold)
				temp_draw_ctrl->second_step_stable_index = temp_draw_ctrl->current_index;
			else
				weld_controller->pid_ctrl->stable_threshold_cnt++;
		}

#if FAST_RISE_ENABLE
		/*fast rise step*/
		if (weld_controller->step_time_tick < FAST_RISE_TIME && weld_controller->Duty_Cycle < fast_rise_duty)
		{
			weld_controller->Duty_Cycle = fast_rise_duty;
		}
#endif

		/*temp rise step ---> restrict output*/
		if (weld_controller->realtime_temp < weld_controller->restrict_temp)
		{
			if (weld_controller->Duty_Cycle > weld_controller->restrict_duty)
				weld_controller->Duty_Cycle = weld_controller->restrict_duty;
		}

		/*enter transition area (heat compensation)*/
		if (weld_controller->realtime_temp > weld_controller->hold_temp)
		{
			/*only execute the code below one time*/
			if (weld_controller->enter_transition_flag == false &&
				weld_controller->enter_transition_time == 0)
			{
				weld_controller->enter_transition_flag = true;
				weld_controller->enter_transition_time = weld_controller->step_time_tick;
			}
		}

		/*in transition area hold the temp (heat compensation)*/
		if (weld_controller->enter_transition_flag == true)
		{
			current_hold_time = weld_controller->step_time_tick - weld_controller->enter_transition_time;
			if (current_hold_time < weld_controller->weld_time[2])
			{
				if (weld_controller->Duty_Cycle < weld_controller->final_duty)
					weld_controller->Duty_Cycle = weld_controller->final_duty;
			}
		}

		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			switch (cur_Pgae_id)
			{
			case WAVE_PAGE:
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
				command_set_comp_val("step3", "val", weld_controller->realtime_temp);
				break;
			case PARAM_PAGE:
				command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
				break;
			}
		}
#endif
	}

	/*timer reset*/
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	/*PWM off*/
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	/*end of step*/
	weld_controller->state = IDEAL_STATE;
	/*Record end coordinates*/
	temp_draw_ctrl->second_step_index_end = temp_draw_ctrl->current_index - 1;
}

static void Second_Step_KCB(void)
{
	Page_ID cur_Pgae_id = request_PGManger()->id;
	Steady_state_coefficient ss = weld_controller->ss_coefficient;
	uint16_t second_temp_record_start = weld_controller->weld_temp[1] * TEMP_AVG_SAMPLE_START;
	uint16_t current_hold_time = 0;

#if FAST_RISE_ENABLE
	uint16_t fast_rise_duty = 0;
	fast_rise_duty = ss.slope * weld_controller->weld_temp[1] + ss.intercept;
#endif

	/*enter second step*/
	weld_controller->state = SECOND_STATE;
	/*start sample*/
	temp_draw_ctrl->second_step_index_start = temp_draw_ctrl->current_index;
	temp_draw_ctrl->second_step_stable_index = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;
	/*heat compensation reset*/
	weld_controller->enter_transition_flag = false;
	weld_controller->enter_transition_time = 0;
	/*restrict param*/
	weld_controller->restrict_temp = weld_controller->weld_temp[1] * RESTRICT_TEMP_COFF;
	weld_controller->restrict_duty = PD_MAX * (0.1 + 0.9 * weld_controller->temp_gain2);
	/*hold temp*/
	weld_controller->hold_temp = weld_controller->weld_temp[1] * COMPENSATION_THRESHOLD;
	/*Steady-state estimation output (coefficients can be added here for correction)*/
	weld_controller->final_duty = ss.slope * weld_controller->weld_temp[1] + ss.intercept;
	if (weld_controller->final_duty > FINAL_DUTY_LIMIT)
		weld_controller->final_duty = FINAL_DUTY_LIMIT;

#if PID_DEBUG
	weld_controller->pid_ctrl->kp = pid_ctrl_debug->kp;
	weld_controller->pid_ctrl->ki = pid_ctrl_debug->ki;
	weld_controller->pid_ctrl->kd = pid_ctrl_debug->kd;
#else
	/*pid reload*/
	pid_dynamic_reload();
#endif

	/*real-time control*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[3])
	{
		/*temp sample*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		/*alarm*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[2])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}

		/*low temp alarm*/
		//		if (weld_controller->realtime_temp < weld_controller->second_step_start_temp + 10 &&
		//			weld_controller->realtime_temp > weld_controller->second_step_start_temp - 10 &&
		//			weld_controller->step_time_tick > 100)
		//		{
		//			stop_weld();
		//			err_get_type(err_ctrl, TEMP_DOWN)->state = true;
		//			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		//			OS_ERR err;
		//			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		//			break;
		//		}

		/*second step temp avg record*/
		if (weld_controller->realtime_temp >= second_temp_record_start &&
			temp_draw_ctrl->second_step_stable_index == 0)
		{
			if (weld_controller->pid_ctrl->stable_threshold_cnt >= weld_controller->pid_ctrl->stable_threshold)
				temp_draw_ctrl->second_step_stable_index = temp_draw_ctrl->current_index;
			else
				weld_controller->pid_ctrl->stable_threshold_cnt++;
		}

#if FAST_RISE_ENABLE
		/*fast rise step*/
		if (weld_controller->step_time_tick < FAST_RISE_TIME && weld_controller->Duty_Cycle < fast_rise_duty)
		{
			weld_controller->Duty_Cycle = fast_rise_duty;
		}
#endif

		/*temp rise step ---> restrict output*/
		if (weld_controller->realtime_temp < weld_controller->restrict_temp)
		{
			if (weld_controller->Duty_Cycle > weld_controller->restrict_duty)
				weld_controller->Duty_Cycle = weld_controller->restrict_duty;
		}

		/*enter transition area (heat compensation)*/
		if (weld_controller->realtime_temp > weld_controller->hold_temp)
		{
			/*only execute the code below one time*/
			if (weld_controller->enter_transition_flag == false &&
				weld_controller->enter_transition_time == 0)
			{
				weld_controller->enter_transition_flag = true;
				weld_controller->enter_transition_time = weld_controller->step_time_tick;
			}
		}

		/*in transition area hold the temp (heat compensation)*/
		if (weld_controller->enter_transition_flag == true)
		{
			current_hold_time = weld_controller->step_time_tick - weld_controller->enter_transition_time;
			if (current_hold_time < weld_controller->weld_time[2])
			{
				if (weld_controller->Duty_Cycle < weld_controller->final_duty)
					weld_controller->Duty_Cycle = weld_controller->final_duty;
			}
		}

		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			switch (cur_Pgae_id)
			{
			case WAVE_PAGE:
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
				command_set_comp_val("step3", "val", weld_controller->realtime_temp);
				break;
			case PARAM_PAGE:
				command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
				break;
			}
		}
#endif
	}

	/*timer reset*/
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	/*PWM off*/
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	/*end of step*/
	weld_controller->state = IDEAL_STATE;
	/*Record end coordinates*/
	temp_draw_ctrl->second_step_index_end = temp_draw_ctrl->current_index - 1;
}
