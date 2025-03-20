#include "welding_process.h"

#include "includes.h"
#include "key.h"
#include "pwm.h"
#include "pid.h"
#include "adc.h"
#include "timer.h"
#include "protect.h"
#include "spi.h"
#include "usart.h"
#include "Kalman.h"
#include "tempcomp.h"
#include "touchscreen.h"

/*---------------------------------------------------Real-time control---------------------------------------------------------*/
volatile WELD_MODE welding_flag = IDEAL_MODE; // Welding different stage markers
extern uint16_t kalman_comp_temp;			  // Temperature compensation value
extern weld_ctrl *weld_controller;			  // Welding controllers
extern double fitting_curves[3];			  // pid Parameters dynamically fit curves ax*bx+x+c
/*------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------Compatible touchscreen data interface------------------------------------------*/
extern Component_Queue *temp_page_list;	 // Queue in the UI of the Temperature Limit page
extern Component_Queue *param_page_list; // A list of components on the parameter setting screen
extern uint8_t ID_OF_MAS;
/*------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------False alarms--------------------------------------------------------------*/
extern OS_SEM ERROR_HANDLE_SEM;
extern Error_ctrl *err_ctrl;
/*------------------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------Control algorithms-----------------------------------------------------------*/
/*Kalman filtering*/
Kalman kfp;
dynamical_comp dynam_comp;
extern last_temp_sotre lasttemp;

/*-----------------------------------------------------temp plot-----------------------------------------------------------------*/
extern OS_SEM TEMP_DISPLAY_SEM;						 // Plot event signals
extern Temp_draw_ctrl *temp_draw_ctrl;				 // Drawing controllers
extern Page_Param *page_param;						 // Real-time page parameters
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN]; // Temperature preservation buffer
/*------------------------------------------------------------------------------------------------------------------------------*/

extern Thermocouple *current_Thermocouple; // The current thermocouple object
extern OS_SEM HOST_WELD_CTRL_SEM;		   // The upper computer turns on the welding signal

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
		ctrl->temp_gain1 = DEFAULT_GAIN;
		ctrl->temp_gain2 = DEFAULT_GAIN;

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

		ctrl->temp_gain1 = DEFAULT_GAIN;
		ctrl->temp_gain2 = DEFAULT_GAIN;
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

/*--------------------------------------------------------------------------------------------------------------------*/
/*													 Process Control API                                              */
/*--------------------------------------------------------------------------------------------------------------------*/

static void down_temp_line(void);
/**
 * @description: load data from eeprom
 * @return {*}
 */
static void Load_Data()
{
	uint8_t GP_Temp = 0;
	uint8_t key = 0;
	/*The scan button reads the corresponding parameter group data according to the startup mode and sends it to the touch screen*/
	GP_Temp = get_comp(param_page_list, "GP")->val;
	if ((GP_Temp % 2 == 0))
	{
		key = new_key_scan();
		if (key == KEY_PC1_PRES)
		{
			GP_Temp += 1;
			Load_param(weld_controller, GP_Temp);
			Load_param_alarm(weld_controller, GP_Temp);
			command_set_comp_val("GP", "val", GP_Temp);
		}
	}
	else
	{
		key = new_key_scan();
		if ((key == KEY_PC0_PRES))
		{
			if (GP_Temp != 0)
				GP_Temp -= 1;
			Load_param(weld_controller, GP_Temp);
			Load_param_alarm(weld_controller, GP_Temp);
			command_set_comp_val("GP", "val", GP_Temp);
		}
	}
}

/**
 * @description: The PID parameters as well as the pre-climb point can be controlled with the set of gain
 * @param {weld_ctrl} *ctrl
 * @return {*}
 */
static void ctrl_param_config(weld_ctrl *ctrl)
{

	/*动态调整曲线*/
	double set_gain = 0;
	double percent = 0;
	uint16_t delta_temp = 0;

	/*-------------------------------------------------------User-defined mode-------------------------------------------------------*/
	if (get_comp(temp_page_list, "switch")->val == 0)
	{
		switch (current_Thermocouple->type)
		{
		case K_TYPE:
		case J_TYPE:
		case E_TYPE:
			switch (ctrl->state)
			{
			case FIRST_STATE:
				/*一阶段参数*/
				ctrl->first_step_set = (weld_controller->temp_gain2 * 0.1 + 0.9) * (double)(ctrl->weld_temp[0] + STABLE_ERR); // 阶跃目标
				ctrl->first_step_turn = weld_controller->temp_gain1 * ctrl->first_step_set;									  // 刹车点
				break;
			case SECOND_STATE:
				/*二阶段参数*/
				ctrl->second_step_set = (weld_controller->temp_gain2 * 0.1 + 0.9) * (double)(ctrl->weld_temp[1] + STABLE_ERR); // 阶跃目标
				ctrl->second_step_turn = weld_controller->temp_gain1 * ctrl->second_step_set;								   // 刹车点
				break;
			case THIRD_STATE:
				break;
			}
			break;
		}
	}
	/*----------------------------------------------------------auto mode------------------------------------------------------------*/
	else
	{
		switch (current_Thermocouple->type)
		{
		case K_TYPE:
		case J_TYPE:
		case E_TYPE:
			switch (ctrl->state)
			{
			case FIRST_STATE:
				/*一阶段参数*/
				/*冷启动模式:冷启动防止升温不足，导致二阶段过冲*/
				if (ctrl->first_step_start_temp < ctrl->weld_temp[0] / 3)
				{
					ctrl->first_step_set = ctrl->first_step_start_temp + ctrl->weld_temp[0]; // 第一个设定点
					ctrl->first_step_turn = 0.95 * ctrl->weld_temp[0];						 // 刹车点
				}
				else
				{
					/*1、和设定点较近，改进pid，加速升温*/
					delta_temp = ctrl->weld_temp[0] - ctrl->first_step_start_temp;
					if (delta_temp < DELTA_COMPENSATE_MAX && delta_temp > DETTA_COMPENSATE_MIN)
					{

						ctrl->first_step_set = ctrl->first_step_start_temp + DELTA_COMPENSATE_MAX; // 第一个设定点（设大，为了快速性）
						ctrl->first_step_turn = 0.95 * ctrl->weld_temp[0];						   // 刹车点
					}
					/*2、离设定点较远或较近，无需改进*/
					else
					{
						/*根据起始温度调节设定值，起始温度越低，设定点就越低，防止过大的超调*/
						percent = (double)ctrl->first_step_start_temp / (double)ctrl->weld_temp[0]; // 起始温度对最终温度占比
						set_gain = 0.3969 * log(percent) + 1.0021;									// 参见excel表格
						if (set_gain <= 0)
							set_gain = 0;
						if (set_gain >= 1)
							set_gain = 1;

						/* 参数动态调整后，数据保存 */
						weld_controller->temp_gain1 = set_gain;
						get_comp(temp_page_list, "GAIN1")->val = set_gain * 100;

						ctrl->first_step_set = (0.9 + 0.1 * set_gain) * (double)ctrl->weld_temp[0]; // 第1个设定点(0.85-1)
						ctrl->first_step_turn = 0.95 * ctrl->weld_temp[0];							// 刹车点
					}
				}
				break;
			case SECOND_STATE:
/*二阶段参数*/
#if 1
				/*1、和设定点较近，改进pid，加速升温*/
				delta_temp = ctrl->weld_temp[1] - ctrl->second_step_start_temp;
				if (delta_temp < 0.8 * DELTA_COMPENSATE_MAX && delta_temp > DETTA_COMPENSATE_MIN)
				{
					ctrl->second_step_set = ctrl->second_step_start_temp + DELTA_COMPENSATE_MAX * 0.8; // 第1个阶跃目标（设大，为了快速性）
					ctrl->second_step_turn = 0.95 * ctrl->weld_temp[1];								   // 刹车点
				}
				/*2、离设定点较远或较近，无需改进*/
				else
				{
					/*根据起始温度调节设定值，起始温度越低，设定点就越低，防止过大的超调*/
					percent = (double)ctrl->second_step_start_temp / (double)ctrl->weld_temp[1]; // 起始温度对最终温度占比
					set_gain = 0.3969 * log(percent) + 1.0021;									 // 参见excel表格
					if (set_gain <= 0)
						set_gain = 0;
					if (set_gain >= 1)
						set_gain = 1;

					/*参数动态调整后，数据保存 */
					weld_controller->temp_gain2 = set_gain;
					get_comp(temp_page_list, "GAIN2")->val = set_gain * 100;

					ctrl->second_step_set = (set_gain * 0.1 + 0.9) * (double)(ctrl->weld_temp[1] + STABLE_ERR); // 阶跃目标
					ctrl->second_step_turn = 0.95 * ctrl->second_step_set;										// 刹车点
				}
#endif

#if 0
				/*根据起始温度调节设定值，起始温度越低，设定点就越低，防止过大的超调*/
				percent = (double)ctrl->second_step_start_temp / (double)ctrl->weld_temp[1]; // 起始温度对最终温度占比
				set_gain = 0.3969 * log(percent) + 1.0021;									 // 参见excel表格
				if (set_gain <= 0)
					set_gain = 0;
				if (set_gain >= 1)
					set_gain = 1;

				/*参数动态调整后，数据保存 */
				weld_controller->temp_gain2 = set_gain;
				get_comp(temp_page_list, "GAIN2")->val = set_gain * 100;

				ctrl->second_step_set = (set_gain * 0.1 + 0.9) * (double)(ctrl->weld_temp[1] + STABLE_ERR); // 阶跃目标
				ctrl->second_step_turn = 0.95 * ctrl->second_step_set;										// 刹车点
				break;
#endif

			case THIRD_STATE:
				break;
			}
			break;
		}
	}
}

/**
 * @description: Preparation before welding
 * @return {*}
 */
static void Weld_Preparation()
{

	/*表示进入焊接过程*/
	welding_flag = BUSY_MODE;
	/*时间刻度复位*/
	weld_controller->step_time_tick = 0; // 焊接周期计数值
	weld_controller->weld_time_tick = 0; // 单段焊接时间计数

	/*参数限制*/
	if (weld_controller->weld_time[1] > MAX_WELD_TIME)
		weld_controller->weld_time[1] = MAX_WELD_TIME;
	if (weld_controller->weld_time[2] > MAX_WELD_TIME)
		weld_controller->weld_time[2] = MAX_WELD_TIME;
	if (weld_controller->weld_time[3] > MAX_WELD_TIME)
		weld_controller->weld_time[3] = MAX_WELD_TIME;

	if (weld_controller->weld_temp[0] < USER_SET_MIN_TEMP)
		weld_controller->weld_temp[0] = USER_SET_MIN_TEMP;

	if (weld_controller->weld_temp[1] < USER_SET_MIN_TEMP)
		weld_controller->weld_temp[1] = USER_SET_MIN_TEMP;

	if (weld_controller->weld_temp[0] > USER_SET_MAX_TEMP)
		weld_controller->weld_temp[0] = USER_SET_MAX_TEMP;

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

	/*算法部分*/
	reset_forword_ctrl(weld_controller->pid_ctrl); // pid控制器初始化
	dynamic_init(&dynam_comp, 100);				   // 动态补偿初始化
	window_init(&lasttemp);						   // 滑动窗口初始化
	Kalman_Init(&kfp);							   // 卡尔曼滤波器初始化，初始值归零
	kalman_comp_temp = 0;						   // 卡尔曼估计值初始化

	/*pid*/
	pid_param_dynamic_reload(weld_controller, fitting_curves, weld_controller->first_step_set);

	/*IO控制*/
	OVER = 0;  // 1为焊接结束信号
	RLY10 = 1; // 气阀1启动
	RLY11 = 1; // 气阀2启动
	RLY12 = 1; // 气阀3启动
	CUNT = 0;  // 1为计数，0清除计数信号

	/*绘图部分*/
	reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);

	/*pwm部分*/
	uint16_t tmp_ccmr1 = 0; // 重新设定pwm模式
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

	// weld_ctrl_reset(weld_controller);
}
/**
 * @description: Pre-stress before welding
 * @return {*}
 */
static void Preload()
{

	/*焊接周期第一段，预压*/
	weld_controller->state = PRE_STATE;
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[0] + 1)
	{
		__nop();
		__nop();
	}
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	weld_controller->state = IDEAL_STATE;
	weld_controller->step_time_tick = 0;

/*预热阶段——加热到基值温度，改善超调*/
#if PRE_HEAT == 1
	u16 current_temp = temp_convert(current_Thermocouple);
	if (current_temp < BASE_TEMP)
	{
		/*根据起始温度设定加热时长到基值温度(线性映射)20：250 150：0*/
		u16 pre_heat_time = -0.77 * (float)current_temp + 115.4;
		if (pre_heat_time >= PRE_HEAT_MAX_TIME)
			pre_heat_time = PRE_HEAT_MAX_TIME;
		if (pre_heat_time <= PRE_HEAT_MIN_TIME)
			pre_heat_time = PRE_HEAT_MIN_TIME;

		weld_controller->state = PRE_STATE;
		weld_controller->step_time_tick = 0;
		TIM_Cmd(TIM5, ENABLE);
		while (weld_controller->step_time_tick < pre_heat_time)
		{
			TIM_SetCompare1(TIM1, PD_MAX * 0.8);
			TIM_SetCompare1(TIM4, PD_MAX * 0.8);
			/*实时温度显示*/
			if (page_param->id == WAVE_PAGE && weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
		}
		TIM_Cmd(TIM5, DISABLE);
		TIM5->CNT = 0;
		weld_controller->state = IDEAL_STATE;
		weld_controller->step_time_tick = 0;
	}
#endif
}

/**
 * @description: The first stage of welding, the temperature rises
 * @return {*}
 */
static void First_Step()
{

	weld_controller->state = FIRST_STATE; // 进入一阶段标志

	ctrl_param_config(weld_controller);															// 参数动态配置
	pid_param_dynamic_reload(weld_controller, fitting_curves, weld_controller->weld_temp[0]); // kp动态调整
	if ((page_param->key2 == ION) && (weld_controller->weld_time[1] != 0))					  // ION_IOF==0开PWM
	{
		/*1、一阶段采样开始*/
		temp_draw_ctrl->first_step_index_start = 0;
		/*3、实时控制部分*/

		weld_controller->Duty_Cycle = 0;	 // 占空比复位
		weld_controller->weld_time_tick = 0; // 总时长计数值复位
		TIM_Cmd(TIM3, ENABLE);				 // 焊接周期统计开启
		weld_controller->step_time_tick = 0; // 阶段计数值复位
		TIM_Cmd(TIM5, ENABLE);				 // 开启是实时控制器
		while (weld_controller->step_time_tick < weld_controller->weld_time[1])
		{

			weld_controller->realtime_temp = temp_convert(current_Thermocouple);
			if (weld_controller->realtime_temp >= weld_controller->first_step_turn && weld_controller->pid_ctrl->stable_flag == false)
			{
				weld_controller->pid_ctrl->stable_flag = true;
			}

			/*失温报警*/
			if (weld_controller->realtime_temp > weld_controller->alarm_temp[0])
			{
				stop_weld();
				err_get_type(err_ctrl, TEMP_UP)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
				break;
			}
			if (weld_controller->realtime_temp > weld_controller->weld_time[1] * 0.5)
			{
				if (weld_controller->realtime_temp < weld_controller->first_step_start_temp)
				{
					stop_weld();
					err_get_type(err_ctrl, TEMP_DOWN)->state = true;
					OS_ERR err;
					OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
					break;
				}
			}

#if REALTIME_TEMP_DISPLAY == 1

			/*实时温度显示*/
			if (page_param->id == WAVE_PAGE && weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
#endif
		}

		/*4、第一段结束后，将一些标志位清零*/
		TIM_Cmd(TIM5, DISABLE);													  // 关闭实时控制器
		TIM5->CNT = 0;															  // 计数值复位
		TIM_SetCompare1(TIM1, 0);												  // 关闭pwm 输出
		TIM_SetCompare1(TIM4, 0);												  // 关闭pwm 输出
		weld_controller->step_time_tick = 0;									  // 实时控制器阶段性时间刻度复位
		weld_controller->state = IDEAL_STATE;									  // 焊接状态复位
		temp_draw_ctrl->first_step_index_end = temp_draw_ctrl->current_index - 1; // 记录一阶段结束绘图坐标
	}
}

/**
 * @description: In the second stage of welding, the temperature is maintained
 * @return {*}
 */
static void Second_Step()
{
	weld_controller->state = SECOND_STATE; // 进入二阶段

	ctrl_param_config(weld_controller);															 // 参数动态配置
	pid_param_dynamic_reload(weld_controller, fitting_curves, weld_controller->weld_temp[1]); // kp动态调整
	if ((page_param->key2 == ION) && (weld_controller->weld_time[2] != 0))
	{
		/*1、进入二阶段采样*/
		temp_draw_ctrl->second_step_index_start = temp_draw_ctrl->current_index;
		/*2、实时控制部分*/

		weld_controller->step_time_tick = 0; // 计数值复位
		TIM_Cmd(TIM5, ENABLE);				 // 开启实时控制器
		while (weld_controller->step_time_tick < weld_controller->weld_time[2])
		{
			weld_controller->realtime_temp = temp_convert(current_Thermocouple);
			/*到达刹车点，转阶段*/
			if (weld_controller->realtime_temp >= weld_controller->second_step_turn && weld_controller->pid_ctrl->stable_flag == false)
			{
				weld_controller->pid_ctrl->stable_flag = true;
			}

			/*后续计算稳定温度值索引*/
			if (weld_controller->realtime_temp >= 0.98 * weld_controller->weld_temp[1] && temp_draw_ctrl->second_step_stable_index == 0)
			{
				if (weld_controller->pid_ctrl->stable_threshold_cnt >= weld_controller->pid_ctrl->stable_threshold)
					temp_draw_ctrl->second_step_stable_index = temp_draw_ctrl->current_index;
				else
					weld_controller->pid_ctrl->stable_threshold_cnt++;
			}

			/*失温报警*/
			if (weld_controller->realtime_temp > weld_controller->alarm_temp[2])
			{
				stop_weld();
				err_get_type(err_ctrl, TEMP_UP)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
				break;
			}
			if (weld_controller->realtime_temp > weld_controller->weld_time[2] * 0.5)
			{
				if (weld_controller->realtime_temp < weld_controller->second_step_start_temp)
				{
					stop_weld();
					err_get_type(err_ctrl, TEMP_DOWN)->state = true;
					OS_ERR err;
					OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
					break;
				}
			}

#if REALTIME_TEMP_DISPLAY == 1
			/*实时温度显示*/
			if (page_param->id == WAVE_PAGE && weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
#endif
		}
		/*3、第二段结束后，对变量进行复位*/
		TIM_Cmd(TIM5, DISABLE);														// 关闭实时控制器
		TIM5->CNT = 0;																// 计数值复位
		TIM_SetCompare1(TIM1, 0);													// 关闭pwm 输出
		TIM_SetCompare1(TIM4, 0);													// 关闭pwm 输出
		weld_controller->step_time_tick = 0;										// 计数值复位
		weld_controller->third_step_start_duty_cycle = weld_controller->Duty_Cycle; // 三阶段从这个占空比往下下降
		weld_controller->state = IDEAL_STATE;										// 焊接状态复位
		temp_draw_ctrl->second_step_index_end = temp_draw_ctrl->current_index - 1;	// 二阶段结束，记录绘图坐标
	}
}

/**
 * @description: The third stage of welding, the temperature drops
 * @return {*}
 */
static void Third_Step()
{
	weld_controller->state = THIRD_STATE;													  // 进入二阶段
	pid_param_dynamic_reload(weld_controller, fitting_curves, weld_controller->weld_temp[2]); // kp动态调整
	if ((page_param->key2 == ION) && (weld_controller->weld_time[3] != 0))
	{
		/*1、三阶段采样开始，坐标记录*/
		temp_draw_ctrl->third_step_index_start = temp_draw_ctrl->current_index;
		/*2、实时控制部分*/
		weld_controller->step_time_tick = 0; // 阶段计数值复位
		TIM_Cmd(TIM5, ENABLE);				 // 开启实时控制器
		while (weld_controller->step_time_tick < weld_controller->weld_time[3])
		{
			weld_controller->realtime_temp = temp_convert(current_Thermocouple);
			if (weld_controller->realtime_temp > weld_controller->alarm_temp[4])
			{
				stop_weld();
				err_get_type(err_ctrl, TEMP_UP)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
				break;
			}

#if REALTIME_TEMP_DISPLAY == 1
			/*实时温度显示*/
			if (page_param->id == WAVE_PAGE && weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
				draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
#endif
		}

		/*3、第三段结束*/

		TIM_Cmd(TIM5, DISABLE);													  // 关闭实时控制器
		TIM5->CNT = 0;															  // 计数值复位
		TIM_SetCompare1(TIM1, 0);												  // 关闭pwm 输出
		TIM_SetCompare1(TIM4, 0);												  // 关闭pwm 输出
		weld_controller->step_time_tick = 0;									  // 阶段计数值复位
		weld_controller->state = IDEAL_STATE;									  // 焊接状态复位
		weld_controller->Duty_Cycle = 0;										  // 占空比复位
		temp_draw_ctrl->third_step_index_end = temp_draw_ctrl->current_index - 1; // 三阶段结束，绘图坐标记录
	}
}

/**
 * @description: End welding and reset parameters
 * @return {*}
 */
static void End_of_Weld()
{

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
	/*根据计数模式更新焊接计数值*/
	if (get_comp(param_page_list, "UP_DOWN")->val == UP_CNT)
		weld_controller->weld_count++;
	else if (get_comp(param_page_list, "UP_DOWN")->val == DOWN_CNT && weld_controller->weld_count > 0)
		weld_controller->weld_count--;
	/*计数值更新*/
	command_set_comp_val("param_page.count", "val", weld_controller->weld_count);
	command_set_comp_val("temp_page.count", "val", weld_controller->weld_count);

	RLY10 = 0; // 气阀1关闭
	RLY11 = 0; // 气阀2关闭
	RLY12 = 0; // 气阀3关闭
	CUNT = 1;  // 1为计数，0清除计数信号
	OVER = 1;  // 1为焊接结束信号
}

#if COMMUNICATE == 1

/**
 * @description: Transmit data to the host computer
 * @param {uint16_t} Temp_Send
 * @return {*}
 */
static void data_transfer_to_computer(const uint16_t Temp_Send[])
{
	// 调试时暂时注释掉，防止对温度曲线进行干扰
	//  发送各段焊接温度给上位机
	unsigned char output_welding_data[15] = {0x00};
	output_welding_data[0] = ID_OF_MAS;							/* 焊机号 */
	output_welding_data[1] = 0x0f;								/* 指令码 */
	output_welding_data[2] = 0x00;								/*发送字节数 */
	output_welding_data[3] = weld_controller->weld_count / 256; /* 焊接计数值高字节 */
	output_welding_data[4] = weld_controller->weld_count % 256; /* 焊接计数低字节 */
	output_welding_data[5] = Temp_Send[0] / 256;
	output_welding_data[6] = Temp_Send[0] % 256;
	output_welding_data[7] = Temp_Send[1] / 256;
	output_welding_data[8] = Temp_Send[1] % 256;
	output_welding_data[9] = Temp_Send[2] / 256;
	output_welding_data[10] = Temp_Send[2] % 256;
	int crc16_data = CRC16(output_welding_data, 11);
	output_welding_data[11] = crc16_data >> 8;
	output_welding_data[12] = crc16_data & 0x00ff;
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 13; t1++)
	{
		USART3->SR;
		USART_SendData(USART3, output_welding_data[t1]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 0; // 设置为接收模式
}

#endif

static void simulate_weld()
{

	/*IO控制*/
	RLY10 = 1; // 气阀1启动
	RLY11 = 1; // 气阀2启动
	RLY12 = 1; // 气阀3启动
	CUNT = 0;  // 1为计数，0清除计数信号
	OVER = 0;  // 1为焊接结束信号

	weld_controller->step_time_tick = 0; // 实时控制器阶段性时间刻度复位
	weld_controller->weld_time_tick = 0; // 焊接周期时间刻度复位
	TIM_Cmd(TIM3, ENABLE);				 // 开启时间统计计数器，同时开启温度采集
	while (weld_controller->weld_time_tick < weld_controller->weld_time[0] + weld_controller->weld_time[1] + weld_controller->weld_time[2] + weld_controller->weld_time[3])
		;
	TIM_Cmd(TIM3, DISABLE);				 // 关闭时间统计计数器
	weld_controller->weld_time_tick = 0; // 焊接周期时间刻度复位

	RLY10 = 0; // 气阀1关闭
	RLY11 = 0; // 气阀2关闭
	RLY12 = 0; // 气阀3关闭
	CUNT = 1;  // 1为计数，0清除计数信号
	OVER = 1;  // 1为焊接结束信号

	/*+++根据计数模式更新计数值+++*/
	if (get_comp(param_page_list, "UP_DOWN")->val == UP_CNT)
		weld_controller->weld_count++;
	else if (get_comp(param_page_list, "UP_DOWN")->val == DOWN_CNT && weld_controller->weld_count > 0)
		weld_controller->weld_count--;

	command_set_comp_val("param_page.count", "val", weld_controller->weld_count);
	command_set_comp_val("temp_page.count", "val", weld_controller->weld_count);
}

/**
 * @description: 焊接实时控制
 * @return {*}
 */
static void weld_real_time_ctrl()
{

	OS_ERR err;
	/*变量初始化以及相关初始化工作：控制器复位、初始化、缓存擦除、参数限幅*/
	Weld_Preparation();

	/*预压*/
	Preload();

	/*一阶段*/
	if (weld_controller->weld_time[1] != 0)
	{
		err_cnt_clear(err_ctrl);													 // 报错统计值复位
		weld_controller->first_step_start_temp = temp_convert(current_Thermocouple); // 起始温度
		First_Step();																 // 一阶段
		if (true == err_occur(err_ctrl))											 // 唤醒错误处理线程
			goto STOP_LABEL;
	}

	/*二阶段*/
	if (weld_controller->weld_time[2] != 0)
	{

		err_cnt_clear(err_ctrl);													  // 报错统计值复位
		weld_controller->second_step_start_temp = temp_convert(current_Thermocouple); // 起始温度
		Second_Step();																  // 二阶段
		if (true == err_occur(err_ctrl))											  // 唤醒错误处理线程
			goto STOP_LABEL;
	}

	/*三阶段*/
	if (weld_controller->weld_time[3] != 0)
	{
		err_cnt_clear(err_ctrl);													 // 报错统计值复位
		weld_controller->third_step_start_temp = temp_convert(current_Thermocouple); // 起始温度
		Third_Step();																 // 三阶段
		if (true == err_occur(err_ctrl))											 // 唤醒错误处理线程
			goto STOP_LABEL;
	}

	/*结束一轮焊接*/
	End_of_Weld();

STOP_LABEL:
	if (true == err_occur(err_ctrl))
	{
		stop_weld();
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
}

static void Timer_Pre_Init()
{
	TIM1_PWM_Init();
	TIM4_PWM_Init();
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
	TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);

	TIM_Cmd(TIM3, DISABLE);						// 焊接周期计数器关闭
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志位
	TIM_Cmd(TIM5, DISABLE);						// 中实时控制器关闭
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); // 清除中断标志位
}
/**
 * @description: Welding real-time control portal
 * @return {*}
 */
void welding_process(void)
{
	/*向下计数模式到底，停止焊接*/
	if (weld_controller->weld_count == 0 && get_comp(param_page_list, "UP_DOWN")->val == DOWN_CNT)
		return;

	/*定时器配置PWM 5KHz*/
	Timer_Pre_Init();
	/*扫描按键，根据不同按键加载不同参数组别*/
	Load_Data();

	/*----------------------------------------------------------------------------------------------------------------------------------------------*/
	/*----------------------------------------------------------------焊接实时控制部分---------------------------------------------------------------*/
	/*----------------------------------------------------------------------------------------------------------------------------------------------*/

	/*-------------------------------------------------------连点模式-------------------------------------------------------*/
	if (page_param->key3 == CTW && page_param->key2 == ION)
	{
		OS_ERR err;
		uint8_t key = 0;
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		key = new_key_scan();
		/*不松开脚踏就进入持续焊接模式*/
		while (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{
			if (true == err_occur(err_ctrl))
				break;

			/*焊前擦除上次温度显示*/
			if (page_param->id == WAVE_PAGE)
			{
				command_send("cle wave_line.id,0");
				OSTimeDly(2, OS_OPT_TIME_DLY, &err);
				command_send("cle wave_line.id,0");
				OSTimeDly(2, OS_OPT_TIME_DLY, &err);
			}
			/*进入焊接的条件*/
			if (page_param->key1 != RDY || weld_controller->realtime_temp > weld_controller->weld_temp[2])
				return;

			/*实时焊接控制*/
			weld_real_time_ctrl();

			/*绘制降温曲线*/
			down_temp_line();

			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DISPLAY_SEM, OS_OPT_POST_ALL, &err);

			/*焊接间隔*/
			OVER = 0;
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);

			/*循环焊接*/
			key = new_key_scan();
			if (!(key == KEY_PC1_PRES || key == KEY_PC0_PRES))
				break;
		}
	}
	/*模拟焊接*/
	else if (page_param->key3 == CTW && page_param->key2 == IOFF)
	{
		OS_ERR err;
		uint8_t key = 0;
		key = new_key_scan();
		while (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{

			simulate_weld();
			/*设置连续焊接间隔——同时也腾出时间片*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DISPLAY_SEM, OS_OPT_POST_ALL, &err);

			key = new_key_scan();
			if (!(key == KEY_PC1_PRES || key == KEY_PC0_PRES))
				break;
		}
	}

	/*-------------------------------------------------------单点模式-------------------------------------------------------*/
	if (page_param->key3 == SGW && page_param->key2 == ION)
	{
		OS_ERR err;
		uint8_t key = 0;
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		key = new_key_scan();
		if (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{
			/*焊前擦除上次温度显示*/
			if (page_param->id == WAVE_PAGE)
			{
				command_send("cle wave_line.id,0");
				OSTimeDly(2, OS_OPT_TIME_DLY, &err);
				command_send("cle wave_line.id,0");
				OSTimeDly(2, OS_OPT_TIME_DLY, &err);
			}

			/*进入焊接的条件*/
			if (page_param->key1 != RDY || weld_controller->realtime_temp > weld_controller->weld_temp[2])
				return;

			/*实时焊接控制*/
			weld_real_time_ctrl();

			/*绘制降温曲线*/
			down_temp_line();

			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DISPLAY_SEM, OS_OPT_POST_ALL, &err);

			OVER = 0;
			/*设置焊接间隔*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);

			/*等待释放按钮*/
			while (1)
			{
				key = new_key_scan();
				if (key != KEY_PC1_PRES && key != KEY_PC0_PRES)
					break;
				OSTimeDly(10, OS_OPT_TIME_DLY, &err);
			}
		}

#if HOST_WELD_CTRL == 1
		/*485启动模式*/
		OSSemPend(&HOST_WELD_CTRL_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			/*焊前擦除上次温度显示*/
			if (page_param->id == WAVE_PAGE)
			{
				command_send("cle wave_line.id,0");
				OSTimeDly(2, OS_OPT_TIME_DLY, &err);
				command_send("cle wave_line.id,0");
				OSTimeDly(2, OS_OPT_TIME_DLY, &err);
			}

			/*进入焊接的条件*/
			if (page_param->key1 != RDY || weld_controller->realtime_temp > weld_controller->weld_temp[2])
				return;

			/*实时焊接控制*/
			weld_real_time_ctrl();

			/*绘制降温曲线*/
			down_temp_line();

			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DISPLAY_SEM, OS_OPT_POST_ALL, &err);

			OVER = 0;
			/*设置焊接间隔*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
		}

#endif
	}
	/*模拟焊接*/
	else if (page_param->key3 == SGW && page_param->key2 == IOFF)
	{
		OS_ERR err;
		uint8_t key = 0;
		key = new_key_scan();
		if (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{

			simulate_weld();
			/*设置连续焊接间隔——同时也腾出时间片*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DISPLAY_SEM, OS_OPT_POST_ALL, &err);
			OVER = 0;
			/*设置焊接间隔*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
		}
	}
}

static void down_temp_line()
{
	uint16_t index = 0;
	uint16_t weld_win_width = 0;
	uint16_t win_width = 0;
	uint16_t total_time = 0;
	uint16_t temp = 0;
	uint8_t temp_display = 0;

	for (uint8_t i = 0; i < 5; i++)
		total_time += weld_controller->weld_time[i];

	weld_win_width = total_time / (temp_draw_ctrl->delta_tick - 1); // 转换计算(参见读写线程的坐标绘制)：焊接周期绘图区域大小
	win_width = WIN_WIDTH - weld_win_width - DRAW_RESERVE;			// 温降曲线绘图区域大小（留一个余量）
	if (win_width >= WIN_WIDTH / 2)
		win_width = WIN_WIDTH / 2;
	while (index < win_width)
	{
		if (get_weld_flag() == BUSY_MODE) // 非焊接状态才进行绘制
			break;
		temp = temp_convert(current_Thermocouple); // 温度采样
		if (temp > MAX_TEMP_DISPLAY)			   // 限幅
			temp = MAX_TEMP_DISPLAY;

		temp_display = temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY; // 坐标放缩
		draw_point(temp_display);								 // 绘图
		command_set_comp_val("temp33", "val", temp_display);	 // 显示实时温度数值
		delay_ms(temp_draw_ctrl->delta_tick);					 // 采样间隔

		index++;
	}
}
