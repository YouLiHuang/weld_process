#include "user_config.h"
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
#include "touchscreen.h"
#include "dynamic_correct.h"

uint8_t err_comp;
#if PID_DEBUG
extern pid_feedforword_ctrl *pid_ctrl_debug;
extern pid_feedforword_ctrl *pid_ctrl;
#endif

/*---------------------------------------------------Real-time control---------------------------------------------------------*/
volatile WELD_MODE welding_flag = IDEAL_MODE;  // Welding different stage markers
extern weld_ctrl *weld_controller;			   // Welding controllers
Correction_factor corrct_factor = {0.1, 1.25}; // Steady-state fitting curve correction coefficient
/*------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------Compatible touchscreen data interface------------------------------------------*/
extern Component_Queue *temp_page_list;	 // Queue in the UI of the Temperature Limit page
extern Component_Queue *param_page_list; // A list of components on the parameter setting screen
extern uint8_t ID_OF_MAS;
/*------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------False alarms--------------------------------------------------------------*/
extern OS_SEM ERROR_HANDLE_SEM;
extern Error_ctrl *err_ctrl;
/*-------------------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------Control algorithms-----------------------------------------------------------*/
#if COMPENSATION
#include "tempcomp.h"
extern last_temp_sotre lasttemp;
dynamical_comp dynam_comp;
#endif

#if KALMAN_FILTER
#include "Kalman.h"
extern uint16_t kalman_comp_temp; // Temperature compensation value
Kalman kfp;
#endif

/*-----------------------------------------------------temp plot-----------------------------------------------------------------*/
extern OS_SEM TEMP_DISPLAY_SEM;						 // Plot event signals
extern Temp_draw_ctrl *temp_draw_ctrl;				 // Drawing controllers
extern Page_Param *page_param;						 // Real-time page parameters
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN]; // Temperature preservation buffer
/*--------------------------------------------------------------------------------------------------------------------------------*/

extern Thermocouple *current_Thermocouple; // The current thermocouple object
extern OS_SEM HOST_WELD_CTRL_SEM;		   // The upper computer turns on the welding signal

/*-------------------------------------------------------------- USB---------------------------------------------------------------*/
extern OS_SEM DATA_SAVE_SEM;
/*--------------------------------------------------------------------------------------------------------------------------------*/

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

		ctrl->ss_coefficient.slope = 1.3;
		ctrl->ss_coefficient.intercept = 1060;

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

#if 0
static pid_fitting_curve fitting_curves = {0.0002, -0.23, 76}; // pid Parameters dynamically fit curves ax*bx+x+c

/**
 * @description: Dynamically adjust pid parameters according to set values
 * @param {void} *controller
 * @param {double} *fitting_curves
 * @param {uint16_t} setting
 * @return {*}
 */
static void pid_param_dynamic_reload(void *controller, pid_fitting_curve fitting_curves, uint16_t setting)
{

	weld_ctrl *ctrl = (weld_ctrl *)controller;
	ctrl->pid_ctrl->stable_flag = false;
	uint8_t new_kd = 0;
	switch (ctrl->state)
	{
	case PRE_STATE:
		break;
	case FIRST_STATE:
		new_kd = fitting_curves.a * setting * setting +
				 fitting_curves.b * setting +
				 fitting_curves.c;

		if (new_kd > 25)
			new_kd = 25;
		if (new_kd <= 10)
			new_kd = 10;

		ctrl->pid_ctrl->kp = 15;
		ctrl->pid_ctrl->ki = 0.028;
		ctrl->pid_ctrl->kd = new_kd;
		break;

	case SECOND_STATE:
		if (ctrl->weld_time[1] <= 100)
		{
			new_kd = fitting_curves.a * setting * setting +
					 fitting_curves.b * setting +
					 fitting_curves.c;

			if (new_kd > 25)
				new_kd = 25;
			if (new_kd <= 10)
				new_kd = 10;

			ctrl->pid_ctrl->kp = 15;
			ctrl->pid_ctrl->ki = 0.028;
			ctrl->pid_ctrl->kd = new_kd;
		}
		else
		{
			uint16_t delta_temp = ctrl->weld_temp[1] - ctrl->second_step_start_temp;
			new_kd = fitting_curves.a * delta_temp * delta_temp +
					 fitting_curves.b * delta_temp +
					 fitting_curves.c;

			if (new_kd > 25)
				new_kd = 25;
			if (new_kd <= 10)
				new_kd = 10;

			ctrl->pid_ctrl->kp = 15;
			ctrl->pid_ctrl->ki = 0.025;
			ctrl->pid_ctrl->kd = new_kd;
		}

		/*稳态标志复位*/
		weld_controller->pid_ctrl->stable_flag = false;

		break;
	}
}

#endif

/**
 * @description: Dynamically adjust pid parameters according to different step
 * @return {*}
 */
static void pid_dynamic_reload(void)
{
	/*two step*/
	if (weld_controller->weld_time[1] > 100 && weld_controller->weld_time[2] > 0)
	{
		switch (weld_controller->state)
		{
		case FIRST_STATE:
			weld_controller->pid_ctrl->kp = DEFAULT_KP;
			weld_controller->pid_ctrl->ki = DEFAULT_KI;
			weld_controller->pid_ctrl->kd = DEFAULT_KD;
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
}

/*--------------------------------------------------------------------------------------------------------------------*/
/*													 Process Control API                                              */
/*--------------------------------------------------------------------------------------------------------------------*/

static void down_temp_line()
{
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

	while (index < win_width)
	{
		if (get_weld_flag() == BUSY_MODE)
			break;

		/*stop draw start another weld*/
		key = new_key_scan();
		if (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
			break;

		temp = temp_convert(current_Thermocouple); // temp sample
		if (temp > MAX_TEMP_DISPLAY)			   // limit
			temp = MAX_TEMP_DISPLAY;

		temp_display = temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY;			  // Coordinate scaling
		draw_point(temp_display);											  // draw
		command_set_comp_val("step3", "val", weld_controller->realtime_temp); // display realtime temp
		delay_ms(temp_draw_ctrl->delta_tick);								  // Sampling interval
		index++;
	}
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
		if (key == KEY_PC0_PRES)
		{
			if (GP_Temp != 0)
			{
				GP_Temp -= 1;
				Load_param(weld_controller, GP_Temp);
				Load_param_alarm(weld_controller, GP_Temp);
			}
			command_set_comp_val("GP", "val", GP_Temp);
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
	/*----------------------------------------时间刻度复位----------------------------------------*/
	weld_controller->step_time_tick = 0; // 焊接周期计数值
	weld_controller->weld_time_tick = 0; // 单段焊接时间计数

	/*------------------------------------------参数限制------------------------------------------*/
	if (weld_controller->weld_time[1] > 999)
		weld_controller->weld_time[1] = 999;
	if (weld_controller->weld_time[2] > 9999)
		weld_controller->weld_time[2] = 9999;
	if (weld_controller->weld_time[3] > 999)
		weld_controller->weld_time[3] = 999;

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

	err_comp = 0.2 * weld_controller->weld_temp[0] - 30;

	/*------------------------------------------绘图部分------------------------------------------*/
	reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);

	/*------------------------------------------IO控制-------------------------------------------*/
	OVER = 0;  // 1为焊接结束信号
	RLY10 = 1; // 气阀1启动
	RLY11 = 1; // 气阀2启动
	RLY12 = 1; // 气阀3启动
	CUNT = 0;  // 1为计数，0清除计数信号

	/*------------------------------------------pwm部分-------------------------------------------*/
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
	Steady_state_coefficient ss = weld_controller->ss_coefficient;

	/*enter first step*/
	weld_controller->state = FIRST_STATE;
	/*start sample*/
	temp_draw_ctrl->first_step_index_start = 0;
	/*reset cotroller*/
	weld_controller->Duty_Cycle = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;
	/*Heat compensation*/
	weld_controller->enter_transition_flag = false;
	weld_controller->enter_transition_time = 0;
	/*Steady-state estimation output (coefficients can be added here for correction)*/
	weld_controller->final_duty = (corrct_factor.base + corrct_factor.amplitude * weld_controller->temp_gain2) *
								  (ss.slope * weld_controller->weld_temp[0] + ss.intercept);

#if PID_DEBUG
	weld_controller->pid_ctrl = pid_ctrl;
#else
	/*pid reload*/
	pid_dynamic_reload();
#endif

	/*real-time control*/
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[1])
	{
		/*alarm*/
		// over temp
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[0])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}
		// low temp
		if (weld_controller->realtime_temp < weld_controller->first_step_start_temp - 10)
		{
			if (err_ctrl->temp_low_cnt++ > err_ctrl->temp_low_threshold)
			{
				stop_weld();
				err_get_type(err_ctrl, TEMP_DOWN)->state = true;
				err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
				break;
			}
		}

		/*limit output*/
		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (page_param->id == WAVE_PAGE && weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
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

/**
 * @description: In the second stage of welding, the temperature is maintained
 * @return {*}
 */
static void Second_Step()
{

	/*---user config param---*/
	uint16_t hold_time = (TRANSITION_TIME_BASE + TRANSITION_TIME_CORRECT * weld_controller->temp_gain1) * TRANSITION_TIME;
	uint16_t current_hold_time = 0;
	uint16_t fast_rise_duty = 0;
	Steady_state_coefficient ss = weld_controller->ss_coefficient;

	/*enter first step*/
	weld_controller->state = SECOND_STATE;
	/*start sample*/
	temp_draw_ctrl->second_step_index_start = temp_draw_ctrl->current_index;
	temp_draw_ctrl->second_step_stable_index = 0;
	/*reset timer*/
	weld_controller->step_time_tick = 0;
	/*heat compensation reset*/
	weld_controller->enter_transition_flag = false;
	weld_controller->enter_transition_time = 0;
	/*Steady-state estimation output (coefficients can be added here for correction)*/
	weld_controller->final_duty = (corrct_factor.base + corrct_factor.amplitude * weld_controller->temp_gain2) *
								  (ss.slope * weld_controller->weld_temp[1] + ss.intercept);
	fast_rise_duty = ss.slope * weld_controller->weld_temp[1] + ss.intercept;

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
	while (weld_controller->step_time_tick < weld_controller->weld_time[2])
	{
		/*alarm*/
		if (weld_controller->realtime_temp > weld_controller->alarm_temp[2])
		{
			stop_weld();
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
			break;
		}
		/*second step temp avg*/
		if (weld_controller->realtime_temp >= weld_controller->weld_temp[1] * 0.95 && temp_draw_ctrl->second_step_stable_index == 0)
		{
			if (weld_controller->pid_ctrl->stable_threshold_cnt >= weld_controller->pid_ctrl->stable_threshold)
				temp_draw_ctrl->second_step_stable_index = temp_draw_ctrl->current_index;
			else
				weld_controller->pid_ctrl->stable_threshold_cnt++;
		}

#if PID_DEBUG == 0
		/*enter transition area (heat compensation)*/
		weld_controller->realtime_temp = temp_convert(current_Thermocouple);
		if (weld_controller->realtime_temp > COMPENSATION_THRESHOLD * weld_controller->weld_temp[1])
		{
			/*only execute the code below one time*/
			if (weld_controller->enter_transition_flag == false && weld_controller->enter_transition_time == 0)
			{
				weld_controller->enter_transition_flag = true;
				weld_controller->enter_transition_time = weld_controller->step_time_tick;
			}
		}

		/*in transition area hold the temp (heat compensation)*/
		if (weld_controller->enter_transition_flag == true)
		{
			current_hold_time = weld_controller->step_time_tick - weld_controller->enter_transition_time;
			if (current_hold_time < hold_time)
			{
				if (weld_controller->Duty_Cycle < weld_controller->final_duty)
					weld_controller->Duty_Cycle = weld_controller->final_duty;
			}
		}

		/*fast rise step*/
		if (weld_controller->step_time_tick < FAST_RISE_TIME && weld_controller->Duty_Cycle < fast_rise_duty)
		{
			weld_controller->Duty_Cycle = fast_rise_duty;
		}
#endif

		/*limit output*/
		if (weld_controller->Duty_Cycle > PD_MAX)
			weld_controller->Duty_Cycle = PD_MAX;
		TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
		TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

#if REALTIME_TEMP_DISPLAY == 1
		if (page_param->id == WAVE_PAGE && weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
		{
			draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
			// command_set_comp_val("step2", "val", weld_controller->realtime_temp);
		}

#endif
	}

	/*timer reset*/
	TIM_Cmd(TIM5, DISABLE);
	TIM5->CNT = 0;
	/*end of step*/
	weld_controller->state = IDEAL_STATE;
	/*Record end coordinates*/
	temp_draw_ctrl->second_step_index_end = temp_draw_ctrl->current_index - 1;
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
	while (weld_controller->step_time_tick < weld_controller->weld_time[3])
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
		if (page_param->id == WAVE_PAGE && weld_controller->step_time_tick % temp_draw_ctrl->delta_tick == 0)
			draw_point(weld_controller->realtime_temp * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
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
	if (weld_controller->Count_Dir == UP)
		weld_controller->weld_count++;
	if (weld_controller->Count_Dir == DOWN && weld_controller->weld_count > 0)
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

#if COMMUNICATE

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

/**
 * @description: Conduct a simulation of welding with no actual current output
 * @return {*}
 */
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
	if (weld_controller->Count_Dir == UP)
		weld_controller->weld_count++;
	else if (weld_controller->Count_Dir == DOWN && weld_controller->weld_count > 0)
		weld_controller->weld_count--;

	command_set_comp_val("param_page.count", "val", weld_controller->weld_count);
	command_set_comp_val("temp_page.count", "val", weld_controller->weld_count);
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

	/*开启焊接*/
	start_of_weld();

	/*预压*/
	Preload();
	/*一阶段*/
	if (weld_controller->weld_time[1] != 0)
	{
		weld_controller->first_step_start_temp = temp_convert(current_Thermocouple); // 起始温度
		First_Step();																 // 一阶段
		if (true == err_occur(err_ctrl))											 // 唤醒错误处理线程
			goto STOP_LABEL;
	}

	/*二阶段*/
	if (weld_controller->weld_time[2] != 0)
	{
		reset_forword_ctrl(weld_controller->pid_ctrl);
#if PID_DEBUG
		reset_forword_ctrl(pid_ctrl_debug);
		reset_forword_ctrl(weld_controller->pid_ctrl);
#endif
		weld_controller->second_step_start_temp = temp_convert(current_Thermocouple); // 起始温度
		Second_Step();																  // 二阶段
		if (true == err_occur(err_ctrl))											  // 唤醒错误处理线程
			goto STOP_LABEL;
	}

	/*三阶段*/
	if (weld_controller->weld_time[3] != 0)
	{
		weld_controller->third_step_start_temp = temp_convert(current_Thermocouple); // 起始温度
		Third_Step();																 // 三阶段
		if (true == err_occur(err_ctrl))											 // 唤醒错误处理线程
			goto STOP_LABEL;
	}

	/*结束一轮焊接*/
	End_of_Weld();

	/*动态修正参数*/
	dynamic_pwm_adjust();							  // final pwm dynamic
	if (get_comp(temp_page_list, "switch")->val == 1) // gain dynamic
		dynamic_param_adjust();

STOP_LABEL:
	if (true == err_occur(err_ctrl))
	{
		stop_weld();
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
}

/**
 * @description: Welding real-time control portal
 * @return {*}
 */
void welding_process(void)
{
	/*The welding is stopped when the countdown timer reaches the end*/
	if (weld_controller->weld_count == 0 && weld_controller->Count_Dir == DOWN)
		return;

	/*定时器配置PWM 5KHz*/
	TIM1_PWM_Init();
	TIM4_PWM_Init();
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
	TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);

	TIM_Cmd(TIM3, DISABLE);
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_Cmd(TIM5, DISABLE);
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

	/*Scan buttons to load different parameter sets based on the specific button pressed.*/
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

			/*计算三段显示温度值*/
			display_temp_cal();

			/*绘制降温曲线*/
			down_temp_line();

			/*存储数据到U盘*/
			OSSemPost(&DATA_SAVE_SEM, OS_OPT_POST_ALL, &err);

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

			/*计算三段显示温度值*/
			display_temp_cal();

			/*绘制降温曲线*/
			if (page_param->id == WAVE_PAGE)
			{
				down_temp_line();
			}

			/*存储数据到U盘*/
			OSSemPost(&DATA_SAVE_SEM, OS_OPT_POST_ALL, &err);

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
			if (page_param->id == WAVE_PAGE)
			{
				down_temp_line();
				reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);
			}

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
