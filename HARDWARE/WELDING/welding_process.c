#include "welding_process.h"
#include "includes.h"

#include "key.h"
#include "pwm.h"
#include "PID.h"
#include "adc.h"
#include "timer.h"
#include "protect.h"
#include "spi.h"
#include "usart.h"
#include "Kalman.h"
#include "tempcomp.h"
#include "touchscreen.h"

/*--------------------------------------------------旧版本接口---------------------------------------------------------*/
volatile WELD_MODE welding_flag = IDEAL_MODE; // 焊接的标志
extern u16 kalman_comp_temp;				  // 温度补偿值

/*-------------------------------------------------------新接口--------------------------------------------------------*/

/*----------------------------------------------新版本实时控制---------------------------------------------------------*/
extern weld_ctrl *weld_controller; // 焊接控制器
extern double fitting_curves[3];   // pid 参数动态拟合曲线x*x+x+1
/*-----------------------------------------------适配的触摸屏数据接口--------------------------------------------------*/
extern Component_Queue *temp_page_list;	 // 温度限制界面UI队列
extern Component_Queue *param_page_list; // 参数设定界面的组件列表
extern u8 ID_OF_MAS;
/*--------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------错误报警-----------------------------------------------------------*/
extern OS_SEM ERROR_HANDLE_SEM; // 错误信号
extern Error_ctrl *err_ctrl;	// 错误管理器
/*--------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------算法控制-----------------------------------------------------------*/
/*卡尔曼滤波*/
Kalman kfp;
dynamical_comp dynam_comp;		 // 动态补偿
extern last_temp_sotre lasttemp; // 过往温度记录
/*--------------------------------------------------------绘图---------------------------------------------------------*/
extern OS_SEM TEMP_DRAW_SEM;					// 绘图事件信号
extern Temp_draw_ctrl *temp_draw_ctrl;			// 绘图控制器
extern Page_Param *page_param;					// 实时页面参数
extern u16 realtime_temp_buf[TEMP_BUF_MAX_LEN]; // 温度保存缓冲区

/*热电偶*/
extern Thermocouple *current_Thermocouple;

/*--------------------------------------------------------------------------------------------------------------------*/
/*													 数据对象API                                                       */
/*--------------------------------------------------------------------------------------------------------------------*/

WELD_MODE get_weld_flag()
{
	return welding_flag;
}

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
		for (u8 i = 0; i < 5; i++)
		{
			ctrl->weld_time[i] = 0;
		}
		for (u8 i = 0; i < 3; i++)
		{
			ctrl->weld_temp[i] = 0;
		}
		for (u8 i = 0; i < 6; i++)
		{
			ctrl->alarm_temp[i] = 0;
		}
		ctrl->temp_gain1 = 0.85;
		ctrl->temp_gain2 = 0.85;
		ctrl->temp_gain2 = 0.85;

		return ctrl;
	}
	return NULL;
}

/**
 * @description: number convert to string
 * @param {char} *buffer
 * @param {u8} buf_len
 * @param {u16} value
 * @return {*}
 */
void user_value_convert_to_string(char *buffer, const u8 buf_len, const u16 value)
{

	/*data length calculate*/
	u8 length = 1;
	u16 cal_val = value;
	while (cal_val > 0)
	{
		cal_val /= 10;
		if (cal_val > 0)
			length++;
	}

	/*value convert to string*/
	u8 per_bit = 0;
	u8 cnt;
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
/*													 过程控制API                                                       */
/*--------------------------------------------------------------------------------------------------------------------*/

static void Load_Data()
{
	u8 GP_Temp = 0;
	u8 key = 0;
	Component *comp = get_comp(param_page_list, "GP");
	/*扫描按键，根据启动模式对对应参数组数据进行读取，并将其发送至触摸屏*/
	GP_Temp = comp->val;
	if ((GP_Temp % 2 == 0)) // 如果GP是双数
	{
		key = new_key_scan();
		if (key == KEY_PC1_PRES) // 踩下单数
		{
			GP_Temp += 1;
			Load_param(weld_controller, GP_Temp);
			Load_param_alarm(weld_controller, GP_Temp);
			command_set_comp_val("GP", "val", GP_Temp);
		}
	}
	else // 如果GP是单数
	{
		key = new_key_scan();
		if ((key == KEY_PC0_PRES)) // 踩下双数
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
 * @description: 借助增益调控pid参数以及预爬升点
 * @param {weld_ctrl} *ctrl
 * @return {*}
 */
static void ctrl_param_config(weld_ctrl *ctrl)
{

	/*动态调整曲线*/
	double set_gain;
	double percent;

	/*-------------------------------------------------------用户自定义模式-------------------------------------------------------*/
	if (get_comp(temp_page_list, "switch")->val == 0)
	{
		switch (ctrl->state)
		{
		case FIRST_STATE:
			/*一阶段参数*/
			if (ctrl->weld_temp[0] < 300) // 设定目标较小，快速升温
			{
				ctrl->first_step_turn = ctrl->weld_temp[0]; // 刹车点
				ctrl->first_step_set = ctrl->weld_temp[0];
			}
			else
			{
				ctrl->first_step_turn = ctrl->weld_temp[0];													   // 刹车点
				ctrl->first_step_set = (0.9 + 0.1 * weld_controller->temp_gain1) * (double)ctrl->weld_temp[0]; // 第1个阶跃目标(0.85-1)
				if (ctrl->first_step_set >= ctrl->first_step_turn)
					ctrl->first_step_set = ctrl->first_step_turn;
			}
			break;

		case SECOND_STATE:
			/*二阶段参数*/
			if (ctrl->second_step_start_temp < 100) // 起始温度较小，防止超调
			{
				ctrl->second_step_turn = ctrl->weld_temp[1];			   // 刹车点
				ctrl->second_step_set = 0.85 * (double)ctrl->weld_temp[1]; // 第2个阶跃目标(0.85-1)
			}
			else
			{
				ctrl->second_step_turn = ctrl->weld_temp[1];													  // 刹车点
				ctrl->second_step_set = (0.85 + 0.15 * weld_controller->temp_gain2) * (double)ctrl->weld_temp[1]; // 第2个阶跃目标(0.85-1)
				if (ctrl->second_step_set >= ctrl->second_step_turn)
					ctrl->second_step_set = ctrl->second_step_turn;
			}
			break;

		case THIRD_STATE:
			break;
		}
	}
	/*----------------------------------------------------------自动模式----------------------------------------------------------*/
	else
	{
		switch (ctrl->state)
		{
		case FIRST_STATE:
			/*一阶段参数*/
			if (ctrl->weld_temp[0] < 300) // 设定目标较小，快速升温
			{
				ctrl->first_step_turn = ctrl->weld_temp[0]; // 刹车点
				ctrl->first_step_set = ctrl->weld_temp[0];
				/* 参数动态调整后，数据保存 */
				weld_controller->temp_gain1 = 0.85;
				get_comp(temp_page_list, "GAIN1")->val = 0.85 * 100;
			}
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

				ctrl->first_step_turn = ctrl->weld_temp[0];									// 刹车点
				ctrl->first_step_set = (0.9 + 0.1 * set_gain) * (double)ctrl->weld_temp[0]; // 第1个阶跃目标(0.85-1)
				if (ctrl->first_step_set >= ctrl->first_step_turn)
					ctrl->first_step_set = ctrl->first_step_turn;
			}
			break;
		case SECOND_STATE:
			/*二阶段参数*/
			if (ctrl->second_step_start_temp < 100) // 起始温度较小，防止超调
			{
				ctrl->second_step_turn = ctrl->weld_temp[1];			  // 刹车点
				ctrl->second_step_set = 0.8 * (double)ctrl->weld_temp[1]; // 第2个阶跃目标(0.85-1)
				/* 参数动态调整后，数据保存 */
				weld_controller->temp_gain2 = 0.85;
				get_comp(temp_page_list, "GAIN2")->val = 0.85 * 100;
			}
			else
			{
				/*根据起始温度调节设定值，起始温度越低，设定点就越低，防止过大的超调*/
				percent = (double)ctrl->second_step_start_temp / (double)ctrl->weld_temp[1]; // 起始温度对最终温度占比
				set_gain = 0.3969 * log(percent) + 1.0021;									 // 参见excel表格
				if (set_gain <= 0)
					set_gain = 0;
				if (set_gain >= 1)
					set_gain = 1;

				/* 参数动态调整后，数据保存 */
				weld_controller->temp_gain2 = set_gain;
				get_comp(temp_page_list, "GAIN2")->val = set_gain * 100;

				ctrl->second_step_turn = ctrl->weld_temp[1];								   // 刹车点
				ctrl->second_step_set = (0.85 + 0.15 * set_gain) * (double)ctrl->weld_temp[1]; // 第2个阶跃目标(0.85-1)
				if (ctrl->second_step_set >= ctrl->second_step_turn)
					ctrl->second_step_set = ctrl->second_step_turn;
			}
			break;

		case THIRD_STATE:
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
	if (weld_controller->temp_gain2 >= 1)
		weld_controller->temp_gain2 = 1;

	/*算法部分*/
	reset_forword_ctrl(weld_controller->pid_ctrl); // pid控制器初始化
	dynamic_init(&dynam_comp, 100);				   // 动态补偿初始化
	window_init(&lasttemp);						   // 滑动窗口初始化
	Kalman_Init(&kfp);							   // 卡尔曼滤波器初始化，初始值归零
	kalman_comp_temp = 0;						   // 卡尔曼估计值初始化

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
static void stop_weld()
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
 * @description: Pre-stress before welding
 * @return {*}
 */
static void Preload()
{

	// 焊接周期第一段，预压(预热)
	weld_controller->state = PRE_STATE;
	TIM_Cmd(TIM5, ENABLE);
	while (weld_controller->step_time_tick < weld_controller->weld_time[0] + 1)
	{
		__nop();
		__nop();
	}
	TIM_Cmd(TIM5, DISABLE);
	weld_controller->state = IDEAL_STATE;
	weld_controller->step_time_tick = 0;
}

/**
 * @description: The first stage of welding, the temperature rises
 * @return {*}
 */
static void First_Step()
{

	weld_controller->state = FIRST_STATE;														// 进入一阶段标志
	ctrl_param_config(weld_controller);															// 参数动态配置
	pid_param_dynamic_reload(weld_controller, fitting_curves, weld_controller->first_step_set); // kp动态调整
	if ((page_param->key2 == ION) && (weld_controller->weld_time[1] != 0))						// ION_IOF==0开PWM
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

			current_Thermocouple->slope *ADC_Value_avg(ADC_Channel_7) + current_Thermocouple->intercept;
			if (weld_controller->realtime_temp >= weld_controller->first_step_turn && weld_controller->pid_ctrl->stable_flag == false)
				weld_controller->pid_ctrl->stable_flag = true;
			// 过温保护
			if (weld_controller->realtime_temp > weld_controller->alarm_temp[0])
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
				draw_point(weld_controller->realtime_temp * 7 / 25);
#endif
		}

		/*4、第一段结束后，将一些标志位清零*/
		TIM_Cmd(TIM5, DISABLE);				  // 关闭实时控制器
		weld_controller->step_time_tick = 0;  // 实时控制器阶段性时间刻度复位
		TIM5->CNT = 0;						  // 计数值复位
		TIM_SetCompare1(TIM1, 0);			  // 关闭pwm 输出
		TIM_SetCompare1(TIM4, 0);			  // 关闭pwm 输出
		weld_controller->state = IDEAL_STATE; // 焊接状态复位
		/*5、记录一阶段结束坐标*/
		temp_draw_ctrl->first_step_index_end = temp_draw_ctrl->current_index - 1;
	}
}

/**
 * @description: In the second stage of welding, the temperature is maintained
 * @return {*}
 */
static void Second_Step()
{
	weld_controller->state = SECOND_STATE;														 // 进入二阶段
	ctrl_param_config(weld_controller);															 // 参数动态配置
	pid_param_dynamic_reload(weld_controller, fitting_curves, weld_controller->second_step_set); // kp动态调整
	if ((page_param->key2 == ION) && (weld_controller->weld_time[2] != 0))
	{
		/*1、进入二阶段采样*/
		temp_draw_ctrl->second_step_index_start = temp_draw_ctrl->current_index;
		/*2、实时控制部分*/

		weld_controller->step_time_tick = 0; // 计数值复位
		TIM_Cmd(TIM5, ENABLE);				 // 开启实时控制器
		while (weld_controller->step_time_tick < weld_controller->weld_time[2])
		{
			current_Thermocouple->slope *ADC_Value_avg(ADC_Channel_7) + current_Thermocouple->intercept;
			/*到达刹车点，转阶段*/
			if (weld_controller->realtime_temp >= weld_controller->second_step_turn && weld_controller->pid_ctrl->stable_flag == false)
				weld_controller->pid_ctrl->stable_flag = true;

			/*后续计算稳定温度值索引（待改进）*/
			if (weld_controller->realtime_temp >= 0.95 * weld_controller->second_step_set && temp_draw_ctrl->second_step_stable_index == 0)
			{
				if (weld_controller->pid_ctrl->stable_threshold_cnt >= weld_controller->pid_ctrl->stable_threshold)
					temp_draw_ctrl->second_step_stable_index = temp_draw_ctrl->current_index;
				else
					weld_controller->pid_ctrl->stable_threshold_cnt++;
			}

			/*过温保护*/
			if (weld_controller->realtime_temp > weld_controller->alarm_temp[2])
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
				draw_point(weld_controller->realtime_temp * 7 / 25);
#endif
		}
		/*3、第二段结束后，对变量进行复位*/
		TIM_Cmd(TIM5, DISABLE);														// 关闭实时控制器
		weld_controller->step_time_tick = 0;										// 计数值复位
		TIM5->CNT = 0;																// 计数值复位
		TIM_SetCompare1(TIM1, 0);													// 关闭pwm 输出
		TIM_SetCompare1(TIM4, 0);													// 关闭pwm 输出
		weld_controller->third_step_start_duty_cycle = weld_controller->Duty_Cycle; // 三阶段从这个占空比往下下降
		weld_controller->state = IDEAL_STATE;										// 焊接状态复位

		/*4、二阶段结束，记录坐标*/
		temp_draw_ctrl->second_step_index_end = temp_draw_ctrl->current_index - 1;
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
			current_Thermocouple->slope *ADC_Value_avg(ADC_Channel_7) + current_Thermocouple->intercept;
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
				draw_point(weld_controller->realtime_temp * 7 / 25);
#endif
		}

		/*3、第三段结束*/

		TIM_Cmd(TIM5, DISABLE);				  // 关闭实时控制器
		weld_controller->step_time_tick = 0;  // 阶段计数值复位
		TIM5->CNT = 0;						  // 计数值复位
		TIM_SetCompare1(TIM1, 0);			  // 关闭pwm 输出
		TIM_SetCompare1(TIM4, 0);			  // 关闭pwm 输出
		weld_controller->state = IDEAL_STATE; // 焊接状态复位
		weld_controller->Duty_Cycle = 0;	  // 占空比复位
		/*4、三阶段结束，坐标记录*/
		temp_draw_ctrl->third_step_index_end = temp_draw_ctrl->current_index - 1;
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
	else if (get_comp(param_page_list, "UP_DOWN")->val == DOWN_CNT)
		weld_controller->weld_count--;

	RLY10 = 0; // 气阀1关闭
	RLY11 = 0; // 气阀2关闭
	RLY12 = 0; // 气阀3关闭
	CUNT = 1;  // 1为计数，0清除计数信号
	OVER = 1;  // 1为焊接结束信号
}

#if COMMUNICATE == 1

/**
 * @description: Transmit data to the host computer
 * @param {u16} Temp_Send
 * @return {*}
 */
static void data_transfer_to_computer(const u16 Temp_Send[])
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
	else if (get_comp(param_page_list, "UP_DOWN")->val == DOWN_CNT)
		weld_controller->weld_count--;
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
	err_cnt_clear(err_ctrl);																		 // 报错统计值复位
	weld_controller->first_step_start_temp = TEMP_GAIN1 * ADC_Value_avg(ADC_Channel_7) + TEMP_GAIN2; // 起始温度
	First_Step();																					 // 一阶段
	if (true == err_occur(err_ctrl))																 // 唤醒错误处理线程
		goto STOP_LABEL;

	/*二阶段*/
	err_cnt_clear(err_ctrl);																		  // 报错统计值复位
	weld_controller->second_step_start_temp = TEMP_GAIN1 * ADC_Value_avg(ADC_Channel_7) + TEMP_GAIN2; // 起始温度
	Second_Step();																					  // 二阶段
	if (true == err_occur(err_ctrl))																  // 唤醒错误处理线程
		goto STOP_LABEL;

	/*三阶段*/
	err_cnt_clear(err_ctrl);																		 // 报错统计值复位
	weld_controller->third_step_start_temp = TEMP_GAIN1 * ADC_Value_avg(ADC_Channel_7) + TEMP_GAIN2; // 起始温度
	Third_Step();																					 // 三阶段
	if (true == err_occur(err_ctrl))																 // 唤醒错误处理线程
		goto STOP_LABEL;

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

	/*定时器配置PWM 5KHz*/
	Timer_Pre_Init();
	/*扫描按键，根据不同按键加载不同参数组别*/
	Load_Data();

	/*向下计数模式到底，停止焊接*/
	if (get_comp(param_page_list, "count")->val == 0 && get_comp(param_page_list, "UP_DOWN")->val == DOWN_CNT)
		return;

	/*----------------------------------------------------------------------------------------------------------------------------------------------*/
	/*----------------------------------------------------------------焊接实时控制部分---------------------------------------------------------------*/
	/*----------------------------------------------------------------------------------------------------------------------------------------------*/
	/*-------------------------------------------------------连点模式-------------------------------------------------------*/
	if (page_param->key3 == CTW && page_param->key2 == ION)
	{
		OS_ERR err;
		u8 key = 0;
		/*获取当前实时温度*/
		current_Thermocouple->slope *ADC_Value_avg(ADC_Channel_7) + current_Thermocouple->intercept;
		key = new_key_scan();
		// 不松开脚踏就进入持续焊接模式
		while (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{
			/*焊前擦除上次温度显示*/
			if (page_param->id == WAVE_PAGE)
			{
				command_send("cle wave_line.id,0");
				command_send("cle wave_line.id,0");
				command_send("cle wave_line.id,0");
			}
			/*进入焊接的条件*/
			if (page_param->key1 != RDY || weld_controller->realtime_temp > weld_controller->weld_temp[2])
				return;

			/*实时焊接控制*/
			weld_real_time_ctrl();
			if (true == err_occur(err_ctrl))
				break;
			/*每一轮焊接都清空绘图缓存*/
			reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);
			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DRAW_SEM, OS_OPT_POST_ALL, &err);

#if COMMUNICATE == 1
			/*数据传输到上位机*/
			data_transfer_to_computer(Temp_Send);
#endif
			OVER = 0;
			/*设置连续焊接间隔——同时也腾出时间片*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);

			key = new_key_scan();
			if (!(key == KEY_PC1_PRES || key == KEY_PC0_PRES))
				break;
		}
	}
	/*模拟焊接*/
	else if (page_param->key3 == CTW && page_param->key2 == IOFF)
	{
		OS_ERR err;
		u8 key = 0;
		key = new_key_scan();
		while (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{

			simulate_weld();
			/*设置连续焊接间隔——同时也腾出时间片*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DRAW_SEM, OS_OPT_POST_ALL, &err);

			key = new_key_scan();
			if (!(key == KEY_PC1_PRES || key == KEY_PC0_PRES))
				break;
		}
	}

	/*-------------------------------------------------------单点模式-------------------------------------------------------*/
	if (page_param->key3 == SGW && page_param->key2 == ION)
	{
		OS_ERR err;
		u8 key = 0;
		/*获取当前实时温度*/
		current_Thermocouple->slope *ADC_Value_avg(ADC_Channel_7) + current_Thermocouple->intercept;
		key = new_key_scan();
		if (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{
			/*焊前擦除上次温度显示*/
			if (page_param->id == WAVE_PAGE)
			{
				command_send("cle wave_line.id,0");
				command_send("cle wave_line.id,0");
				command_send("cle wave_line.id,0");
			}

			/*进入焊接的条件*/
			if (page_param->key1 != RDY || weld_controller->realtime_temp > weld_controller->weld_temp[2])
				return;

			/*实时焊接控制*/
			weld_real_time_ctrl();
			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DRAW_SEM, OS_OPT_POST_ALL, &err);

#if COMMUNICATE == 1
			/*数据传输到上位机*/
			data_transfer_to_computer(Temp_Send);
#endif
			OVER = 0;
			/*设置焊接间隔*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
		}
	}
	/*模拟焊接*/
	else if (page_param->key3 == SGW && page_param->key2 == IOFF)
	{
		OS_ERR err;
		u8 key = 0;
		key = new_key_scan();
		if (key == KEY_PC1_PRES || key == KEY_PC0_PRES)
		{

			simulate_weld();
			/*设置连续焊接间隔——同时也腾出时间片*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
			/*三段温度显示&计数值更新*/
			OSSemPost(&TEMP_DRAW_SEM, OS_OPT_POST_ALL, &err);
			OVER = 0;
			/*设置焊接间隔*/
			OSTimeDly(weld_controller->weld_time[4], OS_OPT_TIME_DLY, &err);
		}
	}
}
