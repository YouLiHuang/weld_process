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
double fitting_curves[3] = {-0.00004, 0.0159, 12.756}; // 二次曲线拟合系数
/*温度补偿部分*/
extern Kalman kfp;
extern dynamical_comp dynam_comp;
last_temp_sotre lasttemp;
u16 kalman_comp_temp = 0; // 卡尔曼滤波+动态补偿后的温度值

/*错误处理*/
extern Error_ctrl *err_ctrl;	// 错误注册表
extern OS_SEM ERROR_HANDLE_SEM; // 错误信号

/*绘图专用数据*/
extern Temp_draw_ctrl *temp_draw_ctrl;
u16 realtime_temp_buf[TEMP_BUF_MAX_LEN] = {0}; // 温度保存缓冲区

/*热电偶*/
extern Thermocouple *current_Thermocouple;

/**
 * @description: create an new controller
 * @param {u16} *buf
 * @param {u16} len1
 * @param {u16} len2
 * @param {u16} len3
 * @return {*}
 */
Temp_draw_ctrl *new_temp_draw_ctrl(u16 *buf, u16 len1, u16 len2, u16 len3)
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
void reset_temp_draw_ctrl(Temp_draw_ctrl *ctrl, const u16 welding_time[])
{
	u16 total_time_length = welding_time[0] + welding_time[1] + welding_time[2] + welding_time[3] + welding_time[4];
	if (total_time_length > ctrl->buf_len_max)
		ctrl->sample_freq = (u8)(total_time_length / ctrl->buf_len_max) + 1;
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
	for (u16 i = 0; i < ctrl->buf_len_max; i++)
	{
		ctrl->temp_buf[i] = 0;
	}
}

/**
 * @description: Dynamically adjust pid parameters according to set values
 * @param {void} *controller
 * @param {double} *fitting_curves
 * @param {u16} setting
 * @return {*}
 */
void pid_param_dynamic_reload(void *controller, double *fitting_curves, u16 setting)
{
#if PID_DEBUG == 0

	/*pid调试模式关闭时才动态调整*/
	weld_ctrl *ctrl = (weld_ctrl *)controller;
	double new_kp = 0;
	if (setting < 650 && setting > 150)
	{

		new_kp = fitting_curves[0] * setting * setting +
				 fitting_curves[1] * setting +
				 fitting_curves[2];

		if (new_kp > 15)
			new_kp = 15;
		if (new_kp <= 6)
			new_kp = 6;

		ctrl->pid_ctrl->kp = new_kp;
		ctrl->pid_ctrl->ki = 0.01;
		ctrl->pid_ctrl->kd = 0.5 * new_kp;
		/*稳态标志复位*/
		weld_controller->pid_ctrl->stable_flag = false;
	}
#endif
}

void TIM2_Int_Init(void)
{
	u16 arr = 1000 - 1; // 1ms发生中断
	u16 psc = 168 - 1;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); /// 使能TIM3时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr;						// 自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;					// 定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure); // 初始化TIM3

	TIM_Cmd(TIM2, DISABLE); // 使能定时器3
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // 允许定时器3更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				 // 定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; // 抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		 // 子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
   定时器初始化 时钟主频168MHz
   tim3 为焊接周期控制定时器
   也记录焊接过程温度
*/

void TIM3_Int_Init(void)
{
	u16 arr = 2000 - 1; // 1ms发生中断
	u16 psc = 84 - 1;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); /// 使能TIM3时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr;						// 自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;					// 定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure); // 初始化TIM3

	TIM_Cmd(TIM3, DISABLE); // 使能定时器3
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // 允许定时器3更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;				 // 定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; // 抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		 // 子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
   定时器初始化，时钟主频168MHz
   tim3 为pid控制定时器
   也记录焊接过程温度
   定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
*/
void TIM5_Int_Init(void)
{
	u16 arr = 2000 - 1;
	u16 psc = 84 - 1;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); /// 使能TIM5时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr;						// 自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;					// 定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure); // 初始化TIM5

	TIM_Cmd(TIM5, DISABLE); // 使能定时器5
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); // 允许定时器5更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;				 // 定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; // 抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		 // 子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static volatile u16 current_temp_comp = 0; // 当前温度估计值
/*
  定时器5中断函数——焊接实时闭环控制
 */
void TIM5_IRQHandler(void)
{

#if SYSTEM_SUPPORT_OS
	OSIntEnter();
#endif

	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		/*Ⅰ、反馈*/
#if COMPENSATION == 1
		weld_controller->realtime_temp = current_Thermocouple->slope * ADC_Value_avg(ADC_Channel_7) + current_Thermocouple->intercept;
		;																			 // 原始温度数据实时温度
		u16 kalman_filter_temp = KalmanFilter(&kfp, weld_controller->realtime_temp); // 卡尔曼滤波
		slid_windows(&lasttemp, kalman_filter_temp);								 // 滑动窗口
		kalman_comp_temp = dynamic_temp_comp(lasttemp, dynam_comp);					 // 动态补偿
		current_temp_comp = kalman_comp_temp;										 // 获取当前温度估计值
#else
		weld_controller->realtime_temp = current_Thermocouple->slope * ADC_Value_avg(ADC_Channel_7) + current_Thermocouple->intercept;
		;
		kalman_comp_temp = weld_controller->realtime_temp;
		current_temp_comp = weld_controller->realtime_temp;
#endif

		/*Ⅱ、算法控制器*/
		switch (weld_controller->state)
		{
		case PRE_STATE:
			weld_controller->step_time_tick++;
			break;
			/*--------------------------------------------------------------------一阶段----------------------------------------------------------------------*/
		case FIRST_STATE:
			/*1、算法控制——模拟斜坡输入*/
			weld_controller->step_time_tick++;
			if (current_temp_comp >= weld_controller->first_step_turn && weld_controller->pid_ctrl->stable_flag == false)
			{
				/*到达刹车点，转阶段*/
				weld_controller->pid_ctrl->stable_flag = true;
			}

			if (weld_controller->pid_ctrl->stable_flag == false)
			{
				// 第一个设定点
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

			/*2、执行*/
			TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
			TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

			/*3：错误检测*/
			/*注意：阶段之间需要将错误计数值清空(此处为了报警的实时性，暂时不使用阈值模式)*/
			if (current_temp_comp < weld_controller->first_step_start_temp && weld_controller->step_time_tick > weld_controller->weld_time[1] * 0.5)
			{
#if PROTECT_ON == 1
				err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
#endif
			}
			if (current_temp_comp > weld_controller->alarm_temp[0])
			{
#if PROTECT_ON == 1
				err_get_type(err_ctrl, TEMP_UP)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
#endif
			}

			break;

			/*--------------------------------------------------------------------二阶段----------------------------------------------------------------------*/
		case SECOND_STATE:
			/*时间更新*/
			weld_controller->step_time_tick++;

			if (current_temp_comp >= weld_controller->second_step_turn && weld_controller->pid_ctrl->stable_flag == false)
			{
				/*到达刹车点，转阶段*/
				weld_controller->pid_ctrl->stable_flag = true;
			}

			if (weld_controller->pid_ctrl->stable_flag == false)
			{
				// 第一个设定点
				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->second_step_set + STABLE_ERR,
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}
			else
			{
				// 第二个设定点
				weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[1] + STABLE_ERR, // 动态补偿（后续线性补偿）
															 current_temp_comp,
															 weld_controller->Duty_Cycle,
															 weld_controller->pid_ctrl);
			}

			/*2、执行*/
			TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
			TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);

			/*3、错误检测*/
			if (current_temp_comp < weld_controller->second_step_start_temp && weld_controller->step_time_tick > weld_controller->weld_time[2] * 0.1)
			{
#if PROTECT_ON == 1
				err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
#endif
			}
			if (current_temp_comp > weld_controller->alarm_temp[2])
			{

#if PROTECT_ON == 1
				err_get_type(err_ctrl, TEMP_UP)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
#endif
			}

			break;

			/*--------------------------------------------------------------------三阶段----------------------------------------------------------------------*/
			/*冷却过程，温度控制要求不高，应该可以给定一个指定斜率的下降曲线，PDC的起始值参考上一阶段的终了值*/
		case THIRD_STATE:
			/*时间更新*/
			weld_controller->step_time_tick++;
			/*1、：算法控制——缓降，暂时不实现，直接关闭输出*/
			weld_controller->Duty_Cycle = PI_ctrl_output(weld_controller->weld_temp[2] + STABLE_ERR,
														 current_temp_comp,
														 weld_controller->Duty_Cycle,
														 weld_controller->pid_ctrl);
			/*2、执行*/
			TIM_SetCompare1(TIM1, weld_controller->Duty_Cycle);
			TIM_SetCompare1(TIM4, weld_controller->Duty_Cycle);
			/*3、：错误检测*/
			if (current_temp_comp > weld_controller->alarm_temp[4])
			{
#if PROTECT_ON == 1
				err_get_type(err_ctrl, TEMP_UP)->state = true;
				OS_ERR err;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
#endif
			}

			break;
		}

		/*Ⅲ、数据可视化采集*/
		/*降低采样率*/
		if (weld_controller->weld_time_tick % temp_draw_ctrl->sample_freq == 0)
		{
			if (temp_draw_ctrl->current_index < temp_draw_ctrl->buf_len_max)
				temp_draw_ctrl->temp_buf[temp_draw_ctrl->current_index++] = kalman_comp_temp;
		}
	}

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); // 清除中断标志位

#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

/*
   定时器3中断服务函数
   实现功能
   1、焊接时间统计——温度采集处理
   2、记录过程中温度点数据
   3、对焊头温度进行补偿
*/
void TIM3_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) // 溢出中断
	{
		weld_controller->weld_time_tick += 1;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志位
#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

/**
 * @description: tim2 irq handle
 * @return {*}
 */
static u16 tim2_user_cnt;
void TIM2_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) // 溢出中断
	{
		tim2_user_cnt++;
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志位
#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

void tim2_cnt_set(u16 val)
{
	tim2_user_cnt = val;
}

u16 tim2_cnt_get()
{
	return tim2_user_cnt;
}

/**
 * @description: this api is use to provide precise delay
 * @param {u16} time_ms
 * @return {*}
 */
void user_tim_delay(u16 time_ms)
{
	TIM_Cmd(TIM2, ENABLE);
	tim2_user_cnt = 0;
	while (tim2_user_cnt < time_ms)
	{
		__NOP();
	}
	TIM_Cmd(TIM2, DISABLE);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志位
	TIM2->CNT = 0;
	tim2_user_cnt = 0;
}
