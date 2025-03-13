/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-11 15:47:16
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-03-13 11:27:17
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "includes.h"
#include "HCSR04.h"
#include "crc16.h"
#include "welding_process.h"
#include "spi.h"
#include "timer.h"
#include "adc.h"
#include "protect.h"
#include "pwm.h"
#include "Kalman.h"
#include "touchscreen.h"
#include "PID.h"
#include "filter.h"
#include "ThermocoupleIO.h"

/*user config*/
#define ROOM_TEMP 20	 // 默认室温
#define SAMPLE_LEN 100	 // 采样深度
#define ADC_BIAS_MAX 650 // 最大偏置值

/*debug option*/
#define TEMP_ADJUST 1	// 温度校准
#define VOLTAGE_CHECK 1 // 过欠压报警
#define JK_TEMP_SHOW 0	// JK热电偶显示

// 任务优先级
#define START_TASK_PRIO 3
// 任务堆栈大小
#define START_STK_SIZE 128
// 任务控制块
OS_TCB StartTaskTCB;
// 任务堆栈
CPU_STK START_TASK_STK[START_STK_SIZE];
// 任务函数
void start_task(void *p_arg);

// 任务优先级
#define ERROR_TASK_PRIO 4
// 任务堆栈大小
#define ERROR_STK_SIZE 3072
// 任务控制块
OS_TCB ErrorTaskTCB;
// 任务堆栈
CPU_STK ERROR_TASK_STK[ERROR_STK_SIZE];
// 任务函数
void error_task(void *p_arg);

// 任务优先级
#define MAIN_TASK_PRIO 5
// 任务堆栈大小
#define MAIN_STK_SIZE 4096
// 任务控制块
OS_TCB Main_TaskTCB;
// 任务堆栈
CPU_STK MAIN_TASK_STK[MAIN_STK_SIZE];
void main_task(void *p_arg);

// 任务优先级
#define READ_TASK_PRIO 6
// 任务堆栈大小
#define READ_STK_SIZE 3072
// 任务控制块
OS_TCB READ_TaskTCB;
// 任务堆栈
CPU_STK READ_TASK_STK[READ_STK_SIZE];
// 任务函数
void read_task(void *p_arg);

// 任务优先级
#define COMPUTER_TASK_PRIO 7
// 任务堆栈大小
#define COMPUTER_STK_SIZE 512
// 任务控制块
OS_TCB COMPUTER_TaskTCB;
// 任务堆栈
CPU_STK COMPUTER_TASK_STK[COMPUTER_STK_SIZE];
// 任务函数
void computer_read_task(void *p_arg);

/*--------------------------------------------------------新触摸屏界面部分------------------------------------------------------------------*/
////////////////////////RS485的消息队列/////////////////////////////////////////
extern uint32_t baud_list[11];
extern uint8_t USART_RX_BUF[USART_REC_LEN];			 // 触摸屏串口接收缓冲
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN]; // 温度保存缓冲区
#define UART_Q_NUM 100								 // 按键消息队列的数量
OS_Q UART_Msg;										 // 串口数据队列
////////////////////////UART3资源保护：互斥锁（暂时未用）////////////////////////
OS_MUTEX UARTMutex;
////////////////////////线程同步：信号量////////////////////////////////////////
OS_SEM PAGE_UPDATE_SEM;	  // 页面刷新信号
OS_SEM COMP_VAL_GET_SEM;  // 组件属性值成功获取信号
OS_SEM COMP_STR_GET_SEM;  // 组件属性值(字符串型)成功获取信号
OS_SEM ALARM_RESET_SEM;	  // 报警复位信号
OS_SEM SENSOR_UPDATE_SEM; // 热电偶校准信号

OS_SEM COMPUTER_DATA_SYN_SEM; // 上位机数据同步信号
OS_SEM HOST_WELD_CTRL_SEM;	  // 上位机开启焊接信号
/*主线程使用*/
OS_SEM ERROR_HANDLE_SEM; // 错误信号
OS_SEM TEMP_DISPLAY_SEM; // 绘图信号

////////////////////////新界面组件列表///////////////////////////////////////////
Page_Param *page_param = NULL;			   // 记录当前所在界面id及三个按钮的状态
Component_Queue *param_page_list = NULL;   // 参数设定界面的组件列表
Component_Queue *temp_page_list = NULL;	   // 温度限制界面的组件列表
Component_Queue *setting_page_list = NULL; // 通讯界面组件列表
Error_ctrl *err_ctrl = NULL;			   // 错误注册表
Temp_draw_ctrl *temp_draw_ctrl = NULL;	   // 绘图控制器
weld_ctrl *weld_controller = NULL;		   // 焊接实时控制器
pid_feedforword_ctrl *pid_ctrl = NULL;
Thermocouple *current_Thermocouple = NULL;
/*用户可自行根据需要添加需要的组件列表*/
/*......*/
/////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////错误处理回调//////////////////////////////
static bool Thermocouple_recheck_callback(uint8_t index);
static bool Current_out_of_ctrl_callback(uint8_t index);
static bool Temp_up_err_callback(uint8_t index);
static bool Temp_down_err_callback(uint8_t index);

/////////////////////////////////////错误复位回调/////////////////////////////////
static bool Thermocouple_reset_callback(uint8_t index);
static bool Current_out_of_ctrl_reset_callback(uint8_t index);
static bool Temp_up_reset_callback(uint8_t index);
static bool Temp_down_reset_callback(uint8_t index);

/*采用硬件校准模式，不同的热电偶的板级放大系数不同，
需要单独进行校准，但是软件层是一致的*/
// E：0.222*3300/4096=0.1788
// J：0.2189*3300/4096=0.1763
// K：0.218*3300/4096=0.1756
static Thermocouple Thermocouple_Lists[] = {
	{E_TYPE, 0.17, 0, 0},
	{K_TYPE, 0.17, 0, 0}, // 主控板上的热电偶检测通道，实际上校准成了E
	{J_TYPE, 0.17, 0, 0},
};

const error_match_list match_list[] = {
	{CURRENT_OUT_OT_CTRL, "p4", Current_out_of_ctrl_callback, Current_out_of_ctrl_reset_callback},
	{TEMP_UP, "p5", Temp_up_err_callback, Temp_up_reset_callback},
	{TEMP_DOWN, "p6", Temp_down_err_callback, Temp_down_reset_callback},
	{VOLTAGE_TOO_HIGH, "p7", NULL, NULL},
	{VOLTAGE_TOO_LOW, "p8", NULL, NULL},
	{MCU_OVER_HEAT, "p9", NULL, NULL},
	{TRANSFORMER_OVER_HEAT, "p10", NULL, NULL},
	{SENSOR_ERROR, "p11", Thermocouple_recheck_callback, Thermocouple_reset_callback},
};

static char *key_name_list[] = {"RDY_SCH", "ION_OFF", "SGW_CTW", "UP_DOWN"};

static char *weld_time_name_list[] = {
	"time1",
	"time2",
	"time3",
	"time4",
	"time5",
};
static char *weld_temp_name_list[] = {
	"temp1",
	"temp2",
	"temp3",
};

static char *alarm_temp_name_list[] = {
	"alarm1",
	"alarm2",
	"alarm3",
	"alarm4",
	"alarm5",
	"alarm6"};
static char *gain_name_list[] = {
	"GAIN1",
	"GAIN2",
	"GAIN3"};

/*list init name*/
static char *temp_page_name_list[] = {
	"alarm1",
	"alarm2",
	"alarm3",
	"alarm4",
	"alarm5",
	"alarm6",
	"GAIN1",
	"GAIN2",
	"GAIN3",
	"RDY_SCH",
	"ION_OFF",
	"SGW_CTW",
	"UP_DOWN",
	"count"};

static char *param_page_name_list[] = {
	"temp1",
	"temp2",
	"temp3",
	"time1",
	"time2",
	"time3",
	"time4",
	"time5",
	"RDY_SCH",
	"ION_OFF",
	"SGW_CTW",
	"UP_DOWN",
	"count"};

static char *setting_page_name_list[] = {
	"adress",
	"baudrate",
	"sensortype"};

/*--------------------------------------------------------全局变量------------------------------------------------------------------*/
/*上位机485参数*/
uint8_t ID_OF_MAS = 0;		// 焊机485通讯机号，默认是零,最大可设置15
uint8_t last_id_of_mas = 0; // 机号存储
uint32_t BOUND_SET = 0;		// 从机波特率设定
/*上位机通信参数*/
extern int Host_action;
extern int Host_GP;
extern uint16_t ModBus_time[6];		  // 焊接时间6段
extern uint16_t ModBus_temp[3];		  // 焊接温度3段
extern uint16_t ModBus_alarm_temp[6]; // 限制温度6段

extern float Host_gain1;
extern float Host_gain2;
extern int Host_gain1_raw;
extern int Host_gain2_raw;
/*触摸屏通信参数*/
extern uint16_t remember_array;
extern int ION_IOF;
extern int SGW_CTW;
extern int RDY_SCH;

int main(void)
{

	/*------------------------------------------------------User layer data objects-----------------------------------------------------------*/
	/*pid controller*/
	pid_ctrl = new_pid_forword_ctrl(0, 12, 0.1, 6);
	/*Welding controller*/
	weld_controller = new_weld_ctrl(pid_ctrl);
	/*New interface component list initialization*/
	page_param = new_page_param();
	/*param page*/
	param_page_list = newList(PARAM_PAGE);
	page_list_init(param_page_list,
				   param_page_name_list,
				   sizeof(param_page_name_list) / sizeof(char *));

	/*Temperature Limit Page*/
	temp_page_list = newList(TEMP_PAGE);
	page_list_init(temp_page_list,
				   temp_page_name_list,
				   sizeof(temp_page_name_list) / sizeof(char *));
	component_insert(temp_page_list, newComponet("switch", 1)); // 添加开关组件

	/*setting page*/
	setting_page_list = newList(UART_PAGE);
	page_list_init(setting_page_list,
				   setting_page_name_list,
				   sizeof(setting_page_name_list) / sizeof(char *));

	/*...Users can create their own page queues as needed...*/

	/*Error Registry*/
	err_ctrl = new_error_ctrl();
	/*Wrong type registration*/
	for (uint8_t i = 0; i < sizeof(match_list) / sizeof(error_match_list); i++)
	{
		error_handle *handle = new_error_handle(match_list[i].type,
												match_list[i].pic,
												match_list[i].err_callback,
												match_list[i].reset_callback);
		register_error(err_ctrl, handle);
	}

	/*绘图控制器初始化*/
	temp_draw_ctrl = new_temp_draw_ctrl(realtime_temp_buf, 500, 2000, 500);

	/*默认e型热电偶*/
	// current_Thermocouple = &Thermocouple_Lists[1];

	/*------------------------------------------------------Hardware layer data initialization-----------------------------------------------------------*/
	/*Peripheral initialization*/
	delay_init(168);								// clock init
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // Interrupt Group Configuration
	uart_init(115200);								// Touch screen communication interface initialization
	usart3_init(115200);							// Host computer communication initialization
	KEYin_Init();									// key io init
	OUT_Init();										// output pin init
	Check_IO_init();								// Thermocouple detection io initialization
	SPI1_Init();
	ADC_DMA_INIT();

	/*TIM7-TIM3(time)-TIM5(pid)-TIM1(pwm1)-TIM4(pwm2)*/
	TIM2_Int_Init();
	TIM3_Int_Init();
	TIM5_Int_Init();

	/*硬件保护初始化*/
	Current_Check_IO_Config(); // Current detection io configuration
	// PROTECT_Init();			   // MCU 过热外部中断/变压器过热外部中断

	Touchscreen_init();
	Load_data_from_mem();
	Host_computer_reset();

	/*硬件测试代码段*/
	/*...*/

	/*------------------------------------------------------System level data objects-----------------------------------------------------------*/
	/*UCOSIII init*/
	OS_ERR err;
	CPU_SR_ALLOC();
	OSInit(&err);
	OS_CRITICAL_ENTER();

	OSTaskCreate((OS_TCB *)&StartTaskTCB,							// TCB
				 (CPU_CHAR *)"start task",							// task name
				 (OS_TASK_PTR)start_task,							// tack function
				 (void *)0,											// Parameters passed to the task function
				 (OS_PRIO)START_TASK_PRIO,							// Task Priority
				 (CPU_STK *)&START_TASK_STK[0],						// stack base adress
				 (CPU_STK_SIZE)START_STK_SIZE / 10,					// Task stack depth limit
				 (CPU_STK_SIZE)START_STK_SIZE,						// task stack size
				 (OS_MSG_QTY)0,										// The maximum number of messages that the task's internal message queue can receive(0:disable)
				 (OS_TICK)0,										// When the time slice rotation is enabled, the time slice length is 0, which is the default length.
				 (void *)0,											// User-supplemented storage area
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, // task option
				 (OS_ERR *)&err);									// Store the return value of this function when an error occurs

	OS_CRITICAL_EXIT();
	/*start UCOSIII*/
	OSStart(&err);
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------Initialize Thread------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err); // 统计任务
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN // 如果使能了测量中断关闭时间
	CPU_IntDisMeasMaxCurReset();
#endif

#if OS_CFG_SCHED_ROUND_ROBIN_EN // 当使用时间片轮转的时候,使能时间片轮转调度功能,时间片长度为1个系统时钟节拍,1ms
	OSSchedRoundRobinCfg(DEF_ENABLED, 1, &err);
#endif

	OS_CRITICAL_ENTER(); // 进入临界区
	// 创建消息队列KEY_Msg
	OSQCreate((OS_Q *)&UART_Msg,	  // 消息队列
			  (CPU_CHAR *)"UART Msg", // 消息队列名称
			  (OS_MSG_QTY)UART_Q_NUM, // 消息队列长度，这里设置为100
			  (OS_ERR *)&err);		  // 错误码

	// 创建互斥锁——串口资源访问（暂时未用）
	OSMutexCreate(&UARTMutex, "USART_Mutex", &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	/*--------------------------------------------UI相关信号量--------------------------------------------*/
	// 创建页面更新信号
	OSSemCreate(&PAGE_UPDATE_SEM, "page update", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 专门用于获取组件属性值的信号量
	OSSemCreate(&COMP_VAL_GET_SEM, "comp val get", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}
	OSSemCreate(&COMP_STR_GET_SEM, "comp str get", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 热电偶校准信号
	OSSemCreate(&SENSOR_UPDATE_SEM, "SENSOR_UPDATE_SEM", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	/*--------------------------------------------上位机信号量------------------------------------------*/
	// 创建上位机数据同步信号
	OSSemCreate(&COMPUTER_DATA_SYN_SEM, "data sync", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 创建上位机开启焊接信号量
	OSSemCreate(&HOST_WELD_CTRL_SEM, "HOST_WELD_CTRL_SEM", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	/*--------------------------------------------其他信号量--------------------------------------------*/
	//  创建报警复位信号量
	OSSemCreate(&ALARM_RESET_SEM, "alarm reset", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 创建绘图信号
	OSSemCreate(&TEMP_DISPLAY_SEM, "temp draw sem", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 创建报错信号
	OSSemCreate(&ERROR_HANDLE_SEM, "err sem", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 错误处理任务——最高优先级
	// 优先级：4
	// 50ms调度——50ms时间片
	OSTaskCreate((OS_TCB *)&ErrorTaskTCB,
				 (CPU_CHAR *)"error task",
				 (OS_TASK_PTR)error_task,
				 (void *)0,
				 (OS_PRIO)ERROR_TASK_PRIO,
				 (CPU_STK *)&ERROR_TASK_STK[0],
				 (CPU_STK_SIZE)ERROR_STK_SIZE / 10,
				 (CPU_STK_SIZE)ERROR_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)10,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// 创建主任务
	// 优先级：5
	// 30ms调度
	OSTaskCreate((OS_TCB *)&Main_TaskTCB,
				 (CPU_CHAR *)"Main task",
				 (OS_TASK_PTR)main_task,
				 (void *)0,
				 (OS_PRIO)MAIN_TASK_PRIO,
				 (CPU_STK *)&MAIN_TASK_STK[0],
				 (CPU_STK_SIZE)MAIN_STK_SIZE / 10,
				 (CPU_STK_SIZE)MAIN_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)100, // 最大连续运行时长（时间片）
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// 创建触摸屏通讯任务
	// 20ms调度
	// 优先级：6
	OSTaskCreate((OS_TCB *)&READ_TaskTCB,
				 (CPU_CHAR *)"Read task",
				 (OS_TASK_PTR)read_task,
				 (void *)0,
				 (OS_PRIO)READ_TASK_PRIO,
				 (CPU_STK *)&READ_TASK_STK[0],
				 (CPU_STK_SIZE)READ_STK_SIZE / 10,
				 (CPU_STK_SIZE)READ_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)20, // 设置最大连续运行时长（时间片）
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// 创建上位机通讯任务
	// 优先级：7
	OSTaskCreate((OS_TCB *)&COMPUTER_TaskTCB,
				 (CPU_CHAR *)"computer read task",
				 (OS_TASK_PTR)computer_read_task,
				 (void *)0,
				 (OS_PRIO)COMPUTER_TASK_PRIO,
				 (CPU_STK *)&COMPUTER_TASK_STK[0],
				 (CPU_STK_SIZE)COMPUTER_STK_SIZE / 10,
				 (CPU_STK_SIZE)COMPUTER_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)10,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	OS_CRITICAL_EXIT();			  // 退出临界区
	OSTaskDel((OS_TCB *)0, &err); // 删除start_task任务自身
}
#if 0
/**
 * @description: Temperature rise monitoring API
 * @return {*}
 */
static bool Temp_up_check(void)
{

	uint16_t Init_Temperature = 0;
	uint16_t New_Temperature = 0;

	Init_Temperature = temp_convert(current_Thermocouple);
	TIM1_PWM_Init();
	TIM4_PWM_Init();
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
	TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	/*开启定时器*/
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

	// 重启定时器，固定脉宽pwm输出50ms，检测温升。
	TIM_SetCompare1(TIM1, PD_MAX / 2);
	TIM_SetCompare1(TIM4, PD_MAX / 2);

	// 加热80ms
	user_tim_delay(100);

	// 关闭
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
	TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
	TIM_Cmd(TIM4, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	// 延时检测温度
	user_tim_delay(100);
	New_Temperature = temp_convert(current_Thermocouple);

	/*判断温度变化*/
	if (New_Temperature < Init_Temperature + 5 || New_Temperature > 4 * Init_Temperature || New_Temperature > 200)
		return false;
	/*热电偶恢复正常*/
	else
	{
		return true;
	}
	return true;
}
#endif

/*---------------------------------------------------------------------------------------*/
/*--------------------------------------错误回调API--------------------------------------*/
/*---------------------------------------------------------------------------------------*/

static bool Temp_up_err_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}

static bool Temp_down_err_callback(uint8_t index)
{

	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}

static bool Current_out_of_ctrl_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}
static bool Thermocouple_recheck_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}

/*---------------------------------------------------------------------------------------*/
/*--------------------------------------复位回调API--------------------------------------*/
/*---------------------------------------------------------------------------------------*/
static void Thermocouple_check(void);

static bool Thermocouple_reset_callback(uint8_t index)
{
	bool ret;
	/*热电偶恢复正常,复位*/
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // 清除错误状态
	ret = true;
	/*再次检测热点偶是否正常*/
	Thermocouple_check();
	return ret;
}
static bool Current_out_of_ctrl_reset_callback(uint8_t index)
{

	bool ret = false;
	if (EXTI_GetITStatus(EXTI_Line0) == RESET)
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // 清除错误状态
		ret = true;
		// NVIC_SystemReset();
	}
	else
	{
		/*仍旧异常*/
		Page_to(page_param, ALARM_PAGE);
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
		ret = false;
	}
	return ret;
}
static bool Temp_up_reset_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // 清除错误状态
	// NVIC_SystemReset();
	return true;
}
static bool Temp_down_reset_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // 清除错误状态
	// NVIC_SystemReset();
	return true;
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------错误处理线程--------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

void error_task(void *p_arg)
{
	OS_ERR err;
	/*重新配置串口*/
	uart_init(115200);
	while (1)
	{
		/*订阅报错信号*/
		OSSemPend(&ERROR_HANDLE_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{
			/*1、关闭输出*/
			TIM_SetCompare1(TIM1, 0);
			TIM_SetCompare1(TIM4, 0);
			TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
			TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Cmd(TIM1, DISABLE);

			TIM_Cmd(TIM3, DISABLE);						// 计数定时器3关闭
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断3标志位
			TIM3->CNT = 0;
			TIM_Cmd(TIM5, DISABLE);						// 中断定时器5关闭
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update); // 清除中断5标志位
			TIM5->CNT = 0;

			/*2、关闭气阀*/
			ERROR1 = 1; // 出错信号置1
			RLY10 = 0;	// 气阀1关
			RLY11 = 0;	// 气阀2关
			RLY12 = 0;	// 气阀3关

			/*3、错误处理*/
			user_tim_delay(20);
			Page_to(page_param, ALARM_PAGE);
			user_tim_delay(20);
			Page_to(page_param, ALARM_PAGE);
			page_param->id = ALARM_PAGE;
			for (uint8_t i = 0; i < err_ctrl->error_cnt; i++)
			{
				if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->error_callback != NULL)
					err_ctrl->err_list[i]->error_callback(i);
			}
		}

		/*等待用户复位操作*/
		OSSemPend(&ALARM_RESET_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			for (uint8_t i = 0; i < err_ctrl->error_cnt; i++)
			{
				if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->reset_callback != NULL)
					err_ctrl->err_list[i]->reset_callback(i);
			}
			/*重新配置*/
			uart_init(115200);
			/*串口屏复位*/
			Touchscreen_init();
			/*加载数据*/
			Load_data_from_mem();
			/*上位机复位*/
			Host_computer_reset();
		}

		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------主线程---------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/
static void voltage_check(void);
static bool current_out_of_ctrl(void);
static bool current_out_of_ctrl(void)
{
	OS_ERR err;
	if (CURRENT_CHECK == 0)
	{
		OSTimeDly(15, OS_OPT_TIME_DLY, &err); // 消抖
		if (CURRENT_CHECK == 0)
			return true;
		else
			return false;
	}
	return false;
}

static void Power_on_check(void)
{
	OS_ERR err;
	/*1、开机自检需要检测当前是哪一路热电偶*/
	GPIO_SetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
	GPIO_SetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
	GPIO_SetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);

	OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_PERIODIC, &err);

	if (GPIO_ReadInputDataBit(CHECK_GPIO_E, CHECKIN_PIN_E) != 0)
	{
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (Thermocouple_Lists[i].type == E_TYPE)
				current_Thermocouple = &Thermocouple_Lists[i];
		}
	}

	if (GPIO_ReadInputDataBit(CHECK_GPIO_K, CHECKIN_PIN_K) != 0)
	{
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (Thermocouple_Lists[i].type == K_TYPE)
				current_Thermocouple = &Thermocouple_Lists[i];
		}
	}

	if (GPIO_ReadInputDataBit(CHECK_GPIO_J, CHECKIN_PIN_J) != 0)
	{
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (Thermocouple_Lists[i].type == J_TYPE)
				current_Thermocouple = &Thermocouple_Lists[i];
		}
	}

	/*2、同时需要适配存储接口，读取出上次保存的热电偶校准值*/
	/* ... */
}

float voltage_test[6] = {0};

static void Temp_updata_realtime()
{
	// voltage_test[3] = (ADC_Value_avg(ADC_Channel_7) * 825) >> 10;
	// voltage_test[4] = (ADC_Value_avg(ADC_Channel_14) * 825) >> 10;
	// voltage_test[5] = (ADC_Value_avg(ADC_Channel_15) * 825) >> 10;
#if TEMP_ADJUST == 1
	weld_controller->realtime_temp = temp_convert(current_Thermocouple);
	command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
#endif
}

static void Thermocouple_check(void)
{

	OS_ERR err;
	// stop adc to avoid temp display change
	ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);

	static uint8_t IO_val;
	switch (current_Thermocouple->type)
	{

	case J_TYPE:
		IO_val = 0;
		GPIO_SetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
		OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_PERIODIC, &err);
		IO_val = GPIO_ReadInputDataBit(CHECK_GPIO_J, CHECKIN_PIN_J);
		if (IO_val == 0)
		{
			// 热电偶依旧异常，报警
			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
			/*唤醒错误处理线程*/
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}
		GPIO_ResetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
		break;

		// 现在的版本主板接线不方便，因此暂时不检测主板的热电偶
		// case K_TYPE:
		// 	IO_val = 0;
		// 	GPIO_SetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);
		// 	OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_PERIODIC, &err);
		// 	IO_val = GPIO_ReadInputDataBit(CHECK_GPIO_K, CHECKIN_PIN_K);
		// 	if (IO_val == 0)
		// 	{
		// 		// 热电偶依旧异常，报警
		// 		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		// 		/*唤醒错误处理线程*/
		// 		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		// 	}
		// 	GPIO_ResetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);
		// 	break;

	case E_TYPE:
		IO_val = 0;
		GPIO_SetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
		OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_PERIODIC, &err);
		uint8_t IO_val = GPIO_ReadInputDataBit(CHECK_GPIO_E, CHECKIN_PIN_E);
		// 断路报警
		if (IO_val == 0)
		{
			// 热电偶依旧异常，报警
			err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
			/*唤醒错误处理线程*/
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}
		GPIO_ResetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
		break;
	}

	// RESUME ADC
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
}

static void voltage_check(void)
{
	uint16_t ADC_Voltage = 0;
	OS_ERR err;
	ADC_Voltage = ADC_Value_avg(ADC_Channel_6);
	//  如果过压
	if (ADC_Voltage > OVER_VOLTAGE)
	{
#if VOLTAGE_CHECK
		err_get_type(err_ctrl, VOLTAGE_TOO_HIGH)->state = true;
		/*唤醒错误处理线程*/
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
#endif
	}
	// 如果欠压
	if (ADC_Voltage < DOWN_VOLTAGE)
	{
#if VOLTAGE_CHECK
		err_get_type(err_ctrl, VOLTAGE_TOO_LOW)->state = true;
		/*唤醒错误处理线程*/
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
#endif
	}
}

/*
	焊接主任务
	功能描述：
	1、开机自检
	2、热电偶监测
	3、电压监测
	4、执行焊接任务
	5、实时温度更新
*/
void main_task(void *p_arg)
{

	OS_ERR err;
	KEYin_Init();
	/*开机自检*/
	Power_on_check();
	while (1)
	{

		/*part1：电流失控检测*/
		if (current_out_of_ctrl() == true)
		{
			err_get_type(err_ctrl, CURRENT_OUT_OT_CTRL)->state = true;
			OS_ERR err;
			/*唤醒错误处理线程*/
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}

		/*part2：主焊接任务*/
		welding_process();
		/*part3:空闲时进行热点偶监测*/
		Thermocouple_check();
		/*part4:空闲时过欠压监测*/
		voltage_check();
		/*part5：空闲温度显示*/
		Temp_updata_realtime();

		OSTimeDlyHMSM(0, 0, 0, 30, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------触摸屏通信线程--------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

/**
 * @description: 消除热电偶偏置
 * @return {*}
 */
static void Thermocouple_err_eliminate()
{

	uint32_t sum = 0;
	uint16_t ADC_channel_init_val = 0;
	float adc_channel_data[SAMPLE_LEN] = {0};
	float adc_channel_fliter_buf[SAMPLE_LEN] = {0};
	/*根据当前热电偶类型采样对应的电压值*/
	switch (current_Thermocouple->type)
	{
	case E_TYPE:
		for (uint16_t i = 0; i < SAMPLE_LEN; i++)
		{
			adc_channel_data[i] = ADC_Value_avg(THERMOCOUPLE_CHANNEL_E);
			user_tim_delay(1);
		}

		break;

	case K_TYPE:
		for (uint16_t i = 0; i < SAMPLE_LEN; i++)
		{
			adc_channel_data[i] = ADC_Value_avg(THERMOCOUPLE_CHANNEL_K);
			user_tim_delay(1);
		}
		break;

	case J_TYPE:
		for (uint16_t i = 0; i < SAMPLE_LEN; i++)
		{
			adc_channel_data[i] = ADC_Value_avg(THERMOCOUPLE_CHANNEL_J);
			user_tim_delay(1);
		}
		break;
	}
	// 低通滤波
	low_pass_Filter((float *)adc_channel_data,
					SAMPLE_LEN,
					(float *)adc_channel_fliter_buf,
					1000,
					1000);

	/*均值滤波——计算初始偏置*/
	sum = 0;
	for (uint16_t i = 0; i < SAMPLE_LEN; i++)
	{
		sum += adc_channel_fliter_buf[i];
	}
	ADC_channel_init_val = sum / SAMPLE_LEN;

	if (ADC_channel_init_val > ADC_BIAS_MAX)
	{
		/*偏置过高，异常报警*/
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		OS_ERR err;
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
	else
	{
		// 常温对应的电压
		uint16_t room_temp_voltage = 0;

		// 更新对应的热电偶参数值
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (current_Thermocouple->type == Thermocouple_Lists[i].type)
			{
				// 常温下热点偶曲线计算出的理论电压值
				room_temp_voltage = (ROOM_TEMP - Thermocouple_Lists[i].intercept) / Thermocouple_Lists[i].slope;
				// 曲线修正
				Thermocouple_Lists[i].Bias = ADC_channel_init_val - room_temp_voltage;
				// 更新当前热电偶
				current_Thermocouple = &Thermocouple_Lists[i];
			}
		}

		/*校准完成，开启进度条动画*/
		command_set_comp_val("tm0", "en", 1);

		/*存储热电偶校准参数到eeprom*/
		uint8_t group = get_comp(param_page_list, "GP")->val;
		switch (current_Thermocouple->type)
		{
		case E_TYPE:
			SPI_Save_Word(current_Thermocouple->Bias, CARLIBRATION_BASE(group));
			break;
		case J_TYPE:
			SPI_Save_Word(current_Thermocouple->Bias, CARLIBRATION_BASE(group) + ADDR_OFFSET);
			break;
		}
	}

}

/*通信任务API*/
static void key_action_callback_param(Component_Queue *page_list);
static void key_action_callback_temp(Component_Queue *page_list);
static void parse_key_action(Page_ID id);
static void key_action_callback_param(Component_Queue *page_list)
{

	// 读取RDY_SCH按键值
	Component *comp = get_comp(page_list, "RDY_SCH");
	// 处于就绪态度————完成了参数修改 或者 静态无动作加载参数
	// 1、SCH——>RDY：退出修改模式，保存用户修改的参数
	// 2、RDY——>RDY：用户无动作，根据GP值加载参数
	if (RDY == comp->val)
	{
		// 1、SCH——>RDY：退出修改模式，保存用户修改的参数
		if (comp->val != page_param->key1)
		{
			/*Ⅰ、读取界面上的参数*/
			for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, weld_temp_name_list[i], "val");
			}
			for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, weld_time_name_list[i], "val");
			}

			/*Ⅱ、读取用户设定的参数*/
			uint16_t temp[3] = {0}, time[5] = {0};
			for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
			{
				/*获取更新的组件属性值*/
				if (get_comp(page_list, weld_time_name_list[i]) != NULL)
					time[i] = get_comp(page_list, weld_time_name_list[i])->val;
			}
			for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
			{
				/*获取更新的组件属性值*/
				if (get_comp(page_list, weld_temp_name_list[i]) != NULL)
					temp[i] = get_comp(page_list, weld_temp_name_list[i])->val;
			}

			/*Ⅲ、保存到eeprom*/
			if (get_comp(page_list, "GP"))
			{
				save_param(weld_controller,
						   get_comp(page_list, "GP")->val,
						   temp,
						   sizeof(temp) / sizeof(uint16_t),
						   time,
						   sizeof(time) / sizeof(uint16_t));
			}
		}
		// 2、RDY——>RDY：无动作，根据GP值加载参数
		else if (get_comp(page_list, "GP")->val <= GP_MAX)
		{
			/*Ⅰ、从内存加载参数*/
			uint8_t GP = get_comp(page_list, "GP")->val; // 当前设定的GP值
			if (GP != page_param->GP && GP <= GP_MAX)	 // GP值和上次不一致（用户修改）降低内存读写次数
				Load_param(weld_controller, GP);
			/*发送到触摸屏*/
			for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, weld_time_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(weld_time_name_list[i], "val", comp->val);
			}
			for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, weld_temp_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(weld_temp_name_list[i], "val", comp->val);
			}
		}
	}
	// 处于修改模式————进入修改模式
	// 3、RDY——>SCH：进入修改模式
	// 4、SCH——>SCH：正在修改参数
	else if (SCH == comp->val)
	{
		/*Ⅰ、从内存加载参数*/
		uint8_t GP = get_comp(page_list, "GP")->val; // 当前设定的GP值
		if (GP != page_param->GP && GP <= GP_MAX)	 // GP值和上次不一致（用户修改）
		{
			/*降低内存读写次数*/
			Load_param(weld_controller, GP);
			/*发送参数到触摸屏————三个温度，5个时间点*/

			/*发送到触摸屏*/
			for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, weld_time_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(weld_time_name_list[i], "val", comp->val);
			}
			for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, weld_temp_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(weld_temp_name_list[i], "val", comp->val);
			}
		}
		/*Ⅱ、读取界面上的参数*/
		for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, weld_temp_name_list[i], "val");
		}
		for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, weld_time_name_list[i], "val");
		}
	}
	// 状态同步
	page_param->GP = get_comp(page_list, "GP")->val;
	page_param->key1 = (RDY_SCH_STATE)get_comp(page_list, "RDY_SCH")->val;
	page_param->key2 = (ION_OFF_STATE)get_comp(page_list, "ION_OFF")->val;
	page_param->key3 = (SGW_CTW_STATE)get_comp(page_list, "SGW_CTW")->val;
}
static void key_action_callback_temp(Component_Queue *page_list)
{

	// 读取RDY_SCH按键值
	Component *comp = get_comp(page_list, "RDY_SCH");
	// 处于就绪态度————完成了参数修改 或者 静态无动作加载参数
	// 1、SCH——>RDY：退出修改模式，保存用户修改的参数
	// 2、RDY——>RDY：用户无动作，根据GP值加载参数
	if (RDY == comp->val)
	{
		// 1、SCH——>RDY：退出修改模式，保存用户修改的参数
		if (comp->val != page_param->key1)
		{
			/*Ⅰ、读取界面上的参数*/
			for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, gain_name_list[i], "val");
			}
			for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, alarm_temp_name_list[i], "val");
			}

			/*Ⅱ、保存用户设定的参数*/
			uint16_t temp_HL[6] = {0}, gain[2] = {0};
			for (uint8_t i = 0; i < sizeof(temp_HL) / sizeof(uint16_t); i++)
			{
				/*获取更新的组件属性值*/
				if (get_comp(page_list, alarm_temp_name_list[i]) != NULL)
					temp_HL[i] = get_comp(page_list, alarm_temp_name_list[i])->val;
			}
			for (uint8_t i = 0; i < sizeof(gain) / sizeof(uint16_t); i++)
			{
				/*获取更新的组件属性值*/
				if (get_comp(page_list, gain_name_list[i]) != NULL)
					gain[i] = get_comp(page_list, gain_name_list[i])->val;
			}

			/*Ⅱ、存储到eeprom*/
			if (get_comp(page_list, "GP") != NULL)
			{
				save_param_alarm(weld_controller,
								 get_comp(page_list, "GP")->val,
								 temp_HL,
								 sizeof(temp_HL) / sizeof(uint16_t),
								 gain);
			}
		}
		// 2、RDY——>RDY：用户无动作，根据GP值加载参数
		else if (comp->val == page_param->key1 && get_comp(page_list, "GP")->val <= GP_MAX)
		{
			/*Ⅰ、从内存加载参数————两个温度系数，6个温度*/
			uint8_t GP = get_comp(page_list, "GP")->val; // 当前设定的GP值
			if (GP != page_param->GP && GP <= GP_MAX)	 // GP值和上次不一致（用户修改）
				Load_param_alarm(weld_controller, GP);	 // 加载温度限制/温度增益参数
			/*发送到触摸屏*/
			for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, alarm_temp_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(alarm_temp_name_list[i], "val", comp->val);
			}
			for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, gain_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(gain_name_list[i], "val", comp->val);
			}
		}
	}
	// 处于修改模式————进入修改模式
	// 3、RDY——>SCH：进入修改模式
	// 4、SCH——>SCH：正在修改参数
	else if (SCH == comp->val)
	{
		/*Ⅰ、根据GP加载参数*/
		uint8_t GP = get_comp(page_list, "GP")->val; // 当前设定的GP值GP值和上次不一致 降低内存读写次数
		if (GP != page_param->GP && GP <= GP_MAX)
		{
			/*加载参数*/
			Load_param_alarm(weld_controller, GP);
			/*发送到触摸屏*/
			for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, alarm_temp_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(alarm_temp_name_list[i], "val", comp->val);
			}
			for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
			{
				Component *comp = get_comp(page_list, gain_name_list[i]);
				if (comp != NULL)
					command_set_comp_val(gain_name_list[i], "val", comp->val);
			}
		}
		/*Ⅱ、读取界面上的参数*/
		for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, gain_name_list[i], "val");
		}
		for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, alarm_temp_name_list[i], "val");
		}
	}

	// 状态同步
	page_param->GP = get_comp(page_list, "GP")->val;
	page_param->key1 = (RDY_SCH_STATE)get_comp(page_list, "RDY_SCH")->val;
	page_param->key2 = (ION_OFF_STATE)get_comp(page_list, "ION_OFF")->val;
	page_param->key3 = (SGW_CTW_STATE)get_comp(page_list, "SGW_CTW")->val;
}
static void parse_key_action(Page_ID id)
{
	switch (id)
	{
	case PARAM_PAGE:
		key_action_callback_param(param_page_list);
		break;
	case TEMP_PAGE:
		key_action_callback_temp(temp_page_list);
		break;
	case WAVE_PAGE:
		/*...*/
		break;
	case ALARM_PAGE:
		/*...*/
		break;
	case UART_PAGE:
		/*...*/
		break;
	default:
		break;
	}
}
#if PID_DEBUG == 1
static bool pid_param_get(uint16_t *pid_raw_param)
{
	const char *pid_param_name_list[4] = {
		"kpf",
		"kp",
		"ki",
		"kd"};

	uint8_t *msg = NULL;
	OS_ERR err;
	OS_MSG_SIZE msg_size = 0;
	uint8_t param_get_success_cnt = 0;

	for (uint8_t i = 0; i < 4; i++)
	{

		char buffer[50] = {0};
		sprintf(buffer, "get %s.%s", pid_param_name_list[i], "val"); // 指令预处理
		strcat(buffer, END_OF_CMD);									 // 加上结束符
		RS485_send(buffer, strlen(buffer));							 // 发送数据
		/*订阅队列，并进行消息处理*/
		msg = (uint8_t *)OSQPend(&UART_Msg,			   // 消息队列指针
								 20,				   // 等待时长
								 OS_OPT_PEND_BLOCKING, // 阻塞等待模式
								 &msg_size,			   // 获取消息大小
								 NULL,				   // 获取消息的时间戳
								 &err);				   // 返回错误代码
		if (err == OS_ERR_NONE && msg != NULL && msg_size >= MIN_CMD_LEN)
		{
			if (msg[0] == CMD_INT_VAR_RETURN && msg[msg_size - 1] == END_FLAG && msg[msg_size - 2] == END_FLAG && msg[msg_size - 3] == END_FLAG)
			{
				param_get_success_cnt++;
				pid_raw_param[i] = msg[1] | msg[2] << 8 | msg[3] << 16 | msg[4] << 24;
			}
		}
		else
		{
			break;
		}
	}

	if (param_get_success_cnt == 4)
		return true;
	else
		return false;
}
#endif
static void page_process(Page_ID id)
{
	const char *tick_name[] = {"tick1", "tick2", "tick3", "tick4", "tick5"};
	switch (page_param->id)
	{
	case PARAM_PAGE:
	{
		OS_ERR err;
		/*1、读取界面上的参数*/
		for (uint8_t i = 0; i < sizeof(key_name_list) / sizeof(char *); i++)
		{
			command_get_comp_val(param_page_list, key_name_list[i], "pic");
		}
		command_get_comp_val(param_page_list, "GP", "val");	   // 读取gp值
		command_get_comp_val(param_page_list, "count", "val"); // 读取计数值（用户可能修改）
		/*2、解析按键动作*/
		parse_key_action(page_param->id);
		/*3、显示实时温度*/
		command_set_comp_val("temp33", "val", weld_controller->realtime_temp);

#if JK_TEMP_SHOW == 1
		command_set_comp_val("temp11", "val", temp_convert(&Thermocouple_Lists[1])); // k
		command_set_comp_val("temp22", "val", temp_convert(&Thermocouple_Lists[2])); // j
#endif

		/*4、焊接结束后显示三段温度&计数值*/
		OSSemPend(&TEMP_DISPLAY_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			uint16_t temp_display[3] = {0};
			char *temp_display_name[] = {"temp11", "temp22", "temp33"};
			/*三段均温显示*/
			temp_display[0] = weld_controller->second_step_start_temp;
			uint32_t sum = 0;
			for (uint16_t i = temp_draw_ctrl->second_step_stable_index; i < temp_draw_ctrl->second_step_index_end; i++)
				sum += temp_draw_ctrl->temp_buf[i];
			temp_display[1] = sum / (temp_draw_ctrl->second_step_index_end - temp_draw_ctrl->second_step_stable_index + 1);
			/*一二段均值发送到触摸屏*/
			command_set_comp_val(temp_display_name[0], "val", temp_display[0]);
			command_set_comp_val(temp_display_name[1], "val", temp_display[1]);

			/*绘图控制器复位*/
			reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);
			/*绘图结束清空缓存*/
			memset(temp_draw_ctrl->temp_buf, 0, sizeof(temp_draw_ctrl->temp_buf) / sizeof(uint16_t));

			/*更新焊接计数值*/
			command_set_comp_val("count", "val", weld_controller->weld_count);
		}
	}
	break;

	case TEMP_PAGE:
	{
		/*1、读取界面上的参数*/
		for (uint8_t i = 0; i < sizeof(key_name_list) / sizeof(char *); i++)
		{
			command_get_comp_val(temp_page_list, key_name_list[i], "pic");
		}
		command_get_comp_val(temp_page_list, "GP", "val");
		command_get_comp_val(temp_page_list, "count", "val");
		/*2、读取auto/user模式*/
		command_get_comp_val(temp_page_list, "switch", "val");
		/*3、解析按键动作*/
		parse_key_action(page_param->id);
	}
	break;

	case WAVE_PAGE:
	{
#if PID_DEBUG == 1
		command_set_comp_val("wave_page.kpf", "aph", 127);
		command_set_comp_val("wave_page.kp", "aph", 127);
		command_set_comp_val("wave_page.ki", "aph", 127);
		command_set_comp_val("wave_page.kd", "aph", 127);
		uint16_t pid_param[4] = {0};
		if (pid_param_get(pid_param) == true)
		{
			weld_controller->pid_ctrl->kp_f = pid_param[0] / 100.0;
			weld_controller->pid_ctrl->kp = pid_param[1] / 100.0;
			weld_controller->pid_ctrl->ki = pid_param[2] / 100.0;
			weld_controller->pid_ctrl->kd = pid_param[3] / 100.0;
		}

#else
		OS_ERR err;
		uint16_t total_time = 0;	 // 总焊接时长
		uint16_t delta_tick = 0;	 // 坐标间隔
		uint16_t total_tick_len = 0; // 横坐标总长度
		uint16_t win_width = 0;		 // 绘图区域占据的实际窗口大小

		/*实时温度显示*/
		command_set_comp_val("step3", "val", weld_controller->realtime_temp);

		/*更新坐标*/
		for (uint8_t i = 0; i < 5; i++)
			total_time += weld_controller->weld_time[i];
		/*坐标划分*/
		if (total_time <= 500)
			delta_tick = 100;
		else if (total_time > 500 && total_time <= 1000)
			delta_tick = 200;
		else if (total_time > 1000 && total_time <= 2500)
			delta_tick = 500;
		else if (total_time > 2500 && total_time <= 5000)
			delta_tick = 1000;
		else if (total_time > 5000 && total_time <= 10000)
			delta_tick = 2000;
		else if (total_time > 10000 && total_time <= 15000)
			delta_tick = 3000;
		else if (total_time > 15000 && total_time <= 20000)
			delta_tick = 4000;
		else
			delta_tick = 5000;

		/*绘图间隔*/
		total_tick_len = 5 * delta_tick;						 // 横坐标总长度
		win_width = WIN_WIDTH * total_time / total_tick_len;	 // 焊接周期绘图区域
		temp_draw_ctrl->delta_tick = total_time / win_width + 1; // 绘点时间间隔
		/*坐标发送到触摸屏*/
		for (uint8_t i = 0; i < sizeof(tick_name) / sizeof(char *); i++)
			command_set_comp_val(tick_name[i], "val", (1 + i) * delta_tick);

		/*焊接接收后显示三段温度*/
		OSSemPend(&TEMP_DISPLAY_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			uint16_t temp_display[3] = {0};
			char *temp_display_name[] = {"step1", "step2", "step3"};
			/*三段均温显示*/
			temp_display[0] = weld_controller->second_step_start_temp;
			uint32_t sum = 0;
			for (uint16_t i = temp_draw_ctrl->second_step_stable_index; i < temp_draw_ctrl->second_step_index_end; i++)
				sum += temp_draw_ctrl->temp_buf[i];
			temp_display[1] = sum / (temp_draw_ctrl->second_step_index_end - temp_draw_ctrl->second_step_stable_index + 1);
			/*一二段均值发送到触摸屏*/
			command_set_comp_val(temp_display_name[0], "val", temp_display[0]);
			command_set_comp_val(temp_display_name[1], "val", temp_display[1]);

			/*绘图控制器复位*/
			reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);
			/*绘图结束清空缓存*/
			memset(temp_draw_ctrl->temp_buf, 0, sizeof(temp_draw_ctrl->temp_buf) / sizeof(uint16_t));
		}
#endif
	}
	break;

	case ALARM_PAGE:
	{
		/*响应错误*/
		if (true == err_occur(err_ctrl))
		{
			OS_ERR err;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}
	}
	break;

	case UART_PAGE:
	{
		OS_ERR err;

		/*1、读取界面设定的地址*/
		command_get_comp_val(setting_page_list, "adress", "val");
		/*2、读取页面设定的波特率*/
		command_get_comp_val(setting_page_list, "baudrate", "val");
		/*3、读取热电偶类型*/
		command_get_comp_val(setting_page_list, "sensortype", "val");
		/*4、订阅热电偶校准信号*/
		OSSemPend(&SENSOR_UPDATE_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{

			if (weld_controller->realtime_temp < 2 * ROOM_TEMP && weld_controller->realtime_temp > 0.5 * ROOM_TEMP) // 在室温下才允许校准（10-40°C范围内才允许校准）
				Thermocouple_err_eliminate();
			else // 焊头尚未冷却，警报
				command_set_comp_val("warning", "aph", 127);
		}
		/*5、显示实时温度*/
		command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
	}
	break;

	default:
		/*...用户可自行添加需要的页面...*/
		break;
	}
}
static bool data_syn(Page_ID id)
{
	/*1、按键状态同步*/
	RDY_SCH = (page_param->key1 == SCH) ? 1 : 0;  // RDY_SCH==0时为RDY
	ION_IOF = (page_param->key2 == IOFF) ? 1 : 0; // ION_IOF==0为ION
	SGW_CTW = (page_param->key3 == CTW) ? 1 : 0;  // SGW_CTW==0为SGW

	/*2、参数同步*/
	switch (id)
	{
	case PARAM_PAGE:
	{

		Component *comp = NULL;
		for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
		{
			comp = get_comp(param_page_list, weld_time_name_list[i]);
			if (comp != NULL)
				weld_controller->weld_time[i] = comp->val;
		}
		for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
		{
			comp = get_comp(param_page_list, weld_temp_name_list[i]);
			if (comp != NULL)
				weld_controller->weld_temp[i] = comp->val;
		}
		/*同步计数值（用户可能修改）*/
		comp = get_comp(param_page_list, "count");
		if (comp != NULL)
			weld_controller->weld_count = comp->val;

		break;
	}

	case TEMP_PAGE:
	{
		Component *comp = NULL;
		for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
		{
			comp = get_comp(temp_page_list, alarm_temp_name_list[i]);
			if (comp != NULL)
				weld_controller->alarm_temp[i] = comp->val;
		}

		if (get_comp(temp_page_list, "GAIN1") != NULL)
		{
			float gain = get_comp(temp_page_list, "GAIN1")->val / 100.0;
			weld_controller->temp_gain1 = gain;
		}
		if (get_comp(temp_page_list, "GAIN2") != NULL)
		{
			float gain = get_comp(temp_page_list, "GAIN2")->val / 100.0;
			weld_controller->temp_gain2 = gain;
		}
		/*同步计数值（用户可能修改）*/
		comp = get_comp(param_page_list, "count");
		if (comp != NULL)
			weld_controller->weld_count = comp->val;

		break;
	}

	case UART_PAGE:
	{
		Component *comp = NULL;
		/*更新机号*/
		comp = get_comp(setting_page_list, "adress");
		if (comp != NULL)
			ID_OF_MAS = comp->val;

		/*更新波特率*/
		comp = get_comp(setting_page_list, "baudrate");
		if (comp != NULL)
		{
			uint8_t index = get_comp(setting_page_list, "baudrate")->val;
			usart3_set_bound(baud_list[index - 1]);
		}

		/*更新热电偶类型*/
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple_Lists[0]); i++)
		{
			if (get_comp(setting_page_list, "sensortype")->val == Thermocouple_Lists[i].type)
				current_Thermocouple = &Thermocouple_Lists[i]; // 更新当前热电偶类型
		}
	}

	default:
		break;
	}

	return true;
}

/*
	触摸屏的通讯任务
	功能描述：
	触摸屏读写任务，完成页面查询/更新，页面组件属性值更新，响应不同页面的动作
*/
void read_task(void *p_arg)
{

	OS_ERR err;
	while (1)
	{
		/*处理页面逻辑*/
		if (Page_id_get() == true)
		{
			page_process(page_param->id);
			data_syn(page_param->id);
		}

		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------上位机通信线程--------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

static void data_sync_from_computer()
{
	/*数据同步：上位机——>新数据接口*/
	char *time_name[] = {
		"time1",
		"time2",
		"time3",
		"time4",
		"time5",
	};
	for (uint8_t i = 0; i < sizeof(time_name) / sizeof(time_name[0]); i++)
	{
		get_comp(param_page_list, time_name[i])->val = weld_controller->weld_time[i];
	}
	char *temp_name[] = {
		"temp1",
		"temp2",
		"temp3",
	};
	for (uint8_t i = 0; i < sizeof(temp_name) / sizeof(temp_name[0]); i++)
	{
		get_comp(param_page_list, temp_name[i])->val = weld_controller->weld_temp[i];
	}

	char *alarm_name[] = {
		"alarm1",
		"alarm2",
		"alarm3",
		"alarm4",
		"alarm5",
		"alarm6",
	};
	for (uint8_t i = 0; i < sizeof(alarm_name) / sizeof(alarm_name[0]); i++)
	{
		get_comp(param_page_list, alarm_name[i])->val = weld_controller->alarm_temp[i];
	}
	get_comp(temp_page_list, "GAIN1")->val = weld_controller->temp_gain1;
	get_comp(temp_page_list, "GAIN2")->val = weld_controller->temp_gain2;
}

/**
 * @description: 上位机通信任务
 * @param {void} *p_arg
 * @return {*}
 */
void computer_read_task(void *p_arg)
{
	/*
	上位机维护的数据：
	ModBus_time ModBus_temp ModBus_alarm_temp Host_GP Host_gain1_raw Host_gain1 Host_gain2_raw Host_gain2
	用户维护的数据：
	weld_controller->weld_time 	weld_controller->weld_temp weld_controller->alarm_temp
	上位机修改数据后
	（1）更新用户维护的数据
	（2）完成数据同步
	*/
	OS_ERR err;
	usart3_init(115200);
	while (1)
	{

		/*订阅上位机的数据更新信号*/
		OSSemPend(&COMPUTER_DATA_SYN_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{

			/*1、Host_action不为0时为处理上位机数据，并将数据刷新到触摸屏*/
			if (Host_action == 1)
			{
				/*1、数据更新*/
				remember_array = Host_GP; // ：上位机维护的数据 remember_array：全局维护的GP值
				/*2、屏幕刷新*/
				for (int i = 0; i < 2; i++)
				{
					command_set_comp_val("GP", "val", remember_array); // 将上一次的GP值发给从机
					Load_param(weld_controller, remember_array);
					Load_param_alarm(weld_controller, remember_array);
				}
				SPI_Save_Word(0, 40 * (remember_array + 1) + 4);
				Host_action = 0;
			}
			else if ((Host_action >= 2) && (Host_action <= 17)) // Host_action 为2 - 17 ，表示上位机修改单个数据，同步修改控制板参数数组和触摸屏数据
			{
				/*1、数据更新*/
				switch (Host_action)
				{

				/*6个焊接时间（实际上只用五个）ModBus_time[2]弃用*/
				case 2:
					weld_controller->weld_time[0] = ModBus_time[0];
					break;
				case 3:
					weld_controller->weld_time[1] = ModBus_time[1];
					break;
				case 4:
					weld_controller->weld_time[2] = ModBus_time[3];
					break;
				case 5:
					weld_controller->weld_time[3] = ModBus_time[4];
					break;
				case 6:
					weld_controller->weld_time[4] = ModBus_time[5];
					break;
				/*三段焊接温度*/
				case 7:
					weld_controller->weld_temp[0] = ModBus_temp[0];
					break;
				case 8:
					weld_controller->weld_temp[1] = ModBus_temp[1];
					break;
				case 9:
					weld_controller->weld_temp[2] = ModBus_temp[2];
					break;
				/*6段警报温度*/
				case 10:
					weld_controller->alarm_temp[0] = ModBus_alarm_temp[0];
					break;
				case 11:
					weld_controller->alarm_temp[1] = ModBus_alarm_temp[1];
					break;
				case 12:
					weld_controller->alarm_temp[2] = ModBus_alarm_temp[2];
					break;
				case 13:
					weld_controller->alarm_temp[3] = ModBus_alarm_temp[3];
					break;
				case 14:
					weld_controller->alarm_temp[4] = ModBus_alarm_temp[4];
					break;
				case 15:
					weld_controller->alarm_temp[5] = ModBus_alarm_temp[5];
					break;
				/*两个增益系数*/
				case 16:
					weld_controller->temp_gain1 = Host_gain1;
					break;
				case 17:
					weld_controller->temp_gain2 = Host_gain2;
					break;
				}

				/*保存焊接时间*/
				SPI_Save_Word(weld_controller->weld_time[0], 40 * (remember_array + 1));
				SPI_Save_Word(weld_controller->weld_time[1], 40 * (remember_array + 1) + 2);
				SPI_Save_Word(weld_controller->weld_time[2], 40 * (remember_array + 1) + 4);
				SPI_Save_Word(weld_controller->weld_time[3], 40 * (remember_array + 1) + 6);
				SPI_Save_Word(weld_controller->weld_time[4], 40 * (remember_array + 1) + 8);
				SPI_Save_Word(0, 40 * (remember_array + 1) + 10);
				/*保存温度*/
				SPI_Save_Word(weld_controller->weld_temp[0], 40 * (remember_array + 1) + 12);
				SPI_Save_Word(weld_controller->weld_temp[1], 40 * (remember_array + 1) + 14);
				SPI_Save_Word(weld_controller->weld_temp[2], 40 * (remember_array + 1) + 16);
				/*保存限制温度*/
				SPI_Save_Word(weld_controller->alarm_temp[0], 40 * (remember_array + 1) + 18);
				SPI_Save_Word(weld_controller->alarm_temp[1], 40 * (remember_array + 1) + 20);
				SPI_Save_Word(weld_controller->alarm_temp[2], 40 * (remember_array + 1) + 22);
				SPI_Save_Word(weld_controller->alarm_temp[3], 40 * (remember_array + 1) + 24);
				SPI_Save_Word(weld_controller->alarm_temp[4], 40 * (remember_array + 1) + 26);
				SPI_Save_Word(weld_controller->alarm_temp[5], 40 * (remember_array + 1) + 28);
				/*保存增益*/
				SPI_Save_Word(weld_controller->temp_gain1 * 100, 40 * (remember_array + 1) + 30);
				SPI_Save_Word(weld_controller->temp_gain2 * 100, 40 * (remember_array + 1) + 32);

				/*2、屏幕刷新*/
				/*时间刷新*/
				char *time_name_list[] = {
					"param_page.time1",
					"param_page.time2",
					"param_page.time3",
					"param_page.time4",
					"param_page.time5",
				};
				for (uint8_t i = 0; i < sizeof(time_name_list) / sizeof(time_name_list[0]); i++)
				{
					command_set_comp_val(time_name_list[i], "val", weld_controller->weld_time[i]);
				}
				/*温度刷新*/
				char *temp_name_list[] = {"param_page.temp1", "param_page.temp2", "param_page.temp3"};
				for (uint8_t i = 0; i < sizeof(temp_name_list) / sizeof(temp_name_list[0]); i++)
				{
					command_set_comp_val(temp_name_list[i], "val", weld_controller->weld_temp[i]);
				}

				char *alarm_name_list[] = {
					"temp_page.alarm1",
					"temp_page.alarm2",
					"temp_page.alarm3",
					"temp_page.alarm4",
					"temp_page.alarm5",
					"temp_page.alarm6",
				};
				for (uint8_t i = 0; i < sizeof(alarm_name_list) / sizeof(alarm_name_list[0]); i++)
				{
					command_set_comp_val(alarm_name_list[i], "val", weld_controller->alarm_temp[i]);
				}

				/*增益刷新*/
				command_set_comp_val("temp_page.GAIN1", "val", weld_controller->temp_gain1 * 100);
				command_set_comp_val("temp_page.GAIN2", "val", weld_controller->temp_gain2 * 100);

				Host_action = 0;
			}
			else if (Host_action == 20) // Host_action= 20 表示改变整一页数据，将修改数据同步到参数数组和触摸屏，
			{
				/*1、数据更新*/
				remember_array = Host_GP;

				weld_controller->weld_time[0] = ModBus_time[0];
				weld_controller->weld_time[1] = ModBus_time[1];
				weld_controller->weld_time[2] = ModBus_time[3];
				weld_controller->weld_time[3] = ModBus_time[4];
				weld_controller->weld_time[4] = ModBus_time[5];

				weld_controller->weld_temp[0] = ModBus_temp[0];
				weld_controller->weld_temp[1] = ModBus_temp[1];
				weld_controller->weld_temp[2] = ModBus_temp[2];

				weld_controller->alarm_temp[0] = ModBus_alarm_temp[0];
				weld_controller->alarm_temp[1] = ModBus_alarm_temp[1];
				weld_controller->alarm_temp[2] = ModBus_alarm_temp[2];
				weld_controller->alarm_temp[3] = ModBus_alarm_temp[3];
				weld_controller->alarm_temp[4] = ModBus_alarm_temp[4];
				weld_controller->alarm_temp[5] = ModBus_alarm_temp[5];

				/*3、将用户数据更新至外部flash*/
				/*保存焊接时间*/
				SPI_Save_Word(weld_controller->weld_time[0], 40 * (remember_array + 1));
				SPI_Save_Word(weld_controller->weld_time[1], 40 * (remember_array + 1) + 2);
				SPI_Save_Word(weld_controller->weld_time[2], 40 * (remember_array + 1) + 4);
				SPI_Save_Word(weld_controller->weld_time[3], 40 * (remember_array + 1) + 6);
				SPI_Save_Word(weld_controller->weld_time[4], 40 * (remember_array + 1) + 8);
				SPI_Save_Word(0, 40 * (remember_array + 1) + 10);
				/*保存温度*/
				SPI_Save_Word(weld_controller->weld_temp[0], 40 * (remember_array + 1) + 12);
				SPI_Save_Word(weld_controller->weld_temp[1], 40 * (remember_array + 1) + 14);
				SPI_Save_Word(weld_controller->weld_temp[2], 40 * (remember_array + 1) + 16);
				/*保存限制温度*/
				SPI_Save_Word(weld_controller->alarm_temp[0], 40 * (remember_array + 1) + 18);
				SPI_Save_Word(weld_controller->alarm_temp[1], 40 * (remember_array + 1) + 20);
				SPI_Save_Word(weld_controller->alarm_temp[2], 40 * (remember_array + 1) + 22);
				SPI_Save_Word(weld_controller->alarm_temp[3], 40 * (remember_array + 1) + 24);
				SPI_Save_Word(weld_controller->alarm_temp[4], 40 * (remember_array + 1) + 26);
				SPI_Save_Word(weld_controller->alarm_temp[5], 40 * (remember_array + 1) + 28);
				/*保存增益*/
				SPI_Save_Word(weld_controller->temp_gain1 * 100, 40 * (remember_array + 1) + 30);
				SPI_Save_Word(weld_controller->temp_gain2 * 100, 40 * (remember_array + 1) + 32);
				Host_action = 0;
			}

			data_sync_from_computer();
		}

		OSTimeDlyHMSM(0, 0, 0, 25, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}
