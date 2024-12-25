/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-24 16:28:14
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
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

/*调试接口——测试版本*/

#define TEMP_ADJUST 0	 // 温度校准
#define REALTIME_TEMP 1	 // 实时温度显示使能
#define TEMP_CHECK 1	 // 热电偶检测
#define VOLTAGE_CHECK 1	 // 过欠压报警
#define POWER_ON_CHECK 1 // 开机自检报警（开机后完成一次热点电偶检测）
#define RESET_ACTION 1	 // 复位热电偶报警（复位后完成一次热电偶检测）

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
#define ERROR_STK_SIZE 2048
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

// 任务优先级
#define DRAW_TASK_PRIO 7
// 任务堆栈大小
#define DRAW_STK_SIZE 256
// 任务控制块
OS_TCB DRAW_TaskTCB;
// 任务堆栈
CPU_STK DRAW_TASK_STK[DRAW_STK_SIZE];
// 任务函数
void draw_task(void *p_arg);

/*--------------------------------------------------------新触摸屏界面部分------------------------------------------------------------------*/
///////////////////////////////////////////////////////////////////////////////

////////////////////////RS485的消息队列/////////////////////////////////////////
extern u32 baud_list[11];
extern uint8_t USART_RX_BUF[USART_REC_LEN];		// 触摸屏串口接收缓冲
extern u16 realtime_temp_buf[TEMP_BUF_MAX_LEN]; // 温度保存缓冲区
#define UART_Q_NUM 100							// 按键消息队列的数量
OS_Q UART_Msg;									// 串口数据队列
////////////////////////UART3资源保护：互斥锁（暂时未用）////////////////////////
OS_MUTEX UARTMutex;
////////////////////////线程同步：信号量////////////////////////////////////////
OS_SEM PAGE_UPDATE_SEM;		  // 页面刷新信号
OS_SEM COMP_VAL_GET_SEM;	  // 组件属性值成功获取信号
OS_SEM COMP_STR_GET_SEM;	  // 组件属性值(字符串型)成功获取信号
OS_SEM ALARM_RESET_SEM;		  // 报警复位信号
OS_SEM RESET_FINISH;		  // 复位完成信号
OS_SEM COMPUTER_DATA_SYN_SEM; // 上位机数据同步信号
/*主线程使用*/
OS_SEM ERROR_HANDLE_SEM; // 错误信号
OS_SEM TEMP_DRAW_SEM;	 // 绘图信号
////////////////////////新界面组件列表///////////////////////////////////////////
Page_Param *page_param = NULL;			   // 记录当前所在界面id及RDY三个按钮的状态
Component_Queue *param_page_list = NULL;   // 参数设定界面的组件列表
Component_Queue *temp_page_list = NULL;	   // 温度限制界面的组件列表
Component_Queue *setting_page_list = NULL; // 通讯界面组件列表
Error_ctrl *err_ctrl = NULL;			   // 错误注册表
Temp_draw_ctrl *temp_draw_ctrl = NULL;	   // 绘图控制器
weld_ctrl *weld_controller = NULL;		   // 焊接实时控制器
pid_feedforword_ctrl *pid_ctrl = NULL;
/*用户可自行根据需要添加需要的组件列表*/
/*......*/
/////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////错误处理回调/////////////////////////////////////////
static bool Thermocouple_recheck_callback(u8 index);
static bool Current_out_of_ctrl_callback(u8 index);
static bool Temp_up_err_callback(u8 index);
static bool Temp_down_err_callback(u8 index);

/////////////////////////////////////错误复位回调////////////////////////////////////////////
static bool Thermocouple_reset_callback(u8 index);
static bool Current_out_of_ctrl_reset_callback(u8 index);
static bool Temp_up_reset_callback(u8 index);
static bool Temp_down_reset_callback(u8 index);

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

static char *key_name_list[] = {"RDY_SCH", "ION_OFF", "SGW_CTW"};

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
	"SGW_CTW"};

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
	"SGW_CTW"};

static char *setting_page_name_list[] = {
	"adress",
	"baudrate",
};

/*--------------------------------------------------------全局变量------------------------------------------------------------------*/
/*上位机485参数*/
uint8_t ID_OF_MAS = 0;		// 焊机485通讯机号，默认是零,最大可设置15
uint8_t last_id_of_mas = 0; // 机号存储
u32 BOUND_SET = 0;			// 从机波特率设定
u32 last_bound_set = 0;		// 波特率存储
/*温度存储数据*/
/*上位机通信参数*/
extern int statee3;
extern int remember_array_1;
extern uint16_t ModBus_time[6];		  // 焊接时间6段
extern uint16_t ModBus_temp[3];		  // 焊接温度3段
extern uint16_t ModBus_alarm_temp[6]; // 限制温度6段

extern float temp_temperature_coefficient;
extern float temp_temperature_coefficient_1;
extern int temp_remember_coefficient;
extern int temp_remember_coefficient_1;
/*触摸屏通信参数*/
extern uint16_t remember_array;
extern int ION_IOF;
extern int SGW_CTW;
extern int RDY_SCH;
u8 KEY;
int main(void)
{

	/*pid控制器*/
	pid_ctrl = new_pid_forword_ctrl(0, 12, 0.1, 6);
	/*焊接控制器*/
	weld_controller = new_weld_ctrl(pid_ctrl);
	/*新的界面组件列表初始化*/
	page_param = new_page_param();
	/*参数页面*/
	param_page_list = newList(PARAM_PAGE);
	page_list_init(param_page_list,
				   param_page_name_list,
				   sizeof(param_page_name_list) / sizeof(char *));
	/*温度限制页面*/
	temp_page_list = newList(TEMP_PAGE);
	page_list_init(temp_page_list,
				   temp_page_name_list,
				   sizeof(temp_page_name_list) / sizeof(char *));
	/*设置页面*/
	setting_page_list = newList(UART_PAGE);
	page_list_init(setting_page_list,
				   setting_page_name_list,
				   sizeof(setting_page_name_list) / sizeof(char *));

	/*...用户可自行根据需要创建自己页面的队列...*/
	/*错误类型注册表*/
	err_ctrl = new_error_ctrl();
	/*注册错误类型*/
	for (u8 i = 0; i < sizeof(match_list) / sizeof(error_match_list); i++)
	{
		error_handle *handle = new_error_handle(match_list[i].type,
												match_list[i].pic,
												match_list[i].err_callback,
												match_list[i].reset_callback);
		register_error(err_ctrl, handle);
	}

	temp_draw_ctrl = new_temp_draw_ctrl(realtime_temp_buf, 500, 2000, 500);
	/*外设初始化*/
	delay_init(168);								// 时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 中断分组配置
	uart_init(115200);								// 触摸屏通讯接口初始化——波特率：115200
	usart3_init(115200);							// 上位机通信接口初始化
	RNG_Init();										// 随机数初始化
	KEYin_Init();									// 按键输入初始化
	OUT_Init();										// 输出初始化
	SPI1_Init();
	ADC_DMA_INIT();

	/*硬件测试代码段*/

	/*已定义定时器：TIM7-TIM3(time)-TIM5(pid)-TIM1(pwm1)-TIM4(pwm2)*/
	TIM3_Int_Init();
	TIM5_Int_Init();
	TIM2_Int_Init();

	TestCurrent_GPIO_Config();
	PROTECT_Init();
	/*1、串口屏复位*/
	touchscreen_init();
	/*2、该接口已适配*/
	spi_data_init();
	/*3、该接口已适配*/
	welding_data_and_mode_reproduce();

	/*和上位机通信*/
	uint8_t Mas_ID_stauts[5] = {0};
	Mas_ID_stauts[0] = 0x01;
	Mas_ID_stauts[1] = 0x01;		   // 指令码
	Mas_ID_stauts[2] = ID_OF_MAS;	   // 机号地址
	Mas_ID_stauts[3] = remember_array; // 表示焊机工作
	Mas_ID_stauts[4] = 0x01;		   // 表示焊机工作
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 1;   // 设置为发送模式
	for (int t1 = 0; t1 < 5; t1++)
	{
		USART3->SR;
		USART_SendData(USART3, Mas_ID_stauts[t1]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 0; // 设置为接收模式

	/*初始化UCOSIII*/
	OS_ERR err;
	CPU_SR_ALLOC();
	OSInit(&err);
	OS_CRITICAL_ENTER(); // 进入临界区
	// 创建开始任务
	OSTaskCreate((OS_TCB *)&StartTaskTCB,							// 任务控制块
				 (CPU_CHAR *)"start task",							// 任务名字
				 (OS_TASK_PTR)start_task,							// 任务函数
				 (void *)0,											// 传递给任务函数的参数
				 (OS_PRIO)START_TASK_PRIO,							// 任务优先级
				 (CPU_STK *)&START_TASK_STK[0],						// 任务堆栈基地址
				 (CPU_STK_SIZE)START_STK_SIZE / 10,					// 任务堆栈深度限位
				 (CPU_STK_SIZE)START_STK_SIZE,						// 任务堆栈大小
				 (OS_MSG_QTY)0,										// 任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
				 (OS_TICK)0,										// 当使能时间片轮转时的时间片长度，为0时为默认长度，
				 (void *)0,											// 用户补充的存储区
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, // 任务选项
				 (OS_ERR *)&err);									// 存放该函数错误时的返回值

	OS_CRITICAL_EXIT(); // 退出临界区
	/*开启UCOSIII*/
	OSStart(&err);
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------初始化线程--------------------------------------------------------*/
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

#if OS_CFG_SCHED_ROUND_ROBIN_EN // 当使用时间片轮转的时候
	// 使能时间片轮转调度功能,时间片长度为1个系统时钟节拍,1ms
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

	//   创建页面更新信号
	OSSemCreate(&PAGE_UPDATE_SEM, "page update", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	//  创建报警复位信号量
	OSSemCreate(&ALARM_RESET_SEM, "alarm reset", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 创建复位完成信号
	OSSemCreate(&RESET_FINISH, "reset finish", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 创建上位机数据同步信号
	OSSemCreate(&COMPUTER_DATA_SYN_SEM, "data sync", 0, &err);
	if (err != OS_ERR_NONE)
	{
		;
		// 创建失败
	}

	// 创建绘图信号
	OSSemCreate(&TEMP_DRAW_SEM, "temp draw sem", 0, &err);
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
	// 50ms调度——100ms时间片
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
	// 30ms调度
	// 优先级：7
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
	// 优先级：8
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

	// 绘图任务
	// 优先级：8
	OSTaskCreate((OS_TCB *)&DRAW_TaskTCB,
				 (CPU_CHAR *)"draw task",
				 (OS_TASK_PTR)draw_task,
				 (void *)0,
				 (OS_PRIO)DRAW_TASK_PRIO,
				 (CPU_STK *)&DRAW_TASK_STK[0],
				 (CPU_STK_SIZE)DRAW_STK_SIZE / 10,
				 (CPU_STK_SIZE)DRAW_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)10,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	OS_CRITICAL_EXIT();			  // 退出临界区
	OSTaskDel((OS_TCB *)0, &err); // 删除start_task任务自身
}

/**
 * @description: Temperature rise monitoring API
 * @return {*}
 */
static bool Temp_up_check(void)
{
	uint16_t Init_Temperature = 0;
	uint16_t New_Temperature = 0;

#if TEMP_CHECK

	Init_Temperature = TEMP_GAIN1 * ADC_Value_avg(ADC_Channel_7) + TEMP_GAIN2;
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
	TIM_SetCompare1(TIM1, 6000);
	TIM_SetCompare1(TIM4, 6000);

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
	New_Temperature = TEMP_GAIN1 * ADC_Value_avg(ADC_Channel_7) + TEMP_GAIN2;

#endif

	/*判断温度变化*/
	if (New_Temperature < Init_Temperature + 5 || New_Temperature > 4 * Init_Temperature || New_Temperature > 200)
		return false;
	/*热电偶恢复正常*/
	else
	{
		return true;
	}
}

/*---------------------------------------------------------------------------------------*/
/*--------------------------------------错误回调API--------------------------------------*/
/*---------------------------------------------------------------------------------------*/

static bool Temp_up_err_callback(u8 index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}

static bool Temp_down_err_callback(u8 index)
{

	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}

static bool Current_out_of_ctrl_callback(u8 index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}
static bool Thermocouple_recheck_callback(u8 index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}

/*---------------------------------------------------------------------------------------*/
/*--------------------------------------复位回调API--------------------------------------*/
/*---------------------------------------------------------------------------------------*/

static bool Thermocouple_reset_callback(u8 index)
{
	bool ret;
	if (true == Temp_up_check())
	{
		/*热电偶恢复正常,复位*/
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // 清除错误状态
		ret = true;
	}
	else
	{
		/*热点偶仍旧异常*/
		Page_to(page_param, ALARM_PAGE);
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
		ret = false;
	}
	return ret;
}
static bool Current_out_of_ctrl_reset_callback(u8 index)
{

	bool ret = false;
	if (EXTI_GetITStatus(EXTI_Line0) == RESET)
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // 清除错误状态
		ret = true;
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
static bool Temp_up_reset_callback(u8 index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // 清除错误状态
	return true;
}
static bool Temp_down_reset_callback(u8 index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // 清除错误状态
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
			OSSemPost(&PAGE_UPDATE_SEM, OS_OPT_POST_ALL, &err);
			for (u8 i = 0; i < err_ctrl->error_cnt; i++)
			{
				if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->error_callback != NULL)
					err_ctrl->err_list[i]->error_callback(i);
			}
		}
		/*等待用户复位操作*/
		OSSemPend(&ALARM_RESET_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			for (u8 i = 0; i < err_ctrl->error_cnt; i++)
			{
				if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->reset_callback != NULL)
					err_ctrl->err_list[i]->reset_callback(i);
			}
			/*重新配置*/
			uart_init(115200);
			/*串口屏复位*/
			touchscreen_init();
			/*该接口已适配*/
			spi_data_init();
			/*该接口已适配*/
			welding_data_and_mode_reproduce();

			/*复位完成*/
			OSSemPost(&RESET_FINISH, OS_OPT_POST_ALL, &err);
			err_clear(err_ctrl); // 清除出错标志
			ERROR1 = 0;			 // 报警信号（模拟量）清除

			// /*检查是否还有其他错误*/
			// if (err_occur(err_ctrl) == false)
			// {
			// 	err_clear(err_ctrl); // 清除出错标志
			// 	ERROR1 = 0;			 // 报警信号（模拟量）清除
			// 	alram_clear(page_param);
			// 	Page_to(page_param, PARAM_PAGE);
			// }
		}

		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------主线程---------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/
static void Power_on_check(void)
{
#if POWER_ON_CHECK == 1
	// 热电偶开机自检标志位——上电只进行一次检查
	if (false == Temp_up_check())
	{
		OS_ERR err;
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		/*唤醒错误处理线程*/
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
#endif
}

static void Temp_updata_realtime()
{
#if TEMP_ADJUST == 1
	u16 voltage = (ADC_Value_avg(ADC_Channel_7) * 825) >> 10; //*3300/4096
#endif
	weld_controller->realtime_temp = TEMP_GAIN1 * ADC_Value_avg(ADC_Channel_7) + TEMP_GAIN2;
	//	float temp2=0.1618 * adcx7 + 11.048;

#if TEMP_ADJUST == 1
	command_set_comp_val("temp22", "val", voltage);
#endif

	if (page_param->id == PARAM_PAGE)
		command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
}

static void Thermocouple_check(void)
{
	OS_ERR err;
	int adc_refer1 = 0; // 记录一次采集温度
	int adc_refer2 = 0; // 记录下一次采集温度

	// 可加入滤波算法...
	adc_refer1 = ADC_Value_avg(ADC_Channel_7);
	OSTimeDly(100, OS_OPT_TIME_DLY, &err);
	adc_refer2 = ADC_Value_avg(ADC_Channel_7);

	/*插拔过程导致电压突变*/
	if (abs(adc_refer1 - adc_refer2) > VOLTAGE_FLOW)
	{
		// 热电偶依旧异常，报警
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		/*唤醒错误处理线程*/
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
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
	/*part1：开机自检*/
	Power_on_check();
	while (1)
	{

		/*part2：主焊接任务*/
		welding_process();
		/*part3:空闲时进行热点偶监测*/
		Thermocouple_check();
		/*part4:空闲时过欠压监测*/
		voltage_check();
		/*part5：空闲温度采集*/
		Temp_updata_realtime();

		OSTimeDlyHMSM(0, 0, 0, 30, OS_OPT_TIME_PERIODIC, &err); // 休眠50ms
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------触摸屏通信线程--------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/
/*通信任务API*/
static void key_action_callback_param(Component_Queue *page_list);
static void key_action_callback_temp(Component_Queue *page_list);
static void parse_key_action(Page_ID id);

static void CMD_touchscreen_reset_callback(void);
static bool wait_data_parse(OS_TICK wait_time);

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
			for (u8 i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, weld_temp_name_list[i], "val");
				wait_data_parse(15);
			}
			for (u8 i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, weld_time_name_list[i], "val");
				wait_data_parse(15);
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
			{
				/*加载时间参数 温度参数*/
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
		for (u8 i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, weld_temp_name_list[i], "val");
			wait_data_parse(15);
		}
		for (u8 i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, weld_time_name_list[i], "val");
			wait_data_parse(15);
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
			for (u8 i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, gain_name_list[i], "val");
				wait_data_parse(15);
			}
			for (u8 i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
			{
				/*参数读取*/
				command_get_comp_val(page_list, alarm_temp_name_list[i], "val");
				wait_data_parse(15);
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
			{
				/*降低内存读写次数*/
				Load_param_alarm(weld_controller, GP); // 加载温度限制/温度增益参数
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
		/*Ⅰ、读取界面上的参数*/
		for (u8 i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, gain_name_list[i], "val");
			wait_data_parse(15);
		}
		for (u8 i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
		{
			/*参数读取*/
			command_get_comp_val(page_list, alarm_temp_name_list[i], "val");
			wait_data_parse(15);
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
static bool pid_param_get(u16 *pid_raw_param)
{
	const char *pid_param_name_list[4] = {
		"kpf",
		"kp",
		"ki",
		"kd"};

	uint8_t *msg = NULL;
	OS_ERR err;
	OS_MSG_SIZE msg_size = 0;
	u8 param_get_success_cnt = 0;

	for (u8 i = 0; i < 4; i++)
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
	//	OS_ERR err;
	const char *tick_name[] = {"tick1", "tick2", "tick3", "tick4", "tick5"};
	switch (page_param->id)
	{
	case PARAM_PAGE:
	{
		/*1、读取界面上的参数*/
		for (u8 i = 0; i < sizeof(key_name_list) / sizeof(char *); i++)
		{
			command_get_comp_val(param_page_list, key_name_list[i], "pic");
			wait_data_parse(20);
		}
		command_get_comp_val(param_page_list, "GP", "val");
		wait_data_parse(20);
		// command_get_comp_val(param_page_list, "sensortype", "val");
		// wait_data_parse(20);
		parse_key_action(page_param->id);
	}
	break;

	case TEMP_PAGE:
	{
		/*1、读取界面上的参数*/
		for (u8 i = 0; i < sizeof(key_name_list) / sizeof(char *); i++)
		{
			command_get_comp_val(temp_page_list, key_name_list[i], "pic");
			wait_data_parse(20);
		}
		command_get_comp_val(temp_page_list, "GP", "val");
		wait_data_parse(20);
		parse_key_action(page_param->id);
	}
	break;

	case WAVE_PAGE:
	{
#if PID_DEBUG == 1
		u16 pid_param[4] = {0};
		if (pid_param_get(pid_param) == true)
		{
			weld_controller->pid_ctrl->kp_f = pid_param[0] / 100.0;
			weld_controller->pid_ctrl->kp = pid_param[1] / 100.0;
			weld_controller->pid_ctrl->ki = pid_param[2] / 100.0;
			weld_controller->pid_ctrl->kd = pid_param[3] / 100.0;
		}

#endif
		command_set_comp_val("step3", "val", weld_controller->realtime_temp);
		/*更新坐标*/
		u16 total_time = 0;		// 总焊接时长
		u16 delta_tick = 0;		// 坐标间隔
		u16 total_tick_len = 0; // 横坐标总长度
		u16 win_width = 0;		// 绘图区域占据的实际窗口大小

		for (u8 i = 0; i < 5; i++)
			total_time += weld_controller->weld_time[i];
		/*坐标划分*/
		if (total_time < 500)
			delta_tick = 100;
		else if (total_time < 1000 && total_time > 500)
			delta_tick = 200;
		else if (total_time > 1000 && total_time <= 2500)
			delta_tick = 500;
		else if (total_time > 2500 && total_time <= 5000)
			delta_tick = 1000;
		else
			delta_tick = 2000;
		/*绘图间隔*/
		total_tick_len = 5 * delta_tick;
		win_width = WIN_WIDTH * total_time / total_tick_len;
		temp_draw_ctrl->delta_tick = total_time / win_width + 1;
		/*坐标发送到触摸屏*/
		for (u8 i = 0; i < sizeof(tick_name) / sizeof(char *); i++)
		{
			command_set_comp_val(tick_name[i], "val", (1 + i) * delta_tick);
		}
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
		/*...机地址以及波特率修改，同步到上位机...*/
		/*1、读取界面设定的地址*/
		command_get_comp_val(setting_page_list, "adress", "val");
		wait_data_parse(20);
		/*2、读取页面设定的波特率*/
		command_get_comp_val(setting_page_list, "baudrate", "val");
		wait_data_parse(20);
		/*更新波特率*/
		if (get_comp(setting_page_list, "baudrate") != NULL)
		{
			uint8_t index = get_comp(setting_page_list, "baudrate")->val;
			usart3_set_bound(baud_list[index]);
		}
	}
	break;

	default:
		/*...用户可自行添加需要的页面...*/
		break;
	}
}

static void CMD_touchscreen_reset_callback()
{
	u8 remember_array_init = 0;
	Load_param(weld_controller, remember_array_init);
	Load_param_alarm(weld_controller, remember_array_init);

	command_send_raw("page 4");
	command_set_comp_val("ION_OFF", "pic", ION);
	for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
	{
		Component *comp = get_comp(temp_page_list, alarm_temp_name_list[i]);
		if (comp != NULL)
			command_set_comp_val(alarm_temp_name_list[i], "val", comp->val);
	}
	for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
	{
		Component *comp = get_comp(temp_page_list, gain_name_list[i]);
		if (comp != NULL)
			command_set_comp_val(gain_name_list[i], "val", comp->val);
	}
	command_set_comp_val("count", "val", 0);

	command_send("page 1");
	command_set_comp_val("ION_OFF", "pic", ION);
	/*发送到触摸屏*/
	for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
	{
		Component *comp = get_comp(param_page_list, weld_time_name_list[i]);
		if (comp != NULL)
			command_set_comp_val(weld_time_name_list[i], "val", comp->val);
	}
	for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
	{
		Component *comp = get_comp(param_page_list, weld_temp_name_list[i]);
		if (comp != NULL)
			command_set_comp_val(weld_temp_name_list[i], "val", comp->val);
	}
	command_set_comp_val("count", "val", 0);
}

static bool wait_data_parse(OS_TICK wait_time)
{
	/*补充一个页面切换的处理*/
	uint8_t *msg = NULL;
	OS_ERR err;
	OS_MSG_SIZE msg_size = 0;
	bool ret = false;
	/*订阅队列，并进行消息处理*/
	msg = (uint8_t *)OSQPend(&UART_Msg,			   // 消息队列指针
							 wait_time,			   // 等待时长
							 OS_OPT_PEND_BLOCKING, // 等待模式
							 &msg_size,			   // 获取消息大小
							 NULL,				   // 获取消息的时间戳
							 &err);				   // 返回错误代码

	/*满足标准通信格式*/
	if (err == OS_ERR_NONE && msg != NULL && msg_size >= MIN_CMD_LEN && msg[msg_size - 1] == END_FLAG && msg[msg_size - 2] == END_FLAG && msg[msg_size - 3] == END_FLAG)
	{

		/*处理消息*/
		switch (msg[0])
		{
		case CMD_FAIL:
			ret = true;
			break;
		case CMD_OK:
			ret = true;
			break;
		case CMD_PAGEID_RETURN:
			ret = true;
			/*更新页面id*/
			if (msg[1] <= UART_PAGE && msg[1] >= PARAM_PAGE)
				page_param->id = (Page_ID)msg[1];
			break;
		case CMD_INT_VAR_RETURN:
			ret = true;
			if (PARAM_PAGE == page_param->id)
				param_page_list->updata->val = msg[1] | msg[2] << 8 | msg[3] << 16 | msg[4] << 24;
			else if (TEMP_PAGE == page_param->id)
				temp_page_list->updata->val = msg[1] | msg[2] << 8 | msg[3] << 16 | msg[4] << 24;
			else if (UART_PAGE == page_param->id)
				setting_page_list->updata->val = msg[1] | msg[2] << 8 | msg[3] << 16 | msg[4] << 24;
			break;
		case CMD_STR_VAR_RETURN:
			ret = true;
			break;
		case CMD_DATA_TRANSFER_READY:
			ret = true;
			break;
		default:
			break;
		}

		if (msg[msg_size - 1] == END_FLAG && msg[msg_size - 2] == END_FLAG && msg[msg_size - 3] == END_FLAG && msg_size >= 10 && msg[6] == CMD_SYSTEM_START_OK)
		{
			CMD_touchscreen_reset_callback();
			ret = true;
		}
	}

	/*清空缓存*/
	set_receive_number(0);
	for (u16 i = 0; i < USART_REC_LEN; i++)
	{
		USART_RX_BUF[i] = 0;
	}
	return ret;
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

		for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
		{
			Component *comp = get_comp(param_page_list, weld_time_name_list[i]);
			if (comp != NULL)
				weld_controller->weld_time[i] = comp->val;
		}
		for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
		{
			Component *comp = get_comp(param_page_list, weld_temp_name_list[i]);
			if (comp != NULL)
				weld_controller->weld_temp[i] = comp->val;
		}

		break;
	}

	case TEMP_PAGE:
	{

		for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
		{
			Component *comp = get_comp(temp_page_list, alarm_temp_name_list[i]);
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
		if (get_comp(temp_page_list, "GAIN3") != NULL)
		{
			float gain = get_comp(temp_page_list, "GAIN3")->val / 100.0;
			weld_controller->temp_gain3 = gain;
		}
		break;
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
	uart_init(115200);

	while (1)
	{
		/*触发报警*/
		OSSemPend(&PAGE_UPDATE_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			page_param->id = ALARM_PAGE;
		}
		else
		{
			/*1、查询页面id，根据所处页面执行不同动作*/
			Page_id_get();
			wait_data_parse(100);
			/*2、处理页面逻辑*/
			page_process(page_param->id);
			/*3、数据同步*/
			data_syn(page_param->id);
		}

		OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------上位机通信线程--------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/
/*
	通讯任务
	功能描述：
	和上位机的通信任务，读取来自上位机的数据，更新响应参数值，并通知数据同步线程完成参数更新
*/
void computer_read_task(void *p_arg)
{
	/*
	上位机维护的数据：
	ModBus_time ModBus_temp ModBus_alarm_temp remember_array_1 temp_remember_coefficient temp_temperature_coefficient temp_remember_coefficient_1 temp_temperature_coefficient_1
	用户维护的数据：
	weld_controller->weld_time 	weld_controller->weld_temp weld_controller->alarm_temp
	上位机修改数据后
	（1）更新用户维护的数据
	（2）完成数据同步
	*/
	OS_ERR err;
	while (1)
	{

		/*订阅上位机的数据更新信号*/
		OSSemPend(&COMPUTER_DATA_SYN_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{

			/*1、statee3不为0时为处理上位机数据，并将数据刷新到触摸屏*/
			if (statee3 == 1)
			{
				/*1、数据更新*/
				remember_array = remember_array_1; // ：上位机维护的数据 remember_array：全局维护的GP值
				/*2、屏幕刷新*/
				for (int i = 0; i < 2; i++)
				{
					command_set_comp_val("GP", "val", remember_array); // 将上一次的GP值发给从机
					Load_param(weld_controller, remember_array);
					Load_param_alarm(weld_controller, remember_array);
				}
				SPI_Save_Word(0, 40 * (remember_array + 1) + 4);
				statee3 = 0;
			}
			else if ((statee3 >= 2) && (statee3 <= 18)) // statee3 为2 - 18 ，表示上位机修改单个数据，同步修改控制板参数数组和触摸屏数据
			{
				/*1、数据更新*/
				switch (statee3)
				{

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
				case 7:
					weld_controller->weld_temp[0] = ModBus_temp[0];
					break;
				case 8:
					weld_controller->weld_temp[1] = ModBus_temp[1];
					break;
				case 9:
					weld_controller->weld_temp[2] = ModBus_temp[2];
					break;
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
				case 16:
					weld_controller->temp_gain1 = temp_temperature_coefficient;
					break;
				case 17:
					weld_controller->temp_gain2 = temp_temperature_coefficient_1;
					break;
				}
				/*2、屏幕刷新*/
				char *name = (char *)calloc(10, sizeof(char));
				if (name)
				{

					for (uint8_t i = 1; i <= sizeof(weld_controller->weld_temp) / sizeof(uint16_t); i++)
					{
						memset(name, 0, 10);
						sprintf(name, "param_page.temp%d", i);
						command_set_comp_val(name, "val", weld_controller->weld_temp[i - 1]);
					}
					for (uint8_t i = 1; i <= sizeof(weld_controller->weld_time) / sizeof(uint16_t); i++)
					{
						memset(name, 0, 10);
						sprintf(name, "param_page.time%d", i);
						command_set_comp_val(name, "val", weld_controller->weld_time[i - 1]);
					}
					for (uint8_t i = 1; i <= sizeof(weld_controller->alarm_temp) / sizeof(uint16_t); i++)
					{
						memset(name, 0, 10);
						sprintf(name, "temp_page.temp%d", i);
						command_set_comp_val(name, "val", weld_controller->weld_time[i - 1]);
					}
					command_set_comp_val("temp_page.GAIN1", "val", weld_controller->temp_gain1 * 100);
					command_set_comp_val("temp_page.GAIN2", "val", weld_controller->temp_gain2 * 100);

					free(name);
				}
				/*3、将用户数据更新至外部flash*/
				SPI_Save_Word(weld_controller->weld_time[0], 40 * (remember_array + 1)); // 保存更新数据在EEPROM
				SPI_Save_Word(weld_controller->weld_time[1], 40 * (remember_array + 1) + 2);
				SPI_Save_Word(0, 40 * (remember_array + 1) + 4);
				SPI_Save_Word(weld_controller->weld_time[2], 40 * (remember_array + 1) + 6);
				SPI_Save_Word(weld_controller->weld_time[3], 40 * (remember_array + 1) + 8);
				SPI_Save_Word(weld_controller->weld_time[4], 40 * (remember_array + 1) + 10);

				SPI_Save_Word(weld_controller->weld_temp[0], 40 * (remember_array + 1) + 12);
				SPI_Save_Word(weld_controller->weld_temp[1], 40 * (remember_array + 1) + 14);
				SPI_Save_Word(weld_controller->weld_temp[2], 40 * (remember_array + 1) + 16);
				SPI_Save_Word(weld_controller->alarm_temp[0], 40 * (remember_array + 1) + 18);
				SPI_Save_Word(weld_controller->alarm_temp[1], 40 * (remember_array + 1) + 20);
				SPI_Save_Word(weld_controller->alarm_temp[2], 40 * (remember_array + 1) + 22);
				SPI_Save_Word(weld_controller->alarm_temp[3], 40 * (remember_array + 1) + 24);
				SPI_Save_Word(weld_controller->alarm_temp[4], 40 * (remember_array + 1) + 26);
				SPI_Save_Word(weld_controller->alarm_temp[5], 40 * (remember_array + 1) + 28);
				SPI_Save_Word(weld_controller->temp_gain1 * 100, 40 * (remember_array + 1) + 30);
				SPI_Save_Word(weld_controller->temp_gain2 * 100, 40 * (remember_array + 1) + 32);

				statee3 = 0;
			}
			else if (statee3 == 20) // statee3= 20 表示改变整一页数据，将修改数据同步到参数数组和触摸屏，
			{
				/*1、数据更新*/
				remember_array = remember_array_1;

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

				/*2、屏幕刷新*/
				command_set_comp_val("GP", "val", remember_array); // 将GP值发给从机
				char *name = (char *)calloc(10, sizeof(char));
				if (name)
				{

					for (uint8_t i = 1; i <= sizeof(weld_controller->weld_temp) / sizeof(uint16_t); i++)
					{
						memset(name, 0, 10);
						sprintf(name, "param_page.temp%d", i);
						command_set_comp_val(name, "val", weld_controller->weld_temp[i - 1]);
					}
					for (uint8_t i = 1; i <= sizeof(weld_controller->weld_time) / sizeof(uint16_t); i++)
					{
						memset(name, 0, 10);
						sprintf(name, "param_page.time%d", i);
						command_set_comp_val(name, "val", weld_controller->weld_time[i - 1]);
					}
					for (uint8_t i = 1; i <= sizeof(weld_controller->alarm_temp) / sizeof(uint16_t); i++)
					{
						memset(name, 0, 10);
						sprintf(name, "temp_page.temp%d", i);
						command_set_comp_val(name, "val", weld_controller->alarm_temp[i - 1]);
					}
					command_set_comp_val("temp_page.GAIN1", "val", weld_controller->temp_gain1 * 100);
					command_set_comp_val("temp_page.GAIN2", "val", weld_controller->temp_gain2 * 100);

					free(name);
				}

				/*3、将用户数据更新至外部flash*/
				SPI_Save_Word(weld_controller->weld_time[0], 40 * (remember_array + 1));
				SPI_Save_Word(weld_controller->weld_time[1], 40 * (remember_array + 1) + 2);
				SPI_Save_Word(weld_controller->weld_time[2], 40 * (remember_array + 1) + 4);
				SPI_Save_Word(weld_controller->weld_time[3], 40 * (remember_array + 1) + 6);
				SPI_Save_Word(weld_controller->weld_time[4], 40 * (remember_array + 1) + 8);

				SPI_Save_Word(weld_controller->weld_temp[0], 40 * (remember_array + 1) + 12);
				SPI_Save_Word(weld_controller->weld_temp[1], 40 * (remember_array + 1) + 14);
				SPI_Save_Word(weld_controller->weld_temp[2], 40 * (remember_array + 1) + 16);
				SPI_Save_Word(weld_controller->alarm_temp[0], 40 * (remember_array + 1) + 18);
				SPI_Save_Word(weld_controller->alarm_temp[1], 40 * (remember_array + 1) + 20);
				SPI_Save_Word(weld_controller->alarm_temp[2], 40 * (remember_array + 1) + 22);
				SPI_Save_Word(weld_controller->alarm_temp[3], 40 * (remember_array + 1) + 24);
				SPI_Save_Word(weld_controller->alarm_temp[4], 40 * (remember_array + 1) + 26);
				SPI_Save_Word(weld_controller->alarm_temp[5], 40 * (remember_array + 1) + 28);
				SPI_Save_Word(weld_controller->temp_gain1 * 100, 40 * (remember_array + 1) + 30);
				SPI_Save_Word(weld_controller->temp_gain2 * 100, 40 * (remember_array + 1) + 32);
				statee3 = 0;
			}

			statee3 = 0;

			/*2、数据同步：上位机——>旧数据接口——>新数据接口*/
			char *name = (char *)calloc(10, sizeof(char));
			if (name != NULL)
			{
				/*参数界面组件*/
				/*1、同步GP*/
				get_comp(param_page_list, "GP")->val = remember_array;
				get_comp(temp_page_list, "GP")->val = remember_array;
				/*2、同步6个焊接时间*/
				for (uint8_t i = 1; i <= sizeof(weld_controller->weld_time) / sizeof(uint16_t); i++)
				{
					memset(name, 0, 10);
					sprintf(name, "time%d", i);
					get_comp(param_page_list, name)->val = weld_controller->weld_time[i - 1];
				}
				/*3、同步3个焊接温度*/
				for (uint8_t i = 1; i <= sizeof(weld_controller->weld_temp) / sizeof(uint16_t); i++)
				{
					memset(name, 0, 10);
					sprintf(name, "temp%d", i);
					get_comp(param_page_list, name)->val = weld_controller->weld_temp[i - 1];
				}

				/*温度界面组件*/
				/*4、同步6个限制温度*/
				for (uint8_t i = 1; i <= sizeof(weld_controller->alarm_temp) / sizeof(uint16_t); i++)
				{
					memset(name, 0, 10);
					sprintf(name, "temp%d", i);
					get_comp(temp_page_list, name)->val = weld_controller->alarm_temp[i - 1];
				}
				/*5、同步温度系数*/
				get_comp(temp_page_list, "GAIN1")->val = weld_controller->temp_gain1;
				get_comp(temp_page_list, "GAIN2")->val = weld_controller->temp_gain2;

				free(name);
			}
		}

		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------绘图线程--------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/
void draw_task(void *p_arg)
{
	OS_ERR err;
	const char *temp_display_name[] = {"step1", "step2", "step3"};
	u16 temp_display[3] = {0};
	while (1)
	{

		/*焊接结束后继续采集温度*/
		OSSemPend(&TEMP_DRAW_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			/*三段均温显示*/
			temp_display[0] = weld_controller->second_step_start_temp;
			u32 sum = 0;
			for (u16 i = temp_draw_ctrl->second_step_stable_index; i < temp_draw_ctrl->second_step_index_end; i++)
			{
				sum += temp_draw_ctrl->temp_buf[i];
			}
			temp_display[1] = sum / (temp_draw_ctrl->second_step_index_end - temp_draw_ctrl->second_step_stable_index + 1);
			/*一二段均值发送到触摸屏*/
			if (page_param->id == WAVE_PAGE)
			{
				command_set_comp_val(temp_display_name[0], "val", temp_display[0]);
				command_set_comp_val(temp_display_name[1], "val", temp_display[1]);
			}

			/*绘图控制器复位*/
			reset_temp_draw_ctrl(temp_draw_ctrl, weld_controller->weld_time);

			/*绘图结束清空缓存*/
			memset(temp_draw_ctrl->temp_buf, 0, sizeof(temp_draw_ctrl->temp_buf) / sizeof(u16));
		}

		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}
