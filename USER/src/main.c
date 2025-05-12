/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-03-19 08:22:00
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-05-12 09:49:59
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

#include "includes.h"
#include "sys.h"
#include "delay.h"

#include "usart.h"
#include "key.h"
#include "crc16.h"
#include "welding_process.h"
#include "spi.h"
#include "timer.h"
#include "adc.h"
#include "protect.h"
#include "pwm.h"
#include "Kalman.h"
#include "touchscreen.h"
#include "pid.h"
#include "filter.h"
#include "thermocoupleIO.h"
#include "usbh_msc_usr.h"
#include "log.h"

/* Private define ------------------------------------------------------------*/
/*user config*/
#define USE_STM32_DEMO 0  // USB test code
#define KEJ_CHECK_DUTY 40 // Thermocouple detection filter cycles
#define ROOM_TEMP 20	  // Default room temperature
#define SAMPLE_LEN 100	  // Sampling depth
#define ADC_BIAS_MAX 650  // Maximum offset value

/*debug option*/
#define TEMP_ADJUST 0	  // Temperature calibration
#define VOLTAGE_CHECK 1	  // Overvoltage and undervoltage alarm
#define OVER_LOAD_CHECK 1 // Overload protection
#define POWER_ON_CHECK 1  // Boot detection

/* Private macro -------------------------------------------------------------*/
#define START_TASK_PRIO 3
#define START_STK_SIZE 128
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

#define ERROR_TASK_PRIO 4
#define ERROR_STK_SIZE 1024
OS_TCB ErrorTaskTCB;
CPU_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *p_arg);

#define MAIN_TASK_PRIO 5
#define MAIN_STK_SIZE 3072
OS_TCB Main_TaskTCB;
CPU_STK MAIN_TASK_STK[MAIN_STK_SIZE];
void main_task(void *p_arg);

#define READ_TASK_PRIO 6
#define READ_STK_SIZE 2048
OS_TCB READ_TaskTCB;
CPU_STK READ_TASK_STK[READ_STK_SIZE];

void read_task(void *p_arg);

#define COMPUTER_TASK_PRIO 7
#define COMPUTER_STK_SIZE 512
OS_TCB COMPUTER_TaskTCB;
CPU_STK COMPUTER_TASK_STK[COMPUTER_STK_SIZE];
void computer_read_task(void *p_arg);

#define USB_TASK_PRIO 6
#define USB_STK_SIZE 2048
OS_TCB USB_TaskTCB;
CPU_STK USB_TASK_STK[USB_STK_SIZE];
void usb_task(void *p_arg);

/* Private function prototypes -----------------------------------------------*/

/*Error handling callbacks*/
static bool Temp_up_err_callback(uint8_t index);
static bool Temp_down_err_callback(uint8_t index);
static bool Current_out_of_ctrl_callback(uint8_t index);
static bool Thermocouple_recheck_callback(uint8_t index);
/*Error reset callback*/
static bool Thermocouple_reset_callback(uint8_t index);
static bool Current_out_of_ctrl_reset_callback(uint8_t index);
static bool Temp_up_reset_callback(uint8_t index);
static bool Temp_down_reset_callback(uint8_t index);
/*main task functions*/
static void Power_on_check(void);
static void Power_on_check(void);
static void Temp_updata_realtime(void);
static void Thermocouple_check(void);
static void voltage_check(void);
static void Overload_check(void);

/*Using hardware calibration mode,
different thermocouples have different board-level amplification coefficients.
Calibration is required separately, but the software layer is consistent*/
// E：0.222*3300/4096=0.1788
// J：0.2189*3300/4096=0.1763
// K：0.218*3300/4096=0.1756
static Thermocouple Thermocouple_Lists[] = {
	{E_TYPE, 0.17, 0, 0},
	{K_TYPE, 0.17, 0, 0}, // 主控板上的热电偶检测通道，实际上校准成了E
	{J_TYPE, 0.17, 0, 0},
};

const error_match_list match_list[] = {
	{CURRENT_OUT_OT_CTRL, "f1", Current_out_of_ctrl_callback, Current_out_of_ctrl_reset_callback},
	{TEMP_UP, "f2", Temp_up_err_callback, Temp_up_reset_callback},
	{TEMP_DOWN, "f3", Temp_down_err_callback, Temp_down_reset_callback},
	{VOLTAGE_TOO_HIGH, "f4", NULL, NULL},
	{VOLTAGE_TOO_LOW, "f5", NULL, NULL},
	{MCU_OVER_HEAT, "f6", NULL, NULL},
	{TRANSFORMER_OVER_HEAT, "f7", NULL, NULL},
	{SENSOR_ERROR, "f8", Thermocouple_recheck_callback, Thermocouple_reset_callback},
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
	"GAIN2"};

/* Private variables ---------------------------------------------------------*/
// 11 default baud rates
extern uint32_t baud_list[11];
// The serial port of the touch screen receives buffering
extern uint8_t USART_RX_BUF[USART_REC_LEN];
// Temperature preservation buffer
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN];
// The maximum length of a serial message queue
#define UART_Q_NUM 100
// Serial data queue
OS_Q UART_Msg;
/*UART3 Resource Protection: Mutex (Temporarily Unused)*/
OS_MUTEX UARTMutex;

/*Thread Synchronization: Semaphore*/
// Page refresh signal
OS_SEM PAGE_UPDATE_SEM;
// The component property value successfully obtains the signal
OS_SEM COMP_VAL_GET_SEM;
// The component property value (string) successfully obtains the signal
OS_SEM COMP_STR_GET_SEM;
// Alarm reset signal
OS_SEM ALARM_RESET_SEM;
// Thermocouple calibration signal
OS_SEM SENSOR_UPDATE_SEM;
// Host computer data synchronization signal
OS_SEM COMPUTER_DATA_SYN_SEM;
// The upper computer turns on the welding signal
OS_SEM HOST_WELD_CTRL_SEM;
// err sem
OS_SEM ERROR_HANDLE_SEM;
// draw sem
OS_SEM TEMP_DISPLAY_SEM;

/*A list of new interface components*/
// Record the ID of the current screen and the status of the three buttons
Page_Param *page_param = NULL;
// A list of components on the parameter setting screen
Component_Queue *param_page_list = NULL;
// A list of components for the temperature limit interface
Component_Queue *temp_page_list = NULL;
// A list of communication interface components
Component_Queue *setting_page_list = NULL;
// A list of components on the Waveform page
Component_Queue *wave_page_list = NULL;
/*Users can add the required component list as needed*/
/*......*/

// error controller
Error_ctrl *err_ctrl = NULL;
// Drawing controllers
Temp_draw_ctrl *temp_draw_ctrl = NULL;
// Welding real-time controller
weld_ctrl *weld_controller = NULL;
pid_feedforword_ctrl *pid_ctrl = NULL;
// Current thermocouple
Thermocouple *current_Thermocouple = NULL;
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

static char *wave_page_name_list[] = {
	"kp",
	"ki",
	"kd"};

/*host computer 485 param*/
/*max to 15*/
uint8_t ID_OF_MAS = 0;
uint8_t last_id_of_mas = 0;
uint32_t BOUND_SET = 0;
/*Communication parameters of the host computer*/
extern int Host_action;
extern int Host_GP;
extern uint16_t ModBus_time[6];
extern uint16_t ModBus_temp[3];
extern uint16_t ModBus_alarm_temp[6];
extern float Host_gain1;
extern float Host_gain2;
extern int Host_gain1_raw;
extern int Host_gain2_raw;
/*Touchscreen communication parameters*/
extern uint16_t remember_array;
extern int ION_IOF;
extern int SGW_CTW;
extern int RDY_SCH;
extern USBH_HOST USB_Host __ALIGN_END;
extern USB_OTG_CORE_HANDLE USB_OTG_Core __ALIGN_END;

int main(void)
{

	/*------------------------------------------------------User layer data objects-----------------------------------------------------------*/
	/*pid controller*/
	pid_ctrl = new_pid_forword_ctrl(0, 12, 0.022, 15);
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

	/*wave list*/
	wave_page_list = newList(WAVE_PAGE);
	page_list_init(wave_page_list,
				   wave_page_name_list,
				   sizeof(wave_page_name_list) / sizeof(char *));

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
	current_Thermocouple = &Thermocouple_Lists[0];

	/*------------------------------------------------------Hardware layer data initialization-----------------------------------------------------------*/
	/*Peripheral initialization*/
	delay_init(168);								// clock init
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // Interrupt Group Configuration
	uart_init(115200);								// Touch screen communication interface initialization
	usart3_init(115200);							// Host computer communication initialization
	KEYin_Init();									// key io init
	OUT_Init();										// output pin init
	Check_IO_init();								// Thermocouple detection io initialization
	SPI1_Init();									// spi init
	ADC_DMA_INIT();									// ADC init
	Current_Check_IO_Config();						// Current detection io configuration
	Temp_Protect_IO_Config();						// temp overload init

	/*TIM3(weld time) TIM5(pid) TIM1(pwm1) TIM4(pwm2)*/
	TIM3_Int_Init();
	TIM5_Int_Init();

	/*user device init*/
	Touchscreen_init();
	Load_data_from_mem();
	Host_computer_reset();

	/*Hardware test snippet*/
	/*...*/

	/*-----------------------------------------------------------System level data objects-----------------------------------------------------------------*/
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

	/*Statistical tasks*/
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);
#endif

/*If the measurement interrupt shutdown time is enabled*/
#ifdef CPU_CFG_INT_DIS_MEAS_EN
	CPU_IntDisMeasMaxCurReset();
#endif

/*
When time slice rotation is used,
the time slice rotation scheduling function is enabled,
and the time slice length is 1 system clock beat, 1 ms
*/
#if OS_CFG_SCHED_ROUND_ROBIN_EN
	OSSchedRoundRobinCfg(DEF_ENABLED, 1, &err);
#endif

	OS_CRITICAL_ENTER();
	// Create a message queue UART Msg
	OSQCreate((OS_Q *)&UART_Msg,	  // 消息队列
			  (CPU_CHAR *)"UART Msg", // 消息队列名称
			  (OS_MSG_QTY)UART_Q_NUM, // 消息队列长度，这里设置为100
			  (OS_ERR *)&err);		  // 错误码

	// Create a mutex - serial port resource access (temporarily unused)
	OSMutexCreate(&UARTMutex, "USART_Mutex", &err);

	/*--------------------------------------------SEM use for UI--------------------------------------------*/
	// Page update signals
	OSSemCreate(&PAGE_UPDATE_SEM, "page update", 0, &err);
	// semaphore specifically used to obtain the value of a component's attributes
	OSSemCreate(&COMP_VAL_GET_SEM, "comp val get", 0, &err);
	OSSemCreate(&COMP_STR_GET_SEM, "comp str get", 0, &err);
	// Thermocouple calibration signal
	OSSemCreate(&SENSOR_UPDATE_SEM, "SENSOR_UPDATE_SEM", 0, &err);

	/*--------------------------------------------SEM use for host computer-----------------------------------*/
	// 创建上位机数据同步信号
	OSSemCreate(&COMPUTER_DATA_SYN_SEM, "data sync", 0, &err);
	if (err != OS_ERR_NONE)
		// 创建上位机开启焊接信号量
		OSSemCreate(&HOST_WELD_CTRL_SEM, "HOST_WELD_CTRL_SEM", 0, &err);
	/*--------------------------------------------other SEM----------------------------------------------------*/
	//  创建报警复位信号量
	OSSemCreate(&ALARM_RESET_SEM, "alarm reset", 0, &err);
	// 创建绘图信号
	OSSemCreate(&TEMP_DISPLAY_SEM, "temp draw sem", 0, &err);
	// 创建报错信号
	OSSemCreate(&ERROR_HANDLE_SEM, "err sem", 0, &err);

	// 错误处理任务——最高优先级
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
				 (OS_TICK)10, // 最大连续运行时长（时间片）
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// 创建主任务
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
				 (OS_TICK)100,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// 创建触摸屏通讯任务
	// 20ms调度
	OSTaskCreate((OS_TCB *)&READ_TaskTCB,
				 (CPU_CHAR *)"Read task",
				 (OS_TASK_PTR)read_task,
				 (void *)0,
				 (OS_PRIO)READ_TASK_PRIO,
				 (CPU_STK *)&READ_TASK_STK[0],
				 (CPU_STK_SIZE)READ_STK_SIZE / 10,
				 (CPU_STK_SIZE)READ_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)20,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// USB任务
	OSTaskCreate((OS_TCB *)&USB_TaskTCB,
				 (CPU_CHAR *)"usb task",
				 (OS_TASK_PTR)usb_task,
				 (void *)0,
				 (OS_PRIO)USB_TASK_PRIO,
				 (CPU_STK *)&USB_TASK_STK[0],
				 (CPU_STK_SIZE)USB_STK_SIZE / 10,
				 (CPU_STK_SIZE)USB_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)40,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// 创建上位机通讯任务
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

static void Thermocouple_check(void);
/*--------------------------------------------------------------------------------------*/
/*--------------------------------------err callback------------------------------------*/
/*--------------------------------------------------------------------------------------*/

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
/*--------------------------------------reset callback-----------------------------------*/
/*---------------------------------------------------------------------------------------*/

static bool Thermocouple_reset_callback(uint8_t index)
{
	bool ret;

	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // clear error state
	ret = true;

	return ret;
}
static bool Current_out_of_ctrl_reset_callback(uint8_t index)
{

	bool ret = false;

	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // clear error state
	ret = true;

	return ret;
}
static bool Temp_up_reset_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // clear error state

	return true;
}
static bool Temp_down_reset_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // clear error state

	return true;
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------err Thread---------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

void error_task(void *p_arg)
{
	OS_ERR err;

	while (1)
	{
		OSSemPend(&ERROR_HANDLE_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{
			/*pwm off*/
			TIM_SetCompare1(TIM1, 0);
			TIM_SetCompare1(TIM4, 0);
			TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
			TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Cmd(TIM1, DISABLE);

			/*timer reset*/
			TIM_Cmd(TIM3, DISABLE);
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			TIM3->CNT = 0;
			TIM_Cmd(TIM5, DISABLE);
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
			TIM5->CNT = 0;

			/*Valve off*/
			ERROR1 = 1; // error signal
			RLY10 = 0;	// Valve1 off
			RLY11 = 0;	// Valve2 off
			RLY12 = 0;	// Valve3 off

			/*3、err handle*/
			Page_to(page_param, ALARM_PAGE);
			page_param->id = ALARM_PAGE;
			for (uint8_t i = 0; i < err_ctrl->max_len; i++)
			{
				if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->error_callback != NULL)
					err_ctrl->err_list[i]->error_callback(i);
			}
			/*wait until user reset*/
			OSSemPend(&ALARM_RESET_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
			if (err == OS_ERR_NONE)
			{
				ERROR1 = 0; // error signal reset
				for (uint8_t i = 0; i < err_ctrl->max_len; i++)
				{
					if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->reset_callback != NULL)
						err_ctrl->err_list[i]->reset_callback(i);
				}
			}
		}

		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err);
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------main Thread-------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/
/*------------------------------Condition Monitoring API--------------------------------*/
/*--------------------------------------------------------------------------------------*/

static void Power_on_check(void)
{
#if POWER_ON_CHECK == 1
	OS_ERR err;
	/*1、开机自检需要检测当前是哪一路热电偶*/
	GPIO_SetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
	GPIO_SetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
	GPIO_SetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);

	OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err);

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

	/*no Thermocouple detect*/
	if (current_Thermocouple == NULL)
	{
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}

	/*load last time adjust data*/
	// switch (current_Thermocouple->type)
	// {
	// case K_TYPE:
	// 	current_Thermocouple->Bias = SPI_Load_Word(CARLIBRATION_BASE(remember_array));
	// 	break;
	// case E_TYPE:
	// 	current_Thermocouple->Bias = SPI_Load_Word(CARLIBRATION_BASE(remember_array) + ADDR_OFFSET);
	// 	break;
	// case J_TYPE:
	// 	current_Thermocouple->Bias = SPI_Load_Word(CARLIBRATION_BASE(remember_array) + 2 * ADDR_OFFSET);
	// 	break;

	// default:
	// 	break;
	// }

#endif
}

static void Temp_updata_realtime()
{
	weld_controller->realtime_temp = temp_convert(current_Thermocouple);
	command_set_comp_val("temp33", "val", weld_controller->realtime_temp);

#if TEMP_ADJUST == 1
	command_set_comp_val("temp11", "val", temp_convert(&Thermocouple_Lists[1])); // k
	command_set_comp_val("temp22", "val", temp_convert(&Thermocouple_Lists[2])); // j
#endif
}

static void Thermocouple_check(void)
{

	OS_ERR err;

	static uint8_t IO_val;
	switch (current_Thermocouple->type)
	{

	case J_TYPE:
		IO_val = 0;
		GPIO_SetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
		OSTimeDlyHMSM(0, 0, 0, KEJ_CHECK_DUTY, OS_OPT_TIME_PERIODIC, &err);
		IO_val = GPIO_ReadInputDataBit(CHECK_GPIO_J, CHECKIN_PIN_J);
		if (IO_val == 0)
		{
			if (err_ctrl->sensor_err_cnt++ > SENSOR_ERR_THRESHOLD)
			{
				err_ctrl->sensor_err_cnt = 0;
				Page_to(page_param, ALARM_PAGE);
				err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_ALL, &err);
			}
		}
		GPIO_ResetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
		break;

	case K_TYPE:
		IO_val = 0;
		GPIO_SetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);
		OSTimeDlyHMSM(0, 0, 0, KEJ_CHECK_DUTY, OS_OPT_TIME_PERIODIC, &err);
		IO_val = GPIO_ReadInputDataBit(CHECK_GPIO_K, CHECKIN_PIN_K);
		if (IO_val == 0)
		{
			if (err_ctrl->sensor_err_cnt++ > SENSOR_ERR_THRESHOLD)
			{
				err_ctrl->sensor_err_cnt = 0;
				Page_to(page_param, ALARM_PAGE);
				err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_ALL, &err);
			}
		}
		GPIO_ResetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);
		break;

	case E_TYPE:
		IO_val = 0;
		GPIO_SetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
		OSTimeDlyHMSM(0, 0, 0, KEJ_CHECK_DUTY, OS_OPT_TIME_PERIODIC, &err);
		uint8_t IO_val = GPIO_ReadInputDataBit(CHECK_GPIO_E, CHECKIN_PIN_E);
		if (IO_val == 0)
		{
			/*use old board , close alarm*/

			//			if (err_ctrl->sensor_err_cnt++ > SENSOR_ERR_THRESHOLD)
			//			{
			//				err_ctrl->sensor_err_cnt = 0;
			//				Page_to(page_param, ALARM_PAGE);
			//				err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
			//				OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_ALL, &err);
			//			}
		}
		GPIO_ResetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
		break;
	}
}

static void voltage_check(void)
{
	uint16_t ADC_Voltage = 0;
	OS_ERR err;
	ADC_Voltage = ADC_Value_avg(ADC_Channel_6);
	//  voltage too high
	if (ADC_Voltage > OVER_VOLTAGE)
	{
		err_get_type(err_ctrl, VOLTAGE_TOO_HIGH)->state = true;

		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
	// voltage too low
	if (ADC_Voltage < DOWN_VOLTAGE)
	{

		err_get_type(err_ctrl, VOLTAGE_TOO_LOW)->state = true;

		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
}

static void Overload_check(void)
{
	OS_ERR err;

#if VOLTAGE_CHECK
	voltage_check();
#endif

	if (GPIO_ReadInputDataBit(CURRENT_OVERLOAD_GPIO, CURRENT_PIN) == 0)
	{
		OSTimeDlyHMSM(0, 0, 0, 15, OS_OPT_TIME_PERIODIC, &err);
		if (GPIO_ReadInputDataBit(CURRENT_OVERLOAD_GPIO, CURRENT_PIN) == 0)
		{
			err_get_type(err_ctrl, CURRENT_OUT_OT_CTRL)->state = true;

			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}
	}

#if 0
	if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RECTIFICATION_PIN) == 0)
	{
		OSTimeDlyHMSM(0, 0, 0, 15, OS_OPT_TIME_PERIODIC, &err);
		if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RECTIFICATION_PIN) == 0)
		{
			err_get_type(err_ctrl, MCU_OVER_HEAT)->state = true;
	
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}
	}

	if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RADIATOR_PIN) == 0)
	{
		OSTimeDlyHMSM(0, 0, 0, 15, OS_OPT_TIME_PERIODIC, &err);
		if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RADIATOR_PIN) == 0)
		{
			err_get_type(err_ctrl, MCU_OVER_HEAT)->state = true;
	
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}
	}

#endif

	/*temp overload protect 1*/
	weld_controller->realtime_temp = temp_convert(current_Thermocouple);
	if (weld_controller->realtime_temp > USER_SET_MAX_TEMP)
	{
		err_get_type(err_ctrl, TEMP_UP)->state = true;
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}

	/*temp overload protect 2*/
	switch (current_Thermocouple->type)
	{
	case E_TYPE:
		if (ADC_Value_avg(THERMOCOUPLE_CHANNEL_E) > ADC_SAMPLE_LIMIT)
		{
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}

		break;

	case K_TYPE:
		if (ADC_Value_avg(THERMOCOUPLE_CHANNEL_J) > ADC_SAMPLE_LIMIT)
		{
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}

		break;

	case J_TYPE:
		if (ADC_Value_avg(THERMOCOUPLE_CHANNEL_K) > ADC_SAMPLE_LIMIT)
		{
			err_get_type(err_ctrl, TEMP_UP)->state = true;
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}

		break;
	}
}

/**
 * @description: main task
 * @param {void} *p_arg
 * @return {*}
 */
void main_task(void *p_arg)
{

	OS_ERR err;
	Power_on_check();
	while (1)
	{

#if OVER_LOAD_CHECK
		Overload_check();
#endif
		Thermocouple_check();
		welding_process();

		OSTimeDlyHMSM(0, 0, 0, 30, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------touch screan Thread--------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/
/*----------------------Touchscreen communication thread internal API-------------------*/
/*--------------------------------------------------------------------------------------*/
/**
 * @description: Thermocouple adjust
 * @return {*}
 */
static void Thermocouple_err_eliminate()
{

	uint32_t sum = 0;
	uint16_t ADC_channel_init_val = 0;
	float adc_channel_data[SAMPLE_LEN] = {0};
	float adc_channel_fliter_buf[SAMPLE_LEN] = {0};

	/*select channel according to different type of thermocouple*/
	switch (current_Thermocouple->type)
	{
	case E_TYPE:
		for (uint16_t i = 0; i < SAMPLE_LEN; i++)
		{
			adc_channel_data[i] = ADC_Value_avg(THERMOCOUPLE_CHANNEL_E);
			delay_ms(1);
		}

		break;

	case K_TYPE:
		for (uint16_t i = 0; i < SAMPLE_LEN; i++)
		{
			adc_channel_data[i] = ADC_Value_avg(THERMOCOUPLE_CHANNEL_K);
			delay_ms(1);
		}
		break;

	case J_TYPE:
		for (uint16_t i = 0; i < SAMPLE_LEN; i++)
		{
			adc_channel_data[i] = ADC_Value_avg(THERMOCOUPLE_CHANNEL_J);
			delay_ms(1);
		}
		break;
	}

	/*filter*/
	low_pass_Filter((float *)adc_channel_data,
					SAMPLE_LEN,
					(float *)adc_channel_fliter_buf,
					1000,
					1000);

	/*calculate average*/
	sum = 0;
	for (uint16_t i = 0; i < SAMPLE_LEN; i++)
	{
		sum += adc_channel_fliter_buf[i];
	}
	ADC_channel_init_val = sum / SAMPLE_LEN;

	if (ADC_channel_init_val > ADC_BIAS_MAX)
	{
		/*voltage too high*/
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		OS_ERR err;
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
	else
	{

		uint16_t room_temp_voltage = 0;

		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (current_Thermocouple->type == Thermocouple_Lists[i].type)
			{
				// Theoretical voltage value at room temperature
				room_temp_voltage = (ROOM_TEMP - Thermocouple_Lists[i].intercept) / Thermocouple_Lists[i].slope;
				// update calibration parameters
				Thermocouple_Lists[i].Bias = ADC_channel_init_val - room_temp_voltage;
				// update current Thermocouple
				current_Thermocouple = &Thermocouple_Lists[i];
			}
		}

		/*adjust finished start animation*/
		command_set_comp_val("tm0", "en", 1);

		/*Store thermocouple calibration parameters to EEPROM*/
		uint8_t group = get_comp(param_page_list, "GP")->val;
		switch (current_Thermocouple->type)
		{
		case K_TYPE:
			SPI_Save_Word(current_Thermocouple->Bias, CARLIBRATION_BASE(group));
			break;
		case E_TYPE:
			SPI_Save_Word(current_Thermocouple->Bias, CARLIBRATION_BASE(group) + ADDR_OFFSET);
			break;
		case J_TYPE:
			SPI_Save_Word(current_Thermocouple->Bias, CARLIBRATION_BASE(group) + 2 * ADDR_OFFSET);
			break;
		}
	}
}

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
		/*读取焊接计数值*/
		command_get_comp_val(page_list, "count", "val");
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

		/*读取焊接计数值*/
		command_get_comp_val(page_list, "count", "val");
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
	const char *pid_param_name_list[] = {
		"kp",
		"ki",
		"kd"};
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < sizeof(pid_param_name_list) / sizeof(char *); i++)
	{
		if (command_get_comp_val(wave_page_list, pid_param_name_list[i], "val") == true)
			cnt++;
	}
	if (cnt == sizeof(pid_param_name_list) / sizeof(char *))
	{
		pid_raw_param[0] = get_comp(wave_page_list, "kp")->val;
		pid_raw_param[1] = get_comp(wave_page_list, "ki")->val;
		pid_raw_param[2] = get_comp(wave_page_list, "kd")->val;
		return true;
	}
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

		/*3、焊接接收后显示三段温度*/
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
		command_set_comp_val("wave_page.kp", "aph", 127);
		command_set_comp_val("wave_page.ki", "aph", 127);
		command_set_comp_val("wave_page.kd", "aph", 127);
		uint16_t pid_param[3] = {0};
		if (pid_param_get(pid_param) == true)
		{
			weld_controller->pid_ctrl->kp = pid_param[0] / 100.0;
			weld_controller->pid_ctrl->ki = pid_param[1] / 1000.0;
			weld_controller->pid_ctrl->kd = pid_param[2] / 100.0;
		}

#endif
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
		total_tick_len = 5 * delta_tick;					 // 横坐标总长度
		win_width = WIN_WIDTH * total_time / total_tick_len; // 焊接周期绘图区域占全屏比例

		if (total_time == win_width)
			temp_draw_ctrl->delta_tick = 1;
		else
			temp_draw_ctrl->delta_tick = (total_time / win_width) + 1;

		/*坐标发送到触摸屏*/
		for (uint8_t i = 0; i < sizeof(tick_name) / sizeof(char *); i++)
			command_set_comp_val(tick_name[i], "val", (1 + i) * delta_tick);
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
		/*3、设定热电偶类型*/
		switch (current_Thermocouple->type)
		{
		case K_TYPE:
			command_set_comp_val("KEJ", "val", 0);
			break;

		case E_TYPE:
			command_set_comp_val("KEJ", "val", 1);
			break;

		case J_TYPE:
			command_set_comp_val("KEJ", "val", 2);
			break;
		}

		/*4、订阅热电偶校准信号*/
		OSSemPend(&SENSOR_UPDATE_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{

			//			if (weld_controller->realtime_temp < 2 * ROOM_TEMP && weld_controller->realtime_temp > 0.5 * ROOM_TEMP) // 在室温下才允许校准（10-40°C范围内才允许校准）
			//				Thermocouple_err_eliminate();
			//			else // 焊头尚未冷却，警报
			//				command_set_comp_val("warning", "aph", 127);
			Thermocouple_err_eliminate();
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

/**
 * @description: sync data from screen to user data
 * @param {Page_ID} id
 * @return {*}
 */
static bool data_syn(Page_ID id)
{
	/*key state sync*/
	RDY_SCH = (page_param->key1 == SCH) ? 1 : 0;
	ION_IOF = (page_param->key2 == IOFF) ? 1 : 0;
	SGW_CTW = (page_param->key3 == CTW) ? 1 : 0;

	/*weld_controller sync*/
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
		/*Synchronization Count Values (User Subject to Change)*/
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
		/*Synchronization Count Values (User Subject to Change)*/
		comp = get_comp(param_page_list, "count");
		if (comp != NULL)
			weld_controller->weld_count = comp->val;

		break;
	}

	case UART_PAGE:
	{
		Component *comp = NULL;
		/*update machine id*/
		comp = get_comp(setting_page_list, "adress");
		if (comp != NULL)
			ID_OF_MAS = comp->val;

		/*update baudrate*/
		comp = get_comp(setting_page_list, "baudrate");
		if (comp != NULL)
		{
			uint8_t index = get_comp(setting_page_list, "baudrate")->val;
			usart3_set_bound(baud_list[index - 1]);
		}
	}

	default:
		break;
	}

	return true;
}

/**
 * @description: Touchscreen communication threads
 * @param {void} *p_arg
 * @return {*}
 */
void read_task(void *p_arg)
{

	OS_ERR err;
	while (1)
	{
		/*query page id*/
		if (Page_id_get() == true)
		{
			page_process(page_param->id);
			data_syn(page_param->id);
		}
		/*display Real-time temperature*/
		Temp_updata_realtime();
		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_PERIODIC, &err);
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------The upper computer communication thread------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

/**
 * @description: Data synchronization
 * @return {*}
 */
static void Hsot_Computer_Data_Sync()
{
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
 * @description: The upper computer communication thread
 * @param {void} *p_arg
 * @return {*}
 */
void computer_read_task(void *p_arg)
{
	/*
	Data receive from host computer:
	ModBus_time||ModBus_temp||ModBus_alarm_temp||Host_GP||Host_gain1_raw||Host_gain1||Host_gain2_raw||Host_gain2
	User-maintained data:
	weld_controller->weld_time||weld_controller->weld_temp||weld_controller->alarm_temp
	*/
	OS_ERR err;
	usart3_init(115200);
	while (1)
	{

		/*Subscribe to data update signals*/
		OSSemPend(&COMPUTER_DATA_SYN_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{

			/*
			When the Host_action is not 0, it is to process the data of the host computer and refresh the data to the touch screen
			*/
			if (Host_action == 1)
			{
				/*data update*/
				remember_array = Host_GP;
				/*screan refresh*/
				for (int i = 0; i < 2; i++)
				{
					/*The GP value of the previous time is sent to the slave*/
					command_set_comp_val("GP", "val", remember_array);
					Load_param(weld_controller, remember_array);
					Load_param_alarm(weld_controller, remember_array);
				}
				SPI_Save_Word(0, 40 * (remember_array + 1) + 4);
				Host_action = 0;
			}
			/*
			Host_action is 2 - 17, which means that the host computer modifies a single data,
			and synchronously modifies the parameter array of the control board and the touch screen data
			*/
			else if ((Host_action >= 2) && (Host_action <= 17))
			{
				/*data update*/
				switch (Host_action)
				{

				/*weld time*/
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
				/*weld temp*/
				case 7:
					weld_controller->weld_temp[0] = ModBus_temp[0];
					break;
				case 8:
					weld_controller->weld_temp[1] = ModBus_temp[1];
					break;
				case 9:
					weld_controller->weld_temp[2] = ModBus_temp[2];
					break;
				/*alarm temp*/
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
				/*gain*/
				case 16:
					weld_controller->temp_gain1 = Host_gain1;
					break;
				case 17:
					weld_controller->temp_gain2 = Host_gain2;
					break;
				}

				/*save param*/
				/*time1-time5*/
				for (uint8_t i = 0; i < TIME_NUM; i++)
				{
					SPI_Save_Word(weld_controller->weld_time[i], TIME_BASE(remember_array) + ADDR_OFFSET * i);
				}
				/*temp1-temp3*/
				for (uint8_t i = 0; i < TEMP_NUM; i++)
				{
					SPI_Save_Word(weld_controller->weld_temp[i], TEMP_BASE(remember_array) + ADDR_OFFSET * i);
				}
				/*alarm1-alarm6*/
				for (uint8_t i = 0; i < ALARM_NUM; i++)
				{
					SPI_Save_Word(weld_controller->alarm_temp[i], ALARM_BASE(remember_array) + ADDR_OFFSET * i);
				}

				/*gain1-gain2*/
				SPI_Save_Word(weld_controller->temp_gain1 * 100, GAIN_BASE(remember_array));
				SPI_Save_Word(weld_controller->temp_gain2 * 100, GAIN_BASE(remember_array) + ADDR_OFFSET);

				/*screan refresh*/
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

				command_set_comp_val("temp_page.GAIN1", "val", weld_controller->temp_gain1 * 100);
				command_set_comp_val("temp_page.GAIN2", "val", weld_controller->temp_gain2 * 100);

				/*reset host action*/
				Host_action = 0;
			}
			/*
			Host_action = 20 means that the whole page of data is changed,
			and the modified data is synchronized to the parameter array and touch screen.
			*/
			else if (Host_action == 20)
			{
				/*data update*/
				remember_array = Host_GP;

				for (uint8_t i = 0; i < sizeof(weld_controller->weld_time) / sizeof(uint16_t); i++)
				{
					weld_controller->weld_time[i] = ModBus_time[i];
				}

				for (uint8_t i = 0; i < sizeof(weld_controller->weld_temp) / sizeof(uint16_t); i++)
				{
					weld_controller->weld_temp[i] = ModBus_temp[i];
				}

				for (uint8_t i = 0; i < sizeof(weld_controller->alarm_temp) / sizeof(uint16_t); i++)
				{
					weld_controller->alarm_temp[i] = ModBus_alarm_temp[i];
				}

				/*save user data to EEPROM*/
				/*time1-time5*/
				for (uint8_t i = 0; i < TIME_NUM; i++)
				{
					SPI_Save_Word(weld_controller->weld_time[i], TIME_BASE(remember_array) + ADDR_OFFSET * i);
				}
				/*temp1-temp3*/
				for (uint8_t i = 0; i < TEMP_NUM; i++)
				{
					SPI_Save_Word(weld_controller->weld_temp[i], TEMP_BASE(remember_array) + ADDR_OFFSET * i);
				}
				/*alarm1-alarm6*/
				for (uint8_t i = 0; i < ALARM_NUM; i++)
				{
					SPI_Save_Word(weld_controller->alarm_temp[i], ALARM_BASE(remember_array) + ADDR_OFFSET * i);
				}
				/*gain1-gain2*/
				SPI_Save_Word(weld_controller->temp_gain1 * 100, GAIN_BASE(remember_array));
				SPI_Save_Word(weld_controller->temp_gain2 * 100, GAIN_BASE(remember_array) + ADDR_OFFSET);
				Host_action = 0;
			}

			Hsot_Computer_Data_Sync();
		}

		OSTimeDlyHMSM(0, 0, 0, 25, OS_OPT_TIME_PERIODIC, &err);
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------USB read-write thread---------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

void usb_task(void *p_arg)
{

	OS_ERR err;
	USBH_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
			  USB_OTG_FS_CORE_ID,
#elif defined USE_USB_OTG_HS
			  USB_OTG_HS_CORE_ID,
#endif
			  &USB_Host,
			  &USBH_MSC_cb,
			  &USR_USBH_MSC_cb);

	USB_OTG_BSP_mDelay(500);

	while (1)
	{

		/* Host Task handler */
		USBH_Process(&USB_OTG_Core, &USB_Host);
		OSTimeDlyHMSM(0, 0, 0, 40, OS_OPT_TIME_PERIODIC, &err);
	}
}
