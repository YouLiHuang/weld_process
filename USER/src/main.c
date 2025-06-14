/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-03-19 08:22:00
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-14 12:48:27
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

/*user*/
#include "user_config.h"
#include "includes.h"
#include "sys.h"
#include "touch_screen_app.h"

/*bsp*/
#include "delay.h"
#include "usart.h"
#include "welding_process.h"
#include "spi.h"
#include "timer.h"
#include "adc.h"
#include "protect.h"
#include "pwm.h"
#include "touchscreen.h"
#include "pid.h"
#include "filter.h"
#include "thermocouple.h"
#include "io_ctrl.h"

/*FreeModbus includes*/
#include "mb.h"
#include "mbport.h"
#include "mbrtu.h"
#include "port_bsp.h"
#include "modbus_app.h"

/*USB includes*/
#include "usbh_app.h"
#include "log.h"

/* Private macro -------------------------------------------------------------*/
#define START_TASK_PRIO 3
#define START_STK_SIZE 128
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

#define ERROR_TASK_PRIO 4
#define ERROR_STK_SIZE 512
OS_TCB ErrorTaskTCB;
CPU_STK ERROR_TASK_STK[ERROR_STK_SIZE];
void error_task(void *p_arg);

#define MAIN_TASK_PRIO 5
#define MAIN_STK_SIZE 2048
OS_TCB Main_TaskTCB;
CPU_STK MAIN_TASK_STK[MAIN_STK_SIZE];
void main_task(void *p_arg);

#define READ_TASK_PRIO 6
#define READ_STK_SIZE 4096
OS_TCB READ_TaskTCB;
CPU_STK READ_TASK_STK[READ_STK_SIZE];

void read_task(void *p_arg);

#define COMPUTER_TASK_PRIO 7
#define COMPUTER_STK_SIZE 1024
OS_TCB COMPUTER_TaskTCB;
CPU_STK COMPUTER_TASK_STK[COMPUTER_STK_SIZE];
void computer_read_task(void *p_arg);

#define USB_TASK_PRIO 7
#define USB_STK_SIZE 1024
OS_TCB USB_TaskTCB;
CPU_STK USB_TASK_STK[USB_STK_SIZE];
void usb_task(void *p_arg);

/* Private function prototypes -----------------------------------------------*/

/*Error handling callbacks ---------------------------------------------------*/
static bool Temp_up_err_callback(uint8_t index);
static bool Temp_down_err_callback(uint8_t index);
static bool Current_out_of_ctrl_callback(uint8_t index);
static bool Thermocouple_recheck_callback(uint8_t index);
static bool Transformer_over_heat_callback(uint8_t index);
static bool Radiator_over_heat_callback(uint8_t index);
/*Error reset callback -------------------------------------------------------*/
static bool Thermocouple_reset_callback(uint8_t index);
static bool Current_out_of_ctrl_reset_callback(uint8_t index);
static bool Temp_up_reset_callback(uint8_t index);
static bool Temp_down_reset_callback(uint8_t index);
static bool Transformer_reset_callback(uint8_t index);
static bool Radiator_reset_callback(uint8_t index);
/*main task functions --------------------------------------------------------*/
static void Power_on_check(void);
static bool Thermocouple_check(void);
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
	{K_TYPE, 0.17, 0, 0},
	{J_TYPE, 0.17, 0, 0},
};

const error_match_list match_list[] = {
	{CURRENT_OUT_OT_CTRL, "f1", Current_out_of_ctrl_callback, Current_out_of_ctrl_reset_callback},
	{TEMP_UP, "f2", Temp_up_err_callback, Temp_up_reset_callback},
	{TEMP_DOWN, "f3", Temp_down_err_callback, Temp_down_reset_callback},
	{VOLTAGE_TOO_HIGH, "f4", NULL, NULL},
	{VOLTAGE_TOO_LOW, "f5", NULL, NULL},
	{RADIATOR, "f6", Radiator_over_heat_callback, Radiator_reset_callback},
	{TRANSFORMER_OVER_HEAT, "f7", Transformer_over_heat_callback, Transformer_reset_callback},
	{SENSOR_ERROR, "f8", Thermocouple_recheck_callback, Thermocouple_reset_callback},
	{MCU_OVER_HEAT, "f9", NULL, NULL},

};



/* Private variables ---------------------------------------------------------*/

/*UART3 Resource Protection: Mutex (Temporarily Unused)*/
OS_MUTEX ModBus_Mux;

/*UART4 Resource Protection: Mutex (Temporarily Unused)*/
OS_MUTEX PLOT_Mux;

/*Thread Synchronization: Semaphore*/
// START SEM
OS_SEM WELD_START_SEM;
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
// err sem
OS_SEM ERROR_HANDLE_SEM;
// data save sem
OS_SEM DATA_SAVE_SEM;
// plot sem
OS_SEM PLOT_SEM;

// error controller
Error_ctrl *err_ctrl = NULL;
// Drawing controllers
Temp_draw_ctrl *temp_draw_ctrl = NULL;
// Welding real-time controller
weld_ctrl *weld_controller = NULL;
pid_feedforword_ctrl *pid_ctrl = NULL;
#if PID_DEBUG
pid_feedforword_ctrl *pid_ctrl_debug = NULL;
#endif
// Current thermocouple
Thermocouple *current_Thermocouple = NULL;
// current date
Date current_date;

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
	"count",
	"rtc0",
	"rtc1",
	"rtc2",
	"rtc3",
	"rtc4",
};

static char *setting_page_name_list[] = {
	"adress",
	"baudrate",
	"sensortype"};

static char *wave_page_name_list[] = {
	"kp",
	"ki",
	"kd"};

uint8_t ID_OF_DEVICE = 0;

extern USBH_HOST USB_Host __ALIGN_END;
extern USB_OTG_CORE_HANDLE USB_OTG_Core __ALIGN_END;
extern START_TYPE start_type;
// Temperature preservation buffer
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN];
// MODBUS BUFFER
extern uint16_t usRegInputBuf[REG_INPUT_NREGS];
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8];
extern uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];

/*Touch Screen list*/
// Record the ID of the current screen and the status of the three buttons
extern Page_Param *page_param;
// A list of components on the parameter setting screen
extern Component_Queue *param_page_list;
// A list of components for the temperature limit interface
extern Component_Queue *temp_page_list;
// A list of communication interface components
extern Component_Queue *setting_page_list;
// A list of components on the Waveform page
extern Component_Queue *wave_page_list;

int main(void)
{

	/*------------------------------------------------------User layer data objects-----------------------------------------------------------*/
	/*pid controller*/
	pid_ctrl = new_pid_forword_ctrl(0, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
#if PID_DEBUG
	pid_ctrl_debug = new_pid_forword_ctrl(0, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
#endif
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
	component_insert(temp_page_list, newComponet("switch", 0));

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
	err_clear(err_ctrl);

	/*绘图控制器初始化*/
	temp_draw_ctrl = new_temp_draw_ctrl(realtime_temp_buf, 500, 2000, 500);

	/*------------------------------------------------------Hardware layer data initialization-----------------------------------------------------------*/
	/*Peripheral initialization*/
	delay_init(168);								// clock init
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // Interrupt Group Configuration
	TIM1_PWM_Init();								// tim1 PWM(change RCC must init first)
	TIM4_PWM_Init();								// tim4 PWM(change RCC must init first)
	TIM3_INIT();									// PID TIMER
	TIM5_INIT();									// COUNT TIMER
	uart_init(115200);								// Touch screen communication interface initialization
#if MODBUSSLAVE_ENABLE
	eMBInit(MB_RTU, 1, 3, 115200, MB_PAR_NONE, 1);
	eMBEnable();
#else
	modbus_serial_init(115200);
#endif
	log_bsp_init(115200);

	/*IO Config*/
	START_IO_INIT(); // start signals
	INPUT_IO_INIT(); // input IO init
	OUT_Init();		 // output pin init
	Check_IO_init(); // Thermocouple detection io initialization

	/*user device init*/
	SPI1_Init();	// spi init
	ADC_DMA_INIT(); // ADC init
	Touchscreen_init();
	Load_data_from_mem();

	/*Hardware test*/
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

	// Create a mutex - serial port resource access (temporarily unused)
	OSMutexCreate(&ModBus_Mux, "MODBUS_Mutex", &err);

	// Create a mutex - serial port resource access (temporarily unused)
	OSMutexCreate(&PLOT_Mux, "plot mux", &err);

	// WELD_START_SEM
	OSSemCreate(&WELD_START_SEM, "weld start", 0, &err);
	/*--------------------------------------------SEM use for UI--------------------------------------------*/
	// Page update signals
	OSSemCreate(&PAGE_UPDATE_SEM, "page update", 0, &err);
	// semaphore specifically used to obtain the value of a component's attributes
	OSSemCreate(&COMP_VAL_GET_SEM, "comp val get", 0, &err);
	OSSemCreate(&COMP_STR_GET_SEM, "comp str get", 0, &err);
	// Thermocouple calibration signal
	OSSemCreate(&SENSOR_UPDATE_SEM, "SENSOR_UPDATE_SEM", 0, &err);
	// plot sem
	OSSemCreate(&PLOT_SEM, "plot sem", 0, &err);

	/*--------------------------------------------other SEM--------------------------------------------------*/
	//  err reset sem
	OSSemCreate(&ALARM_RESET_SEM, "alarm reset", 0, &err);
	// err sem
	OSSemCreate(&ERROR_HANDLE_SEM, "err sem", 0, &err);
	// data save sem
	OSSemCreate(&DATA_SAVE_SEM, "data save", 0, &err);

	// err task
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

	// main task
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

	// UI task
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
				 (OS_TICK)50,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// USB task
	OSTaskCreate((OS_TCB *)&USB_TaskTCB,
				 (CPU_CHAR *)"usb task",
				 (OS_TASK_PTR)usb_task,
				 (void *)0,
				 (OS_PRIO)USB_TASK_PRIO,
				 (CPU_STK *)&USB_TASK_STK[0],
				 (CPU_STK_SIZE)USB_STK_SIZE / 10,
				 (CPU_STK_SIZE)USB_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)20,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	// modbus task
	OSTaskCreate((OS_TCB *)&COMPUTER_TaskTCB,
				 (CPU_CHAR *)"computer read task",
				 (OS_TASK_PTR)computer_read_task,
				 (void *)0,
				 (OS_PRIO)COMPUTER_TASK_PRIO,
				 (CPU_STK *)&COMPUTER_TASK_STK[0],
				 (CPU_STK_SIZE)COMPUTER_STK_SIZE / 10,
				 (CPU_STK_SIZE)COMPUTER_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)40,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR *)&err);

	OS_CRITICAL_EXIT();

	// delete start task
	OSTaskDel((OS_TCB *)0, &err);
}

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

static bool Transformer_over_heat_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
	return true;
}

static bool Radiator_over_heat_callback(uint8_t index)
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

	if (Thermocouple_check() == true)
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // clear error state
		ret = true;
	}
	else
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
		err_ctrl->err_list[index]->state = true; // active error state
		ret = true;
	}

	return ret;
}
static bool Current_out_of_ctrl_reset_callback(uint8_t index)
{

	bool ret = false;
	if (GPIO_ReadInputDataBit(CURRENT_OVERLOAD_GPIO, CURRENT_PIN) != 0)
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // clear error state
		ret = true;
	}
	else
	{
		err_ctrl->err_list[index]->state = true; // active error state
		ret = false;
	}

	return ret;
}
static bool Temp_up_reset_callback(uint8_t index)
{

	bool ret = false;

	weld_controller->realtime_temp = temp_convert(current_Thermocouple);
	if (weld_controller->realtime_temp < weld_controller->alarm_temp[0] &&
		weld_controller->realtime_temp < weld_controller->alarm_temp[1] &&
		weld_controller->realtime_temp < weld_controller->alarm_temp[2])
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // clear error state
		ret = true;
	}
	else
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
		err_ctrl->err_list[index]->state = true; // active error state
		ret = false;
	}

	return ret;
}
static bool Temp_down_reset_callback(uint8_t index)
{
	command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
	err_ctrl->err_list[index]->state = false; // clear error state

	return true;
}
static bool Transformer_reset_callback(uint8_t index)
{
	bool ret = false;
	if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RECTIFICATION_PIN) != 1)
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // clear error state
		ret = true;
	}
	else
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
		err_ctrl->err_list[index]->state = true; // active error state
	}

	return ret;
}

static bool Radiator_reset_callback(uint8_t index)
{
	bool ret;
	if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RADIATOR_PIN) != 1)
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_OFF);
		err_ctrl->err_list[index]->state = false; // clear error state
		ret = true;
	}
	else
	{
		command_set_comp_val(err_ctrl->err_list[index]->pic_name, "aph", SHOW_ON);
		err_ctrl->err_list[index]->state = true; // active error state
		ret = true;
	}
	return ret;
}

/*--------------------------------------------------------------------------------------*/
/*-----------------------------------main task  API-------------------------------------*/
/*--------------------------------------------------------------------------------------*/

static void Power_on_check(void)
{

	OS_ERR err;
	uint16_t start_temp = 0;
	uint16_t end_temp = 0;

#if POWER_ON_CHECK == 1

	/*check if senor is already*/
	GPIO_SetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
	GPIO_SetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
	GPIO_SetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);

	OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_PERIODIC, &err);

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

	GPIO_ResetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
	GPIO_ResetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
	GPIO_ResetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);

	/*no Thermocouple detect*/
	if (current_Thermocouple == NULL)
	{
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}

#endif

	/*reserve check*/
	start_temp = temp_convert(current_Thermocouple);

	/*PWM ON*/
	TIM_SetCompare1(TIM1, PD_MAX / 4);
	TIM_SetCompare1(TIM4, PD_MAX / 4);
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

	/*heat 100ms*/
	OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err);

	/*PWM OFF*/
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare1(TIM4, 0);
	TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
	TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
	TIM_Cmd(TIM4, DISABLE);
	TIM_Cmd(TIM1, DISABLE);

	end_temp = temp_convert(current_Thermocouple);

	if (end_temp < start_temp)
	{
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
}

/**
 * @description: sensor check
 * @return {*}
 */
static bool Thermocouple_check(void)
{

	OS_ERR err;
	bool check_state = false;

	GPIO_SetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
	GPIO_SetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
	GPIO_SetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);

	OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_PERIODIC, &err);

	if (GPIO_ReadInputDataBit(CHECK_GPIO_E, CHECKIN_PIN_E) != 0)
	{
		check_state = true;
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (Thermocouple_Lists[i].type == E_TYPE)
				current_Thermocouple = &Thermocouple_Lists[i];
		}
	}

	if (GPIO_ReadInputDataBit(CHECK_GPIO_K, CHECKIN_PIN_K) != 0)
	{
		check_state = true;
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (Thermocouple_Lists[i].type == K_TYPE)
				current_Thermocouple = &Thermocouple_Lists[i];
		}
	}

	if (GPIO_ReadInputDataBit(CHECK_GPIO_J, CHECKIN_PIN_J) != 0)
	{
		check_state = true;
		for (uint8_t i = 0; i < sizeof(Thermocouple_Lists) / sizeof(Thermocouple); i++)
		{
			if (Thermocouple_Lists[i].type == J_TYPE)
				current_Thermocouple = &Thermocouple_Lists[i];
		}
	}

	GPIO_ResetBits(CHECK_GPIO_E, CHECKOUT_PIN_E);
	GPIO_ResetBits(CHECK_GPIO_J, CHECKOUT_PIN_J);
	GPIO_ResetBits(CHECK_GPIO_K, CHECKOUT_PIN_K);

	/*no Thermocouple detect*/
	if (check_state == false)
	{
		err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_ALL, &err);
	}

	return check_state;
}

/**
 * @description: voltage check
 * @return {*}
 */
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

/**
 * @description: over load check
 * @return {*}
 */
static void Overload_check(void)
{
	OS_ERR err;

#if VOLTAGE_CHECK
	voltage_check();
#endif

	/*over current*/
	if (GPIO_ReadInputDataBit(CURRENT_OVERLOAD_GPIO, CURRENT_PIN) == 0)
	{
		OSTimeDlyHMSM(0, 0, 0, 15, OS_OPT_TIME_PERIODIC, &err);
		if (GPIO_ReadInputDataBit(CURRENT_OVERLOAD_GPIO, CURRENT_PIN) == 0)
		{
			err_get_type(err_ctrl, CURRENT_OUT_OT_CTRL)->state = true;
		}
	}
	/*transfoemer*/
	if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RECTIFICATION_PIN) == 1)
	{
		OSTimeDlyHMSM(0, 0, 0, 15, OS_OPT_TIME_PERIODIC, &err);
		if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RECTIFICATION_PIN) == 1)
		{
			err_get_type(err_ctrl, TRANSFORMER_OVER_HEAT)->state = true;
		}
	}
	/*radiator*/
	if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RADIATOR_PIN) == 1)
	{
		OSTimeDlyHMSM(0, 0, 0, 15, OS_OPT_TIME_PERIODIC, &err);
		if (GPIO_ReadInputDataBit(TEMP_OVERLOAD_GPIO, RADIATOR_PIN) == 1)
		{
			err_get_type(err_ctrl, RADIATOR)->state = true;
		}
	}

	/*temp overload protect 1*/
	weld_controller->realtime_temp = temp_convert(current_Thermocouple);
	if (weld_controller->realtime_temp > USER_SET_MAX_TEMP)
	{
		err_get_type(err_ctrl, TEMP_UP)->state = true;
	}

	/*sensor maybe disconnect*/
	Thermocouple_check();
	/*temp overload protect 2*/
	switch (current_Thermocouple->type)
	{
	case E_TYPE:
		if (ADC_Value_avg(THERMOCOUPLE_CHANNEL_E) > ADC_SAMPLE_LIMIT)
		{
			err_get_type(err_ctrl, TEMP_UP)->state = true;
		}

		break;

	case K_TYPE:
		if (ADC_Value_avg(THERMOCOUPLE_CHANNEL_J) > ADC_SAMPLE_LIMIT)
		{
			err_get_type(err_ctrl, TEMP_UP)->state = true;
		}

		break;

	case J_TYPE:
		if (ADC_Value_avg(THERMOCOUPLE_CHANNEL_K) > ADC_SAMPLE_LIMIT)
		{
			err_get_type(err_ctrl, TEMP_UP)->state = true;
		}

		break;
	}

	if (err_occur(err_ctrl))
	{
		OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
	}
}




#if TEMP_ADJUST
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
#endif


/*-------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------err Thread---------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

void error_task(void *p_arg)
{
	OS_ERR err;
	bool err_wait = false;

	while (1)
	{
		OSSemPend(&ERROR_HANDLE_SEM, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		if (OS_ERR_NONE == err)
		{
			OSSemSet(&ERROR_HANDLE_SEM, 0, &err);

			/*PWM OFF / Timer Reset*/
			TIM_SetCompare1(TIM1, 0);
			TIM_SetCompare1(TIM4, 0);
			TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
			TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
			TIM_Cmd(TIM4, DISABLE);
			TIM_Cmd(TIM1, DISABLE);

			TIM_Cmd(TIM3, DISABLE);
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
			TIM_Cmd(TIM5, DISABLE);
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
			TIM3->CNT = 0;
			TIM5->CNT = 0;

			/*MODBUS update*/
			OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
			RLY_AIR0 = 0;
			ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_0);
			RLY_AIR1 = 0;
			ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_1);
			RLY_AIR2 = 0;
			ucRegCoilsBuf[0] &= ~(0x01 << COIL_ADDR_2);
			RLY_ERR = 1;
			ucRegCoilsBuf[0] |= 0x01 << COIL_ADDR_4;
			OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);

			/*wait hanlde*/
			err_wait = true;
		}

		/*err handle*/
		while (err_wait)
		{
			/*wait until user reset*/
			OSSemPend(&ALARM_RESET_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
			if (err == OS_ERR_NONE)
			{
				RLY_ERR = 0; // error signal reset
				for (uint8_t i = 0; i < err_ctrl->error_cnt; i++)
				{
					if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->reset_callback != NULL)
						err_ctrl->err_list[i]->reset_callback(i);
				}

				/*exit*/
				err_clear(err_ctrl);
				err_wait = false;
				OSSemSet(&ERROR_HANDLE_SEM, 0, &err);
				break;
			}

			Page_to(page_param, ALARM_PAGE);
			page_param->id = ALARM_PAGE;
			for (uint8_t i = 0; i < err_ctrl->error_cnt; i++)
			{
				if (true == err_ctrl->err_list[i]->state && err_ctrl->err_list[i]->error_callback != NULL)
					err_ctrl->err_list[i]->error_callback(i);
			}
			OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err);
		}

		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_PERIODIC, &err);
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------main Thread-------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

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

		/*when the main task is ideal , check the sensor and current*/
#if OVER_LOAD_CHECK
		Overload_check();
#endif

		if (err_occur(err_ctrl))
		{
			OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
		}

		/*trigger by exit irq(keys are pressed)*/
		OSSemPend(&WELD_START_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
		if (err == OS_ERR_NONE)
		{
			/*only check sensor before weld(avoid temp display error)*/
			switch (start_type)
			{

			case KEY0:
				start_type = START_IDEAL;
				if (Thermocouple_check() == true)
				{
					welding_process(KEY0);
				}
				break;
			case KEY1:
				start_type = START_IDEAL;
				if (Thermocouple_check() == true)
				{
					welding_process(KEY1);
				}
				break;
			}
		}
		Modbus_reg_sync();

		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_PERIODIC, &err); // 休眠
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------touch screan Thread--------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

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
			TSpage_process(page_param->id);
		}

		OSTimeDlyHMSM(0, 0, 0, 30, OS_OPT_TIME_PERIODIC, &err);
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------MODBUS Thread----------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------------*/

/**
 * @description: The upper computer communication thread
 * @param {void} *p_arg
 * @return {*}
 */
void computer_read_task(void *p_arg)
{

	OS_ERR err;

	while (1)
	{
#if MODBUSSLAVE_ENABLE
		(void)eMBPoll();

#else
		printf("> MODBUS NOT SUPPORT!\n");
#endif

		OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_PERIODIC, &err);
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------USB read-write Thread---------------------------------------------------*/
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
		OSTimeDlyHMSM(0, 0, 0, 30, OS_OPT_TIME_PERIODIC, &err);
	}
}
