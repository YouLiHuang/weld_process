/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-13 19:22:56
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-20 14:18:31
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
#include "user_config.h"
#include "spi.h"
#include "delay.h"
#include "io_ctrl.h"
#include "touchscreen.h"
#include "usart.h"
#include "welding_process.h"
#include "modbus_app.h"
#include "touch_screen_app.h"

/*---------------------------Variables-----------------------------*/
extern weld_ctrl *weld_controller;
extern Steady_state_coefficient steady_coefficient;
extern uint8_t ID_OF_DEVICE;
extern uint8_t cur_GP;

// input reg
extern uint16_t usRegInputBuf[REG_INPUT_NREGS];
// hold reg
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
// coil state
extern uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8];
// switch state
extern uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];
/*-----------------------------------------------------------------*/

/*Functions prototype----------------------------------------------*/
static void WREN(void);

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/*RCC Config*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/*GPIO INIT*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*CS PIN Config*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	CSN = 1;
	/*SPI1 PIN*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	/*SPI Config*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// 设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						// 设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					// 设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							// 串行同步时钟的空闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						// 串行同步时钟的第一个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							// NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // 定义波特率预分频的值:波特率预分频值为64
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					// 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							// CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);									// 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	SPI_Cmd(SPI2, ENABLE);
}

static void SPI_Sendbyte(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;

	SPI_I2S_SendData(SPI2, data);

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;

	SPI_I2S_ReceiveData(SPI2);
}

static uint8_t SPI_Readbyte(void)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;

	SPI_I2S_SendData(SPI2, 0xff);

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;

	return SPI_I2S_ReceiveData(SPI2);
}

static void WREN(void)
{
	CSN = 0;
	SPI_Sendbyte(0x06);
	CSN = 1;
}

void SPI_Save_Word(uint16_t data, uint16_t addr)
{
	WREN();
	CSN = 0;
	SPI_Sendbyte(0x02);
	SPI_Sendbyte(addr >> 8);
	SPI_Sendbyte(addr);
	SPI_Sendbyte(data >> 8);
	SPI_Sendbyte(data);
	CSN = 1;
	delay_ms(5);
}

uint16_t SPI_Load_Word(uint16_t addr)
{
	uint16_t tmp;
	CSN = 0;
	SPI_Sendbyte(0x03);
	SPI_Sendbyte(addr >> 8);
	SPI_Sendbyte(addr);
	tmp = SPI_Readbyte() << 8;
	tmp = tmp | SPI_Readbyte();
	CSN = 1;
	delay_ms(5);
	return tmp;
}

void Load_data_from_mem(void)
{
#if RESET_SPI_DATA
	/*数据初始化*/
	int time_init[] = {100, 500, 50, 2500, 500, 100};
	int temp_init[] = {250, 400, 150};
	int alarm_temp[] = {400, 100, 650, 100, 650, 100};

	for (int array_of_data = 1; array_of_data < 20; array_of_data++)
	{
		// GP
		SPI_Save_Word(0, array_of_data);
		// time1-time5
		for (uint8_t i = 0; i < sizeof(time_init) / sizeof(int); i++)
		{
			SPI_Save_Word(time_init[i], TIME_BASE(array_of_data) + ADDR_OFFSET * i);
		}

		for (uint8_t i = 0; i < sizeof(temp_init) / sizeof(int); i++)
		{
			SPI_Save_Word(temp_init[i], TEMP_BASE(array_of_data) + ADDR_OFFSET * i);
		}

		for (uint8_t i = 0; i < sizeof(alarm_temp) / sizeof(int); i++)
		{
			SPI_Save_Word(alarm_temp[i], ALARM_BASE(array_of_data) + ADDR_OFFSET * i);
		}

		// gain1 gain2
		SPI_Save_Word(72 + array_of_data, GAIN_BASE(array_of_data));
		SPI_Save_Word(60 + array_of_data, GAIN_BASE(array_of_data) + ADDR_OFFSET);
	}
#endif

	// 软起动
	delay_ms(2000);
	RLY_TRAN = 1;
	delay_ms(2000);
	RLY_TRAN = 0;
	// 从内存加载首个GP值
	cur_GP = 0;
	cur_GP = SPI_Load_Word(0);
	if (cur_GP >= 20)
		cur_GP = 0;

	/*首次从内存读取数据*/
	Load_param(weld_controller, cur_GP);
	Load_param_alarm(weld_controller, cur_GP);
	Load_Coefficient(cur_GP);

	/*modbus data init*/
	usRegHoldingBuf[0] = weld_controller->alarm_temp[0];
	usRegHoldingBuf[1] = weld_controller->alarm_temp[1];
	usRegHoldingBuf[2] = weld_controller->alarm_temp[2];
	usRegHoldingBuf[3] = weld_controller->alarm_temp[3];
	usRegHoldingBuf[4] = weld_controller->alarm_temp[4];
	usRegHoldingBuf[5] = weld_controller->alarm_temp[5];

	usRegHoldingBuf[6] = weld_controller->temp_gain1 * 100;
	usRegHoldingBuf[7] = weld_controller->temp_gain2 * 100;

	usRegHoldingBuf[8] = weld_controller->weld_temp[0];
	usRegHoldingBuf[9] = weld_controller->weld_temp[1];
	usRegHoldingBuf[10] = weld_controller->weld_temp[2];

	usRegHoldingBuf[11] = weld_controller->weld_time[0];
	usRegHoldingBuf[12] = weld_controller->weld_time[1];
	usRegHoldingBuf[13] = weld_controller->weld_time[2];
	usRegHoldingBuf[14] = weld_controller->weld_time[3];
	usRegHoldingBuf[15] = weld_controller->weld_time[4];

	usRegHoldingBuf[16] = weld_controller->weld_count;
	usRegHoldingBuf[17] = 0;

	/*首次加载参数后需要发送到触摸屏*/
	char *param_time_list[] = {
		"time1",
		"time2",
		"time3",
		"time4",
		"time5",
		"time6"

	};
	char *param_temp_list[] = {
		"temp1",
		"temp2",
		"temp3",
	};
	char *alarm_name_list[] = {
		"temp_page.alarm1",
		"temp_page.alarm2",
		"temp_page.alarm3",
		"temp_page.alarm4",
		"temp_page.alarm5",
		"temp_page.alarm6",
	};

	command_set_comp_val_raw("wave_page.kpf", "val", 0);
	command_set_comp_val_raw("wave_page.kp", "val", 0);
	command_set_comp_val_raw("wave_page.ki", "val", 0);
	command_set_comp_val_raw("wave_page.kd", "val", 0);

	/*温度限制界面UI初始化*/
	for (uint8_t i = 0; i < sizeof(alarm_name_list) / sizeof(alarm_name_list[0]); i++)
	{
		command_set_comp_val_raw(alarm_name_list[i], "val", weld_controller->alarm_temp[i]);
	}
	command_set_comp_val_raw("temp_page.GAIN1", "val", weld_controller->temp_gain1 * 100);
	command_set_comp_val_raw("temp_page.GAIN2", "val", weld_controller->temp_gain2 * 100);
	/*默认自动模式*/
	command_set_comp_val_raw("temp_page.switch", "val", 0);

	/*参数页面UI初始化*/
	command_send("page 1");
	command_set_comp_val_raw("RDY_SCH", "pic", RDY);
	command_set_comp_val_raw("ION_OFF", "pic", ION);
	command_set_comp_val_raw("SGW_CTW", "pic", SGW);
	command_set_comp_val_raw("GP", "val", 0);
	command_set_comp_val_raw("count", "val", 0);

	for (uint8_t i = 0; i < sizeof(param_time_list) / sizeof(param_time_list[0]); i++)
	{
		command_set_comp_val_raw(param_time_list[i], "val", weld_controller->weld_time[i]);
	}

	for (uint8_t i = 0; i < sizeof(param_temp_list) / sizeof(param_temp_list[0]); i++)
	{
		command_set_comp_val_raw(param_temp_list[i], "val", weld_controller->weld_temp[i]);
	}
}

void save_param(void *controller,
				int array_of_data,
				const uint16_t *temp,
				const uint8_t temp_len,
				const uint16_t *time,
				const uint8_t time_len)
{
	// weld_ctrl *ctrl = (weld_ctrl *)controller;

	SPI_Save_Word(array_of_data, 0);
	/*time1-time5*/
	for (uint8_t i = 0; i < TIME_NUM; i++)
	{
		SPI_Save_Word(time[i], TIME_BASE(array_of_data) + ADDR_OFFSET * i);
	}
	/*temp1-temp3*/
	for (uint8_t i = 0; i < TEMP_NUM; i++)
	{
		SPI_Save_Word(temp[i], TEMP_BASE(array_of_data) + ADDR_OFFSET * i);
	}

	// /*数据同步*/
	// for (uint8_t i = 0; i < sizeof(ctrl->weld_time) / sizeof(uint16_t); i++)
	// {
	// 	ctrl->weld_time[i] = time[i];
	// }
	// /*补偿时间*/
	// ctrl->hold_time = time[time_len - 1];
	// /*三段温度*/
	// for (uint8_t i = 0; i < sizeof(ctrl->weld_temp) / sizeof(uint16_t); i++)
	// {
	// 	ctrl->weld_temp[i] = temp[i];
	// }
}

void save_param_alarm(void *controller,
					  int array_of_data,
					  const uint16_t *temp,
					  const uint8_t temp_len,
					  const uint16_t *gain)
{
	weld_ctrl *ctrl = (weld_ctrl *)controller;
	/*存储组号*/
	SPI_Save_Word(array_of_data, 0);
	/*alarm1-alarm6*/
	for (uint8_t i = 0; i < ALARM_NUM; i++)
	{
		SPI_Save_Word(temp[i], ALARM_BASE(array_of_data) + ADDR_OFFSET * i);
	}

	/*gain1-gain2*/
	for (uint8_t i = 0; i < GAIN_NUM; i++)
	{
		SPI_Save_Word(gain[i], GAIN_BASE(array_of_data) + ADDR_OFFSET * i);
	}
	/*数据同步*/
	for (uint8_t i = 0; i < sizeof(ctrl->alarm_temp) / sizeof(uint16_t); i++)
	{
		ctrl->alarm_temp[i] = temp[i];
	}
	ctrl->temp_gain1 = gain[0] / 100.0;
	ctrl->temp_gain2 = gain[1] / 100.0;
}
void Load_param(void *controller, int array_of_data)
{
	Component_Queue *list = get_page_list(PARAM_PAGE);

	char *param_time_name_list[] = {
		"time1",
		"time2",
		"time3",
		"time4",
		"time5",
		"time6"};
	char *param_temp_name_list[] = {
		"temp1",
		"temp2",
		"temp3",
	};

	weld_ctrl *ctrl = (weld_ctrl *)controller;
	SPI_Save_Word(array_of_data, 0);
	/*参数加载*/
	uint16_t welding_time_load[5] = {0}, welding_Temp_load[3] = {0};
	for (uint8_t i = 0; i < sizeof(welding_time_load) / sizeof(uint16_t); i++)
	{
		welding_time_load[i] = SPI_Load_Word(TIME_BASE(array_of_data) + ADDR_OFFSET * i);
	}

	/* temp1~temp3*/
	for (uint8_t i = 0; i < sizeof(welding_Temp_load) / sizeof(uint16_t); i++)
	{
		welding_Temp_load[i] = SPI_Load_Word(TEMP_BASE(array_of_data) + ADDR_OFFSET * i);
	}

	/*param check & sync*/
	// time1-time6
	for (uint8_t i = 0; i < sizeof(welding_time_load) / sizeof(uint16_t); i++)
	{
		ctrl->weld_time[i] = welding_time_load[i];
		get_comp(list, param_time_name_list[i])->val = welding_time_load[i];
	}
	// temp1-temp3
	for (uint8_t i = 0; i < sizeof(welding_Temp_load) / sizeof(uint16_t); i++)
	{
		ctrl->weld_temp[i] = welding_Temp_load[i];
		get_comp(list, param_temp_name_list[i])->val = welding_Temp_load[i];
	}
}
void Load_param_alarm(void *controller, int array_of_data)
{
	Component_Queue *list = get_page_list(PARAM_PAGE);
	double gain_raw[2] = {0};
	uint16_t alarm_temperature_load[6] = {0};
	char *temp_name_list[] = {
		"alarm1",
		"alarm2",
		"alarm3",
		"alarm4",
		"alarm5",
		"alarm6",
	};
	weld_ctrl *ctrl = (weld_ctrl *)controller;
	/*param load*/
	for (uint8_t i = 0; i < sizeof(alarm_temperature_load) / sizeof(uint16_t); i++)
	{
		alarm_temperature_load[i] = SPI_Load_Word(ALARM_BASE(array_of_data) + ADDR_OFFSET * i);
	}

	for (uint8_t i = 0; i < sizeof(gain_raw) / sizeof(double); i++)
	{
		gain_raw[i] = SPI_Load_Word(GAIN_BASE(array_of_data) + ADDR_OFFSET * i);
	}

	/*param check & sync*/
	for (uint8_t i = 0; i < sizeof(alarm_temperature_load) / sizeof(uint16_t); i++)
	{
		if (alarm_temperature_load[i] > ALARM_MAX_TEMP)
			alarm_temperature_load[i] = ALARM_MAX_TEMP;

		ctrl->alarm_temp[i] = alarm_temperature_load[i];
		get_comp(list, temp_name_list[i])->val = alarm_temperature_load[i];
	}

	/*data sync to screen list*/
	if (gain_raw[0] / 100.0 != 0 && gain_raw[1] / 100.0 != 0)
	{
		ctrl->temp_gain1 = (double)gain_raw[0] / 100.0;
		ctrl->temp_gain2 = (double)gain_raw[1] / 100.0;
		get_comp(list, "GAIN1")->val = ctrl->temp_gain1 * 100;
		get_comp(list, "GAIN2")->val = ctrl->temp_gain2 * 100;
	}
	else
	{
		ctrl->temp_gain1 = DEFAULT_GAIN1;
		ctrl->temp_gain2 = DEFAULT_GAIN2;
		get_comp(list, "GAIN1")->val = DEFAULT_GAIN1 * 100;
		get_comp(list, "GAIN2")->val = DEFAULT_GAIN2 * 100;
	}
}

void Load_Coefficient(int array_of_data)
{
	uint16_t gain_raw1, gain_raw2;
	gain_raw1 = SPI_Load_Word(FIT_COEFFICIENT_BASE(array_of_data) + ADDR_OFFSET * 0);
	gain_raw2 = SPI_Load_Word(FIT_COEFFICIENT_BASE(array_of_data) + ADDR_OFFSET * 1);
	if (gain_raw1 != 0 && gain_raw2 != 0)
	{
		weld_controller->ss_coefficient.slope = gain_raw1 / 100.0;
		weld_controller->ss_coefficient.intercept = gain_raw2;
	}
}
