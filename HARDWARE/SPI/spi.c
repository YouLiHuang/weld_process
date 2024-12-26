/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-13 19:22:56
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-26 09:28:36
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
#include "spi.h"
#include "delay.h"
#include "key.h"
#include "welding_process.h"
#include "crc16.h"
#include "touchscreen.h"

/*--------------------------------------------------------------新触摸屏相关组件列表--------------------------------------------------------------*/
extern Page_Param *page_param;
extern Component_Queue *param_page_list;
extern Component_Queue *temp_page_list;
extern weld_ctrl *weld_controller;
/*-----------------------------------------------------------------------------------------------------------------------------------------------*/

extern int remember_array;
int text_1 = 0;
void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  // 使能SPI1时钟

	// GPIOB13(SCK)&GPIOB14(MISO)&GPIOB14(MISO)GPIOB15(MOSI)初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化

	// GPIOB12(CS)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		   // PB12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化
	CSN = 1;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); // PB13复用为 SPI1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2); // PB14复用为 SPI1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); // PB15复用为 SPI1

	// 这里只针对SPI口初始化
	// RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI1
	// RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI1

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

	SPI_Cmd(SPI2, ENABLE); // 使能SPI外?
}

// 发送数据函数(1个字节)
void SPI_Sendbyte(u8 data)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		; // 等待发送区空

	SPI_I2S_SendData(SPI2, data); // 通过外设SPIx发送一个byte数据

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		; // 等待接收完一个byte

	SPI_I2S_ReceiveData(SPI2); // 返回通过SPIx最近接收的数据
}

u8 SPI_Readbyte(void)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;

	SPI_I2S_SendData(SPI2, 0xff);

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;

	return SPI_I2S_ReceiveData(SPI2);

	//	Delay_ms(1);
}

void SPI_Save_Word(u16 data, u16 addr)
{
	WREN(); // SPI_Sendbyte(0x06)写使能
	CSN = 0;
	SPI_Sendbyte(0x02); // w_code
	SPI_Sendbyte(addr >> 8);
	SPI_Sendbyte(addr);
	SPI_Sendbyte(data >> 8);
	SPI_Sendbyte(data);
	CSN = 1;
	delay_ms(5);
}

u16 SPI_Load_Word(u16 addr)
{
	u16 tmp;
	CSN = 0;
	SPI_Sendbyte(0x03); // READ
	SPI_Sendbyte(addr >> 8);
	SPI_Sendbyte(addr);
	tmp = SPI_Readbyte() << 8;
	tmp = tmp | SPI_Readbyte();
	CSN = 1;
	delay_ms(5);
	return tmp;
}

void WREN(void)
{
	CSN = 0;
	SPI_Sendbyte(0x06); // WREN
	CSN = 1;
}

void spi_data_init(void)
{
#if RESET_SPI_DATA == 1
	/*数据初始化*/
	for (int array_of_data = 0; array_of_data < 21; array_of_data++)
	{
		// GP
		SPI_Save_Word(0, array_of_data);
		// welding_time[0]-welding_time[5]
		SPI_Save_Word(100 + array_of_data, 40 * (array_of_data + 1));
		SPI_Save_Word(300 + array_of_data, 40 * (array_of_data + 1) + 2);
		SPI_Save_Word(100 + array_of_data, 40 * (array_of_data + 1) + 4); // 可去除
		SPI_Save_Word(2500 + array_of_data, 40 * (array_of_data + 1) + 6);
		SPI_Save_Word(200 + array_of_data, 40 * (array_of_data + 1) + 8);
		SPI_Save_Word(50 + array_of_data, 40 * (array_of_data + 1) + 10);
		// welding_Temp
		SPI_Save_Word(150 + array_of_data, 40 * (array_of_data + 1) + 12);
		SPI_Save_Word(330 + array_of_data, 40 * (array_of_data + 1) + 14);
		SPI_Save_Word(100 + array_of_data, 40 * (array_of_data + 1) + 16);
		// 限制温度
		SPI_Save_Word(220 + array_of_data, 40 * (array_of_data + 1) + 18);
		SPI_Save_Word(50 + array_of_data, 40 * (array_of_data + 1) + 20);
		SPI_Save_Word(420 + array_of_data, 40 * (array_of_data + 1) + 22);
		SPI_Save_Word(200 + array_of_data, 40 * (array_of_data + 1) + 24);
		SPI_Save_Word(420 + array_of_data, 40 * (array_of_data + 1) + 26);
		SPI_Save_Word(50 + array_of_data, 40 * (array_of_data + 1) + 28);
		// 温度增益
		SPI_Save_Word(95 + array_of_data, 40 * (array_of_data + 1) + 30);
		SPI_Save_Word(70 + array_of_data, 40 * (array_of_data + 1) + 32);
	}
#endif

	// 软起动
	delay_ms(1000);
	TRAN1 = 1;
	delay_ms(1000);
	TRAN1 = 0;
	int remember_array_init = 0;
	remember_array_init = SPI_Load_Word(0); // 从内存加载首个GP值
	if (remember_array_init >= 20)
		remember_array_init = 0;
	remember_array = remember_array_init;

	/*首次从内存读取数据*/
	Load_param(weld_controller, remember_array_init);
	Load_param_alarm(weld_controller, remember_array_init);

	/*首次加载参数后需要发送到触摸屏*/
	char *param_name_list[] = {
		"time1",
		"time2",
		"time3",
		"time4",
		"time5",
		"temp1",
		"temp2",
		"temp3",
	};
	char *temp_name_list[] = {
		"alarm1",
		"alarm2",
		"alarm3",
		"alarm4",
		"alarm5",
		"alarm6",
		"GAIN1",
		"GAIN2",
		"GAIN3",
	};

	command_set_comp_val_raw("wave_page.kpf", "val", 0);
	command_set_comp_val_raw("wave_page.kp", "val", 0);
	command_set_comp_val_raw("wave_page.ki", "val", 0);
	command_set_comp_val_raw("wave_page.kd", "val", 0);
	/*温度限制界面UI初始化*/
	command_send("page 4");
	for (u8 i = 0; i < sizeof(temp_name_list) / sizeof(char *); i++)
	{
		Component *comp = get_comp(temp_page_list, temp_name_list[i]);
		if (comp != NULL)
			command_set_comp_val_raw(temp_name_list[i], "val", comp->val);
	}
	/*参数页面UI初始化*/
	command_send("page 1");
	command_set_comp_val_raw("RDY_SCH", "pic", RDY);
	command_set_comp_val_raw("ION_OFF", "pic", ION);
	command_set_comp_val_raw("SGW_CTW", "pic", SGW);
	command_set_comp_val_raw("GP", "val", 0);
	command_set_comp_val_raw("count", "val", 0);
	for (u8 i = 0; i < sizeof(param_name_list) / sizeof(char *); i++)
	{
		Component *comp = get_comp(param_page_list, param_name_list[i]);
		if (comp != NULL)
			command_set_comp_val_raw(param_name_list[i], "val", comp->val);
	}
}

void save_param(weld_ctrl *ctrl, int array_of_data, const u16 *temp, const u8 temp_len, const u16 *time, const u8 time_len)
{
	/*保存组别*/
	SPI_Save_Word(array_of_data, 0);
	/*保存三段温度和6个时间*/
	SPI_Save_Word(time[0], 40 * (array_of_data + 1));	   // time1
	SPI_Save_Word(time[1], 40 * (array_of_data + 1) + 2);  // time2
	SPI_Save_Word(time[2], 40 * (array_of_data + 1) + 4);  // time3
	SPI_Save_Word(time[3], 40 * (array_of_data + 1) + 6);  // time4
	SPI_Save_Word(time[4], 40 * (array_of_data + 1) + 8);  // time5
	SPI_Save_Word(time[5], 40 * (array_of_data + 1) + 10); // 0

	SPI_Save_Word(temp[0], 40 * (array_of_data + 1) + 12); // temp1
	SPI_Save_Word(temp[1], 40 * (array_of_data + 1) + 14); // temp2
	SPI_Save_Word(temp[2], 40 * (array_of_data + 1) + 16); // temp3

	/*数据同步*/
	for (u8 i = 0; i < sizeof(ctrl->weld_time) / sizeof(u16); i++)
	{
		ctrl->weld_time[i] = time[i];
	}
	for (u8 i = 0; i < sizeof(ctrl->weld_temp) / sizeof(u16); i++)
	{
		ctrl->weld_temp[i] = temp[i];
	}
}
void save_param_alarm(weld_ctrl *ctrl, int array_of_data, const u16 *temp, const u8 temp_len, const u16 *gain)
{
	/*存储组号*/
	SPI_Save_Word(array_of_data, 0);
	/*存储6个限制温度*/
	SPI_Save_Word(temp[0], 40 * (array_of_data + 1) + 18);
	SPI_Save_Word(temp[1], 40 * (array_of_data + 1) + 20);
	SPI_Save_Word(temp[2], 40 * (array_of_data + 1) + 22);
	SPI_Save_Word(temp[3], 40 * (array_of_data + 1) + 24);
	SPI_Save_Word(temp[4], 40 * (array_of_data + 1) + 26);
	SPI_Save_Word(temp[5], 40 * (array_of_data + 1) + 28);
	/*存储2个温度系数*/
	SPI_Save_Word(gain[0], 40 * (array_of_data + 1) + 30);
	SPI_Save_Word(gain[1], 40 * (array_of_data + 1) + 32);

	/*数据同步*/
	for (u8 i = 0; i < sizeof(ctrl->alarm_temp) / sizeof(u16); i++)
	{
		ctrl->alarm_temp[i] = temp[i];
	}

	ctrl->temp_gain1 = gain[0] / 100.0;
	ctrl->temp_gain2 = gain[1] / 100.0;
}
void Load_param(weld_ctrl *ctrl, int array_of_data)
{
	SPI_Save_Word(array_of_data, 0);
	u16 welding_time_load[5] = {0}, welding_Temp_load[3] = {0};
	welding_time_load[0] = SPI_Load_Word(40 * (array_of_data + 1));		// pre load
	welding_time_load[1] = SPI_Load_Word(40 * (array_of_data + 1) + 2); // first step
	welding_time_load[2] = SPI_Load_Word(40 * (array_of_data + 1) + 4); // second step
	welding_time_load[3] = SPI_Load_Word(40 * (array_of_data + 1) + 6); // third step
	welding_time_load[4] = SPI_Load_Word(40 * (array_of_data + 1) + 8); // duty

	welding_Temp_load[0] = SPI_Load_Word(40 * (array_of_data + 1) + 12); // temp1~temp3
	welding_Temp_load[1] = SPI_Load_Word(40 * (array_of_data + 1) + 14);
	welding_Temp_load[2] = SPI_Load_Word(40 * (array_of_data + 1) + 16);

	/*数据校验*/

	/*参数修改*/
	for (u8 i = 0; i < sizeof(welding_time_load) / sizeof(u16); i++)
	{
		ctrl->weld_time[i] = welding_time_load[i];
	}

	for (u8 i = 0; i < sizeof(welding_Temp_load) / sizeof(u16); i++)
	{
		ctrl->weld_temp[i] = welding_Temp_load[i];
	}

	/*数据同步*/
	// time1-time5
	for (u8 i = 0; i < sizeof(welding_time_load) / sizeof(u16); i++)
	{
		char *name = (char *)calloc(10, sizeof(char));
		if (name)
		{
			memset(name, 0, 10);
			sprintf(name, "time%d", i + 1);
			if (get_comp(param_page_list, name) != NULL)
				get_comp(param_page_list, name)->val = welding_time_load[i];
			free(name);
		}
	}
	// temp1-temp3
	for (u8 i = 0; i < sizeof(welding_Temp_load) / sizeof(u16); i++)
	{
		char *name = (char *)calloc(10, sizeof(char));
		if (name)
		{
			memset(name, 0, 10);
			sprintf(name, "temp%d", i + 1);
			if (get_comp(param_page_list, name) != NULL)
				get_comp(param_page_list, name)->val = welding_Temp_load[i];
			free(name);
		}
	}
}
void Load_param_alarm(weld_ctrl *ctrl, int array_of_data)
{
	u16 alarm_temperature_load[6] = {0};
	alarm_temperature_load[0] = SPI_Load_Word(40 * (array_of_data + 1) + 18);
	alarm_temperature_load[1] = SPI_Load_Word(40 * (array_of_data + 1) + 20);
	alarm_temperature_load[2] = SPI_Load_Word(40 * (array_of_data + 1) + 22);
	alarm_temperature_load[3] = SPI_Load_Word(40 * (array_of_data + 1) + 24);
	alarm_temperature_load[4] = SPI_Load_Word(40 * (array_of_data + 1) + 26);
	alarm_temperature_load[5] = SPI_Load_Word(40 * (array_of_data + 1) + 28);

	/*参数校验*/
	for (u8 i = 0; i < sizeof(alarm_temperature_load) / sizeof(u16); i++)
	{
		if (alarm_temperature_load[i] > MAX_TEMP)
			alarm_temperature_load[i] = MAX_TEMP;
	}

	for (u8 i = 0; i < sizeof(alarm_temperature_load) / sizeof(u16); i++)
	{
		ctrl->alarm_temp[i] = alarm_temperature_load[i];
	}

	/*参数修改*/
	double gain1_raw, gain2_raw;
	gain1_raw = (double)SPI_Load_Word(40 * (array_of_data + 1) + 30);
	gain2_raw = (double)SPI_Load_Word(40 * (array_of_data + 1) + 32);

	ctrl->temp_gain1 = (double)gain1_raw / 100.0;
	ctrl->temp_gain2 = (double)gain2_raw / 100.0;

	/*参数同步*/
	get_comp(temp_page_list, "GAIN1")->val = gain1_raw;
	get_comp(temp_page_list, "GAIN2")->val = gain2_raw;
	for (u8 i = 0; i <= sizeof(alarm_temperature_load) / sizeof(u16); i++)
	{
		char *name = (char *)calloc(10, sizeof(char));
		if (name)
		{
			memset(name, 0, 10);
			sprintf(name, "temp%d", i + 1);
			get_comp(temp_page_list, name)->val = alarm_temperature_load[i];
			free(name);
		}
	}
}
