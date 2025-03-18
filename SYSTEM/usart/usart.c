
#include "sys.h"
#include "usart.h"
#include "crc16.h"
#include "protect.h"
#include "touchscreen.h"
#include "welding_process.h"
//////////////////////////////////////////////////////////////////////////////////
// 如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos 使用
#endif
//////////////////////////////////////////////////////////////////
// 加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 0
#pragma import(__use_no_semihosting)
// 标准库需要的支持函数
struct __FILE
{
	int handle;
};

FILE __stdout;
// 定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
// 重定义fputc函数
int fputc(int ch, FILE *f)
{
	while ((USART1->SR & 0X40) == 0)
		;
	USART1->DR = (uint8_t)ch;
	return ch;
}
#endif

uint8_t USART_RX_BUF3[USART3_REC_LEN3]; // 接收缓冲,最大USART_REC_LEN个字节.
uint8_t USART_RX_BUF[USART_REC_LEN];	// 接收缓冲,最大USART_REC_LEN个字节.

uint16_t ModBus_time[6];	   // 焊接时间6段
uint16_t ModBus_temp[3];	   // 焊接温度3段
uint16_t ModBus_alarm_temp[6]; // 限制温度6段

int receive_number_computer = 0; // 接收的数据个数
int Host_action = 0;			 // 主机动作
int Host_GP = 0;				 // GP
int Host_gain1_raw;				 // 增益暂存
int Host_gain2_raw;				 // 增益暂存
float Host_gain2;				 // 上位机增益1参数暂存（浮点数）
float Host_gain1;				 // 上位机增益2参数暂存（浮点数）

extern uint8_t ID_OF_MAS; // 焊机485通讯机号，默认是零
extern int RDY_SCH;		  // 参数设置按键

extern OS_Q UART_Msg;		   // 消息队列
extern Error_ctrl *err_ctrl;   // 错误控制器
extern Page_Param *page_param; // 页面信息

/*信号量接口*/
extern OS_SEM PAGE_UPDATE_SEM;
extern OS_SEM COMP_VAL_GET_SEM;
extern OS_SEM COMP_STR_GET_SEM;
extern OS_SEM ALARM_RESET_SEM;
extern OS_SEM COMPUTER_DATA_SYN_SEM; // 上位机数据同步信号
extern OS_SEM HOST_WELD_CTRL_SEM;	 // 上位机开启焊接信号
extern OS_SEM SENSOR_UPDATE_SEM;	 // 热电偶校准信号

/*焊接控制器*/
extern weld_ctrl *weld_controller;
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------触摸屏---------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/**
 * @description: touchscreen usart config
 * @param {u32} bound
 * @return {*}
 */
void uart_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/*GPIO AF config*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

	/*GPIO RXD TXD config*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*GPIO PA2 RX/TX control config*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*usart config*/
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	USART_Cmd(UART4, ENABLE);
	USART_ClearFlag(UART4, USART_FLAG_TC);

#if EN_USART4_RX
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
	/*NVIC config*/
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif
	RS485_TX_EN = 0; // 默认为接收模式
}

static volatile uint16_t receive_number = 0;
uint16_t get_receive_number()
{
	return receive_number;
}
void set_receive_number(uint16_t val)
{
	receive_number = val;
}

/**
 * @description: 读写线程的串口中断服务函数，进行初步数据解析，发送不同的信号
 * @return {*}
 */
void UART4_IRQHandler(void)
{

#if SYSTEM_SUPPORT_OS
	OSIntEnter();
#endif
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) // clear receive flag
	{
		USART_RX_BUF[receive_number++] = USART_ReceiveData(UART4);
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}

	if (USART_GetITStatus(UART4, USART_IT_IDLE) == SET) // ideal flag clear
	{
		OS_ERR err;
		if (receive_number >= MIN_CMD_LEN)
		{
			switch (USART_RX_BUF[0])
			{
			case CMD_OK:
				OSSemPost(&PAGE_UPDATE_SEM, OS_OPT_POST_ALL, &err);
				break;
			case CMD_PAGEID_RETURN:
				OSSemPost(&PAGE_UPDATE_SEM, OS_OPT_POST_ALL, &err);
				page_param->id = (Page_ID)USART_RX_BUF[1];
				break;

			case CMD_ALARM_RESET:
				OSSemPost(&ALARM_RESET_SEM, OS_OPT_POST_ALL, &err);
				break;

			case CMD_INT_VAR_RETURN:
				OSSemPost(&COMP_VAL_GET_SEM, OS_OPT_POST_ALL, &err);
				break;

			case CMD_STR_VAR_RETURN:
				OSSemPost(&COMP_STR_GET_SEM, OS_OPT_POST_ALL, &err);
				break;
			case CMD_SENSOR_UPDATE:
				OSSemPost(&SENSOR_UPDATE_SEM, OS_OPT_POST_ALL, &err);
				break;

			case CMD_DATA_TRANSFER_READY:
				break;

			default:
				break;
			}

			receive_number = 0;
		}

		/*clear flag*/
		receive_number = 0;
		int temp;
		temp = UART4->SR;
		temp = UART4->DR;
		temp = temp;
		USART_ClearITPendingBit(UART4, USART_IT_IDLE);
	}

	if (USART_GetITStatus(UART4, USART_IT_TC) == SET)
	{
		USART_ITConfig(UART4, USART_IT_TC, DISABLE);
		USART_ClearITPendingBit(UART4, USART_IT_TC);
	}

#if SYSTEM_SUPPORT_OS
	OSIntExit();
#endif
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------上位机通信---------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void usart3_init(u32 bound)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // 使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART3时钟

	// 串口2引脚复用映射
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10复用为USAR3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11复用为USART3

	// USART2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;		   // GPIOB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;		   // GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// PG9推挽输出，485模式控制
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		   // GPIOG9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化PG8

	// USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART3, &USART_InitStructure);										// 初始化串口3

	USART_Cmd(USART3, ENABLE); // 使能串口 2

	USART_ClearFlag(USART3, USART_FLAG_TC);

#if EN_USART3_RX
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启接收中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	// Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、
#endif
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 0; // 默认为接收模式
}

/**
 * @description:  设置上位机波特率，在触摸屏上修改波特率将调用此函数
 * @param {u32} bound
 * @return {*}
 */
void usart3_set_bound(u32 bound)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;										// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART3, &USART_InitStructure);										// 初始化串口2
	USART_ClearFlag(USART3, USART_FLAG_TC);
}

/**
 * @description:  控制板给上位机发送响应数据
 * @param {uint8_t} IDNUM
 * @param {uint8_t} flag
 * @param {uint8_t} comd
 * @return {*}
 */
void usart3_ack_to_host(uint8_t IDNUM, uint8_t flag, uint8_t comd)
{
	int crc16_data = 0;
	unsigned char array_ouput_end[7] = {0};
	array_ouput_end[0] = IDNUM; /* 焊接机号 */
	array_ouput_end[1] = 0xff;	/* 发送字节数 */
	array_ouput_end[2] = comd;	/* 返回当前指令码 */
	array_ouput_end[3] = (flag == 1) ? 0xff : 0x00;
	array_ouput_end[4] = (flag == 1) ? 0xff : 0x00; // 接收正确 发送ff 否则 00
	//	tempbk = get_crc16_x(array_ouput_end, 5);
	crc16_data = CRC16(array_ouput_end, 5);
	array_ouput_end[5] = crc16_data >> 8;
	array_ouput_end[6] = crc16_data & 0x00ff;

	BIT_ADDR(GPIOB_ODR_Addr, 9) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 7; t1++)
	{
		USART3->SR;
		USART_SendData(USART3, array_ouput_end[t1]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 0; // 设置为接收模式
}

static void computer_write_callback(void);
static void computer_write_all_callback(void);

/**
 * @description: 串口3中断，实现对上位机数据的接收和处理
 * @return {*}
 */
void USART3_IRQHandler(void) // 串口3中断服务程序
{
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif

	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // 接收中断
	{
		USART_RX_BUF3[receive_number_computer++] = USART_ReceiveData(USART3);
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}

	if (USART_GetITStatus(USART3, USART_IT_IDLE) == SET) // 空闲中断
	{
		OS_ERR err;
		int crc16_get = 0;
		uint8_t crc16H = 0, crc16L = 0;
		int sr_read;

		USART_ClearITPendingBit(USART3, USART_IT_IDLE); // 清除空闲中断
		/* 需要保证此时不在焊接过程才能进行数据通讯 */
		if (receive_number_computer > 2 && (page_param->key1 == RDY) && (ID_OF_MAS == USART_RX_BUF3[0]) && (false == err_occur(err_ctrl)) && (get_weld_flag() == IDEAL_MODE))
		{
			crc16_get = CRC16(USART_RX_BUF3, receive_number_computer - 2); // 检测数据是否正确
			crc16H = crc16_get >> 8;
			crc16L = crc16_get;
			/*crc16校验*/
			if (USART_RX_BUF3[receive_number_computer - 2] == crc16H && USART_RX_BUF3[receive_number_computer - 1] == crc16L)
			{
				switch (USART_RX_BUF3[1])
				{
					/*单个数据写入处理*/
				case RS485_CMD_WRITE:
					computer_write_callback();
					break;
					/*写入多个数值，就是整一页修改*/
				case RS485_CMD_WRITE_ALL:
					if (receive_number_computer < 20)
						break;
					computer_write_all_callback();
					break;

				case RS485_CMD_WELD_START:
					/*开启焊接信号*/
					OSSemPost(&HOST_WELD_CTRL_SEM, OS_OPT_POST_ALL, &err);
					break;
				case RS485_CMD_WELD_STOP:
					weld_controller->user_hook_callback(); // 停止焊接
					break;
				}
			}
		}

		/*单个数据修改，对上位机进行应答*/
		if ((Host_action >= 1 && Host_action <= 17) || USART_RX_BUF3[1] == 0x06)
			usart3_ack_to_host(ID_OF_MAS, 1, 0x06);

		/*多个数据修改，对上位机进行应答*/
		if (Host_action == 20 || USART_RX_BUF3[1] == 0x10)
			usart3_ack_to_host(ID_OF_MAS, 1, 0x10);

		/*清除标志*/
		sr_read = USART3->SR;
		sr_read = USART3->DR;
		sr_read = sr_read;

		for (uint8_t i = 0; i < sizeof(USART_RX_BUF3) / sizeof(USART_RX_BUF3[0]); i++) // 接收数组复位
			USART_RX_BUF3[i] = 0;

		receive_number_computer = 0;							  // 接收长度复位
		OSSemPost(&COMPUTER_DATA_SYN_SEM, OS_OPT_POST_ALL, &err); // 通知线程进行数据同步
	}

	if (USART_GetITStatus(USART3, USART_IT_TC) == SET)
	{
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);
		USART_ClearITPendingBit(USART3, USART_IT_TC);
	}

#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

static void computer_write_callback(void)
{
	int VALUE_DATA = 0;										// 被写入的单个数值大小
	VALUE_DATA = USART_RX_BUF3[4] * 256 + USART_RX_BUF3[5]; // 即16位，前面8位和后面8位
	switch (USART_RX_BUF3[3])								// USART_RX_BUF3[3]就是写入的地址
	{
	case 0:
		if (VALUE_DATA <= 20)
		{
			Host_GP = VALUE_DATA; // 参数组号
			Host_action = 1;
		}
		break;
	case 1:
		if (VALUE_DATA <= 999)
		{
			ModBus_time[0] = VALUE_DATA; // 焊接时间0
			Host_action = 2;
		}
		break;
	case 2:
		if (VALUE_DATA <= 999)
		{
			ModBus_time[1] = VALUE_DATA; // 焊接时间1
			Host_action = 3;
		}
		break;
	case 3:
		if (VALUE_DATA <= 999)
		{
			ModBus_time[3] = VALUE_DATA; // 焊接时间3
			Host_action = 4;
		}
		break;
	case 4:
		if (VALUE_DATA <= 999)
		{
			ModBus_time[4] = VALUE_DATA; // 焊接时间4
			Host_action = 5;
		}
		break;
	case 5:
		if (VALUE_DATA <= 999)
		{
			ModBus_time[5] = VALUE_DATA; // 焊接时间5
			Host_action = 6;
		}
		break;
	case 6:
		if (VALUE_DATA <= 650)
		{
			ModBus_temp[0] = VALUE_DATA; // 焊接温度0
			Host_action = 7;
		}
		break;
	case 7:
		if (VALUE_DATA <= 650)
		{
			ModBus_temp[1] = VALUE_DATA; // 焊接温度1
			Host_action = 8;
		}
		break;
	case 8:
		if (VALUE_DATA <= 650)
		{
			ModBus_temp[2] = VALUE_DATA; // 焊接温度2
			Host_action = 9;
		}
		break;
	case 9:
		if (VALUE_DATA <= 650)
		{
			ModBus_alarm_temp[0] = VALUE_DATA; // 报警温度0 up
			Host_action = 0;
		}
		break;
	case 10:
		if (VALUE_DATA <= 650)
		{
			ModBus_alarm_temp[1] = VALUE_DATA; // 报警温度0 down
			Host_action = 11;
		}
		break;
	case 11:
		if (VALUE_DATA <= 650)
		{
			ModBus_alarm_temp[2] = VALUE_DATA; // 报警温度1 up
			Host_action = 12;
		}
		break;
	case 12:
		if (VALUE_DATA <= 650)
		{
			ModBus_alarm_temp[3] = VALUE_DATA; // 报警温度1 down
			Host_action = 13;
		}
		break;
	case 13:
		if (VALUE_DATA <= 650)
		{
			ModBus_alarm_temp[4] = VALUE_DATA; // 报警温度2 up
			Host_action = 14;
		}
		break;
	case 14:
		if (VALUE_DATA <= 650)
		{
			ModBus_alarm_temp[5] = VALUE_DATA; // 报警温度2 down
			Host_action = 15;
		}
		break;
	case 15:
		if (VALUE_DATA <= 999)
		{
			Host_gain1 = VALUE_DATA * 0.01; // 温度增益1
			Host_gain1_raw = VALUE_DATA;
			Host_action = 16;
		}
		break;
	case 16:
		if (VALUE_DATA <= 999)
		{

			Host_gain2 = VALUE_DATA * 0.01; // 温度增益2
			Host_gain2_raw = VALUE_DATA;
			Host_action = 17;
		}
		break;
	}
}

static void computer_write_all_callback(void)
{

	int VALUE_DATA2[18] = {0}; // 定义一个存放16个数的数组
	for (uint8_t i = 0; i <= 16; i++)
	{
		VALUE_DATA2[i] = USART_RX_BUF3[2 * i + 4] * 256 + USART_RX_BUF3[2 * i + 5];
	}
	// GP值
	if (VALUE_DATA2[0] < 20)
		Host_GP = VALUE_DATA2[0];
	// 焊接各段时间,ModBus_time[2]弃用
	if (VALUE_DATA2[1] <= 999)
		ModBus_time[0] = VALUE_DATA2[1];
	if (VALUE_DATA2[2] <= 999)
		ModBus_time[1] = VALUE_DATA2[2];
	if (VALUE_DATA2[3] <= 999)
		ModBus_time[3] = VALUE_DATA2[3];
	if (VALUE_DATA2[4] <= 9999)
		ModBus_time[4] = VALUE_DATA2[4];
	if (VALUE_DATA2[5] <= 999)
		ModBus_time[5] = VALUE_DATA2[5];
	//  三段焊接温度
	if (VALUE_DATA2[6] <= 650)
		ModBus_temp[0] = VALUE_DATA2[6];
	if (VALUE_DATA2[7] <= 650)
		ModBus_temp[1] = VALUE_DATA2[7];
	if (VALUE_DATA2[8] <= 650)
		ModBus_temp[2] = VALUE_DATA2[8];
	// 温度限制
	if (VALUE_DATA2[9] <= 650)
		ModBus_alarm_temp[0] = VALUE_DATA2[9];
	if (VALUE_DATA2[10] <= 650)
		ModBus_alarm_temp[1] = VALUE_DATA2[10];
	if (VALUE_DATA2[11] <= 650)
		ModBus_alarm_temp[2] = VALUE_DATA2[11];
	if (VALUE_DATA2[12] <= 650)
		ModBus_alarm_temp[3] = VALUE_DATA2[12];
	if (VALUE_DATA2[13] <= 650)
		ModBus_alarm_temp[4] = VALUE_DATA2[13];
	if (VALUE_DATA2[14] <= 650)
		ModBus_alarm_temp[5] = VALUE_DATA2[14];
	// gain
	if (VALUE_DATA2[15] <= 999)
	{
		Host_gain1 = VALUE_DATA2[15] * 0.01;
		Host_gain1_raw = VALUE_DATA2[15];
	}
	if (VALUE_DATA2[16] <= 999)
	{
		Host_gain2 = VALUE_DATA2[16] * 0.01;
		Host_gain2_raw = VALUE_DATA2[16];
	}
	Host_action = 20;
}
