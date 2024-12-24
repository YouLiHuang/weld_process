#include "sys.h"
#include "usart.h"
#include "crc16.h"
#include "protect.h"
#include "touchscreen.h"
//////////////////////////////////////////////////////////////////////////////////
// 如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos 使用
#endif
//////////////////////////////////////////////////////////////////
// 加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
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
		; // 循环发送,直到发送完毕
	USART1->DR = (u8)ch;
	return ch;
}
#endif

// 注意,读取USARTx->SR能避免莫名其妙的错误

struct crc16_struct1
{
	unsigned char crc16_high;
	unsigned char crc16_low;
};
struct crc16_struct3
{

	unsigned char crc16_high;
	unsigned char crc16_low;
};

u8 USART_RX_BUF3[USART3_REC_LEN3]; // 接收缓冲,最大USART_REC_LEN个字节.
u8 USART_RX_BUF[USART_REC_LEN];	   // 接收缓冲,最大USART_REC_LEN个字节.

uint16_t ModBus_time[6];	   // 焊接时间6段
uint16_t ModBus_temp[3];	   // 焊接温度3段
uint16_t ModBus_alarm_temp[6]; // 限制温度6段

int receive_number3 = 0;
int statee3 = 0;

int remember_array_1 = 0; // GP
int temp_remember_coefficient;

int dispose_flag = 0;
int temp_remember_coefficient_1; // 上位机增益参数暂存

struct crc16_struct1 temp1 = {0x00, 0x00}; // 触摸屏crc16 校验
struct crc16_struct3 temp2 = {0x00, 0x00}; // 上位机crc16 校验
float temp_temperature_coefficient_1;	   // 上位机增益1参数暂存
float temp_temperature_coefficient;		   // 上位机增益2参数暂存

extern int RDY_SCH;
int require_state;
extern u8 ID_OF_MAS;
extern u32 BOUND_SET;
extern u32 last_bound_set;
extern u8 last_id_of_mas;
extern u8 welding_flag;

/*消息队列*/
extern OS_Q UART_Msg;
extern Error_ctrl *err_ctrl;
extern Page_Param *page_param;
extern OS_SEM ALARM_RESET_SEM;

/**
 * @description: 串口初始化，用于触摸屏通讯处理
 * 				用到GPIOA2  GPIOA3 以及 PG8做使能
 * @param {u32} bound 波特率设置
 * @return {*}
 */
void uart_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); // 使能USART2时钟

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4); // GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); // GPIOA3复用为USART2

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		   // GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   // 初始化PA2，PA3

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		   // GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   // 初始化PA2，PA3
	// PG8推挽输出，485模式控制
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		   // GPIOG8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   // 初始化PG8

	USART_InitStructure.USART_BaudRate = bound;										// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(UART4, &USART_InitStructure);										// 初始化串口2

	USART_Cmd(UART4, ENABLE); // 使能串口 2

	USART_ClearFlag(UART4, USART_FLAG_TC);

#if EN_USART4_RX
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); // 开启接受中?
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
	// Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、

#endif
	RS485_TX_EN = 0; // 默认为接收模式
}

static volatile u16 receive_number = 0; // 记录接收字节长度
u16 get_receive_number()
{
	return receive_number;
}
void set_receive_number(u16 val)
{
	receive_number = val;
}

/**
 * @description: 定时器4中断函数
 * 				实现功能，使用空闲中断接收触摸屏回传数据，并对数据进行crc16校验，同时调用disposeof_information()分析数据
 * @return {*}
 */
void UART4_IRQHandler(void) // 串口4中断服务程序
{

#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) // 接收中断
	{
		USART_RX_BUF[receive_number++] = USART_ReceiveData(UART4);
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}

	if (USART_GetITStatus(UART4, USART_IT_IDLE) == SET) // 空闲中断
	{
		OS_ERR err;
		if (receive_number >= 2)
		{
			/*实在没办法，页面刷新需要特别及时，因此在中断当中刷新*/
			if (USART_RX_BUF[0] == CMD_PAGEID_RETURN && USART_RX_BUF[1] <= UART_PAGE && USART_RX_BUF[1] >= PARAM_PAGE)
				page_param->id = (Page_ID)USART_RX_BUF[1];
			if (USART_RX_BUF[0] == CMD_ALARM_RESET && USART_RX_BUF[1] <= UART_PAGE && USART_RX_BUF[1] >= PARAM_PAGE)
				OSSemPost(&ALARM_RESET_SEM, OS_OPT_POST_ALL, &err);

			OSQPost(&UART_Msg,
					(void *)&USART_RX_BUF[0],
					sizeof(u8) * receive_number,
					OS_OPT_POST_FIFO,
					&err); // 发送消息到队列
		}
		// 清除空闲中断标志位
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
	OSIntExit(); // 退出中断
#endif
}

/**
 * @description:  使用串口4发送数据，暂未使用
 * @param {u8} *buf 数组
 * @param {u8} len 长度
 * @return {*}
 */
void RS485_Send_Data(u8 *buf, u8 len)
{
	u8 t;
	RS485_TX_EN = 1;		  // 设置为发送模式
	for (t = 0; t < len; t++) // 循环发送数据
	{
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
			;						   // 等待发送结束
		USART_SendData(UART4, buf[t]); // 发送数据
	}
	while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
		;			 // 等待发送结束
	RS485_TX_EN = 0; // 设置为接收模式
}

/**
 * @description: 串口3初始化，用于触摸屏通讯处理
 * 				用到GPIOB10  GPIOB11 以及 PG9做使能
 * @param {u32} bound 波特率设置
 * @return {*}
 */
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
	USART_Init(USART3, &USART_InitStructure);										// 初始化串口2

	USART_Cmd(USART3, ENABLE); // 使能串口 2

	USART_ClearFlag(USART3, USART_FLAG_TC);

#if EN_USART3_RX
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启接受中?
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	// Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、
#endif
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 1; // 默认为接收模式
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
 * @param {u8} IDNUM
 * @param {u8} flag
 * @param {u8} comd
 * @return {*}
 */
void usart3_ack_to_host(u8 IDNUM, u8 flag, u8 comd)
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

extern OS_SEM COMPUTER_DATA_SYN_SEM; // 上位机数据同步信号
/**
 * @description:  串口3中断
 * 			1、实现对上位机数据的接收和处理
 * @return {*}
 */
void USART3_IRQHandler(void) // 串口3中断服务程序
{
	u8 Res_3;
#if SYSTEM_SUPPORT_OS // 使用UCOS操作系统
	OSIntEnter();
#endif

	if (USART_GetITStatus(USART3, USART_IT_TC) == SET)
	{
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);
		USART_ClearITPendingBit(USART3, USART_IT_TC);
	}
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // 接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{

		Res_3 = USART_ReceiveData(USART3); //(USART1->DR);	//读取接收到的数据
		USART_RX_BUF3[receive_number3] = Res_3;
		receive_number3++;
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}

	if (USART_GetFlagStatus(USART3, USART_FLAG_IDLE) != RESET)
	{
		int crc16_get = 0;
		int temp;
		if (receive_number3 >= 2 && (RDY_SCH == 0) && (ID_OF_MAS == USART_RX_BUF3[0]) && (false == err_occur(err_ctrl)) && (welding_flag == 1)) /* 需要保证此时不在焊接过程才能进行数据通讯 */
		{
			crc16_get = CRC16(USART_RX_BUF3, receive_number3 - 2); // 检测数据是否正确
			temp2.crc16_high = crc16_get >> 8;
			temp2.crc16_low = crc16_get;
			if (USART_RX_BUF3[receive_number3 - 2] == temp2.crc16_high && USART_RX_BUF3[receive_number3 - 1] == temp2.crc16_low)
			{
				if (USART_RX_BUF3[1] == 0x06) // 单个数据写入处理
				{
					int VALUE_DATA = 0;										// 被写入的单个数值大小
					VALUE_DATA = USART_RX_BUF3[4] * 256 + USART_RX_BUF3[5]; // 即16位，前面8位和后面8位
					switch (USART_RX_BUF3[3])								// USART_RX_BUF3[3]就是写入的地址
					{
					case 0:
						if (VALUE_DATA <= 20)
						{
							remember_array_1 = VALUE_DATA; // 参数组号
							statee3 = 1;
						}
						break;
					case 1:
						if (VALUE_DATA <= 999)
						{
							ModBus_time[0] = VALUE_DATA; // 焊接时间0
							statee3 = 2;
						}
						break;
					case 2:
						if (VALUE_DATA <= 999)
						{
							ModBus_time[1] = VALUE_DATA; // 焊接时间1
							statee3 = 3;
						}
						break;
					case 3:
						if (VALUE_DATA <= 999)
						{
							ModBus_time[3] = VALUE_DATA; // 焊接时间3
							statee3 = 4;
						}
						break;
					case 4:
						if (VALUE_DATA <= 999)
						{
							ModBus_time[4] = VALUE_DATA; // 焊接时间4
							statee3 = 5;
						}
						break;
					case 5:
						if (VALUE_DATA <= 999)
						{
							ModBus_time[5] = VALUE_DATA; // 焊接时间5
							statee3 = 6;
						}
						break;
					case 6:
						if (VALUE_DATA <= 650)
						{
							ModBus_temp[0] = VALUE_DATA; // 焊接温度0
							statee3 = 7;
						}
						break;
					case 7:
						if (VALUE_DATA <= 650)
						{
							ModBus_temp[1] = VALUE_DATA; // 焊接温度1
							statee3 = 8;
						}
						break;
					case 8:
						if (VALUE_DATA <= 650)
						{
							ModBus_temp[2] = VALUE_DATA; // 焊接温度2
							statee3 = 9;
						}
						break;
					case 9:
						if (VALUE_DATA <= 650)
						{
							ModBus_alarm_temp[0] = VALUE_DATA; // 报警温度0 up
							statee3 = 0;
						}
						break;
					case 10:
						if (VALUE_DATA <= 650)
						{
							ModBus_alarm_temp[1] = VALUE_DATA; // 报警温度0 down
							statee3 = 11;
						}
						break;
					case 11:
						if (VALUE_DATA <= 650)
						{
							ModBus_alarm_temp[2] = VALUE_DATA; // 报警温度1 up
							statee3 = 12;
						}
						break;
					case 12:
						if (VALUE_DATA <= 650)
						{
							ModBus_alarm_temp[3] = VALUE_DATA; // 报警温度1 down
							statee3 = 13;
						}
						break;
					case 13:
						if (VALUE_DATA <= 650)
						{
							ModBus_alarm_temp[4] = VALUE_DATA; // 报警温度2 up
							statee3 = 14;
						}
						break;
					case 14:
						if (VALUE_DATA <= 650)
						{
							ModBus_alarm_temp[5] = VALUE_DATA; // 报警温度2 down
							statee3 = 15;
						}
						break;
					case 15:
						if (VALUE_DATA <= 999)
						{
							temp_temperature_coefficient = VALUE_DATA * 0.01; // 温度增益1
							temp_remember_coefficient = VALUE_DATA;
							statee3 = 16;
						}
						break;
					case 16:
						if (VALUE_DATA <= 999)
						{

							temp_temperature_coefficient_1 = VALUE_DATA * 0.01; // 温度增益2
							temp_remember_coefficient_1 = VALUE_DATA;
							statee3 = 17;
						}
						break;
					}
				}
				if (USART_RX_BUF3[1] == 0x10 && receive_number3 == 40) // 写入多个数值，就是整一页修改
				{
					int VALUE_DATA2[18] = {0}; // 定义一个存放16个数的数组
					VALUE_DATA2[0] = USART_RX_BUF3[4] * 256 + USART_RX_BUF3[5];
					VALUE_DATA2[1] = USART_RX_BUF3[6] * 256 + USART_RX_BUF3[7];
					VALUE_DATA2[2] = USART_RX_BUF3[8] * 256 + USART_RX_BUF3[9];
					VALUE_DATA2[3] = USART_RX_BUF3[10] * 256 + USART_RX_BUF3[11];
					VALUE_DATA2[4] = USART_RX_BUF3[12] * 256 + USART_RX_BUF3[13];
					VALUE_DATA2[5] = USART_RX_BUF3[14] * 256 + USART_RX_BUF3[15];
					VALUE_DATA2[6] = USART_RX_BUF3[16] * 256 + USART_RX_BUF3[17];
					VALUE_DATA2[7] = USART_RX_BUF3[18] * 256 + USART_RX_BUF3[19];
					VALUE_DATA2[8] = USART_RX_BUF3[20] * 256 + USART_RX_BUF3[21];
					VALUE_DATA2[9] = USART_RX_BUF3[22] * 256 + USART_RX_BUF3[23];
					VALUE_DATA2[10] = USART_RX_BUF3[24] * 256 + USART_RX_BUF3[25];
					VALUE_DATA2[11] = USART_RX_BUF3[26] * 256 + USART_RX_BUF3[27];
					VALUE_DATA2[12] = USART_RX_BUF3[28] * 256 + USART_RX_BUF3[29];
					VALUE_DATA2[13] = USART_RX_BUF3[30] * 256 + USART_RX_BUF3[31];
					VALUE_DATA2[14] = USART_RX_BUF3[32] * 256 + USART_RX_BUF3[33];
					VALUE_DATA2[15] = USART_RX_BUF3[34] * 256 + USART_RX_BUF3[35];
					VALUE_DATA2[16] = USART_RX_BUF3[36] * 256 + USART_RX_BUF3[37];
					// VALUE_DATA2[17] = USART_RX_BUF3[38] * 256 + USART_RX_BUF3[39];

					// GP值
					if (VALUE_DATA2[0] < 20)
						remember_array_1 = VALUE_DATA2[0];
					// 焊接各段时间
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
					// if (VALUE_DATA2[6] <= 999)
					// ModBus_time[5] = VALUE_DATA2[6];
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
						temp_temperature_coefficient = VALUE_DATA2[15] * 0.01;
						temp_remember_coefficient = VALUE_DATA2[15];
					}
					if (VALUE_DATA2[16] <= 999)
					{
						temp_temperature_coefficient_1 = VALUE_DATA2[16] * 0.01;
						temp_remember_coefficient_1 = VALUE_DATA2[16];
					}
					statee3 = 20;
				}
			}
		}
		if (statee3 >= 1 && statee3 <= 18) // 按照不同的标志位，对上位机进行应答
		{
			usart3_ack_to_host(ID_OF_MAS, 1, 0x06);
		}
		else if (USART_RX_BUF3[1] == 0x06)
		{
			usart3_ack_to_host(ID_OF_MAS, 0, 0x06);
		}
		if (statee3 == 20)
		{
			usart3_ack_to_host(ID_OF_MAS, 1, 0x10);
		}
		else if (USART_RX_BUF3[1] == 0x10)
		{
			usart3_ack_to_host(ID_OF_MAS, 0, 0x10);
		}

		temp = USART3->SR;
		temp = USART3->DR;
		if (temp == 0)
		{
		}
		USART_ClearITPendingBit(USART3, USART_IT_IDLE); // 清除空闲中断
		USART_RX_BUF3[0] = 0;							// 接收数组复位
		USART_RX_BUF3[1] = 0;
		USART_RX_BUF3[2] = 0;
		USART_RX_BUF3[3] = 0;
		USART_RX_BUF3[4] = 0;
		USART_RX_BUF3[5] = 0;
		USART_RX_BUF3[6] = 0;
		USART_RX_BUF3[7] = 0;
		receive_number3 = 0; // 接收长度复位

		/*通知线程进行数据同步*/
		OS_ERR err;
		OSSemPost(&COMPUTER_DATA_SYN_SEM, OS_OPT_POST_ALL, &err);
	}
#if SYSTEM_SUPPORT_OS
	OSIntExit(); // 退出中断
#endif
}

// 电脑串口通信
void usart3_send_char(u8 c)
{
	BIT_ADDR(GPIOB_ODR_Addr, 9) = 1;
	while ((USART3->SR & 0X40) == 0)
		; // 等待上一次发送完毕
	USART3->DR = c;
}
/*		传送数据给匿名四轴上位机软件(V2.6版本)
			fun:功能字. 0XA0~0XAF
			data:数据缓存区,最多28字节!!
			len:data区有效数据个数
			格式为：0x88+FUN+LEN+DATA+SUM
			SUM是0x88一直到DATA最后一字节的和，uint8格式。    */
void usart3_niming_report(u8 fun, u8 *data, u8 len)
{
	u8 send_buf[32];
	u8 i;
	if (len > 28)
		return;			   // 最多28字节数据
	send_buf[len + 3] = 0; // 校验数置零
	send_buf[0] = 0X88;	   // 帧头
	send_buf[1] = fun;	   // 功能字
	send_buf[2] = len;	   // 数据长度
	for (i = 0; i < len; i++)
		send_buf[3 + i] = data[i]; // 复制数据
	for (i = 0; i < len + 3; i++)
		send_buf[len + 3] += send_buf[i]; // 计算校验和
	for (i = 0; i < len + 4; i++)
		usart3_send_char(send_buf[i]); // 发送数据到串口1
}

void test_send(short roll, short pitch, short yaw, short Cduk)
{
	u8 tbuf[12];
	tbuf[0] = (roll >> 8) & 0XFF;
	tbuf[1] = roll & 0XFF;
	tbuf[2] = (pitch >> 8) & 0XFF;
	tbuf[3] = pitch & 0XFF;
	tbuf[4] = (yaw >> 8) & 0XFF;
	tbuf[5] = yaw & 0XFF;
	tbuf[6] = (Cduk >> 8) & 0XFF;
	tbuf[7] = Cduk & 0XFF;

	usart3_niming_report(0XA1, tbuf, 8); // 自定义帧,0XA1
}
