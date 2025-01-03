#include "crc16.h"
#include "delay.h"
#include "usart.h"
#include "HCSR04.h"
#include "includes.h"
#include "spi.h"
#include "touchscreen.h"

extern OS_Q UART_Msg;

struct crc16_struct // crc 16 校验结构体，用于数据校验
{
	unsigned char crc16_high;
	unsigned char crc16_low;
};

u8 welding_data_change_flag[5] = {0};	 // 用于多次验证读数，防止读取数据出错，各组参数改变标志位
u8 last_group_array = 0;				 // 用于多次验证读数，防止读取数据出错，存放上一次的组号
u8 read_group_nums = 0;					 // 用于多次验证读数，防止读取数据出错，表示第几次读数
u8 last_read_group_num = 0;				 // 用于多次验证读数，防止读取数据出错，第一读取的组号
struct crc16_struct temp = {0x00, 0x00}; // 实例化一个crc16对象
int remember_array = -1;				 // 焊接参数组号存储
int ION_IOF = 0;						 // 电流开关
int RDY_SCH = 0;						 // 参数修改标志
int SGW_CTW = 0;						 // 单点连续标志
int RESET_K = 0;						 // 复位按键
int flag_data_1 = 0;					 // 用于上述四个标志位的读取
int disposing_data_flag = 0x0000;		 // 数据修改标志，假设有数据修改，此数据将会被置位
int remember_coefficient = 0;			 // 温度增益1，触摸屏上数据为1.50时，实际为150，需要除以100，将其复原
int remember_coefficient_1 = 0;			 // 温度增益1，触摸屏上数据为1.50时，实际为150，需要除以100，将其复原
int update_idx_floor;					 // 插值线性化去向下取整
int update_idx_ceil;					 // 插值线性化向上取整
double update_interval;					 // 插值线性化，获取间隔
double update_index;					 // 插值线性化，获取坐标
double coffie_t;						 // 插值线性化，均分系数

/*------------------------------------新旧接口对接------------------------------------*/

extern weld_ctrl *weld_controller;

/*------------------------------------------------------------------------------------*/
extern int require_state;

int temp_data;
int counting_tt;

// crc 16 查找表，高字节
unsigned char auchCRCHi[] =
	{
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
		0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
		0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40};

// crc 16 查找表，低字节
unsigned char auchCRCLo[] =
	{
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
		0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
		0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
		0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
		0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
		0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
		0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
		0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
		0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
		0x40};

/*
   crc 16 校验计算
*/
unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen)
{
	unsigned char uchCRCHi = 0xFF;
	unsigned char uchCRCLo = 0xFF;
	unsigned uIndex;
	while (usDataLen--)
	{
		uIndex = uchCRCLo ^ *puchMsg++;
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCLo << 8 | uchCRCHi);
}

/*
   crc 16 校验计算，计算7个字节
*/
struct crc16_struct get_crc16_7(unsigned char input_array[7], int lenof_data)
{
	struct crc16_struct temp = {0x00, 0x00};
	int crc16_data = 0;
	crc16_data = CRC16(input_array, lenof_data);
	temp.crc16_high = crc16_data >> 8;
	temp.crc16_low = crc16_data;
	return temp;
}

/*
  读写寄存器，延时20ms
*/
void output_data_plus(unsigned char anumber, unsigned char aaddress_high, unsigned char aaddress_low, unsigned char count_high, unsigned char count_low)
{
	OS_ERR err;
	unsigned char array_ouput_end[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	array_ouput_end[1] = anumber;
	array_ouput_end[2] = aaddress_high;
	array_ouput_end[3] = aaddress_low;
	array_ouput_end[4] = count_high;
	array_ouput_end[5] = count_low;
	temp = get_crc16_7(array_ouput_end, 6);
	array_ouput_end[6] = temp.crc16_high;
	array_ouput_end[7] = temp.crc16_low;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 8; t1++)
	{
		USART_SendData(UART4, array_ouput_end[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0;						// 设置为接收模式
	OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_PERIODIC, &err); // 延时20ms
}

/*
  读写寄存器，延时60ms,用于适配新屏幕
*/
void output_data_plus_60ms(unsigned char anumber, unsigned char aaddress_high, unsigned char aaddress_low, unsigned char count_high, unsigned char count_low)
{
	OS_ERR err;
	unsigned char array_ouput_end[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	array_ouput_end[1] = anumber;
	array_ouput_end[2] = aaddress_high;
	array_ouput_end[3] = aaddress_low;
	array_ouput_end[4] = count_high;
	array_ouput_end[5] = count_low;
	temp = get_crc16_7(array_ouput_end, 6);
	array_ouput_end[6] = temp.crc16_high;
	array_ouput_end[7] = temp.crc16_low;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 8; t1++)
	{
		USART_SendData(UART4, array_ouput_end[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0;						// 设置为接收模式
	OSTimeDlyHMSM(0, 0, 0, 60, OS_OPT_TIME_PERIODIC, &err); // 延时20ms
}

/*
  读写寄存器，无延时
*/
void output_data_plus_nodelay(unsigned char anumber, unsigned char aaddress_high, unsigned char aaddress_low, unsigned char count_high, unsigned char count_low)
{
	unsigned char array_ouput_end[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	array_ouput_end[1] = anumber;
	array_ouput_end[2] = aaddress_high;
	array_ouput_end[3] = aaddress_low;
	array_ouput_end[4] = count_high;
	array_ouput_end[5] = count_low;
	temp = get_crc16_7(array_ouput_end, 6);
	array_ouput_end[6] = temp.crc16_high;
	array_ouput_end[7] = temp.crc16_low;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 8; t1++)
	{
		USART_SendData(UART4, array_ouput_end[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0; // 设置为接收模式
}

/*
   发送焊接数据，和计数值，暂未使用
*/
void output_data_plus_add(int data_welding, int data_counting)
{
	OS_ERR err;
	unsigned char array_ouput_end[13] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	array_ouput_end[1] = 0x10;
	array_ouput_end[2] = 0x00;
	array_ouput_end[3] = 0xc8; // 200,即图表
	array_ouput_end[4] = 0x00;
	array_ouput_end[5] = 0x02;
	array_ouput_end[6] = 0x04;
	array_ouput_end[7] = data_counting >> 8;
	array_ouput_end[8] = data_counting & 0xff;
	array_ouput_end[9] = data_welding >> 8;
	array_ouput_end[10] = data_welding & 0xff;
	temp = get_crc16_7(array_ouput_end, 11);
	array_ouput_end[11] = temp.crc16_high;
	array_ouput_end[12] = temp.crc16_low;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 13; t1++)
	{
		USART_SendData(UART4, array_ouput_end[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0;						// 设置为接收模式
	OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_PERIODIC, &err); // 延时20ms
	output_data_plus(0x05, 0x00, 0xca, 0xff, 0x00);
}

/*
   报警使用，发送报警标志到触摸屏
*/
void output_data_plus_alarm(int alarm_data)
{

	unsigned char array_ouput_end[10] = {
		0x01,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
	};
	array_ouput_end[1] = 0x0f;
	array_ouput_end[2] = 0x00;
	array_ouput_end[3] = 0x10; // 报警界面-修改Bit状态。16-23bit
	array_ouput_end[4] = 0x00;
	array_ouput_end[5] = 0x08;
	array_ouput_end[6] = 0x01;
	array_ouput_end[7] = alarm_data;
	temp = get_crc16_7(array_ouput_end, 8);
	array_ouput_end[8] = temp.crc16_high;
	array_ouput_end[9] = temp.crc16_low;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 10; t1++)
	{
		USART_SendData(UART4, array_ouput_end[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0; // 设置为接收模式
}
/*
   发送焊接温度和焊接时间设置到触摸屏
*/
void output_data_s(volatile u16 inputarray1[3], volatile u16 inputarray2[6]) // 焊接时间6和温度3
{
	OS_ERR err;
	unsigned char outputarray_s[27] = {0};
	outputarray_s[0] = 0x01;
	outputarray_s[1] = 0x10;
	outputarray_s[2] = 0x00;
	outputarray_s[3] = 0x02;
	outputarray_s[4] = 0x00;
	outputarray_s[5] = 0x09;
	outputarray_s[6] = 0x12;
	outputarray_s[7] = inputarray2[0] >> 8;
	outputarray_s[8] = inputarray2[0] & 0xff;
	outputarray_s[9] = inputarray2[1] >> 8;
	outputarray_s[10] = inputarray2[1] & 0xff;
	outputarray_s[11] = inputarray2[2] >> 8;
	outputarray_s[12] = inputarray2[2] & 0xff;
	outputarray_s[13] = inputarray2[3] >> 8;
	outputarray_s[14] = inputarray2[3] & 0xff;
	outputarray_s[15] = inputarray2[4] >> 8;
	outputarray_s[16] = inputarray2[4] & 0xff;
	outputarray_s[17] = inputarray2[5] >> 8;
	outputarray_s[18] = inputarray2[5] & 0xff;
	outputarray_s[19] = inputarray1[0] >> 8;
	outputarray_s[20] = inputarray1[0] & 0xff;
	outputarray_s[21] = inputarray1[1] >> 8;
	outputarray_s[22] = inputarray1[1] & 0xff;
	outputarray_s[23] = inputarray1[2] >> 8;
	outputarray_s[24] = inputarray1[2] & 0xff;
	temp = get_crc16_7(outputarray_s, 25);
	outputarray_s[25] = temp.crc16_high;
	outputarray_s[26] = temp.crc16_low;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 27; t1++)
	{
		UART4->SR;
		USART_SendData(UART4, outputarray_s[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_PERIODIC, &err); // 延时20ms
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0;						// 设置为接收模式
}

/*
   发送报警温度和增益系数到触摸屏
*/
void output_data_ss(volatile u16 inputarray2[6], u16 single_input) // 温度限幅和增益数值
{
	OS_ERR err;
	unsigned char outputarray_s[23] = {0};
	outputarray_s[0] = 0x01;
	outputarray_s[1] = 0x10;
	outputarray_s[2] = 0x00;
	outputarray_s[3] = 0x0b;
	outputarray_s[4] = 0x00;
	outputarray_s[5] = 0x07;
	outputarray_s[6] = 0x0e;
	outputarray_s[7] = inputarray2[0] >> 8;
	outputarray_s[8] = inputarray2[0] & 0xff;
	outputarray_s[9] = inputarray2[1] >> 8;
	outputarray_s[10] = inputarray2[1] & 0xff;
	outputarray_s[11] = inputarray2[2] >> 8;
	outputarray_s[12] = inputarray2[2] & 0xff;
	outputarray_s[13] = inputarray2[3] >> 8;
	outputarray_s[14] = inputarray2[3] & 0xff;
	outputarray_s[15] = inputarray2[4] >> 8;
	outputarray_s[16] = inputarray2[4] & 0xff;
	outputarray_s[17] = inputarray2[5] >> 8;
	outputarray_s[18] = inputarray2[5] & 0xff;
	outputarray_s[19] = single_input >> 8;
	outputarray_s[20] = single_input & 0xff;
	temp = get_crc16_7(outputarray_s, 21);
	outputarray_s[21] = temp.crc16_high;
	outputarray_s[22] = temp.crc16_low;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 23; t1++)
	{
		UART4->SR;
		USART_SendData(UART4, outputarray_s[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	OSTimeDlyHMSM(0, 0, 0, 20, OS_OPT_TIME_PERIODIC, &err); // 延时10ms
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0;						// 设置为接收模式
}

/*
  有效波特率判断
*/
int bound_trail(u32 bound)
{
	if (bound == 0 || bound == 1200 || bound == 4800 || bound == 9600 || bound == 14000 || bound == 19200 || bound == 38400 || bound == 57600 || bound == 76800 || bound == 115200 || bound == 128000)
		return 1;
	else
		return 0;
}



/**
 * @description: 线性插值函数
 * @param {int} *input              输入数组
 * @param {u16} nums_of_input       输入数组大小
 * @param {int} *output             输出数组
 * @param {u16} nums_of_output      输出数组大小
 * @return {*}
 */
void linear_interpolation(u16 *input, u16 nums_of_input, u16 *output, u16 nums_of_output)
{
	update_interval = (double)(nums_of_input - 1) / (nums_of_output - 1);
	for (u16 i = 0; i < nums_of_output; i++)
	{
		update_index = i * update_interval;
		update_idx_floor = (u16)floor(update_index);
		update_idx_ceil = (u16)ceil(update_index);
		if (update_idx_ceil >= nums_of_input)
		{
			update_idx_ceil = update_idx_floor;
		}
		coffie_t = update_index - update_idx_floor;
		output[i] = input[update_idx_floor] * (1 - coffie_t) + input[update_idx_ceil] * coffie_t;
	}
}

/**
 * @description: 温度绘图使用，每次发送五个数据点
 * @param {int} point_number x坐标
 * @param {int} data_store y坐标
 * @return {*}
 */
void draw_point_function(int point_number, int data_store[5])
{
	__NOP();
	unsigned char array_ouput_end[29] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	array_ouput_end[1] = 0x10;
	array_ouput_end[2] = (counting_tt + 200) / 256;
	array_ouput_end[3] = (counting_tt + 200) % 256;
	array_ouput_end[4] = 0x00;
	array_ouput_end[5] = 0x0a;
	array_ouput_end[6] = 0x14;
	array_ouput_end[7] = point_number / 256;
	array_ouput_end[8] = point_number % 256;
	array_ouput_end[9] = data_store[0] / 256;
	array_ouput_end[10] = data_store[0] % 256;
	array_ouput_end[11] = (point_number + 1) / 256;
	array_ouput_end[12] = (point_number + 1) % 256;
	array_ouput_end[13] = (data_store[1]) / 256;
	array_ouput_end[14] = (data_store[1]) % 256;
	array_ouput_end[15] = (point_number + 2) / 256;
	array_ouput_end[16] = (point_number + 2) % 256;
	array_ouput_end[17] = (data_store[2]) / 256;
	array_ouput_end[18] = (data_store[2]) % 256;
	array_ouput_end[19] = (point_number + 3) / 256;
	array_ouput_end[20] = (point_number + 3) % 256;
	array_ouput_end[21] = (data_store[3]) / 256;
	array_ouput_end[22] = (data_store[3]) % 256;
	array_ouput_end[23] = (point_number + 4) / 256;
	array_ouput_end[24] = (point_number + 4) % 256;
	array_ouput_end[25] = (data_store[4]) / 256;
	array_ouput_end[26] = (data_store[4]) % 256;
	temp_data = CRC16(array_ouput_end, 27);
	array_ouput_end[27] = temp_data >> 8;
	array_ouput_end[28] = temp_data & 0x00ff;
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
	for (int t1 = 0; t1 < 29; t1++)
	{
		USART_SendData(UART4, array_ouput_end[t1]);
		while (USART_GetFlagStatus(UART4, USART_FLAG_TC) != SET)
			; // 把请求类型发送过去
	}
	BIT_ADDR(GPIOA_ODR_Addr, 2) = 0; // 设置为接收模式
	__NOP();
}
