/*
 * @Author: <lan> <1791060296@qq.com>
 * @Date: 2024-06-26 08:47:49
 * @LastEditors: <lan> <1791060296@qq.com>
 * @LastEditTime: 2024-07-02 15:28:09
 * @FilePath: \热压焊第三版_热电偶补偿20240626\SYSTEM\usart\usart.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"
#include "stdbool.h"

#define USART_REC_LEN 30               // 定义最大接收字节数 30
#define USART3_REC_LEN3 100            // 定义最大接收字节数 100
#define EN_USART4_RX 1                 // 使能（1）/禁止（0）串口1接收
#define EN_USART3_RX 1                 // 使能（1）/禁止（0）串口1接收
#define RS485_TX_EN PAout(2)           // 485模式控制.0,接收;1,发送.
extern u8 USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符

void uart_init(u32 bound);
void usart3_init(u32 bound);
void RS485_Send_Data(u8 *buf, u8 len);
void RS485_Receive_Data(u8 *buf, u8 *len);
void usart3_set_bound(u32 bound);
void usart3_niming_report(u8 fun, u8 *data, u8 len);
void test_send(short roll, short pitch, short yaw, short Cduk);
void usart3_send_char(u8 c);
void usart3_ack_to_host(u8 IDNUM, u8 flag, u8 comd); // 应答信号

u16 get_receive_number(void);
void set_receive_number(u16 val);

#endif
