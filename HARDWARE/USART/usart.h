#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"
#include "stdbool.h"

#define USART_REC_LEN 100                   // 定义最大接收字节数 30
#define USART3_REC_LEN3 100                 // 定义最大接收字节数 100
#define EN_USART4_RX 1                      // 使能（1）/禁止（0）串口接收
#define RS485_TX_EN PAout(2)                // 485模式控制.0,接收;1,发送.
extern uint8_t USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符

#define RS485_CMD_WRITE 0x06
#define RS485_CMD_WRITE_ALL 0x10
#define RS485_CMD_WELD_START 0x1f
#define RS485_CMD_WELD_STOP 0xf1

void uart_init(u32 bound);
void usart3_init(u32 bound);
void usart3_set_bound(u32 bound);


uint16_t get_receive_number(void);
void set_receive_number(uint16_t val);

typedef struct crc16_value
{
    unsigned char crc16_high;
    unsigned char crc16_low;
} crc16_value;

#endif
