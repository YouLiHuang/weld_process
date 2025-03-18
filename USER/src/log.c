/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-02-11 15:21:19
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-02-11 19:48:40
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

#include "stm32f4xx.h"
#include "sys.h"
#include "log.h"
#include "usart.h"

#if 0
#pragma import(__use_no_semihosting)
#endif

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else

/*define file object*/
typedef struct FIL __FILE;

/*implementation of sys_exit to avoid hardfault*/
void _sys_exit(int x)
{
    x = x;
}

/*Redirected interface*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{

    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART3->SR;
    /* Loop until the end of transmission */
    BIT_ADDR(GPIOB_ODR_Addr, 9) = 1; // send mode

    USART_SendData(USART3, ch);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET)
        ;
    BIT_ADDR(GPIOB_ODR_Addr, 9) = 0; // receive mode

    return ch;
}

/**
 * @description:
 * @param {uint32_t} bauds
 * @return {*}
 */
void log_bsp_init(uint32_t bauds)
{
    // 使能 GPIOB 和 USART1 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 配置 PB6 为 USART1_TX（复用功能），PB7 为 USART1_RX（复用功能）
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 将 PB6 和 PB7 映射到 USART1（AF7）
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    USART_InitTypeDef USART_InitStruct;

    USART_InitStruct.USART_BaudRate = bauds;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);
}
