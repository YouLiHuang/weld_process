/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-06 21:04:53
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-07 09:17:37
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "mb.h"
#include "mbport.h"
#include "port_bsp.h"
#include "usart.h"
#include "timer.h"

/**
 * @description: modbus usart bsp
 * @param {uint32_t} bound
 * @return {*}
 */
void modbus_serial_init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    /*RX/TX GPIO Config*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*R/T PIN Config*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*USART Config*/
    USART_InitStructure.USART_BaudRate = bound;                                     // rate set
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 8bits
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // one stop bit
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // nm parity
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // no
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // rx/tx
    USART_Init(USART3, &USART_InitStructure);                                       // init
    USART_Cmd(USART3, ENABLE);

    /*NVIC Config(group2)---must lower than timer1 & timer5*/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @description: modbus timer
 * APH1 defalut clock=168M/4=42M pre(=4)!=1 so APB1_TIM12_CLK=42M*2=84M
 * freq=168M
 * @param {uint32_t}
 * @return {*}
 */
void modbus_timer_init(uint32_t usTimerT35_50us)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint32_t period = (usTimerT35_50us * 50) - 1;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // clock no div
    TIM_TimeBaseStructure.TIM_Prescaler = (84 - 1);             // 1MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // Up count
    TIM_TimeBaseStructure.TIM_Period = period;                  // 1750us

    TIM_TimeBaseInit(MODBUS_TIMER, &TIM_TimeBaseStructure);
    TIM_Cmd(MODBUS_TIMER, DISABLE);

    /*NVIC Config*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void MODBUS_UART_SEND(const char *buffer, const uint16_t len)
{
    BIT_ADDR(GPIOB_ODR_Addr, 9) = 1;
    for (int i = 0; i < len; i++)
    {
        USART_SendData(MODBUS_SERIAL_PORT, buffer[i]);
        while (USART_GetFlagStatus(MODBUS_SERIAL_PORT, USART_FLAG_TC) == RESET)
            ;
    }
    BIT_ADDR(GPIOB_ODR_Addr, 9) = 0;
}
