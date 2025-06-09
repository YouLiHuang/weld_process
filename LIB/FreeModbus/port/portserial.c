/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-06 20:41:34
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-06 21:50:39
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "mb.h"
#include "mbport.h"
#include "port_bsp.h"
#include "usart.h"
#include "sys.h"

BOOL xMBPortSerialInit(UCHAR ucPort, ULONG ulBaudRate,
                       UCHAR ucDataBits, eMBParity eParity,
                       UCHAR ucStopBits)
{

    modbus_serial_init(ulBaudRate);

    return TRUE;
}

void vMBPortSerialEnable(BOOL rxEnable, BOOL txEnable)
{
    if (rxEnable)
    {
        USART_ITConfig(MODBUS_SERIAL_PORT, USART_IT_RXNE, ENABLE);
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
    else
    {
        USART_ITConfig(MODBUS_SERIAL_PORT, USART_IT_RXNE, DISABLE);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
    }

    if (txEnable)
    {
        USART_ITConfig(MODBUS_SERIAL_PORT, USART_IT_TC, ENABLE);
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
    }
    else
    {
        USART_ITConfig(MODBUS_SERIAL_PORT, USART_IT_TC, DISABLE);
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    }
}

BOOL xMBPortSerialPutByte(CHAR byte)
{
    USART_SendData(MODBUS_SERIAL_PORT, byte);
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR *byte)
{
    *byte = USART_ReceiveData(MODBUS_SERIAL_PORT);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}

void modbus_serial_irq(void)
{

    OSIntEnter();

    /*receive irq*/
    if (USART_GetITStatus(MODBUS_SERIAL_PORT, USART_IT_RXNE) == SET)
    {

        USART_ClearITPendingBit(MODBUS_SERIAL_PORT, USART_IT_RXNE);
        prvvUARTRxISR();
    }

    if (USART_GetITStatus(MODBUS_SERIAL_PORT, USART_IT_ORE) == SET)
    {

        USART_ClearITPendingBit(MODBUS_SERIAL_PORT, USART_IT_ORE);
        prvvUARTRxISR();
    }

    /*send irq*/
    if (USART_GetITStatus(MODBUS_SERIAL_PORT, USART_IT_TC) == SET)
    {
        USART_ClearITPendingBit(MODBUS_SERIAL_PORT, USART_IT_TC);
        prvvUARTTxReadyISR();
    }

    OSIntExit();
}
