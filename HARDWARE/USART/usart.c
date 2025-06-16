
#include "sys.h"
#include "usart.h"

#include "protect.h"
#include "touchscreen.h"
#include "welding_process.h"

#include "touch_screen_app.h"
//////////////////////////////////////////////////////////////////////////////////
// 如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos 使用
#endif

uint8_t USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.
extern uint8_t ID_OF_DEVICE;		 // 焊机485通讯机号，默认是零

/*信号量接口*/
extern OS_SEM PAGE_UPDATE_SEM;
extern OS_SEM COMP_VAL_GET_SEM;
extern OS_SEM COMP_STR_GET_SEM;
extern OS_SEM ALARM_RESET_SEM;
extern OS_SEM SENSOR_UPDATE_SEM; // 热电偶校准信号

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
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
				// OSSemPost(&PAGE_UPDATE_SEM, OS_OPT_POST_ALL, &err);
				break;
			case CMD_PAGEID_RETURN:
				OSSemPost(&PAGE_UPDATE_SEM, OS_OPT_POST_ALL, &err);
				request_PGManger()->id = (Page_ID)USART_RX_BUF[1];
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
