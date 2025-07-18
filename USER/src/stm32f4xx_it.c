/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.4.0
 * @date    04-August-2014
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/* user indludes--------------------------------------------------------------*/
#include "protect.h"
#include "modbus_bsp.h"
#include "usb_bsp.h"
#include "usb_hcd_int.h"
#include "usbh_core.h"

/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void modbus_timer_irq(void);
extern void modbus_serial_irq(void);
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
  TIM_SetCompare1(TIM1, 0);
  TIM_SetCompare1(TIM4, 0);

  TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
  TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
  TIM_Cmd(TIM1, DISABLE);
  TIM_Cmd(TIM4, DISABLE);
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare1(TIM4, 0);

    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
    TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
    TIM_Cmd(TIM1, DISABLE);
    TIM_Cmd(TIM4, DISABLE);
  }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare1(TIM4, 0);

    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
    TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
    TIM_Cmd(TIM1, DISABLE);
    TIM_Cmd(TIM4, DISABLE);
  }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare1(TIM4, 0);

    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
    TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
    TIM_Cmd(TIM1, DISABLE);
    TIM_Cmd(TIM4, DISABLE);
  }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare1(TIM4, 0);

    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
    TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
    TIM_Cmd(TIM1, DISABLE);
    TIM_Cmd(TIM4, DISABLE);
  }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
  TIM_SetCompare1(TIM1, 0);
  TIM_SetCompare1(TIM4, 0);

  TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
  TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
  TIM_Cmd(TIM1, DISABLE);
  TIM_Cmd(TIM4, DISABLE);
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
  TIM_SetCompare1(TIM1, 0);
  TIM_SetCompare1(TIM4, 0);

  TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);
  TIM_ForcedOC1Config(TIM4, TIM_ForcedAction_InActive);
  TIM_Cmd(TIM1, DISABLE);
  TIM_Cmd(TIM4, DISABLE);
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
// void PendSV_Handler(void)
//{
// }

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
// void SysTick_Handler(void)
//{
//
// }

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
extern USB_OTG_CORE_HANDLE USB_OTG_Core;
extern USBH_HOST USB_Host;
extern void USB_OTG_BSP_TimerIRQ(void);
extern void TIM6_irq(void);

/**
 * @brief  OTG_FS_IRQHandler
 *          This function handles USB-On-The-Go FS global interrupt request.
 *          requests.
 * @param  None
 * @retval None
 */
#ifdef USE_USB_OTG_FS
void OTG_FS_IRQHandler(void)
#else
void OTG_HS_IRQHandler(void)
#endif
{
  USBH_OTG_ISR_Handler(&USB_OTG_Core);
}

/**
 * @brief  TIM2_IRQHandler
 *         This function handles Timer2 Handler.
 * @param  None
 * @retval None
 */
void TIM2_IRQHandler(void)
{
  USB_OTG_BSP_TimerIRQ();
}



void TIM6_DAC_IRQHandler(void)
{
  TIM6_irq();
}

/**
 * @description: TIM12_IRQHandler modbus timer
 * @return {*}
 */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  modbus_timer_irq();
}

/**
 * @description: modbus serial irq
 * @return {*}
 */
void USART3_IRQHandler(void)
{
  modbus_serial_irq(); // UART4_IRQHandler
}

extern void Start_signal_irq(void);
void EXTI0_IRQHandler(void)
{
  Start_signal_irq();
}

void EXTI1_IRQHandler(void)
{
  Start_signal_irq();
}

void EXTI2_IRQHandler(void)
{
  Start_signal_irq();
}
void EXTI3_IRQHandler(void)
{
  Start_signal_irq();
}

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
