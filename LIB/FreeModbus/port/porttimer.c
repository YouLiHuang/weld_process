#include "mb.h"
#include "mbport.h"
#include "port_bsp.h"
#include "timer.h"

/* Static variables */

BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
  modbus_timer_init(usTim1Timerout50us);
  return TRUE;
}

void vMBPortTimersEnable(void)
{
  /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  TIM_ClearITPendingBit(MODBUS_TIMER, TIM_IT_Update);
  TIM_ITConfig(MODBUS_TIMER, TIM_IT_Update, ENABLE);
  TIM_SetCounter(MODBUS_TIMER, 0x00000000);
  TIM_Cmd(MODBUS_TIMER, ENABLE);
}

void vMBPortTimersDisable(void)
{
  /* Disable any pending timers. */
  TIM_ClearITPendingBit(MODBUS_TIMER, TIM_IT_Update);
  TIM_ITConfig(MODBUS_TIMER, TIM_IT_Update, DISABLE);
  TIM_SetCounter(MODBUS_TIMER, 0x00000000);
  TIM_Cmd(MODBUS_TIMER, DISABLE);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR(void)
{
  (void)pxMBPortCBTimerExpired();
}

/**
 * @description: modbus timer irq
 * @return {*}
 */
void modbus_timer_irq(void)
{
#if SYSTEM_SUPPORT_OS
  OSIntEnter();
#endif
  if (TIM_GetITStatus(MODBUS_TIMER, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(MODBUS_TIMER, TIM_IT_Update);
    prvvTIMERExpiredISR();
  }
#if SYSTEM_SUPPORT_OS
  OSIntExit();
#endif
}
