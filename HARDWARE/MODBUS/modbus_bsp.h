#ifndef MODBUS_BSP_H
#define MODBUS_BSP_H
#include "timer.h"

#define MODBUS_TIMER TIM12

void modbus_timer_bsp_init(uint32_t usTim1Timerout50us);
void modbus_serial_bsp_init(uint32_t bound);

#endif
