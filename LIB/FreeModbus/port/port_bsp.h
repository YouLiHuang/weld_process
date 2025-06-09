#ifndef PORTBSP_H
#define PORTBSP_H

#include "includes.h"
#include "sys.h"

#define MODBUS_TIMER TIM12
#define MODBUS_SERIAL_PORT USART3
#define MODBUS_BOUNDS 115200

void modbus_serial_init(uint32_t bound);
void modbus_timer_init(uint32_t period);
void MODBUS_UART_SEND(const char *buffer, const uint16_t len);

#endif
