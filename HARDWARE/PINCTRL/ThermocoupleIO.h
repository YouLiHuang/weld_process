#ifndef _THERIO_H
#define _THERIO_H

#include "sys.h"
#include "stdbool.h"
#include "includes.h"

// Marco of check io define

#define CHECK_RCC_E RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_E GPIOE
#define CHECKOUT_PIN_E GPIO_Pin_13
#define CHECKIN_PIN_E GPIO_Pin_14

#define CHECK_RCC_J RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_J GPIOE
#define CHECKOUT_PIN_J GPIO_Pin_6
#define CHECKIN_PIN_J GPIO_Pin_7

#define CHECK_RCC_K RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_K GPIOE
#define CHECKOUT_PIN_K GPIO_Pin_3
#define CHECKIN_PIN_K GPIO_Pin_4

#define DISCONNECT_E (0X01 << 1)
#define DISCONNECT_J (0X01 << 2)
#define DISCONNECT_K (0X01 << 3)

void Check_IO_init(void);


#endif
