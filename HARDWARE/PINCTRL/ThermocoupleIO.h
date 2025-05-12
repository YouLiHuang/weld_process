/*** 
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-04-22 16:55:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-05-12 09:20:02
 * @Description: 
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved. 
 */
#ifndef _THERIO_H
#define _THERIO_H

#include "sys.h"
#include "stdbool.h"
#include "includes.h"

// Marco of check io define
// MCU board：E external board：K J
//#define CHECK_RCC_E RCC_AHB1Periph_GPIOE
//#define CHECK_GPIO_E GPIOE
//#define CHECKOUT_PIN_E GPIO_Pin_13
//#define CHECKIN_PIN_E GPIO_Pin_14

//#define CHECK_RCC_J RCC_AHB1Periph_GPIOE
//#define CHECK_GPIO_J GPIOE
//#define CHECKOUT_PIN_J GPIO_Pin_6
//#define CHECKIN_PIN_J GPIO_Pin_7

//#define CHECK_RCC_K RCC_AHB1Periph_GPIOE
//#define CHECK_GPIO_K GPIOE
//#define CHECKOUT_PIN_K GPIO_Pin_3
//#define CHECKIN_PIN_K GPIO_Pin_4

#define CHECK_RCC_E RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_E GPIOE
#define CHECKOUT_PIN_E GPIO_Pin_3
#define CHECKIN_PIN_E GPIO_Pin_4

#define CHECK_RCC_J RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_J GPIOE
#define CHECKOUT_PIN_J GPIO_Pin_6
#define CHECKIN_PIN_J GPIO_Pin_7

#define CHECK_RCC_K RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_K GPIOE
#define CHECKOUT_PIN_K GPIO_Pin_13
#define CHECKIN_PIN_K GPIO_Pin_14



void Check_IO_init(void);


#endif
