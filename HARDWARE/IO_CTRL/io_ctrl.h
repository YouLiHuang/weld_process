/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-24 15:23:46
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-24 16:25:44
 * @Description:
 * @
 * @Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
#ifndef __KEY_H
#define __KEY_H
#include "sys.h"
#include "includes.h"

/*启动信号*/
#define KEY_PC0 PCin(0) // 启动1 脚踏开关1
#define KEY_PC1 PCin(1) // 启动2 脚踏开关2
#define KEY_PC2 PCin(2) // 启动3 外部启动信号
#define KEY_PC3 PCin(3) // 启动4 外部启动信号
/*各类外部IO*/
#define KEY_Res PCin(8)   // 复位按键
#define KEY_Ttem PCin(9)  // 整流管温度监测IO
#define KEY_Stem PCin(10) // 散热器监测IO
#define KEY_Wat PCin(11)  // 冷却水IO
/*预留外部IO*/
#define KEY_In3 PAin(6)
#define KEY_In4 PCin(7)

/*启动信号宏*/
#define KEY_PC0_PRES 1
#define KEY_PC1_PRES 2
#define KEY_PC2_PRES 3
#define KEY_PC3_PRES 4

/*预留IO宏*/
#define KEY_In3_PRES 7
#define KEY_In4_PRES 8
#define KEY_Res_PRES 9

/*输出IO*/
#define RLY_AIR0 PDout(0)      // Air valve0
#define RLY_AIR1 PDout(1)      // Air valv1
#define RLY_AIR2 PDout(2)      // Air valve2
#define RLY_OVER PDout(3)      // OVER
#define RLY_ERR PDout(4)       // ERROR
#define RLY_CNT PDout(5)       // CNT
#define RLY_CONTACTOR PDout(6) // contactor
#define RLY_TRAN PDout(7)      // transformer
#define RLY_RESERVE0 PDout(8)  // reserve0
#define RLY_RESERVE1 PDout(9)  // reserve1
#define RLY_RESERVE2 PDout(10) // reserve2
#define RLY_RESERVE3 PDout(11) // reserve3

#define RLY1_AIR0_READ PDin(0)     // Air valve0
#define RLY1_AIR1_READ PDin(1)     // Air valv1
#define RLY1_AIR2_READ PDin(2)     // Air valve2
#define RLY_OVER_READ PDin(3)      // OVER
#define RLY_ERR_READ PDin(4)       // ERROR
#define RLY_CNT_READ PDin(5)       // CNT
#define RLY_CONTACTOR_READ PDin(6) // contactor
#define RLY_TRAN_READ PDin(7)      // transformer
#define RLY_RESERVE0_READ PDin(8)  // reserve0
#define RLY_RESERVE1_READ PDin(9)  // reserve1
#define RLY_RESERVE2_READ PDin(10) // reserve2
#define RLY_RESERVE3_READ PDin(11) // reserve3

/*AT25控制IO*/
#define CSN PBout(12) // AT25 cs


typedef enum START_TYPE
{
    START_IDEAL = 0,
    KEY0,
    KEY1,
    START1,
    START2

} START_TYPE;

void INPUT_IO_INIT(void);
void OUT_Init(void);
void START_IO_INIT(void);

uint8_t key_scan(void);

#endif
