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
#define RLY10 PDout(0)  // 气阀1
#define RLY11 PDout(1)  // 气阀2
#define RLY12 PDout(2)  // 气阀3
#define OVER PDout(3)   // 结束信号
#define ERROR1 PDout(4) // 出错信号
#define CUNT PDout(5)   // 计数
#define RLY13 PDout(6)  // 交流接触器
#define TRAN1 PDout(7)  // 变压器（交流接触器）
/*AT25控制IO*/
#define CSN PBout(12) // AT25 cs
/*预留输出IO*/
#define OUT1 PDout(8)  // 输出1
#define OUT2 PDout(9)  // 输出2
#define OUT3 PDout(10) // 输出3
#define OUT4 PDout(11) // 输出4

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

uint8_t new_key_scan(void);

#endif
