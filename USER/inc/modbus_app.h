/*** 
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-09 17:28:21
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-10 19:30:09
 * @Description: 
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved. 
 */
#ifndef MODBUS_APP_H
#define MODBUS_APP_H

/* ----------------------- Defines ------------------------------------------*/
//输入寄存器起始地址
#define REG_INPUT_START       0x0000
//输入寄存器数量
#define REG_INPUT_NREGS       8
//保持寄存器起始地址
#define REG_HOLDING_START     0x0000
//保持寄存器数量
#define REG_HOLDING_NREGS     20

//线圈起始地址
#define REG_COILS_START       0x0000
//线圈数量
#define REG_COILS_SIZE        8

//开关寄存器起始地址
#define REG_DISCRETE_START    0x0000
//开关寄存器数量
#define REG_DISCRETE_SIZE     8




#endif

