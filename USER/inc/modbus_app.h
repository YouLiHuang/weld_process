/*** 
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-09 17:28:21
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-13 10:13:10
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
#define REG_HOLDING_NREGS     24

//线圈起始地址
#define REG_COILS_START       0x0000
//线圈数量
#define REG_COILS_SIZE        16

//开关寄存器起始地址
#define REG_DISCRETE_START    0x0000
//开关寄存器数量
#define REG_DISCRETE_SIZE     16



/* ----------------------- user Defines --------------------------------------*/
#define HOLD_ADDR_0 0
#define HOLD_ADDR_1 1
#define HOLD_ADDR_2 2
#define HOLD_ADDR_3 3
#define HOLD_ADDR_4 4
#define HOLD_ADDR_5 5
#define HOLD_ADDR_6 6
#define HOLD_ADDR_7 7
#define HOLD_ADDR_8 8
#define HOLD_ADDR_9 9
#define HOLD_ADDR_10 10
#define HOLD_ADDR_11 11
#define HOLD_ADDR_12 12
#define HOLD_ADDR_13 13
#define HOLD_ADDR_14 14
#define HOLD_ADDR_15 15
#define HOLD_ADDR_16 16
#define HOLD_ADDR_17 17
#define HOLD_ADDR_18 18
#define HOLD_ADDR_19 19
#define HOLD_ADDR_20 20


#define INPUT_ADDR_0 0
#define INPUT_ADDR_1 1
#define INPUT_ADDR_2 2
#define INPUT_ADDR_3 3
#define INPUT_ADDR_4 4
#define INPUT_ADDR_5 5
#define INPUT_ADDR_6 6
#define INPUT_ADDR_7 7


#define COIL_ADDR_0 0
#define COIL_ADDR_1 1
#define COIL_ADDR_2 2
#define COIL_ADDR_3 3
#define COIL_ADDR_4 4
#define COIL_ADDR_5 5
#define COIL_ADDR_6 6
#define COIL_ADDR_7 7

#define DISCRETE_ADDR_0 0
#define DISCRETE_ADDR_1 1
#define DISCRETE_ADDR_2 2
#define DISCRETE_ADDR_3 3
#define DISCRETE_ADDR_4 4
#define DISCRETE_ADDR_5 5
#define DISCRETE_ADDR_6 6
#define DISCRETE_ADDR_7 7
#define DISCRETE_ADDR_8 8
#define DISCRETE_ADDR_9 9
#define DISCRETE_ADDR_10 10
#define DISCRETE_ADDR_11 11


void Modbus_reg_sync(void);



#endif

