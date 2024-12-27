/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-20 10:12:12
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-20 10:14:07
 * @Description
 * @
 * @Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */

#ifndef __WELDING_PROCESS_H
#define __WELDING_PROCESS_H

#include "sys.h"
#include "PID.h"

void user_value_convert_to_string(char *buffer, const u8 buf_len, const u16 value);
void welding_process(void);

/*调试接口*/
#define PID_DEBUG 0             // pid调试模式
#define COMMUNICATE 0           // 上位机通信接口
#define REALTIME_TEMP_DISPLAY 1 // 实时温度绘制开关

/*温升控制*/
#define STABLE_ERR 25           // 稳态误差补偿
#define USER_SET_MAX_TEMP 650.0 // 允许用户设定的最大温度
#define USER_SET_MIN_TEMP 200.0 // 允许用户设定的最小温度
#define MAX_WELD_TIME 9999      // 最长焊接用时

/*
 *焊接状态
 */
typedef enum WELD_STATE
{
    IDEAL_STATE = 0,
    PRE_STATE = 1,
    FIRST_STATE = 2,
    SECOND_STATE = 3,
    THIRD_STATE = 4
} WELD_STATE;

typedef enum WELD_MODE
{
    BUSY_MODE = 0,
    IDEAL_MODE
} WELD_MODE;

typedef struct temp_setting_point
{
    u16 time;
    u16 temp;
} temp_setting_point;

typedef struct weld_realtime_controller
{
    u16 weld_time_tick; /*total time tick*/
    u16 step_time_tick; /*every step tick*/
    u16 Duty_Cycle;     /*Duty Cycle*/
    u16 weld_count;     /*weld count*/
    WELD_STATE state;   /*wled state*/

    /*Transition Parameters*/
    u16 first_step_start_temp;
    u16 second_step_start_temp;
    u16 third_step_start_temp;
    u16 third_step_start_duty_cycle;
    u16 realtime_temp;

    /*user parameter*/
    u16 weld_time[5];
    u16 weld_temp[3];
    u16 alarm_temp[6];

    /*ctrl param*/
    u16 first_step_set;
    u16 first_step_turn;

    u16 second_step_set;
    u16 second_step_turn;
    /*gain param*/
    double temp_gain1;
    double temp_gain2;
    double temp_gain3;

    /*pid ctrl*/
    pid_feedforword_ctrl *pid_ctrl;
} weld_ctrl;

weld_ctrl *new_weld_ctrl(pid_feedforword_ctrl *pid_ctrl);

u8 get_weld_flag(void);

#endif
