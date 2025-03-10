/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-15 19:17:48
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-01-16 11:00:13
 * @Description:
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

#ifndef __WELDING_PROCESS_H
#define __WELDING_PROCESS_H

#include "sys.h"
#include "PID.h"


void user_value_convert_to_string(char *buffer, const uint8_t buf_len, const uint16_t value);
void welding_process(void);

/*调试接口*/
#define PID_DEBUG 0             // pid调试模式
#define COMMUNICATE 0           // 上位机通信接口
#define REALTIME_TEMP_DISPLAY 1 // 实时温度绘制开关

#define HOST_WELD_CTRL 1 // 上位机控制焊接

/*温升控制*/
#define STABLE_ERR 40           // 稳态误差补偿
#define BASE_TEMP 120           // 基值温度
#define PRE_HEAT_MAX_TIME 250   // 最大预热时长
#define PRE_HEAT_MIN_TIME 1     // 最小预热时长
#define USER_SET_MAX_TEMP 650.0 // 允许用户设定的最大温度
#define USER_SET_MIN_TEMP 200.0 // 允许用户设定的最小温度
#define MAX_WELD_TIME 9999      // 最长焊接用时
#define DELTA_COMPENSATE_MAX 250
#define DETTA_COMPENSATE_MIN 100
#define DEFAULT_GAIN 0.85

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
    uint16_t time;
    uint16_t temp;
} temp_setting_point;

typedef struct weld_realtime_controller
{
    uint16_t weld_time_tick; /*total time tick*/
    uint16_t step_time_tick; /*every step tick*/
    uint16_t Duty_Cycle;     /*Duty Cycle*/
    uint16_t weld_count;     /*weld count*/
    WELD_STATE state;        /*wled state*/

    /*Transition Parameters*/
    uint16_t first_step_start_temp;
    uint16_t second_step_start_temp;
    uint16_t third_step_start_temp;
    uint16_t third_step_start_duty_cycle;
    uint16_t realtime_temp;

    /*user parameter*/
    uint16_t weld_time[5];
    uint16_t weld_temp[3];
    uint16_t alarm_temp[6];

    /*ctrl param*/
    uint16_t first_step_set;
    uint16_t first_step_turn;

    uint16_t second_step_set;
    uint16_t second_step_turn;
    /*gain param*/
    double temp_gain1;
    double temp_gain2;
    double temp_gain3;

    /*pid ctrl*/
    pid_feedforword_ctrl *pid_ctrl;

    /*hook*/
    void (*user_hook_callback)(void);
} weld_ctrl;

weld_ctrl *new_weld_ctrl(pid_feedforword_ctrl *pid_ctrl);

WELD_MODE get_weld_flag(void);

#endif
