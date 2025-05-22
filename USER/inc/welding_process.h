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
#include "pid.h"

/*state*/
typedef enum WELD_STATE
{
    IDEAL_STATE = 0,
    PRE_LOAD,
    PRE_STATE,
    FIRST_STATE,
    SECOND_STATE,
    THIRD_STATE
} WELD_STATE;

typedef enum COUNT_DIR
{
    UP = 0,
    DOWN,

} COUNT_DIR;

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

typedef struct Correction_factor
{
    float base;
    float amplitude;
} Correction_factor;

typedef struct Steady_state_coefficient
{
    float slope;
    float intercept;

} Steady_state_coefficient;

typedef struct pid_fitting_curve
{
    float a;
    float b;
    float c;
} pid_fitting_curve;

typedef struct weld_realtime_controller
{
    uint16_t weld_time_tick; /*total time tick*/
    uint16_t step_time_tick; /*every step tick*/
    uint16_t Duty_Cycle;     /*Duty Cycle*/
    uint16_t weld_count;     /*weld count*/
    WELD_STATE state;        /*wled state*/
    COUNT_DIR Count_Dir;     /*count direction*/

    /*Transition Parameters*/
    uint16_t first_step_start_temp;
    uint16_t second_step_start_temp;
    uint16_t third_step_start_temp;
    uint16_t realtime_temp;

    /*user parameter*/
    uint16_t weld_time[5];
    uint16_t weld_temp[3];
    uint16_t alarm_temp[6];

    /*ctrl param*/
    uint16_t final_duty;

    /*Heat compensation*/
    uint16_t enter_transition_time;
    bool enter_transition_flag;
    /*gain param*/
    double temp_gain1;
    double temp_gain2;

    Steady_state_coefficient ss_coefficient;

    /*pid ctrl*/
    pid_feedforword_ctrl *pid_ctrl;
    /*hook*/
    void (*user_hook_callback)(void);
} weld_ctrl;

weld_ctrl *new_weld_ctrl(pid_feedforword_ctrl *pid_ctrl);
WELD_MODE get_weld_flag(void);

void pid_param_dynamic_reload(void *controller, pid_fitting_curve fitting_curves, uint16_t setting);
void user_value_convert_to_string(char *buffer, const uint8_t buf_len, const uint16_t value);
void welding_process(void);

#endif
