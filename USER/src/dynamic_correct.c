/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-03-25 10:31:52
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-05-08 10:56:59
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "dynamic_correct.h"
#include "welding_process.h"
#include "timer.h"
#include "usart.h"
#include "filter.h"
#include "string.h"

#define STORAGE_DEPTH 500
#define MAX_CORRECT_GAIN 1.25f
#define MIN_CORRECT_GAIN 0.75f

uint16_t PWM_Record[STORAGE_DEPTH];
float PWM_Filter_Buf[STORAGE_DEPTH];
uint16_t record_index = 0;
uint16_t record_cnt = 0;
uint16_t Stable_Threshold_cnt = 0;

// Drawing controllers
extern Temp_draw_ctrl *temp_draw_ctrl;
// Welding real-time controller
extern weld_ctrl *weld_controller;
// Curve Correction
extern Steady_state_coefficient steady_coefficient; // Steady-state fitting curve coefficient

#if 0
static int16_t findMax(uint16_t arr[], uint16_t size)
{
    if (size <= 0)
        return -1;

    uint16_t max = arr[0];
    for (uint16_t i = 1; i < size; i++)
    {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}

static int16_t findMin(uint16_t arr[], uint16_t size)
{
    if (size <= 0)
        return -1;

    uint16_t min = arr[0];
    for (uint16_t i = 1; i < size; i++)
    {
        if (arr[i] < min)
            min = arr[i];
    }
    return min;
}
#endif

/*Find the first target point that is closest to a given value*/
static int16_t findValue(uint16_t arr[], uint16_t size, uint16_t val)
{
    uint16_t index = 0;

    while (index < size - 1)
    {
        if (arr[index] == val)
            return index;
        if (((int16_t)arr[index] - (int16_t)val) * ((int16_t)arr[index + 1] - (int16_t)val) < 0)
            return index;
        else
            index++;
    }

    return -1;
}

/*测试版本，先考虑一段加热*/
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN];
#define LEARNING_RATE 3
#define MIN_STEP_SIZE 0.01f

void dynamic_param_adjust(void)
{
    uint16_t step_end_time = temp_draw_ctrl->second_step_index_end;
    /*分析有温度曲线几个极值点*/
    int16_t left_err = 0;
    int16_t right_err = 0;

    /*Search starting point*/
    uint16_t index = findValue(realtime_temp_buf, TEMP_BUF_MAX_LEN, weld_controller->weld_temp[1] * 0.95);

    /*The data can be exported later for filtering algorithm testing...*/

    uint8_t point_index = 0;
    point point_arr[30];

    /*Find the extreme point*/
    while (index < step_end_time - 10)
    {
        left_err = realtime_temp_buf[index + 5] - realtime_temp_buf[index];
        right_err = realtime_temp_buf[index + 10] - realtime_temp_buf[index + 5];
        if (left_err * right_err < 0)
        {
            /*Record the maximum value point*/
            if (point_index < 30)
            {
                point_arr[point_index].index = index + 1;
                point_arr[point_index].val = temp_draw_ctrl->temp_buf[index + 1];
                point_index++;
            }
        }

        index++;
    }

    /*Find the maximum and minimum values ​​among these extreme points*/
    uint8_t i = 0;
    uint16_t min = point_arr[0].val;
    uint16_t max = point_arr[0].val;
    while (i < point_index - 1)
    {
        if (point_arr[i].val > max)
        {
            max = point_arr[i].val;
        }
        if (point_arr[i].val < min)
        {
            min = point_arr[i].val;
        }
        i++;
    }

    /*Overshoot analysis --> Dynamically adjust parameters*/
    if ((float)max / (float)weld_controller->weld_temp[1] > 1.05f)
    {
        if (weld_controller->temp_gain2 >= 1 * MIN_STEP_SIZE)
            weld_controller->temp_gain2 -= LEARNING_RATE * MIN_STEP_SIZE;
        else if (weld_controller->temp_gain1 >= 1 * MIN_STEP_SIZE)
            weld_controller->temp_gain1 -= LEARNING_RATE * MIN_STEP_SIZE;
    }
    else if ((float)min / (float)weld_controller->weld_temp[1] < 0.95f)
    {
        if (weld_controller->temp_gain2 <= 1)
            weld_controller->temp_gain2 += LEARNING_RATE * MIN_STEP_SIZE;
        else if (weld_controller->temp_gain1 <= 1)
            weld_controller->temp_gain1 += LEARNING_RATE * MIN_STEP_SIZE;
    }
    else
    {
        /*效果良好——>尝试减小热补偿时间，提高响应速度*/
        // if (weld_controller->temp_gain1 >= 0.01)
        //     weld_controller->temp_gain1 -= LEARNING_RATE * 0.01;
    }

    /*修正热量补偿曲线*/
    uint32_t sum = 0;
    uint16_t Final_PWM = 0;
    if (record_cnt < STORAGE_DEPTH)
    {
        low_pass_Filter((float *)PWM_Record, record_cnt, PWM_Filter_Buf, 1000, 1000);
        /*Calculate the average*/
        for (uint16_t i = 0; i < record_cnt; i++)
        {
            sum += PWM_Filter_Buf[i];
        }
        sum /= record_cnt;
    }
    else
    {
        low_pass_Filter((float *)PWM_Record, STORAGE_DEPTH, PWM_Filter_Buf, 1000, 1000);
        /*Calculate the average*/
        for (uint16_t i = 0; i < STORAGE_DEPTH; i++)
        {
            sum += PWM_Filter_Buf[i];
        }
        Final_PWM = sum / STORAGE_DEPTH;
    }

    /*Original duty cycle*/
    weld_controller->final_duty = steady_coefficient.slope * weld_controller->weld_temp[1] + steady_coefficient.intercept;
    /*Curve Correction*/
    float Proportion = (float)Final_PWM / (float)weld_controller->final_duty;
    if (Proportion < MIN_CORRECT_GAIN)
    {
        Proportion = MIN_CORRECT_GAIN;
    }
    else if (Proportion > MAX_CORRECT_GAIN)
    {
        Proportion = MAX_CORRECT_GAIN;
    }

    steady_coefficient.slope *= Proportion;
    steady_coefficient.intercept *= Proportion;

    /*Sampling reset*/
    record_index = 0;
    record_cnt = 0;
    Stable_Threshold_cnt = 0;
}
