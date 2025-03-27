/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-03-25 10:31:52
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-03-25 11:53:27
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "dynamic_correct.h"
#include "welding_process.h"
#include "timer.h"
#include "usart.h"
#include "string.h"

best_param best = {0, 0, 0};

// Drawing controllers
extern Temp_draw_ctrl *temp_draw_ctrl;
// Welding real-time controller
extern weld_ctrl *weld_controller;

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

static double cur_over_temp;
static double last_over_temp;
/*测试版本，先考虑一段加热*/
void dynamic_rise_correct(float target)
{

    /*The time point corresponding to reaching the target temperature*/
    uint16_t fast_rise_temp = weld_controller->final_temp_record;
    double percent = (float)fast_rise_temp / (float)target;
    /*------reduce heat input------*/
    if (percent >= 0.95)
    {
        if (weld_controller->fast_rise_time > MIN_FAST_RISE_TIME)
        {
            uint16_t target_index = findValue(temp_draw_ctrl->temp_buf, TEMP_BUF_MAX_LEN, target);
            uint16_t target_time = target_index * temp_draw_ctrl->sample_freq;

            /*Try reducing the heating time*/
            if (target_time < weld_controller->fast_rise_time * target / fast_rise_temp)
            {
                weld_controller->fast_rise_time = target_time;
            }
            else
            {
                weld_controller->fast_rise_time = weld_controller->fast_rise_time * target / fast_rise_temp;
            }

            if (weld_controller->fast_rise_time < MIN_FAST_RISE_TIME)
                weld_controller->fast_rise_time = MIN_FAST_RISE_TIME;
        }
        /*Indicates that the duty cycle is too large*/
        if (weld_controller->fast_rise_time == MIN_FAST_RISE_TIME)
        {
            /*try to reduce duty*/
            weld_controller->fast_rise_duty = weld_controller->fast_rise_duty * target / fast_rise_temp;
            if (weld_controller->fast_rise_duty < weld_controller->final_duty)
                weld_controller->fast_rise_duty = weld_controller->final_duty;
        }
    }
    /*------increase heat input------*/
    else if (percent < 0.8)
    {

        /*Indicates that time is too short*/
        if (weld_controller->fast_rise_duty >= MAX_RISE_STEP_DUTY)
        {
            /*try to increase time*/
            weld_controller->fast_rise_duty = MAX_RISE_STEP_DUTY;
            weld_controller->fast_rise_time = weld_controller->fast_rise_time * target / fast_rise_temp;
            if (weld_controller->fast_rise_time > MAX_FAST_RISE_TIME)
                weld_controller->fast_rise_time = MAX_FAST_RISE_TIME;
        }
        else
        {
            /*try to increase duty*/
            weld_controller->fast_rise_duty = weld_controller->fast_rise_duty * target / fast_rise_temp;
            if (weld_controller->fast_rise_duty > MAX_RISE_STEP_DUTY)
                weld_controller->fast_rise_duty = MAX_RISE_STEP_DUTY;
        }
    }
    else
    {
        /*analysis of output*/
        float max_temp = findMax(temp_draw_ctrl->temp_buf, TEMP_BUF_MAX_LEN);
        cur_over_temp = (float)(max_temp - target) / target;
        if (cur_over_temp > 0.05)
        {
            if (weld_controller->fast_rise_time > MIN_FAST_RISE_TIME)
            {

                /*Try reducing the heating time*/
                weld_controller->fast_rise_time = weld_controller->fast_rise_time * (1 - cur_over_temp);
                if (weld_controller->fast_rise_time < MIN_FAST_RISE_TIME)
                    weld_controller->fast_rise_time = MIN_FAST_RISE_TIME;
            }
            /*Indicates that the duty cycle is too large*/
            if (weld_controller->fast_rise_time == MIN_FAST_RISE_TIME)
            {
                /*try to reduce duty*/
                weld_controller->fast_rise_duty = weld_controller->fast_rise_duty * (1 - cur_over_temp);
                if (weld_controller->fast_rise_duty < weld_controller->final_duty)
                    weld_controller->fast_rise_duty = weld_controller->final_duty;
            }
        }
        if (cur_over_temp < last_over_temp)
        {

            if (max_temp < 1.01 * (float)target && max_temp > 0.99 * (float)target)
            {
                /*update best param*/
                best.fast_rise_time = weld_controller->fast_rise_time;
                best.fast_rise_duty = weld_controller->fast_rise_duty;
                best.start_temp = weld_controller->second_step_start_temp;
            }
        }

        last_over_temp = cur_over_temp;
    }

    memset(temp_draw_ctrl->temp_buf, 0, sizeof(temp_draw_ctrl->temp_buf) / sizeof(uint16_t));
}

extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN];
#define LEARNING_RATE 3
void dynamic_param_adjust(void)
{
    uint16_t step_start_time = temp_draw_ctrl->second_step_index_start;
    uint16_t step_end_time = temp_draw_ctrl->second_step_index_end;
    /*分析有温度曲线几个极值点*/
    int16_t left_err = 0;
    int16_t right_err = 0;

    uint16_t index = findValue(realtime_temp_buf, TEMP_BUF_MAX_LEN, weld_controller->weld_temp[1] * 0.95);

    uint8_t point_index = 0;
    point point_arr[30];

    while (index < step_end_time - 10)
    {
        left_err = realtime_temp_buf[index + 5] - realtime_temp_buf[index];
        right_err = realtime_temp_buf[index + 10] - realtime_temp_buf[index + 5];
        if (left_err * right_err < 0)
        {
            if (point_index < 30)
            {
                point_arr[point_index].index = index + 1;
                point_arr[point_index].val = temp_draw_ctrl->temp_buf[index + 1];
                point_index++;
            }
        }

        index++;
    }

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

    if ((float)max / (float)weld_controller->weld_temp[1] > 1.05)
    {
        if (weld_controller->temp_gain2 >= 0.01)
            weld_controller->temp_gain2 -= LEARNING_RATE * 0.01;
        else if (weld_controller->temp_gain1 >= 0.01)
            weld_controller->temp_gain1 -= LEARNING_RATE * 0.01;
    }
    else if ((float)min / (float)weld_controller->weld_temp[1] < 0.95)
    {
        if (weld_controller->temp_gain2 <= 1)
            weld_controller->temp_gain2 += LEARNING_RATE * 0.01;
        else if (weld_controller->temp_gain1 <= 1)
            weld_controller->temp_gain1 += LEARNING_RATE * 0.01;
    }
    else
    {
        /*效果良好——>尝试减小热补偿时间，提高响应速度*/
        if (weld_controller->temp_gain1 >= 0.01)
            weld_controller->temp_gain1 -= LEARNING_RATE * 0.01;
    }
}
