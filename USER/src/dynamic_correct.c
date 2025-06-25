/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-03-25 10:31:52
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-20 14:23:44
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "user_config.h"
#include "dynamic_correct.h"
#include "touchscreen.h"
#include "touch_screen_app.h"
#include "welding_process.h"
#include "timer.h"
#include "usart.h"
#include "string.h"
#include "spi.h"

// last time set
static uint16_t first_step_last;
static uint16_t second_step_last;
// realtime temp
extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN];
// pwm dynamic correct
uint16_t PWM_Record[STORAGE_DEPTH];
uint16_t record_index = 0;
uint16_t record_cnt = 0;
uint16_t Stable_Threshold_cnt = 0;

// Drawing controllers
extern Temp_draw_ctrl *temp_draw_ctrl;
// Welding real-time controller
extern weld_ctrl *weld_controller;
// extern Component_Queue *list;

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
int16_t findValue(uint16_t *arr, uint16_t size, uint16_t val)
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

void dynamic_param_adjust(void)
{

    Component_Queue *list = get_page_list(TEMP_PAGE);
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
    // if ((float)max / (float)weld_controller->weld_temp[1] > OVERSHOOT_THRESHOLD)
    // {
    //     if (weld_controller->temp_gain2 > weld_controller->temp_gain1)
    //         weld_controller->temp_gain2 -= LEARNING_RATE * MIN_STEP_SIZE;
    //     else
    //         weld_controller->temp_gain1 -= LEARNING_RATE * MIN_STEP_SIZE;
    // }
    // else if ((float)min / (float)weld_controller->weld_temp[1] < REVERSE_OVERSHOOT_THRESHOLD)
    // {
    //     if (weld_controller->temp_gain2 < weld_controller->temp_gain1)
    //         weld_controller->temp_gain2 += LEARNING_RATE * MIN_STEP_SIZE;
    //     else
    //         weld_controller->temp_gain1 += LEARNING_RATE * MIN_STEP_SIZE;
    // }

    if ((float)max / (float)weld_controller->weld_temp[1] > OVERSHOOT_THRESHOLD)
    {
        weld_controller->temp_gain2 -= LEARNING_RATE * MIN_STEP_SIZE;
    }
    else if ((float)min / (float)weld_controller->weld_temp[1] < REVERSE_OVERSHOOT_THRESHOLD)
    {
        weld_controller->temp_gain2 += LEARNING_RATE * MIN_STEP_SIZE;
    }
    else
    {
        if (first_step_last != weld_controller->weld_temp[0] ||
            second_step_last != weld_controller->weld_temp[1])
        {
            /*save best gain*/
            SPI_Save_Word((uint16_t)weld_controller->temp_gain1 * 1000, GAIN_BASE(0) + ADDR_OFFSET * 0);
            SPI_Save_Word((uint16_t)weld_controller->temp_gain1 * 1000, GAIN_BASE(0) + ADDR_OFFSET * 1);
        }

        /*record last set*/
        first_step_last = weld_controller->weld_temp[0];
        second_step_last = weld_controller->weld_temp[1];
    }

    /*limit gain*/
    if (weld_controller->temp_gain1 < 0)
    {
        weld_controller->temp_gain1 = 0;
    }
    if (weld_controller->temp_gain2 < 0)
    {
        weld_controller->temp_gain2 = 0;
    }
    if (weld_controller->temp_gain1 > 1)
    {
        weld_controller->temp_gain1 = 1;
    }
    if (weld_controller->temp_gain2 > 1)
    {
        weld_controller->temp_gain2 = 1;
    }

    /*data sync*/
    if (get_comp(list, "GAIN1") != NULL)
    {
        get_comp(list, "GAIN1")->val = weld_controller->temp_gain1 * 1000;
    }
    if (get_comp(list, "GAIN2") != NULL)
    {
        get_comp(list, "GAIN2")->val = weld_controller->temp_gain2 * 1000;
    }
}

void dynamic_pwm_adjust(void)
{

    static uint16_t last_first_temp;
    static uint16_t last_second_temp;

    Steady_state_coefficient ss = weld_controller->ss_coefficient;
    /*修正热量补偿曲线*/
    uint32_t sum = 0;
    uint16_t Final_PWM = 0;
    if (record_cnt < STORAGE_DEPTH)
    {
        /*Calculate the average*/
        for (uint16_t i = 0; i < record_cnt; i++)
        {
            sum += PWM_Record[i];
        }
        Final_PWM = sum / record_cnt;
    }
    else
    {
        /*Calculate the average*/
        for (uint16_t i = 0; i < STORAGE_DEPTH; i++)
        {
            sum += PWM_Record[i];
        }
        Final_PWM = sum / STORAGE_DEPTH;
    }

    /*Original duty cycle*/
    uint16_t last_final_duty = ss.slope * weld_controller->weld_temp[1] + ss.intercept;
    /*Curve Correction*/
    float Proportion = (float)Final_PWM / (float)last_final_duty;
    if (Proportion < MIN_CORRECT_GAIN)
    {
        Proportion = MIN_CORRECT_GAIN;
    }
    else if (Proportion > MAX_CORRECT_GAIN)
    {
        Proportion = MAX_CORRECT_GAIN;
    }

    weld_controller->ss_coefficient.slope *= Proportion;
    weld_controller->ss_coefficient.intercept *= Proportion;

    /*save fit coefficient*/
    if (weld_controller->weld_temp[0] != last_first_temp || weld_controller->weld_temp[1] != last_second_temp)
    {
        SPI_Save_Word((uint16_t)ss.slope * 100, FIT_COEFFICIENT_BASE(0) + ADDR_OFFSET * 0);
        SPI_Save_Word((uint16_t)ss.intercept, FIT_COEFFICIENT_BASE(0) + ADDR_OFFSET * 1);

        last_first_temp = weld_controller->weld_temp[0];
        last_second_temp = weld_controller->weld_temp[1];
    }

    /*Sampling reset*/
    record_index = 0;
    record_cnt = 0;
    Stable_Threshold_cnt = 0;
}
