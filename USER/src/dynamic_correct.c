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

#define MIN_FAST_RISE_TIME 100

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

/*Find the first target point that is closest to a given value*/
static int16_t findValue(uint16_t arr[], uint16_t size, uint16_t val)
{
    uint16_t index = 0;

    while (index < size - 1)
    {
        if (((int16_t)arr[index] - (int16_t)val) * ((int16_t)arr[index + 1] - (int16_t)val) < 0)

            return index;
        else
            index++;
    }

    return -1;
}

/*测试版本，先考虑一段加热*/
void dynamic_rise_correct(float target)
{

    int16_t max_temp = findMax(temp_draw_ctrl->temp_buf, sizeof(temp_draw_ctrl->temp_buf) / sizeof(uint16_t));

    /*The time point corresponding to reaching the target temperature*/
    int16_t target_index = findValue(temp_draw_ctrl->temp_buf, sizeof(temp_draw_ctrl->temp_buf) / sizeof(uint16_t), target);
    int16_t max_temp_index = findValue(temp_draw_ctrl->temp_buf, sizeof(temp_draw_ctrl->temp_buf) / sizeof(uint16_t), max_temp);
    if (target_index != -1 && target_index < max_temp_index)
    {
        double percent = (float)target / (float)max_temp;
        if (percent > 1.05 || percent < 0.95)
        {

            uint16_t target_rise_time = target_index * temp_draw_ctrl->sample_freq;
            uint16_t max_rise_time = max_temp_index * temp_draw_ctrl->sample_freq;

            /*------------------------------------dynamic correct------------------------------------*/
            /*Try to dynamically reduce the time first, then try to dynamically reduce the duty cycle*/
            if (weld_controller->fast_rise_duty <= weld_controller->final_duty)
            {
                printf("...user need to change gain...\n");
            }
            else
            {
                /*Indicates that the duty cycle is too large*/
                if (weld_controller->fast_rise_time == MIN_FAST_RISE_TIME)
                {
                    weld_controller->fast_rise_duty = weld_controller->fast_rise_duty * target_rise_time / max_rise_time;
                    if (weld_controller->fast_rise_duty < weld_controller->final_duty)
                        weld_controller->fast_rise_duty = weld_controller->final_duty;
                }
                /*Try reducing the heating time*/
                else
                {
                    weld_controller->fast_rise_time = weld_controller->fast_rise_time * target_rise_time / max_rise_time;
                    if (weld_controller->fast_rise_time < MIN_FAST_RISE_TIME)
                        weld_controller->fast_rise_time = MIN_FAST_RISE_TIME;
                }
            }


        }
    }

    memset(temp_draw_ctrl->temp_buf, 0, sizeof(temp_draw_ctrl->temp_buf) / sizeof(uint16_t));
}
