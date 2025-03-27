/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-03 18:51:41
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-01-03 19:51:43
 * @Description:
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "stdbool.h"

#define COMPENSATION 1
#define KALMAN_FILTER 0
#define TIME_CHECH 0 // 超时监测
#define PROTECT_ON 1 // 保护开关

#define WIN_WIDTH 525        // 显示区域宽度
#define DRAW_RESERVE 10      // 绘图余量
#define DRAW_AREA_HIGH 210   // 绘图组件高度
#define MAX_TEMP_DISPLAY 780 // 最大能显示的温度
#define TEMP_BUF_MAX_LEN 3000

void TIM3_Int_Init(void);
void TIM5_Int_Init(void);

typedef struct Temp_draw_controller
{
    uint16_t *temp_buf;
    uint16_t current_index;
    uint16_t buf_len_max;

    uint16_t first_step_index_start;
    uint16_t first_step_index_end;

    uint16_t second_step_index_start;
    uint16_t second_step_index_end;
    uint16_t second_step_stable_index;

    uint16_t third_step_index_start;
    uint16_t third_step_index_end;

    uint8_t sample_freq;
    uint16_t tick_record;

    /*绘图间隔*/
    uint8_t delta_tick;
} Temp_draw_ctrl;
Temp_draw_ctrl *new_temp_draw_ctrl(uint16_t *buf, uint16_t len1, uint16_t len2, uint16_t len3);
void reset_temp_draw_ctrl(Temp_draw_ctrl *ctrl, const uint16_t welding_time[]);

#endif
