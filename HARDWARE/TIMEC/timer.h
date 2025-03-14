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

#define COMPENSATION 0
#define KALMAN_FILTER 0

#define WIN_WIDTH 525        // 显示区域宽度
#define DRAW_RESERVE 10      // 绘图余量
#define DRAW_AREA_HIGH 210   // 绘图组件高度
#define MAX_TEMP_DISPLAY 780 // 最大能显示的温度

void TIM3_Int_Init(void);
void TIM5_Int_Init(void);
void TIM2_Int_Init(void);
void user_timer_handle(void);

#define TEMP_BUF_MAX_LEN 3000
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

void pid_param_dynamic_reload(void *controller, double *fitting_curves, uint16_t setting);

/*可将该api进一步封装为，支持回调函数作为参数*/
void user_tim_delay(uint16_t time_ms);
uint16_t tim2_cnt_get(void);

#endif
