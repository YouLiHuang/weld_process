/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-05-21 21:36:35
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-07 09:13:28
 * @Description:
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
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

    /*draw interval*/
    uint8_t delta_tick;

    /*display temp*/
    uint16_t display_temp[3];
} Temp_draw_ctrl;
Temp_draw_ctrl *new_temp_draw_ctrl(uint16_t *buf, uint16_t len1, uint16_t len2, uint16_t len3);
void reset_temp_draw_ctrl(Temp_draw_ctrl *ctrl, const uint16_t welding_time[]);

void TIM3_INIT(void);
void TIM5_INIT(void);

#endif
