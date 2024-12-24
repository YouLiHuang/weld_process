#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "stdbool.h"

#define COMPENSATION 0

#define WIN_WIDTH 560        // 显示区域宽度
#define MAX_TEMP_DISPLAY 800 // 最大能显示的温度

void TIM3_Int_Init(void);
void TIM5_Int_Init(void);
void TIM2_Int_Init(void);
void user_timer_handle(void);

#define TEMP_BUF_MAX_LEN 3000
typedef struct Temp_draw_controller
{
    u16 *temp_buf;
    u16 current_index;
    u16 buf_len_max;

    u16 first_step_index_start;
    u16 first_step_index_end;

    u16 second_step_index_start;
    u16 second_step_index_end;
    u16 second_step_stable_start;
    u16 second_step_stable_end;

    u16 third_step_index_start;
    u16 third_step_index_end;

    u8 sample_freq;
    u16 tick_record;

    /*绘图间隔*/
    u8 delta_tick;
} Temp_draw_ctrl;
Temp_draw_ctrl *new_temp_draw_ctrl(u16 *buf, u16 len1, u16 len2, u16 len3);
void reset_temp_draw_ctrl(Temp_draw_ctrl *ctrl, const u16 welding_time[]);

void pid_param_dynamic_reload(void *controller, double *fitting_curves, u16 setting);

/*可将该api进一步封装为，支持回调函数作为参数*/
void user_tim_delay(u16 time_ms);
u16 tim2_cnt_get(void);
void tim2_cnt_set(u16 val);

#endif
