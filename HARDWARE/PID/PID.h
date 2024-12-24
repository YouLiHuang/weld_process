#ifndef __PID_H
#define __PID_H
#include "sys.h"
#include "stdbool.h"
#include "stdlib.h"

/*PWM占空比上下限*/
#define PD_MAX 8000
#define PD_MIN 10

int INC_PID(u16 Set, u16 Sample, int now_current, float PROPORTION, float INTEGRAL, float Derivative);
int INC_PID_div(int Set, int Sample, int now_PDC, float PROPORTION, float INTEGRAL, float Derivative);
int INC_PID_div01(int Set, int Sample, int now_PDC, float PROPORTION, float INTEGRAL, float Derivative);
void pid_error_init(void);

typedef struct PID_Feedforword_ctrl
{
    bool stable_flag;

    float kp_f; // 前馈
    float kp;   // pid
    float ki;
    float kd;

    int err;
    int pre_err;
    int pre_pre_err;
    float delta;

} pid_feedforword_ctrl;

pid_feedforword_ctrl *new_pid_forword_ctrl(float kp_f, float kp, float ki, float kd);
int PI_ff_ctrl_output(int target, int feedback, int current_output, pid_feedforword_ctrl *ctrl);
int PI_ff_ctrl_output_stable(int target, int feedback, int current_output, pid_feedforword_ctrl *ctrl);
int PI_ff_ctrl_output_fast(int target, int feedback, int current_output, pid_feedforword_ctrl *ctrl);
int PI_ff_ctrl_output_slow(int target, int feedback, int current_output, pid_feedforword_ctrl *ctrl);

int PI_ctrl_output(int target, int feedback, int current_output, pid_feedforword_ctrl *ctrl);

void reset_forword_ctrl(pid_feedforword_ctrl *ctrl);

#endif
