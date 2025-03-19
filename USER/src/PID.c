/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-21 09:52:41
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-03-19 09:10:00
 * @Description:pid control algorithm
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */

#include "PID.h"
#include "tempcomp.h"

pid_feedforword_ctrl *new_pid_forword_ctrl(float kp_f, float kp, float ki, float kd)
{
	pid_feedforword_ctrl *ctrl = (pid_feedforword_ctrl *)malloc(sizeof(pid_feedforword_ctrl));
	if (ctrl == NULL)
		return NULL;

	ctrl->kp_f = kp_f;
	ctrl->kp = kp;
	ctrl->ki = ki;
	ctrl->kd = kd;

	ctrl->delta = 0;
	ctrl->err = 0;
	ctrl->pre_err = 0;
	ctrl->pre_pre_err = 0;

	ctrl->stable_flag = false;
	ctrl->stable_threshold_cnt = 0;
	ctrl->stable_threshold = STABLE_THRESHOLD;

	return ctrl;
}

void reset_forword_ctrl(pid_feedforword_ctrl *ctrl)
{
	ctrl->delta = 0;
	ctrl->err = 0;
	ctrl->pre_err = 0;
	ctrl->pre_pre_err = 0;
	ctrl->stable_flag = false;

	ctrl->stable_threshold_cnt = 0;
	ctrl->stable_threshold = STABLE_THRESHOLD;
}

int PI_ctrl_output(int target, int feedback, int current_output, pid_feedforword_ctrl *ctrl)
{
	int new_output = 0;
	/*current error*/
	ctrl->err = target - feedback;
	/*pid delta*/
	ctrl->delta = ctrl->kp * (ctrl->err - ctrl->pre_err) +
				  ctrl->ki * ctrl->err +
				  ctrl->kd * (ctrl->err - 2 * ctrl->pre_err + ctrl->pre_pre_err);

	// if (feedback <= 1.01 * target && feedback >= 0.99 * target)
	// {
	// 	if (ctrl->delta > PD_MAX * 0.01)
	// 		ctrl->delta = PD_MAX * 0.01;
	// }

	/*forword  delta*/
	/*...*/
	/*update ouput*/
	new_output = current_output + ctrl->delta;
	/*record pre err*/
	ctrl->pre_pre_err = ctrl->pre_err;
	ctrl->pre_err = ctrl->err;

	/*limit output*/
	if (new_output >= PD_MAX)
		new_output = PD_MAX;
	if (new_output <= PD_MIN)
		new_output = PD_MIN;

	return new_output;
}
