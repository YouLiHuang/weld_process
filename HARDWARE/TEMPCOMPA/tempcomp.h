/*
 * @Author: <lan> <1791060296@qq.com>
 * @Date: 2024-06-26 09:09:37
 * @LastEditors: <lan> <1791060296@qq.com>
 * @LastEditTime: 2024-06-26 09:09:49
 * @FilePath: \��ѹ��������_�ȵ�ż����20240626\HARDWARE\TEMPCOMPA\tempcomp.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __TEMPCOMP_H
#define __TEMPCOMP_H
#include "sys.h"


typedef struct
{
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
} dynamical_comp;


typedef struct
{
    u16 temp_0;
    u16 temp_1;
    u16 temp_2;
    u16 temp_3;
    u16 temp_4;

} last_temp_sotre;

void dynamic_init(dynamical_comp *comp, float const_time);
void window_init(last_temp_sotre *temp);
void slid_windows(last_temp_sotre *temp, u16 now_temp);
u16 dynamic_temp_comp(last_temp_sotre temp, dynamical_comp comp);

#endif 
