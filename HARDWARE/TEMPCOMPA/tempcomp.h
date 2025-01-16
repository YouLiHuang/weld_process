/*** 
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-15 19:17:48
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-01-16 10:58:15
 * @Description: 
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved. 
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
