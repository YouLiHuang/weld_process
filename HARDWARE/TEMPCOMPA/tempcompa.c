#include "tempcomp.h"

/**
 * @description: 补偿系数初始化
 * @param {dynamical_comp} *comp
 * @param {float} const_time
 * @return {*}
 */
void dynamic_init(dynamical_comp *comp, float const_time)
{
    comp->a0 = 1 + 0.05f * const_time;
    comp->a1 = 0.025f * const_time;
    comp->a2 = 0;
    comp->a3 = -0.025f * const_time;
    comp->a4 = -0.05f * const_time;
}

/**
 * @description: 温度窗口初始化
 * @param {last_temp_sotre} *temp
 * @return {*}
 */
void window_init(last_temp_sotre *temp)
{
    temp->temp_0 = 0;
    temp->temp_1 = 0;
    temp->temp_2 = 0;
    temp->temp_3 = 0;
    temp->temp_4 = 0;
}
/**
 * @description:  滑动窗口修改数值
 * @param {last_temp_sotre} *temp
 * @param {u16} now_temp
 * @return {*}
 */
void slid_windows(last_temp_sotre *temp, u16 now_temp)
{
    temp->temp_4 = temp->temp_3;
    temp->temp_3 = temp->temp_2;
    temp->temp_2 = temp->temp_1;
    temp->temp_1 = temp->temp_0;
    temp->temp_0 = now_temp;
}

/**
 * @description:  动态补偿
 * @param {last_temp_sotre} *temp
 * @param {dynamical_comp} *comp
 * @return {*}
 */
u16 dynamic_temp_comp(last_temp_sotre temp, dynamical_comp comp)
{
    float result = 0;
    result = (float)(temp.temp_0 * comp.a0) + (float)(temp.temp_1 * comp.a1) +
             (float)(temp.temp_3 * comp.a3) + (float)(temp.temp_4 * comp.a4);
    return (u16)result;
}
