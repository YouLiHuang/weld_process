/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-11 16:02:24
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-01-12 18:02:19
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "filter.h"

/**
 * @description:
 * @param {float} *input
 * @param {float} data_len
 * @param {float} freq_s : sample frequency
 * @param {float} freq_c ：cut-off frequency
 * @return {*}
 */
void low_pass_Filter(float *input, float data_len, float *output, float freq_s, float freq_c)
{
    double alpha = 1.0 / (1.0 + (freq_s / (2.0 * PI * freq_c)));

    output[0] = input[0]; // 初始化第一个输出值为输入值
    for (int i = 1; i < data_len; i++)
    {
        output[i] = alpha * input[i] + (1 - alpha) * output[i - 1];
    }
}

