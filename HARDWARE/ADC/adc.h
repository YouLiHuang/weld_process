/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-11 15:47:16
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-01-12 17:36:21
 * @Description:
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

#ifndef __ADC_H
#define __ADC_H
#include "sys.h"
#include "touchscreen.h"

#define TEMP_GAIN1 0.17
#define TEMP_GAIN2 0
#define TYPE_NUMBER 3
/**/
typedef struct Thermocouple
{
    SENSOR_TYPE type; /*热点偶类型*/
    float slope;      /*斜率*/
    float intercept;  /*截距*/
    uint16_t Bias;    /*初始偏置*/

} Thermocouple;
Thermocouple *newThermocouple(SENSOR_TYPE type, float slope, float intercept);

/*NEW API*/
void ADC_DMA_INIT(void);

uint16_t ADC_Value_avg(uint16_t channel);
uint16_t temp_convert(Thermocouple *thermocouple);

uint16_t Bubble_Calculate_lan(uint16_t arr[], uint16_t n);
float rms_get(uint16_t *arr, uint16_t n, uint16_t start);

#endif
