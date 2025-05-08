/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-04-22 16:55:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-04-24 16:58:03
 * @Description:
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

#ifndef __ADC_H
#define __ADC_H
#include "sys.h"
#include "touchscreen.h"

#define ADC_SAMPLE_LIMIT 3250 // 温度传感器采样电压阈值
#define ADC_SAMPLE_PNUM 15    // AD 采样点数数
#define ADC_SAMPLE_CNUM 6     // AD 采样通道数

#define THERMOCOUPLE_CHANNEL_K ADC_Channel_7
#define THERMOCOUPLE_CHANNEL_E ADC_Channel_14
#define THERMOCOUPLE_CHANNEL_J ADC_Channel_15

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
void Sample_Buffer_clear(void);
void ADC_DMA_INIT(void);
uint16_t ADC_Value_avg(uint16_t channel);
uint16_t temp_convert(Thermocouple *thermocouple);

/*API*/
uint16_t Bubble_Calculate(uint16_t arr[], uint16_t n);
float rms_get(uint16_t *arr, uint16_t n, uint16_t start);

#endif
