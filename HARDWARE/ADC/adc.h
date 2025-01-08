/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-29 15:38:50
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-30 20:36:47
 * @Description:
 * @
 * @Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-05 10:38:29
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-05 10:10:40
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
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
    SENSOR_TYPE type;
    float slope;
    float intercept;

} Thermocouple;
Thermocouple *newThermocouple(SENSOR_TYPE type, float slope, float intercept);


/*NEW API*/
void ADC_DMA_INIT(void);
uint16_t ADC_Value_avg(uint16_t channel);



u16 Bubble_Calculate_lan(u16 arr[], u16 n);
float rms_get(u16 *arr, u16 n, u16 start);


#endif
