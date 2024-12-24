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

#define TEMP_GAIN1 0.1701
#define TEMP_GAIN2 -3.0

/*NEW API*/
void ADC_DMA_INIT(void);
uint16_t ADC_Value_avg(uint16_t channel);


void Adc_Init(void);
u16 Get_Adc1(u8 ch1);
u16 Get_Adc1_Average(u8 ch1, u16 times1);
u16 Bubble_Sort_Calculate(u8 ch1, u16 n);
u16 Bubble_Calculate_lan(u16 arr[], u16 n);
float rms_get(u16 *arr, u16 n, u16 start);
u16 temp_display(u16 *input, u16 nums, u16 start);

#endif
