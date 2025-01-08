/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-01-08 09:51:17
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */

#include "adc.h"
#include "delay.h"
#include "includes.h"
#include "protect.h"

// PA4：output current
// PA5：output voltage
// PA6：voltage overflow check
// PA7：Thermocouple on board
#define ADC_SAMPLE_PNUM 15													// AD sampel ponit number for per channel
#define ADC_SAMPLE_CNUM 6													// AD channel number
volatile unsigned short m_ADCValue[ADC_SAMPLE_PNUM][ADC_SAMPLE_CNUM] = {0}; // buffer to save adc value

/**
 * @description: ADC DAM config ��continuous sampling mode and DMA loop mode for data transmission
 * @return {*}
 */
void ADC_DMA_INIT(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure2;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	/*GPIOA------------------------------------------------------------------------------------------- */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*GPIOC------------------------------------------------------------------------------------------- */
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure2);

	/* ADC Init----------------------------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = ADC_SAMPLE_CNUM;

	ADC_Init(ADC1, &ADC_InitStructure);

	/*ADC1 channel---------------------------------------------------------------------------------------*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 5, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 6, ADC_SampleTime_56Cycles);

	/*DMA config-----------------------------------------------------------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/*stream0 ch0----------------------------------------------------------------------------------------*/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (unsigned int)&m_ADCValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_PNUM * ADC_SAMPLE_CNUM;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure); 
	/*enable----------------------------------------------------------------------------------------------*/
	DMA_Cmd(DMA2_Stream0, ENABLE);				
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	/*ADC start with software---------------------------------------------------------------------------- */
	ADC_SoftwareStartConv(ADC1);
}

/**
 * @description:
 * @param {uint16_t} channel
 * @return {*}
 */
uint16_t ADC_Value_avg(uint16_t channel)
{
	uint32_t value = 0;
	switch (channel)
	{
	case ADC_Channel_14:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][0];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_15:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][1];
		}
		value /= ADC_SAMPLE_PNUM;
		break;

	case ADC_Channel_4:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][2];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_5:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][3];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_6:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][4];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_7:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][5];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	}

	return value;
}




Thermocouple *newThermocouple(SENSOR_TYPE type, float slope, float intercept)
{
	Thermocouple *thermocouple = (Thermocouple *)malloc(sizeof(Thermocouple));
	if (thermocouple != NULL)
	{
		thermocouple->type = type;
		thermocouple->slope = slope;
		thermocouple->intercept = intercept;

		return thermocouple;
	}
	else
	{
		return NULL;
	}
}

/**
 * @description:
 * @param {u16} arr
 * @param {u16} n
 * @return {*}
 */
u16 Bubble_Calculate_lan(u16 arr[], u16 n)
{
	u16 temp = 0;
	u32 sum = 0;
	u16 avg_current = 0;
	for (int i = n - 1; i > 0; i--)
	{
		for (int j = 0; j < i; j++)
		{
			if (arr[j] > arr[j + 1])
			{
				temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
	if (n >= 20)
	{
		for (int j = 4; j < n - 4; j++)
		{
			sum = sum + arr[j];
		}
		avg_current = sum / (n - 8);
	}
	else if (n >= 10)
	{
		for (int j = 6; j < n; j++)
		{
			sum = sum + arr[j];
		}
		avg_current = sum / (n - 6);
	}
	else
	{
		avg_current = arr[n - 4];
	}
	return avg_current;
}

/**
 * @description: ��ʾֵ�������
 * @param {u16} *arr
 * @param {u16} n     ��Ҫ�����õ���
 * @param {u16} start    ��Ч������ʼ�±�
 * @return {*}
 */

float rms_get(u16 *arr, u16 n, u16 start)
{
	float sum = 0;
	int num = 0;
	for (u16 i = 0; i < n; i++)
	{
		if (start + i < 1000)
		{
			num++;
			sum += arr[start + i] * arr[start + i];
		}
	}
	return sqrt(sum / num);
}
