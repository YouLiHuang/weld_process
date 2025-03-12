/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-03-12 20:18:54
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */

#include "adc.h"
#include "delay.h"
#include "includes.h"
#include "protect.h"

// PA4-Irms
// PA5-Vrms(load)
// PA6-Transformer primary voltage
// PA7-The thermocouple outputs a feedback voltage
volatile unsigned short m_ADCValue[ADC_SAMPLE_PNUM][ADC_SAMPLE_CNUM] = {0}; // 列向量

/**
 * @description: ADC DAM config ，continuous sampling mode and DMA loop mode for data transmission
 * PCLK2=HCLK/APB2 Prescale2=168M/2=84M  ADC clock:PCLK2/4=84M/4=21M
 * there are 6 channels so，total time is：{12cycle（convert time）+10cycle（channle delay）+56cycle（sample time）}*6=468cycle
 * Therefore, the frequency of completing a round of sampling is 21M/468=45KHz
 * DMA calculates the average value of every 15 points,
 * so for each point, the frequency of obtaining an effective value is about 3KHz.
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
	/*GPIOA-----------------------------------------------------------------------------*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*GPIOC-----------------------------------------------------------------------------*/
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure2);

	/* ADC Init-------------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = ADC_SAMPLE_CNUM;
	ADC_Init(ADC1, &ADC_InitStructure);

	/*ADC1通道顺序-----------------------------------------------------------------------*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 6, ADC_SampleTime_56Cycles);

	/*DMA配置----------------------------------------------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 使能 DMA2 时钟

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
	/*stream0 ch0-----------------------------------------------------------------------*/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (unsigned int)&m_ADCValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_PNUM * ADC_SAMPLE_CNUM;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure); // 初始化 DMA
	DMA_Cmd(DMA2_Stream0, ENABLE);				// 启动 DMA

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	/*由于没有采用外部触发，所以使用软件触发ADC转换-----------------------------------------*/
	ADC_SoftwareStartConv(ADC1);
}

/**
 * @description:ADC sampling mean calculation
 * @param {uint16_t} channel
 * @return {*}
 */
uint16_t ADC_Value_avg(uint16_t channel)
{
	float value = 0;
	switch (channel)
	{

	case ADC_Channel_4:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][0];
		}
		value /= ADC_SAMPLE_PNUM;
		break;

	case ADC_Channel_5:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][1];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_6:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][2];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_7:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][3];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_14:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][4];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_15:
		value = 0;
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][5];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	}

	return (uint16_t)value;
}

/**
 * @description: Create a new thermocouple object
 * @param {SENSOR_TYPE} type
 * @param {float} slope
 * @param {float} intercept
 * @return {*}
 */
Thermocouple *newThermocouple(SENSOR_TYPE type, float slope, float intercept)
{
	Thermocouple *thermocouple = (Thermocouple *)malloc(sizeof(Thermocouple));
	if (thermocouple != NULL)
	{
		thermocouple->type = type;
		thermocouple->slope = slope;
		thermocouple->intercept = intercept;
		thermocouple->Bias = 0;

		return thermocouple;
	}
	else
	{
		return NULL;
	}
}

/**
 * @description: Temperature conversion function
 * @param {Thermocouple} Thermocouple objects
 * @return {*}
 */
uint16_t temp_convert(Thermocouple *thermocouple)
{
	uint16_t temp = 0;
	switch (thermocouple->type)
	{
	case E_TYPE:
		temp = thermocouple->slope * (ADC_Value_avg(THERMOCOUPLE_CHANNEL_E) - thermocouple->Bias) + thermocouple->intercept;
		break;
	case J_TYPE:
		temp = thermocouple->slope * (ADC_Value_avg(THERMOCOUPLE_CHANNEL_J) - thermocouple->Bias) + thermocouple->intercept;
		break;
	case K_TYPE:
		temp = thermocouple->slope * (ADC_Value_avg(THERMOCOUPLE_CHANNEL_K) - thermocouple->Bias) + thermocouple->intercept;
		break;
	}
	return temp;
}

/**
 * @description:
 * @param {uint16_t} arr
 * @param {uint16_t} n
 * @return {*}
 */
uint16_t Bubble_Calculate(uint16_t arr[], uint16_t n)
{
	uint16_t temp = 0;
	uint32_t sum = 0;
	uint16_t avg_current = 0;
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
 * @description: 显示值求均方根
 * @param {uint16_t} *arr
 * @param {uint16_t} n     需要处理得点数
 * @param {uint16_t} start    有效数据起始下标
 * @return {*}
 */

float rms_get(uint16_t *arr, uint16_t n, uint16_t start)
{
	float sum = 0;
	int num = 0;
	for (uint16_t i = 0; i < n; i++)
	{
		if (start + i < 1000)
		{
			num++;
			sum += arr[start + i] * arr[start + i];
		}
	}
	return sqrt(sum / num);
}
