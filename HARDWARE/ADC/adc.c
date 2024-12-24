/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-05 10:47:57
 * @Description: 
 * 
 * Copyright (c) 2024 by huangyouli, All Rights Reserved. 
 */


#include "adc.h"
#include "delay.h"
#include "includes.h"
#include "protect.h"

// PA4采样一次侧/二次测电流
// PA5采样二次测输出电压
// PA6采样初级电压
// PA7采样温度变送器
#define ADC_SAMPLE_PNUM 15													// AD 采样点数数
#define ADC_SAMPLE_CNUM 4													// AD 采样通道数
volatile unsigned short m_ADCValue[ADC_SAMPLE_PNUM][ADC_SAMPLE_CNUM] = {0}; // 列向量


/**
 * @description: ADC DAM config ，continuous sampling mode and DMA loop mode for data transmission
 * @return {*}
 */
void ADC_DMA_INIT(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/*GPIO----------------------------------------------------------------------------- */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);							 // 使能GPIOA时钟
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;									 // 模拟输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // PA4~7
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;								 // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);											 // 初始化

	/*DMA配置----------------------------------------------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 使能 DMA2 时钟

	DMA_Cmd(DMA2_Stream0, DISABLE);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)&(ADC1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (unsigned int)&m_ADCValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_PNUM * ADC_SAMPLE_CNUM;
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
	DMA_Init(DMA2_Stream0, &DMA_InitStructure); // 初始化 DMA
	DMA_Cmd(DMA2_Stream0, ENABLE);				// 启动 DMA

	/* ADC1 Init--------------------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // 使能ADC1时钟
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8; // 预分频
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles; // 采样间隔
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = ADC_SAMPLE_CNUM;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	DMA_Cmd(DMA2_Stream0, ENABLE);
	/*通道顺序------------------------------------------------------------------------------*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_56Cycles); // ADC1通道4配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_56Cycles); // ADC1通道5配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_56Cycles); // ADC1通道5配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 4, ADC_SampleTime_56Cycles); // ADC1通道7配置

	/*由于没有采用外部触发，所以使用软件触发ADC转换-------------------------------------------- */
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
	case ADC_Channel_4:
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][0];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_5:
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][1];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_6:
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][2];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	case ADC_Channel_7:
		for (uint8_t i = 0; i < ADC_SAMPLE_PNUM; i++)
		{
			value += m_ADCValue[i][3];
		}
		value /= ADC_SAMPLE_PNUM;
		break;
	}

	return value;
}

u16 t1 = 0;
u32 temp_val1 = 0;
/**
 * @description:
 * @return {*}
 */
void Adc_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  // 使能ADC1时钟

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // 使能ADC1时钟
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // 预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
	ADC_CommonInit(&ADC_CommonInitStructure);					// 初始化

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;									 // 模拟输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // PA4~7
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;								 // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);											 // 初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;						// 12位模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;								// 非扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							// 拐连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // 禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						// 右对齐
	ADC_InitStructure.ADC_NbrOfConversion = 1;									// 1个转换在规则序列中 也就是只转换规则序列1
	ADC_Init(ADC1, &ADC_InitStructure);											// ADC1初始化

	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles); // ADC1通道4配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_84Cycles);	// ADC1通道5配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_480Cycles); // ADC1通道5配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_84Cycles);	// ADC1通道7配置
	ADC_Cmd(ADC1, ENABLE);														// 开启ADC1转换器
}

/**
 * @description: 
 * @param {u8} ch1
 * @return {*}
 */
u16 Get_Adc1(u8 ch1)
{
	// 设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch1, 1, ADC_SampleTime_84Cycles); // ADC1,ADC通道15个周期,提高采样时间可以提高精确度

	ADC_SoftwareStartConv(ADC1); // 使能指定的ADC1的软件转换启动功能

	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
		; // 等待转换结束
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

	return ADC_GetConversionValue(ADC1); // 返回最近一次ADC1规则组的转换结果
}

/**
 * @description: 
 * @param {u8} ch1
 * @param {u16} times1
 * @return {*}
 */
u16 Get_Adc1_Average(u8 ch1, u16 times1)
{
	// temp_val1 = 0;
	// for (t1 = 0; t1 < times1; t1++)
	// {
	// 	temp_val1 += Get_Adc1(ch1);
	// }
	// return temp_val1 / times1;
	return ADC_Value_avg(ch1);
}

/**
 * @description: 
 * @param {u8} ch1
 * @param {u16} n
 * @return {*}
 */
u16 Bubble_Sort_Calculate(u8 ch1, u16 n)
{
	int i, j, z;
	u16 Array[n];
	u32 Sum = 0;
	int end;

	for (i = 0; i < n; i++)
	{
		Array[i] = Get_Adc1(ch1);
	}
	for (i = 0; i < n - 1; i++)
	{
		for (j = 0; j < n - i - 1; j++)
		{
			if (Array[j] > Array[j + 1]) // 如果前者大于后者
			{
				int16_t result = Array[j]; // 则交换两者的值
				Array[j] = Array[j + 1];
				Array[j + 1] = result;
			}
		}
	}

	// 30个数据，去掉首尾5个，取20个数据
	for (z = 5; z < n - 5; z++)
	{
		Sum = Sum + Array[z];
	}

	end = Sum / 20;

	return end;
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
 * @description: 显示值求均方根
 * @param {u16} *arr
 * @param {u16} n     需要处理得点数
 * @param {u16} start    有效数据起始下标
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

/**
 * @description:   温度显示
 * @param {u16} *input
 * @param {u16} nums   采样点数
 * @param {u16} start  有效数据起始下标
 * @param {u16} refer  参考值
 * @return {*}
 */
u16 temp_display(u16 *input, u16 nums, u16 start)
{
	return (u16)rms_get(input, nums - start, start);
}
