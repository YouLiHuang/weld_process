#include "pwm.h"

// 输出PC6和PE9
// TIM1-CH1  PE9
// TIM1-CHN  PE8
// TIM1-OC2   触发源

/**
 * @description:  APB2 Prescaler=4（！=1） so APB2_TIM1_CLK=168M/4*2=84M
 * 所以tim1CLK=PCLK1*2=84M*2=168M
 * @return {*}
 */
void TIM1_PWM_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// pre(=4)!=1 --> timer clock x2 --> APB2_Clock(timer1)=168M/4(pre)*2=84M
	RCC_PCLK2Config(RCC_HCLK_Div4);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/*IO CONFIG*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/*TIMER Config--5KHz*/
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 16800 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/*-----------------------------------CH1&CH1N Config-----------------------------------*/
	// CH1 Config
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			   // cnt>=compare，low output
	TIM_OCInitStructure.TIM_Pulse = 0;							   // duty 0(no output)
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	   // output low
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; // out disable
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   // ideal low
	// CH1N-PWMOUT
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;	   // output high(force to high-->make sure the right wave)
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset; // ideal low
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);					   // OC1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/*-----------------------------------CH2 Config-----------------------------------------*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;				 // cnt>=compare，high output
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;	 // sync signal (no out put)
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; // no output
	TIM_OCInitStructure.TIM_Pulse = 16800 / 2;						 // 50% duty delta
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	/*-----------------------------------master slaver mode config-----------------------------------*/
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC2Ref); // TRG Oout
	TIM_ARRPreloadConfig(TIM1, ENABLE);					  // ARPE enable
	TIM_SetCompare1(TIM1, 0);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE); // OC&OCN enable
}

// TIMER4
// PD12-CH1
/**
 * @description: APH1 defalut clock=168M/4=42M pre(=4)!=1 so APB1_TIM1_CLK=42M*2=84M
 * @return {*}
 */
void TIM4_PWM_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/*RCC config*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/*IO config*/
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/*timer config--5KHz*/
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 16800 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // cnt>=compare，low output
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

	/*OC1 config*/
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_SetCompare1(TIM4, 0);						// init duty=0%
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset); // reset mode(sync)
	TIM_SelectInputTrigger(TIM4, TIM_TS_ITR0);		// TIM_TS_ITR0，让TIMER1_CH2->trgger->TIM4
	TIM_Cmd(TIM4, ENABLE);
}
