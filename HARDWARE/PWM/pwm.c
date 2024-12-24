#include "pwm.h"

//输出PC6和PE9
//TIM1-CH1  PE9
//TIM1-CHN  PE8
//TIM1-OC2   触发源
void TIM1_PWM_Init(void)
{		 					 
	// 结构体声明
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//    TIM_BDTRInitTypeDef TIM_BDTRStructure;

	// 时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTC时钟	

	// IO复用
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 

	// IO配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PE
	 
	// 定时器配置
	TIM_TimeBaseStructure.TIM_Prescaler=2-1;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=16800-1;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
	//Fpwm=84M/((arr+1)*(psc+1))Hz

	// 定时器比较输出通道配置
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_Pulse=0; 	
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;  //输出极性高
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Disable; //输出使能
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset; // 输出空闲电平低

	// 互补输出配置
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable; //互补输出使能
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High;	// 互补端输出极性高
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset; //互补输出空闲电平低
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  // OC1
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);   //配置

	// OC2配置
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;  //
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//
	TIM_OCInitStructure.TIM_Pulse = 16800/2;                //相位差180度，既互补
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	//	TIM_SelectOutputTrigger(TIM1,TIM_TRGOSource_OC2Ref);  //TRGO输出
	//    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);   //配置

	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC2Ref); // TRGO输出

	TIM_ARRPreloadConfig(TIM1, ENABLE); // ARPE使能

	TIM_SetCompare1(TIM1, 0);

	TIM_Cmd(TIM1, ENABLE);			  // 使能TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE); // 开启OC和OCN输出
}  
 
 
// 定时器4
// PD12-CH1
void TIM4_PWM_Init(void)
{		 					 
	// 结构体声明
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	// 时钟使能
	RCC_PCLK1Config(RCC_HCLK_Div1);  //APB2不分频
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTD时钟	

	// IO复用
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 

	// GPIOD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           //GPIOD12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化PD12

	// 定时器配置
	TIM_TimeBaseStructure.TIM_Prescaler=2-1;  	
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=16800-1;   	
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器1

	// 定时器比较输出通道配置

	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; 	
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;  //输出极性高
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //输出使能
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset; // 输出空闲电平低


	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  // OC1
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);   //配置

	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 

	TIM_SetCompare1(TIM4, 0);
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset); // 复位模式
	TIM_SelectInputTrigger(TIM4, TIM_TS_ITR0);		// 内部触发源 TIM_TS_ITR0，让CH2触发TIM4

	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
								  
}  
 
 
 
//内部触发配置
void TIM4_IN_Config(void)
{
  TIM_SelectSlaveMode(TIM4,TIM_SlaveMode_Reset);             //复位模式
  TIM_SelectInputTrigger (TIM4,TIM_TS_ITR0);                 //内部触发源 TIM_TS_ITR0，让CH2触发TIM4
}
 

 
 










