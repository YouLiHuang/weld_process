#include "modbus_bsp.h"
#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "sys.h"
#include "usart.h"

/*BSP PARAM*/
uint8_t ID_OF_DEVICE = 0;
uint32_t Baud_Rate_Modbus = 115200;

void modbus_timer_bsp_init(uint32_t usTim1Timerout50us)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint16_t PrescalerValue = 0;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    // HCLK为72MHz
    // 时基频率72 / （1 + Prescaler) = 20KHz
    PrescalerValue = (uint16_t)((SystemCoreClock / 20000) - 1);

    TIM_TimeBaseStructure.TIM_Period = (uint16_t)usTim1Timerout50us;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(MODBUS_TIMER, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(MODBUS_TIMER, ENABLE);

    /*NVIC*/
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearITPendingBit(MODBUS_TIMER, TIM_IT_Update);
    TIM_ITConfig(MODBUS_TIMER, TIM_IT_Update, DISABLE);
    TIM_Cmd(MODBUS_TIMER, DISABLE);
}

void modbus_serial_bsp_init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // 使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART3时钟

    // 串口2引脚复用映射
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10复用为USAR3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11复用为USART3

    // USART2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;         // GPIOB10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;         // GPIOB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // PG9推挽输出，485模式控制
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          // GPIOG9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);             // 初始化PG8

    // USART2 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                                     // 波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 收发模式
    USART_Init(USART3, &USART_InitStructure);                                       // 初始化串口3
    USART_Cmd(USART3, ENABLE);                                                      // 使能串口 2

    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

    // Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        // 子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                           // 根据指定的参数初始化VIC寄存器、

    BIT_ADDR(GPIOB_ODR_Addr, 9) = 0; // 默认为接收模式
}

void modbus_bound_set(uint32_t bound)
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = bound;                                     // 波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 收发模式
    USART_Init(USART3, &USART_InitStructure);                                       // 初始化串口2
    USART_ClearFlag(USART3, USART_FLAG_TC);
}
