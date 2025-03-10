#include "ThermocoupleIO.h"


void Check_IO_init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    // ---------------E config---------------
    RCC_AHB1PeriphClockCmd(CHECK_RCC_E, ENABLE);
    // out
    GPIO_InitStructure.GPIO_Pin = CHECKOUT_PIN_E;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(CHECK_GPIO_E, &GPIO_InitStructure);
    // in
    GPIO_InitStructure.GPIO_Pin = CHECKIN_PIN_E;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(CHECK_GPIO_E, &GPIO_InitStructure);

    //  ---------------J config ---------------
    RCC_AHB1PeriphClockCmd(CHECK_RCC_J, ENABLE);
    // out
    GPIO_InitStructure.GPIO_Pin = CHECKOUT_PIN_J;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(CHECK_GPIO_J, &GPIO_InitStructure);
    // in
    GPIO_InitStructure.GPIO_Pin = CHECKIN_PIN_J;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(CHECK_GPIO_J, &GPIO_InitStructure);

    //  ---------------K config ---------------
    RCC_AHB1PeriphClockCmd(CHECK_RCC_K, ENABLE);
    // out
    GPIO_InitStructure.GPIO_Pin = CHECKOUT_PIN_K;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(CHECK_GPIO_K, &GPIO_InitStructure);
    // in
    GPIO_InitStructure.GPIO_Pin = CHECKIN_PIN_K;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(CHECK_GPIO_K, &GPIO_InitStructure);
}
