#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "stdint.h"
typedef struct point_save
{
    uint16_t index;
    uint16_t val;

} point;

int16_t findValue(uint16_t *arr, uint16_t size, uint16_t val);
void dynamic_param_adjust(void);
void dynamic_pwm_adjust(void);

#endif
