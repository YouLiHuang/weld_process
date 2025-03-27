#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "stdint.h"
typedef struct best_param
{
    uint16_t fast_rise_duty;
    uint16_t fast_rise_time;
    uint16_t start_temp;
} best_param;

typedef struct point_save
{
    uint16_t index;
    uint16_t val;

} point;

void dynamic_rise_correct(float target);
void dynamic_param_adjust(void);

#endif
