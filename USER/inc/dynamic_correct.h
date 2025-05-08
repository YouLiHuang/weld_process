#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "stdint.h"

#define STABLE_THRESHOLD 30
#define TEMP_STABLE_ERR 5

typedef struct point_save
{
    uint16_t index;
    uint16_t val;

} point;

void dynamic_rise_correct(float target);
void dynamic_param_adjust(void);

#endif
