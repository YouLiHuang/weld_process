#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "stdint.h"

#define STABLE_THRESHOLD 30
#define TEMP_STABLE_ERR 5

#define LEARNING_RATE 3
#define MIN_STEP_SIZE 0.01f

#define STORAGE_DEPTH 500
#define MAX_CORRECT_GAIN 1.25f
#define MIN_CORRECT_GAIN 0.75f
#define OVERSHOOT_THRESHOLD 1.03f
#define REVERSE_OVERSHOOT_THRESHOLD 0.97f

typedef struct point_save
{
    uint16_t index;
    uint16_t val;

} point;

void dynamic_rise_correct(float target);
void dynamic_param_adjust(void);

#endif
