#ifndef __FILTER_H
#define __FILTER_H
#include "includes.h"
#define PI 3.1415

void low_pass_Filter(float *input, float data_len, float *output, float freq_s, float freq_c);

#endif
