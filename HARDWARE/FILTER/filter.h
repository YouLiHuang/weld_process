#ifndef __FILTER_H
#define __FILTER_H
#include "includes.h"
#define PI 3.1415

void low_pass_Filter(u16 *input, u16 data_len, u16 *output, u16 freq_s, u16 freq_c);

#endif
