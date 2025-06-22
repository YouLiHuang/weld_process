#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include "includes.h"

#define RESET_SPI_DATA 0
#define ALARM_MAX_TEMP 700

/*------------------mem config------------------*/
/*param number*/
#define TIME_NUM 6
#define TEMP_NUM 3
#define ALARM_NUM 6
#define GAIN_NUM 2
#define FIT_COEFFICIENT_NUM 2
/*address config*/
#define ADDR_PAGE_WIDTH 50
#define ADDR_OFFSET 2
/*address base*/
#define TIME_BASE_OFFSET 0
#define TEMP_BASE_OFFSET TIME_BASE_OFFSET + TIME_NUM *ADDR_OFFSET
#define ALARM_BASE_OFFSET TEMP_BASE_OFFSET + TEMP_NUM *ADDR_OFFSET
#define GAIN_BASE_OFFSET ALARM_BASE_OFFSET + ALARM_NUM *ADDR_OFFSET
#define FIT_COEFFICIENT_BASE_OFFSET GAIN_BASE_OFFSET + GAIN_NUM *ADDR_OFFSET
/*address convert*/
#define GP_BASE 0
#define PAGE_BASE(group) (ADDR_PAGE_WIDTH * (group + 2))
#define TIME_BASE(group) (PAGE_BASE(group) + TIME_BASE_OFFSET)
#define TEMP_BASE(group) (PAGE_BASE(group) + TEMP_BASE_OFFSET)
#define ALARM_BASE(group) (PAGE_BASE(group) + ALARM_BASE_OFFSET)
#define GAIN_BASE(group) (PAGE_BASE(group) + GAIN_BASE_OFFSET)
#define FIT_COEFFICIENT_BASE(group) (PAGE_BASE(group) + FIT_COEFFICIENT_BASE_OFFSET)

/*hardware api*/
void SPI1_Init(void);
uint16_t SPI_Load_Word(uint16_t addr);
void SPI_Save_Word(uint16_t data, uint16_t addr);
/*user api*/
// void save_param(void *controller, int array_of_data, const uint16_t *temp, const uint8_t temp_len, const uint16_t *time, const uint8_t time_len);
// void save_param_alarm(void *controller, int array_of_data, const uint16_t *temp, const uint8_t temp_len, const uint16_t *gain);

void Save_Param_toDisk(void);
void Load_param(void *controller, int array_of_data);
void Load_param_alarm(void *controller, int array_of_data);
void Load_Coefficient(int array_of_data);
/*computer api*/
void Host_computer_reset(void);

/*data init*/
void Load_data_from_mem(void);

#endif
