#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include "includes.h"

#define RESET_SPI_DATA 1
#define ALARM_MAX_TEMP 700

/*------------------mem config------------------*/
/*param number*/
#define TIME_NUM 5
#define TEMP_NUM 3
#define ALARM_NUM 6
#define GAIN_NUM 2
#define CALIBRATION_VALUE_NUM 2
/*address config*/
#define ADDR_PAGE_WIDTH 40
#define ADDR_OFFSET 2
/*address base*/
#define TIME_BASE_OFFSET 0
#define TEMP_BASE_OFFSET 12
#define ALARM_BASE_OFFSET 18
#define GAIN_BASE_OFFSET 30
#define CARLIBRATION_BASE_OFFSET 34 // E J
/*address convert*/
#define GP_BASE 0
#define PAGE_BASE(group) (ADDR_PAGE_WIDTH * (group + 1))

#define TIME_BASE(group) (PAGE_BASE(group) + TIME_BASE_OFFSET)
#define TEMP_BASE(group) (PAGE_BASE(group) + TEMP_BASE_OFFSET)
#define ALARM_BASE(group) (PAGE_BASE(group) + ALARM_BASE_OFFSET)
#define GAIN_BASE(group) (PAGE_BASE(group) + GAIN_BASE_OFFSET)
#define CARLIBRATION_BASE(group) (PAGE_BASE(group) + CARLIBRATION_BASE_OFFSET)

/*hardware api*/
void SPI1_Init(void);
uint16_t SPI_Load_Word(uint16_t addr);
void SPI_Save_Word(uint16_t data, uint16_t addr);
/*user api*/
void save_param(void *controller, int array_of_data, const uint16_t *temp, const uint8_t temp_len, const uint16_t *time, const uint8_t time_len);
void save_param_alarm(void *controller, int array_of_data, const uint16_t *temp, const uint8_t temp_len, const uint16_t *gain);
void Load_param(void *controller, int array_of_data);
void Load_param_alarm(void *controller, int array_of_data);

/*computer api*/
void Host_computer_reset(void);

/*data init*/
void Load_data_from_mem(void);

#endif
