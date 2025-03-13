#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include "welding_process.h"

#define RESET_SPI_DATA 0
#define ALARM_MAX_TEMP 700

/*------------------mem config------------------*/
/*param number*/
#define TIME_NUM 5
#define TEMP_NUM 3
#define ALARM_NUM 6
#define GAIN_NUM 2
/*address config*/
#define ADDR_PAGE_WIDTH 40
#define ADDR_OFFSET 2
/*address base*/
#define TIME_BASE_OFFSET 0
#define TEMP_BASE_OFFSET 12
#define ALARM_BASE_OFFSET 18
#define GAIN_BASE_OFFSET 30
/*address convert*/
#define PAGE_BASE(group) (ADDR_PAGE_WIDTH * (group + 1))

#define TIME_BASE(group) (PAGE_BASE(group) + TIME_BASE_OFFSET)
#define TEMP_BASE(group) (PAGE_BASE(group) + TEMP_BASE_OFFSET)
#define ALARM_BASE(group) (PAGE_BASE(group) + ALARM_BASE_OFFSET)
#define GAIN_BASE(group) (PAGE_BASE(group) + GAIN_BASE_OFFSET)

/*hardware api*/
void SPI1_Init(void);
void spi1_config(void);
void SPI_Sendbyte(u8 data);
u8 SPI_Readbyte(void);
u16 SPI_Load_Word(u16 addr);
void SPI_Save_Word(u16 data, u16 addr);
void WREN(void);

/*user api*/
void save_param(weld_ctrl *ctrl, int array_of_data, const u16 *temp, const u8 temp_len, const u16 *time, const u8 time_len);
void save_param_alarm(weld_ctrl *ctrl, int array_of_data, const u16 *temp, const u8 temp_len, const u16 *gain);
void Load_param(weld_ctrl *ctrl, int array_of_data);
void Load_param_alarm(weld_ctrl *ctrl, int array_of_data);

/*computer api*/
void Host_computer_reset(void);

/*data init*/
void Load_data_from_mem(void);

#endif
