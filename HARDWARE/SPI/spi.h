#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
#include "welding_process.h"

#define RESET_SPI_DATA 0
#define MAX_TEMP 700

void Save_para(int array_of_data);

void Save_para_a(int array_of_data);
void SPI1_Init(void);
void spi1_config(void);
void SPI_Sendbyte(u8 data);
u8 SPI_Readbyte(void);
u16 SPI_Load_Word(u16 addr);
void SPI_Save_Word(u16 data, u16 addr);
void spi_data_init(void);
void WREN(void);

/*api old*/
void Read_para(int array_of_data);
void Read_para_a(int array_of_data);
void user_param_save(int array_of_data, const u16 *temp, const u8 temp_len, const u16 *time, const u8 time_len);
void user_param_save_alarm(int array_of_data, const u16 *temp, const u8 temp_len, const u16 *gain);

/*api*/
void save_param(weld_ctrl *ctrl, int array_of_data, const u16 *temp, const u8 temp_len, const u16 *time, const u8 time_len);
void save_param_alarm(weld_ctrl *ctrl, int array_of_data, const u16 *temp, const u8 temp_len, const u16 *gain);
void Load_param(weld_ctrl *ctrl, int array_of_data);
void Load_param_alarm(weld_ctrl *ctrl, int array_of_data);

#endif
