/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-05 11:20:53
 * @Description:
 * @
 * @Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
#ifndef __PROTECT_H
#define __PROTECT_H

#include "includes.h"
#include "touchscreen.h"
#include "stm32f4xx.h"
#include "stdbool.h"

/*hardware api*/
#define CURRENT_OVERLOAD_GPIO GPIOB
#define CURRENT_PIN GPIO_Pin_0

#define TEMP_OVERLOAD_GPIO GPIOC
#define RECTIFICATION_PIN GPIO_Pin_9
#define RADIATOR_PIN GPIO_Pin_10
#define WATER_PIN GPIO_Pin_11

/*user config*/
#define OVER_VOLTAGE 3500
#define DOWN_VOLTAGE 2600
#define VOLTAGE_FLOW 2048
#define ERR_LIST_MAX_LEN 16

/*threshold kinds*/
#define OVER_TEMP_THRESHOLD 20
#define LOW_TEMP_THRESHOLD 100
#define SENSOR_ERR_THRESHOLD 5
#define REVERSE_ERR_THRESHOLD 100

typedef bool (*err_callback)(u8 index);
typedef bool (*err_reset_callback)(u8 index);

typedef enum mERROR_TYPE
{
    CURRENT_OUT_OT_CTRL = 0x01,
    TEMP_UP,   /*temp too high*/
    TEMP_DOWN, /*temp too low*/
    VOLTAGE_TOO_HIGH,
    VOLTAGE_TOO_LOW,
    MCU_OVER_HEAT,
    IGBT_OVER_HEAT,
    RADIATOR,
    TRANSFORMER_OVER_HEAT,
    SENSOR_ERROR,
    TEMP_RISE_SLOW

} mERROR_TYPE;

typedef struct error_match_list
{
    mERROR_TYPE type;
    char *pic;
    err_callback err_callback;
    err_reset_callback reset_callback;
} error_match_list;

typedef struct error_handle
{
    bool (*error_callback)(u8 index); /*error callback*/
    bool (*reset_callback)(u8 index); /*reset callback*/
    char *error_msg;                  /*error information*/
    char *pic_name;                   /*specifies the picture resources*/
    mERROR_TYPE err_type;             /*sprcifies the error type*/
    bool state;                       /*error state is active or not*/
} error_handle;

typedef struct error_ctrl
{
    error_handle **err_list; /*err handle list?*/
    uint8_t error_cnt;       /*number of error handle*/
    uint8_t index;           /*point to current error handle*/
    uint8_t max_len;         /*the limit of error handle can be register*/

    uint8_t temp_over_threshold;          /*High temperature count threshold*/
    uint8_t temp_low_threshold;           /*Low temperature count threshold*/
    uint8_t sensor_err_threshold;         /*Sensor error threshold*/
    uint8_t Reverse_connection_threshold; /*Reverse connection error threshold*/

    /*The number of occurrences of the three errors*/
    uint8_t temp_over_cnt;
    uint8_t temp_low_cnt;
    uint8_t sensor_err_cnt;
    uint8_t Reverse_connection_cnt;

} Error_ctrl;

error_handle *new_error_handle(mERROR_TYPE type, const char *pic, err_callback err_callback, err_reset_callback reset_callback);
Error_ctrl *new_error_ctrl(void);
bool register_error(Error_ctrl *ctrl, error_handle *err_handle);
error_handle *err_get_type(Error_ctrl *ctrl, mERROR_TYPE type);
bool err_occur(Error_ctrl *ctrl);
void err_clear(Error_ctrl *ctrl);
void err_cnt_clear(Error_ctrl *ctrl);

/*hardword config*/
void Current_Check_IO_Config(void);
void Temp_Protect_IO_Config(void);
void Temp_Protect_IO_Config(void);

#endif
