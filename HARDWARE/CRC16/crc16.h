/*** 
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-13 19:22:56
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-13 21:28:59
 * @Description: 
 * @
 * @Copyright (c) 2024 by huangyouli, All Rights Reserved. 
 */
/*
 * @Author: <lan> <1791060296@qq.com>
 * @Date: 2024-06-26 08:47:48
 * @LastEditors: <lan> <1791060296@qq.com>
 * @LastEditTime: 2024-06-29 21:33:35
 * @FilePath: \热压焊第三版_热电偶补偿20240626\HARDWARE\CRC16\crc16.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef _CRC16_H
#define _CRC16_H
#include "sys.h"

struct crc16_struct get_crc16_6(unsigned char input_array[6]);                 // crc 16校验获取
struct crc16_struct get_crc16_7(unsigned char input_array[7], int lenof_data); // crc 16校验获取
struct crc16_struct get_crc16_x(u8 input_array_text[200], int lenof_data);     // crc 16校验获取
unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);        // crc 16校验获取

void output_data_s(volatile u16 inputarray1[3], volatile u16 inputarray2[6]);                                                                                     // 发送焊接温度和焊接时间设置到触摸屏
void output_data_ss(volatile u16 inputarray2[6], u16 single_input);                                                                                               // 发送报警温度和增益系数到触摸屏
void output_data_plus_add(int data_welding, int data_counting);                                                                                                   // 未使用
void output_data_plus_alarm(int alarm_data);                                                                                                                      // 报警使用，发送报警标志到触摸屏
void output_data_plus(unsigned char anumber, unsigned char aaddress_high, unsigned char aaddress_low, unsigned char count_high, unsigned char count_low);         // 读写寄存器
void output_data_plus_60ms(unsigned char anumber, unsigned char aaddress_high, unsigned char aaddress_low, unsigned char count_high, unsigned char count_low);    // 读写寄存器60ms间隔，用于兼容新屏幕
void output_data_plus_nodelay(unsigned char anumber, unsigned char aaddress_high, unsigned char aaddress_low, unsigned char count_high, unsigned char count_low); // 读写寄存器无延时

void linear_interpolation(u16 *input, u16 nums_of_input, u16 *output, u16 nums_of_output);                                                                        // 线性插值
void draw_point_function(int point_number, int data_store[5]);

#endif
