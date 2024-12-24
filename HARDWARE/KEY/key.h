#ifndef __KEY_H
#define __KEY_H
#include "sys.h"
#include "includes.h"

// ͨ��λ��������ȡ����

#define KEY_PC0 PCin(0)   // ����1 PC0
#define KEY_PC1 PCin(1)   // ����2	PC1
#define KEY_PC2 PCin(2)   // ����3	PC2
#define KEY_PC3 PCin(3)   // ����4	PC3
#define KEY_In1 PCin(4)   // ����1
#define KEY_In2 PCin(5)   // ����2
#define KEY_In3 PAin(6)   // ����3
#define KEY_In4 PCin(7)   // ����4
#define KEY_Res PCin(8)   // ��λ
#define KEY_Ttem PCin(9)  // �������¶�
#define KEY_Stem PCin(10) // ɢ�����¶�
#define KEY_Wat PCin(11)  // ��ȴˮ

#define KEY_PC0_PRES 1
#define KEY_PC1_PRES 2
#define KEY_PC2_PRES 3
#define KEY_PC3_PRES 4
#define KEY_In1_PRES 5
#define KEY_In2_PRES 6
#define KEY_In3_PRES 7
#define KEY_In4_PRES 8
#define KEY_Res_PRES 9

// ����˿ڶ���
#define RLY10 PDout(0)  // ����1
#define RLY11 PDout(1)  // ����2
#define RLY12 PDout(2)  // ����3
#define OVER PDout(3)   // �����ź�
#define ERROR1 PDout(4) // �����ź�
#define CUNT PDout(5)   // ����
#define RLY13 PDout(6)  // �����Ӵ���
#define TRAN1 PDout(7)  // ��ѹ��1
#define OUT1 PDout(8)   // ���1
#define OUT2 PDout(9)   // ���2
#define OUT3 PDout(10)  // ���3
#define OUT4 PDout(11)  // ���4

// spi
#define CSN PBout(12) // AT25Ƭѡ�ź�

/*
#define set_bit(bit_address) *((volatile unsigned long *)(0x22000000+((unsigned long)&bit_data-0x20000000)*32+(bit_address)*4))=1
#define reset_bit(bit_address) *((volatile unsigned long *)(0x22000000+((unsigned long)&bit_data-0x20000000)*32+(bit_address)*4))=0
#define Bit(bit_address) *((volatile unsigned long *)(0x22000000+((unsigned long)&bit_data-0x20000000)*32+(bit_address)*4))

#define Save_flag Bit(25)
*/

void KEYin_Init(void);
void OUT_Init(void);
u8 KEY_Scan(u8 mode);
u8 KEY_Scan2(u8 mode);

u8 new_key_scan(void);

#endif
