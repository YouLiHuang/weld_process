/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2024-12-05 10:27:44
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */
#include "Kalman.h"

void Kalman_Init(Kalman *kfp)
{
   kfp->Last_P = 1;
   kfp->Now_P = 0;
   kfp->out = 0;
   kfp->Kg = 0;
   kfp->Q = 0.001; // 0.001
   kfp->R = 0.5;   // 0.01
}

float KalmanFilter(Kalman *kfp, float input)
{
   // Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
   kfp->Now_P = kfp->Last_P + kfp->Q;
   // ���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   // ��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
   kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // ��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
   // ����Э�����: ���ε�ϵͳЭ����� kfp->LastP Ϊ��һ������׼����
   kfp->Last_P = (1 - kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
