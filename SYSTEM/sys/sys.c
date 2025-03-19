/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-01-18 19:08:13
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-03-19 09:13:07
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "sys.h"

// THUMBָ�֧�ֻ������
// �������·���ʵ��ִ�л��ָ��WFI
__asm void WFI_SET(void)
{
	WFI;
}
// �ر������ж�(���ǲ�����fault��NMI�ж�)
__asm void INTX_DISABLE(void)
{
	CPSID I
		BX LR
}
// ���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE I
		BX LR
}
// ����ջ����ַ
// addr:ջ����ַ
__asm void MSR_MSP(u32 addr)
{
	MSR MSP, r0 // set Main Stack value
				 BX r14
}
