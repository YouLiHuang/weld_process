/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-06 20:56:45
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-07 09:28:58
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#include "port.h"

// void EnterCriticalSection()
// {
// 	__ASM volatile("cpsid i");
// }

// void ExitCriticalSection()
// {
// 	__ASM volatile("cpsie i");
// }

void __aeabi_assert(const char *x1, const char *x2, int x3)
{
	;
}
