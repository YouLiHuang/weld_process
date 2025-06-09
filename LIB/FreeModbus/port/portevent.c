/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-06 20:41:34
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-06 20:44:40
 * @Description: 
 * 
 * Copyright (c) 2025 by huangyouli, All Rights Reserved. 
 */
#include "mb.h"
#include "mbport.h"


/* Static variables */
static eMBEventType eQueuedEvent;
static BOOL xEventInQueue;

BOOL xMBPortEventInit(void)
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
    xEventInQueue = TRUE;
    eQueuedEvent = eEvent;
    return TRUE;
}

BOOL xMBPortEventGet(eMBEventType *eEvent)
{
    BOOL xEventHappened = FALSE;
    
    if(xEventInQueue)
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }
    
    return xEventHappened;
}
