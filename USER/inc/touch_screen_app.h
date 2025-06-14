/***
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-06-14 12:10:14
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-06-14 20:03:35
 * @Description:
 * @
 * @Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */
#ifndef TSCREEN_H
#define TSCREEN_H

#include "touchscreen.h"

/*----------------------------------------------------- user -----------------------------------------------------------*/
typedef void (*page_callback)(Page_ID id);

typedef struct page_map
{
    Page_ID id;
    Component_Queue *que;
    page_callback pg_cb;
    char **init_name_list;
    uint8_t list_len;
} page_map;

typedef struct Page_Manager
{
    Page_ID id;
    uint8_t page_cnt;
    page_map *map;

} Page_Manager;

/*screen task functions -------------------------------------------------------*/
void TSpage_process(Page_ID id);
bool PGManager_init(void);
Page_Manager *request_PGManger(void);
Component_Queue *get_page_list(Page_ID id);

#endif
