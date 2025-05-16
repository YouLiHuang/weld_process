/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2024-12-05 09:43:02
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-01-03 19:23:40
 * @Description:
 *
 * Copyright (c) 2024 by huangyouli, All Rights Reserved.
 */

#define FAST_MODE 1
#include "touchscreen.h"
#include "welding_process.h"
#include "usart.h"

const u32 baud_list[] = {2400, 4800, 9600, 19200, 31200, 38400, 57600, 115200, 230400, 250000, 256000, 512000, 921600};

extern OS_Q UART_Msg;           // 串口数据队列
extern Page_Param *page_param;  // 记录当前所在界面id及RDY三个按钮的状态
extern OS_SEM PAGE_UPDATE_SEM;  // 页面更新信号
extern OS_SEM COMP_VAL_GET_SEM; // 组件属性值成功获取信号
extern OS_SEM COMP_STR_GET_SEM; // 组件属性值(字符串型)成功获取信号
//////////////////////////////////////////////////////////////////////////////////////////////页面参数API//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @description:
 * @return {*}
 */
Page_Param *new_page_param()
{
  Page_Param *p = (Page_Param *)malloc(sizeof(Page_Param));
  p->id = PARAM_PAGE;
  p->key1 = RDY;
  p->key2 = IOFF;
  p->key3 = SGW;
  p->GP = 0;
  p->user_buf_val = NULL;
  p->user_buf_str = NULL;
  return p;
}

/**
 * @description:
 * @param {Page_Param} *p
 * @return {*}
 */
void delete_page_param(Page_Param *p)
{
  if (p != NULL)
  {
    if (p->user_buf_str)
    {
      free(p->user_buf_str);
    }
    if (p->user_buf_val)
    {
      free(p->user_buf_val);
    }
    free(p);
  }
}

/**
 * @description:
 * @param {char} *name
 * @param {int} val
 * @return {*}
 */
Component *newComponet(const char *name, int val)
{

  Component *p = (Component *)malloc(sizeof(Component));
  if (p != NULL)
  {
    p->name = (char *)malloc(sizeof(char) * (strlen(name) + 1)); // 含结束符
    if (p->name != NULL)
    {
      strncpy(p->name, name, (strlen(name) + 1)); // 含结束符
    }

    p->index = 0;
    p->val = val;
    p->next = p;

    return p;
  }
  return NULL;
}

/**
 * @description: delete component
 * @param {Component} *comp
 * @return {*}
 */
void deleteComponent(Component *comp)
{
  if (comp)
  {
    if (comp->name)
    {
      free(comp->name);
    }
    free(comp);
  }
}

/**
 * @description: insert component to the list
 * @param {Component_Queue} *list
 * @param {Component} *comp
 * @return {*}
 */
bool component_insert(Component_Queue *list, Component *comp)
{
  if (comp != NULL && list != NULL)
  {
    Component *cur = list->list_pointer;
    if (cur == NULL && list->len == 0) // 第一个元素入队
    {
      list->list_pointer = comp;
      comp->next = comp;
    }
    else
    {
      // 定位到队尾
      uint8_t count = 0;
      while (cur != cur->next)
      {
        cur = cur->next;
        count++;
      }
      cur->next = comp;
      comp->next = comp;
      comp->index = count + 1;
    }
    list->len++;

    return true;
  }

  return false;
}

/**
 * @description: insert component to the list
 * @param {Component_Queue} *list
 * @param {char} *name
 * @param {uint16_t} val
 * @return {*}
 */
bool insert_comp(Component_Queue *list, const char *name, const uint16_t val)
{
  if (list != NULL)
  {
    Component *comp = newComponet(name, val);
    return component_insert(list, comp);
  }
  return false;
}

/**
 * @description: create new list
 * @param {Page_ID} id
 * @return {*}
 */
Component_Queue *newList(Page_ID id)
{
  Component_Queue *p = (Component_Queue *)malloc(sizeof(Component_Queue));
  if (p != NULL)
  {
    p->id = id;
    p->len = 0;
    p->list_pointer = NULL;
    p->updata = NULL;
    return p;
  }

  return NULL;
}

/**
 * @description: get the component specifies by name
 * @param {Component_Queue} *list
 * @param {char} *name
 * @return {*}
 */
Component *get_comp(Component_Queue *list, const char *name)
{
  if (list != NULL)
  {
    Component *cur = list->list_pointer;
    while (cur)
    {
      if (strcmp(cur->name, name) == 0)
        return cur;
      else if (cur->next == cur) // 到了队尾
        return NULL;
      cur = cur->next;
    }
  }

  return NULL;
}

/**
 * @description: Initialize parameter queue
 * @param {Component_Queue} *param_page_list
 * @return {*}
 */
bool param_page_list_init(Component_Queue *param_page_list)
{
  const char *temp_name_list[] = {
      "temp1",
      "temp2",
      "temp3",
  };
  const char *time_name_list[] = {
      "time1",
      "time2",
      "time3",
      "time4",
      "time5",
  };
  component_insert(param_page_list, newComponet("GP", 0)); // GP
  for (uint8_t i = 0; i < sizeof(temp_name_list) / sizeof(char *); i++)
  {
    /*插入新组件*/
    if (component_insert(param_page_list, newComponet(temp_name_list[i], 0)) != true)
      return false;
  }
  for (uint8_t i = 0; i < sizeof(time_name_list) / sizeof(char *); i++)
  {
    /*插入新组件*/
    if (component_insert(param_page_list, newComponet(time_name_list[i], 0)) != true)
      return false;
  }

  if (component_insert(param_page_list, newComponet("RDY_SCH", RDY)) != true)
    return false;
  if (component_insert(param_page_list, newComponet("ION_OFF", ION)) != true)
    return false;
  if (component_insert(param_page_list, newComponet("SGW_CTW", SGW)) != true)
    return false;
  if (component_insert(param_page_list, newComponet("sensortype", K_TYPE)) != true)
    return false;

  return true;
}

/**
 * @description: Initialize temperature queue
 * @param {Component_Queue} *temp_page_list
 * @return {*}
 */
bool temp_page_list_init(Component_Queue *temp_page_list)
{

  char *temp_name_list[] = {"alarm1",
                            "alarm2",
                            "alarm3",
                            "alarm4",
                            "alarm5",
                            "alarm6"};
  char *gain_name_list[] = {"GAIN1",
                            "GAIN2",
                            "GAIN3"};

  component_insert(temp_page_list, newComponet("GP", 0)); // GP

  for (uint8_t i = 0; i < sizeof(temp_name_list) / sizeof(char *); i++) // temp1~temp3
  {
    /*插入新组件*/
    if (component_insert(temp_page_list, newComponet(temp_name_list[i], 0)) != true)
      return false;
  }

  for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++) // temp1~temp3
  {
    /*插入新组件*/
    if (component_insert(temp_page_list, newComponet(gain_name_list[i], 0)) != true)
      return false;
  }

  if (component_insert(temp_page_list, newComponet("RDY_SCH", RDY)) != true)
    return false;
  if (component_insert(temp_page_list, newComponet("ION_OFF", ION)) != true)
    return false;
  if (component_insert(temp_page_list, newComponet("SGW_CTW", SGW)) != true)
    return false;

  return true;
}

/**
 * @description: 
 * @param {Component_Queue} * ui list object
 * @param {char} * Component name list
 * @param {uint8_t} Number of components
 * @return {*}
 */
bool page_list_init(Component_Queue *page_list, char *name_list[], uint8_t list_len)
{

  component_insert(page_list, newComponet("GP", 0)); // GP
  for (uint8_t i = 0; i < list_len; i++)             // 插入新组件
  {
    if (component_insert(page_list, newComponet(name_list[i], 0)) != true)
      return false;
  }

  return true;
}

/**
 * @description:
 * @param {Component_Queue} *temp_page_list
 * @return {*}
 */
bool uart_page_list_init(Component_Queue *temp_page_list)
{
  char *name = (char *)malloc(NAME_LEN_MAX * sizeof(char));
  if (name != NULL)
  {
    component_insert(temp_page_list, newComponet("cb0", 0)); // adress
    component_insert(temp_page_list, newComponet("cb1", 0)); // bauds

    free(name);
    return true;
  }
  return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////触摸屏API//////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @description: sned data to RS485
 * @param {char} *buffer
 * @param {uint16_t} len
 * @return {*}
 */
void RS485_send(const char *buffer, const uint16_t len)
{
  BIT_ADDR(GPIOA_ODR_Addr, 2) = 1; // 设置为发送模式
  for (int i = 0; i < len; i++)
  {
    USART_SendData(UART4, buffer[i]);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
      ; // 把请求类型发送过去
  }
  BIT_ADDR(GPIOA_ODR_Addr, 2) = 0; // 设置为接收模式
}

// 用于指令数据处理的缓冲区
static char buffer[CMD_BUFFER_LEN] = {0};
/**
 * @description: send the raw command spcifies by cmd
 * @param {char} *cmd
 * @return {*}
 */
bool command_send(const char *cmd)
{
  uart_init(115200);
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  /*指令处理*/
  sprintf(buffer, "%s%s", cmd, END_OF_CMD);

#if FAST_MODE == 1
  RS485_send(buffer, strlen(buffer)); // 发送数据
#else
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();                // 进入临界区
  RS485_send(buffer, strlen(buffer)); // 发送数据
  OS_CRITICAL_EXIT();                 // 退出临界区
#endif

  return true;
}

/**
 * @description: set the component value specifies by the param
 * @param {char} *name
 * @param {char} *compatible
 * @param {int} val
 * @return {*}
 */
bool command_set_comp_val(const char *name, const char *compatible, const int val)
{
  uart_init(115200);
  /*参数检查*/
  if (name == NULL || compatible == NULL)
    return false;
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  /*指令处理*/
  sprintf(buffer, "%s.%s=%d%s", name, compatible, val, END_OF_CMD);

#if FAST_MODE == 1
  RS485_send(buffer, strlen(buffer)); // 发送数据
#else
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();                // 进入临界区
  RS485_send(buffer, strlen(buffer)); // 发送数据
  OS_CRITICAL_EXIT();                 // 退出临界区
#endif

  return true;
}

/**
 * @description: set the component value specifies by the param
 * @param {char} *name
 * @param {char} *compatible
 * @param {char} *str
 * @return {*}
 */
bool command_set_comp_str(const char *name, const char *compatible, const char *str)
{
  uart_init(115200);
  /*参数检查*/
  if (name == NULL || compatible == NULL)
    return false;
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  sprintf(buffer, "%s.%s=%s%s", name, compatible, str, END_OF_CMD); // 指令处理
  // strcat(buffer, END_OF_CMD);                         // 加上结束符
#if FAST_MODE == 1
  RS485_send(buffer, strlen(buffer)); // 发送数据
#else
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();                // 进入临界区
  RS485_send(buffer, strlen(buffer)); // 发送数据
  OS_CRITICAL_EXIT();                 // 退出临界区
#endif

  return true;
}

/**
 * @description: get the int type value from the screen
 * @param {Component_Queue} *list
 * @param {char} *name
 * @param {char} *compatible
 * @return {*}
 */
bool command_get_comp_val(Component_Queue *list, const char *name, const char *compatible)
{
  uart_init(115200);
  /*参数检查*/
  if (list == NULL || name == NULL || compatible == NULL)
    return false;
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  /*数据处理*/
  sprintf(buffer, "get %s.%s%s", name, compatible, END_OF_CMD); // 指令处理
  // strcat(buffer, END_OF_CMD);                     // 加上结束符

#if FAST_MODE == 1
  list->updata = get_comp(list, name);
  RS485_send(buffer, strlen(buffer)); // 发送数据
#else
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();                // 进入临界区
  RS485_send(buffer, strlen(buffer)); // 发送数据
  OS_CRITICAL_EXIT();                 // 退出临界区
#endif
  OS_ERR err;
  OSSemPend(&COMP_VAL_GET_SEM, 200, OS_OPT_PEND_BLOCKING, NULL, &err);
  if (err == OS_ERR_NONE)
  {
    list->updata = get_comp(list, name); // 组件属性值更新
    list->updata->val = USART_RX_BUF[1] | USART_RX_BUF[2] << 8 | USART_RX_BUF[3] << 16 | USART_RX_BUF[4] << 24;
    return true;
  }
  else
    return false;
}

/**
 * @description: get the string from the screen
 * @param {Component_Queue} *list
 * @param {char} *name :specifies the component name
 * @param {char} *compatible :specifies the compatible of the component
 * @return {*}
 */
bool command_get_comp_str(Component_Queue *list, const char *name, const char *compatible)
{
  uart_init(115200);
  /*参数检查*/
  if (list == NULL || name == NULL || compatible == NULL)
    return false;
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  /*数据处理*/
  sprintf(buffer, "get %s.%s%s", name, compatible, END_OF_CMD); // 指令处理

#if FAST_MODE == 1
  list->updata = get_comp(list, name); // 组件属性值更新
  RS485_send(buffer, strlen(buffer));  // 发送数据
#else
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();                // 进入临界区
  RS485_send(buffer, strlen(buffer)); // 发送数据
  OS_CRITICAL_EXIT();                 // 退出临界区
#endif
  OS_ERR err;
  OSSemPend(&COMP_STR_GET_SEM, 100, OS_OPT_PEND_BLOCKING, NULL, &err);
  if (err == OS_ERR_NONE)
  {
    list->updata = get_comp(list, name); // 组件属性值更新
    list->updata->val = USART_RX_BUF[1] | USART_RX_BUF[2] << 8 | USART_RX_BUF[3] << 16 | USART_RX_BUF[4] << 24;
    return true;
  }
  else
    return false;
}

/**
 * @description: raw api that only call before enter os
 * @param {char} *cmd
 * @return {*}
 */
void command_send_raw(const char *cmd)
{
  uart_init(115200);
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  /*指令处理*/
  sprintf(buffer, "%s%s", cmd, END_OF_CMD);
  /*发送数据*/
  RS485_send(buffer, strlen(buffer));
}

/**
 * @description: raw api
 * @param {char} *name
 * @param {char} *compatible
 * @param {int} val
 * @return {*}
 */
void command_set_comp_val_raw(const char *name, const char *compatible, int val)
{
  uart_init(115200);
  /*参数检查*/
  if (name == NULL || compatible == NULL)
    return;
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  sprintf(buffer, "%s.%s=%d%s", name, compatible, val, END_OF_CMD); // 指令处理
  /*发送数据*/
  RS485_send(buffer, strlen(buffer));
}

/**
 * @description: raw api
 * @param {char} *name
 * @param {char} *compatible
 * @param {char} *str
 * @return {*}
 */
void command_set_comp_str_raw(const char *name, const char *compatible, const char *str)
{
  uart_init(115200);
  /*参数检查*/
  if (name == NULL || compatible == NULL)
    return;
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < sizeof(buffer) / sizeof(char); i++)
    buffer[i] = 0;

  sprintf(buffer, "%s.%s=%s%s", name, compatible, str, END_OF_CMD); // 指令处理
  /*发送数据*/
  RS485_send(buffer, strlen(buffer));
}

bool Page_id_get(void)
{
  uart_init(115200);
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < 50; i++)
    buffer[i] = 0;

  /*数据处理*/
  sprintf(buffer, "sendme%s", END_OF_CMD);
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();                // 进入临界区
  RS485_send(buffer, strlen(buffer)); // 发送数据
  OS_CRITICAL_EXIT();                 // 退出临界区

  OS_ERR err;
  OSSemPend(&PAGE_UPDATE_SEM, 100, OS_OPT_PEND_BLOCKING, NULL, &err);
  if (err == OS_ERR_NONE)
    return true;
  else
    return false;
}

/**
 * @description: specifies the page you want to change to
 * @param {Page_Param} *page_param
 * @param {Page_ID} id
 * @return {*}
 */
bool Page_to(const Page_Param *page_param, const Page_ID id)
{
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < 50; i++)
    buffer[i] = 0;

  /*已经处于该界面，不重复发送*/
  if (page_param->id == id)
    return true;

  /*发送数据*/
  sprintf(buffer, "page %d%s", id, END_OF_CMD); // 指令预处理
#if FAST_MODE == 1
  RS485_send(buffer, strlen(buffer)); // 发送数据
#else
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();                // 进入临界区
  RS485_send(buffer, strlen(buffer)); // 发送数据
  OS_CRITICAL_EXIT();                 // 退出临界区
#endif

  return true;
}

/**
 * @description: clear all protect alarm
 * @param {Page_Param} *page_param
 * @return {*}
 */
bool alram_clear(Page_Param *page_param)
{
  char *name_list[] = {
      "p4",
      "p5",
      "p6",
      "p7",
      "p8",
      "p9",
      "p10",
      "p11",
  };
  /*参数校验*/
  if (page_param == NULL || page_param->id != ALARM_PAGE)
    return false;
  uart_init(115200);
  /*清缓存*/
  for (uint16_t i = 0; i < USART_REC_LEN; i++)
    USART_RX_BUF[i] = 0;
  for (uint8_t i = 0; i < 50; i++)
    buffer[i] = 0;

  for (uint8_t i = 0; i < sizeof(name_list) / sizeof(char *); i++)
  {
    sprintf(buffer, "%s.aph=0%s", name_list[i], END_OF_CMD);
    RS485_send(buffer, strlen(buffer)); // 发送数据
  }

  return true;
}

/*绘图——绘点*/
void draw_point(uint16_t val)
{
  char pre_cmd[50] = "add wave_line.id,0,";
  char value_buf[5] = {0};
  user_value_convert_to_string(value_buf, 5, val);
  strcat(pre_cmd, value_buf);
  strcat(pre_cmd, END_OF_CMD);
  RS485_send(pre_cmd, strlen(pre_cmd));
}


#if ERR_DRAW
void draw_point_err(uint16_t val)
{
  char pre_cmd[50] = "add wave_line.id,1,";
  char value_buf[5] = {0};
  user_value_convert_to_string(value_buf, 5, val);
  strcat(pre_cmd, value_buf);
  strcat(pre_cmd, END_OF_CMD);
  RS485_send(pre_cmd, strlen(pre_cmd));
}
#endif

/**
 * @description: init the touch screen
 * @return {*}
 */
void Touchscreen_init(void)
{
  // 串口屏幕复位
  command_send_raw(CMD_RESET);
  //  等待触摸屏复位
  delay_ms(1000);
  command_send_raw(CMD_RESET);
  delay_ms(2000);
  // 使能指令成功返回指令
  command_set_comp_val_raw("bkcmd", "val", 1);
}
