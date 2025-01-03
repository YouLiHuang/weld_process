#ifndef _TOUCHSCREEN_H
#define _TOUCHSCREEN_H

#include "sys.h"
#include "includes.h"
#include "delay.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"

/*自定义的一些参量*/
#define NAME_LEN_MAX 20
#define MAX_LENGTH 100
#define END_FLAG_LEN 3
#define MAX_TIME 1000
#define MAX_RESEND 5

#define GP_MAX 19
#define TEMP_FACTOR 0.6375

#define CMD_PAGE "page "        // 翻页指令
#define CMD_GET "get "          // 获取控件属性指令
#define CMD_GET_PAGEID "sendme" // 读取页面id指令
#define CMD_RESET "rest"        // 复位指令
#define CMD_BACK "bkcmd"        // 设定指令返回指令

#define CMD_BUF_LEN 30
#define END_OF_CMD "\xff\xff\xff"
#define END_FLAG 0xff
#define MIN_CMD_LEN 5
#define CMD_BUFFER_LEN 50
/*------------------------------------------------------------------指令集------------------------------------------------------------------*/
#define CMD_FAIL 0x00
#define CMD_OK 0x01
#define CMD_ID_FAIL 0x02
#define CMD_PAGEID_FAIL 0x03
#define CMD_PICID_FAIL 0x04
#define CMD_FONTID_FAIL 0x05
#define CMD_FILEID_FAIL 0x06
#define CMD_CRC_FAIL 0x09
#define CMD_BAUD_FAIL 0x11
#define CMD_CURVEID_FAIL 0x12
#define CMD_VARNAME_FAIL 0x1A
#define CMD_VARCAL_FAIL 0x1B
#define CMD_VAREQUAL_FAIL 0x1C
#define CMD_POWEROFF_SAVE_FAIL 0x1D
#define CMD_PARAMNUM_FAIL 0x1E
#define CMD_IO_FAIL 0x1F
#define CMD_STRING_COVERT_FAIL 0x20
#define CMD_VARNAME_TOOLONG_FAIL 0x23
#define CMD_UARTMEM_FAIL 0x24

#define CMD_CLICK_RETURN 0x65 // 0X65+ 页面ID+按键ID+触摸事件+结束符 按下事件0x01弹起事件0X00
#define CMD_PAGEID_RETURN 0x66
#define CMD_COORDINATE_RETURN 0x67
#define CMD_SLEEP_MODE_EVENT 0x68
#define CMD_STR_VAR_RETURN 0x70
#define CMD_INT_VAR_RETURN 0x71

#define CMD_ENTER_SLEEP_MODE 0x86
#define CMD_WAKE_UP 0x87
#define CMD_SYSTEM_START_OK 0x88
#define CMD_SD_CARD_UPDATE_START 0x89
#define CMD_ALARM_RESET 0x90 // 这个为自定义的指令！！！
#define CMD_DATA_TRANSFER_OK 0xFD
#define CMD_DATA_TRANSFER_READY 0xFE

/*------------------------------------------------------------------报警组件名称------------------------------------------------------------------*/
#define SHOW_ON 127
#define SHOW_OFF 0
/*-------------------------控件属性集---------------------------*/

typedef enum
{
    PARAM_PAGE = 1,
    ALARM_PAGE = 2,
    WAVE_PAGE = 3,
    TEMP_PAGE = 4,
    UART_PAGE = 5
    /*...用户可扩展页面...*/

} Page_ID;

/*-------------------------按钮标志-------------------------*/
typedef enum RDY_SCH_STATE
{
    SCH = 27,
    RDY = 31
} RDY_SCH_STATE;

typedef enum ION_OFF_STATE
{
    ION = 28,
    IOFF = 25
} ION_OFF_STATE;

typedef enum SGW_CTW_STATE
{
    SGW = 30,
    CTW = 26
} SGW_CTW_STATE;

typedef enum UP_DOWN
{
    UP_CNT = 35,
    DOWN_CNT
} UP_DOWN;

typedef enum SENSOR_TYPE
{
    E_TYPE = 1,
    K_TYPE,
    J_TYPE
} SENSOR_TYPE;

/*-------------------------实时页面参数-------------------------*/
typedef struct page_param
{
    Page_ID id;

    RDY_SCH_STATE key1; /*ready sch*/
    ION_OFF_STATE key2; /*ion ioff*/
    SGW_CTW_STATE key3; /*sgw ctw*/
    uint8_t GP;
    int *user_buf_val;
    char *user_buf_str;
} Page_Param;

/*-------------------------任务状态-------------------------*/
typedef enum
{
    BUSY = 0x01,
    IDEAL = 0x02

} Task_State;

/*-------------------------数值型组件------------------------*/
typedef struct Component
{
    char *name;
    uint8_t index;
    int val;
    struct Component *next;
} Component;

typedef struct Component_Queue
{
    Page_ID id;
    int len;
    Component *updata; // 指向队列等待更新的元素
    Component *list_pointer;
} Component_Queue;

//////////////////////////////////////////触摸屏API////////////////////////////////////////
void RS485_send(const char *buffer, const u16 len);
/*初始化接口*/
void command_send_raw(const char *cmd);
void command_set_comp_val_raw(const char *name, const char *compatible, int val);
void command_set_comp_str_raw(const char *name, const char *compatible, const char *str);
/*其他接口*/
bool command_send(const char *cmd);
bool command_set_comp_val(const char *name, const char *compatible, const int val);
bool command_set_comp_str(const char *name, const char *compatible, const char *str);
bool command_get_comp_val(Component_Queue *list, const char *name, const char *compatible);
/*常用接口*/
void Touchscreen_init(void);
bool Page_to(const Page_Param *page_param, const Page_ID id);
void draw_point(u16 val);
bool alram_clear(Page_Param *page_param);
bool Page_id_get(void);
//////////////////////////////////控件数据对象API///////////////////////////////////////////

Page_Param *new_page_param(void);
void delete_page_param(Page_Param *p);

Component_Queue *newList(Page_ID id);
bool component_insert(Component_Queue *list, Component *comp);
void deleteComponent(Component *comp);
Component *get_comp(Component_Queue *list, const char *name);
Component *newComponet(const char *name, int val);

bool page_list_init(Component_Queue *page_list, char *name_list[], u8 list_len);
bool param_page_list_init(Component_Queue *param_page_list);
bool temp_page_list_init(Component_Queue *temp_page_list);
bool uart_page_list_init(Component_Queue *temp_page_list);

#endif
