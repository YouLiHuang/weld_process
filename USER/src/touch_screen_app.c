/*user include ---------------------------------------------------*/
#include "touch_screen_app.h"
#include "user_config.h"
#include "modbus_app.h"
#include "welding_process.h"

/*bsp include ----------------------------------------------------*/
#include "timer.h"
#include "adc.h"
#include "protect.h"
#include "spi.h"

/*FreeModbus includes*/
#include "mb.h"
#include "mbport.h"
#include "mbrtu.h"
#include "port_bsp.h"
#include "modbus_app.h"

/*--------------------user variables include-----------------------*/
/*SEM */
extern OS_SEM SENSOR_UPDATE_SEM;
extern OS_SEM PLOT_SEM;
extern OS_SEM ERROR_HANDLE_SEM;
/*MUX */
/*UART4 Resource Protection*/
extern OS_MUTEX PLOT_Mux;
extern OS_MUTEX ModBus_Mux;
/*Modbus variables*/
extern uint8_t ID_OF_DEVICE;
extern uint16_t usRegInputBuf[REG_INPUT_NREGS];
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8];
extern uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];
/* Drawing controllers */
extern Temp_draw_ctrl *temp_draw_ctrl;
/*Welding real-time controller*/
extern weld_ctrl *weld_controller;
/*Date */
extern Date current_date;
/*Error controller */
extern Error_ctrl *err_ctrl;
/*Sensor */
extern Thermocouple *current_Thermocouple;
/*debug */
#if PID_DEBUG
extern pid_feedforword_ctrl *pid_ctrl_debug;
#endif
/*key */
extern uint8_t cur_GP;
extern RDY_SCH_STATE cur_key1;
extern ION_OFF_STATE cur_key2;
extern SGW_CTW_STATE cur_key3;
extern SWITCH_STATE switch_mode;

/*-------------------------APP variables---------------------------*/
/*page manager*/
static Page_Manager page_manger;
/*compnent list init name*/
static char *temp_page_name_list[] = {
    "alarm1",
    "alarm2",
    "alarm3",
    "alarm4",
    "alarm5",
    "alarm6",
    "GAIN1",
    "GAIN2",
    "RDY_SCH",
    "ION_OFF",
    "SGW_CTW",
    "UP_DOWN",
    "switch",
    "count",
    "GP"};

static char *param_page_name_list[] = {
    "temp1",
    "temp2",
    "temp3",
    "time1",
    "time2",
    "time3",
    "time4",
    "time5",
    "time6",
    "RDY_SCH",
    "ION_OFF",
    "SGW_CTW",
    "UP_DOWN",
    "count",
    "GP",
    "rtc0",
    "rtc1",
    "rtc2",
    "rtc3",
    "rtc4",
};

static char *setting_page_name_list[] = {
    "adress",
    "baudrate",
    "sensortype"};

static char *wave_page_name_list[] = {
    "kp",
    "ki",
    "kd"};

/*...Users can add the component list as needed...*/

/*internal Functions prototype--------------------------------------*/
static void TSModbus_Sync_FromUi(Page_ID id);
static void TSSync_Date_from_Screen(Component_Queue *page_list);
static void TSTemp_updata_realtime(Page_ID id);

/*User Page Callback Functions--------------------------------------*/
static void TSparam_pg_cb(Page_ID id);
static void TStemp_pg_cb(Page_ID id);
static void TSwave_pg_cb(Page_ID id);
static void TSsetting_pg_cb(Page_ID id);
/*...Users can add callback as needed...*/

/*match list -------------------------------------------------------*/
page_map PAGE_CB_MAP[] = {
    {PARAM_PAGE, NULL, TSparam_pg_cb, param_page_name_list, sizeof(param_page_name_list) / sizeof(char *)},
    {TEMP_PAGE, NULL, TStemp_pg_cb, temp_page_name_list, sizeof(temp_page_name_list) / sizeof(char *)},
    {UART_PAGE, NULL, TSsetting_pg_cb, setting_page_name_list, sizeof(setting_page_name_list) / sizeof(char *)},
    {WAVE_PAGE, NULL, TSwave_pg_cb, wave_page_name_list, sizeof(wave_page_name_list) / sizeof(char *)},
};

/*other variables-----------------------------------------------------*/

extern uint8_t ID_OF_DEVICE;
extern uint32_t Baud_Rate_Modbus;

const uint32_t baud_rate_list[12] = {
    2400,
    4800,
    9600,
    19200,
    31250,
    57600,
    115200,
    230400,
    250000,
    256000,
    512000,
    821600};

/**
 * @description: UI page processing
 * @param {Page_ID} id
 * @return {*}
 */
void TSpage_process(Page_ID id)
{

    uint8_t index = 0;
    Component_Queue *list;
    /*match the page list*/
    for (index = 0; index < sizeof(PAGE_CB_MAP) / sizeof(PAGE_CB_MAP[0]); index++)
    {
        list = PAGE_CB_MAP[index].que;
        if (list->id == id)
            PAGE_CB_MAP[index].pg_cb(id);
    }
}

/**
 * @description: page manager init
 * @param {page_map} *map
 * @return {*}
 */
bool PGManager_init(void)
{
    /*page init*/
    for (uint8_t i = 0; i < sizeof(PAGE_CB_MAP) / sizeof(PAGE_CB_MAP[0]); i++)
    {

        if (PAGE_CB_MAP[i].que == NULL)
        {
            PAGE_CB_MAP[i].que = newList(PAGE_CB_MAP[i].id);
        }
        page_list_init(PAGE_CB_MAP[i].que,
                       PAGE_CB_MAP[i].init_name_list,
                       PAGE_CB_MAP[i].list_len);
    }

    return true;
}

/**
 * @description: request manager
 * @return {*}
 */
Page_Manager *request_PGManger(void)
{
    return &page_manger;
}

/**
 * @description:get page list
 * @return {*}
 */
Component_Queue *get_page_list(Page_ID id)
{
    uint8_t index;
    /*match the page list*/
    for (index = 0; index < sizeof(PAGE_CB_MAP) / sizeof(PAGE_CB_MAP[0]); index++)
    {
        if (PAGE_CB_MAP[index].que->id == id)
            break;
    }

    return PAGE_CB_MAP[index].que;
}

/**
 * @description: real-time temp display
 * @return {*}
 */
static void TSTemp_updata_realtime(Page_ID id)
{
    OS_ERR err;

    weld_controller->realtime_temp = temp_convert(current_Thermocouple);
    switch (current_Thermocouple->type)
    {
    case E_TYPE:
        if (id == WAVE_PAGE)
        {
            command_set_comp_val("step3", "val", weld_controller->realtime_temp);
        }
        else
        {
            command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
        }

        break;
    case K_TYPE:

        if (id == WAVE_PAGE)
        {
            command_set_comp_val("step3", "val", weld_controller->realtime_temp);
        }
        else
        {
            command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
        }
        break;
    case J_TYPE:
        if (id == WAVE_PAGE)
        {
            command_set_comp_val("step3", "val", weld_controller->realtime_temp);
        }
        else
        {
            command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
        }
        break;

    default:
        err_get_type(err_ctrl, SENSOR_ERROR)->state = true;
        OSSemPost(&ERROR_HANDLE_SEM, OS_OPT_POST_1, &err);
        break;
    }
}

/**
 * @description: refresh modbus buffer according to data received from touch screen
 * @return {*}
 */
static void TSModbus_Sync_FromUi(Page_ID id)
{
    uint8_t index = 0;
    OS_ERR err;

    OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    for (index = HOLD_ADDR_0; index < HOLD_ADDR_17; index++)
    {
        switch (index)
        {
            /*six alarm temp*/
        case HOLD_ADDR_0:
            usRegHoldingBuf[index] = weld_controller->alarm_temp[0];
            break;
        case HOLD_ADDR_1:
            usRegHoldingBuf[index] = weld_controller->alarm_temp[1];
            break;
        case HOLD_ADDR_2:
            usRegHoldingBuf[index] = weld_controller->alarm_temp[2];
            break;
        case HOLD_ADDR_3:
            usRegHoldingBuf[index] = weld_controller->alarm_temp[3];
            break;
        case HOLD_ADDR_4:
            usRegHoldingBuf[index] = weld_controller->alarm_temp[4];
            break;
        case HOLD_ADDR_5:
            usRegHoldingBuf[index] = weld_controller->alarm_temp[5];
            break;
            /*two gain*/
        case HOLD_ADDR_6:
            usRegHoldingBuf[index] = weld_controller->temp_gain1 * 100;
            break;
        case HOLD_ADDR_7:
            usRegHoldingBuf[index] = weld_controller->temp_gain2 * 100;
            break;
        case HOLD_ADDR_8:
            usRegHoldingBuf[index] = weld_controller->weld_temp[0];
            break;
        case HOLD_ADDR_9:
            usRegHoldingBuf[index] = weld_controller->weld_temp[1];
            break;
        case HOLD_ADDR_10:
            usRegHoldingBuf[index] = weld_controller->weld_temp[2];
            break;
            /*five time*/
        case HOLD_ADDR_11:
            usRegHoldingBuf[index] = weld_controller->weld_time[0];
            break;
        case HOLD_ADDR_12:
            usRegHoldingBuf[index] = weld_controller->weld_time[1];
            break;
        case HOLD_ADDR_13:
            usRegHoldingBuf[index] = weld_controller->weld_time[3];
            break;
        case HOLD_ADDR_14:
            usRegHoldingBuf[index] = weld_controller->weld_time[4];
            break;
        case HOLD_ADDR_15:
            usRegHoldingBuf[index] = weld_controller->weld_time[5];
            break;
        case HOLD_ADDR_16:
            usRegHoldingBuf[index] = weld_controller->weld_count;
            break;
        case HOLD_ADDR_17:
            usRegHoldingBuf[index] = cur_GP;
            break;

        default:
            break;
        }
    }

    OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);
}

static void TSSync_Date_from_Screen(Component_Queue *page_list)
{
    Page_ID id = request_PGManger()->id;
    /*save data*/
    Save_Param_toDisk();
    /*User Data --->  Modbus buffer*/
    /*sync data to Modbus buffer*/
    TSModbus_Sync_FromUi(id);
}

#if PID_DEBUG == 1
static bool TSpid_param_get(uint16_t *pid_raw_param)
{

    bool ret = true;
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;

    Component_Queue *list = get_page_list(WAVE_PAGE);
    ret = command_get_comp_val(list, "kp", "val");
    ret = command_get_comp_val(list, "ki", "val");
    ret = command_get_comp_val(list, "kd", "val");

    kp = get_comp(list, "kp")->val;
    ki = get_comp(list, "ki")->val;
    kd = get_comp(list, "kd")->val;

    if (ret == true)
    {
        pid_raw_param[0] = kp;
        pid_raw_param[1] = ki;
        pid_raw_param[2] = kd;
    }

    return ret;
}
#endif

/**
 * @description: param page callback
 * @param {Page_ID} id
 * @return {*}
 */
static void TSparam_pg_cb(Page_ID id)
{

    OS_ERR err;
    uint8_t index;
    Component_Queue *list;
    static RDY_SCH_STATE last_key1 = RDY;
    static uint8_t last_gp = 0;

    const char *key_name_list[] = {"RDY_SCH", "ION_OFF", "SGW_CTW", "UP_DOWN"};
    const char *weld_time_name_list[] = {
        "time1",
        "time2",
        "time3",
        "time4",
        "time5",
        "time6"};
    const char *weld_temp_name_list[] = {
        "temp1",
        "temp2",
        "temp3",
    };

    /*match the page list*/
    for (index = 0; index < sizeof(PAGE_CB_MAP) / sizeof(PAGE_CB_MAP[0]); index++)
    {
        list = PAGE_CB_MAP[index].que;
        if (list->id == id)
            break;
    }

    /*record last keys status*/
    last_key1 = cur_key1;
    last_gp = cur_GP;

    /*----------------------------------------- update components compatible -----------------------------------------*/
    /*get value from screen*/
    for (uint8_t i = 0; i < sizeof(key_name_list) / sizeof(key_name_list[0]); i++)
    {
        command_get_comp_val(list, key_name_list[i], "pic");
    }
    command_get_comp_val(list, "GP", "val");

    /*data convert*/
    cur_key1 = (RDY_SCH_STATE)get_comp(list, "RDY_SCH")->val;
    cur_key2 = (ION_OFF_STATE)get_comp(list, "ION_OFF")->val;
    cur_key3 = (SGW_CTW_STATE)get_comp(list, "SGW_CTW")->val;
    weld_controller->Count_Dir = (get_comp(list, "UP_DOWN")->val == UP_CNT) ? UP : DOWN;
    cur_GP = get_comp(list, "GP")->val;

    /*get current data*/
    command_get_variable_val(&current_date.Year, "rtc0");
    command_get_variable_val(&current_date.Month, "rtc1");
    command_get_variable_val(&current_date.Day, "rtc2");
    command_get_variable_val(&current_date.Hour, "rtc3");
    command_get_variable_val(&current_date.Minute, "rtc4");

    /*------------------------------------------- handle key action start ----------------------------------------------*/
    // 1、SCH——>RDY：exit change mode
    // 2、RDY——>RDY：no action
    if (RDY == cur_key1) // current status is RDY
    {
        // 1、SCH——>RDY：exit SCH mode
        if (cur_key1 != last_key1)
        {
            TSSync_Date_from_Screen(list);
        }
        // 2、RDY——>RDY：no action
        else
        {
            /*gp change by user*/
            if (cur_GP != last_gp && cur_GP <= GP_MAX)
                Load_param(weld_controller, cur_GP);

            /*send data to screen*/
            for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
            {
                command_set_comp_val(weld_time_name_list[i], "val",
                                     weld_controller->weld_time[i]);
            }

            for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
            {
                command_set_comp_val(weld_temp_name_list[i], "val",
                                     weld_controller->weld_temp[i]);
            }
            /*sync weld count to screen*/
            command_set_comp_val("count", "val", weld_controller->weld_count);
        }
    }

    // 3、RDY——>SCH：enter SCH MODE
    // 4、SCH——>SCH：in SCH mode
    else if (SCH == cur_key1)
    {
        /*GP change by user */
        if (cur_GP != last_gp && cur_GP <= GP_MAX)
        {
            Load_param(weld_controller, cur_GP);
            /*send data to screen*/
            for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
            {
                command_set_comp_val(weld_time_name_list[i], "val",
                                     weld_controller->weld_time[i]);
            }

            for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
            {
                command_set_comp_val(weld_temp_name_list[i], "val",
                                     weld_controller->weld_temp[i]);
            }
        }

        /*read data from screen*/
        for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(list, weld_temp_name_list[i], "val");
        }
        for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(list, weld_time_name_list[i], "val");
        }

        for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
        {
            weld_controller->weld_temp[i] = get_comp(list, weld_temp_name_list[i])->val;
        }
        for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
        {
            weld_controller->weld_time[i] = get_comp(list, weld_time_name_list[i])->val;
        }

        command_get_comp_val(list, "count", "val");
        if (weld_controller->weld_count != get_comp(list, "count")->val)
            weld_controller->weld_count = get_comp(list, "count")->val;
    }

    /*---------------------------------------------handle key action end-------------------------------------------------*/
    /*display average temperature*/
    command_set_comp_val("temp11", "val", temp_draw_ctrl->display_temp[0]);
    command_set_comp_val("temp22", "val", temp_draw_ctrl->display_temp[1]);

    /*display Real-time temperature*/
    OSMutexPend(&PLOT_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
    TSTemp_updata_realtime(id);
    OSMutexPost(&PLOT_Mux, OS_OPT_POST_NONE, &err);
}

/**
 * @description: temp page callback
 * @param {Page_ID} id
 * @return {*}
 */
static void TStemp_pg_cb(Page_ID id)
{
    OS_ERR err;
    uint8_t index;
    Component_Queue *list;
    static RDY_SCH_STATE last_key1 = RDY;
    static uint8_t last_gp = 0;

    const char *key_name_list[] = {"RDY_SCH", "ION_OFF", "SGW_CTW", "UP_DOWN"};
    const char *val_name_list[] = {"GP", "switch"};
    const char *alarm_temp_name_list[] = {
        "alarm1",
        "alarm2",
        "alarm3",
        "alarm4",
        "alarm5",
        "alarm6"};
    const char *gain_name_list[] = {
        "GAIN1",
        "GAIN2"};

    /*match the page list*/
    for (index = 0; index < sizeof(PAGE_CB_MAP) / sizeof(PAGE_CB_MAP[0]); index++)
    {
        list = PAGE_CB_MAP[index].que;
        if (list->id == id)
            break;
    }

    /*record last keys status*/
    last_key1 = cur_key1;
    last_gp = cur_GP;

    /*----------------------------------------- update components compatible -----------------------------------------*/
    /*get value from screen*/
    for (uint8_t i = 0; i < sizeof(key_name_list) / sizeof(key_name_list[0]); i++)
    {
        command_get_comp_val(list, key_name_list[i], "pic");
    }
    for (uint8_t i = 0; i < sizeof(val_name_list) / sizeof(val_name_list[0]); i++)
    {
        command_get_comp_val(list, val_name_list[i], "val");
    }

    /*data convert*/
    cur_key1 = (RDY_SCH_STATE)get_comp(list, "RDY_SCH")->val;
    cur_key2 = (ION_OFF_STATE)get_comp(list, "ION_OFF")->val;
    cur_key3 = (SGW_CTW_STATE)get_comp(list, "SGW_CTW")->val;
    weld_controller->Count_Dir = (get_comp(list, "UP_DOWN")->val == UP_CNT) ? UP : DOWN;
    switch_mode = (SWITCH_STATE)(get_comp(list, "switch")->val == 1) ? AUTO_MODE : USER_MODE;
    cur_GP = get_comp(list, "GP")->val;

    /*------------------------------------------- handle key action start ---------------------------------------------*/

    // 1、SCH——>RDY：exit change mode
    // 2、RDY——>RDY：no action
    if (RDY == cur_key1)
    {
        // 1、SCH——>RDY：exit change mode
        if (cur_key1 != last_key1)
        {
            TSSync_Date_from_Screen(list);
        }
        // 2、RDY——>RDY：no action
        else if (cur_key1 == last_key1 && cur_GP <= GP_MAX)
        {
            /*GP change by user*/
            if (cur_GP != last_gp && cur_GP <= GP_MAX)
                Load_param_alarm(weld_controller, cur_GP);

            /*send data to screen*/
            for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(alarm_temp_name_list[0]); i++)
            {
                command_set_comp_val(alarm_temp_name_list[i], "val",
                                     weld_controller->alarm_temp[i]);
            }

            float gain_float = 0.0;
            float gain_delta = 0.0;
            uint16_t gain_int = 0;

            gain_float = weld_controller->temp_gain1 * 100.0;
            gain_int = gain_float;
            gain_delta = gain_float - gain_int;
            if (gain_delta >= 0.5)
            {
                command_set_comp_val(gain_name_list[0], "val",
                                     gain_int + 1);
            }
            else
            {
                command_set_comp_val(gain_name_list[0], "val",
                                     gain_int);
            }

            gain_float = weld_controller->temp_gain2 * 100.0;
            gain_int = gain_float;
            gain_delta = gain_float - gain_int;
            if (gain_delta >= 0.5)
            {
                command_set_comp_val(gain_name_list[1], "val",
                                     gain_int + 1);
            }
            else
            {
                command_set_comp_val(gain_name_list[1], "val",
                                     gain_int);
            }

            /*sync weld count to screen*/
            command_set_comp_val("count", "val", weld_controller->weld_count);
        }
    }

    // 3、RDY——>SCH：enter change mode
    // 4、SCH——>SCH：change param...
    else if (SCH == cur_key1)
    {

        if (cur_GP != last_gp && cur_GP <= GP_MAX)
        {
            Load_param_alarm(weld_controller, cur_GP);
            /*send data to screen*/
            for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(alarm_temp_name_list[0]); i++)
            {
                command_set_comp_val(alarm_temp_name_list[i], "val",
                                     weld_controller->alarm_temp[i]);
            }

            command_set_comp_val(gain_name_list[0], "val",
                                 weld_controller->temp_gain1 * 100);

            command_set_comp_val(gain_name_list[1], "val",
                                 weld_controller->temp_gain2 * 100);
        }

        /*read data from screen*/
        for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(list, gain_name_list[i], "val");
        }
        for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(list, alarm_temp_name_list[i], "val");
        }

        weld_controller->temp_gain1 = get_comp(list, gain_name_list[0])->val / 100.0;
        weld_controller->temp_gain2 = get_comp(list, gain_name_list[1])->val / 100.0;
        for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
        {
            weld_controller->alarm_temp[i] = get_comp(list, alarm_temp_name_list[i])->val;
        }

        command_get_comp_val(list, "count", "val");
        if (weld_controller->weld_count != get_comp(list, "count")->val)
        {
            weld_controller->weld_count = get_comp(list, "count")->val;
        }
    }

    /*------------------------------------------- handle key action end -------------------------------------------------*/

    /*display average temperature*/
    command_set_comp_val("temp11", "val", temp_draw_ctrl->display_temp[0]);
    command_set_comp_val("temp22", "val", temp_draw_ctrl->display_temp[1]);

    /*sync weld count to screen*/
    command_set_comp_val("count", "val", weld_controller->weld_count);

    /*display Real-time temperature*/
    OSMutexPend(&PLOT_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
    TSTemp_updata_realtime(id);
    OSMutexPost(&PLOT_Mux, OS_OPT_POST_NONE, &err);
}

/**
 * @description: wave page callback
 * @param {Page_ID} id
 * @return {*}
 */
static void TSwave_pg_cb(Page_ID id)
{
#if PID_DEBUG == 1
    Component_Queue *list = get_page_list(WAVE_PAGE);
    command_set_comp_val("kp", "aph", 127);
    command_set_comp_val("ki", "aph", 127);
    command_set_comp_val("kd", "aph", 127);
    uint16_t pid_param[3] = {0};
    if (TSpid_param_get(pid_param) == true)
    {
        pid_ctrl_debug->kp = pid_param[0] / 100.0;
        pid_ctrl_debug->ki = pid_param[1] / 1000.0;
        pid_ctrl_debug->kd = pid_param[2] / 100.0;
    }

#endif
    OS_ERR err;
    uint16_t total_time = 0;     // 总焊接时长
    uint16_t delta_tick = 0;     // 坐标间隔
    uint16_t total_tick_len = 0; // 横坐标总长度
    uint16_t win_width = 0;      // 绘图区域占据的实际窗口大小
    char *tick_name[] = {"tick1", "tick2", "tick3", "tick4", "tick5"};

    /*updata axis*/
    for (uint8_t i = 0; i < 6; i++)
        total_time += weld_controller->weld_time[i];
    /*坐标划分ms*/
    if (total_time <= 500)
        delta_tick = 100;
    else if (total_time > 500 && total_time <= 1000)
        delta_tick = 200;
    else if (total_time > 1000 && total_time <= 2500)
        delta_tick = 500;
    else if (total_time > 2500 && total_time <= 5000)
        delta_tick = 1000;
    else if (total_time > 5000 && total_time <= 10000)
        delta_tick = 2000;
    else if (total_time > 10000 && total_time <= 15000)
        delta_tick = 3000;
    else if (total_time > 15000 && total_time <= 20000)
        delta_tick = 4000;
    else
        delta_tick = 5000;

    /*plot interval*/
    total_tick_len = 5 * delta_tick;                     // total length of Horizontal axis
    win_width = WIN_WIDTH * total_time / total_tick_len; // Welding cycle drawing area occupies full screen ratio

    if (total_time == win_width)
        temp_draw_ctrl->delta_tick = 1;
    else
        temp_draw_ctrl->delta_tick = (total_time / win_width) + 1;

    /*Horizontal axis*/
    for (uint8_t i = 0; i < sizeof(tick_name) / sizeof(char *); i++)
        command_set_comp_val(tick_name[i], "val", (1 + i) * delta_tick);

    /*display three step average temp*/
    command_set_comp_val("step1", "val", temp_draw_ctrl->display_temp[0]);
    command_set_comp_val("step2", "val", temp_draw_ctrl->display_temp[1]);

    /*display Real-time temperature*/
    OSMutexPend(&PLOT_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
    TSTemp_updata_realtime(id);
    OSMutexPost(&PLOT_Mux, OS_OPT_POST_NONE, &err);

    /*weld over, enter wave page, then plot temp line*/
    OSSemPend(&PLOT_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
    if (err == OS_ERR_NONE)
    {
        uint16_t delta = 0;
        /*clear screen*/
        command_send("cle wave_line.id,0");
        OSTimeDly(10, OS_OPT_TIME_DLY, &err);
        /*后续改为透传模式！！！*/
        if (temp_draw_ctrl->third_step_index_start % win_width == 0)
            delta = temp_draw_ctrl->third_step_index_start / win_width;
        else
            delta = temp_draw_ctrl->third_step_index_start / win_width + 1;

        for (uint16_t i = 0; i < temp_draw_ctrl->third_step_index_end; i += delta)
        {
            draw_point(temp_draw_ctrl->temp_buf[i] * DRAW_AREA_HIGH / MAX_TEMP_DISPLAY);
        }
    }
}

/**
 * @description: setting page callback
 * @param {Page_ID} id
 * @return {*}
 */
static void TSsetting_pg_cb(Page_ID id)
{

    uint8_t index;
    Component_Queue *list;
    OS_ERR err;
    bool modbus_change = false;
    uint8_t new_adress = 0;
    uint8_t rate_index = 0;
    uint32_t new_rate = 115200;

    /*match the page list*/
    for (index = 0; index < sizeof(PAGE_CB_MAP) / sizeof(PAGE_CB_MAP[0]); index++)
    {
        list = PAGE_CB_MAP[index].que;
        if (list->id == id)
            break;
    }

    /*get data from screen*/
    command_get_comp_val(list, "adress", "val");
    command_get_comp_val(list, "baudrate", "val");

    /*3、设定热电偶类型*/
    switch (current_Thermocouple->type)
    {
    case K_TYPE:
        command_set_comp_val("KEJ", "val", 0);
        break;

    case E_TYPE:
        command_set_comp_val("KEJ", "val", 1);
        break;

    case J_TYPE:
        command_set_comp_val("KEJ", "val", 2);
        break;
    }

#if 0
    OSSemPend(&SENSOR_UPDATE_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
    if (OS_ERR_NONE == err)
    {
#if TEMP_ADJUST
        if (weld_controller->realtime_temp < 2 * ROOM_TEMP && weld_controller->realtime_temp > 0.5 * ROOM_TEMP)
            Thermocouple_err_eliminate();
        else // 焊头尚未冷却，警报
            command_set_comp_val("warning", "aph", 127);
        Thermocouple_err_eliminate();
#endif
    }
#endif

    /*display Real-time temperature*/
    OSMutexPend(&PLOT_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
    TSTemp_updata_realtime(id);
    OSMutexPost(&PLOT_Mux, OS_OPT_POST_NONE, &err);

    /*date sync*/
    new_adress = get_comp(list, "adress")->val;
    if (new_adress != ID_OF_DEVICE)
    {
        modbus_change = true;
        ID_OF_DEVICE = new_adress;
    }

    rate_index = get_comp(list, "baudrate")->val;
    if (rate_index < sizeof(baud_rate_list) / sizeof(baud_rate_list[0]))
    {
        new_rate = baud_rate_list[rate_index];
        if (new_rate != Baud_Rate_Modbus)
        {
            modbus_change = true;
            Baud_Rate_Modbus = new_rate;
        }
    }

    if (modbus_change)
    {
#if MODBUSSLAVE_ENABLE
        eMBInit(MB_RTU, new_adress, 3, new_rate, MB_PAR_NONE, 1);
        eMBEnable();
#endif
    }

    /*save new bsp to disk*/
    SPI_Save_Word(ID_OF_DEVICE, SLAVER_ADRESS);
    uint16_t rate_L = Baud_Rate_Modbus & 0xff;
    uint16_t rate_H = Baud_Rate_Modbus >> 16;
    SPI_Save_Word(rate_H, MODBUS_RATE_ADRESSH);
    SPI_Save_Word(rate_L, MODBUS_RATE_ADRESSL);

    /*display Real-time temperature*/
    OSMutexPend(&PLOT_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
    TSTemp_updata_realtime(id);
    OSMutexPost(&PLOT_Mux, OS_OPT_POST_NONE, &err);
}
