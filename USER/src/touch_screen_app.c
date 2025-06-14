#include "touch_screen_app.h"
#include "user_config.h"

#include "modbus_app.h"
#include "welding_process.h"
#include "timer.h"
#include "adc.h"
#include "protect.h"
#include "spi.h"

/*A list of new interface components*/
// Record the ID of the current screen and the status of the three buttons
Page_Param *page_param = NULL;
// A list of components on the parameter setting screen
Component_Queue *param_page_list = NULL;
// A list of components for the temperature limit interface
Component_Queue *temp_page_list = NULL;
// A list of communication interface components
Component_Queue *setting_page_list = NULL;
// A list of components on the Waveform page
Component_Queue *wave_page_list = NULL;
/*Users can add the required component list as needed*/
/*......*/

static char *key_name_list[] = {"RDY_SCH", "ION_OFF", "SGW_CTW", "UP_DOWN"};

static char *weld_time_name_list[] = {
    "time1",
    "time2",
    "time3",
    "time4",
    "time5",
};
static char *weld_temp_name_list[] = {
    "temp1",
    "temp2",
    "temp3",
};

static char *alarm_temp_name_list[] = {
    "alarm1",
    "alarm2",
    "alarm3",
    "alarm4",
    "alarm5",
    "alarm6"};
static char *gain_name_list[] = {
    "GAIN1",
    "GAIN2"};

/*SEM -------------------------------------------------------------*/
extern OS_SEM SENSOR_UPDATE_SEM;
extern OS_SEM PLOT_SEM;
extern OS_SEM ERROR_HANDLE_SEM;

/*MUX -------------------------------------------------------------*/
/*UART4 Resource Protection*/
extern OS_MUTEX PLOT_Mux;

/*Modbus variables ----------------------------------------*/
extern uint8_t ID_OF_DEVICE;
extern OS_MUTEX ModBus_Mux;
extern uint16_t usRegInputBuf[REG_INPUT_NREGS];
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8];
extern uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];

/* Drawing controllers --------------------------------------------*/
extern Temp_draw_ctrl *temp_draw_ctrl;
/*Welding real-time controller-------------------------------------*/
extern weld_ctrl *weld_controller;
/*Date ------------------------------------------------------------*/
extern Date current_date;
/*Error controller ------------------------------------------------*/
extern Error_ctrl *err_ctrl;

/*Sensor ----------------------------------------------------------*/
extern Thermocouple *current_Thermocouple;
/*Plot buffer -----------------------------------------------------*/

extern uint16_t realtime_temp_buf[TEMP_BUF_MAX_LEN];

/*Functions prototype----------------------------------------------*/
static void TSkey_action_callback_param(Component_Queue *page_list);
static void TSkey_action_callback_temp(Component_Queue *page_list);
static void TSparse_key_action(Page_ID id);
static void TSModbus_Sync_FromUi(void);
static void TSSync_Date_from_Screen(Component_Queue *page_list);

/**
 * @description: real-time temp display
 * @return {*}
 */
static void TSTemp_updata_realtime()
{
    OS_ERR err;

    weld_controller->realtime_temp = temp_convert(current_Thermocouple);
    switch (current_Thermocouple->type)
    {
    case E_TYPE:
        if (page_param->id == WAVE_PAGE)
        {
            command_set_comp_val("step3", "val", weld_controller->realtime_temp);
        }
        else
        {
            command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
        }

        break;
    case K_TYPE:

        if (page_param->id == WAVE_PAGE)
        {
            command_set_comp_val("step3", "val", weld_controller->realtime_temp);
        }
        else
        {
            command_set_comp_val("temp33", "val", weld_controller->realtime_temp);
        }
        break;
    case J_TYPE:
        if (page_param->id == WAVE_PAGE)
        {
            command_set_comp_val("step33", "val", weld_controller->realtime_temp);
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
static void TSModbus_Sync_FromUi(void)
{
    uint8_t index = 0;
    OS_ERR err;

    OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    switch (page_param->id)
    {

    case TEMP_PAGE:
        for (index = HOLD_ADDR_0; index < HOLD_ADDR_7; index++)
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
                /*six alarm temp*/
            case HOLD_ADDR_6:
                usRegHoldingBuf[index] = weld_controller->temp_gain1 * 100;
                break;
            case HOLD_ADDR_7:
                usRegHoldingBuf[index] = weld_controller->temp_gain2 * 100;
                break;
            }
        }
        break;

    case PARAM_PAGE:

        /*...modbus sync...*/

        for (index = HOLD_ADDR_8; index < HOLD_ADDR_17; index++)
        {
            switch (index)
            {
            case HOLD_ADDR_8:
                usRegHoldingBuf[index] = weld_controller->weld_temp[0];
                break;
            case HOLD_ADDR_9:
                usRegHoldingBuf[index] = weld_controller->weld_temp[1];
                break;
            case HOLD_ADDR_10:
                usRegHoldingBuf[index] = weld_controller->weld_temp[2];
                break;
                /*five timr*/
            case HOLD_ADDR_11:
                usRegHoldingBuf[index] = weld_controller->weld_time[0];
                break;
            case HOLD_ADDR_12:
                usRegHoldingBuf[index] = weld_controller->weld_time[1];
                break;
            case HOLD_ADDR_13:
                usRegHoldingBuf[index] = weld_controller->weld_time[2];
                break;
            case HOLD_ADDR_14:
                usRegHoldingBuf[index] = weld_controller->weld_time[3];
                break;
            case HOLD_ADDR_15:
                usRegHoldingBuf[index] = weld_controller->weld_time[4];
                break;
            case HOLD_ADDR_16:
                usRegHoldingBuf[index] = weld_controller->weld_count;
                break;
                /*six alarm temp*/
            case HOLD_ADDR_17:
                usRegHoldingBuf[index] = get_comp(param_page_list, "GP")->val;
                break;
            }
        }

        break;
    }

    OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);
}

/**
 * @description: refresh user data according to data received from touch screen
 * @param {Component_Queue} *page_list
 * @return {*}
 */
static void TSSync_Date_from_Screen(Component_Queue *page_list)
{
    uint16_t temp_HL[6] = {0}, gain[2] = {0};
    uint16_t temp[3] = {0}, time[5] = {0};
    uint16_t current_conut;
    /*Ⅰ Screen --->  User Data*/
    switch (page_param->id)
    {
    case PARAM_PAGE:
        /*read dat from screen*/
        for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(page_list, weld_temp_name_list[i], "val");
        }
        for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(page_list, weld_time_name_list[i], "val");
        }

        command_get_comp_val(page_list, "count", "val");

        /*get data from list*/
        for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
        {
            time[i] = get_comp(page_list, weld_time_name_list[i])->val;
        }
        for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
        {
            temp[i] = get_comp(page_list, weld_temp_name_list[i])->val;
        }

        current_conut = get_comp(page_list, "count")->val;

        /*data sync*/
        for (uint8_t i = 0; i < sizeof(temp) / sizeof(temp[0]); i++)
        {
            weld_controller->weld_temp[i] = temp[i];
        }
        for (uint8_t i = 0; i < sizeof(time) / sizeof(time[0]); i++)
        {
            weld_controller->weld_time[i] = time[i];
        }

        if (current_conut != weld_controller->weld_count)
        {
            weld_controller->weld_count = current_conut;
        }

        weld_controller->Count_Dir = get_comp(page_list, "UP_DOWN")->val == UP_CNT ? UP : DOWN;

        /*save data*/
        if (get_comp(page_list, "GP"))
        {
            save_param(weld_controller,
                       get_comp(page_list, "GP")->val,
                       temp,
                       sizeof(temp) / sizeof(uint16_t),
                       time,
                       sizeof(time) / sizeof(uint16_t));
        }

        break;

    case TEMP_PAGE:

        /*Ⅰ、读取界面上的参数*/
        for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
        {
            /*参数读取*/
            command_get_comp_val(page_list, gain_name_list[i], "val");
        }
        for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
        {
            /*参数读取*/
            command_get_comp_val(page_list, alarm_temp_name_list[i], "val");
        }

        /*get data from list*/
        for (uint8_t i = 0; i < sizeof(temp_HL) / sizeof(uint16_t); i++)
        {
            temp_HL[i] = get_comp(page_list, alarm_temp_name_list[i])->val;
        }
        for (uint8_t i = 0; i < sizeof(gain) / sizeof(uint16_t); i++)
        {
            gain[i] = get_comp(page_list, gain_name_list[i])->val;
        }

        /*data sync*/
        for (uint8_t i = 0; i < sizeof(temp_HL) / sizeof(temp_HL[0]); i++)
        {
            weld_controller->alarm_temp[i] = temp_HL[i];
        }
        weld_controller->temp_gain1 = gain[0] / 100.0;
        weld_controller->temp_gain2 = gain[1] / 100.0;

        if (current_conut != weld_controller->weld_count)
        {
            weld_controller->weld_count = current_conut;
        }

        weld_controller->Count_Dir = get_comp(page_list, "UP_DOWN")->val == UP_CNT ? UP : DOWN;

        /*save data to eeprom*/
        if (get_comp(page_list, "GP") != NULL)
        {
            save_param_alarm(weld_controller,
                             get_comp(page_list, "GP")->val,
                             temp_HL,
                             sizeof(temp_HL) / sizeof(uint16_t),
                             gain);
        }

        break;
    }

    /*Ⅱ User Data --->  Modbus*/
    /*sync data to Modbus buffer*/
    TSModbus_Sync_FromUi();
}

/**
 * @description: key callback
 * @param {Component_Queue} *page_list
 * @return {*}
 */
static void TSkey_action_callback_param(Component_Queue *page_list)
{

    uint16_t count;
    uint8_t GP;

    // get key
    Component *comp = get_comp(page_list, "RDY_SCH");

    // 1、SCH——>RDY：exit change mode
    // 2、RDY——>RDY：no action
    if (RDY == comp->val)
    {
        // 1、SCH——>RDY：退出修改模式，保存用户修改的参数
        if (comp->val != page_param->key1)
        {
            TSSync_Date_from_Screen(page_list);
        }
        // 2、RDY——>RDY：无动作/根据GP值加载参数
        else
        {
            GP = get_comp(page_list, "GP")->val;
            if (GP != page_param->GP && GP <= GP_MAX)
                Load_param(weld_controller, GP);

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
    }
    // 处于修改模式————进入修改模式
    // 3、RDY——>SCH：进入修改模式
    // 4、SCH——>SCH：正在修改参数
    else if (SCH == comp->val)
    {
        /*GP change by user */
        GP = get_comp(page_list, "GP")->val;
        if (GP != page_param->GP && GP <= GP_MAX)
        {

            Load_param(weld_controller, GP);

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

        /*default - read data from screen*/
        for (uint8_t i = 0; i < sizeof(weld_temp_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(page_list, weld_temp_name_list[i], "val");
        }
        for (uint8_t i = 0; i < sizeof(weld_time_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(page_list, weld_time_name_list[i], "val");
        }

        /*get count , which would be changed by user*/
        command_get_comp_val(page_list, "count", "val");
        count = get_comp(page_list, "count")->val;
        if (weld_controller->weld_count != count)
            weld_controller->weld_count = count;
    }
    // status sync
    page_param->GP = get_comp(page_list, "GP")->val;
    page_param->key1 = (RDY_SCH_STATE)get_comp(page_list, "RDY_SCH")->val;
    page_param->key2 = (ION_OFF_STATE)get_comp(page_list, "ION_OFF")->val;
    page_param->key3 = (SGW_CTW_STATE)get_comp(page_list, "SGW_CTW")->val;
}

/**
 * @description: key callback
 * @param {Component_Queue} *page_list
 * @return {*}
 */
static void TSkey_action_callback_temp(Component_Queue *page_list)
{
    uint16_t count;
    uint16_t GP;

    // get key
    Component *comp = get_comp(page_list, "RDY_SCH");

    // 1、SCH——>RDY：exit change mode
    // 2、RDY——>RDY：no action
    if (RDY == comp->val)
    {
        // 1、SCH——>RDY：exit change mode
        if (comp->val != page_param->key1)
        {
            TSSync_Date_from_Screen(page_list);
        }
        // 2、RDY——>RDY：no action
        else if (comp->val == page_param->key1 && get_comp(page_list, "GP")->val <= GP_MAX)
        {
            /*GP change by user*/
            GP = get_comp(page_list, "GP")->val;
            if (GP != page_param->GP && GP <= GP_MAX)
                Load_param_alarm(weld_controller, GP);

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
    }

    // 3、RDY——>SCH：enter change mode
    // 4、SCH——>SCH：change param...
    else if (SCH == comp->val)
    {

        /*Ⅰ、根据GP加载参数*/
        GP = get_comp(page_list, "GP")->val; // 当前设定的GP值GP值和上次不一致 降低内存读写次数
        if (GP != page_param->GP && GP <= GP_MAX)
        {
            Load_param_alarm(weld_controller, GP);

            /*send data to screen*/
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

        /*default read data from screen*/
        for (uint8_t i = 0; i < sizeof(gain_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(page_list, gain_name_list[i], "val");
        }
        for (uint8_t i = 0; i < sizeof(alarm_temp_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(page_list, alarm_temp_name_list[i], "val");
        }

        /*get count , which would be changed by user*/
        command_get_comp_val(param_page_list, "count", "val");
        count = get_comp(param_page_list, "count")->val;
        if (weld_controller->weld_count != count)
            weld_controller->weld_count = count;
    }

    // 状态同步
    page_param->GP = get_comp(page_list, "GP")->val;
    page_param->key1 = (RDY_SCH_STATE)get_comp(page_list, "RDY_SCH")->val;
    page_param->key2 = (ION_OFF_STATE)get_comp(page_list, "ION_OFF")->val;
    page_param->key3 = (SGW_CTW_STATE)get_comp(page_list, "SGW_CTW")->val;
}

/**
 * @description: key callback
 * @param {Page_ID} id
 * @return {*}
 */
static void TSparse_key_action(Page_ID id)
{
    switch (id)
    {
    case PARAM_PAGE:
        TSkey_action_callback_param(param_page_list);
        break;
    case TEMP_PAGE:
        TSkey_action_callback_temp(temp_page_list);
        break;
    case WAVE_PAGE:
        /*...*/
        break;
    case ALARM_PAGE:
        /*...*/
        break;
    case UART_PAGE:
        /*...*/
        break;
    default:
        break;
    }
}
#if PID_DEBUG == 1
static bool pid_param_get(uint16_t *pid_raw_param)
{
    const char *pid_param_name_list[] = {
        "kp",
        "ki",
        "kd"};
    uint8_t cnt = 0;

    for (uint8_t i = 0; i < sizeof(pid_param_name_list) / sizeof(char *); i++)
    {
        if (command_get_comp_val(wave_page_list, pid_param_name_list[i], "val") == true)
            cnt++;
    }
    if (cnt == sizeof(pid_param_name_list) / sizeof(char *))
    {
        pid_raw_param[0] = get_comp(wave_page_list, "kp")->val;
        pid_raw_param[1] = get_comp(wave_page_list, "ki")->val;
        pid_raw_param[2] = get_comp(wave_page_list, "kd")->val;
        return true;
    }
    else
        return false;
}
#endif

/**
 * @description: UI page processing
 * @param {Page_ID} id
 * @return {*}
 */
void TSpage_process(Page_ID id)
{
    char *tick_name[] = {"tick1", "tick2", "tick3", "tick4", "tick5"};
    char *temp_display_name[] = {"temp11", "temp22", "temp33"};
    switch (page_param->id)
    {
    case PARAM_PAGE:
    {
        /*get keys status*/
        for (uint8_t i = 0; i < sizeof(key_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(param_page_list, key_name_list[i], "pic");
        }
        weld_controller->Count_Dir = (get_comp(param_page_list, "UP_DOWN")->val == UP_CNT) ? UP : DOWN;

        /*get current array number*/
        command_get_comp_val(param_page_list, "GP", "val");

        TSparse_key_action(page_param->id);

        command_set_comp_val("count", "val", weld_controller->weld_count);

        /*display average temperature*/
        command_set_comp_val(temp_display_name[0], "val", temp_draw_ctrl->display_temp[0]);
        command_set_comp_val(temp_display_name[1], "val", temp_draw_ctrl->display_temp[1]);

        /*get current data*/
        command_get_variable_val(&current_date.Year, "rtc0");
        command_get_variable_val(&current_date.Month, "rtc1");
        command_get_variable_val(&current_date.Day, "rtc2");
        command_get_variable_val(&current_date.Hour, "rtc3");
        command_get_variable_val(&current_date.Minute, "rtc4");

        /*display Real-time temperature*/
        TSTemp_updata_realtime();
    }
    break;

    case TEMP_PAGE:
    {
        /*get keys status*/
        for (uint8_t i = 0; i < sizeof(key_name_list) / sizeof(char *); i++)
        {
            command_get_comp_val(temp_page_list, key_name_list[i], "pic");
        }
        weld_controller->Count_Dir = (get_comp(temp_page_list, "UP_DOWN")->val == UP_CNT) ? UP : DOWN;

        /*get current array number*/
        command_get_comp_val(temp_page_list, "GP", "val");
        /*get count , which would be changed by user*/
        command_get_comp_val(temp_page_list, "count", "val");
        uint16_t count = get_comp(temp_page_list, "count")->val;
        if (count != weld_controller->weld_count)
            weld_controller->weld_count = count;

        /*get switch status*/
        command_get_comp_val(temp_page_list, "switch", "val");

        TSparse_key_action(page_param->id);

        /*display average temperature*/
        command_set_comp_val(temp_display_name[0], "val", temp_draw_ctrl->display_temp[0]);
        command_set_comp_val(temp_display_name[1], "val", temp_draw_ctrl->display_temp[1]);

        command_set_comp_val("count", "val", weld_controller->weld_count);

        /*display Real-time temperature*/
        TSTemp_updata_realtime();
    }
    break;

    case WAVE_PAGE:
    {
#if PID_DEBUG == 1
        command_set_comp_val("wave_page.kp", "aph", 127);
        command_set_comp_val("wave_page.ki", "aph", 127);
        command_set_comp_val("wave_page.kd", "aph", 127);
        uint16_t pid_param[3] = {0};
        if (pid_param_get(pid_param) == true)
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

        /*updata axis*/
        for (uint8_t i = 0; i < 5; i++)
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
        TSTemp_updata_realtime();
        OSMutexPost(&PLOT_Mux, OS_OPT_POST_NONE, &err);

        /*weld over, enter wave page, then plot temp line*/
        OSSemPend(&PLOT_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
        if (err == OS_ERR_NONE)
        {
            uint16_t delta = 0;
            /*后续改为透传模式！！！*/
            if (temp_draw_ctrl->third_step_index_start % win_width == 0)
                delta = temp_draw_ctrl->third_step_index_start / win_width;
            else
                delta = temp_draw_ctrl->third_step_index_start / win_width + 1;

            for (uint16_t i = 0; i < temp_draw_ctrl->third_step_index_end; i += delta)
            {
                draw_point(realtime_temp_buf[i]);
            }
        }
    }
    break;

    case ALARM_PAGE:
    {
    }
    break;

    case UART_PAGE:
    {
        OS_ERR err;
        Component *comp = NULL;
        /*1、读取界面设定的地址*/
        command_get_comp_val(setting_page_list, "adress", "val");
        /*2、读取页面设定的波特率*/
        command_get_comp_val(setting_page_list, "baudrate", "val");
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

        /*4、订阅热电偶校准信号*/
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
        /*5、显示实时温度*/
        command_set_comp_val("temp33", "val", weld_controller->realtime_temp);

        /*6、数据同步*/
        comp = get_comp(setting_page_list, "adress");
        if (comp != NULL)
            ID_OF_DEVICE = comp->val;

        comp = get_comp(setting_page_list, "baudrate");
        if (comp != NULL)
        {
            uint8_t index = get_comp(setting_page_list, "baudrate")->val;
            /*set bounds*/
        }

        /*display Real-time temperature*/
        TSTemp_updata_realtime();
    }
    break;

    case KEY_INPUT_PAGE:
        break;

    default:
        /*...用户可自行添加需要的页面...*/
        break;
    }
}
