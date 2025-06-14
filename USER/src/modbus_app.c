#include "user_config.h"
#include "mb.h"
#include "mbutils.h"
#include "modbus_app.h"
#include "sys.h"
#include "thermocouple.h"
#include "adc.h"
#include "touchscreen.h"
#include "welding_process.h"
#include "protect.h"
#include "io_ctrl.h"

/* Private variables ---------------------------------------------------------*/
extern OS_MUTEX ModBus_Mux;
extern Thermocouple *current_Thermocouple;
extern Date current_date;
extern weld_ctrl *weld_controller;

extern Error_ctrl *err_ctrl;
extern START_TYPE start_type;
extern OS_SEM WELD_START_SEM;

/*key status*/
extern uint8_t cur_GP;
extern RDY_SCH_STATE cur_key1;
extern ION_OFF_STATE cur_key2;
extern SGW_CTW_STATE cur_key3;
extern SWITCH_STATE switch_mode;

/* regs define ---------------------------------------------------------------*/
// input reg
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0x1000, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, 0x1007};
// input reg address start
uint16_t usRegInputStart = REG_INPUT_START;

// hold reg
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x1234, 0x5678, 0x4321, 0x8765, 0x1111, 0x2222, 0x3333, 0x4444};
// hold reg adress start
uint16_t usRegHoldingStart = REG_HOLDING_START;

// coil state
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x00};
// switch state
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x00};

/**
 * @description: Modbus slave input register callback function.
 * @param {UCHAR} *pucRegBuffer-input register buffer
 * @param {USHORT} usAddress-input register address
 * @param {USHORT} usNRegs-input register number
 * @return {*}
 */
eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    OS_ERR err;
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);

    if (((int16_t)usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while (usNRegs > 0)
        {
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);

    return eStatus;
}

/**
 * @description: Modbus slave holding register callback function.
 * @param {UCHAR} *pucRegBuffer: holding register buffer
 * @param {USHORT} usAddress: holding register address
 * @param {USHORT} usNRegs: holding register number
 * @param {eMBRegisterMode} eMode: read or write
 * @return {*}
 */
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    OS_ERR err;
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);

    if (((int16_t)usAddress >= REG_HOLDING_START) && ((usAddress + usNRegs) <= (REG_HOLDING_START + REG_HOLDING_NREGS)))
    {
        iRegIndex = (int)(usAddress - usRegHoldingStart);
        switch (eMode)
        {
        case MB_REG_READ: // read MB_REG_READ = 0
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;
        case MB_REG_WRITE: // write MB_REG_WRITE = 0
            while (usNRegs > 0)
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else // error
    {
        eStatus = MB_ENOREG;
    }

    OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);

    return eStatus;
}

/**
 * @description: Modbus slave coils callback function.
 * @param {UCHAR} *pucRegBuffer-coils buffer
 * @param {USHORT} usAddress-coils address
 * @param {USHORT} usNCoils-coils number
 * @param {eMBRegisterMode} eMode-read or write
 * @return {*}
 */
eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    OS_ERR err;
    // 错误状态
    eMBErrorCode eStatus = MB_ENOERR;
    // 寄存器个数
    int16_t iNCoils = (int16_t)usNCoils;
    // 寄存器偏移量
    int16_t usBitOffset;

    OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);

    // 检查寄存器是否在指定范围内
    if (((int16_t)usAddress >= REG_COILS_START) && (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE))
    {
        // 计算寄存器偏移量
        usBitOffset = (int16_t)(usAddress - REG_COILS_START);
        switch (eMode)
        {
            // 读操作
        case MB_REG_READ:
            while (iNCoils > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf,
                                                 usBitOffset,
                                                 (uint8_t)(iNCoils > 8 ? 8 : iNCoils));
                iNCoils -= 8;
                usBitOffset += 8;
            }
            break;

            // 写操作
        case MB_REG_WRITE:
            while (iNCoils > 0)
            {
                xMBUtilSetBits(ucRegCoilsBuf,
                               usBitOffset,
                               (uint8_t)(iNCoils > 8 ? 8 : iNCoils),
                               *pucRegBuffer++);
                iNCoils -= 8;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);

    return eStatus;
}

/**
 * @description:  Modbus slave discrete callback function.
 * @param {UCHAR} *pucRegBuffer-discrete buffer
 * @param {USHORT} usAddress-discrete address
 * @param {USHORT} usNDiscrete-discrete number
 * @return {*}
 */
eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    OS_ERR err;
    // 错误状态
    eMBErrorCode eStatus = MB_ENOERR;
    // 操作寄存器个数
    int16_t iNDiscrete = (int16_t)usNDiscrete;
    // 偏移量
    uint16_t usBitOffset;

    OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);

    // 判断寄存器时候再制定范围内
    if (((int16_t)usAddress >= REG_DISCRETE_START) &&
        (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE))
    {
        // 获得偏移量
        usBitOffset = (uint16_t)(usAddress - REG_DISCRETE_START);

        while (iNDiscrete > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(ucRegDiscreteBuf, usBitOffset,
                                             (uint8_t)(iNDiscrete > 8 ? 8 : iNDiscrete));
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);

    return eStatus;
}

/**
 * @description: reg sync
 * @return {*}
 */
void Modbus_reg_sync(void)
{
    OS_ERR err;
    eMBEventType eEvent;
    uint8_t reg;
    Component *comp;
    float gain1;
    float gain2;

    static uint8_t discrete_index = 0;
    static uint8_t hold_reg_index = 0;
    static uint8_t input_reg_index = 0;
    static uint8_t coil_index = 0;
    static uint16_t discrete_value = 0x00;

    if (xMBPortEventGet(&eEvent) != TRUE)
    {
        OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);

        /*input reg - read only*/
        if (input_reg_index > INPUT_ADDR_5)
            input_reg_index = INPUT_ADDR_0;

        switch (input_reg_index)
        {
        case INPUT_ADDR_0:
            usRegInputBuf[input_reg_index] = current_Thermocouple->type;
            break;
        case INPUT_ADDR_1:
            usRegInputBuf[input_reg_index] = current_date.Year;
            break;
        case INPUT_ADDR_2:
            usRegInputBuf[input_reg_index] = current_date.Month;
            break;
        case INPUT_ADDR_3:
            usRegInputBuf[input_reg_index] = current_date.Day;
            break;
        case INPUT_ADDR_4:
            usRegInputBuf[input_reg_index] = current_date.Hour;
            break;
        case INPUT_ADDR_5:
            usRegInputBuf[input_reg_index] = current_date.Minute;
            break;

        default:
            break;
        }
        input_reg_index++;
#if 1
        if (hold_reg_index > HOLD_ADDR_17)
        {
            hold_reg_index = HOLD_ADDR_0;
        }

        /*holding reg*/
        switch (hold_reg_index)
        {
            /*six alarm temp(temp_page_list)*/
        case HOLD_ADDR_0:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->alarm_temp[0] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->alarm_temp[0] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_1:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->alarm_temp[1] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->alarm_temp[1] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_2:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->alarm_temp[2] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->alarm_temp[2] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_3:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->alarm_temp[3] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->alarm_temp[3] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_4:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->alarm_temp[4] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->alarm_temp[4] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_5:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->alarm_temp[5] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->alarm_temp[5] = usRegHoldingBuf[hold_reg_index];
            }
            break;
            /*two gain*/
        case HOLD_ADDR_6:
            gain1 = usRegHoldingBuf[hold_reg_index] / 100.0;
            if (gain1 != weld_controller->temp_gain1 && gain1 <= 1)
            {
                weld_controller->temp_gain1 = gain1;
            }
            break;
        case HOLD_ADDR_7:
            gain2 = usRegHoldingBuf[hold_reg_index] / 100.0;
            if (gain2 != weld_controller->temp_gain2 && gain2 <= 1)
            {
                weld_controller->temp_gain1 = gain2;
            }
            break;

            /*three step set(param_page_list)*/
        case HOLD_ADDR_8:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_temp[0] && usRegHoldingBuf[hold_reg_index] <= USER_FIRST_SET_MAX)
            {
                weld_controller->weld_temp[0] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_9:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_temp[1] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->weld_temp[1] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_10:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_temp[2] && usRegHoldingBuf[hold_reg_index] <= USER_SET_MAX_TEMP)
            {
                weld_controller->weld_temp[2] = usRegHoldingBuf[hold_reg_index];
            }
            break;
            /*five time*/
        case HOLD_ADDR_11:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_time[0] && usRegHoldingBuf[hold_reg_index] <= 999)
            {
                weld_controller->weld_time[0] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_12:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_time[1] && usRegHoldingBuf[hold_reg_index] <= 999)
            {
                weld_controller->weld_time[1] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_13:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_time[2] && usRegHoldingBuf[hold_reg_index] <= USER_MAX_WELD_TIME)
            {
                weld_controller->weld_time[2] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_14:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_time[3] && usRegHoldingBuf[hold_reg_index] <= 999)
            {
                weld_controller->weld_time[3] = usRegHoldingBuf[hold_reg_index];
            }
            break;
        case HOLD_ADDR_15:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_time[4] && usRegHoldingBuf[hold_reg_index] <= 999)
            {
                weld_controller->weld_time[4] = usRegHoldingBuf[hold_reg_index];
                // get_comp(param_page_list, page_name_list[hold_reg_index])->val = usRegHoldingBuf[hold_reg_index];
            }
            break;
            /*count(param_page_list)*/
        case HOLD_ADDR_16:
            if (usRegHoldingBuf[hold_reg_index] != weld_controller->weld_count && usRegHoldingBuf[hold_reg_index] <= USER_MAX_COUNT)
            {
                weld_controller->weld_count = usRegHoldingBuf[hold_reg_index];
                // get_comp(param_page_list, page_name_list[hold_reg_index])->val = usRegHoldingBuf[hold_reg_index];
                // get_comp(temp_page_list, page_name_list[hold_reg_index])->val = usRegHoldingBuf[hold_reg_index];
            }
            break;

            /*GP*/
        case HOLD_ADDR_17:

            if (usRegHoldingBuf[hold_reg_index] != comp->val && usRegHoldingBuf[hold_reg_index] <= MAX_GP)
            {
                // get_comp(param_page_list, "GP")->val = usRegHoldingBuf[hold_reg_index];
            }
            hold_reg_index = HOLD_ADDR_0;
            break;

        default:
            break;
        }

        hold_reg_index++;
#endif
        /*coils reg*/
        if (coil_index > COIL_ADDR_7)
        {
            coil_index = COIL_ADDR_0;
        }

        reg = ucRegCoilsBuf[0] & (0x01 << coil_index);
        switch (coil_index)
        {
        case COIL_ADDR_0:
            /*write*/
            if (RLY_AIR0_READ != reg)
                RLY_AIR0 = reg;
            break;
        case COIL_ADDR_1:
            /*write*/
            if (RLY_AIR1_READ != reg)
                RLY_AIR0 = reg;
            break;
        case COIL_ADDR_2:
            /*write*/
            if (RLY_AIR2_READ)
                RLY_AIR0 = reg;

            break;
        case COIL_ADDR_3:
            /*write*/
            if (RLY_OVER_READ != reg)
                RLY_AIR0 = reg;

            break;
        case COIL_ADDR_4:
            /*write*/
            if (RLY_ERR_READ != reg)
                RLY_AIR0 = reg;

            break;
        case COIL_ADDR_5:
            /*write*/
            if (RLY_CNT_READ != reg)
                RLY_AIR0 = reg;

            break;
        case COIL_ADDR_6:
            /*write*/
            if (RLY_CONTACTOR_READ != reg)
                RLY_AIR0 = reg;

            break;
        case COIL_ADDR_7:
            /*write*/
            if (RLY_TRAN_READ != reg)
                RLY_AIR0 = reg;
            break;

        default:
            break;
        }

        coil_index++;

        /*discrete reg - read only*/
        if (discrete_index > DISCRETE_ADDR_11)
        {
            discrete_index = DISCRETE_ADDR_0;
        }

        switch (discrete_index)
        {
        case DISCRETE_ADDR_0:
            if (cur_key1 == RDY)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);

            break;
        case DISCRETE_ADDR_1:
            if (cur_key2 == ION)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_2:
            if (cur_key3 == CTW)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_3:
            if (weld_controller->Count_Dir == DOWN)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_4:
            if (switch_mode == AUTO_MODE)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_5:
            if (err_get_type(err_ctrl, TRANSFORMER_OVER_HEAT)->state == true)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_6:
            if (err_get_type(err_ctrl, RADIATOR)->state == true)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_7:
            if (err_get_type(err_ctrl, SENSOR_ERROR)->state == true)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_8:
            if (RLY_INPUT_SCAN() == RLY_START0_ACTIVE)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_9:
            if (RLY_INPUT_SCAN() == RLY_START1_ACTIVE)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_10:
            if (RLY_INPUT_SCAN() == RLY_START2_ACTIVE)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;
        case DISCRETE_ADDR_11:
            if (RLY_INPUT_SCAN() == RLY_START3_ACTIVE)
                discrete_value |= 0x01 << discrete_index;
            else
                discrete_value &= ~(0x01 << discrete_index);
            break;

        default:
            break;
        }
        discrete_index++;

        ucRegDiscreteBuf[0] = (uint8_t)discrete_value & 0x0f;
        ucRegDiscreteBuf[1] = (uint8_t)discrete_value >> 8;

        OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);
    }
}
