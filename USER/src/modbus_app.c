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
extern Page_Param *page_param;
extern weld_ctrl *weld_controller;
extern Component_Queue *temp_page_list;
extern Error_ctrl *err_ctrl;
extern START_TYPE start_type;
extern OS_SEM WELD_START_SEM;
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
void Modbus_reg_sync()
{
    OS_ERR err;
    eMBEventType eEvent;
    uint8_t index = 0;
    uint8_t discrete_value = 0x00;
    if (xMBPortEventGet(&eEvent) != TRUE)
    {
        OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);

        /*input reg - read only*/
        for (index = 0; index < REG_INPUT_NREGS; index++)
        {
            switch (index)
            {
            case INPUT_ADDR_0:
                usRegInputBuf[index] = current_Thermocouple->type;
                break;
            case INPUT_ADDR_1:
                usRegInputBuf[index] = current_date.Year;
                break;
            case INPUT_ADDR_2:
                usRegInputBuf[index] = current_date.Month;
                break;
            case INPUT_ADDR_3:
                usRegInputBuf[index] = current_date.Day;
                break;
            case INPUT_ADDR_4:
                usRegInputBuf[index] = current_date.Hour;
                break;
            case INPUT_ADDR_5:
                usRegInputBuf[index] = current_date.Minute;
                break;
            case INPUT_ADDR_6:
                break;
            case INPUT_ADDR_7:
                break;

            default:
                break;
            }
        }

        /*holding reg*/
        for (index = 0; index < REG_HOLDING_NREGS; index++)
        {
            switch (index)
            {
            case HOLD_ADDR_0:
                break;
            case HOLD_ADDR_1:
                break;
            case HOLD_ADDR_2:
                break;
            case HOLD_ADDR_3:
                break;
            case HOLD_ADDR_4:
                break;
            case HOLD_ADDR_5:
                break;
            case HOLD_ADDR_6:
                break;
            case HOLD_ADDR_7:
                break;
            case HOLD_ADDR_8:
                break;
            case HOLD_ADDR_9:
                break;
            case HOLD_ADDR_10:
                break;
            case HOLD_ADDR_11:
                break;
            case HOLD_ADDR_12:
                break;
            case HOLD_ADDR_13:
                break;
            case HOLD_ADDR_14:
                break;
            case HOLD_ADDR_15:
                break;
            case HOLD_ADDR_16:
                break;
            case HOLD_ADDR_17:
                break;
            }
        }

        /*coils reg*/
        uint8_t reg;
        for (index = 0; index < REG_COILS_SIZE; index++)
        {
            switch (index)
            {
            case COIL_ADDR_0:
                /*write*/
                reg = ucRegCoilsBuf[0] & (0x01 << index);
                if (key_scan() != KEY_PC0_PRES && reg == 0)
                {
                    /*notify main task to start weld*/
                    start_type = KEY0;
                    OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
                }
                /*key sacn result update to mudbus...*/
                break;
            case COIL_ADDR_1:
                /*write*/
                 reg = ucRegCoilsBuf[0] & (0x01 << index);
                if (key_scan() != KEY_PC1_PRES && reg == 0)
                {
                    /*notify main task to start weld*/
                    start_type = KEY0;
                    OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
                }
                /*key sacn result update to mudbus...*/
                break;
            case COIL_ADDR_2:
                /*write*/
                 reg = ucRegCoilsBuf[0] & (0x01 << index);
                if (key_scan() != KEY_PC2_PRES && reg == 0)
                {
                    /*notify main task to start weld*/
                    start_type = KEY0;
                    OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
                }
                /*key sacn result update to mudbus...*/
                break;
            case COIL_ADDR_3:
                /*write*/
                 reg = ucRegCoilsBuf[0] & (0x01 << index);
                if (key_scan() != KEY_PC3_PRES && reg == 0)
                {
                    /*notify main task to start weld*/
                    start_type = KEY0;
                    OSSemPost(&WELD_START_SEM, OS_OPT_POST_ALL, &err);
                }
                /*key sacn result update to mudbus...*/
                break;
            case COIL_ADDR_4:
                 reg = ucRegCoilsBuf[0] & (0x01 << index);
                break;
            case COIL_ADDR_5:
                 reg = ucRegCoilsBuf[0] & (0x01 << index);
                break;
            case COIL_ADDR_6:
                 reg = ucRegCoilsBuf[0] & (0x01 << index);
                break;
            case COIL_ADDR_7:
                 reg = ucRegCoilsBuf[0] & (0x01 << index);
                break;

            default:
                break;
            }
        }

        /*discrete reg - read only*/
        for (index = 0; index < REG_DISCRETE_SIZE; index++)
        {
            switch (index)
            {
            case DISCRETE_ADDR_0:
                if (page_param->key1 == RDY)
                    discrete_value |= 0x01 << index;
                break;
            case DISCRETE_ADDR_1:
                if (page_param->key2 == ION)
                    discrete_value |= 0x01 << index;
                break;
            case DISCRETE_ADDR_2:
                if (page_param->key3 == CTW)
                    discrete_value |= 0x01 << index;
                break;
            case DISCRETE_ADDR_3:
                if (weld_controller->Count_Dir == DOWN)
                    discrete_value |= 0x01 << index;
                break;
            case DISCRETE_ADDR_4:
                if (get_comp(temp_page_list, "switch")->val == 1)
                    discrete_value |= 0x01 << index;
                break;
            case DISCRETE_ADDR_5:
                if (err_get_type(err_ctrl, TRANSFORMER_OVER_HEAT)->state == true)
                    discrete_value |= 0x01 << index;
                break;
            case DISCRETE_ADDR_6:
                if (err_get_type(err_ctrl, RADIATOR)->state == true)
                    discrete_value |= 0x01 << index;
                break;
            case DISCRETE_ADDR_7:
                if (err_get_type(err_ctrl, SENSOR_ERROR)->state == true)
                    discrete_value |= 0x01 << index;
                break;

            default:
                break;
            }
        }
        ucRegDiscreteBuf[0] = discrete_value;

        OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);
    }
}
