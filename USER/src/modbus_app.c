#include "mb.h"
#include "mbutils.h"
#include "modbus_app.h"
#include "sys.h"
#include "thermocouple.h"
#include "adc.h"
#include "touchscreen.h"

/* Private variables ---------------------------------------------------------*/
extern OS_MUTEX ModBus_Mux;
extern Thermocouple *current_Thermocouple;
extern Date current_date;

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
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x01, 0x02};
// switch state
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x01, 0x02};

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
    if (xMBPortEventGet(&eEvent) != TRUE)
    {
        OSMutexPend(&ModBus_Mux, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);

        /*input reg - read only*/
        usRegInputBuf[0] = current_Thermocouple->type;
        usRegInputBuf[1] = current_date.Year;
        usRegInputBuf[2] = current_date.Month;
        usRegInputBuf[3] = current_date.Day;
        usRegInputBuf[4] = current_date.Hour;
        usRegInputBuf[5] = current_date.Minute;

        /*holding reg*/

        /*coils reg*/

        /*discrete reg - read only*/
        

        OSMutexPost(&ModBus_Mux, OS_OPT_POST_NONE, &err);
    }
}
