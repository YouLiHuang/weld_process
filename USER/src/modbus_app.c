#include "mb.h"
#include "mbutils.h"
#include "modbus_app.h"
#include "sys.h"

/* Private variables ---------------------------------------------------------*/
// 输入寄存器内容 (test)
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0x1000, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, 0x1007};
// 输入寄存器起始地址
uint16_t usRegInputStart = REG_INPUT_START;

// 保持寄存器内容 (test)
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x147b, 0x3f8e, 0x147b, 0x400e, 0x1eb8, 0x4055, 0x147b, 0x408e};
// 保持寄存器起始地址
uint16_t usRegHoldingStart = REG_HOLDING_START;

// 线圈状态 (test)
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x01, 0x02};
// 开关输入状态 (test)
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
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

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
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if (((int16_t)usAddress >= REG_HOLDING_START) && ((usAddress + usNRegs) <= (REG_HOLDING_START + REG_HOLDING_NREGS)))
    {
        iRegIndex = (int)(usAddress - usRegHoldingStart);
        switch (eMode)
        {
        case MB_REG_READ: // 读 MB_REG_READ = 0
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (u8)(usRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;
        case MB_REG_WRITE: // 写 MB_REG_WRITE = 0
            while (usNRegs > 0)
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else // 错误
    {
        eStatus = MB_ENOREG;
    }

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
    // 错误状态
    eMBErrorCode eStatus = MB_ENOERR;
    // 寄存器个数
    int16_t iNCoils = (int16_t)usNCoils;
    // 寄存器偏移量
    int16_t usBitOffset;

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
    // 错误状态
    eMBErrorCode eStatus = MB_ENOERR;
    // 操作寄存器个数
    int16_t iNDiscrete = (int16_t)usNDiscrete;
    // 偏移量
    uint16_t usBitOffset;

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
    return eStatus;
}
