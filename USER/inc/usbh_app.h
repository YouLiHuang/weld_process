/**
 ******************************************************************************
 * @file    usbh_usr.h
 * @author  MCD Application Team
 * @version V1.2.1
 * @date    17-March-2018
 * @brief   Header file for usbh_usr.c
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      <http://www.st.com/SLA0044>
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USH_USR_H__
#define __USH_USR_H__

/* Includes ------------------------------------------------------------------*/
#include "ff.h"
#include "usbh_core.h"
#include "usb_conf.h"
#include <stdio.h>
#include "stdint.h"
#include "usbh_msc_core.h"

/** @addtogroup USBH_USER
 * @{
 */

/** @addtogroup USBH_MSC_DEMO_USER_CALLBACKS
 * @{
 */

/** @defgroup USBH_USR
 * @brief This file is the Header file for usbh_usr.c
 * @{
 */

/** @defgroup USBH_USR_Exported_Types
 * @{
 */

extern USBH_Usr_cb_TypeDef USR_USBH_MSC_cb;

/**
 * @}
 */

/** @defgroup USBH_USR_Exported_Defines
 * @{
 */
/* State Machine for the USBH_USR_ApplicationState */
#define USH_USR_FS_INIT 0
#define USH_USR_FS_READLIST 1
#define USH_USR_FS_WRITEFILE 2
#define USH_USR_FS_IDEAL 3


typedef enum USB_SAVE_STATE
{
    SAVE_INIT = 0,
    SAVE_NEW_OPEN,
    SAVE_TITLE,
    SAVE_DATA ,

} USB_SAVE_STATE;

/**
 * @}
 */

/** @defgroup USBH_USR_Exported_Macros
 * @{
 */

#define DATA_BYTE_NUM 20
#define FILE_PREFIX "0:WELD"
#define FILE_SUFFIX "CSV"

/**
 * @}
 */

/** @defgroup USBH_USR_Exported_Variables
 * @{
 */
extern uint8_t USBH_USR_ApplicationState;
/**
 * @}
 */

/** @defgroup USBH_USR_Exported_FunctionsPrototype
 * @{
 */
void USBH_USR_ApplicationSelected(void);
void USBH_USR_Init(void);
void USBH_USR_DeInit(void);
void USBH_USR_DeviceAttached(void);
void USBH_USR_ResetDevice(void);
void USBH_USR_DeviceDisconnected(void);
void USBH_USR_OverCurrentDetected(void);
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed);
void USBH_USR_Device_DescAvailable(void *);
void USBH_USR_DeviceAddressAssigned(void);
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef *cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc);
void USBH_USR_Manufacturer_String(void *);
void USBH_USR_Product_String(void *);
void USBH_USR_SerialNum_String(void *);
void USBH_USR_EnumerationDone(void);
USBH_USR_Status USBH_USR_UserInput(void);
void USBH_USR_DeviceNotSupported(void);
void USBH_USR_UnrecoveredError(void);
int USBH_USR_MSC_Application(void);

uint8_t Explore_Disk(char *path, uint8_t recu_level);
void Write_Test(const char *file_name, const char *data);

/**
 * @}
 */

#endif /*__USH_USR_H__*/

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
