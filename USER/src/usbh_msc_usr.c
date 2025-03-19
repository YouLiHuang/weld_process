/**
 ******************************************************************************
 * @file    usbh_msc_usr.c
 * @author  MCD Application Team
 * @version V1.2.1
 * @date    17-March-2018
 * @brief   This file includes the usb host library user callbacks
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

/* Includes ------------------------------------------------------------------ */
#include <string.h>
#include "usbh_msc_usr.h"
#include "usb_bsp.h"
#include "log.h"

/** @addtogroup USBH_USER
 * @{
 */

/** @addtogroup USBH_MSC_DEMO_USER_CALLBACKS
 * @{
 */

/** @defgroup USBH_USR
 * @brief    This file includes the usb host stack user callbacks
 * @{
 */

/** @defgroup USBH_USR_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBH_USR_Private_Defines
 * @{
 */
#define IMAGE_BUFFER_SIZE 512
/**
 * @}
 */

/** @defgroup USBH_USR_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup USBH_USR_Private_Variables
 * @{
 */

uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;
FATFS fatfs;
FIL file;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /* !< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /* !< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;

/* Points to the DEVICE_PROP structure of current device */
/* The purpose of this register is to speed up the execution */

USBH_Usr_cb_TypeDef USR_USBH_MSC_cb = {
    USBH_USR_Init,
    USBH_USR_DeInit,
    USBH_USR_DeviceAttached,
    USBH_USR_ResetDevice,
    USBH_USR_DeviceDisconnected,
    USBH_USR_OverCurrentDetected,
    USBH_USR_DeviceSpeedDetected,
    USBH_USR_Device_DescAvailable,
    USBH_USR_DeviceAddressAssigned,
    USBH_USR_Configuration_DescAvailable,
    USBH_USR_Manufacturer_String,
    USBH_USR_Product_String,
    USBH_USR_SerialNum_String,
    USBH_USR_EnumerationDone,
    USBH_USR_UserInput,
    USBH_USR_MSC_Application,
    USBH_USR_DeviceNotSupported,
    USBH_USR_UnrecoveredError};

/**
 * @}
 */

/** @defgroup USBH_USR_Private_Constants
 * @{
 */
/*--------------- LCD Messages ---------------*/
char MSG_DEV_ATTACHED[] = "> Device Attached \n";
char MSG_DEV_DISCONNECTED[] = "> Device Disconnected\n";
char MSG_DEV_ENUMERATED[] = "> Enumeration completed \n";
char MSG_DEV_HIGHSPEED[] = "> High speed device detected\n";
char MSG_DEV_FULLSPEED[] = "> Full speed device detected\n";
char MSG_DEV_LOWSPEED[] = "> Low speed device detected\n";
char MSG_DEV_ERROR[] = "> Device fault \n";

char MSG_MSC_CLASS[] = "> Mass storage device connected\n";
char MSG_DISK_SIZE[] = "> Size of the disk in MBytes: \n";
char MSG_LUN[] = "> LUN Available in the device:\n";
char MSG_ROOT_CONT[] = "> Exploring disk\n";
char MSG_WR_PROTECT[] = "> The disk is write protected\n";
char MSG_MSC_UNREC_ERROR[] = "> UNRECOVERED ERROR STATE\n";

/**
 * @}
 */

/** @defgroup USBH_USR_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @defgroup USBH_USR_Private_Functions
 * @{
 */

/**
 * @brief  USBH_USR_Init
 *         Displays the message on LCD for host lib initialization
 * @param  None
 * @retval None
 */
void USBH_USR_Init(void)
{
  printf("usb2.0 driver init\n");
}

/**
 * @brief  USBH_USR_DeviceAttached
 *         Displays the message on LCD on device attached
 * @param  None
 * @retval None
 */
void USBH_USR_DeviceAttached(void)
{
  printf(MSG_DEV_ATTACHED);
}

/**
 * @brief  USBH_USR_UnrecoveredError
 * @param  None
 * @retval None
 */
void USBH_USR_UnrecoveredError(void)
{

  /* Set default screen color */
  printf(MSG_MSC_UNREC_ERROR);
}

/**
 * @brief  USBH_DisconnectEvent
 *         Device disconnect event
 * @param  None
 * @retval Status
 */
void USBH_USR_DeviceDisconnected(void)
{
  printf(MSG_DEV_DISCONNECTED);
}

/**
 * @brief  USBH_USR_ResetUSBDevice
 * @param  None
 * @retval None
 */
void USBH_USR_ResetDevice(void)
{
  /* callback for USB-Reset */
}

/**
 * @brief  USBH_USR_DeviceSpeedDetected
 *         Displays the message on LCD for device speed
 * @param  Device speed
 * @retval None
 */
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
  if (DeviceSpeed == HPRT0_PRTSPD_HIGH_SPEED)
  {
    printf(MSG_DEV_HIGHSPEED);
  }
  else if (DeviceSpeed == HPRT0_PRTSPD_FULL_SPEED)
  {
    printf(MSG_DEV_FULLSPEED);
  }
  else if (DeviceSpeed == HPRT0_PRTSPD_LOW_SPEED)
  {
    printf(MSG_DEV_LOWSPEED);
  }
  else
  {
    printf(MSG_DEV_ERROR);
  }
}

/**
 * @brief  USBH_USR_Device_DescAvailable
 *         Displays the message on LCD for device descriptor
 * @param  device descriptor
 * @retval None
 */
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
}

/**
 * @brief  USBH_USR_DeviceAddressAssigned
 *         USB device is successfully assigned the Address
 * @param  None
 * @retval None
 */
void USBH_USR_DeviceAddressAssigned(void)
{
}

/**
 * @brief  USBH_USR_Conf_Desc
 *         Displays the message on LCD for configuration descriptor
 * @param  Configuration descriptor
 * @retval None
 */
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef *cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
  USBH_InterfaceDesc_TypeDef *id;

  id = itfDesc;

  if ((*id).bInterfaceClass == 0x08)
  {
    printf(MSG_MSC_CLASS);
  }
}

/**
 * @brief  USBH_USR_Manufacturer_String
 *         Displays the message on LCD for Manufacturer String
 * @param  Manufacturer String
 * @retval None
 */
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
  printf("> Manufacturer : %s\n", (char *)ManufacturerString);
}

/**
 * @brief  USBH_USR_Product_String
 *         Displays the message on LCD for Product String
 * @param  Product String
 * @retval None
 */
void USBH_USR_Product_String(void *ProductString)
{
  printf("> Product : %s\n", (char *)ProductString);
}

/**
 * @brief  USBH_USR_SerialNum_String
 *         Displays the message on LCD for SerialNum_String
 * @param  SerialNum_String
 * @retval None
 */
void USBH_USR_SerialNum_String(void *SerialNumString)
{
  printf("> Serial Number : %s\n", (char *)SerialNumString);
}

/**
 * @brief  EnumerationDone
 *         User response request is displayed to ask application jump to class
 * @param  None
 * @retval None
 */
void USBH_USR_EnumerationDone(void)
{

  /* Enumeration complete */
  printf((void *)MSG_DEV_ENUMERATED);
}

/**
 * @brief  USBH_USR_DeviceNotSupported
 *         Device is not supported
 * @param  None
 * @retval None
 */
void USBH_USR_DeviceNotSupported(void)
{
  printf("> Device not supported : LUN > 0 ");
}

/**
 * @brief  USBH_USR_UserInput
 *         User Action for application state entry
 * @param  None
 * @retval USBH_USR_Status : User response for key button
 */
USBH_USR_Status USBH_USR_UserInput(void)
{
  return USBH_USR_RESP_OK;
}

/**
 * @brief  USBH_USR_OverCurrentDetected
 *         Over Current Detected on VBUS
 * @param  None
 * @retval Status
 */
void USBH_USR_OverCurrentDetected(void)
{
  printf("> Overcurrent detected.");
}

/**
 * @brief  USBH_USR_MSC_Application
 *         Demo application for mass storage
 * @param  None
 * @retval Status
 */
int USBH_USR_MSC_Application(void)
{

  FRESULT res;
  uint8_t writeTextBuff[] = "STM32 Connectivity line Host Demo application using FAT_FS   ";
  uint16_t bytesWritten, bytesToWrite;

  switch (USBH_USR_ApplicationState)
  {
  case USH_USR_FS_INIT:

    /* Initialises the File System */
    if (f_mount(&fatfs, "0:/", 0) != FR_OK)
    {
      /* efs initialisation fails */
      printf("> Cannot initialize File System.\n");
      return (-1);
    }
    printf("> File System initialized.\n");
    printf("> Disk capacity : %lu Bytes\n", USBH_MSC_Param.MSCapacity * USBH_MSC_Param.MSPageLength);

    if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
    {
      printf(MSG_WR_PROTECT);
    }

    USBH_USR_ApplicationState = USH_USR_FS_READLIST;
    break;

  case USH_USR_FS_READLIST:

    printf(MSG_ROOT_CONT);
    Explore_Disk("0:/", 1);

    USBH_USR_ApplicationState = USH_USR_FS_WRITEFILE;

    break;

  case USH_USR_FS_WRITEFILE:

    printf("> write file test\r\n");
    USB_OTG_BSP_mDelay(100);

    /* Key button in polling */
    while (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
      break;
    }
    /* Writes a text file, test.txt in the disk */
    printf("> Writing File to disk\n");
    if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
    {

      printf("> Disk flash is write protected \n");
      USBH_USR_ApplicationState = USH_USR_FS_DRAW;
      break;
    }

    /* Register work area for logical drives */
    f_mount(&fatfs, "", 0);

    if (f_open(&file, "0:test.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      /* Write buffer to file */
      bytesToWrite = sizeof(writeTextBuff);
      res = f_write(&file, writeTextBuff, bytesToWrite, (void *)&bytesWritten);

      if ((bytesWritten == 0) || (res != FR_OK)) /* EOF or Error */
      {
        printf("> test.txt CANNOT be writen.\n");
      }
      else
      {
        printf("> 'test.txt' file created\n");
      }

      /* close file and filesystem */
      f_close(&file);
      f_mount(NULL, "", 0);
    }

    else
    {
      printf("> test.txt created in the disk\n");
    }
    USB_OTG_BSP_mDelay(100);

    USBH_USR_ApplicationState = USH_USR_FS_DRAW;

    break;

  case USH_USR_FS_DRAW:
    /* Key button in polling */
    while (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
      printf("......\n");
      USB_OTG_BSP_mDelay(500);
    }

    while (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
      if (f_mount(&fatfs, "", 0) != FR_OK)
      {
        /* fat_fs initialisation fails */
        return (-1);
      }
    }
    break;

  default:
    break;
  }
  return (0);
}

/**
 * @brief  USBH_USR_DeInit
 *         Deinit User state and associated variables
 * @param  None
 * @retval None
 */
void USBH_USR_DeInit(void)
{
  USBH_USR_ApplicationState = USH_USR_FS_INIT;
}

/**
 * @brief  Explore_Disk
 *         Displays disk content
 * @param  path: pointer to root path
 * @retval None
 */
uint8_t Explore_Disk(char *path, uint8_t recu_level)
{

  FRESULT res;
  FILINFO fno;
  DIR dir;
  char *fn;
  char tmp[14];

  res = f_opendir(&dir, path);
  if (res == FR_OK)
  {
    while (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
#if 0
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      if (fno.fname[0] == '.')
      {
        continue;
      }

      fn = fno.fname;
      strcpy(tmp, fn);

      if (recu_level == 1)
      {
        printf("   |__");
      }
      else if (recu_level == 2)
      {
        printf("   |   |__");
      }
      if ((fno.fattrib & AM_MASK) == AM_DIR)
      {
        strcat(tmp, "\n");
        printf(tmp);
      }
      else
      {
        strcat(tmp, "\n");
        printf(tmp);
      }

      if (((fno.fattrib & AM_MASK) == AM_DIR) && (recu_level == 1))
      {
        Explore_Disk(fn, 2);
      }
#endif

      res = f_readdir(&dir, &fno); // 读取目录项

      if (res != FR_OK || fno.fname[0] == 0)
      {
        break; // 遍历结束
      }

      if (fno.fname[0] == '.')
      {
        continue; // 跳过当前目录和父目录
      }

      // 判断是文件还是目录
      if (fno.fattrib & AM_DIR)
        printf("Directory:  %s\n", fno.fname);
      else
        printf("file:  %s\n", fno.fname);
    }
  }
  else
    printf("> open dir fail\n");

  f_closedir(&dir);
  f_mount(NULL, "", 0);
  printf("> explore finished\n");

  return res;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
