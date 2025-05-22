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

#include "user_config.h"
#include "includes.h"
#include "timer.h"
#include "adc.h"
#include "welding_process.h"
#include "log.h"

/** @addtogroup USBH_USER
 * @{
 */

void Data_Save_Callback(void);

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
uint8_t USB_SAVE_STATE = USB_SAVE_NEW_OPEN;
FATFS fatfs;
static FIL file;
FIL test_file;

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

extern OS_SEM DATA_SAVE_SEM;
extern weld_ctrl *weld_controller;
extern Temp_draw_ctrl *temp_draw_ctrl;
extern Thermocouple *current_Thermocouple;

char *file_name = NULL;
static uint16_t save_times = 0;
static uint8_t file_count = 0;
const char PARAM_NAME_LIST[] = "COUNT,SENSOR_TYPE,temp1,temp2,temp3,time1,time2,time3,time4,time5,TH1,TL1,TH2,TL2,TH3,TL3,gain1,gain2,step1,step2,step3\n";

/** @defgroup USBH_USR_Private_Constants
 * @{
 */
/*--------------- printf Messages ---------------*/
char MSG_DEV_ATTACHED[] = "> Device Attached \n";
char MSG_DEV_DISCONNECTED[] = "> Device Disconnected\n";
char MSG_DEV_ENUMERATED[] = "> Enumeration completed \n";
char MSG_DEV_HIGHSPEED[] = "> High speed device detected\n";
char MSG_DEV_FULLSPEED[] = "> Full speed device detected\n";
char MSG_DEV_LOWSPEED[] = "> Low speed device detected\n";
char MSG_DEV_ERROR[] = "> Device fault \n";

char MSG_MSC_CLASS[] = "> Mass storage device connected\n";
char MSG_HID_CLASS[] = "> HID device connected\n";
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
 *         Displays the message on uart for host lib initialization
 * @param  None
 * @retval None
 */
void USBH_USR_Init(void)
{

#ifdef USE_USB_OTG_HS
#ifdef USE_EMBEDDED_PHY
  printf((uint8_t *)" USB OTG HS_IN_FS MSC Host");
#else
  printf((uint8_t *)" USB OTG HS MSC Host");
#endif
#else
  printf(" USB OTG FS MSC Host");
#endif
  printf("> USB Host library started.\n");
  printf("     USB Host Library v2.2.1");
}

/**
 * @brief  USBH_USR_DeviceAttached
 *         Displays the message on uart on device attached
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
 *         Displays the message on uart for device speed
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
 *         Displays the message on uart for device descriptor
 * @param  device descriptor
 * @retval None
 */
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
  USBH_DevDesc_TypeDef *hs;
  hs = DeviceDesc;

  printf("VID : %04luh\n", (uint32_t)(*hs).idVendor);
  printf("PID : %04luh\n", (uint32_t)(*hs).idProduct);
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
 *         Displays the message on uart for configuration descriptor
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
    printf((char *)MSG_MSC_CLASS);
  }
  else if ((*id).bInterfaceClass == 0x03)
  {
    printf((char *)MSG_HID_CLASS);
  }
}

/**
 * @brief  USBH_USR_Manufacturer_String
 *         Displays the message on uart for Manufacturer String
 * @param  Manufacturer String
 * @retval None
 */
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
  printf("> Manufacturer : %s\n", (char *)ManufacturerString);
}

/**
 * @brief  USBH_USR_Product_String
 *         Displays the message on uart for Product String
 * @param  Product String
 * @retval None
 */
void USBH_USR_Product_String(void *ProductString)
{
  printf("> Product : %s\n", (char *)ProductString);
}

/**
 * @brief  USBH_USR_SerialNum_String
 *         Displays the message on uart for SerialNum_String
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
 *         Applications for user disk manipulation
 * @param  None
 * @retval Status
 */
int USBH_USR_MSC_Application(void)
{

  OS_ERR err;

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
    else
    {
      printf("> File System initialized.\n");
      printf("> Disk capacity : %lu Bytes\n", USBH_MSC_Param.MSCapacity * USBH_MSC_Param.MSPageLength);
    }

    if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
    {
      printf(MSG_WR_PROTECT);
    }

    USBH_USR_ApplicationState = USH_USR_FS_READLIST;
    break;

  case USH_USR_FS_READLIST:

#if EXPLORE_DISK_ENABLE
    printf(MSG_ROOT_CONT);
    Explore_Disk("0:/", 1);
#endif

    USBH_USR_ApplicationState = USH_USR_FS_WRITEFILE;
    break;

  case USH_USR_FS_WRITEFILE:

    /* Writes a text file, test.txt in the disk */
    if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
    {
      printf("> Disk flash is write protected \n");
      break;
    }

#if WRITE_CSV_ENABLE
    Write_Test("0:Test.csv", "temp1,temp2,temp3\n250,400,150\n");
#endif

    USBH_USR_ApplicationState = USH_USR_FS_IDEAL;
    break;

  case USH_USR_FS_IDEAL:
    if (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
      OSSemPend(&DATA_SAVE_SEM, 0, OS_OPT_PEND_NON_BLOCKING, NULL, &err);
      if (err == OS_ERR_NONE)
      {
#if !WRITE_CSV_ENABLE
        Data_Save_Callback();
#endif
      }
    }

    break;
  }

  return 0;
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

  printf("-----------------------Disk File Directory-----------------------\n");

  res = f_opendir(&dir, path);
  if (res == FR_OK)
  {
    while (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
#if USE_STM32_DEMO

      char *fn;
      char tmp[14];

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
      // 读取目录项
      res = f_readdir(&dir, &fno);
      // 遍历结束
      if (res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      // 跳过当前目录和父目录
      if (fno.fname[0] == '.')
      {
        continue;
      }

      // 判断是文件还是目录
      if (fno.fattrib & AM_DIR)
        printf("Directory:  %s\n", fno.fname);
      else
        printf("File:  %s\n", fno.fname);
    }
  }
  else
    printf("> Open dir fail\n");

  f_closedir(&dir);

  printf("-----------------------------------------------------------------\n");

  return res;
}

/**
 * @description: write file test function
 * @param {char} * file name
 * @param {char} * data write to file
 * @return {*}
 */
void Write_Test(const char *file_name, const char *data)
{

  FRESULT res;
  uint16_t bytesWritten;

  if (strlen(file_name) > 8)
  {
    printf("> name too long !");
    return;
  }

  f_mount(&fatfs, "", 0);
  /*csv write test*/

  if (f_open(&file, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
  {

    res = f_write(&file, data, sizeof(data) / sizeof(uint8_t), (void *)&bytesWritten);

    if (res != FR_OK) /* EOF or Error */
      printf("> FILE Cannot be writen.\n");

    /* close file*/
    f_close(&file);

    printf("> File written successfully.\n");
  }

  else
    printf("> WRITE TEST FAIL !\n");
}

/**
 * @description: save weld data to disk
 * @param {char} file name that will be saved to
 * @return {*}
 */
static FRESULT Save2Disk(char *file_name)
{
  FRESULT res;
  uint16_t bytesWritten;

  char data_buf[DATA_BYTE_NUM] = "";

  // weld count
  memset(data_buf, 0, sizeof(data_buf) / sizeof(char));
  sprintf(data_buf, "%d,", weld_controller->weld_count);
  res = f_write(&file, data_buf, strlen(data_buf), (void *)&bytesWritten);
  if (res != FR_OK) /* EOF or Error */
  {
    printf("> %s CANNOT be writen.\n", file_name);
  }

  // sensor type
  switch (current_Thermocouple->type)
  {
  case E_TYPE:
    res = f_write(&file, "E_TYPE,", strlen("E_TYPE,"), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s CANNOT be writen.\n", file_name);
    }
    break;

  case J_TYPE:
    res = f_write(&file, "J_TYPE,", strlen("J_TYPE,"), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s CANNOT be writen.\n", file_name);
    }
    break;

  case K_TYPE:
    res = f_write(&file, "K_TYPE,", strlen("K_TYPE,"), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s CANNOT be writen.\n", file_name);
    }
    break;
  }

  // temp1-temp3
  for (uint16_t i = 0; i < sizeof(weld_controller->weld_temp) / sizeof(uint16_t); i++)
  {
    memset(data_buf, 0, sizeof(data_buf) / sizeof(data_buf[0]));
    sprintf(data_buf, "%d,", weld_controller->weld_temp[i]);
    res = f_write(&file, data_buf, strlen(data_buf), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s CANNOT be writen.\n", file_name);
    }
  }

  // time1 -time5
  for (uint16_t i = 0; i < sizeof(weld_controller->weld_time) / sizeof(uint16_t); i++)
  {
    memset(data_buf, 0, sizeof(data_buf) / sizeof(data_buf[0]));
    sprintf(data_buf, "%d,", weld_controller->weld_time[i]);
    res = f_write(&file, data_buf, strlen(data_buf), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s CANNOT be writen.\n", file_name);
    }
  }

  // alarm1-alarm6
  for (uint16_t i = 0; i < sizeof(weld_controller->alarm_temp) / sizeof(uint16_t); i++)
  {
    memset(data_buf, 0, sizeof(data_buf) / sizeof(data_buf[0]));
    sprintf(data_buf, "%d,", weld_controller->alarm_temp[i]);
    res = f_write(&file, data_buf, strlen(data_buf), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s CANNOT be writen.\n", file_name);
    }
  }

  // gain1 gain2
  memset(data_buf, 0, sizeof(data_buf) / sizeof(data_buf[0]));
  sprintf(data_buf, "%f,", (float)weld_controller->temp_gain1);
  res = f_write(&file, data_buf, strlen(data_buf), (void *)&bytesWritten);
  if (res != FR_OK) /* EOF or Error */
  {
    printf("> %s CANNOT be writen.\n", file_name);
  }

  memset(data_buf, 0, sizeof(data_buf) / sizeof(data_buf[0]));
  sprintf(data_buf, "%f,", (float)weld_controller->temp_gain2);
  res = f_write(&file, data_buf, strlen(data_buf), (void *)&bytesWritten);
  if (res != FR_OK) /* EOF or Error */
  {
    printf("> %s CANNOT be writen.\n", file_name);
  }

  /*step1 - step3*/
  for (uint16_t i = 0; i < sizeof(temp_draw_ctrl->display_temp) / sizeof(temp_draw_ctrl->display_temp[0]); i++)
  {
    memset(data_buf, 0, sizeof(data_buf) / sizeof(char));
    sprintf(data_buf, "%d,", temp_draw_ctrl->display_temp[i]);
    res = f_write(&file, data_buf, strlen(data_buf), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s CANNOT be writen.\n", file_name);
    }
  }

  // next line
  f_write(&file, "\n", 1, (void *)&bytesWritten);

  return res;
}

/**
 * @description: It will be called every time welding is completed, trying to store data to disk
 * @return {*}
 */
void Data_Save_Callback(void)
{
  FRESULT res;
  uint16_t bytesWritten;
  uint8_t name_len;
  static uint16_t last_count;
  char temp_name[20] = "";

  /*new file name*/
  if ((file_count == 0 && save_times == 0) || last_count != file_count) // create the first file or a new file
  {
    sprintf(temp_name, "%s%d.%s", FILE_PREFIX, file_count, FILE_SUFFIX);
    name_len = strlen(temp_name);

    if (file_name == NULL)
    {
      file_name = malloc(sizeof(char) * (name_len + 1));
      if (file_name == NULL)
      {
        printf("> Malloc fail!\n");
        return;
      }
    }

    else if (strlen(file_name) != name_len)
    {
      file_name = realloc(file_name, sizeof(char) * (name_len + 1));
      if (file_name == NULL)
      {
        printf("> Realloc fail!\n");
        return;
      }
    }

    strcpy(file_name, temp_name);
  }

  /*State Machine——save data as csv file*/
  switch (USB_SAVE_STATE)
  {
  case USB_SAVE_NEW_OPEN:

    /*mount to root path*/
    f_mount(&fatfs, "", 0);
    /*open new file*/
    res = f_open(&file, file_name, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
      printf("> %s OPNE FAIL , RETRY...\n", file_name);
      break;
    }
    else
    {
      printf("> OPEN NEW FILE  \"%s\" \n", file_name);
      USB_SAVE_STATE = USB_SAVE_TITLE;
    }

  case USB_SAVE_TITLE:
    /*write title*/
    res = f_write(&file, PARAM_NAME_LIST, strlen(PARAM_NAME_LIST), (void *)&bytesWritten);
    if (res != FR_OK) /* EOF or Error */
    {
      printf("> %s TITLE WRITE FAIL, RETRY...\n", file_name);
      break;
    }
    else
    {
      printf("> FIRST LINE HAVE BEEN WRITTEN !\n");
      USB_SAVE_STATE = USB_SAVE_DATA;
    }

  case USB_SAVE_DATA:
    res = Save2Disk(file_name);
    if (res == FR_OK)
    {
      printf("> DATA HAVE BEEN SAVED to \"%s\" !\n", file_name);
      save_times++;

      /*save to next file*/
      if (save_times % MAX_SAVE_TIMES == 0)
      {
        /*close last file*/
        file_count++;
        f_close(&file);

        USB_SAVE_STATE = USB_SAVE_NEW_OPEN;
      }
    }
    else
      printf("> DATA SAVE FAIL , RETRY...\n");

    break;
  }

  last_count = file_count;
}
