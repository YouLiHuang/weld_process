/*
 * @Author: huangyouli.scut@gmail.com
 * @Date: 2025-02-10 20:42:26
 * @LastEditors: YouLiHuang huangyouli.scut@gmail.com
 * @LastEditTime: 2025-02-11 17:20:39
 * @Description:
 *
 * Copyright (c) 2025 by huangyouli, All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------ */
#include "string.h"
#include "dual_func_demo.h"
#include "usbh_core.h"
#include "usbh_msc_usr.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbh_msc_core.h"
#include "usbd_msc_core.h"
#include "usb_conf.h"

/** @addtogroup USBH_USER
 * @{
 */

/** @addtogroup USBH_DUAL_FUNCT_DEMO
 * @{
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO
 * @brief    This file includes the usb host stack user callbacks
 * @{
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Defines
 * @{
 */

#if 0
#define BF_TYPE 0x4D42 /* "MB" */
#define BI_RGB 0       /* No compression - straight BGR data */
#define BI_RLE8 1      /* 8-bit run-length compression */
#define BI_RLE4 2      /* 4-bit run-length compression */
#define BI_BITFIELDS 3 /* RGB bitmap with RGB masks */
#endif

/**
 * @}
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Variables
 * @{
 */

uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;
FATFS fatfs;
FIL file;

uint8_t Enum_Done = 0;
uint8_t writeTextBuff[] = "STM32 Connectivity line Host Demo application using FAT_FS   ";

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
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /* !< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_FS_Core __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__ICCARM__) /* !< IAR Compiler */
#pragma data_alignment = 4
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HOST USB_FS_Host __ALIGN_END;

/**
 * @}
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Constants
 * @{
 */
static void USB_Host_Application(void);

/**
 * @}
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_FunctionPrototypes
 * @{
 */
/**
 * @}
 */

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Functions
 * @{
 */

/**
 * @brief  Demo_Init
 *         Demo initialization
 * @param  None
 * @retval None
 */

void Demo_Init(void)
{

  USBH_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
            USB_OTG_FS_CORE_ID,
#elif defined USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#endif
            &USB_Host,
            &USBH_MSC_cb,
            &USR_USBH_MSC_cb);

  USB_OTG_BSP_mDelay(500);
  DEMO_UNLOCK();
}

/**
 * @brief  USB_Application
 *         Demo background task
 * @param  None
 * @retval None
 */

void Demo_Process(void)
{
  if (HCD_IsDeviceConnected(&USB_OTG_Core))
  {
    USBH_Process(&USB_OTG_Core, &USB_Host);
  }
  //USB_Host_Application();
}

static void USB_Host_Application(void)
{
  uint16_t bytesWritten, bytesToWrite;
  FRESULT res;

  /* Init HS Core : Demo start in host mode */
#ifdef USE_USB_OTG_HS
  printf("> Initializing USB Host High speed...\n");
#else
  printf("> Initializing USB Host Full speed...\n");
#endif
  USBH_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
            USB_OTG_FS_CORE_ID,
#elif defined USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#endif
            &USB_Host,
            &USBH_MSC_cb,
            &USR_USBH_MSC_cb);

  if (!HCD_IsDeviceConnected(&USB_OTG_Core))
  {
    Demo_HandleDisconnect();
    printf("Please, connect a device and try again.\n");
  }
  if (Enum_Done == 0)
  {
    Enum_Done = 1;
  }

  if (Enum_Done == 1)
  {
#ifdef USE_USB_OTG_HS
    printf("> USB Host High speed initialized.\n");
#else
    printf("> USB Host Full speed initialized.\n");
#endif

    /* Initialises the File System */
    if (f_mount(&fatfs, "0:", 1) != FR_OK)
    {
      /* efs initialisation fails */
      printf("> Cannot initialize File System.\n");
    }
    printf("> File System initialized.\n");
    printf("> Disk capacity : %lu Bytes\n", USBH_MSC_Param.MSCapacity * USBH_MSC_Param.MSPageLength);
    Enum_Done = 2;
  }

  if (Enum_Done == 2)
  {
    USB_OTG_DisableGlobalInt(&USB_OTG_Core);
    USB_OTG_EnableGlobalInt(&USB_OTG_Core);

    /*1、磁盘挂载测试*/
    if (f_mount(&fatfs, "0:", 1) != FR_OK)
    {
      /* fat_fs initialisation fails */
      printf("fat_fs initialisation fails");
    }

    /*2、磁盘扫描测试*/
    Explore_Disk("0:", 1);

    /*3、文件读写测试*/
    /* Writes a text file, STM32.TXT in the disk */
    printf("> Writing File to disk flash ...\n");
    if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
    {
      printf("> Disk flash is write protected \n");
      USBH_USR_ApplicationState = USH_USR_FS_DRAW;
    }
    DEMO_LOCK();
    /* Register work area for logical drives */
    f_mount(&fatfs, "0:", 1);
    if (f_open(&file, "0:test.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      /* Write buffer to file */
      bytesToWrite = sizeof(writeTextBuff);
      res = f_write(&file, writeTextBuff, bytesToWrite, (void *)&bytesWritten);

      if ((bytesWritten == 0) || (res != FR_OK)) /* EOF or Error */
        printf("> test.txt CANNOT be writen.\n");
      else
        printf("> 'test.txt' file created\n");
      /* close file and filesystem */
      f_close(&file);
    }

/*4、断开连接*/
#if 0
    Demo_HandleDisconnect();
    f_mount(NULL, "", 0);
    USB_OTG_StopHost(&USB_OTG_Core);

    /* Manage User disconnect operations */
    USB_Host.usr_cb->DeviceDisconnected();

    /* Re-Initilaize Host for new Enumeration */
    USBH_DeInit(&USB_OTG_Core, &USB_Host);
    USB_Host.usr_cb->DeInit();
    USB_Host.class_cb->DeInit(&USB_OTG_Core, &USB_Host.device_prop);
    printf("> > usb host disconnect.\r\n");
#endif
  }
}

#if 0
static void USB_Device_Application(void)
{

  USBD_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
            USB_OTG_FS_CORE_ID,
#elif defined USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#endif
            &USR_desc,
            &USBD_MSC_cb,
            &USR_cb);

  printf("> Device application closed.\n");
  DCD_DevDisconnect(&USB_OTG_Core);
  USB_OTG_StopDevice(&USB_OTG_Core);
}

#endif


/**
 * @brief  Demo_HandleDisconnect
 *         deinit demo and astart again the enumeration
 * @param  None
 * @retval None
 */
void Demo_HandleDisconnect(void)
{
  demo.state = DEMO_IDLE;
  USBH_DeInit(&USB_OTG_Core, &USB_Host);
  Enum_Done = 0;
  DEMO_UNLOCK();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
