/**
 ******************************************************************************
 * @file    usb_conf.h
 * @author  MCD Application Team
 * @version V1.2.1
 * @date    17-March-2018
 * @brief   General low level driver configuration
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
#ifndef __USB_CONF__H__
#define __USB_CONF__H__

/* Includes ------------------------------------------------------------------*/
#if defined(USE_STM322xG_EVAL)
#include "stm322xg_eval.h"
#include "stm322xg_eval_lcd.h"
#include "stm322xg_eval_ioe.h"
#include "stm322xg_eval_sdio_sd.h"

#elif defined(USE_STM324xG_EVAL)
#include "stm32f4xx.h"
#include "stm324xg_eval.h"
#include "stm324xg_eval_lcd.h"
#include "stm324xg_eval_ioe.h"
#include "stm324xg_eval_sdio_sd.h"

#elif defined(USE_STM324x9I_EVAL)
#include "stm32f4xx.h"
#include "stm324x9i_eval.h"
#include "stm324x9i_eval_lcd.h"
#include "stm324x9i_eval_ioe8.h"
#include "stm324x9i_eval_ioe16.h"
#include "stm324x9i_eval_sdio_sd.h"

#elif defined(USE_STM3210C_EVAL)
#include "stm32f10x.h"
#include "stm3210c_eval.h"
#include "stm3210c_eval_lcd.h"
#include "stm3210c_eval_ioe.h"
#include "stm3210c_eval_spi_sd.h"
#else
#include "stm32f4xx.h"
#include "stm324xg_eval_sdio_sd.h"
// #error "Missing define: Evaluation board (ie. USE_STM322xG_EVAL)"
#endif

/** @addtogroup USB_OTG_DRIVER
 * @{
 */

/** @defgroup USB_CONF
 * @brief USB low level driver configuration file
 * @{
 */

/** @defgroup USB_CONF_Exported_Defines
 * @{
 */

/* USB Core and PHY interface configuration.
   Tip: To avoid modifying these defines each time you need to change the USB
        configuration, you can declare the needed define in your toolchain
        compiler preprocessor.
   */
/****************** USB OTG FS PHY CONFIGURATION *******************************
 *  The USB OTG FS Core supports one on-chip Full Speed PHY.
 *
 *  The USE_EMBEDDED_PHY symbol is defined in the project compiler preprocessor
 *  when FS core is used.
 *******************************************************************************/
#ifndef USE_USB_OTG_FS
#define USE_USB_OTG_FS
#endif /* USE_USB_OTG_FS */

#ifdef USE_USB_OTG_FS
#define USB_OTG_FS_CORE
#endif

/****************** USB OTG HS PHY CONFIGURATION *******************************
 *  The USB OTG HS Core supports two PHY interfaces:
 *   (i)  An ULPI interface for the external High Speed PHY: the USB HS Core will
 *        operate in High speed mode
 *   (ii) An on-chip Full Speed PHY: the USB HS Core will operate in Full speed mode
 *
 *  You can select the PHY to be used using one of these two defines:
 *   (i)  USE_ULPI_PHY: if the USB OTG HS Core is to be used in High speed mode
 *   (ii) USE_EMBEDDED_PHY: if the USB OTG HS Core is to be used in Full speed mode
 *
 *  Notes:
 *   - The USE_ULPI_PHY symbol is defined in the project compiler preprocessor as
 *     default PHY when HS core is used.
 *   - On STM322xG-EVAL and STM324xG-EVAL boards, only configuration(i) is available.
 *     Configuration (ii) need a different hardware, for more details refer to your
 *     STM32 device datasheet.
 *******************************************************************************/
#ifndef USE_USB_OTG_HS
/* #define USE_USB_OTG_HS */
#endif /* USE_USB_OTG_HS */

#ifndef USE_ULPI_PHY
/* #define USE_ULPI_PHY */
#endif /* USE_ULPI_PHY */

#ifndef USE_EMBEDDED_PHY
/* #define USE_EMBEDDED_PHY */
#endif /* USE_EMBEDDED_PHY */

#ifdef USE_USB_OTG_HS
#define USB_OTG_HS_CORE
#endif

/*******************************************************************************
 *                      FIFO Size Configuration in Device mode
 *
 *  (i) Receive data FIFO size = RAM for setup packets +
 *                   OUT endpoint control information +
 *                   data OUT packets + miscellaneous
 *      Space = ONE 32-bits words
 *     --> RAM for setup packets = 10 spaces
 *        (n is the nbr of CTRL EPs the device core supports)
 *     --> OUT EP CTRL info      = 1 space
 *        (one space for status information written to the FIFO along with each
 *        received packet)
 *     --> data OUT packets      = (Largest Packet Size / 4) + 1 spaces
 *        (MINIMUM to receive packets)
 *     --> OR data OUT packets  = at least 2*(Largest Packet Size / 4) + 1 spaces
 *        (if high-bandwidth EP is enabled or multiple isochronous EPs)
 *     --> miscellaneous = 1 space per OUT EP
 *        (one space for transfer complete status information also pushed to the
 *        FIFO with each endpoint's last packet)
 *
 *  (ii)MINIMUM RAM space required for each IN EP Tx FIFO = MAX packet size for
 *       that particular IN EP. More space allocated in the IN EP Tx FIFO results
 *       in a better performance on the USB and can hide latencies on the AHB.
 *
 *  (iii) TXn min size = 16 words. (n  : Transmit FIFO index)
 *   (iv) When a TxFIFO is not used, the Configuration should be as follows:
 *       case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
 *       --> Txm can use the space allocated for Txn.
 *       case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
 *       --> Txn should be configured with the minimum space of 16 words
 *  (v) The FIFO is used optimally when used TxFIFOs are allocated in the top
 *       of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
 *   (vi) In HS case12 FIFO locations should be reserved for internal DMA registers
 *        so total FIFO size should be 1012 Only instead of 1024
 *******************************************************************************/

/*******************************************************************************
 *                     FIFO Size Configuration in Host mode
 *
 *  (i) Receive data FIFO size = (Largest Packet Size / 4) + 1 or
 *                             2x (Largest Packet Size / 4) + 1,  If a
 *                             high-bandwidth channel or multiple isochronous
 *                             channels are enabled
 *
 *  (ii) For the host nonperiodic Transmit FIFO is the largest maximum packet size
 *      for all supported nonperiodic OUT channels. Typically, a space
 *      corresponding to two Largest Packet Size is recommended.
 *
 *  (iii) The minimum amount of RAM required for Host periodic Transmit FIFO is
 *        the largest maximum packet size for all supported periodic OUT channels.
 *        If there is at least one High Bandwidth Isochronous OUT endpoint,
 *        then the space must be at least two times the maximum packet size for
 *        that channel.
 *******************************************************************************/

/****************** USB OTG HS CONFIGURATION **********************************/
#ifdef USB_OTG_HS_CORE
#define RX_FIFO_HS_SIZE 512
#define TX0_FIFO_HS_SIZE 128
#define TX1_FIFO_HS_SIZE 372
#define TX2_FIFO_HS_SIZE 0
#define TX3_FIFO_HS_SIZE 0
#define TX4_FIFO_HS_SIZE 0
#define TX5_FIFO_HS_SIZE 0
#define TXH_NP_HS_FIFOSIZ 256
#define TXH_P_HS_FIFOSIZ 256

/* #define USB_OTG_HS_SOF_OUTPUT_ENABLED */
#define USB_OTG_EXTERNAL_VBUS_ENABLED
/* #define USB_OTG_INTERNAL_VBUS_ENABLED */

#ifdef USE_ULPI_PHY
#define USB_OTG_ULPI_PHY_ENABLED
#endif
#ifdef USE_EMBEDDED_PHY
#define USB_OTG_EMBEDDED_PHY_ENABLED
/* wakeup is working only when HS core is configured in FS mode */
/* #define USB_OTG_HS_LOW_PWR_MGMT_SUPPORT */
#endif
#define USB_OTG_HS_DEDICATED_EP1_ENABLED
#endif

/****************** USB OTG FS CONFIGURATION **********************************/
#ifdef USB_OTG_FS_CORE

#define RX_FIFO_FS_SIZE 128
#define TX0_FIFO_FS_SIZE 64
#define TX1_FIFO_FS_SIZE 128
#define TX2_FIFO_FS_SIZE 0
#define TX3_FIFO_FS_SIZE 0

#define TXH_NP_FS_FIFOSIZ 96
#define TXH_P_FS_FIFOSIZ 96

/* #define USB_OTG_FS_LOW_PWR_MGMT_SUPPORT */
/* #define USB_OTG_FS_SOF_OUTPUT_ENABLED */
#endif

/****************** USB OTG MISC CONFIGURATION ********************************/
#define VBUS_SENSING_ENABLED

/****************** USB OTG MODE CONFIGURATION ********************************/
#define USE_HOST_MODE
#define USE_DEVICE_MODE
/* #define USE_OTG_MODE */

#ifndef USB_OTG_FS_CORE
#ifndef USB_OTG_HS_CORE
#error "USB_OTG_HS_CORE or USB_OTG_FS_CORE should be defined"
#endif
#endif

#ifndef USE_DEVICE_MODE
#ifndef USE_HOST_MODE
#error "USE_DEVICE_MODE or USE_HOST_MODE should be defined"
#endif
#endif

#ifndef USE_USB_OTG_HS
#ifndef USE_USB_OTG_FS
#error "USE_USB_OTG_HS or USE_USB_OTG_FS should be defined"
#endif
#else /* USE_USB_OTG_HS */
#ifndef USE_ULPI_PHY
#ifndef USE_EMBEDDED_PHY
#error "USE_ULPI_PHY or USE_EMBEDDED_PHY should be defined"
#endif
#endif
#endif

/****************** C Compilers dependant keywords ****************************/
/* In HS mode and when the DMA is used, all variables and data structures dealing
   with the DMA during the transaction process should be 4-bytes aligned */
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined(__GNUC__) /* GNU Compiler */
#define __ALIGN_END __attribute__((aligned(4)))
#define __ALIGN_BEGIN
#else
#define __ALIGN_END
#if defined(__CC_ARM) /* ARM Compiler */
#define __ALIGN_BEGIN __align(4)
#elif defined(__ICCARM__) /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __GNUC__ */
#else
#define __ALIGN_BEGIN
#define __ALIGN_END
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined(__CC_ARM) /* ARM Compiler */
#define __packed __packed
#elif defined(__ICCARM__) /* IAR Compiler */
#define __packed __packed
#elif defined(__GNUC__) /* GNU Compiler */
#define __packed __attribute__((__packed__))
#endif /* __CC_ARM */

/**
 * @}
 */

/** @defgroup USB_CONF_Exported_Types
 * @{
 */
/**
 * @}
 */

/** @defgroup USB_CONF_Exported_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USB_CONF_Exported_Variables
 * @{
 */
/**
 * @}
 */

/** @defgroup USB_CONF_Exported_FunctionsPrototype
 * @{
 */
/**
 * @}
 */

#endif /* __USB_CONF__H__ */

/**
 * @}
 */

/**
 * @}
 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
