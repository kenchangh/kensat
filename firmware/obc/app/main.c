/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\main.c,v $
$Author: aek $
$Revision: 3.2 $
$Date: 2010-03-28 19:40:15-08 $

******************************************************************************/
#include "events.h"
#include "init.h"
#include "main.h"
#include "task_5sec.h"
#include "task_cmd.h"
#include "task_exercise_io.h"
#include "task_mhx.h"
#include "task_onchip_temp.h"
#include "task_status.h"
#include "task_si4463.h"
#include "task_beacon.h"
#include "task_jetson.h"
#include "task_usb.h"
#include "task_sdlog.h"
#include "task_bmi088.h"
#include "task_wdt.h"
#include "tasks.h"

// Pumpkin Salvo headers
#include "salvo.h"



csk_status_t csk_status;
char strTmp[160];                 // usable to anyone who wants it ...


/******************************************************************************
****                                                                       ****
**                                                                           **
main()

Conventional Salvo main(), with peripheral initialization, Salvo 
initialization, and the Salvo scheduler.

**                                                                           **
****                                                                       ****
******************************************************************************/
int main() {

  // Do target-specific (e.g., clocks, UARTs) and general 
  //  (e.g., CSK IO) initialization.
  init();
                              
  // Initialize Salvo RTOS.
  OSInit();

  // Create tasks.
  OSCreateTask(task_cmd_do,          TASK_CMD_DO_P,            1);
//OSCreateTask(task_status_mon,      TASK_STATUS_MON_P,        3);
//OSCreateTask(task_usb_detect,      TASK_USB_DETECT_P,        8);
//OSCreateTask(task_usb_talk,        TASK_USB_TALK_P,          5); // Disabled: noise on debug console
//OSCreateTask(task_5sec,            TASK_5SEC_P,              15);
//OSCreateTask(task_onchip_temp,     TASK_ONCHIP_TEMP_P,       11);
//OSCreateTask(task_exercise_io,     TASK_EXERCISE_IO_P,      14);
//OSCreateTask(task_mhx_talk,        TASK_MHX_TALK_P,          7);
//OSCreateTask(task_si4463_rx,       TASK_SI4463_RX_P,         9); // Disabled: use 's'/'x' commands
  OSCreateTask(task_jetson,          TASK_JETSON_P,            9);
  OSCreateTask(task_sdlog,           TASK_SDLOG_P,            12);
  OSCreateTask(task_beacon,          TASK_BEACON_P,           10);
  OSCreateTask(task_bmi088,          TASK_BMI088_P,           11);
  OSCreateTask(task_wdt,             TASK_WDT_P,               2); // external TPS3430 watchdog: high priority so its ~54s kick is never starved

  // Create events.
  OSCreateBinSem(RSRC_USB_MHX_IF_P, 1);    // Initially available
  OSCreateSem(SEM_CMD_CHAR_P,       0);    // No chars received yet
  OSCreateSem(SEM_JETSON_RX_P,      0);    // No Jetson chars received yet
  OSCreateBinSem(RSRC_SD_CARD_P,    0);    // SD card: unavailable until task_sdlog inits

  // Enable interrupts (enables UART tx & rx).
  __enable_interrupt();

  // Multitask.
  while (1) {
    OSSched();
  } /* while */
  
} /* main() */
