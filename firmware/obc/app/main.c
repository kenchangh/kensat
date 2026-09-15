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
#include "csk_hw.h"       /* SFR definitions (TMR1) for the idle counter */
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

/* Debug-console verbosity; see CONSOLE_VERBOSE_DEFAULT in main.h.
   Read by task_beacon (telemetry lines) and task_si4463 (AX.25 hex dump),
   toggled from the console by the 'q' command. */
volatile unsigned char g_console_verbose = CONSOLE_VERBOSE_DEFAULT;

/* Timer1 counts (Fcy/256 = 15625 Hz) accumulated while the core was halted in
   Idle.  Divided by elapsed Salvo ticks this is the CPU idle fraction -- the
   only on-board measurement of what POWER_IDLE actually buys, on a board with
   no current sensing.  Reported and re-zeroed by the 'w' console command. */
volatile unsigned long g_idle_counts = 0;

/* Salvo tick at which the current idle-measurement window opened. */
volatile unsigned long g_idle_mark_ticks = 0;


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

#if POWER_IDLE
    // Halt the core until the next interrupt (PWRSAV #1 = Idle mode).
    //  Timer2 was opened T2_IDLE_CON so the 100 Hz Salvo tick keeps running
    //  and always wakes us within 10 ms; UART RX/TX interrupts wake us
    //  immediately, so console responsiveness is unchanged.
    //
    //  Note this idles after EVERY pass, not only the ones where no task was
    //  eligible, so a dispatch can be followed by up to one tick of sleep
    //  before the next task runs. With six tasks that is ~50 ms of added skew
    //  in the worst case -- immaterial against task periods of 0.5 s and up,
    //  and against the ~9 s of late margin task_wdt has on the TPS3430.
    //  Doing it properly via OSIdlingHook() would mean adding
    //  Salvo/Src/MCC30/salvohook_idle.c to the build, which the MPLAB makefile
    //  generator cannot do headless; this needs no build-system change.
    {
      unsigned int t1_before = TMR1;
      __builtin_pwrsav(1);
      // Unsigned 16-bit subtraction, so a TMR1 wrap during the span is
      //  handled correctly. The span is at most one 10 ms tick (~156 counts)
      //  against a ~4.19 s wrap period, so it can never alias.
      g_idle_counts += (unsigned long)(unsigned int)(TMR1 - t1_before);
    }
#endif
  } /* while */
  
} /* main() */
