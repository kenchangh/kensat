/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_cmd.c,v $
$Author: aek $
$Revision: 3.4 $
$Date: 2010-02-13 13:17:57-08 $

******************************************************************************/
#include "main.h"
#include "cmd.h"
#include "events.h"
#include "task_cmd.h"
#include "task_onchip_temp.h"
#include "task_si4463.h"
#include "bootcount.h"
#include "deploy.h"
#include "tasks.h"

// C headers
#include <ctype.h>

// Pumpkin CubeSat Kit headers
#include "csk_hw.h"
#include "csk_io.h"
#include "csk_power.h"
#include "csk_uart.h"
#include "csk_wdt.h"

// Pumpkin Salvo headers
#include "salvo.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
cmd_explain()
task_cmd_do()

Act on single-character commands:

  h|?: help
  i:   toggle Exercise I/O task on/off
  m:   cycle between various MCLK output settings
  r:   restart application
  t:   display on-chip temperature
  v:   display version number
  z:   go to sleep
 
**                                                                           **
****                                                                       ****
******************************************************************************/
static
void cmd_explain(void) {

  user_debug_msg(STR_CMD_EXPLAIN "h|?: Commands: {h|?, b, c, d, g, i, j, k, m, r, s, t, v, x, y, z}");

} /* cmd_explain() */


void task_cmd_do(void) {

  unsigned char cmd;

  
  user_debug_msg(STR_TASK_CMD_DO "Starting.");
  OS_Delay(50);
  
  csk_status.MCLKOutEnabled = 0;
  
  // Intro messages. Buffer is pretty small, so wait a bit to flush it.
  user_debug_msg(STR_TASK_CMD_DO "Control via RS-232 I/F at " STR_BAUD_RATE ",N,8,1.");
  OS_Delay(50);

  user_debug_msg(STR_TASK_CMD_DO "Microhard MHX modem at " STR_BAUD_RATE ",N,8,1.");
  OS_Delay(50);

  cmd_explain();

  while (1) {

    OS_WaitSem(SEM_CMD_CHAR_P, OSNO_TIMEOUT);

    if ((cmd=csk_uart1_getchar())) {

      switch (tolower(cmd)) {
      
        // Help.
        case 'h':
        case '?':
          cmd_explain();
          break;


        // Show persistent boot count (stored in program flash).
        case 'b':
          sprintf(strTmp, STR_TASK_CMD_DO "b: Boot count = %u.", bootcount_get());
          user_debug_msg(strTmp);
          break;


        // Clear (zero) the persistent boot count -- run before vibration test.
        case 'k':
          bootcount_reset();
          user_debug_msg(STR_TASK_CMD_DO "k: Boot count cleared to 0.");
          break;


        // 'd': normally shows persistent antenna deployment state (program
        // flash).  With DEPLOY_TEST=1 (main.h) it instead FIRES the burn-wire
        // deployment sequence immediately -- bench testing only.
        case 'd':
#if DEPLOY_TEST
          if (deploy_get()) {
            user_debug_msg(STR_TASK_CMD_DO "d: already deployed -- press 'y' to clear first.");
          }
          else if (deploy_attempts_get() >= DEPLOY_MAX_ATTEMPTS) {
            user_debug_msg(STR_TASK_CMD_DO "d: attempt limit reached -- press 'y' to clear.");
          }
          else {
            unsigned int n = deploy_attempt_inc();   /* persist before burn */
            sprintf(strTmp,
                    STR_TASK_CMD_DO "d: deploy attempt %u/%u: nichrome IO.24+IO.25 HIGH.",
                    n, DEPLOY_MAX_ATTEMPTS);
            user_debug_msg(strTmp);

            /* Drive burn wires HIGH on IO.24 (RF1) and IO.25 (RF0). */
            _TRISF1 = 0;
            _TRISF0 = 0;
            csk_io24_high();
            csk_io25_high();

            /* Burn duration: 10 s = 5 x OS_Delay(200), at the task top level. */
            OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);

            csk_io24_low();
            csk_io25_low();

            deploy_set();   /* burn completed -- record DEPLOYED */
            user_debug_msg(STR_TASK_CMD_DO "d: nichrome OFF, antenna deployed (flag set).");
          }
#else
          sprintf(strTmp,
                  STR_TASK_CMD_DO "d: deployed = %u, attempts = %u, rf_unlocked = %u.",
                  deploy_get(), deploy_attempts_get(), deploy_rf_unlocked_get());
          user_debug_msg(strTmp);
#endif
          break;


        // Clear the persistent deployment flag -- bench testing only, lets the
        // burn-wire sequence run again on the next flight-mode boot.
        case 'y':
          deploy_reset();
          user_debug_msg(STR_TASK_CMD_DO "y: Antenna deployed flag cleared.");
          break;


        // Si4463 + PIC24 clock diagnostic dump.
        case 'g':
          user_debug_msg(STR_TASK_CMD_DO "g: Si4463 / clock diagnostic...");
          si4463_cmd_get_props();
          break;


        // 'i' (exercise_io) disabled — task slot reused by task_beacon.


        // Trigger Jetson inference cycle (boot -> infer -> shutdown).
        // If already running, force a hard power cut instead.
        case 'j':
          if (csk_status.jetson_power_on == FALSE) {
            csk_status.jetson_run_requested = TRUE;
            user_debug_msg(STR_TASK_CMD_DO "j: Jetson inference requested.");
          }
          else {
            csk_io17_low();   /* drop POWER_EN first */
            csk_io16_low();   /* then JETSON_5V */
            csk_status.jetson_power_on = FALSE;
            user_debug_msg(STR_TASK_CMD_DO "j: Jetson hard power cut.");
          }
          break;


        // Toggle MCLCK output for frequency & duty-cycle check.
        //  This is a highly target-dependent function.
        case 'm':
          cmd_mclk();
          break;
        

        // Reset.
        case 'r':
          user_debug_msg(STR_TASK_CMD_DO "r: Reset in 1 s.");
          OS_Delay(100);
          csk_wdt_force();
          break;


        // Si4463 radio: init and set RX mode.
        case 's':
          user_debug_msg(STR_TASK_CMD_DO "s: Si4463 init + RX mode...");
          si4463_cmd_rx();
          break;


        // Read on-chip temp sensor.
        case 't':
          sprintf(strTmp, STR_TASK_CMD_DO "t: Uncalibrated on-chip temperature is %dC.", onchip_temp_deg_c());
          user_debug_msg(strTmp);
          break;


        // Tell version.
        case 'v':
          user_debug_msg(STR_TASK_CMD_DO "v: " STR_VERSION ".");
          break;


        // Si4463 radio: init and set TX mode.
        case 'x':
          user_debug_msg(STR_TASK_CMD_DO "x: Si4463 init + TX mode...");
          si4463_cmd_tx();
          break;


        // Si4463 radio: unmodulated CW carrier (for VNA S11 measurement).
        case 'c':
          user_debug_msg(STR_TASK_CMD_DO "c: Si4463 CW carrier for VNA...");
          si4463_cmd_cw();
          break;


        // Shut down PPM MCU, for static current measurement.         
        case 'z':
          user_debug_msg(STR_TASK_CMD_DO "z: Full shutdown in 2 s.");

          // Stop all tasks but this one.  OK to use <cmd> since we'll sleep before
          //  exiting this case.
          for (cmd=2; cmd<=OSTASKS; cmd++) {
            OSStopTask(OSTCBP(cmd));
          }

          // Wait a while to flush RS-232, etc.
          OS_Delay(100);

          // Final message.
          user_debug_msg(STR_TASK_CMD_DO "z: System is now shut down.");
          OS_Delay(100);

          // Shut everything down.
          csk_power_shutdown();
          break;


        default:
          sprintf(strTmp, STR_TASK_CMD_DO "Unknown command: '%c' (0x%X)", cmd, cmd);
          user_debug_msg(strTmp);
          break;

      } /* switch */
      
    } /* if */
    
  } /* while */
  
} /* task_cmd_do() */

