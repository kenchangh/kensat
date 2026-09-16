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
#include "task_beacon.h"
#include "task_sdlog.h"
#include "task_bmi088.h"
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
  b:   show boot count
  c:   Si4463 CW carrier (VNA S11)
  d:   show antenna deploy state
  g:   Si4463 / clock diagnostics
  j:   start Jetson inference, or hard power cut if running
  k:   clear boot count
  l:   dump SD card files
  m:   cycle MCLK output settings
  n:   inject an LLM PROMPT uplink frame (bench builds only)
  p:   inject a PING uplink (bench builds only)
  q:   toggle console verbosity
  r:   restart application
  s:   Si4463 init + RX
  t:   on-chip temperature
  u:   inject BCN ON/OFF uplink (bench builds only)
  v:   version
  w:   power report: CPU idle %, BMI088 power state, PMD registers
  x:   Si4463 init + TX
  y:   clear antenna deployed flag
  z:   shut down
 
**                                                                           **
****                                                                       ****
******************************************************************************/
static
void cmd_explain(void) {

  user_debug_msg(STR_CMD_EXPLAIN "h|?: Commands: {h|?, b, c, d, g, i, j, k, l, m, n, p, q, r, s, t, u, v, w, x, y, z}");

} /* cmd_explain() */


void task_cmd_do(void) {

  unsigned char cmd;

  
  user_debug_msg(STR_TASK_CMD_DO "Starting.");
  OS_Delay(50);
  
  csk_status.MCLKOutEnabled = 0;
  
  // Startup status: boot count and a direct 3V3 read (g_rail_3v3_mv is not
  // populated yet).
  {
    unsigned int v3 = beacon_read_3v3();
    sprintf(strTmp, STR_TASK_CMD_DO "Boot count = %u, 3V3 = %u mV.",
            bootcount_get(), v3);
    user_debug_msg(strTmp);
    OS_Delay(50);
  }

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


        // Show boot count.
        case 'b':
          sprintf(strTmp, STR_TASK_CMD_DO "b: Boot count = %u.", bootcount_get());
          user_debug_msg(strTmp);
          break;


        // Clear boot count (run before vibration test).
        case 'k':
          bootcount_reset();
          user_debug_msg(STR_TASK_CMD_DO "k: Boot count cleared to 0.");
          break;


        // Dump SD files to the console.  sd_dump_file() doesn't yield, so a
        // large TELEM.CSV can outlast the TPS3430 watchdog margin and reset
        // the OBC; the small counter files go first so they always get out.
        case 'l':
          user_debug_msg(STR_TASK_CMD_DO "l: Dumping SD files...");
          sd_dump_file("COMMS0.DAT");
          sd_dump_file("COMMS1.DAT");
          sd_dump_file("BOOT.LOG");
          sd_dump_file("TELEM.CSV");
          sd_dump_file("INFER.LOG");
          break;


        // Show antenna deployment state.
        case 'd':
          sprintf(strTmp,
                  STR_TASK_CMD_DO "d: deployed = %u, attempts = %u, rf_unlocked = %u.",
                  deploy_get(), deploy_attempts_get(), deploy_rf_unlocked_get());
          user_debug_msg(strTmp);
          break;


        // Clear deployment flag (bench only); burn sequence reruns next
        // flight-mode boot.
        case 'y':
          deploy_reset();
          user_debug_msg(STR_TASK_CMD_DO "y: Antenna deployed flag cleared.");
          break;


        // Si4463 + PIC24 clock diagnostic dump.
        case 'g':
          user_debug_msg(STR_TASK_CMD_DO "g: Si4463 / clock diagnostic...");
          si4463_cmd_get_props();
          break;


        // 'i' (exercise_io) disabled; task slot reused by task_beacon.


        // Start a Jetson inference cycle, or hard power cut if already on.
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


        // Toggle MCLK output for frequency & duty-cycle check.
        case 'm':
          cmd_mclk();
          break;
        

        // Reset.
        case 'r':
          user_debug_msg(STR_TASK_CMD_DO "r: Reset in 1 s.");
          OS_Delay(100);
          csk_wdt_force();
          break;


#if DEVELOPMENT
        // Bench only: inject BCN ON/OFF through the uplink dispatcher.
        //  Compiled out of flight builds because of the phantom-command bug
        //  (stale SEM_CMD_CHAR_P counts; "Known issues" in
        //  firmware/obc/README.md).
        case 'u':
          if (beacon_inhibited())
            beacon_uplink_inject("KO6IKZ KENSAT1 BCN ON");
          else
            beacon_uplink_inject("KO6IKZ KENSAT1 BCN OFF");
          break;


        // Bench only: inject PING (expect an ACK downlink).
        case 'p':
          beacon_uplink_inject("KO6IKZ KENSAT1 PING");
          break;


        // Bench only: inject a PROMPT frame (256-byte NUL-padded field)
        //  through the uplink dispatcher, exercising ACK, Jetson handoff and
        //  TXT downlink.  Look for "uplink: PROMPT <n>B h=<crc32>".
        case 'n':
          beacon_uplink_inject_prompt("Describe the Earth from orbit in one line.");
          break;
#endif


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


        // Toggle console verbosity (see CONSOLE_VERBOSE_DEFAULT).
        case 'q':
          g_console_verbose = !g_console_verbose;
          sprintf(strTmp, STR_TASK_CMD_DO "q: console %s.",
                  g_console_verbose
                    ? "VERBOSE (telemetry lines + AX.25 hex dump)"
                    : "terse (one line per beacon)");
          user_debug_msg(strTmp);
          break;


        // Power report: CPU idle fraction since the last 'w', BMI088 power
        //  mode readback, and PMD module-disable registers.  The board has no
        //  current sensing, so this is the only power check.
        case 'w': {
          unsigned long now    = OSGetTicks();
          unsigned long counts = g_idle_counts;
          unsigned long span   = now - g_idle_mark_ticks;
          unsigned char lpm1 = 0, pwrconf = 0;

          g_idle_counts     = 0;
          g_idle_mark_ticks = now;

          if (span == 0) span = 1;
          /* Timer1 = 15625 Hz, so a 10 ms tick is 156.25 counts.
             Per-mille idle = 1000 * counts / (span * 156.25)
                            = counts * 64 / (span * 10). */
          {
            unsigned long permille = (counts * 64UL) / (span * 10UL);
            sprintf(strTmp, STR_TASK_CMD_DO
                    "w: CPU idle %lu.%lu%% over %lu.%lus",
                    permille / 10UL, permille % 10UL,
                    span / 100UL, (span / 10UL) % 10UL);
          }
          user_debug_msg(strTmp);

          if (bmi088_get_pwr_state(&lpm1, &pwrconf) == 0) {
            sprintf(strTmp, STR_TASK_CMD_DO
                    "w: BMI088 GYR_LPM1=%02X (%s) ACC_PWR_CONF=%02X (%s)",
                    lpm1,    (lpm1    == 0x80) ? "suspend" : "AWAKE",
                    pwrconf, (pwrconf == 0x03) ? "suspend" : "AWAKE");
          } else {
            sprintf(strTmp, STR_TASK_CMD_DO "w: BMI088 power readback FAILED");
          }
          user_debug_msg(strTmp);

          sprintf(strTmp, STR_TASK_CMD_DO
                  "w: PMD1=%04X PMD2=%04X PMD3=%04X PMD4=%04X PMD5=%04X PMD6=%04X",
                  PMD1, PMD2, PMD3, PMD4, PMD5, PMD6);
          user_debug_msg(strTmp);
          break;
        }


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

