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
  i:   toggle Exercise I/O task on/off
  m:   cycle between various MCLK output settings
  n:   inject a two-chunk uplinked LLM prompt + RUN (bench builds only)
  r:   restart application
  q:   toggle debug-console verbosity (terse <-> full)
  w:   power report: CPU idle %, sensor power state, PMD registers
  t:   display on-chip temperature
  v:   display version number
  z:   go to sleep
 
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
  
  // Startup status: persistent boot count (program flash) + a fresh VCC_SYS
  // (3V3) read (the beacon cycle hasn't populated g_rail_3v3_mv yet).
  // No battery SoC: this board has no battery-sense hardware.
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


        // Dump SD card log files to the console (read the card without
        // pulling it). sd_dump_file() doesn't yield, so the WHOLE dump is one
        // uninterruptible stretch -- and at 9600 baud a few KB of TELEM.CSV
        // easily outruns the ~9 s of late margin task_wdt has before the
        // TPS3430 resets the OBC. Bench-only command, but that is why the two
        // tiny counter files are dumped FIRST: they are the ones you usually
        // want, and they are guaranteed to make it out before the big logs
        // blow the deadline.
        case 'l':
          user_debug_msg(STR_TASK_CMD_DO "l: Dumping SD files...");
          sd_dump_file("COMMS0.DAT");
          sd_dump_file("COMMS1.DAT");
          sd_dump_file("BOOT.LOG");
          sd_dump_file("TELEM.CSV");
          sd_dump_file("INFER.LOG");
          break;


        // 'd': shows persistent antenna deployment state (program flash).
        case 'd':
          sprintf(strTmp,
                  STR_TASK_CMD_DO "d: deployed = %u, attempts = %u, rf_unlocked = %u.",
                  deploy_get(), deploy_attempts_get(), deploy_rf_unlocked_get());
          user_debug_msg(strTmp);
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


#if DEVELOPMENT
        // Bench only: fake an uplink telecommand, toggling the beacon
        //  inhibit through the real command dispatcher.  Compiled out for
        //  flight (DEVELOPMENT=0) so the phantom-command bug in this task
        //  -- stale SEM_CMD_CHAR_P counts after a long UART1 response, see
        //  "Known issues" in firmware/obc/README.md -- can never silence the
        //  transmitter on orbit.
        case 'u':
          if (beacon_inhibited())
            beacon_uplink_inject("KO6IKZ KENSAT1 BCN ON");
          else
            beacon_uplink_inject("KO6IKZ KENSAT1 BCN OFF");
          break;


        // Bench only: fake a PING telecommand (expects an ACK downlink).
        case 'p':
          beacon_uplink_inject("KO6IKZ KENSAT1 PING");
          break;


        // Bench only: fake an uplinked LLM prompt -- two chunks plus RUN,
        //  pushed through the real uplink dispatcher.  Exercises the whole
        //  chain with no ground transmitter: chunk reassembly, the ACK
        //  bitmaps, the handoff to task_jetson, "PROMPT:<text>" on the
        //  payload UART, and then the TXT text downlink of whatever the
        //  model generates.  Watch for "prompt chunk stored" then
        //  "COMPLETE, awaiting RUN".
        case 'n':
          beacon_uplink_inject("KO6IKZ KENSAT1 PR 1/2 Describe the Earth ");
          beacon_uplink_inject("KO6IKZ KENSAT1 PR 2/2 from orbit in one line.");
          beacon_uplink_inject("KO6IKZ KENSAT1 RUN");
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


        // Toggle debug-console verbosity.  Terse (the default) is one line per
        //  beacon; verbose adds the rails/tumble lines and the AX.25 hex dump
        //  of every frame -- ~500 extra chars per cycle, which at 9600 baud is
        //  ~0.5 s of busy-waited UART in which the CPU cannot idle.
        case 'q':
          g_console_verbose = !g_console_verbose;
          sprintf(strTmp, STR_TASK_CMD_DO "q: console %s.",
                  g_console_verbose
                    ? "VERBOSE (telemetry lines + AX.25 hex dump)"
                    : "terse (one line per beacon)");
          user_debug_msg(strTmp);
          break;


        // Power report.  Three things, none of which the board can sense
        //  electrically (there is no current-sense hardware anywhere on it):
        //   1. CPU idle fraction -- what POWER_IDLE actually buys, measured.
        //   2. BMI088 power-mode readback -- proves the sensors really are
        //      suspended between samples rather than the writes silently
        //      failing, which would look identical from the readings.
        //   3. PMD registers -- proves the module-disable bits stuck.
        //  The idle window is re-zeroed on every call, so two 'w's separated
        //  by an interval measure that interval. Compare terse vs verbose
        //  ('q') to see what console traffic costs in idle time.
        case 'w': {
          unsigned long now    = OSGetTicks();
          unsigned long counts = g_idle_counts;
          unsigned long span   = now - g_idle_mark_ticks;
          unsigned char lpm1 = 0, pwrconf = 0;

          g_idle_counts     = 0;
          g_idle_mark_ticks = now;

          if (span == 0) span = 1;
          /* Timer1 runs at Fcy/256 = 15625 Hz, so a fully-idle 10 ms tick is
             156.25 counts and a fully-idle window is span * 156.25.  Hence
             1000 * counts / (span * 156.25) == counts * 64 / (span * 10),
             which is tenths of a percent -- no float, and the intermediate
             stays inside 32 bits for windows of several hours. */
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

