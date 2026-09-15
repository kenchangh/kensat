/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\main.h,v $
$Author: aek $
$Revision: 3.8 $
$Date: 2011-09-17 22:12:29-07 $

******************************************************************************/
#ifndef __main_h
#define __main_h


/* Flight-configuration gate.  This is THE switch that arms the antenna
   deployment hardware -- read it as "is this build going to space?".

   FLIGHT_MODE = 0 (FALSE, bench/default) -> the nichrome burn FETs on
                     IO.24 (RF1) / IO.25 (RF0) are NEVER energised: the deploy
                     block is compiled out entirely, not merely skipped at
                     run time.  The safe setting for all bench, vibration and
                     radio work.
   FLIGHT_MODE = 1 (TRUE, flight)         -> once uptime passes FLIGHT_HOLD_S
                     the burn wires fire once, subject to the persistent
                     DEPLOYED flag and the 3-attempt cap in deploy.h.

   NOT gated on FLIGHT_MODE: the RF-silence hold and the beacon itself.  The
   beacon task starts ~5 s after boot and transmits from FLIGHT_HOLD_S onwards
   in BOTH builds, regardless of whether deployment happened, succeeded, or
   was compiled out -- a faint pre-deploy signal is still a signal, and there
   is no deployment sense line to condition it on anyway. */
#define FLIGHT_MODE           1

/* ===================== TEMPORARY DEBUG DISARM =====================
   Added 2026-08-23 while chasing the FLIGHT_MODE boot loop.

   DEPLOY_ARMED = 0 -> the burn-wire block in task_beacon.c is compiled out
                       EVEN IN A FLIGHT_MODE BUILD.  Everything else about
                       FLIGHT_MODE stays live (RF hold, timings, the
                       flight-only console gating), so the boot loop can be
                       reproduced and debugged with zero chance of tripping
                       the nichrome.
   DEPLOY_ARMED = 1 -> normal: FLIGHT_MODE alone decides.

   >>> THIS MUST BE 1 FOR FLIGHT. <<<  It is deliberately a separate switch
   from FLIGHT_MODE so that "am I debugging?" and "is this build going to
   space?" cannot be confused for one another, and so this temporary state is
   greppable.  The boot banner reports it (see STR_FLIGHT_MODE below) -- do
   not trust FLIGHT_MODE=true in the banner to mean the wires are live.
   =================================================================== */
#define DEPLOY_ARMED          1

/* Post-power-on hold, seconds.  One constant, two uses, deliberately equal:
     - the deployer-safety hold before the burn wires may be energised, and
     - the RF-silence compliance hold before any carrier goes out.
   Both are measured against OSGetTicks(), which restarts at 0 on every boot,
   so a reset re-honours the full hold.  The RF half latches once it has been
   satisfied (deploy_rf_unlock(), see deploy.h) so later reboots transmit
   immediately; the deploy half is latched separately by the DEPLOYED flag. */
#define FLIGHT_HOLD_S         125UL

/* Legacy alias.  Its one remaining job is gating the bench-only 'u' / 'p'
   uplink-injection console commands in task_cmd.c, which stay compiled out of
   flight builds because of the stale-semaphore phantom-command bug (see
   "Known issues" in firmware/obc/README.md -- a spurious command must never
   be able to silence the transmitter on orbit). */
#define DEVELOPMENT           (!FLIGHT_MODE)

#if FLIGHT_MODE
  #if DEPLOY_ARMED
    #define STR_FLIGHT_MODE   "FLIGHT_MODE=true (flight build, ANTENNA DEPLOY ARMED)"
  #else
    #define STR_FLIGHT_MODE   "FLIGHT_MODE=true (flight build, DEPLOY DISARMED -- DEBUG BUILD, NOT FLIGHT)"
  #endif
#else
  #define STR_FLIGHT_MODE     "FLIGHT_MODE=false (bench build, deploy disabled)"
#endif

/* Deployment burn-wire bench test.
   When 1, task_beacon fires a one-shot deploy pulse 10 s after startup:
   nichrome gate FETs on IO.24 (RF1) + IO.25 (RF0) are driven HIGH for 10 s,
   then back LOW.  Runs once per boot, independent of the flight auto-deploy
   sequence and the persistent DEPLOYED flag (deploy.h) -- it is purely a
   bench burn test of the FETs / burn wires.
   !!! Energises nichrome IO.24+IO.25 HIGH for 10 s -- disconnect the burn
   wires or fit a dummy load.  MUST be 0 for flight / normal bench work. */
#define DEPLOY_TEST_MODE      0

/* ---------------------------------------------------------------------------
   Power management (beacon mode)
   --------------------------------------------------------------------------- */

/* POWER_IDLE = 1 -> main()'s scheduler loop executes PWRSAV #1 (Idle) after
   every OSSched() pass: the CPU core clock halts until the next interrupt,
   while Timer2 (opened T2_IDLE_CON), both UARTs, SPI and I2C keep running.
   Between beacons every task is delayed, so without this the core spins at
   4 MIPS for the whole gap for no work.

   Deliberately NOT gated on __DEBUG: the MPLAB default configuration defines
   __DEBUG for ordinary builds too, so keying off it would silently disable
   idling everywhere.  Set to 0 by hand if an attached ICD/PICkit needs the
   core running continuously. */
#define POWER_IDLE            1

/* Debug-console verbosity.
   terse (0)   -- one summary line per beacon cycle.
   verbose (1) -- adds the rails/tumble telemetry lines and the ~6-line AX.25
                  hex dump of every transmitted frame.
   That dump is ~500 chars; at 9600 baud it is ~0.5 s per cycle during which
   uart2_msg_ts busy-waits on TX ring space and the CPU cannot idle, so terse
   is the default and verbose is a bench aid.  Toggle at runtime with 'q'.
   The console is not the system of record -- TELEM.CSV on the SD card and the
   ground-station decode of the actual RF both still carry everything. */
#define CONSOLE_VERBOSE_DEFAULT  0

extern volatile unsigned char g_console_verbose;

/* Idle-fraction instrumentation; see main.c and the 'w' console command.
   g_idle_counts accumulates Timer1 counts (Fcy/256) spent halted in Idle;
   g_idle_mark_ticks is the OSGetTicks() value when the window opened. */
extern volatile unsigned long g_idle_counts;
extern volatile unsigned long g_idle_mark_ticks;


// Symbols, etc.
#define STR_APP_NAME          "Test\\Test1 application"
#define STR_VERSION           "v1.2.4" " built on " __DATE__ " at " __TIME__
#define STR_WARNING           "WARNING: Use 'z' command with caution -- refer to project's abstract.txt"
#define STR_BAUD_RATE         "9600"
#define STR_1TAB              "\t"
#define STR_2TABS             "\t\t"
#define STR_CRLF              "\r\n"
  
// Macros for user-readable messages.
#define user_debug_msg(x)     csk_uart1_msg_ts(x)
#define data_debug_msg(x)     csk_uart1_msg_ts(x)

// Other macros.
#define LOOP_HERE()            do { ; } while (1)

// Target-specific symbols.
#define SYSTEM_TIMER_RELOAD    328
#define NOP                    _NOP()

// Function declarations.
extern void init_devices(void);

/* Reset-cause reporting (captured from RCON at the top of init(), see init.c).
   The condensed byte below rides in every beacon as "rst=%02X" so the ground
   can tell a watchdog timeout / brownout / trap from an ordinary power-up. */
#define RESET_CAUSE_POR        0x01    /* power-on reset                    */
#define RESET_CAUSE_BOR        0x02    /* brown-out reset                   */
#define RESET_CAUSE_WDT        0x04    /* watchdog time-out                 */
#define RESET_CAUSE_SWR        0x08    /* software reset (RESET instruction)*/
#define RESET_CAUSE_EXTR       0x10    /* external /MCLR                    */
#define RESET_CAUSE_CM         0x20    /* configuration mismatch            */
#define RESET_CAUSE_IOPUWR     0x40    /* illegal opcode / uninitialised W  */
#define RESET_CAUSE_TRAPR      0x80    /* trap conflict                     */

extern unsigned int  g_rcon_boot;      /* raw RCON snapshot from this boot */
extern unsigned char reset_cause_code(void);

// Structure declarations  
typedef struct {
  unsigned int usb_present:1;
  unsigned int usb_connected:1;
  unsigned int mhx_connected:1;
  unsigned int MCLKOutEnabled:2;
  unsigned int exercise_io_running:1;
  unsigned int jetson_power_on:1;
  unsigned int jetson_run_requested:1;
} csk_status_t;

// Extern variable declarations.
extern csk_status_t csk_status;
extern char strTmp[];


#endif /* __main_h */






