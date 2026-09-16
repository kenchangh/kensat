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


/* Flight build switch; arms antenna deployment.
   0 = bench: deploy code (nichrome FETs on IO.24/RF1, IO.25/RF0) is compiled
       out.  Use for all bench, vibration and radio work.
   1 = flight: burn wires fire once uptime passes FLIGHT_HOLD_S, subject to
       the DEPLOYED flag and 3-attempt cap in deploy.h.
   The RF-silence hold and the beacon run in both builds. */
#define FLIGHT_MODE           1

/* Temporary debug disarm (added 2026-08-23 for the FLIGHT_MODE boot loop).
   0 = burn-wire code in task_beacon.c is compiled out even with FLIGHT_MODE=1;
       everything else in the flight build stays live.
   1 = normal, FLIGHT_MODE decides.
   MUST BE 1 FOR FLIGHT.  The boot banner (STR_FLIGHT_MODE) shows this state. */
#define DEPLOY_ARMED          1

/* Hold after power-on, seconds, before the burn wires may fire and before any
   RF transmission.  Measured from OSGetTicks(), so it restarts on every boot.
   The RF hold latches once satisfied (deploy_rf_unlock() in deploy.h); the
   deploy hold is latched by the DEPLOYED flag. */
#define FLIGHT_HOLD_S         125UL

/* Gates the bench-only 'u'/'p'/'n' uplink-injection commands in task_cmd.c.  They
   stay out of flight builds because of the phantom-command bug ("Known
   issues" in firmware/obc/README.md). */
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

/* Burn-wire bench test.  When 1, task_beacon drives IO.24 (RF1) and IO.25
   (RF0) high for 10 s, 10 s after startup, once per boot, ignoring the
   DEPLOYED flag.  Disconnect the burn wires or fit a dummy load.
   MUST be 0 for flight and normal bench work. */
#define DEPLOY_TEST_MODE      0

/* ---------------------------------------------------------------------------
   Power management (beacon mode)
   --------------------------------------------------------------------------- */

/* 1 = main() enters Idle (PWRSAV #1) after every OSSched() pass; the core
   halts until the next interrupt while Timer2, UARTs, SPI and I2C keep
   running.  Not keyed off __DEBUG because MPLAB defines that for ordinary
   builds too.  Set to 0 if a debugger needs the core running. */
#define POWER_IDLE            1

/* Console verbosity: 0 = one line per beacon cycle, 1 = also rails/tumble
   lines and an AX.25 hex dump of each frame (~0.5 s of busy-wait at 9600
   baud).  Toggle at runtime with 'q'. */
#define CONSOLE_VERBOSE_DEFAULT  0

extern volatile unsigned char g_console_verbose;

/* Idle-fraction measurement, reported by the 'w' command (see main.c). */
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

/* Reset cause bits, from RCON captured in init().  Sent in the beacon as
   "rst=%02X". */
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






