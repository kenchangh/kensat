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


/* Build-time mode flag.
   DEVELOPMENT = 1 -> bench mode: skip launch sequence, beacon starts after
                      the short 5 s startup delay (existing behaviour).
   DEVELOPMENT = 0 -> flight mode: wait 120 s after boot, fire the nichrome
                      antenna burn wires on IO.24 and IO.25, wait another
                      10 s, then begin beaconing. */
#define DEVELOPMENT           1

#if DEVELOPMENT
  #define STR_DEVELOPMENT     "DEVELOPMENT=true (bench mode)"
#else
  #define STR_DEVELOPMENT     "DEVELOPMENT=false (flight mode)"
#endif

/* BENCH_TEST mirrors the build mode: on the bench (DEVELOPMENT=1) the antenna
   burn wires must NEVER be energised.  In flight (DEVELOPMENT=0) the wires are
   fired once, gated additionally by the persistent DEPLOYED flag (deploy.h)
   so a reboot after a successful deploy does not re-fire them. */
#define BENCH_TEST            DEVELOPMENT

/* TEMPORARY -- bench deployment testing.
   When 1, the 'd' console command FIRES the antenna burn-wire sequence
   immediately (no 120 s hold), honouring the persistent DEPLOYED flag and the
   3-attempt limit (deploy.h).  When 0, 'd' just prints the deploy state (its
   normal behaviour) and the wires can only fire via the flight auto-sequence.
   !!! Firing energises nichrome IO.24+IO.25 HIGH for 10 s -- disconnect the
   burn wires or fit a dummy load.  MUST be 0 for flight / normal bench work. */
#define DEPLOY_TEST           0

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






