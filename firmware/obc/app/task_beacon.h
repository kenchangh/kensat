/******************************************************************************
KENSAT - Telemetry Beacon Task

Periodically transmits housekeeping telemetry via Si4463 GFSK TX.
Default interval: 30 seconds.

******************************************************************************/
#ifndef __task_beacon_h
#define __task_beacon_h

extern void task_beacon(void);

/* EPS rail telemetry, refreshed by task_beacon every beacon cycle.
   Both expressed in millivolts. Zero means "unknown / rail down".
   task_sdlog reads these for the CSV row; cmd handler can dump them. */
extern volatile unsigned int g_rail_5v_mv;
extern volatile unsigned int g_rail_3v3_mv;
extern volatile unsigned int g_vbatt_mv;

#define STR_TASK_BEACON "task_beacon: " STR_1TAB

#endif /* __task_beacon_h */
