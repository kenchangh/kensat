/******************************************************************************
KENSAT - Telemetry Beacon Task

Periodically transmits housekeeping telemetry via Si4463 GFSK TX.
Default interval: 30 seconds.

******************************************************************************/
#ifndef __task_beacon_h
#define __task_beacon_h

extern void task_beacon(void);

/* VCC_SYS (3V3) rail telemetry in millivolts, refreshed by task_beacon every
   beacon cycle. Zero means "unknown / rail down". task_sdlog reads this for
   the CSV row; cmd handler can dump it. (No VBATT: no battery-sense hardware.) */
extern volatile unsigned int g_rail_3v3_mv;

/* Fresh VCC_SYS (3V3) read in mV, independent of the beacon cycle. */
extern unsigned int beacon_read_3v3(void);

/* 1 if the ground has commanded the transmitter silent (persistent). */
extern unsigned int beacon_inhibited(void);

/* Bench aid: run a string through the real uplink command dispatcher as if
   it had arrived over the air.  See the 'u' console command in task_cmd.c. */
extern void beacon_uplink_inject(const char *payload);

#define STR_TASK_BEACON "task_beacon: " STR_1TAB

#endif /* __task_beacon_h */
