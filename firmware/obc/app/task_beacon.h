/******************************************************************************
KENSAT - Telemetry Beacon Task

Housekeeping beacon, LLM result downlink and uplink telecommands over the
Si4463.

******************************************************************************/
#ifndef __task_beacon_h
#define __task_beacon_h

extern void task_beacon(void);

/* VCC_SYS (3V3) in mV, updated every beacon cycle. 0 = unknown/rail down. */
extern volatile unsigned int g_rail_3v3_mv;

/* Fresh VCC_SYS (3V3) read in mV, independent of the beacon cycle. */
extern unsigned int beacon_read_3v3(void);

/* 1 if the ground has commanded TX off (persistent). */
extern unsigned int beacon_inhibited(void);

/* Bench aid: dispatch a string as if it were an uplink frame ('u' command). */
extern void beacon_uplink_inject(const char *payload);

/* Bench aid: wrap text in a NUL-padded PROMPT frame and dispatch it
   ('n' command). */
extern void beacon_uplink_inject_prompt(const char *text);

#define STR_TASK_BEACON "task_beacon: " STR_1TAB

#endif /* __task_beacon_h */
