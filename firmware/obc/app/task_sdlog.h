/******************************************************************************
KENSAT SD Card Telemetry Logger

Periodically writes telemetry data to TELEM.CSV on the SD card.
Provides sd_read_last_telem() to restore telemetry across reboots.

******************************************************************************/
#ifndef __task_sdlog_h
#define __task_sdlog_h

#include "task_jetson.h"

// Function declarations
extern void task_sdlog(void);
extern unsigned char sd_read_last_telem(jetson_telem_t *telem);

// Symbols
#define STR_TASK_SDLOG "task_sdlog: " STR_1TAB

#endif /* __task_sdlog_h */
