/******************************************************************************
KENSAT SD Card Telemetry Logger

SD card logging, telemetry restore and beacon counter checkpoints.

******************************************************************************/
#ifndef __task_sdlog_h
#define __task_sdlog_h

#include "task_jetson.h"

// Function declarations
extern void task_sdlog(void);
extern unsigned char sd_read_last_telem(jetson_telem_t *telem);
extern void sd_dump_file(const char *name);

/* Beacon counters, saved on every beacon.  Kept on SD rather than program
   flash (~10k erase cycles); see commstate.h.  Neither call yields. */
#define SD_CTR_NO_CARD  0   /* not mounted: retry later, don't save    */
#define SD_CTR_LOADED   1   /* valid checkpoint restored               */
#define SD_CTR_EMPTY    2   /* no valid checkpoint: start at 0         */

extern unsigned char sd_counters_load(unsigned long *seq,
                                      unsigned long *tx,
                                      unsigned long *rx);
extern unsigned char sd_counters_save(unsigned long seq,
                                      unsigned long tx,
                                      unsigned long rx);

// Symbols
#define STR_TASK_SDLOG "task_sdlog: " STR_1TAB

#endif /* __task_sdlog_h */
