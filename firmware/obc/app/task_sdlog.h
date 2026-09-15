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
extern void sd_dump_file(const char *name);

/* Cumulative beacon counters, checkpointed to the card on every beacon.
   They live here rather than in program flash because flash is rated ~10k
   erase cycles and could not survive that write rate; see commstate.h for
   the split (the commanded beacon-inhibit flag does stay in flash).
   Neither call yields, so both are safe from a Salvo helper. */
#define SD_CTR_NO_CARD  0   /* not mounted -- retry later, do NOT save */
#define SD_CTR_LOADED   1   /* valid checkpoint restored               */
#define SD_CTR_EMPTY    2   /* card up, no valid checkpoint -- start at 0 */

extern unsigned char sd_counters_load(unsigned long *seq,
                                      unsigned long *tx,
                                      unsigned long *rx);
extern unsigned char sd_counters_save(unsigned long seq,
                                      unsigned long tx,
                                      unsigned long rx);

// Symbols
#define STR_TASK_SDLOG "task_sdlog: " STR_1TAB

#endif /* __task_sdlog_h */
