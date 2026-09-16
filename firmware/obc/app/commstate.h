/******************************************************************************
commstate.h

Persistent comms flag word in PIC24 program flash (own page, separate from
bootcount.h and deploy.h).  Holds the beacon inhibit (RF silence) set by
uplink BCN OFF.

The inhibit lives in flash, not on the SD card, so a commanded shutdown
survives a card failure (licensing requirement).  It is written rarely, so
wear is not an issue.  It has no expiry; only BCN ON or a reflash clears it.
The beacon counters change every beacon and would wear out flash (~10 k
cycles), so they live on the SD card (sd_counters_load/save, task_sdlog.h).
******************************************************************************/
#ifndef __commstate_h
#define __commstate_h

/* Flag bits. */
#define COMMSTATE_FLAG_BEACON_INHIBIT   0x0001u

/* Stored flags, or 0 if the page is blank. */
unsigned int commstate_flags_get(void);

/* Write the flags (erase, program, verify, with retries); no-op if already
   stored.  Returns 1 if the value is in flash, 0 on failure.  Blocks a few
   ms at IPL 7. */
unsigned char commstate_flags_set(unsigned int flags);

/* Clear the page back to zero (bench retest). */
void commstate_reset(void);

#endif /* __commstate_h */
