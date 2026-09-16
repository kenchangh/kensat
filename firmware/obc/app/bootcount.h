/******************************************************************************
bootcount.h

Persistent boot counter in PIC24 program flash.  Detects unintended
power-ups (e.g. during vibration testing).
******************************************************************************/
#ifndef __bootcount_h
#define __bootcount_h

/* Stored boot count, or 0 if the page is blank. */
unsigned int bootcount_get(void);

/* Increment and persist; returns the new count.  Called once from init().
   Blocks a few ms. */
unsigned int bootcount_increment(void);

/* Reset the stored boot count to 0 (e.g. just before a vibration test). */
void bootcount_reset(void);

#endif /* __bootcount_h */
