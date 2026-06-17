/******************************************************************************
bootcount.h

Persistent boot counter stored in PIC24 program flash.

Used to detect unintended power-ups / resets of the OBC (e.g. during
vibration testing, where the system is supposed to stay off).  The count
survives power cycles and reprogramming-free resets.
******************************************************************************/
#ifndef __bootcount_h
#define __bootcount_h

/* Read the stored boot count.  Returns 0 if flash has never been written
   (blank or no valid magic). */
unsigned int bootcount_get(void);

/* Increment the stored boot count by 1, persist it to flash, and return
   the new value.  Call once early in init().  Performs a flash page
   erase + row write, so it blocks for a few ms. */
unsigned int bootcount_increment(void);

/* Reset the stored boot count to 0 (e.g. just before a vibration test). */
void bootcount_reset(void);

#endif /* __bootcount_h */
