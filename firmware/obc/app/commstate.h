/******************************************************************************
commstate.h

Persistent flag word for the comms subsystem, stored in PIC24 program flash
(its own reserved page, independent of bootcount.h and deploy.h so the three
never clobber each other).

Currently holds one bit: the commanded beacon inhibit ("RF silence") set by
an uplink telecommand.

WHY THIS IS IN FLASH AND THE COUNTERS ARE NOT
---------------------------------------------
The beacon sequence / TX / RX counters live on the SD card instead
(sd_counters_load/save in task_sdlog.h).  They change every beacon, which
program flash cannot absorb -- the page is rated ~10 k erase cycles and a
beacon every 10 s would destroy it in about a day -- whereas the card does
not care, so the counters can be checkpointed exactly rather than
approximated on an hourly schedule.

This flag is the opposite case.  It is written only when the ground sends
BCN ON / BCN OFF, i.e. a handful of times in the whole mission, so there is
no wear argument at all.  And it is the one piece of comms state that MUST
survive hardware the SD card cannot be trusted for: the card is removable,
socketed, and a plausible single-point failure on orbit.  If a commanded RF
silence evaporated because a card went missing, the spacecraft would start
transmitting again on its own -- exactly what the licensing requirement
exists to prevent.  So it stays in flash, which is always there.

Losing the counters to a dead card is survivable by comparison: the beacon
also carries bc= (persistent boot count, also flash) and rst= (reset cause),
and those are what actually let the ground tell a reboot from a gap in
coverage.

BEACON INHIBIT
--------------
COMMSTATE_FLAG_BEACON_INHIBIT is deliberately PERSISTENT and has no
auto-expiry: a licensing authority that requires a commandable shutdown
expects the shutdown to hold until it is commanded otherwise, not to quietly
undo itself.  It is cleared only by an uplink telecommand (or by
reprogramming the part, which resets the reserved page).
******************************************************************************/
#ifndef __commstate_h
#define __commstate_h

/* Flag bits in the persisted flag word. */
#define COMMSTATE_FLAG_BEACON_INHIBIT   0x0001u

/* Read the stored flag word.  Returns 0 if flash has never been written
   (blank page / bad magic). */
unsigned int commstate_flags_get(void);

/* Persist the flag word (erase + row program + read-back verify, retried a
   few times).  No-op if the value is already stored, so callers may call it
   unconditionally without burning an erase cycle.  Returns 1 if the value is
   in flash afterwards, 0 if the write would not stick.  Blocks a few ms with
   interrupts masked at IPL 7. */
unsigned char commstate_flags_set(unsigned int flags);

/* Clear the page back to zero (bench retest). */
void commstate_reset(void);

#endif /* __commstate_h */
