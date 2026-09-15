/******************************************************************************
deploy.h

Persistent antenna-deployment state stored in PIC24 program flash.

Records whether the nichrome burn-wire deployment has COMPLETED, plus how many
burn attempts have been started.  Both survive power cycles / resets, so:

  - the OBC will not re-fire the burn wires (IO.24 / IO.25) after a reboot once
    the antenna has already been deployed, and
  - a reboot mid-burn (e.g. brownout from the high-current load) retries on the
    next boot, but only up to DEPLOY_MAX_ATTEMPTS times.

A reflash of the .hex resets the reserved page to its initializer, clearing
both fields (i.e. a freshly programmed board is "not deployed, 0 attempts").
******************************************************************************/
#ifndef __deploy_h
#define __deploy_h

/* Maximum number of burn attempts before giving up. */
#define DEPLOY_MAX_ATTEMPTS   3u

/* Return 1 if the antenna deployment has completed, else 0 (blank flash /
   never written / explicitly reset). */
unsigned int deploy_get(void);

/* Return the number of burn attempts started so far. */
unsigned int deploy_attempts_get(void);

/* Increment the persisted burn-attempt count, return the new value.  Call
   this just BEFORE energising the wires, so a reboot during the burn still
   counts against the attempt limit.  Blocks a few ms (flash erase + write). */
unsigned int deploy_attempt_inc(void);

/* Mark the antenna as deployed and persist to flash (preserves the attempt
   count).  Call only AFTER the burn fully completes.  Blocks a few ms. */
void deploy_set(void);

/* Return 1 if the initial FLIGHT_HOLD_S RF-silence compliance hold has already
   been satisfied once (so later reboots may transmit immediately), else 0. */
unsigned int deploy_rf_unlocked_get(void);

/* Persist the rf_unlocked flag (idempotent -- only writes flash the first
   time).  Call once the initial FLIGHT_HOLD_S hold has elapsed.

   NOTE: that hold applies to bench builds too (it is not gated on
   FLIGHT_MODE), so an ordinary bench session long enough to clear it will
   latch this flag.  A board flown in that state transmits from its first
   orbital loop with no wait -- clear the page with deploy_reset() (console
   'y') as the last step before flight, or reflash, then confirm with 'd'. */
void deploy_rf_unlock(void);

/* Clear all persisted deployment state -- deployed flag, attempt count and
   rf_unlocked flag (e.g. for bench testing so the sequence runs again). */
void deploy_reset(void);

#endif /* __deploy_h */
