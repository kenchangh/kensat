/******************************************************************************
deploy.h

Persistent antenna deployment state in PIC24 program flash: deployed flag,
burn attempt count and RF-unlock flag.  Once deployed, the burn wires
(IO.24/IO.25) never fire again; a reboot mid-burn retries up to
DEPLOY_MAX_ATTEMPTS times.  Reflashing clears everything.
******************************************************************************/
#ifndef __deploy_h
#define __deploy_h

#define DEPLOY_MAX_ATTEMPTS   3u

/* 1 if deployment has completed, else 0. */
unsigned int deploy_get(void);

/* Number of burn attempts started. */
unsigned int deploy_attempts_get(void);

/* Increment and persist the attempt count; returns the new value.  Call just
   before energising the wires.  Blocks a few ms. */
unsigned int deploy_attempt_inc(void);

/* Mark deployed (keeps the attempt count).  Call after the burn completes.
   Blocks a few ms. */
void deploy_set(void);

/* 1 if the FLIGHT_HOLD_S RF-silence hold has been satisfied once, else 0. */
unsigned int deploy_rf_unlocked_get(void);

/* Latch rf_unlocked (writes flash only the first time).  Call once
   FLIGHT_HOLD_S has elapsed.
   Bench builds latch this too.  Before flight, clear it with 'y' (or
   reflash) and confirm with 'd', or the satellite will transmit with no
   hold after deployment. */
void deploy_rf_unlock(void);

/* Clear deployed flag, attempt count and rf_unlocked (bench testing). */
void deploy_reset(void);

#endif /* __deploy_h */
