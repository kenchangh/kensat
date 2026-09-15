/******************************************************************************
KENSAT - External Watchdog (TPS3430) Service Task

The TPS3430 (U14) is a window watchdog. It drives -RESET (WDO) and reboots the
OBC unless it sees a falling edge on its WDI input inside a valid time window.
This task generates that handshake.

----------------------------------------------------------------------------
 Hardware
----------------------------------------------------------------------------
 WDI  = CSK IO.7 (header H1.17) = PIC24 RF4 (driven by csk_io7_high/low()).
        RF4 was the old URX1/MHX RX, unused on this board, so it is free.
        init() leaves RF4 as an input; this task takes it over as an output.
 WDI idles HIGH; the watchdog triggers on the FALLING edge, so each service
 cycle pulses WDI LOW briefly and returns it HIGH.

----------------------------------------------------------------------------
 Timing window (derived from the TPS3430 datasheet, SBVS366A)
----------------------------------------------------------------------------
 Board strapping: CWD = C59 = 0.82 uF cap to GND, SET0 = 0, SET1 = 1.

 Datasheet derivation (Eq. 4):  tWDU(typ) = 77.4 * C_CWD[uF] + 0.055
                                          = 77.4 * 0.82 + 0.055 = 63.52 s
   tWDU(min) = 0.905 * 63.52 = 57.49 s
   tWDU(max) = 1.095 * 63.52 = 69.56 s

 MEASURED on this board, which is what we actually design against:
 with the WDI edge never delivered, reset-to-reset was 55.09 / 55.27 /
 55.08 / 55.33 s over four consecutive cycles -- i.e. the real upper
 boundary is

        tWDU(actual) ~= 55 s

 That is BELOW the datasheet tWDU(min) of 57.49 s, so C59 is evidently
 nearer 0.71 uF than its nominal 0.82 uF (tolerance + temperature).  Any
 interval derived from the nominal cap value is therefore unsafe; the
 numbers below come from the measurement.

 SET0=0,SET1=1 gives a 3/4 ratio, so the lower boundary tracks the upper:
   tWDL = 0.75 * tWDU ~= 41.3 s

        valid window  =  ( 41.3 s , 55 s )     (~13.7 s wide)

 We target a 46.0 s service interval, biased toward the early side because
 cooperative-scheduler jitter can only ever *delay* the kick:
        early margin = 46.0 - 41.3 = 4.7 s
        late  margin = 55.0 - 46.0 = 9.0 s

 The previous 54.0 s target sat only 1.0 s inside the measured upper
 boundary, and was missed outright: task_beacon blocks the cooperative
 scheduler for ~20 s per transmission (the post-TX wait in
 si4463_tx_ax25()), so a TX starting at ~41 s uptime ran straight through
 the 54 s deadline and the watchdog reset the OBC at ~55 s -- every boot,
 before the first WDI edge was ever emitted.  That busy-wait has been cut
 to ~0.5 s; keep any new long busy-wait well under the 9 s late margin, or
 service WDI from inside it.

 At power-up the watchdog is disabled until tRST (CRST = C60 = 0.1 uF ->
 ~0.32 s) expires, and the *first* WDI edge has no lower bound -- it only
 has to arrive before tWDU, which 46 s from task start satisfies.

----------------------------------------------------------------------------
 Implementation note
----------------------------------------------------------------------------
 Salvo (OSLIBRARY_CONFIG=OST, OSBYTES_OF_DELAYS=1) caps OS_Delay() at 255
 ticks (~2.55 s @ 100 Hz). Long waits MUST be unrolled as multiple OS_Delay()
 calls at the task's top level - yielding from a helper or a counted while
 loop corrupts the stack on resume on this build.

 The 46 s wait used to be built as 23 x OS_Delay(200) chained back to back.
 That is RELATIVE timing, and it drifts: each delay only starts counting once
 the task is dispatched after the previous one expired, so every stretch where
 another task holds the (cooperative) CPU is added to the total.  task_beacon
 blocks for a good fraction of a second per cycle -- hex-dumping each AX.25
 frame to a 9600 baud console, the post-TX settle, and now the uplink
 listening window -- and 23 chained delays accumulated enough slip that the
 kick landed AFTER the ~55 s deadline instead of at 46 s.  The TPS3430 then
 reset the OBC on every boot, before a single WDI edge was ever emitted.

 So the deadline is now ABSOLUTE.  The task wakes on a short 1 s heartbeat and
 compares OSGetTicks() against the timestamp of the last kick, which makes the
 interval independent of how much drift the heartbeat itself accumulates: the
 only requirement is that no single uninterruptible stretch exceeds the ~9 s
 of late margin.  Anything added to another task can now cost latency, but it
 can no longer walk the kick out of the window.

 The WDI low pulse is a CPU busy-wait rather than OS_Delay(2), so there is no
 yield inside the conditional -- keeping every OS_Delay in this task at the
 top level of while(1), per the Salvo constraint above.
******************************************************************************/
#include "main.h"
#include "task_wdt.h"

#include "csk_io.h"
#include "csk_uart.h"

#include "salvo.h"


/* Target service interval: 46.00 s at the 100 Hz tick -- centre-biased inside
   the MEASURED (41.3 s, 55 s) window, early side, per the derivation above. */
#define WDT_KICK_TICKS   4600UL

/* Tick timestamp of the last WDI falling edge.  File scope rather than an
   automatic, so it cannot straddle an OS_Delay yield. */
static unsigned long wdt_last_kick = 0;


/******************************************************************************
 task_wdt()

 Salvo task entry point. Drives IO.7 (WDI) high, then kicks the TPS3430 with
 a brief low pulse every ~46 s (absolute, not cumulative), forever.
******************************************************************************/
void task_wdt(void) {

  /* Take RF4 (IO.7) as a digital output and park WDI high (armed). */
  _TRISF4 = 0;
  csk_io7_high();

  wdt_last_kick = OSGetTicks();

  csk_uart1_puts(STR_TASK_WDT "TPS3430 service started (WDI = IO.7, ~46s).\r\n");

  while (1) {

    /* Short heartbeat.  Waking once a second costs nothing and bounds how
       late the absolute deadline below can be noticed. */
    OS_Delay(100);

    if ((OSGetTicks() - wdt_last_kick) >= WDT_KICK_TICKS) {

      /* Kick the watchdog: WDI falling edge, then re-arm high.  Busy-wait
         (not OS_Delay) so this conditional never yields -- ~10 ms low, far
         beyond the 50 ns minimum WDI pulse. */
      csk_io7_low();          /* <-- FALLING EDGE feeds the TPS3430 */
      { volatile unsigned int d;
        for (d = 0; d < 5000u; d++) { __builtin_nop(); } }
      csk_io7_high();         /* re-arm high for the next window */

      /* Timestamp AFTER the pulse, so the measured interval is edge-to-edge
         and can only ever be >= WDT_KICK_TICKS -- never early enough to trip
         the window watchdog's lower bound (~41 s). */
      wdt_last_kick = OSGetTicks();
    }

  } /* while */

} /* task_wdt() */
