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

 Upper boundary (Eq. 4):  tWDU(typ) = 77.4 * C_CWD[uF] + 0.055
                                    = 77.4 * 0.82 + 0.055 = 63.52 s
   tWDU(min) = 0.905 * 63.52 = 57.49 s
   tWDU(max) = 1.095 * 63.52 = 69.56 s

 SET0=0,SET1=1 gives a 3/4 ratio, so the lower boundary tWDL = 0.75 * tWDU:
   tWDL(max) = 0.75 * tWDU(max) = 52.17 s

 Per the WDI pin spec, a falling edge must land between tWDL(max) and
 tWDU(min) to keep WDO de-asserted:

        valid window  =  ( 52.17 s , 57.49 s )     (~5.3 s wide)

 We target a 54.0 s service interval (centre-biased toward the early side so
 that cooperative-scheduler jitter, which can only *delay* us, is absorbed by
 the larger late-side margin):
        early margin = 54.0 - 52.17 = 1.83 s
        late  margin = 57.49 - 54.0 = 3.49 s

 At power-up the watchdog is disabled until tRST (CRST = C60 = 0.1 uF ->
 ~0.32 s) expires, and the *first* WDI edge only has to arrive before
 tWDU(max) = 69.56 s (no lower bound). 54 s from task start satisfies that.

----------------------------------------------------------------------------
 Implementation note
----------------------------------------------------------------------------
 Salvo (OSLIBRARY_CONFIG=OST, OSBYTES_OF_DELAYS=1) caps OS_Delay() at 255
 ticks (~2.55 s @ 100 Hz). Long waits MUST be unrolled as multiple OS_Delay()
 calls at the task's top level - yielding from a helper or a counted while
 loop corrupts the stack on resume on this build. So the 54 s wait below is
 27 x OS_Delay(200) (= 5400 ticks = 54.00 s) written out in the loop body,
 matching the pattern in task_beacon.c.

 Falling-edge interval = 5400 ticks (HIGH) + 2 ticks (LOW) = 5402 = 54.02 s.
******************************************************************************/
#include "main.h"
#include "task_wdt.h"

#include "csk_io.h"
#include "csk_uart.h"

#include "salvo.h"


/******************************************************************************
 task_wdt()

 Salvo task entry point. Drives IO.7 (WDI) high, then kicks the TPS3430 with
 a brief low pulse every ~54 s, forever.
******************************************************************************/
void task_wdt(void) {

  /* Take RF4 (IO.7) as a digital output and park WDI high (armed). */
  _TRISF4 = 0;
  csk_io7_high();

  csk_uart1_puts(STR_TASK_WDT "TPS3430 service started (WDI = IO.7, ~54s).\r\n");

  while (1) {

    /* Hold WDI HIGH for ~54 s (the bulk of the watchdog window).
       27 x 200 ticks = 5400 ticks = 54.00 s, unrolled at top level. */
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);
    OS_Delay(200); OS_Delay(200);                                  /* = 27 */

    /* Kick the watchdog: WDI falling edge, then re-arm high. */
    csk_io7_low();      /* <-- FALLING EDGE feeds the TPS3430 */
    OS_Delay(2);        /* hold low 20 ms (>> 50 ns min WDI pulse) */
    csk_io7_high();     /* re-arm high for the next window */

  } /* while */

} /* task_wdt() */
