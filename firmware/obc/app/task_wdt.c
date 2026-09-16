/******************************************************************************
KENSAT - External Watchdog (TPS3430) Service Task

The TPS3430 (U14) is a window watchdog: it resets the OBC unless WDI sees a
falling edge inside its valid window.

 WDI = CSK IO.7 (H1.17) = PIC24 RF4 (old URX1/MHX RX, unused).  init() leaves
 RF4 as an input; this task makes it an output.  WDI idles high and is
 pulsed low to kick.

 Timing (TPS3430 datasheet SBVS366A).  CWD = C59 = 0.82 uF, SET0=0, SET1=1.
   Datasheet: tWDU = 77.4 * 0.82 + 0.055 = 63.5 s typ (57.5 min, 69.6 max)
   Measured on this board, WDI never kicked: 55.09 / 55.27 / 55.08 / 55.33 s
 so the real tWDU is ~55 s, below datasheet min; design to the measurement.
 SET0=0, SET1=1 gives tWDL = 0.75 * tWDU ~= 41.3 s.

   valid window = (41.3 s, 55 s)
   target 46 s:   early margin 4.7 s, late margin 9.0 s

 Scheduler jitter can only make the kick late, hence the early bias.  No
 task may block for longer than the ~9 s late margin.  (An earlier 54 s
 target was missed every boot because si4463_tx_ax25() blocked for ~20 s.)

 The first edge after power-up has no lower bound, only tWDU.  tRST
 (CRST = C60 = 0.1 uF) is ~0.32 s.

 The deadline is absolute: a 1 s heartbeat compares OSGetTicks() with the
 last kick.  Chained relative OS_Delay()s drifted past 55 s under load.
 OS_Delay is capped at 255 ticks and must stay at the task's top level, so
 the WDI pulse is a busy-wait.
******************************************************************************/
#include "main.h"
#include "task_wdt.h"

#include "csk_io.h"
#include "csk_uart.h"

#include "salvo.h"


/* Service interval, 100 Hz ticks (46 s). */
#define WDT_KICK_TICKS   4600UL

/* Tick of the last WDI edge.  Static so it survives OS_Delay. */
static unsigned long wdt_last_kick = 0;


/******************************************************************************
 task_wdt()

 Parks WDI high, then pulses it low every 46 s.
******************************************************************************/
void task_wdt(void) {

  /* RF4 (IO.7) as output, WDI high. */
  _TRISF4 = 0;
  csk_io7_high();

  wdt_last_kick = OSGetTicks();

  csk_uart1_puts(STR_TASK_WDT "TPS3430 service started (WDI = IO.7, ~46s).\r\n");

  while (1) {

    /* 1 s heartbeat. */
    OS_Delay(100);

    if ((OSGetTicks() - wdt_last_kick) >= WDT_KICK_TICKS) {

      /* ~10 ms low pulse (min 50 ns), busy-wait so we don't yield here. */
      csk_io7_low();          /* falling edge kicks the TPS3430 */
      { volatile unsigned int d;
        for (d = 0; d < 5000u; d++) { __builtin_nop(); } }
      csk_io7_high();

      /* Timestamp after the pulse so the interval is never short. */
      wdt_last_kick = OSGetTicks();
    }

  } /* while */

} /* task_wdt() */
