/******************************************************************************
KENSAT - External Watchdog (TPS3430) Service Task

Services the TPS3430 window watchdog (U14) by producing a falling edge on its
WDI input every ~54 s. WDI is wired to CSK IO.7 (header H1.17) = PIC24 RF4.

The TPS3430 resets the system (WDO -> -RESET) unless a WDI falling edge lands
inside its valid window. With this board's components (CWD = C59 = 0.82 uF,
SET0 = 0, SET1 = 1 -> 3/4 window ratio) the window is narrow, so the service
interval is chosen to sit at the safe centre. See task_wdt.c for the timing
derivation.
******************************************************************************/
#ifndef __task_wdt_h
#define __task_wdt_h

extern void task_wdt(void);

#define STR_TASK_WDT "task_wdt: " STR_1TAB

#endif /* __task_wdt_h */
