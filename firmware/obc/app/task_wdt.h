/******************************************************************************
KENSAT - External Watchdog (TPS3430) Service Task

Kicks the TPS3430 window watchdog (U14) with a WDI falling edge every 46 s.
WDI = CSK IO.7 (H1.17) = PIC24 RF4.  Timing derivation in task_wdt.c.
******************************************************************************/
#ifndef __task_wdt_h
#define __task_wdt_h

extern void task_wdt(void);

#define STR_TASK_WDT "task_wdt: " STR_1TAB

#endif /* __task_wdt_h */
