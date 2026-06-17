/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_status.c,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2009-11-02 00:45:06-08 $

******************************************************************************/
#include "main.h"
#include "task_status.h"

// Pumpkin CubeSat Kit headers
#include "csk_led.h"
#include "csk_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
task_status_mon()

Gives a visual indication on P1OUT.1 of the system status, via pulses.

                ...90123456789012345678901234...
At power-up:    ...__|_________|_________|___...
USB present:    ...___|_________|_________|__...
MHX connected:  ...____|_________|_________|_...

The flashes (5 system ticks long) are "OR'd" together.

This task demonstates:

  1) Using a state machine inside a task. A single state is executed each time 
     the task runs. The time between states is set by the current value of 
     <delay>.

  2) Initializing a variable (<state>) that is local to a task in the task's 
      preamble.

  3) Using static variables to ensure that their values are preserved across 
     context switches.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_status_mon(void) {

  static char state;
  static OStypeDelay delay;


  user_debug_msg(STR_TASK_STATUS_MON "Starting.");
  state = 0;  
  csk_led_status_open();

  while (1) {
  
    switch (state) {
      // Single flash -- turn it on for 5 ticks.
      case 0:
        csk_led_status_on();
        delay = 5;
        break;


      // ... and turn it off for 5 ticks 
      //  (<delay> is still 5).  
      case 1:
        csk_led_status_off();
        break;


      // Turn it on (again) for 5 ticks if
      //  task_exercise_io() is running.
      case 2:
        if (csk_status.exercise_io_running) {
          csk_led_status_on();
        }
        break;


      // Always off.
      case 3:
        csk_led_status_off();
        break;

      // Turn it on (again) for 5 ticks if we're
      //  talking to the MHX.
      case 4:
        if (csk_status.mhx_connected) {
          csk_led_status_on();
        }
        break;


      // Always off.  
      case 5:
        csk_led_status_off();
        break;

      // Remaining time to fill 100 ticks ...  
      default:
        delay = 80;
        break;
        
    } /* switch */

    // Increment state (sequence #).
    state++;

    // Roll over after last state.
    if (state > 6) {
      state = 0;
    }

    OS_Delay(delay);
    
  } /* while */
  
}  /* task_status_mon() */
