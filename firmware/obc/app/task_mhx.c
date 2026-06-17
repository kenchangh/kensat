/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_mhx.c,v $
$Author: aek $
$Revision: 3.2 $
$Date: 2010-01-29 20:04:20-08 $

******************************************************************************/
#include "main.h"
#include "events.h"
#include "task_onchip_temp.h"
#include "task_mhx.h"

// Pumpkin CubeSat Kit headers
#include "csk_mhx.h"
#include "csk_rand.h"
#include "csk_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"




/******************************************************************************
****                                                                       ****
**                                                                           **
task_mhx_talk()

Task to talk to the on-board MHX2400 transceiver/radio or equivalent.

After a 2.5s delay, every 5s or so the task will attempt to acquire the USB/MHX 
interface (I/F). When it succeeds, it turns MHX_PWR ON for Ns, then turns it 
off again and releases the USB/MHX I/F.

If a MHX is plugged in, it will cycle its RSSI[3..1] during the 5s that MHX_PWR 
is ON.

This task demonstates:

  1) Activity before the "infinite loop". In this case, there is a one-time
     wait of 2.5s before the task runs indefinitely.

  2) Acquiring the rights to use a shared resource (the USB/MHX I/F) via a 
     binary semaphore (binSem), and releasing that resource when done. The task 
     "holds" the resource during the Ns that MHX_PWR is on, and therefore no 
     other task can meddle with the USB/MHX I/F during that time ...

  3) Delays in excess of 255 ticks when using a standard library (built with 
     8-bit delays).

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_mhx_talk(void) {

  user_debug_msg(STR_TASK_MHX_TALK "Starting.");

  // Begin after 2.5 seconds.
  OS_Delay(250);

  while (1)  {
  
    // Wait 5s + some random time. Avoid 
    //  OS_Delay(0) because that will stop this task.
    OS_Delay(250); 
    OS_Delay(250);
    OS_Delay(1 + ((csk_rand()>>9)& 0x007F));

    // Proceed if USB/MHX I/F is not in use.
    OS_WaitBinSem(RSRC_USB_MHX_IF_P, OSNO_TIMEOUT);
    user_debug_msg(STR_TASK_MHX_TALK "Acquired USB/MHX I/F for MHX.");
    user_debug_msg(STR_TASK_MHX_TALK "Enabling MHX I/F.");

    // Access MHX.
    csk_mhx_open();
    csk_mhx_pwr_on();
    csk_status.mhx_connected = 1;

    // Send a simple message to debug port.
    user_debug_msg(STR_TASK_MHX_TALK "MHX_PWR is ON.");

    // Wait 1s for modem to power-up.
    user_debug_msg(STR_TASK_MHX_TALK "Waiting for modem to power-up.");
    OS_Delay(100);

    // Place modem in command mode and wait for response (should be "OK").
    user_debug_msg(STR_TASK_MHX_TALK "\"mhx\": place modem in command mode.");
    csk_uart1_puts("mhx");
    OS_Delay(100);

    // Configure for Factory Setting #1 (Master mode) and wait for response (should be "OK").
    user_debug_msg(STR_TASK_MHX_TALK "\"AT&F1&K0S108=0\": factory #1, no h/s, 1mW.");
    csk_uart1_puts("AT&F1&K0S108=0");
    OS_Delay(50);

    // Enter data mode.
    user_debug_msg(STR_TASK_MHX_TALK "\"ATA\": enter data mode.");
    csk_uart1_puts("ATA");
    OS_Delay(150);

    // Send some data in the form of human-readable strings.
    user_debug_msg(STR_TASK_MHX_TALK "Sending some data.");
    csk_uart1_puts(STR_CRLF);
    sprintf(strTmp, "CubeSat Kit phoning home ..." );
    data_debug_msg(strTmp);
    OS_Delay(100);
    sprintf(strTmp, "Uncalibrated on-chip temperature is %dC." , onchip_temp_deg_c());
    data_debug_msg(strTmp);
    OS_Delay(100);

    // Leave data mode after 1s of "silence".
    user_debug_msg(STR_TASK_MHX_TALK "\"+++\": leave data mode.");
    OS_Delay(100);
    csk_uart1_puts("+++");

    // Wait for xmission to finish before powering down ...
    OS_Delay(100);
    
    // We're done here.
    user_debug_msg(STR_TASK_MHX_TALK "Disabling MHX I/F.");
    csk_mhx_pwr_off();
    csk_mhx_close();
    user_debug_msg(STR_TASK_MHX_TALK "MHX_PWR is OFF.");
    csk_status.mhx_connected = 0;

    // Release USB/MHX I/F.
    OSSignalBinSem(RSRC_USB_MHX_IF_P);
    user_debug_msg(STR_TASK_MHX_TALK "Released USB/MHX I/F.");

  } /* while */

} /* task_mhx_talk() */


