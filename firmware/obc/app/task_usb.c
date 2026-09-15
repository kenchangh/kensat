/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_usb.c,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2009-11-02 00:45:05-08 $

******************************************************************************/
#include "main.h"
#include "events.h"
#include "task_onchip_temp.h"
#include "task_usb.h"

// Pumpkin CubeSat Kit headers
#include "csk_mhx.h"
#include "csk_rand.h"
#include "csk_uart.h"
#include "csk_usb.h"

// Pumpkin Salvo headers
#include "salvo.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
task_usb_talk()

Talk to the USB serial interface.

Immediately on power-up, attempt to acquire the USB/MHX interface (I/F). Once 
the I/F is acquired, configure the control pins and send a message out the 
I/F. Release the I/F when done, and repeat indefinitely.

This task demonstates:

  1) Acquiring the rights to use a shared resource (the USB/MHX I/F) via 
     a binary semaphore (binSem), and releasing that resource when done. The 
     task "holds" the resource during the N system ticks while USART1 is 
     transmitting the serial bitstream, and therefore no other task can meddle 
     with the USB/MHX I/F during that time ...

  2) Using a local (auto) variable (<strTmp[]>) for appropriate local storage.

  3) Intertask communications using the binary semaphore (binSem) 
     RSRC_USB_MHX_IF_P.  

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_usb_talk(void ){

  user_debug_msg(STR_TASK_USB_TALK "Starting.");

  // For now, until USB can be detected.
  csk_status.usb_present = 1; 

  // Wait a moment so that task_onchip_temp can start conversions.
  OS_Delay(10);

  while (1) {
  
    // Is USB connected?
    if (csk_status.usb_present) {
    
      // Proceed if USB/MHX I/F is not in use.
      OS_WaitBinSem(RSRC_USB_MHX_IF_P, OSNO_TIMEOUT);
      
      // Access USB.
      csk_usb_open();
      csk_status.usb_connected = 1;

      // Send a simple message out the USB port. 
      user_debug_msg(STR_TASK_USB_TALK "Sending to USB.");
      sprintf(strTmp, STR_TASK_USB_TALK "bla bla %dC.", onchip_temp_deg_c());
      data_debug_msg(strTmp);

      // Don't close the USB connection until the
      //  message is completely gone (derived
      //  empirically  ... N ticks is adequate for 9600 baud).
      OS_Delay(25);

      // Done w/USB.
      csk_usb_close();
      csk_status.usb_connected = 0;

      // Release USB/MHX I/F.
      OSSignalBinSem(RSRC_USB_MHX_IF_P);
      
    } /* if */

    // Repeat every 5 seconds. Split across two delays because Salvo Pro-lib
    // delays are 8-bit (OSBYTES_OF_DELAYS=1) and OS_Delay(475) would truncate
    // mod 256 to 219 ticks. 237 + 238 = 475 ticks (~5 s @ 100 Hz tick).
    OS_Delay(237);
    OS_Delay(238);

  } /* while */

} /* task_usb_talk() */



/******************************************************************************
****                                                                       ****
**                                                                           **
task_usb_detect()

Check USB I/F every 25 ticks to detect whether a live USB cable is connected
... signals the resource-controlling binSem RSRC_USB_MHX_IF_P and reports 
on change ...

This task demonstates:

  1) Inter-task communication by signaling a binSem.


**                                                                           **
****                                                                       ****
******************************************************************************/
#if 0 // Not used as of Rev D
void task_usb_detect(void) {

  user_debug_msg(STR_TASK_USB_DETECT "Starting.");
        
  while (1) {
  
    // Proceed if USB/MHX I/F is not in use.
    OS_WaitBinSem(RSRC_USB_MHX_IF_P, OSNO_TIMEOUT);
    
    // Access USB.
    csk_usb_open();

    // If USB is present and status hasn't changed ...
    if (!csk_status.usb_connected && (csk_usb_present())) {
      csk_status.USBpresent = 1;
      user_debug_msg(STR_TASK_USB_DETECT "USB connected.");
    }
    else if (csk_status.usb_connected && !(csk_usb_present())) {
      csk_status.USBpresent = 0;
      user_debug_msg(STR_TASK_USB_DETECT "USB disconnected.");
    }

    // Done w/USB.
    csk_usb_close();

    // Release USB/MHX I/F.
    OSSignalBinSem(RSRC_USB_MHX_IF_P);

    // Come back in 25 ticks.
    OS_Delay(25);
    
  } /* while */

} /* task_usb_detect() */
#endif 
