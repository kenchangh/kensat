/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_led.c,v $
$Author: aek $
$Revision: 3.2 $
$Date: 2010-01-24 20:36:54-08 $

******************************************************************************/
#include "csk_led.h"
#include "csk_hw.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_open()

Open the status LED for use.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_open(void) {

  // Configure PORTB.5 as an output.
  PORTB &= ~BIT5;
  TRISB &= ~BIT5;
  
} /* csk_led_status_open() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_close()

Close the status LED.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_close(void) {

  // Configure PORTB.5 as an input.
  PORTB &= ~BIT5;
  TRISB |=  BIT5;
  
} /* csk_led_status_close() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_on()

Turn the status LED ON.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_on(void) {

  // Take status LED on PORTB.5 HIGH.        
  PORTB |=  BIT5;
  
} /* csk_led_status_on() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_off()

Turn the status LED OFF.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_off(void) {

  // Take status LED on PORTB.5 LOW.            
  PORTB &= ~BIT5;
  
} /* csk_led_status_off() */
