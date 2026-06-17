/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_usb.c,v $
$Author: aek $
$Revision: 3.3 $
$Date: 2011-09-17 16:24:45-07 $

******************************************************************************/
#include "csk_usb.h"
#include "csk_hw.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_usb_open()

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_usb_open(void) {

  // Disable all control signals to avoid spurious
  //  writes when -OE_USB goes active (LOW).          
  #if   defined(__PIC24FJ256GA110__)    
  PORTD |=  (HS5+HS4+HS3            );
  TRISD &= ~(HS5+HS4+HS3            );
  #elif defined(__PIC24FJ256GB210__)  
  PORTD |=  (HS5+    HS3            );
  PORTG |=  (    HS4                );  
  TRISD &= ~(HS5+    HS3            );  
  TRISG &= ~(    HS4                );   
  #else
  #error PIC24F device not supported by CubeSat Kit.
  #endif   
  TRISC |=  (            HS2+HS1+HS0);  

  // Configure -OE_USB as an output, and make
  //  it active (i.e., LOW)
  PORTC &= ~OE_USB_BAR;
  TRISC &= ~OE_USB_BAR;
  
} /* csk_usb_open() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_usb_close()

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_usb_close(void) {

  // Disable all control signals to avoid spurious
  //  writes when -OE_USB goes active (LOW).          
  #if   defined(__PIC24FJ256GA110__)    
  PORTD |=  (HS5+HS4+HS3            );
  TRISD &= ~(HS5+HS4+HS3            );
  #elif defined(__PIC24FJ256GB210__)  
  PORTD |=  (HS5+    HS3            );
  PORTG |=  (    HS4                );  
  TRISD &= ~(HS5+    HS3            );  
  TRISG &= ~(    HS4                );   
  #else
  #error PIC24F device not supported by CubeSat Kit.
  #endif   
  TRISC |=  (            HS2+HS1+HS0);  

  // Restore -OE_USB to an input.
  PORTC |=  OE_USB_BAR;
  TRISC |=  OE_USB_BAR;
  
} /* csk_usb_close() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_usb_present()

**                                                                           **
****                                                                       ****
******************************************************************************/
unsigned char csk_usb_present(void) {

  // In CSK Revs A-C, it was possible to detect USB
  //  presence by changing -OE_USB to an output,
  //  forcing it LOW, and then seeing if it went 
  //  HIGH again after a few tc's due to being pulled
  //  up by VCC_IO through R13. In CSK Rev D, R13
  //  is now pulled up through VCC, there is no way
  //  to sense VCC_IO or +5V_USB, and XXX_USB are not
  //  pulled down :(. Therefore there is no way
  //  to sense USB presence in CSK Rev D.
  // Adding a pull-down resistor to TXD_USB would
  //  permit the auto-detection if USB by looking
  //  for USB Tx output idling high -- that would
  //  indicate that USB is present.

  return (1);

} /* csk_usb_present() */
