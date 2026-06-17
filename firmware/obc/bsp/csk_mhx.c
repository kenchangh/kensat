/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_mhx.c,v $
$Author: aek $
$Revision: 3.3 $
$Date: 2011-09-17 16:24:46-07 $

******************************************************************************/
#include "csk_mhx.h"
#include "csk_hw.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_mhx_open()

Open the MHX interface for use.

Note: Does not explicitly close the USB interface.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_mhx_open(void) {

  // Disable all control signals to avoid spurious
  //  writes when -OE_MHX goes active (LOW). 
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
  
  // Configure -OE_MHX as an output, and make
  //  it active (i.e., LOW)
  PORTE &= ~OE_MHX_BAR;
  TRISE &= ~OE_MHX_BAR;

} /* csk_mhx_open() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_mhx_close()

Close the MHX interface.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_mhx_close(void) {

  // Disable all control signals to avoid spurious
  //  writes when -OE_MHX goes active (LOW). 
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

  // restore -OE_MHX to an input.
  PORTE |=  OE_MHX_BAR;
  TRISE |=  OE_MHX_BAR;

} /* csk_mhx_close() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_mhx_pwr_on()

Power on the MHX transceiver.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_mhx_pwr_on(void) {

  // Configure -ON_MHX as an output, and make
  //  it active (i.e., LOW)
  PORTE &= ~ON_MHX_BAR;
  TRISE &= ~ON_MHX_BAR;
  
} /* csk_mhx_pwr_on() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_mhx_pwr_off()

Power off the MHX transceiver.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_mhx_pwr_off(void) {

  // Configure -ON_MHX as an output, and make
  //  it inactive (i.e., HIGH)
  PORTE |=  ON_MHX_BAR;
  TRISE &= ~ON_MHX_BAR;
  
} /* csk_mhx_pwr_off() */
