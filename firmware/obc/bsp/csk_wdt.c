/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_wdt.c,v $
$Author: aek $
$Revision: 3.4 $
$Date: 2010-03-30 22:17:58-08 $

******************************************************************************/
#include "csk_wdt.h"
#include "csk_hw.h"

// Microchip MPLAB-C30 Library.
#include "wdt.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_wdt_force()

Force a reset via the WDT system.

(Actually forces a reset simply via a reset).

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_wdt_force(void) {

  __asm__ volatile("reset");
  
} /* csk_wdt_force() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_wdt_off()

Disable the WDT.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_wdt_off(void) {

  EnableWDT(WDT_DISABLE);
  
} /* csk_wdt_off() */



