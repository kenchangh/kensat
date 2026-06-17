/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_sd.c,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2010-01-24 20:36:56-08 $

******************************************************************************/
#include "csk_sd.h"
#include "csk_hw.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_sd_open()

Open the SD Card for use. Just makes -CS_SD an output, leaves it inactive.

Note: Requires that SD Card be powered prior to use.


**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_sd_open(void) {

  // Configure -CS_SD as an output, and make
  //  it inactive (i.e., HIGH)
  PORTE |=  CS_SD_BAR;
  TRISE &= ~CS_SD_BAR;

} /* csk_sd_open() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_sd_close()

Close the SD Card.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_sd_close(void) {

  // Restore -CS_SD to an input. It's therefore pulled HIGH by its pull-up 
  //  resistor.
  PORTE |=  CS_SD_BAR;
  TRISE |=  CS_SD_BAR;

} /* csk_sd_close() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_sd_pwr_on()

Power on the SD Card.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_sd_pwr_on(void) {

  // Configure -ON_SD as an output, and make
  //  it active (i.e., LOW)
  PORTE &= ~ON_SD_BAR;
  TRISE &= ~ON_SD_BAR;
  
} /* csk_sd_pwr_on() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_sd_pwr_off()

Power off the SD Card.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_sd_pwr_off(void) {

  // Configure -ON_SD as an output, and make
  //  it inactive (i.e., HIGH)
  PORTE |=  ON_SD_BAR;
  TRISE &= ~ON_SD_BAR;
  
} /* csk_sd_pwr_off() */
