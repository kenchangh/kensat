/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_rand.c,v $
$Author: aek $
$Revision: 3.0 $
$Date: 2010-01-24 10:25:35-08 $

******************************************************************************/
#include "csk_rand.h"
#include "csk_hw.h"


unsigned int seed = 123;

/******************************************************************************
****                                                                       ****
**                                                                           **
csk_rand()

Simple shift-register-based pseudo-random number generator.

**                                                                           **
****                                                                       ****
******************************************************************************/
unsigned int csk_rand(void) {

  if ( seed & 1 ) {
    seed >>= 1;
    seed ^= 0x8805;
  } else {
    seed >>= 1;
  }
  return seed;
  
} /* csk_rand() */
