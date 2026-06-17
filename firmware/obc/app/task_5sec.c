/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_5sec.c,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2009-11-02 00:45:07-08 $

******************************************************************************/
#include "main.h"
#include "task_5sec.h"

// Pumpkin CubeSat Kit headers
#include "csk_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"




/******************************************************************************
****                                                                       ****
**                                                                           **
task_5sec()

Simple task that runs every 5s.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_5sec(void) {

  user_debug_msg(STR_TASK_5SEC "Starting.");

  while (1) {

    // Wait 5sec.
    OS_Delay(250);
    OS_Delay(250);

    // Send message.
    user_debug_msg(STR_TASK_5SEC "Ran again.");

  } /* while */

} /* task_5sec() */


