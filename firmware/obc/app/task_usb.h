/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_usb.h,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2010-01-29 20:04:21-08 $

******************************************************************************/
#ifndef __task_usb_h
#define __task_usb_h


// Function declarations
extern void task_usb_detect(void);
extern void task_usb_talk(void);

// Symbols, etc.
#define STR_TASK_USB_TALK   "task_usb_talk: " STR_1TAB
#define STR_TASK_USB_DETECT "task_usb_detect: " STR_1TAB


#endif /* __task_usb_h */


