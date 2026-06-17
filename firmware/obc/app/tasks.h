/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\tasks.h,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2010-01-29 20:04:19-08 $

******************************************************************************/
#define TASK_CMD_DO_P                           OSTCBP(1) // needs to be cmd #1 (see task_cmd_do())
//#define TASK_STATUS_MON_P                       OSTCBP(2)
//#define TASK_ONCHIP_TEMP_P                      OSTCBP(3)
//#define TASK_EXERCISE_IO_P                      OSTCBP(4)  // disabled — slot reused by task_beacon
//#define TASK_USB_DETECT_P                       OSTCBP(5)
//#define TASK_USB_TALK_P                         OSTCBP(2) // Disabled
//#define TASK_MHX_TALK_P                         OSTCBP(7)
//#define TASK_5SEC_P                             OSTCBP(8)
//#define TASK_SI4463_RX_P                        OSTCBP(3) // Disabled: manual cmd control
#define TASK_JETSON_P                           OSTCBP(2)
#define TASK_SDLOG_P                            OSTCBP(3)
#define TASK_BEACON_P                           OSTCBP(4)
#define TASK_BMI088_P                           OSTCBP(5)
#define TASK_WDT_P                              OSTCBP(6)
