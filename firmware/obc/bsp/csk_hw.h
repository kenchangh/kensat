/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Inc\\csk_hw.h,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2011-09-17 16:24:47-07 $

******************************************************************************/
#ifndef __csk_hw_h
#define __csk_hw_h


// PIC24-specific symbols
#include <p24fxxxx.h>

#define BIT0  (1 <<  0)
#define BIT1  (1 <<  1)
#define BIT2  (1 <<  2)
#define BIT3  (1 <<  3)
#define BIT4  (1 <<  4)
#define BIT5  (1 <<  5)
#define BIT6  (1 <<  6)
#define BIT7  (1 <<  7)
#define BIT8  (1 <<  8)
#define BIT9  (1 <<  9)
#define BIT10 (1 << 10)
#define BIT11 (1 << 11)
#define BIT12 (1 << 12)
#define BIT13 (1 << 13)
#define BIT14 (1 << 14)
#define BIT15 (1 << 15)

// SD Card signals. CSK's SPI0 is on PIC24's SPI1
#if   defined(__PIC24FJ256GA110__)
#define SDO0        BIT8      // RF8(RP15)
#define SDI0        BIT7      // RF7(RPI44)
#define SCK0        BIT9      // RD9(RP4)
#elif defined(__PIC24FJ256GB210__)
#define SDO0        BIT8      // RF8(RP15)
#define SDI0        BIT1      // RD1(RP24)
#define SCK0        BIT2      // RD2(RP23)
#else
#error PIC24F device not supported by CubeSat Kit.
#endif

// Handshake control signals
#define HS0         BIT4      // RC4(RPI41)
#define HS1         BIT3      // RC3(RPI40)
#define HS2         BIT2      // RC2(RPI39)
#define HS3         BIT5      // RD5
#if   defined(__PIC24FJ256GA110__)
#define HS4         BIT2      // RD2  
#define HS5         BIT1      // RD1
#elif defined(__PIC24FJ256GB210__)
#define HS4         BIT15     // RG15  
#define HS5         BIT7      // RD7
#else
#error PIC24F device not supported by CubeSat Kit.
#endif 

// Motherboard control signals
#define CS_SD_BAR   BIT5      // RE5
#define OE_USB_BAR  BIT1      // RC1
#define OE_MHX_BAR  BIT2      // RE2
#define ON_SD_BAR   BIT4      // RE4
#define ON_MHX_BAR  BIT3      // RE3

// UART signals
#if   defined(__PIC24FJ256GA110__)
#define TX0         BIT3      // RF3(RP16) <-> IO.4
#elif defined(__PIC24FJ256GB210__)
#define TX0         BIT9      // RD9(RP4)  <-> IO.4
#else
#error PIC24F device not supported by CubeSat Kit.
#endif 
#define RX0         BIT2      // RF2(RP30) <-> IO.5
#define TX1         BIT5      // RF5(RP17) <-> IO.6
#define RX1         BIT4      // RF4(RP10) <-> IO.7


#endif /* __csk_hw_h */
