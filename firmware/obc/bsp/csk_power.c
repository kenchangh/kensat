/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_power.c,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2011-09-17 22:19:13-07 $

******************************************************************************/
#include "init.h"

// Pumpkin CubeSat Kit headers
#include "csk_hw.h"
#include "csk_led.h"
#include "csk_mhx.h"
#include "csk_power.h"
#include "csk_uart.h"
#include "csk_usb.h"

// Pumpkin PIC24 Library.
#include "pic24_uart.h"

// Microchip PIC24F Peripehral Library
#include <pps.h>
#include <PwrMgnt.h>
#include <timer.h>
#include <uart.h>
#include <wdt.h>


/******************************************************************************
****                                                                       ****
**                                                                           **
CHANGELOG:

  aek     2008-12-20
  * REV D must take P1.7 high 'cause R13 is pulled high to
   VCC AND U1 & U16 are dual-powered from VCC and VCC_IO.
  * I thought that REV D would also want -CS_SD/ON_I2C held 
  low in cases where a PCA9515A is functioning as the I2C 
  isolator (e.g. for MSP430F161x on FPMA), thereby making
  ON_I2C (and therefore the PCA9515A) inactive. But it turns
  out that the PCA9515A draws around 500uA regardless of the
  state of its EN input (= ON_I2C), so it's not necessary.

**                                                                           **
****                                                                       ****
******************************************************************************/

/******************************************************************************
****                                                                       ****
**                                                                           **
csk_power_shutdown()

Shut down entire system.

Note that PPM Ds only work on Rev D and later, so no compatibility back to
Revs A-C is required.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_power_shutdown(void) {

  // Now really shut things down now that multitasking has stopped!
  __disable_interrupt();

  // High-level funcs
  csk_mhx_close();
  csk_mhx_pwr_off();
  csk_usb_close();
  csk_led_status_close();

  // Disable all special funcs
  // Set all I/O to LOW outputs save for:      
  //  IO.0 /RE5 -- HIGH  (pulled up RevB onwards) 
  //  IO.5 /RF2 -- input (from MAX3221 on Dev Board)  
  //        RE4 -- HIGH  (disable -ON_SD)      
  //        RE3 -- HIGH  (disable -ON_MHX)        
  //        RE2 -- HIGH  (disable -OE_MHX)                         
  //        RC1 -- HIGH  (disable -OE_USB)                  
  //  IO.7, HS[5..0] set LOW because U1 & U2 are high-Z (U1 may be unpowered).         
  #if   defined(__PIC24FJ256GA110__)
 
  TRISA = ~(BIT15+BIT14+BIT13+BIT12+                                          BIT3+BIT2          );
  TRISB = ~(BIT15+BIT14+BIT13+BIT12+BIT11+BIT10+BIT9+BIT8+BIT7+BIT6+BIT5+BIT4+BIT3+BIT2+BIT1+BIT0);
  TRISC = ~(                                                             BIT4+BIT3+BIT2+BIT1     );
  TRISD = ~(BIT15+BIT14+      BIT12+            BIT9+BIT8+          BIT5+BIT4+BIT3+BIT2+BIT1+BIT0);
  TRISE = ~(                                    BIT9+BIT8+BIT7+BIT6+BIT5+BIT4+BIT3+BIT2          );
  TRISF = ~(            BIT13+BIT12+                 BIT8+BIT7+BIT6+BIT5+BIT4+BIT3+     BIT1+BIT0); // RF2 is an input
  TRISG = ~(                                    BIT9+BIT8+BIT7+BIT6+          BIT3+BIT2+BIT1+BIT0);

  PORTA = 0x0000;
  PORTB = 0x0000;
  PORTC = 0x0000+BIT1;                      // -OE_USB disabled
  PORTD = 0x0000;
  PORTE = 0x0000+BIT5+BIT4+BIT3+BIT2; 		  // -CS_SD, -ON_SD, -ON_MHX, -OE_MHX disabled
  PORTF = 0x0000;
  PORTG = 0x0000;
  
  AD1PCFGL = 0xFFFF;

  #elif defined(__PIC24FJ256GB210__)      

  TRISA = ~(            BIT13+BIT12+                      BIT7+BIT6+          BIT3+BIT2          );
  TRISB = ~(BIT15+BIT14+BIT13+BIT12+BIT11+BIT10+BIT9+BIT8+BIT7+BIT6+BIT5+BIT4+BIT3+BIT2+BIT1+BIT0);
  TRISC = ~(                                                             BIT4+BIT3+BIT2+BIT1     );
  TRISD = ~(BIT15+BIT14+      BIT12+            BIT9+BIT8+BIT7+     BIT5+BIT4+BIT3+BIT2+BIT1+BIT0);
  TRISE = ~(                                    BIT9+BIT8+BIT7+BIT6+BIT5+BIT4+BIT3+BIT2          );
  TRISF = ~(            BIT13+BIT12+                 BIT8+BIT7+BIT6+BIT5+               BIT1+BIT0); // leave RF2 & RF4 alone
  TRISG = ~(BIT15+                              BIT9+BIT8+BIT7+BIT6+          BIT3+BIT2+BIT1+BIT0);

  PORTA = 0x0000;
  PORTB = 0x0000;
  PORTC = 0x0000+BIT1;           // -OE_USB is OFF
  PORTD = 0x0000;               
  PORTE = 0x0000+BIT5+BIT4+BIT3+BIT2; // -ON_SD, -ON_MHX, -OE_MHX are OFF
  PORTF = 0x0000;
  PORTG = 0x0000;
  
  ANSA = 0x0000;
  ANSB = 0x0000;
  ANSC = 0x0000;
  ANSD = 0x0000;
  ANSE = 0x0000;
  ANSF = 0x0000;
  ANSG = 0x0000;
  
  #else
  #error PIC24F device not supported by CubeSat Kit.
  #endif 


  // Clock changes

  // Stop watchdog (conserves power) 
  EnableWDT(WDT_DISABLE);  

  // USARTs off 
  csk_uart0_close();
  csk_uart1_close();

  // Undo PPS for UART Tx pins
  #if   defined(__PIC24FJ256GA110__)
  iPPSOutput(OUT_PIN_PPS_RP16,OUT_FN_PPS_NULL);
  #elif defined(__PIC24FJ256GB210__)
  iPPSOutput(OUT_PIN_PPS_RP4,OUT_FN_PPS_NULL);
  #else
  #error PIC24F device not supported by CubeSat Kit
  #endif  
  iPPSOutput(OUT_PIN_PPS_RP17,OUT_FN_PPS_NULL);
  
  // Timer2 off
  ConfigIntTimer2(T2_INT_OFF);
  CloseTimer2();

  // Disable all interrupt sources (?) 
  __disable_interrupt();

  // Go to sleep forever 
  while (1) {
    mPWRMGNT_GotoSleepMode();;
  } 
  
} /* csk_power_shutdown() */
