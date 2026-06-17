/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\csk_io.c,v $
$Author: aek $
$Revision: 3.3 $
$Date: 2011-09-17 16:24:46-07 $

******************************************************************************/
#include "csk_io.h"
#include "csk_hw.h"


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_ioN_high()
csk_ioN_low()

Functions to force signals IO.[47..0] HIGH (VCC) or LOW (GND). Assumes that 
corresponding port direction register is set to be an output.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_io0_high(void)    { PORTE |=  BIT5;  }
void csk_io0_low(void)     { PORTE &= ~BIT5;  }
void csk_io1_high(void)    { PORTF |=  BIT8;  }
void csk_io1_low(void)     { PORTF &= ~BIT8;  }

#if   defined(__PIC24FJ256GA110__)
void csk_io2_high(void)    { PORTF |=  BIT7;  }
void csk_io2_low(void)     { PORTF &= ~BIT7;  }
void csk_io3_high(void)    { PORTD |=  BIT9;  }
void csk_io3_low(void)     { PORTD &= ~BIT9;  }
void csk_io4_high(void)    { PORTF |=  BIT3;  }
void csk_io4_low(void)     { PORTF &= ~BIT3;  }
#elif defined(__PIC24FJ256GB210__)
void csk_io2_high(void)    { PORTD |=  BIT1;  }
void csk_io2_low(void)     { PORTD &= ~BIT1;  }
void csk_io3_high(void)    { PORTD |=  BIT2;  }
void csk_io3_low(void)     { PORTD &= ~BIT2;  }
void csk_io4_high(void)    { PORTD |=  BIT9;  }
void csk_io4_low(void)     { PORTD &= ~BIT9;  }
#else
#error PIC24F device not supported by CubeSat Kit.
#endif 

void csk_io5_high(void)    { PORTF |=  BIT2;  }
void csk_io5_low(void)     { PORTF &= ~BIT2;  }
void csk_io6_high(void)    { PORTF |=  BIT5;  }
void csk_io6_low(void)     { PORTF &= ~BIT5;  }
void csk_io7_high(void)    { PORTF |=  BIT4;  }
void csk_io7_low(void)     { PORTF &= ~BIT4;  }

void csk_io8_high(void)    { PORTG |=  BIT9;  }
void csk_io8_low(void)     { PORTG &= ~BIT9;  }
void csk_io9_high(void)    { PORTG |=  BIT8;  }
void csk_io9_low(void)     { PORTG &= ~BIT8;  }
void csk_io10_high(void)   { PORTG |=  BIT7;  }
void csk_io10_low (void)   { PORTG &= ~BIT7;  }
void csk_io11_high(void)   { PORTG |=  BIT6;  }
void csk_io11_low (void)   { PORTG &= ~BIT6;  }
void csk_io12_high(void)   { PORTD |=  BIT15; }
void csk_io12_low (void)   { PORTD &= ~BIT15; }
void csk_io13_high(void)   { PORTD |=  BIT14; }
void csk_io13_low (void)   { PORTD &= ~BIT14; }
void csk_io14_high(void)   { PORTF |=  BIT13; }
void csk_io14_low (void)   { PORTF &= ~BIT13; }
void csk_io15_high(void)   { PORTF |=  BIT12; }
void csk_io15_low (void)   { PORTF &= ~BIT12; }

void csk_io16_high(void)   { PORTD |=  BIT8;  }
void csk_io16_low (void)   { PORTD &= ~BIT8;  }
void csk_io17_high(void)   { PORTD |=  BIT3;  }
void csk_io17_low (void)   { PORTD &= ~BIT3;  }
void csk_io18_high(void)   { PORTD |=  BIT12; }
void csk_io18_low (void)   { PORTD &= ~BIT12; }
void csk_io19_high(void)   { PORTD |=  BIT4;  }
void csk_io19_low (void)   { PORTD &= ~BIT4;  }
void csk_io20_high(void)   { PORTA |=  BIT2;  }
void csk_io20_low (void)   { PORTA &= ~BIT2;  }
void csk_io21_high(void)   { PORTA |=  BIT3;  }
void csk_io21_low (void)   { PORTA &= ~BIT3;  }
void csk_io22_high(void)   { PORTE |=  BIT6;  }
void csk_io22_low (void)   { PORTE &= ~BIT6;  }
void csk_io23_high(void)   { PORTE |=  BIT7;  }
void csk_io23_low (void)   { PORTE &= ~BIT7;  }

void csk_io24_high(void)   { PORTF |=  BIT1;  }
void csk_io24_low (void)   { PORTF &= ~BIT1;  }
void csk_io25_high(void)   { PORTF |=  BIT0;  }
void csk_io25_low (void)   { PORTF &= ~BIT0;  }
void csk_io26_high(void)   { PORTG |=  BIT1;  }
void csk_io26_low (void)   { PORTG &= ~BIT1;  }
void csk_io27_high(void)   { PORTG |=  BIT0;  }
void csk_io27_low (void)   { PORTG &= ~BIT0;  }

#if   defined(__PIC24FJ256GA110__)
void csk_io28_high(void)   { PORTA |=  BIT14; }
void csk_io28_low (void)   { PORTA &= ~BIT14; }
void csk_io29_high(void)   { PORTA |=  BIT15; }
void csk_io29_low (void)   { PORTA &= ~BIT15; }
#elif defined(__PIC24FJ256GB210__)
void csk_io28_high(void)   { PORTA |=  BIT6;  }
void csk_io28_low (void)   { PORTA &= ~BIT6;  }
void csk_io29_high(void)   { PORTA |=  BIT7;  }
void csk_io29_low (void)   { PORTA &= ~BIT7;  }
#else
#error PIC24F device not supported by CubeSat Kit.
#endif 

void csk_io30_high(void)   { PORTE |=  BIT8;  }
void csk_io30_low (void)   { PORTE &= ~BIT8;  }
void csk_io31_high(void)   { PORTE |=  BIT9;  }
void csk_io31_low (void)   { PORTE &= ~BIT9;  }

void csk_io32_high(void)   { PORTB |=  BIT5;  }
void csk_io32_low (void)   { PORTB |=  BIT5;  }
void csk_io33_high(void)   { PORTB |=  BIT4;  }
void csk_io33_low (void)   { PORTB &= ~BIT4;  }
void csk_io34_high(void)   { PORTB |=  BIT3;  }
void csk_io34_low (void)   { PORTB &= ~BIT3;  }
void csk_io35_high(void)   { PORTB |=  BIT2;  }
void csk_io35_low (void)   { PORTB &= ~BIT2;  }
void csk_io36_high(void)   { PORTB |=  BIT1;  }
void csk_io36_low (void)   { PORTB &= ~BIT1;  }
void csk_io37_high(void)   { PORTB |=  BIT0;  }
void csk_io37_low (void)   { PORTB &= ~BIT0;  }
void csk_io38_high(void)   { PORTB |=  BIT6;  }
void csk_io38_low (void)   { PORTB &= ~BIT6;  }
void csk_io39_high(void)   { PORTB |=  BIT7;  }
void csk_io39_low (void)   { PORTB &= ~BIT7;  }

void csk_io40_high(void)   { PORTB |=  BIT8;  }
void csk_io40_low (void)   { PORTB &= ~BIT8;  }
void csk_io41_high(void)   { PORTB |=  BIT9;  }
void csk_io41_low (void)   { PORTB &= ~BIT9;  }
void csk_io42_high(void)   { PORTB |=  BIT10; }
void csk_io42_low (void)   { PORTB &= ~BIT10; }
void csk_io43_high(void)   { PORTB |=  BIT11; }
void csk_io43_low (void)   { PORTB &= ~BIT11; }
void csk_io44_high(void)   { PORTB |=  BIT12; }
void csk_io44_low (void)   { PORTB &= ~BIT12; }
void csk_io45_high(void)   { PORTB |=  BIT13; }
void csk_io45_low (void)   { PORTB &= ~BIT13; }
void csk_io46_high(void)   { PORTB |=  BIT14; }
void csk_io46_low (void)   { PORTB &= ~BIT14; }
void csk_io47_high(void)   { PORTB |=  BIT15; }
void csk_io47_low (void)   { PORTB &= ~BIT15; }
