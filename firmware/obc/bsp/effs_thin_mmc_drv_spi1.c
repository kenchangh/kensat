/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Src\\effs_thin_mmc_drv_spi1.c,v $
$Author: aek $
$Revision: 3.6 $
$Date: 2016-02-18 16:54:43-08 $

Vaguely related to HCC Embedded's mmc_drv.c (c)2003 for the MSP430F16x. Used 
with  permission.

EFFS-THIN MMC/SD Card driver for CubeSat Kit /PIC24, using SPI1. Two baud rates
are supported, based on the default CSK PPM D1 crystal of 8.000MHz.

******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif


#include "effs_thin_mmc_drv_spi1.h"

// Microchip PIC24 Libraries
#define USE_AND_OR 
#include <PPS.h>
#include <spi.h>


// Pumpkin CubeSat Kit
#include "csk_sd.h"


// Status register bits of interest
#define SPI1_Rcv_Complete SPI1STATbits.SPIRBF


// Local data.
// The selected baudrate, in Hz
unsigned long baudrate;  


// For setting and reading SPI baud rates:
//  * symbols for init and run mode values
//  * init and running SPI baud rates
//  * valid indeces into the SPI prescalar array
//  * SPI baud rate prescalar array
//  * SPI baud rates array
#define SPI_INIT_MODE_PRESCALAR          	SPI_PRESCALAR_DIV_32 	// 125kHz @ 8MHz
#define SPI_RUN_MODE_PRESCALAR          	SPI_PRESCALAR_DIV_4		// 1MHz @ 8MHz
#define SPI_INIT_SPEED_PRESCALAR         	(prescalars[SPI_INIT_MODE_PRESCALAR])
#define SPI_RUN_SPEED_PRESCALAR          	(prescalars[SPI_RUN_MODE_PRESCALAR])
#define SPI_INIT_BAUD_RATE              	(baudrates[SPI_INIT_MODE_PRESCALAR])
#define SPI_RUN_BAUD_RATE                	(baudrates[SPI_RUN_MODE_PRESCALAR])

#define SPI_PRESCALAR_DIV_512            	0
#define SPI_PRESCALAR_DIV_256            	1
#define SPI_PRESCALAR_DIV_128            	2
#define SPI_PRESCALAR_DIV_64            	3
#define SPI_PRESCALAR_DIV_32            	4
#define SPI_PRESCALAR_DIV_16            	5
#define SPI_PRESCALAR_DIV_8              	6
#define SPI_PRESCALAR_DIV_4              	7
#define SPI_PRESCALAR_DIV_2              	8
#define SPI_PRESCALAR_DIV_1              	9

const unsigned int prescalars[SPI_PRESCALAR_DIV_1+1] = { 
  PRI_PRESCAL_64_1 + SEC_PRESCAL_8_1,    	// :512
  PRI_PRESCAL_64_1 + SEC_PRESCAL_4_1,    	// :256
  PRI_PRESCAL_64_1 + SEC_PRESCAL_2_1,    	// :128
  PRI_PRESCAL_64_1 + SEC_PRESCAL_1_1,    	// :64
  PRI_PRESCAL_16_1 + SEC_PRESCAL_2_1,    	// :32
  PRI_PRESCAL_16_1 + SEC_PRESCAL_1_1,    	// :16
  PRI_PRESCAL_4_1  + SEC_PRESCAL_2_1,    	// :8
  PRI_PRESCAL_4_1  + SEC_PRESCAL_1_1,    	// :4
  PRI_PRESCAL_1_1  + SEC_PRESCAL_2_1,    	// :2
  PRI_PRESCAL_1_1  + SEC_PRESCAL_1_1     	// :1
  };
   
const unsigned long baudrates[SPI_PRESCALAR_DIV_1+1] = { 
  CSK_SD_OSC / 2 /512,                          
  CSK_SD_OSC / 2 /256,                        
  CSK_SD_OSC / 2 /128,                     
  CSK_SD_OSC / 2 / 64,  
  CSK_SD_OSC / 2 / 32, 
  CSK_SD_OSC / 2 / 16, 
  CSK_SD_OSC / 2 /  8, 
  CSK_SD_OSC / 2 /  4,
  CSK_SD_OSC / 2 /  2, 
  CSK_SD_OSC / 2 /  1,
  };
   

// User-configurable options.
// NOT CURRENTLY SUPPORTED 2010-04-24 -- change SPI_RUN_MODE_PRESCALAR instead
#define ENABLE_VARIABLE_MAX_BAUDRATE  0

#if ENABLE_VARIABLE_MAX_BAUDRATE
unsigned long spi_max_baudrate = 1000000; // in Hz
#endif


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_cs_lo()
spi_cs_hi()

Primitives to control the chip select line to the SD Card.

**                                                                           **
****                                                                       ****
******************************************************************************/
void spi_cs_lo(void) { PORTE &= ~CS_SD_BAR; }
void spi_cs_hi(void) { PORTE |=  CS_SD_BAR; }


/******************************************************************************
****                                                                       ****
**                                                                           **
ConfigureSPI1()

INPUT: prescalar_bits - bit values for primary and secondary prescalars.

Configure the SPI1 peripheral. Uses Microchip library functions.

**                                                                           **
****                                                                       ****
******************************************************************************/
void ConfigureSPI1(unsigned int prescalar_bits) {
  unsigned int SPICON1Value;
  unsigned int SPICON2Value;
  unsigned int SPISTATValue;

  
  // SPI1 configuration.
  //
  //   * SPI1 communication is BYTE (8 bits) wide.
  //   * At SD Card (SPI slave), MISO data is latched on the rising 
  //      edge of the clock. Therefore, at the SPI Master, MOSI data
  //      must be present and stable before this, so "transmit happens
  //      from active clock state to idle clock state."
  //   * SPI1 in MASTER mode.
  //   * Primary and secondary prescalar bits are passed in as paramaters
  //   * Frame SPI support Disable.
  //   * SPI1 enabled.
  //
  SPICON1Value = ENABLE_SCK_PIN         | ENABLE_SDO_PIN      | SPI_MODE8_ON     
               | MASTER_ENABLE_ON       | SPI_SMP_OFF         | SPI_CKE_ON       
               | SLAVE_ENABLE_OFF       | CLK_POL_ACTIVE_HIGH | MASTER_ENABLE_ON
               | (prescalar_bits & (SEC_PRESCAL_1_1 | PRI_PRESCAL_1_1));
  SPICON2Value = FRAME_ENABLE_OFF       | FRAME_SYNC_OUTPUT   | FRAME_SYNC_ACTIVE_LOW 
               | SPI_FRM_PULSE_PREV_CLK | SPI_ENH_BUFF_DISABLE;
  SPISTATValue = SPI_ENABLE             | SPI_IDLE_CON        | SPI_RX_OVFLOW_CLR;
  OpenSPI1(SPICON1Value,SPICON2Value,SPISTATValue);

} /* ConfigureSPI1() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_tx1()
spi_tx512()

Transmit 1 byte.
Transmit 512 bytes.

Thanks to user <ohmite> on Microchip forums 20090911 for this method of 
testing for transfer done (by looking at the receiver instead of the IFG flag).

Microchip library functions are not used here, for speed and code size.

**                                                                           **
****                                                                       ****
******************************************************************************/
void spi_tx1(unsigned char data8) {
  volatile int dummy;
  
  
  // Ensure buffer full flag is cleared before xfer starts.
  dummy = SPI1BUF;  
  
  // Start transfer.
  SPI1BUF = (unsigned int) data8;  
  
  // Checking the receive bit is done (instead of the IFG bit -- there are 
  //  errata and observed problems with that) is done appears to be a highly 
  //  reliable method to ensure the transfer is complete. Must do this so 
  //  that e.g. subsequent spi_cs_hi() does not occur before transfer is 
  //  truly complete.
  while (!SPI1_Rcv_Complete);

} /* spi_tx1() */


void spi_tx512(unsigned char *buf) { 
  unsigned int i;
  
  i = 512;
  do {
    spi_tx1(*buf++);
  } while (--i); /* do while */
  
} /* spi_tx512() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_rx1()
spi_rx512()

Receive 1 byte.
Receive 512 bytes.

Receiving from an SD Card via SPI is the same as transmitting 0xFF on MOSI 
and reading what came back simultaneously on MISO.

**                                                                           **
****                                                                       ****
******************************************************************************/
unsigned char spi_rx1(void) {

  // Receive is the same as transmit, but we pick up what came in ...
  spi_tx1(0xFF);

  return (unsigned char) SPI1BUF & 0xFF;

} /* spi_rx1() */


void spi_rx512(unsigned char *buf) {
  unsigned int i;
  
  
  i = 512;
  do {
    *buf++ = spi_rx1();
  } while (--i); /* do while */
  
} /* spi_rx512() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_init()

Init SPI peripheral to talk to an SD Card. requires general config of I/O
pins, as well as config of the SPI1 peripheral, and the related PPS pins.

**                                                                           **
****                                                                       ****
******************************************************************************/
int spi_init(void) {

  // Enable -CS_SD (IO.0) control, keep it high / inactive.
  csk_sd_open();  
  spi_cs_hi();  
  
  
  // Configure I/O pins.
  #if   defined(__PIC24FJ256GA110__)      
  // SDO1:    RF8  / IO.1(SDO0)
  // SDI1:    RF7  / IO.2(SDI0)
  // SCK1OUT: RD9  / IO.3(SCK0)
  PORTF |=  (SDO0); // SDO0 is an output, idle low
  TRISF &= ~(SDO0); //  ""
  PORTF &= ~(SDI0); // SDO1 is an input
  TRISF |=  (SDI0); // SDI0 is an input
  PORTD &= ~(SCK0); // SCK0 is an output, idle low
  TRISD &= ~(SCK0); //  ""
  #elif defined(__PIC24FJ256GB210__)  
  // SDO1:    RF8  / IO.1(SDO0)
  // SDI1:    RD1  / IO.2(SDI0)
  // SCK1OUT: RD2  / IO.3(SCK0)  
  PORTF |=  (SDO0); // SDO0 is an output, idle low
  TRISF &= ~(SDO0); //  ""
  PORTD &= ~(SDI0); // SDO1 is an input
  TRISD |=  (SDI0); // SDI0 is an input
  PORTD &= ~(SCK0); // SCK0 is an output, idle low
  TRISD &= ~(SCK0); //  ""
  #else
  #error PIC24F device not supported by CubeSat Kit.
  #endif 


  // Disable SPI1 module if previously enabled. Required because SPIxCONx
  //  registers cannot be written to while SPIx modules are enabled.
  CloseSPI1();    

  
  // 8 MHz (4MHz Fcy) oscillator with 32:1 prescalar yields 125MHz initial
  //  clock speed (must be <400kHz during card identification stage).
  ConfigureSPI1(SPI_INIT_SPEED_PRESCALAR);


  // Set baudrate placeholder
  baudrate = SPI_INIT_BAUD_RATE;
  
  
  // Configure I/O for SPI1 via PPS:
  #if   defined(__PIC24FJ256GA110__)      
  // SDO1:    RP15  / IO.1
  // SDI1:    RPI44 / IO.2
  // SCK1OUT: RP4   / IO.3
  iPPSOutput(OUT_PIN_PPS_RP15,OUT_FN_PPS_SDO1);
  iPPSInput(IN_FN_PPS_SDI1,   IN_PIN_PPS_RPI44);
  iPPSOutput(OUT_PIN_PPS_RP4, OUT_FN_PPS_SCK1OUT);
  #elif defined(__PIC24FJ256GB210__)    
  // SDO1:    RP15  / IO.1
  // SDI1:    RP24  / IO.2
  // SCK1OUT: RP23  / IO.3
  iPPSOutput(OUT_PIN_PPS_RP15,OUT_FN_PPS_SDO1);
  iPPSInput(IN_FN_PPS_SDI1,   IN_PIN_PPS_RP24);
  iPPSOutput(OUT_PIN_PPS_RP23,OUT_FN_PPS_SCK1OUT);  
  #else
  #error PIC24F device not supported by CubeSat Kit.
  #endif   


  // Done, leaving with SPI1 configured, and -CS_SD as an output and high/inactive.
  return 0;

} /* spi_init() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_set_baudrate()

Set SPI baudrate.

INPUT: br - baudrate

PIC24 has only coarse-scale tuning of SPI rates -- it has a 64/16/4/1:1 primary 
prescalar and a 8/7/6/5/4/3/2/1:1 secondary prescalar. For simplicity, we
just manhandle the SPI baud rate for two possible operating speeds: 
CSK_SD_INIT_SPEED & CSK_SD_MAX_SPEED, by specifying the appropriate prescalars.

Note the need to CloseSPI1() before changing the baud rates.

**                                                                           **
****                                                                       ****
******************************************************************************/
void spi_set_baudrate(unsigned long br) {

  // Send out a sweet nothing and ensure that all xfers are complete.
  spi_tx1(0xFF);
  
  // Either run at <400kHz (startup) or at target-specific maximum SD Card
  //  interface speed.
  if (br != CSK_SD_INIT_SPEED) {
  
    #if ENABLE_VARIABLE_MAX_BAUDRATE
    // Change the prescalar here to something that works with your particular
    //  configuration ...
    CloseSPI1();  
    ConfigureSPI1(prescalars[SPI_PRESCALAR_DIV_2]);   
    baudrate = baudrates[SPI_PRESCALAR_DIV_2];
    
    #else
    // Set baud rate to predefined max rate, based on prescalars.
    CloseSPI1();     
    ConfigureSPI1(SPI_RUN_SPEED_PRESCALAR);    
    baudrate = SPI_RUN_BAUD_RATE;
    #endif
    
  }
  else {
  
    // Set baud rate to identification-phase rate (< 400kHz).
    CloseSPI1();    
    ConfigureSPI1(SPI_INIT_SPEED_PRESCALAR); 
    baudrate = SPI_INIT_BAUD_RATE;
    
  } /* if */
} /* spi_set_baudrate() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_get_baudrate()

Get SPI baudrate.

Return: baudrate

**                                                                           **
****                                                                       ****
******************************************************************************/
unsigned long spi_get_baudrate(void) {

  return baudrate;
  
} /* spi_get_baudrate() */



/******************************************************************************
****                                                                       ****
**                                                                           **
spi_set_max_baudrate()

Set maximum SPI baudrate. User can change the baud rate at which EFFS-THIN
interacts with the SD Card by calling this before calling f_initvolume().

Note: No bounds checking!

**                                                                           **
****                                                                       ****
******************************************************************************/
#if ENABLE_VARIABLE_MAX_BAUDRATE

void spi_set_max_baudrate(unsigned long br) {

  spi_max_baudrate = br;

} /* spi_set_max_baudrate() */

#endif /* ENABLE_VARIABLE_MAX_BAUDRATE */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_get_max_baudrate()

Get maximum SPI baudrate.

Return: baudrate

**                                                                           **
****                                                                       ****
******************************************************************************/
#if ENABLE_VARIABLE_MAX_BAUDRATE

unsigned long spi_get_max_baudrate(void) {

  return spi_max_baudrate;

} /* spi_get_max_baudrate() */

#endif /* ENABLE_VARIABLE_MAX_BAUDRATE */


#ifdef __cplusplus
}
#endif

