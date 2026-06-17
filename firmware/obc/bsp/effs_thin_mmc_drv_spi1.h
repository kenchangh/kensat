/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\PIC24\\Inc\\effs_thin_mmc_drv_spi1.h,v $
$Author: aek $
$Revision: 3.3 $
$Date: 2016-02-18 16:55:01-08 $

Derived from HCC Embedded's mmc_drv.h (c)2003 for the MSP430F16x. Used with 
permission.

EFFS-THIN MMC/SD Card driver for CubeSat Kit /PIC24, using SPI1.

******************************************************************************/
#ifndef __effs_thin_mmc_drv_spi1_h
#define __effs_thin_mmc_drv_spi1_h


#ifdef __cplusplus
extern "C" {
#endif


// For PIC24FJ256GA110 (PPM D1) & PIC24FJ256GB210 (PPM E1)
#include <p24fxxxx.h>
#include "csk_hw.h"
#include "csk_sd.h"


// EFFS-THIN speed settings
#define CSK_SD_OSC          8000000     // Hz, CSK's external oscillator (no PLL active)
#define CSK_SD_INIT_SPEED    100000     // Hz, EFFS-THIN default (conservative)
#define CSK_SD_MAX_SPEED    1000000     // Hz, for most CSK hardware


// MMC driver function declarations.
         void spi_cs_lo(void);                    /* make SPI -CS signal active */
         void spi_cs_hi(void);                    /* make SPI -CS signal inactive */
         int  spi_init(void);    									/* init SPI */
         int  spi_delete(void);                
unsigned long spi_get_baudrate(void);  						/* get baudrate */
unsigned long spi_get_max_baudrate(void);  	
         void spi_set_baudrate(unsigned long);  	/* set baudrate */
         void spi_set_max_baudrate(unsigned long);  
unsigned char spi_rx1(void);  										/* receive 1 byte */
         void spi_rx512(unsigned char *);					/* receive 512 bytes */
         void spi_tx1(unsigned char);							/* transmit one SPI byte */
         void spi_tx512(unsigned char *);					/* transmit 512 bytes */


#define get_cd()            1											/* since these functions are not supported */
#define get_wp()            0											/*  by the CSK hardware, macros are OK. */


#ifdef __cplusplus
}
#endif


#endif /* __effs_thin_mmc_drv_spi1_h */

