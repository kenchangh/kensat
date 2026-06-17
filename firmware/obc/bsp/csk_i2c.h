/******************************************************************************
KENSAT - Minimal I2C1 master driver for PIC24FJ256GB210

I2C1 SDA1 = RG3 (pin 56, CubeSat Kit H1.41 SDA_SYS)
I2C1 SCL1 = RG2 (pin 57, CubeSat Kit H1.43 SCL_SYS)

Polled, blocking, master-only. Bounded timeouts so a stuck bus does not
hang the cooperative scheduler. Bus is shared with EPS and the M41T81S
RTC; every transaction ends with STOP to leave the bus idle.

Return values: 0 = ok, negative = error (NACK or bus timeout).
******************************************************************************/
#ifndef __csk_i2c_h
#define __csk_i2c_h

#if defined(__PIC24FJ256GB210__)

#define CSK_I2C_OK              0
#define CSK_I2C_ERR_TIMEOUT    -1
#define CSK_I2C_ERR_NACK       -2

extern void csk_i2c1_open(void);
extern int  csk_i2c1_write(unsigned char addr7,
                           const unsigned char *buf, unsigned int n);
extern int  csk_i2c1_read (unsigned char addr7,
                           unsigned char *buf, unsigned int n);
extern int  csk_i2c1_write_read(unsigned char addr7,
                                const unsigned char *wbuf, unsigned int wn,
                                unsigned char *rbuf, unsigned int rn);

#endif /* __PIC24FJ256GB210__ */
#endif /* __csk_i2c_h */
