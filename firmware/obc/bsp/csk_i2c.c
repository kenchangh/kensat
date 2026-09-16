/******************************************************************************
KENSAT - Minimal I2C1 master driver for PIC24FJ256GB210

See csk_i2c.h. Fcy = 4 MHz (Fosc 8 MHz / 2, XT mode, no PLL).
I2C1BRG for 100 kHz: BRG = (Fcy/Fscl) - (Fcy * 130ns) - 1
                         = (4000000/100000) - 1 - 1 = 38
******************************************************************************/
#include "csk_i2c.h"

#if defined(__PIC24FJ256GB210__)

#include <p24Fxxxx.h>

/* ~10 ms timeout @ Fcy=4MHz; spin loop body is several cycles, so this
   is conservative; actual wall time is comfortably under 50 ms. */
#define I2C_SPIN_TIMEOUT  20000UL

#define WAIT_FLAG(cond) do {                              \
    unsigned long _t = I2C_SPIN_TIMEOUT;                  \
    while ((cond) && _t) { _t--; }                        \
    if (!_t) { I2C1CONbits.PEN = 1; return CSK_I2C_ERR_TIMEOUT; } \
} while (0)

static int wait_idle(void) {
    unsigned long t = I2C_SPIN_TIMEOUT;
    while (t && (I2C1CONbits.SEN || I2C1CONbits.RSEN || I2C1CONbits.PEN ||
                 I2C1CONbits.RCEN || I2C1CONbits.ACKEN || I2C1STATbits.TRSTAT)) {
        t--;
    }
    return t ? CSK_I2C_OK : CSK_I2C_ERR_TIMEOUT;
}

void csk_i2c1_open(void) {
    /* Disable while configuring */
    I2C1CONbits.I2CEN = 0;

    /* 100 kHz @ Fcy = 4 MHz */
    I2C1BRG = 38;

    /* Default 7-bit addressing, slew rate disabled (100 kHz),
       SMBus levels off, IPMI off, general call off. */
    I2C1CON = 0x0000;
    I2C1CONbits.DISSLW = 1;   /* slew rate control off for 100 kHz */
    I2C1CONbits.I2CEN  = 1;   /* enable, takes over RG2/RG3 */
}

static int start(void) {
    if (wait_idle() != CSK_I2C_OK) return CSK_I2C_ERR_TIMEOUT;
    I2C1CONbits.SEN = 1;
    WAIT_FLAG(I2C1CONbits.SEN);
    return CSK_I2C_OK;
}

static int rstart(void) {
    I2C1CONbits.RSEN = 1;
    WAIT_FLAG(I2C1CONbits.RSEN);
    return CSK_I2C_OK;
}

static int stop(void) {
    I2C1CONbits.PEN = 1;
    WAIT_FLAG(I2C1CONbits.PEN);
    return CSK_I2C_OK;
}

static int tx_byte(unsigned char b) {
    I2C1TRN = b;
    WAIT_FLAG(I2C1STATbits.TRSTAT);  /* wait for byte + ACK slot to complete */
    if (I2C1STATbits.ACKSTAT) {       /* 1 = NACK */
        I2C1CONbits.PEN = 1;
        return CSK_I2C_ERR_NACK;
    }
    return CSK_I2C_OK;
}

static int rx_byte(unsigned char *out, unsigned char nack) {
    I2C1CONbits.RCEN = 1;
    WAIT_FLAG(!I2C1STATbits.RBF);
    *out = (unsigned char)I2C1RCV;
    I2C1CONbits.ACKDT = nack ? 1 : 0;
    I2C1CONbits.ACKEN = 1;
    WAIT_FLAG(I2C1CONbits.ACKEN);
    return CSK_I2C_OK;
}

int csk_i2c1_write(unsigned char addr7, const unsigned char *buf, unsigned int n) {
    int rc;
    unsigned int i;
    if ((rc = start()) != CSK_I2C_OK) return rc;
    if ((rc = tx_byte((unsigned char)((addr7 << 1) | 0))) != CSK_I2C_OK) return rc;
    for (i = 0; i < n; i++) {
        if ((rc = tx_byte(buf[i])) != CSK_I2C_OK) return rc;
    }
    return stop();
}

int csk_i2c1_read(unsigned char addr7, unsigned char *buf, unsigned int n) {
    int rc;
    unsigned int i;
    if (n == 0) return CSK_I2C_OK;
    if ((rc = start()) != CSK_I2C_OK) return rc;
    if ((rc = tx_byte((unsigned char)((addr7 << 1) | 1))) != CSK_I2C_OK) return rc;
    for (i = 0; i < n; i++) {
        if ((rc = rx_byte(&buf[i], (i == n - 1) ? 1 : 0)) != CSK_I2C_OK) return rc;
    }
    return stop();
}

int csk_i2c1_write_read(unsigned char addr7,
                        const unsigned char *wbuf, unsigned int wn,
                        unsigned char *rbuf, unsigned int rn) {
    int rc;
    unsigned int i;
    if ((rc = start()) != CSK_I2C_OK) return rc;
    if ((rc = tx_byte((unsigned char)((addr7 << 1) | 0))) != CSK_I2C_OK) return rc;
    for (i = 0; i < wn; i++) {
        if ((rc = tx_byte(wbuf[i])) != CSK_I2C_OK) return rc;
    }
    if ((rc = rstart()) != CSK_I2C_OK) return rc;
    if ((rc = tx_byte((unsigned char)((addr7 << 1) | 1))) != CSK_I2C_OK) return rc;
    for (i = 0; i < rn; i++) {
        if ((rc = rx_byte(&rbuf[i], (i == rn - 1) ? 1 : 0)) != CSK_I2C_OK) return rc;
    }
    return stop();
}

#endif /* __PIC24FJ256GB210__ */
