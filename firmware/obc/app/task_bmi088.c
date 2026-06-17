/******************************************************************************
KENSAT - BMI088 IMU Read Task

Wiring (radio board → CubeSat Kit bus):
  PS=VCC          → I2C protocol mode
  CSB1=VCC, SDO1=VCC → accel @ 0x19
  SDO2=VCC        → gyro  @ 0x69 (CSB2 NC; relies on internal pull-up)
  SDA_SYS/SCL_SYS → PIC24 I2C1 (RG3/RG2)

Polled. Reads accel + gyro every ~0.5 s, prints raw signed 16-bit
counts on the debug console (CSK UART1).

Salvo-on-PIC24 quirk: yielding (OS_Delay) from inside a function that
this task calls (e.g. bmi088_init()) corrupts the return path back into
task_bmi088 — the chip resets on the function epilogue. To work around
this, all delays inside bmi088_init() use a CPU busy-wait (busy_ms())
instead of OS_Delay. Yields only happen at the top level of the task.
******************************************************************************/
#include "main.h"
#include "task_bmi088.h"
#include "csk_i2c.h"
#include "csk_uart.h"
#include "salvo.h"

#include <stdio.h>


/* Tumbling state, published to task_beacon. BMI088 gyro at ±2000 dps,
   so per-axis values are clamped to int range with plenty of headroom. */
volatile int          g_gyro_x_dps     = 0;
volatile int          g_gyro_y_dps     = 0;
volatile int          g_gyro_z_dps     = 0;
volatile unsigned int g_tumble_mag_dps = 0;


/* Integer square root of an unsigned long. Newton iteration, ~6 iters
   for any 32-bit input. No <math.h>, no FPU traffic. */
static unsigned int isqrt_u32(unsigned long n) {
    unsigned long x;
    unsigned long y;
    unsigned char k;

    if (n == 0) return 0;
    x = n;
    y = (x + 1UL) >> 1;
    for (k = 0; k < 16 && y < x; k++) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return (unsigned int)x;
}


/******************************************************************************
 BMI088 register map (only what we use)
******************************************************************************/
#define BMI088_ACC_ADDR        0x19
#define BMI088_GYR_ADDR        0x69

/* Accel registers */
#define ACC_CHIP_ID            0x00   /* expect 0x1E */
#define ACC_X_LSB              0x12
#define ACC_CONF               0x40
#define ACC_RANGE              0x41
#define ACC_PWR_CONF           0x7C
#define ACC_PWR_CTRL           0x7D
#define ACC_SOFTRESET          0x7E

/* Gyro registers */
#define GYR_CHIP_ID            0x00   /* expect 0x0F */
#define GYR_RATE_X_LSB         0x02
#define GYR_RANGE              0x0F
#define GYR_BANDWIDTH          0x10
#define GYR_SOFTRESET          0x14


/* Busy-wait approximately n milliseconds at Fcy = 4 MHz. Must be used
   instead of OS_Delay inside any helper called by task_bmi088. */
static void busy_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 400U; j++) {
            __builtin_nop();
        }
        __builtin_clrwdt();
    }
}


/******************************************************************************
 Low-level helpers
******************************************************************************/
static int reg_write(unsigned char addr, unsigned char reg, unsigned char val) {
    unsigned char buf[2];
    buf[0] = reg;
    buf[1] = val;
    return csk_i2c1_write(addr, buf, 2);
}

static int reg_read(unsigned char addr, unsigned char reg,
                    unsigned char *out, unsigned int n) {
    return csk_i2c1_write_read(addr, &reg, 1, out, n);
}


/* This task does NOT use the shared strTmp / user_debug_msg path —
   uart2_msg_ts's busy-wait on tx_free was destabilising the system
   when called from this task. Use csk_uart1_puts() directly. */
static char bmi_buf[96];

static void bmi_print(const char *s) {
    csk_uart1_puts(s);
    csk_uart1_puts("\r\n");
}


/******************************************************************************
 BMI088 init. Returns 0 on success.
******************************************************************************/
static int bmi088_init(void) {
    unsigned char id;
    int rc;

    /* --- Accel --- */
    (void)reg_write(BMI088_ACC_ADDR, ACC_SOFTRESET, 0xB6);
    busy_ms(50);

    /* First I2C transaction after reset can NACK; do a dummy chip-id
       read and ignore the result, then read again. */
    (void)reg_read(BMI088_ACC_ADDR, ACC_CHIP_ID, &id, 1);
    busy_ms(20);
    rc = reg_read(BMI088_ACC_ADDR, ACC_CHIP_ID, &id, 1);
    if (rc != CSK_I2C_OK) {
        sprintf(bmi_buf, "[bmi088] accel CHIP_ID read failed (%d)", rc);
        bmi_print(bmi_buf);
        return -1;
    }
    sprintf(bmi_buf, "[bmi088] accel CHIP_ID=0x%02X (expect 0x1E)", id);
    bmi_print(bmi_buf);

    /* Power on accel */
    if (reg_write(BMI088_ACC_ADDR, ACC_PWR_CTRL, 0x04) != CSK_I2C_OK) return -2;
    busy_ms(60);
    if (reg_write(BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00) != CSK_I2C_OK) return -3;
    busy_ms(60);

    /* Range ±6 g, ODR 100 Hz / OSR4 */
    (void)reg_write(BMI088_ACC_ADDR, ACC_RANGE, 0x01);
    (void)reg_write(BMI088_ACC_ADDR, ACC_CONF,  0xA8);

    /* --- Gyro --- */
    rc = reg_read(BMI088_GYR_ADDR, GYR_CHIP_ID, &id, 1);
    if (rc != CSK_I2C_OK) {
        sprintf(bmi_buf, "[bmi088] gyro CHIP_ID read failed (%d) "
                        "(check CSB2 pull-up?)", rc);
        bmi_print(bmi_buf);
        return -4;
    }
    sprintf(bmi_buf, "[bmi088] gyro  CHIP_ID=0x%02X (expect 0x0F)", id);
    bmi_print(bmi_buf);

    (void)reg_write(BMI088_GYR_ADDR, GYR_SOFTRESET, 0xB6);
    busy_ms(50);
    (void)reg_write(BMI088_GYR_ADDR, GYR_RANGE,     0x00);  /* ±2000 dps */
    (void)reg_write(BMI088_GYR_ADDR, GYR_BANDWIDTH, 0x07);  /* 100 Hz ODR */

    bmi_print("[bmi088] init ok");
    return 0;
}


/******************************************************************************
 Read 6 axes. BMI088 over I2C does NOT need a dummy byte (that's a
 SPI-only quirk). Read 6 LSB/MSB pairs straight from X_LSB.
******************************************************************************/
static int bmi088_read_accel(int *x, int *y, int *z) {
    unsigned char raw[6];
    int rc = reg_read(BMI088_ACC_ADDR, ACC_X_LSB, raw, 6);
    if (rc != CSK_I2C_OK) return rc;
    *x = (int)((unsigned int)raw[0] | ((unsigned int)raw[1] << 8));
    *y = (int)((unsigned int)raw[2] | ((unsigned int)raw[3] << 8));
    *z = (int)((unsigned int)raw[4] | ((unsigned int)raw[5] << 8));
    return CSK_I2C_OK;
}

static int bmi088_read_gyro(int *x, int *y, int *z) {
    unsigned char raw[6];
    int rc = reg_read(BMI088_GYR_ADDR, GYR_RATE_X_LSB, raw, 6);
    if (rc != CSK_I2C_OK) return rc;
    *x = (int)((unsigned int)raw[0] | ((unsigned int)raw[1] << 8));
    *y = (int)((unsigned int)raw[2] | ((unsigned int)raw[3] << 8));
    *z = (int)((unsigned int)raw[4] | ((unsigned int)raw[5] << 8));
    return CSK_I2C_OK;
}


/******************************************************************************
 task_bmi088() — Salvo entry point
******************************************************************************/
void task_bmi088(void) {
    int ax, ay, az, gx, gy, gz;
    int rc_a, rc_g;

    /* Let other tasks bring up first. */
    OS_Delay(200);
    OS_Delay(200);

    if (bmi088_init() != 0) {
        bmi_print("[bmi088] init failed; halting task");
        while (1) { OS_Delay(250); }
    }

    while (1) {
        __builtin_clrwdt();

        rc_a = bmi088_read_accel(&ax, &ay, &az);
        rc_g = bmi088_read_gyro (&gx, &gy, &gz);

        if (rc_a == CSK_I2C_OK && rc_g == CSK_I2C_OK) {
            /* Convert raw gyro counts to deg/s. Range = ±2000 dps over
               int16, so 1 LSB = 2000/32768 dps ≈ 0.061 dps.
               dps = raw * 2000 / 32768. */
            int dx = (int)(((long)gx * 2000L) >> 15);
            int dy = (int)(((long)gy * 2000L) >> 15);
            int dz = (int)(((long)gz * 2000L) >> 15);
            unsigned long sq;

            g_gyro_x_dps = dx;
            g_gyro_y_dps = dy;
            g_gyro_z_dps = dz;

            sq = (unsigned long)((long)dx * (long)dx)
               + (unsigned long)((long)dy * (long)dy)
               + (unsigned long)((long)dz * (long)dz);
            g_tumble_mag_dps = isqrt_u32(sq);

            sprintf(bmi_buf,
                    "[bmi088] A=%6d,%6d,%6d  G=%6d,%6d,%6d  tumble=%udps",
                    ax, ay, az, gx, gy, gz, g_tumble_mag_dps);
        } else {
            sprintf(bmi_buf, "[bmi088] read err: a=%d g=%d", rc_a, rc_g);
        }
        // bmi_print(bmi_buf);

        OS_Delay(500);    /* ~50 ms @ 100 Hz tick — temporarily fast for live viz */
    }
}
