/******************************************************************************
KENSAT - BMI088 IMU Read Task

Wiring (radio board → CubeSat Kit bus):
  PS=VCC          → I2C protocol mode
  CSB1=VCC, SDO1=VCC → accel @ 0x19
  SDO2=VCC        → gyro  @ 0x69 (CSB2 NC; relies on internal pull-up)
  SDA_SYS/SCL_SYS → PIC24 I2C1 (RG3/RG2)

Polled. Wakes the sensors, waits out the gyro's start-up, reads accel +
gyro, then returns both to suspend; repeats every ~5 s.  Raw signed 16-bit
counts are formatted for the debug console (CSK UART1) but the print is
currently commented out.

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
#define GYR_LPM1               0x11   /* 0x00 normal, 0x80 suspend         */
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
 Power management.

 The gyro defaults to normal mode and draws ~5 mA continuously -- by far the
 largest steady load on the 3V3 rail after the radio, and pure waste here: the
 beacon consumes gx/gy/gz once per ~12 s cycle and nothing else reads the IMU.
 So the sensors are parked in suspend (gyro ~20 uA, accel ~3 uA) and woken
 only around each sample.

 Neither helper delays.  The gyro's start-up wait belongs to the caller and is
 spent as an OS_Delay at the task's top level -- see BMI088_GYR_WAKE_TICKS
 below and the note at the top of this file about yielding from a helper.
******************************************************************************/
/* Start-up allowance between the suspend->normal write and the first read,
   in 10 ms Salvo ticks (30 = 300 ms).

   The datasheet's 30 ms gyro start-up is the point at which the part becomes
   operational, NOT the point at which its rate output is within the ~1 dps
   zero-rate spec.  Reading at 30 ms returns the start-up transient instead
   of a rate: bench-measured on 2026-08-22, stationary, the readings came out
   as exact integer multiples of a fixed vector -- raw (4026, -9865, 3075)
   counts per step -- walking up the ladder sample to sample and clipping at
   full scale, i.e. up to +/-2000 dps out of a unit sitting still on a desk.
   That is the bug this constant fixes; it is not sensor drift (a BMI088's
   zero-rate offset is ~1 dps with a 0.015 dps/K tempco) and not noise.

   Measured on the bench, same boot, gyro otherwise untouched:
        30 ms  -> 17 dps and climbing; 1598 dps after a spell of normal mode
       100 ms  -> 0.2 dps
       300 ms  -> 0.3 dps
       never suspended -> 0.2 dps
   300 ms is 3x the shortest setting measured good and 10x the datasheet
   figure.  The margin is close to free: the gyro is ~5 mA awake, so this is
   6% duty (~0.3 mA) against 5 mA for leaving it in normal mode.  The
   start-up transient is a mechanical settling process, so re-check this in
   thermal-vac before flight rather than assuming a bench margin holds cold.

   Spent as an OS_Delay at the task's top level, not busy_ms(): 300 ms of
   busy-wait every 5 s would burn ~0.36 mA of CPU and defeat POWER_IDLE for
   6% of the time, which would cost about as much as the gyro saving. */
#define BMI088_GYR_WAKE_TICKS  30

static void bmi088_wake(void) {
    /* Gyro first: its suspend->normal start-up dominates and the accel's
       (~1 ms) then overlaps it.  NO delay here -- the caller yields for
       BMI088_GYR_WAKE_TICKS at the task's top level. */
    (void)reg_write(BMI088_GYR_ADDR, GYR_LPM1,     0x00);  /* normal  */
    (void)reg_write(BMI088_ACC_ADDR, ACC_PWR_CONF, 0x00);  /* active  */
}

static void bmi088_suspend(void) {
    (void)reg_write(BMI088_ACC_ADDR, ACC_PWR_CONF, 0x03);  /* suspend */
    (void)reg_write(BMI088_GYR_ADDR, GYR_LPM1,     0x80);  /* suspend */
}


/******************************************************************************
 BMI088 init. Returns 0 on success.

 Leaves both sensors suspended; the sample loop wakes them per reading.
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

    /* Configured -- park both sensors until the first sample.  No start-up
       wait needed here: nothing is read before the loop's first wake. */
    bmi088_suspend();

    bmi_print("[bmi088] init ok");
    return 0;
}


/******************************************************************************
 bmi088_get_pwr_state()

 Read back the two power-mode registers so the console can confirm the
 sensors really are suspended between samples.  Without this the suspend
 writes are unverifiable: if they silently failed, the readings would look
 exactly the same and the ~5 mA saving would simply not happen.

 Both devices answer I2C in suspend (that is how they are woken), so this is
 safe to call at any time.  Expected while idle: GYR_LPM1 = 0x80,
 ACC_PWR_CONF = 0x03.
******************************************************************************/
int bmi088_get_pwr_state(unsigned char *gyr_lpm1, unsigned char *acc_pwr_conf) {
    int rc_g = reg_read(BMI088_GYR_ADDR, GYR_LPM1,     gyr_lpm1,     1);
    int rc_a = reg_read(BMI088_ACC_ADDR, ACC_PWR_CONF, acc_pwr_conf, 1);
    return (rc_g == CSK_I2C_OK && rc_a == CSK_I2C_OK) ? 0 : -1;
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

        /* Wake, then yield for the gyro's start-up.  The delay has to sit
           here rather than inside bmi088_wake() for two reasons: OS_Delay
           must stay at the task's top level (see the note at the top of this
           file), and yielding lets the core idle through the 300 ms instead
           of spinning in busy_ms().  Nothing else touches I2C1 or the
           BMI088, so it is safe to hold the sensors awake across the yield. */
        bmi088_wake();
        OS_Delay(BMI088_GYR_WAKE_TICKS);

        rc_a = bmi088_read_accel(&ax, &ay, &az);
        rc_g = bmi088_read_gyro (&gx, &gy, &gz);
        bmi088_suspend();

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

        /* ~5.3 s between samples (5 s here + the 300 ms start-up above).
           The beacon only publishes gx/gy/gz once per ~12 s cycle, so this is
           still 2x oversampled, and every second of it is a second the
           sensors spend suspended.
           Unrolled deliberately: Salvo Pro-lib delays are 8-bit
           (OSBYTES_OF_DELAYS=1) so args >255 truncate mod 256, and a for/while
           loop around OS_Delay corrupts the task on resume -- the delays have
           to sit at the task's top level. */
        OS_Delay(100);
        OS_Delay(100);
        OS_Delay(100);
        OS_Delay(100);
        OS_Delay(100);
    }
}
