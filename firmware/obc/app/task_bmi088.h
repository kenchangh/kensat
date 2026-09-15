/******************************************************************************
KENSAT - BMI088 IMU Read Task

Polls the BMI088 accel (I2C 0x19) and gyro (I2C 0x69) on the shared
I2C1 bus and prints raw 16-bit counts to the debug console (CSK UART1).
******************************************************************************/
#ifndef __task_bmi088_h
#define __task_bmi088_h

extern void task_bmi088(void);

/* Read back GYR_LPM1 / ACC_PWR_CONF (0x80 / 0x03 == both suspended).
   Returns 0 on success. */
int  bmi088_get_pwr_state(unsigned char *gyr_lpm1, unsigned char *acc_pwr_conf);

/* Latest gyro reading in degrees/second (body frame), plus the magnitude
   of the angular-velocity vector. Updated by task_bmi088 each cycle;
   read by task_beacon for the tumbling fields of the beacon packet.
   Zero on startup until the first successful read. */
extern volatile int          g_gyro_x_dps;
extern volatile int          g_gyro_y_dps;
extern volatile int          g_gyro_z_dps;
extern volatile unsigned int g_tumble_mag_dps;

#define STR_TASK_BMI088 "task_bmi088: " STR_1TAB

#endif /* __task_bmi088_h */
