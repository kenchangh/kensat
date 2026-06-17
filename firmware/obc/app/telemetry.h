/******************************************************************************
KENSAT - Telemetry Frame Definitions

Two TX packet formats, both transmitted via Si4463 GFSK:

  tlm_beacon_t (frame 0x03)  - small, sent every BEACON_INTERVAL (~30 s).
    Power rails + tumbling state + comms counters. The "is the satellite
    alive and tumbling" packet.

  tlm_llm_t    (frame 0x04)  - larger, sent once per inference (~daily).
    Jetson thermals + LLM result metadata. Only meaningful right after an
    inference cycle, so we don't pay for it in every beacon.

Both structs are packed and little-endian; layout is wire format. Bump the
frame_type byte whenever the layout changes so the groundstation can parse.

******************************************************************************/
#ifndef __telemetry_h
#define __telemetry_h

#if defined(__PIC24FJ256GB210__)

/* Frame type identifiers (groundstation discriminator) */
#define TLM_FRAME_BEACON    0x03    /* tlm_beacon_t */
#define TLM_FRAME_LLM       0x04    /* tlm_llm_t */

/* Beacon interval in Salvo ticks (100 Hz -> 3000 = 30 s) */
#define BEACON_INTERVAL     3000

/* Callsign length (no null terminator in frame) */
#define TLM_CALLSIGN_LEN    6

/* EPS rail status bit flags */
#define TLM_RAIL_3V3        0x01
#define TLM_RAIL_5V         0x02
#define TLM_RAIL_VBATT      0x04
#define TLM_RAIL_JETSON     0x08

/* LLM status flags */
#define TLM_LLM_MODEL_LOADED   0x01
#define TLM_LLM_INFERENCE_OK   0x02

/* Mission modes */
#define TLM_MODE_SAFE       0x00
#define TLM_MODE_NOMINAL    0x01
#define TLM_MODE_SCIENCE    0x02
#define TLM_MODE_DOWNLINK   0x03


/******************************************************************************
 tlm_beacon_t - frequent housekeeping beacon (frame 0x03)

 Tumbling representation: three signed angular-rate components in deg/s
 (from the BMI088 gyro at ±2000 dps range) plus a precomputed magnitude.
 The components tell ground "how" the satellite is tumbling (which axis
 dominates and in which direction); the magnitude tells "how fast".
 Ground can recompute the magnitude if it wants, but precomputing it on
 the OBC keeps the beacon self-describing for quick triage.
******************************************************************************/
typedef struct __attribute__((packed)) {
    /* Identification (9 B) */
    unsigned char  callsign[TLM_CALLSIGN_LEN];  /* "KO6IKZ" */
    unsigned char  frame_type;                  /* TLM_FRAME_BEACON */
    unsigned int   seq_num;                     /* beacon counter */

    /* OBC state (5 B) */
    unsigned long  uptime_s;                    /* seconds since boot */
    unsigned char  reboot_count;                /* saturates at 255 */

    /* Power rails (7 B). 0 = unknown / rail down. */
    unsigned int   vbatt_mv;                    /* battery (SoC proxy), EPS I2C */
    unsigned int   vcc_5v_mv;                   /* +5V_SYS */
    unsigned int   vcc_3v3_mv;                  /* VCC_SYS (PIC24 VDD) */
    unsigned char  eps_rails;                   /* TLM_RAIL_* bitflags */

    /* Tumbling (8 B). BMI088 gyro, body frame, deg/s. */
    int            gyro_x_dps;
    int            gyro_y_dps;
    int            gyro_z_dps;
    unsigned int   tumble_mag_dps;              /* sqrt(x^2 + y^2 + z^2) */

    /* Comms (8 B) */
    unsigned char  tx_power;                    /* PA_PWR_LVL */
    unsigned char  last_rssi;                   /* raw RSSI of last RX */
    unsigned int   tx_count;
    unsigned int   rx_count;
    unsigned int   crc_err_count;

    /* Operational (1 B) */
    unsigned char  mission_mode;                /* TLM_MODE_* */
} tlm_beacon_t;
/* Total: 38 bytes */


/******************************************************************************
 tlm_llm_t - LLM inference report (frame 0x04)

 Transmitted once per inference cycle (roughly daily). Carries all the
 Jetson-side data so it doesn't have to ride along in every beacon.
******************************************************************************/
typedef struct __attribute__((packed)) {
    /* Identification (13 B) */
    unsigned char  callsign[TLM_CALLSIGN_LEN];
    unsigned char  frame_type;                  /* TLM_FRAME_LLM */
    unsigned int   seq_num;                     /* LLM-frame counter */
    unsigned long  uptime_s;                    /* uptime at inference completion */

    /* Prompt that was run (2 B) */
    unsigned int   prompt_id;                   /* 0..NUM_PROMPTS-1 */

    /* Jetson thermals, degC signed (8 B) */
    int            cpu_temp;
    int            gpu_temp;
    int            cpu_temp_peak;
    int            gpu_temp_peak;

    /* Jetson resources (4 B) */
    unsigned int   power_mw;
    unsigned int   ram_mb;

    /* Timing (6 B) */
    unsigned int   model_load_s;
    unsigned int   infer_dur_s;
    unsigned int   llm_latency_ms;

    /* Result (5 B) */
    unsigned char  llm_status;                  /* TLM_LLM_* bitflags */
    unsigned long  llm_hash;                    /* hash of response text */
} tlm_llm_t;
/* Total: 38 bytes */

#endif /* __PIC24FJ256GB210__ */

#endif /* __telemetry_h */
