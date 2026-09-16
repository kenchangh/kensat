/******************************************************************************
KENSAT - Telemetry Frame Definitions

  tlm_beacon_t (frame 0x03)  housekeeping: power, tumble rates, comms
  tlm_llm_t    (frame 0x04)  Jetson thermals and LLM result, once per inference

Packed, little-endian. Bump frame_type when a layout changes.

******************************************************************************/
#ifndef __telemetry_h
#define __telemetry_h

#if defined(__PIC24FJ256GB210__)

/* Frame type identifiers */
#define TLM_FRAME_BEACON    0x03    /* tlm_beacon_t */
#define TLM_FRAME_LLM       0x04    /* tlm_llm_t */

/* Beacon interval in Salvo ticks (100 Hz -> 3000 = 30 s) */
#define BEACON_INTERVAL     3000

/* Callsign length (not NUL-terminated) */
#define TLM_CALLSIGN_LEN    6

/* LLM status flags */
#define TLM_LLM_MODEL_LOADED   0x01
#define TLM_LLM_INFERENCE_OK   0x02

/* Mission modes */
#define TLM_MODE_SAFE       0x00
#define TLM_MODE_NOMINAL    0x01
#define TLM_MODE_SCIENCE    0x02
#define TLM_MODE_DOWNLINK   0x03


/******************************************************************************
 tlm_beacon_t - housekeeping beacon (frame 0x03)

 Tumble: BMI088 gyro rates per axis in deg/s (+/-2000 dps range) plus the
 magnitude.
******************************************************************************/
typedef struct __attribute__((packed)) {
    /* Identification (9 B) */
    unsigned char  callsign[TLM_CALLSIGN_LEN];  /* "KO6IKZ" */
    unsigned char  frame_type;                  /* TLM_FRAME_BEACON */
    unsigned int   seq_num;                     /* beacon counter */

    /* OBC state (5 B) */
    unsigned long  uptime_s;                    /* seconds since boot */
    unsigned char  reboot_count;                /* saturates at 255 */

    /* Power (2 B). 0 = unknown / rail down. No battery-sense hardware. */
    unsigned int   vcc_3v3_mv;                  /* VCC_SYS (PIC24 VDD) */

    /* Tumbling (8 B). BMI088 gyro, body frame, deg/s. */
    int            gyro_x_dps;
    int            gyro_y_dps;
    int            gyro_z_dps;
    unsigned int   tumble_mag_dps;              /* sqrt(x^2 + y^2 + z^2) */

    /* Comms (5 B) */
    unsigned char  last_rssi;                   /* raw RSSI of last RX */
    unsigned int   tx_count;
    unsigned int   rx_count;

    /* Operational (1 B) */
    unsigned char  mission_mode;                /* TLM_MODE_* */
} tlm_beacon_t;
/* Total: 30 bytes */


/******************************************************************************
 tlm_llm_t - LLM inference report (frame 0x04), once per inference
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
