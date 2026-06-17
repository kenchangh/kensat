/******************************************************************************
KENSAT Jetson Orin Nano inference lifecycle task.

Orchestrates boot -> handshake -> infer -> shutdown via UART0 and IO.16
power control.  Triggered by setting csk_status.jetson_run_requested.

After each inference, populates jetson_last_telem.tx (a tlm_llm_t TX
packet) and raises pending_tx so task_beacon will transmit it on the
next beacon cycle.

******************************************************************************/
#ifndef __task_jetson_h
#define __task_jetson_h

#include "telemetry.h"


/* Wrapper around the on-air LLM telemetry packet, with OBC-side
   bookkeeping that doesn't go over the radio. */
typedef struct {
    unsigned char  valid;             /* 1 once first inference completes */
    unsigned char  pending_tx;        /* set by jetson, cleared by beacon TX */
    unsigned long  timestamp_ticks;   /* OSGetTicks() when populated */
    tlm_llm_t      tx;                /* packed wire-format payload */
} jetson_telem_t;

extern jetson_telem_t jetson_last_telem;

/* Inference result buffer — set by task_jetson, consumed by task_sdlog */
#define INFER_BUF_SIZE  512
typedef struct {
    unsigned char  pending;                /* 1 = new result ready for SD write */
    unsigned int   prompt_id;              /* prompt index 0-19 */
    unsigned long  uptime_s;               /* uptime when inference completed */
    unsigned int   response_len;           /* bytes in response[] */
    char           response[INFER_BUF_SIZE]; /* LLM output text (null-terminated) */
} jetson_infer_result_t;

extern jetson_infer_result_t jetson_infer_result;

// Function declarations
extern void task_jetson(void);

// Symbols, etc.
#define STR_TASK_JETSON "task_jetson: " STR_1TAB


#endif /* __task_jetson_h */
