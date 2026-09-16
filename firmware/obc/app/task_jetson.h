/******************************************************************************
KENSAT Jetson Orin Nano inference lifecycle task.

Boots the Jetson, runs one inference over UART0, then powers it off.
Triggered by csk_status.jetson_run_requested or jetson_request_*().

Each run produces:
  jetson_last_telem   housekeeping (temps, power, durations, hash); sent by
                      task_beacon as one "LLM" frame
  jetson_infer_result the response text; sent by task_beacon as a series of
                      "TXT" frames (see beacon_tx_llm_text())

Both carry the same seq and CRC-32 hash so the ground can match and verify
them.

******************************************************************************/
#ifndef __task_jetson_h
#define __task_jetson_h

#include "telemetry.h"


/* LLM telemetry packet plus OBC-side bookkeeping. */
typedef struct {
    unsigned char  valid;             /* 1 once first inference completes */
    unsigned char  pending_tx;        /* set by jetson, cleared by beacon TX */
    unsigned long  timestamp_ticks;   /* OSGetTicks() when populated */
    tlm_llm_t      tx;                /* packed wire-format payload */
} jetson_telem_t;

extern jetson_telem_t jetson_last_telem;

/* Written by task_jetson, read by task_sdlog (INFER.LOG) and task_beacon
   (TXT downlink).  Filled completely before either flag is raised; each
   consumer clears only its own flag. */
#define INFER_BUF_SIZE  512
typedef struct {
    unsigned char  pending;                /* 1 = new result ready for SD write */
    unsigned char  dl_pending;             /* 1 = text still to be downlinked */
    unsigned int   seq;                    /* result sequence (matches LLM frame) */
    unsigned int   prompt_id;              /* 0-19, or JETSON_PROMPT_UPLINK */
    unsigned long  uptime_s;               /* uptime when inference completed */
    unsigned int   response_len;           /* bytes in response[] */
    unsigned long  hash;                   /* CRC-32 of response[0..len) */
    char           response[INFER_BUF_SIZE]; /* LLM output text (null-terminated) */
} jetson_infer_result_t;

extern jetson_infer_result_t jetson_infer_result;


/* ---- Prompt selection ----------------------------------------------------
   Library prompt: /opt/kensat/prompts/00.txt..19.txt on the Jetson, sent as
   "RUN:XX".  Uplinked prompt: forwarded as "PROMPT:<text>" and reported as
   JETSON_PROMPT_UPLINK.  The uplinked prompt is RAM only and is lost on
   reset. */
#define JETSON_PROMPT_MAX     256   /* max uplinked prompt length (chars)     */
#define JETSON_PROMPT_UPLINK  99    /* prompt_id reported for uplinked text  */
#define JETSON_PROMPT_NEXT    0xFF  /* next prompt in the library            */

/* The functions below don't yield, so they can be called from another
   task's helper. */

/* Queue a run on library prompt id (0..19 or JETSON_PROMPT_NEXT).
   Returns 0 if busy or out of range. */
extern unsigned char jetson_request_prompt_id(unsigned char id);

/* Queue a run on an uplinked prompt (copied, truncated at
   JETSON_PROMPT_MAX).  Returns 0 if text is empty or busy. */
extern unsigned char jetson_request_prompt_text(const char *text);

/* 1 if the payload is powered or a run is already queued. */
extern unsigned char jetson_busy(void);

/* Standard CRC-32 (zlib.crc32). */
extern unsigned long jetson_crc32(const char *buf, unsigned int len);

// Function declarations
extern void task_jetson(void);

// Symbols, etc.
#define STR_TASK_JETSON "task_jetson: " STR_1TAB


#endif /* __task_jetson_h */
