/******************************************************************************
KENSAT Jetson Orin Nano inference lifecycle task.

Orchestrates boot -> handshake -> infer -> shutdown via UART0 and IO.16
power control.  Triggered by setting csk_status.jetson_run_requested, or by
either of the jetson_request_*() calls below (which the uplink telecommand
handler in task_beacon.c uses).

After each inference the task populates two separate things, because they go
to the ground by two separate routes:

  1. jetson_last_telem.tx -- a tlm_llm_t housekeeping packet (temperatures,
     power, durations, result hash).  pending_tx is raised and task_beacon
     sends it as ONE "LLM ..." frame on the next beacon cycle.

  2. jetson_infer_result -- the generated TEXT itself, which is the actual
     mission product.  It is too long for one AX.25 info field, so dl_pending
     is raised and task_beacon downlinks it as a numbered series of "TXT ..."
     frames, one chunk per beacon cycle (see beacon_tx_llm_text()).

Both carry the same result sequence number (seq), and the same CRC-32 of the
text (hash), so the ground can tie the housekeeping frame to the text frames
and verify a reassembled result against the hash the spacecraft computed.

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

/* Inference result buffer — written by task_jetson, read by task_sdlog
   (INFER.LOG) and by task_beacon (the "TXT" text downlink).

   Single-writer by construction: task_jetson fills the whole struct before
   raising either flag, and each consumer clears only its own flag.  Nothing
   here needs a lock because no helper that touches it yields -- a Salvo
   context switch cannot land in the middle of one. */
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

   Two ways to pick what the payload infers on:

     - a prompt from the library pre-loaded on the Jetson's eMMC
       (/opt/kensat/prompts/00.txt .. 19.txt), selected by index.  The OBC
       sends "RUN:XX" and never touches prompt content.

     - a free-text prompt uplinked from the ground, which the OBC forwards
       verbatim as "PROMPT:<text>".  Reported in telemetry as prompt_id
       JETSON_PROMPT_UPLINK so a result from an uplinked prompt can never be
       mistaken for library prompt 0.

   The uplinked prompt lives in RAM only: it is deliberately NOT persisted,
   so a reset always drops back to the library and the spacecraft cannot be
   stuck re-running a garbled prompt nobody can remember sending. */
#define JETSON_PROMPT_MAX     240   /* chars of uplinked prompt text         */
#define JETSON_PROMPT_UPLINK  99    /* prompt_id reported for uplinked text  */
#define JETSON_PROMPT_NEXT    0xFF  /* "whatever is next in the library"     */

/* Queue an inference run on a library prompt.  id is 0..19, or
   JETSON_PROMPT_NEXT to use the next one in sequence (the behaviour of the
   'j' console command).  Returns 0 if the payload is already busy. */
extern unsigned char jetson_request_prompt_id(unsigned char id);

/* Queue an inference run on an uplinked free-text prompt, which is copied in
   (truncated at JETSON_PROMPT_MAX).  Returns 0 if text is empty or the
   payload is already busy. */
extern unsigned char jetson_request_prompt_text(const char *text);

/* 1 if the payload is powered or a run is already queued.  Neither request
   call above, nor this one, yields -- all three are safe to call from
   another task's helper. */
extern unsigned char jetson_busy(void);

// Function declarations
extern void task_jetson(void);

// Symbols, etc.
#define STR_TASK_JETSON "task_jetson: " STR_1TAB


#endif /* __task_jetson_h */
