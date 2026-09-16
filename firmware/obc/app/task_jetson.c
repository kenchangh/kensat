/******************************************************************************
KENSAT Jetson Orin Nano inference lifecycle task.

  IDLE      wait for jetson_run_requested ('j' command or an uplink request)
  BOOT      IO.16 HIGH, 5 s later IO.17 (POWER_EN) HIGH, wait for "READY"
  SEND_RUN  send "RUN:XX" or "PROMPT:<text>", wait up to 120 s for "DONE:<len>"
  READ_RESP read the response line (30 s timeout)
  READ_TELEM read "TELEM:..." (20 s timeout, non-fatal)
  SHUTDOWN  send "OK" then "OFF", wait 10 s, cut power

A timeout in SEND_RUN or READ_RESP cuts power and returns to IDLE.

UART: CSK UART0 (IO.4 TX / IO.5 RX), 9600 8N1.
Power: IO.16 (RD8) via AP22811 load switch, active HIGH.

******************************************************************************/
#include "main.h"
#include "events.h"
#include "task_jetson.h"

// Pumpkin CubeSat Kit headers
#include "csk_io.h"
#include "csk_uart.h"
#include "pic24_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"

// Microchip peripheral library
#include <uart.h>

// C headers
#include <string.h>


jetson_telem_t jetson_last_telem;
jetson_infer_result_t jetson_infer_result;


/* Queued request, written by jetson_request_prompt_*() and consumed in
   JS_IDLE.  Neither writer yields, and req_pending is set last. */
static char          req_text[JETSON_PROMPT_MAX + 1];
static unsigned char req_is_text = 0;    /* 1 = run req_text, 0 = library  */
static unsigned char req_id      = JETSON_PROMPT_NEXT;
static unsigned char req_pending = 0;

/* Set from dispatch until power is cut.  jetson_power_on only goes TRUE 5 s
   into JS_BOOT, so this closes the window for a duplicate request. */
static unsigned char run_active  = 0;


/* ---- Timing (10 ms per tick) ---- */
#define TICK_250MS            25
#define TICKS_1S              100
#define TICKS_2S5             250

/* Hold JETSON_5V (IO.16) 5 s before asserting POWER_EN (IO.17). */
#define PWR_EN_DELAY_TICKS    500

/* Boot: 60 s = 24 x 2.5 s */
#define BOOT_WAIT_TICKS       TICKS_2S5
#define BOOT_WAIT_LOOPS       60

/* Boot RX poll 10 ms: the UART0 RX ring is only 64 bytes (RX1_BUFF_SIZE)
   and the boot log arrives at ~960 B/s. */
#define BOOT_POLL_TICKS       1

/* Inference: 120 s = 48 x 2.5 s */
#define INFER_WAIT_TICKS      TICKS_2S5
#define INFER_WAIT_LOOPS      48

/* Response body: 30 s = 12 x 2.5 s */
#define RESP_WAIT_TICKS       TICKS_2S5
#define RESP_WAIT_LOOPS       12

/* Telemetry: 20 s = 8 x 2.5 s */
#define TELEM_WAIT_TICKS      TICKS_2S5
#define TELEM_WAIT_LOOPS      8

/* Shutdown grace: 10 s = 4 x 2.5 s */
#define SHUTDOWN_WAIT_TICKS   TICKS_2S5
#define SHUTDOWN_WAIT_LOOPS   4

/* Idle poll: 2.5 s */
#define IDLE_POLL_TICKS       TICKS_2S5

#define RX_BUF_SIZE           512
#define MATCH_BUF_SIZE        64

#define NUM_PROMPTS           20


/* ---- State machine ---- */
typedef enum {
  JS_IDLE,
  JS_BOOT,
  JS_SEND_RUN,
  JS_READ_RESP,
  JS_READ_TELEM,
  JS_SHUTDOWN
} jetson_state_t;


/*--- helper: drain UART0 RX ---*/
static void jetson_drain_rx(void) {

  while (csk_uart0_getchar())
    ;
}


/*--- helper: polled TX on UART0 (PIC24 UART1), bypasses the BSP ring buffer ---*/
static void jetson_polled_puts(const char *s) {

  unsigned int saved_ie = IEC0bits.U1TXIE;
  IEC0bits.U1TXIE = 0;

  while (*s) {
    while (U1STAbits.UTXBF)
      ;
    U1TXREG = (unsigned int)*s++;
  }
  while (!U1STAbits.TRMT)
    ;

  IEC0bits.U1TXIE = saved_ie;
}


/*--- helper: parse unsigned decimal, skip a trailing ',' ---*/
static unsigned int parse_uint(const char **pp) {

  unsigned int val = 0;
  const char *p = *pp;

  while (*p >= '0' && *p <= '9') {
    val = val * 10U + (*p - '0');
    p++;
  }
  if (*p == ',')
    p++;
  *pp = p;
  return val;
}


/*--- helper: parse signed decimal, skip a trailing ',' ---*/
static int parse_int(const char **pp) {

  const char *p = *pp;
  int sign = 1;

  if (*p == '-') {
    sign = -1;
    p++;
  }
  *pp = p;
  return sign * (int)parse_uint(pp);
}


/*--- CRC-32 (IEEE 802.3, reflected, init/final 0xFFFFFFFF) -----------------
  Same as Python zlib.crc32.  Nibble table: 64 bytes of flash, ~2 ms for
  512 bytes.  Sent in the LLM and TXT frames as the only end-to-end check
  on the result text. */
unsigned long jetson_crc32(const char *buf, unsigned int len) {

  static const unsigned long tbl[16] = {
    0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
    0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
    0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
    0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL
  };
  unsigned long crc = 0xFFFFFFFFUL;
  unsigned int  i;

  for (i = 0; i < len; i++) {
    crc = tbl[(crc ^ (unsigned long)(unsigned char)buf[i]) & 0x0FUL]
          ^ (crc >> 4);
    crc = tbl[(crc ^ ((unsigned long)(unsigned char)buf[i] >> 4)) & 0x0FUL]
          ^ (crc >> 4);
  }
  return crc ^ 0xFFFFFFFFUL;

} /* jetson_crc32() */


/******************************************************************************
 jetson_busy()

 1 if the payload is powered or a run is already queued.
******************************************************************************/
unsigned char jetson_busy(void) {

  return (run_active
          || csk_status.jetson_power_on
          || csk_status.jetson_run_requested
          || req_pending) ? 1u : 0u;

} /* jetson_busy() */


/******************************************************************************
 jetson_request_prompt_id()

 Queue a run on library prompt id (0..NUM_PROMPTS-1) or JETSON_PROMPT_NEXT.
 Returns 0 if busy or id is out of range.  Refusing while busy keeps a
 repeated telecommand from stacking up back-to-back runs.
******************************************************************************/
unsigned char jetson_request_prompt_id(unsigned char id) {

  if (jetson_busy())
    return 0;

  if (id != JETSON_PROMPT_NEXT && id >= NUM_PROMPTS)
    return 0;

  req_is_text = 0;
  req_id      = id;
  req_pending = 1;
  csk_status.jetson_run_requested = TRUE;
  return 1;

} /* jetson_request_prompt_id() */


/******************************************************************************
 jetson_request_prompt_text()

 Queue a run on an uplinked prompt.  The text is copied, truncated at
 JETSON_PROMPT_MAX.  Returns 0 if the text is empty or the payload is busy.
******************************************************************************/
unsigned char jetson_request_prompt_text(const char *text) {

  unsigned int n = 0;

  if (text == 0 || text[0] == '\0')
    return 0;

  if (jetson_busy())
    return 0;

  while (text[n] != '\0' && n < JETSON_PROMPT_MAX) {
    req_text[n] = text[n];
    n++;
  }
  req_text[n] = '\0';

  req_is_text = 1;
  req_id      = JETSON_PROMPT_UPLINK;
  req_pending = 1;
  csk_status.jetson_run_requested = TRUE;
  return 1;

} /* jetson_request_prompt_text() */


/*--- helper: cut Jetson power (every path back to JS_IDLE goes through here) ---*/
static void jetson_hard_cut(void) {

  csk_io17_low();   /* POWER_EN first */
  csk_io16_low();   /* then JETSON_5V */
  csk_status.jetson_power_on = FALSE;
  run_active = 0;
}


/******************************************************************************
****                                                                       ****
**                                                                           **
task_jetson()

Jetson inference state machine.  Salvo context switches (OS_Delay etc.) only
work at task level, so all waits are inline here.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_jetson(void) {

  static jetson_state_t state;
  static unsigned int   i;
  static unsigned char  ch;

  /* prompt_id is the library cursor and only advances after a library run.
     run_pid is what gets reported: the library index or
     JETSON_PROMPT_UPLINK. */
  static unsigned int   prompt_id;
  static unsigned char  run_is_text;
  static unsigned int   run_pid;

  /* Result sequence number, shared by the LLM frame and its TXT frames. */
  static unsigned int   llm_seq;
  static unsigned int   result_seq;

  /* Protocol line buffer (READY, DONE:nnn, TELEM:...) */
  static char           match_buf[MATCH_BUF_SIZE];
  static unsigned int   match_len;

  /* Response payload buffer */
  static char           rx_buf[RX_BUF_SIZE];
  static unsigned int   rx_len;
  static unsigned char  got_reply;

  user_debug_msg(STR_TASK_JETSON "Starting.");
  OS_Delay(100);

  state       = JS_IDLE;
  run_active  = 0;
  prompt_id   = 0;
  run_is_text = 0;
  run_pid     = 0;
  llm_seq     = 0;
  result_seq  = 0;

  while (1) {

    switch (state) {

    /* ================================================================ */
    case JS_IDLE:
    /* ================================================================ */
      if (csk_status.jetson_run_requested) {
        csk_status.jetson_run_requested = FALSE;

        /* A bare jetson_run_requested ('j' command) means next library
           prompt. */
        run_is_text = 0;
        if (req_pending) {
          req_pending = 0;
          if (req_is_text) {
            run_is_text = 1;
          }
          else if (req_id != JETSON_PROMPT_NEXT) {
            prompt_id = req_id;
          }
        }
        run_pid    = run_is_text ? JETSON_PROMPT_UPLINK : prompt_id;
        run_active = 1;

        if (run_is_text) {
          sprintf(strTmp, STR_TASK_JETSON
                  "Inference requested on UPLINKED prompt (%u chars), "
                  "booting Jetson.", (unsigned int)strlen(req_text));
        } else {
          sprintf(strTmp, STR_TASK_JETSON
                  "Inference requested on library prompt %02u, booting Jetson.",
                  prompt_id);
        }
        user_debug_msg(strTmp);
        state = JS_BOOT;
      } else {
        OS_Delay(IDLE_POLL_TICKS);
      }
      break;


    /* ================================================================ */
    case JS_BOOT:
    /* ================================================================ */
      /* IO.16 HIGH enables the load switch (JETSON_5V), then after 5 s
         IO.17 HIGH drives POWER_EN through the 74LVC1G07 buffer. */
      _TRISD8 = 0;
      csk_io16_high();
      user_debug_msg(STR_TASK_JETSON "IO.16 HIGH (JETSON_5V on), waiting 5s before POWER_EN...");
      _TRISD3 = 0;
      csk_io17_low();
      OS_Delay(PWR_EN_DELAY_TICKS);
      csk_io17_high();
      user_debug_msg(STR_TASK_JETSON "IO.17 HIGH (POWER_EN asserted).");
      csk_status.jetson_power_on = TRUE;
      {
        unsigned long t0 = OSGetTicks();
        unsigned long t_last;
        unsigned long t_now;
        unsigned long dt_total;
        unsigned long dt_step;

        sprintf(strTmp, STR_TASK_JETSON "IO.16 HIGH at t=0.000s, waiting for READY (no timeout)...");
        user_debug_msg(strTmp);

        if (U1STAbits.OERR)
          U1STAbits.OERR = 0;
        jetson_drain_rx();

        match_len = 0;
        memset(match_buf, 0, MATCH_BUF_SIZE);
        got_reply = 0;
        t_last = t0;

        /* No timeout: wait for READY indefinitely with IO.16 held high. */
        for (i = 0; ; i++) {

          OS_Delay(BOOT_POLL_TICKS);

          /* A latched OERR stops RX until cleared. */
          if (U1STAbits.OERR)
            U1STAbits.OERR = 0;

          while ((ch = csk_uart0_getchar()) != 0) {
            if (ch == '\n') {
              match_buf[match_len] = '\0';
              /* Echo each line to the console.  HB: lines carry their own
                 uptime, so no timestamp on those. */
              if (match_len > 0) {
                if (strncmp(match_buf, "HB:", 3) == 0) {
                  sprintf(strTmp, STR_TASK_JETSON "RX: %s", match_buf);
                } else {
                  /* 16-bit tick diffs wrap at 655 s, well past boot. */
                  unsigned int t_now16  = (unsigned int)OSGetTicks();
                  unsigned int t0_16    = (unsigned int)t0;
                  unsigned int t_last16 = (unsigned int)t_last;
                  unsigned int dt_total16 = t_now16 - t0_16;
                  unsigned int dt_step16  = t_now16 - t_last16;
                  unsigned long dt_total_ms = (unsigned long)dt_total16 * 10UL;
                  unsigned long dt_step_ms  = (unsigned long)dt_step16  * 10UL;
                  t_last = (unsigned long)t_now16;
                  (void)t_now;
                  (void)dt_total;
                  (void)dt_step;
                  sprintf(strTmp,
                          STR_TASK_JETSON "[t=%lu.%03lus +%lu.%03lus] RX: %s",
                          dt_total_ms / 1000UL, dt_total_ms % 1000UL,
                          dt_step_ms  / 1000UL, dt_step_ms  % 1000UL,
                          match_buf);
                }
                user_debug_msg(strTmp);
              }
              if (strncmp(match_buf, "READY", 5) == 0) {
                got_reply = 1;
              }
              match_len = 0;
              break;
            }
            if (ch != '\r' && match_len < MATCH_BUF_SIZE - 1) {
              match_buf[match_len++] = ch;
            }
          }

          if (got_reply)
            break;
        }
      }

      if (!got_reply) {
        user_debug_msg(STR_TASK_JETSON "READY timeout — hard cut.");
        jetson_hard_cut();
        state = JS_IDLE;
        break;
      }

      user_debug_msg(STR_TASK_JETSON "Got READY.");
      state = JS_SEND_RUN;
      break;


    /* ================================================================ */
    case JS_SEND_RUN:
    /* ================================================================ */
      /* Debug: PPS register for RP4 and UART1 status */
      sprintf(strTmp, STR_TASK_JETSON "RPOR2=0x%04X U1MODE=0x%04X U1STA=0x%04X",
              RPOR2, U1MODE, U1STA);
      user_debug_msg(strTmp);

      /* "RUN:XX\r\n" or "PROMPT:<text>\r\n".  The prompt text is sent
         directly, not through strTmp (160 bytes, prompt up to 256). */
      if (run_is_text) {
        jetson_polled_puts("PROMPT:");
        jetson_polled_puts(req_text);
        jetson_polled_puts("\r\n");

        sprintf(strTmp, STR_TASK_JETSON
                "Sent PROMPT (%u chars): \"%.100s\", waiting for DONE (120 s)...",
                (unsigned int)strlen(req_text), req_text);
      } else {
        sprintf(strTmp, "RUN:%02u\r\n", prompt_id);
        jetson_polled_puts(strTmp);

        sprintf(strTmp, STR_TASK_JETSON
                "Sent RUN:%02u, waiting for DONE (120 s)...", prompt_id);
      }
      user_debug_msg(strTmp);

      /* Wait for "DONE:<length>\n" */
      match_len = 0;
      memset(match_buf, 0, MATCH_BUF_SIZE);
      got_reply = 0;

      for (i = 0; i < INFER_WAIT_LOOPS; i++) {

        OS_Delay(INFER_WAIT_TICKS);

        while ((ch = csk_uart0_getchar()) != 0) {
          if (ch == '\n') {
            match_buf[match_len] = '\0';
            /* Echo HB: and other lines while waiting */
            if (match_len > 0 && strncmp(match_buf, "DONE:", 5) != 0) {
              sprintf(strTmp, STR_TASK_JETSON "RX: %s", match_buf);
              user_debug_msg(strTmp);
            }
            if (strncmp(match_buf, "DONE:", 5) == 0) {
              got_reply = 1;
            }
            match_len = 0;
            break;
          }
          if (ch != '\r' && match_len < MATCH_BUF_SIZE - 1) {
            match_buf[match_len++] = ch;
          }
        }

        if (got_reply)
          break;
      }

      if (!got_reply) {
        user_debug_msg(STR_TASK_JETSON "DONE timeout — hard cut.");
        jetson_hard_cut();
        state = JS_IDLE;
        break;
      }

      user_debug_msg(STR_TASK_JETSON "Got DONE header.");
      state = JS_READ_RESP;
      break;


    /* ================================================================ */
    case JS_READ_RESP:
    /* ================================================================ */
      /* Read the response body up to '\n' */
      rx_len   = 0;
      got_reply = 0;

      for (i = 0; i < RESP_WAIT_LOOPS; i++) {

        OS_Delay(RESP_WAIT_TICKS);

        while ((ch = csk_uart0_getchar()) != 0) {
          if (ch == '\n') {
            got_reply = 1;
            break;
          }
          if (rx_len < RX_BUF_SIZE - 1) {
            rx_buf[rx_len++] = ch;
          }
        }

        if (got_reply)
          break;

        /* Give up after ~12 s of silence */
        if (rx_len == 0 && i >= 4)
          break;
      }

      rx_buf[rx_len] = '\0';

      /* Goes out as seq= in the LLM frame and s= in each TXT frame. */
      result_seq = llm_seq++;

      if (rx_len > 0) {
        sprintf(strTmp, STR_TASK_JETSON "Response [prompt %02u, %u bytes]:",
                run_pid, rx_len);
        user_debug_msg(strTmp);
        csk_uart1_puts(rx_buf);
        csk_uart1_puts(STR_CRLF);

        /* Copy to jetson_infer_result for task_sdlog and task_beacon,
           replacing anything outside printable ASCII with a space.  The
           CRC covers the sanitised text. */
        {
          unsigned int copy_len = rx_len;
          unsigned int ci;
          if (copy_len > INFER_BUF_SIZE - 1)
            copy_len = INFER_BUF_SIZE - 1;
          for (ci = 0; ci < copy_len; ci++) {
            char c = rx_buf[ci];
            jetson_infer_result.response[ci] =
                (c >= 0x20 && c <= 0x7E) ? c : ' ';
          }
          jetson_infer_result.response[copy_len] = '\0';
          jetson_infer_result.response_len = copy_len;
          jetson_infer_result.prompt_id = run_pid;
          jetson_infer_result.seq       = result_seq;
          jetson_infer_result.uptime_s = OSGetTicks() / 100UL;
          jetson_infer_result.hash =
              jetson_crc32(jetson_infer_result.response, copy_len);

          sprintf(strTmp, STR_TASK_JETSON
                  "Result #%u: %u bytes, CRC-32 %08lX.",
                  result_seq, copy_len, jetson_infer_result.hash);
          user_debug_msg(strTmp);
          /* pending flags are raised in JS_READ_TELEM */
        }
      } else {
        sprintf(strTmp, STR_TASK_JETSON "No response body for prompt %02u.",
                run_pid);
        user_debug_msg(strTmp);
      }

      state = JS_READ_TELEM;
      break;


    /* ================================================================ */
    case JS_READ_TELEM:
    /* ================================================================ */
      /* Wait for "TELEM:cpu,gpu,cpu_pk,gpu_pk,pwr,ram,infer,load\n" */
      match_len = 0;
      memset(match_buf, 0, MATCH_BUF_SIZE);
      got_reply = 0;

      for (i = 0; i < TELEM_WAIT_LOOPS; i++) {

        OS_Delay(TELEM_WAIT_TICKS);

        while ((ch = csk_uart0_getchar()) != 0) {
          if (ch == '\n') {
            match_buf[match_len] = '\0';
            if (strncmp(match_buf, "TELEM:", 6) == 0) {
              got_reply = 1;
            }
            match_len = 0;
            break;
          }
          if (ch != '\r' && match_len < MATCH_BUF_SIZE - 1) {
            match_buf[match_len++] = ch;
          }
        }

        if (got_reply)
          break;
      }

      if (got_reply) {
        const char *p = match_buf + 6;

        memcpy(jetson_last_telem.tx.callsign, "KO6IKZ", TLM_CALLSIGN_LEN);
        jetson_last_telem.tx.frame_type     = TLM_FRAME_LLM;
        jetson_last_telem.tx.seq_num        = result_seq;
        jetson_last_telem.tx.uptime_s       = OSGetTicks() / 100UL;
        jetson_last_telem.tx.prompt_id      = run_pid;

        jetson_last_telem.tx.cpu_temp       = parse_int(&p);
        jetson_last_telem.tx.gpu_temp       = parse_int(&p);
        jetson_last_telem.tx.cpu_temp_peak  = parse_int(&p);
        jetson_last_telem.tx.gpu_temp_peak  = parse_int(&p);
        jetson_last_telem.tx.power_mw       = parse_uint(&p);
        jetson_last_telem.tx.ram_mb         = parse_uint(&p);
        jetson_last_telem.tx.infer_dur_s    = parse_uint(&p);
        jetson_last_telem.tx.model_load_s   = parse_uint(&p);

        /* Latency mirrors infer_dur_s for now.  Hash is 0 if there was no
           response text. */
        jetson_last_telem.tx.llm_latency_ms = jetson_last_telem.tx.infer_dur_s * 1000U;
        jetson_last_telem.tx.llm_status     = TLM_LLM_MODEL_LOADED | TLM_LLM_INFERENCE_OK;
        jetson_last_telem.tx.llm_hash       =
            (jetson_infer_result.response_len > 0) ? jetson_infer_result.hash : 0UL;

        jetson_last_telem.timestamp_ticks   = OSGetTicks();
        jetson_last_telem.valid             = 1;
        jetson_last_telem.pending_tx        = 1;   /* task_beacon will TX */

        sprintf(strTmp, STR_TASK_JETSON "TELEM: cpu=%d gpu=%d pwr=%u infer=%us",
                jetson_last_telem.tx.cpu_temp_peak,
                jetson_last_telem.tx.gpu_temp_peak,
                jetson_last_telem.tx.power_mw,
                jetson_last_telem.tx.infer_dur_s);
        user_debug_msg(strTmp);
      } else {
        user_debug_msg(STR_TASK_JETSON "TELEM timeout (non-fatal).");
      }

      /* Release the result even if TELEM timed out:
           pending    -> task_sdlog writes INFER.LOG
           dl_pending -> task_beacon sends TXT frames */
      if (jetson_infer_result.response_len > 0) {
        jetson_infer_result.pending    = 1;
        jetson_infer_result.dl_pending = 1;
      }

      state = JS_SHUTDOWN;
      break;


    /* ================================================================ */
    case JS_SHUTDOWN:
    /* ================================================================ */
      jetson_polled_puts("OK\r\n");
      OS_Delay(TICK_250MS);
      jetson_polled_puts("OFF\r\n");

      user_debug_msg(STR_TASK_JETSON "Sent OK + OFF, waiting 10 s for shutdown...");

      for (i = 0; i < SHUTDOWN_WAIT_LOOPS; i++)
        OS_Delay(SHUTDOWN_WAIT_TICKS);

      jetson_hard_cut();
      user_debug_msg(STR_TASK_JETSON "IO.16 LOW, Jetson powered off.");

      /* Uplinked prompts don't advance the library cursor. */
      if (!run_is_text) {
        prompt_id++;
        if (prompt_id >= NUM_PROMPTS)
          prompt_id = 0;
      }

      sprintf(strTmp, STR_TASK_JETSON
              "%s Next library prompt: %02u. Returning to IDLE.",
              run_is_text ? "Uplinked prompt done (library cursor unchanged)."
                          : "Library prompt done.",
              prompt_id);
      user_debug_msg(strTmp);

      state = JS_IDLE;
      break;

    } /* switch */

  } /* while */

} /* task_jetson() */
