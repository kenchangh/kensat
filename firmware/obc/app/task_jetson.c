/******************************************************************************
KENSAT Jetson Orin Nano inference lifecycle task.

Orchestrates the full boot -> handshake -> infer -> shutdown cycle:

  1. IDLE:      Wait for jetson_run_requested flag (set by 'j' command).
  2. BOOT:      Drive IO.16 HIGH to power the Jetson.  Wait up to 60 s
                for the daemon to send "READY\n".
  3. SEND_RUN:  Send "RUN:XX\n" (prompt ID).  Wait up to 120 s for
                "DONE:<length>\n" response header.
  4. READ_RESP: Accumulate response bytes until '\n' or 30 s timeout.
  5. SHUTDOWN:  Send "OK\n" then "OFF\n".  Wait 10 s for graceful
                shutdown, then drive IO.16 LOW.

Any timeout in states 2-4 triggers a hard power cut (IO.16 LOW) and
returns to IDLE.

UART: CSK UART0 (IO.4 TX / IO.5 RX), 9600 N81.
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


/* ---- Global telemetry from last Jetson inference ---- */
jetson_telem_t jetson_last_telem;

/* ---- Inference result buffer for SD logging ---- */
jetson_infer_result_t jetson_infer_result;


/* ---- Timing (10 ms per tick) ---- */
#define TICK_250MS            25
#define TICKS_1S              100
#define TICKS_2S5             250

/* Power sequencing: hold JETSON_5V (IO.16) for 5 s before asserting
   POWER_EN (IO.17) so the 5V rail is fully up first. */
#define PWR_EN_DELAY_TICKS    500

/* Boot: 60 s = 24 x 2.5 s */
#define BOOT_WAIT_TICKS       TICKS_2S5
#define BOOT_WAIT_LOOPS       60

/* Boot RX poll: 10 ms. The Jetson UART ring buffer is only 64 bytes
   (RX1_BUFF_SIZE) and the daemon dumps boot logs at ~960 B/s, so we must
   drain it far faster than the old 2.5 s cadence or it overruns and stalls. */
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


/*--- helper: drain all pending chars from UART0 RX buffer ---*/
static void jetson_drain_rx(void) {

  while (csk_uart0_getchar())
    ;
}


/*--- helper: polled TX on UART0 (PIC24 UART1) — bypasses BSP ring buffer ---*/
static void jetson_polled_puts(const char *s) {

  unsigned int saved_ie = IEC0bits.U1TXIE;
  IEC0bits.U1TXIE = 0;          /* disable TX ISR so it can't interfere */

  while (*s) {
    while (U1STAbits.UTXBF)     /* wait for TX buffer not full */
      ;
    U1TXREG = (unsigned int)*s++;
  }
  /* Wait for last byte to finish shifting out */
  while (!U1STAbits.TRMT)
    ;

  IEC0bits.U1TXIE = saved_ie;   /* restore TX ISR */
}


/*--- helper: parse unsigned decimal from string, advance pointer ---*/
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


/*--- helper: parse signed decimal from string, advance pointer ---*/
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


/*--- helper: hard-cut Jetson power ---*/
static void jetson_hard_cut(void) {

  csk_io17_low();   /* drop POWER_EN first */
  csk_io16_low();   /* then JETSON_5V */
  csk_status.jetson_power_on = FALSE;
}


/******************************************************************************
****                                                                       ****
**                                                                           **
task_jetson()

Main Salvo task: state machine for the Jetson inference lifecycle.

All OS_WaitSem / OS_Delay calls are inline because Salvo context-switch
macros cannot be used in sub-functions.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_jetson(void) {

  static jetson_state_t state;
  static unsigned int   i;
  static unsigned char  ch;
  static unsigned int   prompt_id;

  /* Match buffer for protocol lines (READY, DONE:nnn) */
  static char           match_buf[MATCH_BUF_SIZE];
  static unsigned int   match_len;

  /* Response payload buffer */
  static char           rx_buf[RX_BUF_SIZE];
  static unsigned int   rx_len;
  static unsigned char  got_reply;

  user_debug_msg(STR_TASK_JETSON "Starting.");
  OS_Delay(100);

  /* Auto-fire Jetson boot+inference on PIC24 startup so the UART debugger
     can capture READY / RUN / DONE traffic without a manual 'j' command. */
  state     = JS_BOOT;
  prompt_id = 0;
  user_debug_msg(STR_TASK_JETSON "Auto-firing Jetson boot on startup.");

  while (1) {

    switch (state) {

    /* ================================================================ */
    case JS_IDLE:
    /* ================================================================ */
      if (csk_status.jetson_run_requested) {
        csk_status.jetson_run_requested = FALSE;
        user_debug_msg(STR_TASK_JETSON "Inference requested, booting Jetson.");
        state = JS_BOOT;
      } else {
        OS_Delay(IDLE_POLL_TICKS);
      }
      break;


    /* ================================================================ */
    case JS_BOOT:
    /* ================================================================ */
      /* Power on: two-stage sequence.
         1. IO.16 HIGH enables the TPS22975 load switch -> JETSON_5V flows.
         2. Wait 5 s for the 5V rail to settle.
         3. IO.17 HIGH drives POWER_EN through the 74LVC1G07 buffer. */
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

        /* Clear overrun error and drain stale bytes */
        if (U1STAbits.OERR)
          U1STAbits.OERR = 0;
        jetson_drain_rx();

        /* Wait for "READY\n" — accumulate chars into match_buf */
        match_len = 0;
        memset(match_buf, 0, MATCH_BUF_SIZE);
        got_reply = 0;
        t_last = t0;

        /* No timeout — keep waiting for READY indefinitely.  IO.16 stays
           high so the Jetson keeps trying to boot.  Use 'i' just as a
           loop counter for reference; we never exit the loop on count. */
        for (i = 0; ; i++) {

          OS_Delay(BOOT_POLL_TICKS);

          /* Recover the UART if the ring buffer overran: a latched OERR
             freezes RX until cleared, which would silence the rest of the
             Jetson boot log. */
          if (U1STAbits.OERR)
            U1STAbits.OERR = 0;

          while ((ch = csk_uart0_getchar()) != 0) {
            if (ch == '\n') {
              match_buf[match_len] = '\0';
              /* Forward every line received from Jetson to debug console.
                 Skip timestamp prefix on HB:* (they carry their own uptime
                 and arrive long after t0 — 16-bit tick diff would wrap). */
              if (match_len > 0) {
                if (strncmp(match_buf, "HB:", 3) == 0) {
                  sprintf(strTmp, STR_TASK_JETSON "RX: %s", match_buf);
                } else {
                  /* Use 16-bit subtraction so the diff wraps cleanly at
                     65535 ticks (655 s). Boot breadcrumbs all land
                     well within that window. */
                  unsigned int t_now16  = (unsigned int)OSGetTicks();
                  unsigned int t0_16    = (unsigned int)t0;
                  unsigned int t_last16 = (unsigned int)t_last;
                  unsigned int dt_total16 = t_now16 - t0_16;
                  unsigned int dt_step16  = t_now16 - t_last16;
                  unsigned long dt_total_ms = (unsigned long)dt_total16 * 10UL;
                  unsigned long dt_step_ms  = (unsigned long)dt_step16  * 10UL;
                  t_last = (unsigned long)t_now16;
                  (void)t_now;  /* keep var ref to avoid warning */
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
      /* Debug: dump PPS register for RP4 and UART1 status */
      sprintf(strTmp, STR_TASK_JETSON "RPOR2=0x%04X U1MODE=0x%04X U1STA=0x%04X",
              RPOR2, U1MODE, U1STA);
      user_debug_msg(strTmp);

      /* Send RUN:XX — use polled TX to bypass BSP ring buffer */
      sprintf(strTmp, "RUN:%02u\r\n", prompt_id);
      jetson_polled_puts(strTmp);

      sprintf(strTmp, STR_TASK_JETSON "Sent RUN:%02u, waiting for DONE (120 s)...",
              prompt_id);
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
            /* Forward HB:* / unexpected lines to debug console while waiting */
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
      /* Accumulate response body until '\n' or timeout */
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

        /* Give up early if nothing received */
        if (rx_len == 0 && i >= 4)
          break;
      }

      rx_buf[rx_len] = '\0';

      if (rx_len > 0) {
        sprintf(strTmp, STR_TASK_JETSON "Response [prompt %02u, %u bytes]:",
                prompt_id, rx_len);
        user_debug_msg(strTmp);
        csk_uart1_puts(rx_buf);
        csk_uart1_puts(STR_CRLF);

        /* Buffer inference result for SD logging by task_sdlog */
        {
          unsigned int copy_len = rx_len;
          unsigned int ci;
          if (copy_len > INFER_BUF_SIZE - 1)
            copy_len = INFER_BUF_SIZE - 1;
          for (ci = 0; ci < copy_len; ci++)
            jetson_infer_result.response[ci] = rx_buf[ci];
          jetson_infer_result.response[copy_len] = '\0';
          jetson_infer_result.response_len = copy_len;
          jetson_infer_result.prompt_id = prompt_id;
          jetson_infer_result.uptime_s = OSGetTicks() / 100UL;
          /* pending is set after TELEM is parsed (in JS_READ_TELEM) */
        }
      } else {
        sprintf(strTmp, STR_TASK_JETSON "No response body for prompt %02u.",
                prompt_id);
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
        /* Parse 8 CSV fields after "TELEM:" straight into the TX struct. */
        const char *p = match_buf + 6;
        static unsigned int llm_seq;

        /* Static metadata that doesn't change between inferences. */
        memcpy(jetson_last_telem.tx.callsign, "KO6IKZ", TLM_CALLSIGN_LEN);
        jetson_last_telem.tx.frame_type     = TLM_FRAME_LLM;
        jetson_last_telem.tx.seq_num        = llm_seq++;
        jetson_last_telem.tx.uptime_s       = OSGetTicks() / 100UL;
        jetson_last_telem.tx.prompt_id      = prompt_id;

        jetson_last_telem.tx.cpu_temp       = parse_int(&p);
        jetson_last_telem.tx.gpu_temp       = parse_int(&p);
        jetson_last_telem.tx.cpu_temp_peak  = parse_int(&p);
        jetson_last_telem.tx.gpu_temp_peak  = parse_int(&p);
        jetson_last_telem.tx.power_mw       = parse_uint(&p);
        jetson_last_telem.tx.ram_mb         = parse_uint(&p);
        jetson_last_telem.tx.infer_dur_s    = parse_uint(&p);
        jetson_last_telem.tx.model_load_s   = parse_uint(&p);

        /* Latency / status / hash: latency mirrors infer_dur_s for now,
           hash is populated once a CRC of the response is wired in. */
        jetson_last_telem.tx.llm_latency_ms = jetson_last_telem.tx.infer_dur_s * 1000U;
        jetson_last_telem.tx.llm_status     = TLM_LLM_MODEL_LOADED | TLM_LLM_INFERENCE_OK;
        jetson_last_telem.tx.llm_hash       = 0;

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

      /* Mark inference result ready for SD logging (even if TELEM timed out,
         the response text is still worth saving) */
      if (jetson_infer_result.response_len > 0)
        jetson_infer_result.pending = 1;

      state = JS_SHUTDOWN;
      break;


    /* ================================================================ */
    case JS_SHUTDOWN:
    /* ================================================================ */
      /* Acknowledge and request shutdown — polled TX */
      jetson_polled_puts("OK\r\n");
      OS_Delay(TICK_250MS);
      jetson_polled_puts("OFF\r\n");

      user_debug_msg(STR_TASK_JETSON "Sent OK + OFF, waiting 10 s for shutdown...");

      /* Wait 10 s for graceful shutdown */
      for (i = 0; i < SHUTDOWN_WAIT_LOOPS; i++)
        OS_Delay(SHUTDOWN_WAIT_TICKS);

      /* Cut power */
      jetson_hard_cut();
      user_debug_msg(STR_TASK_JETSON "IO.16 LOW, Jetson powered off.");

      /* Advance prompt ID for next run */
      prompt_id++;
      if (prompt_id >= NUM_PROMPTS)
        prompt_id = 0;

      sprintf(strTmp, STR_TASK_JETSON "Next prompt ID: %02u. Returning to IDLE.",
              prompt_id);
      user_debug_msg(strTmp);

      state = JS_IDLE;
      break;

    } /* switch */

  } /* while */

} /* task_jetson() */
