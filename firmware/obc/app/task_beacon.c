/******************************************************************************
KENSAT - Telemetry Beacon Task

Salvo cooperative task. Owns the Si4463, so the whole RF link lives here.
Each ~10 s cycle it transmits, subject to the TX gate (RF-silence hold and
ground-commanded inhibit):

    "BCN ..."   housekeeping beacon
    "LLM ..."   inference report, once per inference
    "TXT ..."   one chunk of the generated text, until the result is out
    "ACK ..."   reply to an uplink telecommand

then spends the rest of the cycle in RX polling for telecommands.

EPS (I2C) and Jetson (UART1) beacon fields are zero until those drivers
exist.

******************************************************************************/
#include "main.h"
#include "task_beacon.h"
#include "task_bmi088.h"
#include "task_jetson.h"
#include "task_si4463.h"
#include "telemetry.h"
#include "ax25.h"
#include "deploy.h"
#include "bootcount.h"
#include "commstate.h"
#include "task_sdlog.h"

#include <stdio.h>

/* AX.25 info field (one ASCII telemetry line). */
static char ax25_info[180];

#include "csk_hw.h"
#include "csk_io.h"
#include "csk_uart.h"

#include "salvo.h"

#include <string.h>


/******************************************************************************
 Module state

 seq/tx/rx are cumulative over the mission, not per boot: restored from SD at
 start and checkpointed after every beacon (sd_counters_load/save). 32-bit
 because 16 bits wraps in 7.6 days at 10 s. The flag word lives in program
 flash instead so it survives a missing card (see commstate.h).
******************************************************************************/
static unsigned long beacon_seq  = 0;
static unsigned char boot_count  = 0;

/* Persisted flag word (currently just the beacon inhibit). */
static unsigned int  comms_flags = 0;

/* Set once the counters have been restored. Don't save before then or a good
   checkpoint gets overwritten with zeros; the card may mount late. */
static unsigned char comms_loaded = 0;

/* Latest VCC_SYS reading in mV, read by task_sdlog. */
volatile unsigned int g_rail_3v3_mv = 0;

/* Salvo library (OSLIBRARY_CONFIG=OST) has OSBYTES_OF_DELAYS=1: OS_Delay()
   max is 255 ticks (2.55 s at 100 Hz). OS_Delay only works at task top level
   on this build - calling it from a helper or inside a loop corrupts the
   stack on resume. Longer waits are unrolled OS_Delay calls. */

/* Comms counters, persisted with beacon_seq. */
static unsigned long total_tx    = 0;
static unsigned long total_rx    = 0;

/* Hash of the uplinked prompt last handed to the payload, valid while its run
   is pending. Used to ACK a repeated PROMPT frame as "dup". */
static unsigned long up_prompt_hash  = 0;
static unsigned char up_prompt_live  = 0;   /* 1 = up_prompt_hash's run pending */

/* TXT downlink progress. The text stays in jetson_infer_result; task_jetson
   does not overwrite it while a run is pending. */
static unsigned char llm_txt_chunk = 0;  /* next chunk to send, 0-based */
static unsigned char llm_txt_pass  = 0;  /* completed passes over the set */


/******************************************************************************
 Uplink telecommand format

 One Si4463 packet-handler frame (preamble, sync 0x2DD4, length, payload,
 CRC-16). Payload is ASCII:

     "KO6IKZ KENSAT1 <VERB>[ <ARG>]"

 UPLINK_AUTH is a shared telecommand code; FCC 97.211(b) allows space
 telecommand codes to obscure meaning. Change it before flight and keep the
 flight value out of the repo.

 Verbs:
     BCN OFF    inhibit TX (persistent, no expiry; no reply - the carrier
                stopping is the confirmation)
     BCN ON     resume TX, ACKed
     PING       ACKed
     PROMPT <field>
                run an inference now on the uplinked prompt, ACKed with
                length and CRC-32
     RUN        run the next library prompt now
     RUN <id>   run library prompt <id> (0-19) now

 PROMPT frame: "KO6IKZ KENSAT1 PROMPT " + field[256]. The field is printable
 ASCII (0x20-0x7E) then NUL padding to exactly 256 bytes. Wrong length,
 control chars, non-NUL after padding, or empty -> NAK with the reason. The
 ~280 B frame needs the 2-byte-length frame and FIFO-draining RX in
 si4463_rx_poll(), and a ground preamble longer than the 400 ms poll period
 (ground-station/pluto_tx.py).

 ACK is "PROMPT <n>B h=<crc32>". The ground repeats the frame until ACKed; a
 repeat while the same run is pending gets "dup", a different prompt while
 busy gets "NAK PROMPT busy". After the run finishes the same frame runs
 again. The prompt is not persisted; after a reset the payload falls back to
 its on-board library.
******************************************************************************/
#define UPLINK_AUTH     "KENSAT1"
#define UPLINK_PREFIX   "KO6IKZ " UPLINK_AUTH " "
#define UPLINK_PREFIX_LEN  (sizeof(UPLINK_PREFIX) - 1)

#define UP_PROMPT_VERB      "PROMPT "
#define UP_PROMPT_VERB_LEN  (sizeof(UP_PROMPT_VERB) - 1)
#define UP_PROMPT_FIELD     256

#if UP_PROMPT_FIELD > JETSON_PROMPT_MAX
#error "PROMPT field is larger than the payload will accept (JETSON_PROMPT_MAX)"
#endif

/* Compile-time check that a PROMPT frame fits the RX buffer (XC16 has no
   _Static_assert). */
typedef char up_prompt_frame_fits_check[
    (UPLINK_PREFIX_LEN + UP_PROMPT_VERB_LEN + UP_PROMPT_FIELD
     <= SI4463_RX_MAX_PAYLOAD) ? 1 : -1];



/******************************************************************************
 read_vcc_3v3_mv()

 Measure VDD (= VCC_SYS) against the internal band gap with AVDD as Vref:
 VDD = V_BG * 1023 / adc. Returns mV, or 0 if the reading is implausibly low.
******************************************************************************/
#define VCC_3V3_VBG_MV   1200UL    /* nominal band gap, 1.16-1.24 V range */

static unsigned int adc_read_chan(unsigned char ch) {
    unsigned long sum = 0;
    unsigned char i;
    volatile unsigned int j;

    AD1CON1 = 0x0000;
    AD1CON1bits.SSRC = 0b111;      /* auto-convert after sample time */
    AD1CON1bits.ASAM = 0;           /* manual sample start */
    AD1CON1bits.FORM = 0;           /* unsigned integer */
    AD1CON2 = 0x0000;               /* AVDD/AVSS reference, single channel */
    AD1CON3 = 0x1F02;               /* SAMC=31 Tad, ADCS=2 (Tad = 3 Tcy) */
    AD1CHS  = (unsigned int)ch & 0x001F;

    AD1CON1bits.ADON = 1;
    for (j = 0; j < 200; j++) { __builtin_nop(); }   /* ADC settling */

    /* Discard the first conversion after ADON / a MUX change; the S/H cap
       isn't fully charged and it reads low. */
    AD1CON1bits.SAMP = 1;
    while (!AD1CON1bits.DONE) { /* spin */ }
    AD1CON1bits.DONE = 0;

    for (i = 0; i < 4; i++) {
        AD1CON1bits.SAMP = 1;
        while (!AD1CON1bits.DONE) { /* spin */ }
        sum += ADC1BUF0;
        AD1CON1bits.DONE = 0;
    }

    AD1CON1bits.ADON = 0;
    return (unsigned int)(sum >> 2);
}

static unsigned int read_vcc_3v3_mv(void) {
    unsigned int adc;
    volatile unsigned int j;

    /* Band gap is channel 25 on this silicon (found by sweeping 16-31; reads
       ~372 at VDD = 3.3 V). */
    ANCFGbits.VBGEN  = 1;
    for (j = 0; j < 2000; j++) { __builtin_nop(); }

    adc = adc_read_chan(25);

    if (adc < 100) {
        return 0;
    }

    return (unsigned int)((VCC_3V3_VBG_MV * 1023UL) / (unsigned long)adc);
}


/******************************************************************************
 beacon_read_3v3()

 Fresh VCC_SYS reading in mV, also cached in g_rail_3v3_mv so the startup
 banner has a value before the first beacon.
******************************************************************************/
unsigned int beacon_read_3v3(void) {
    unsigned int v3 = read_vcc_3v3_mv();
    g_rail_3v3_mv = v3;
    return v3;
}


/******************************************************************************
 collect_telemetry()

 Fill the beacon struct. Fields with no source yet stay zero.
******************************************************************************/
static void collect_telemetry(tlm_beacon_t *tlm) {

    memset(tlm, 0, sizeof(tlm_beacon_t));

    memcpy(tlm->callsign, "KO6IKZ", TLM_CALLSIGN_LEN);
    tlm->frame_type   = TLM_FRAME_BEACON;
    /* Truncated to 16 bits; the struct is internal only. */
    tlm->seq_num      = (unsigned int)beacon_seq;

    tlm->uptime_s     = OSGetTicks() / 100UL;
    tlm->reboot_count = boot_count;

    /* No battery-sense hardware and no ADC path to +5V_SYS, so 3V3 only. */
    tlm->vcc_3v3_mv = read_vcc_3v3_mv();
    g_rail_3v3_mv = tlm->vcc_3v3_mv;

    tlm->gyro_x_dps     = g_gyro_x_dps;
    tlm->gyro_y_dps     = g_gyro_y_dps;
    tlm->gyro_z_dps     = g_gyro_z_dps;
    tlm->tumble_mag_dps = g_tumble_mag_dps;

    tlm->last_rssi     = si4463_last_rssi();      /* raw; dBm ~= raw/2 - 134 */
    tlm->tx_count      = (unsigned int)total_tx;
    tlm->rx_count      = (unsigned int)total_rx;

    tlm->mission_mode  = TLM_MODE_NOMINAL;
}


/******************************************************************************
 beacon_rf_hold_cleared()

 1 once the post-power-on FLIGHT_HOLD_S RF-silence hold has been met. That is
 latched in flash (deploy_rf_unlock), so later boots transmit immediately.

 Not gated on FLIGHT_MODE, so a long bench session latches it too. Clear it
 with the 'y' console command (deploy_reset) or a reflash before flight.
 Deployment is not checked: there is no deploy sense line, and TX through a
 stowed antenna is better than nothing.
******************************************************************************/
static int beacon_rf_hold_cleared(unsigned long uptime_s) {
    if (deploy_rf_unlocked_get()) {
        return 1;
    }
    else if (uptime_s >= FLIGHT_HOLD_S) {
        deploy_rf_unlock();              /* persist once */
        return 1;
    }
    return 0;
}


/******************************************************************************
 beacon_compute_tx_allowed()

 1 if TX is allowed: RF hold cleared and no ground-commanded inhibit. Gates
 every transmission (BCN, LLM, TXT, ACK).
******************************************************************************/
static int beacon_compute_tx_allowed(unsigned long uptime_s) {
    if (comms_flags & COMMSTATE_FLAG_BEACON_INHIBIT) {
        return 0;
    }
    return beacon_rf_hold_cleared(uptime_s);
}


/******************************************************************************
 beacon_print_debug()

 One status line per cycle; rails and tumble too in verbose mode ('q'). The
 UART is busy-waited, so the extra lines cost ~0.3 s.
******************************************************************************/
static void beacon_print_debug(const tlm_beacon_t *tlm, int tx_allowed) {
    extern char strTmp[];
    if (tx_allowed)
        sprintf(strTmp, STR_TASK_BEACON "TX beacon #%lu (uptime %lus)",
                beacon_seq, tlm->uptime_s);
    else if (comms_flags & COMMSTATE_FLAG_BEACON_INHIBIT)
        sprintf(strTmp, STR_TASK_BEACON
                "RF silent (BCN OFF commanded from ground; still listening)");
    else
        sprintf(strTmp, STR_TASK_BEACON
                "RF silent (uptime %lus < %lus compliance hold)",
                tlm->uptime_s, (unsigned long)FLIGHT_HOLD_S);
    user_debug_msg(strTmp);

    if (!g_console_verbose) return;

    sprintf(strTmp,
            STR_TASK_BEACON "rails: 3V3=%umV",
            tlm->vcc_3v3_mv);
    user_debug_msg(strTmp);
    sprintf(strTmp,
            STR_TASK_BEACON "tumble: gx=%d gy=%d gz=%d |w|=%udps",
            tlm->gyro_x_dps, tlm->gyro_y_dps, tlm->gyro_z_dps,
            tlm->tumble_mag_dps);
    user_debug_msg(strTmp);
}


/******************************************************************************
 LLM result text downlink

 A result is up to 512 bytes and the info field holds 180, so it goes out as

     "TXT s=<seq> c=<k>/<m> h=<crc32> <up to 144 bytes of text>"

 s= and h= match the "LLM" frame for the same result. No ARQ: the ground
 groups by s=, orders by c= and checks the CRC-32 with h=.

 One chunk per beacon cycle, and the whole set sent LLM_TXT_PASSES times. A
 chunk is ~1.5 s of airtime at 1200 bps; sending all of them back to back
 would use most of the cycle, leave no RX window and push TX duty cycle past
 60%. Spacing them out (~40 s per pass) and repeating the set also makes it
 unlikely one fade or a late-acquiring station loses the same chunk twice.
******************************************************************************/
#define LLM_TXT_CHUNK     144   /* text bytes per TXT frame   */
#define LLM_TXT_PASSES    2     /* times the whole set is sent */


/******************************************************************************
 llm_txt_chunks()

 Number of TXT frames the pending result needs.
******************************************************************************/
static unsigned char llm_txt_chunks(void) {
    unsigned int len = jetson_infer_result.response_len;
    return (unsigned char)((len + (LLM_TXT_CHUNK - 1)) / LLM_TXT_CHUNK);
}


/******************************************************************************
 beacon_tx_llm_frame()

 Send the LLM inference report queued by task_jetson. Caller has checked
 tx_allowed and jetson_last_telem.pending_tx.
******************************************************************************/
static void beacon_tx_llm_frame(void) {
    extern char strTmp[];
    int ax25_len;

    jetson_last_telem.pending_tx = 0;
    sprintf(strTmp,
            STR_TASK_BEACON "TX LLM frame #%u (prompt %u, infer=%us)",
            jetson_last_telem.tx.seq_num,
            jetson_last_telem.tx.prompt_id,
            jetson_last_telem.tx.infer_dur_s);
    user_debug_msg(strTmp);

    ax25_len = sprintf(ax25_info,
        "LLM seq=%u pid=%u up=%lum "
        "cpu=%d gpu=%d pk=%d/%d mw=%u ram=%u "
        "ml=%us inf=%us lat=%ums st=%02X h=%08lX txt=%uB/%u",
        jetson_last_telem.tx.seq_num,
        jetson_last_telem.tx.prompt_id,
        jetson_last_telem.tx.uptime_s / 60UL,
        jetson_last_telem.tx.cpu_temp,
        jetson_last_telem.tx.gpu_temp,
        jetson_last_telem.tx.cpu_temp_peak,
        jetson_last_telem.tx.gpu_temp_peak,
        jetson_last_telem.tx.power_mw,
        jetson_last_telem.tx.ram_mb,
        jetson_last_telem.tx.model_load_s,
        jetson_last_telem.tx.infer_dur_s,
        jetson_last_telem.tx.llm_latency_ms,
        (unsigned int)jetson_last_telem.tx.llm_status,
        jetson_last_telem.tx.llm_hash,
        /* txt=<bytes>/<frames> of the TXT set for this result */
        jetson_infer_result.response_len,
        (unsigned int)llm_txt_chunks());

    if (ax25_len > 0 && ax25_len < (int)sizeof(ax25_info)) {
        si4463_tx_ax25((const unsigned char *)ax25_info,
                       (unsigned int)ax25_len);
    }
    total_tx++;
}


/******************************************************************************
 beacon_tx_llm_text()

 Send one TXT chunk of the pending result and advance. Clears
 jetson_infer_result.dl_pending after LLM_TXT_PASSES passes. Caller has
 checked tx_allowed and dl_pending.

 The text is memcpy'd, not passed through sprintf, since model output can
 contain '%'. task_jetson has already reduced it to printable ASCII.
******************************************************************************/
static void beacon_tx_llm_text(void) {
    extern char   strTmp[];
    static unsigned int llm_txt_seq = 0xFFFFu;   /* which result we are on */
    unsigned char chunks = llm_txt_chunks();
    unsigned int  off, n;
    int           hdr;

    if (chunks == 0) {
        jetson_infer_result.dl_pending = 0;
        return;
    }

    /* A newer result arrived mid-downlink: drop the old one and start over. */
    if (jetson_infer_result.seq != llm_txt_seq) {
        llm_txt_seq   = jetson_infer_result.seq;
        llm_txt_chunk = 0;
        llm_txt_pass  = 0;
    }

    if (llm_txt_chunk >= chunks) {
        llm_txt_chunk = 0;
    }

    off = (unsigned int)llm_txt_chunk * LLM_TXT_CHUNK;
    n   = jetson_infer_result.response_len - off;
    if (n > LLM_TXT_CHUNK) n = LLM_TXT_CHUNK;

    hdr = sprintf(ax25_info, "TXT s=%u c=%u/%u h=%08lX ",
                  jetson_infer_result.seq,
                  (unsigned int)(llm_txt_chunk + 1),
                  (unsigned int)chunks,
                  jetson_infer_result.hash);

    if (hdr > 0 && (unsigned int)hdr + n < sizeof(ax25_info)) {
        memcpy(ax25_info + hdr, jetson_infer_result.response + off, n);

        sprintf(strTmp, STR_TASK_BEACON
                "TX LLM text #%u chunk %u/%u (%uB, pass %u/%u)",
                jetson_infer_result.seq,
                (unsigned int)(llm_txt_chunk + 1), (unsigned int)chunks, n,
                (unsigned int)(llm_txt_pass + 1), (unsigned int)LLM_TXT_PASSES);
        user_debug_msg(strTmp);

        si4463_tx_ax25((const unsigned char *)ax25_info,
                       (unsigned int)hdr + n);
        total_tx++;
    }

    llm_txt_chunk++;
    if (llm_txt_chunk >= chunks) {
        llm_txt_chunk = 0;
        llm_txt_pass++;
        if (llm_txt_pass >= LLM_TXT_PASSES) {
            llm_txt_pass = 0;
            jetson_infer_result.dl_pending = 0;
            sprintf(strTmp, STR_TASK_BEACON
                    "LLM text #%u downlinked (%u chunks x %u passes).",
                    jetson_infer_result.seq, (unsigned int)chunks,
                    (unsigned int)LLM_TXT_PASSES);
            user_debug_msg(strTmp);
        }
    }
}


/******************************************************************************
 beacon_counters_restore()

 Restore the counters from SD. Sets comms_loaded once the card answers
 (loaded or empty). Retried every cycle because task_sdlog may mount the card
 late.
******************************************************************************/
static void beacon_counters_restore(void) {
    extern char   strTmp[];
    unsigned long seq = 0, tx = 0, rx = 0;

    switch (sd_counters_load(&seq, &tx, &rx)) {

    case SD_CTR_LOADED:
        beacon_seq   = seq;
        total_tx     = tx;
        total_rx     = rx;
        comms_loaded = 1;
        sprintf(strTmp, STR_TASK_BEACON
                "counters restored from SD: seq=%lu tx=%lu rx=%lu",
                beacon_seq, total_tx, total_rx);
        user_debug_msg(strTmp);
        break;

    case SD_CTR_EMPTY:
        comms_loaded = 1;
        user_debug_msg(STR_TASK_BEACON
                       "no counter checkpoint on SD; starting from zero.");
        break;

    default:                            /* SD_CTR_NO_CARD: retry next cycle */
        break;
    }
}


/******************************************************************************
 beacon_counters_save()

 Checkpoint the counters to SD after every beacon (small write to two
 alternating files). Retries the restore first if it hasn't succeeded yet.
******************************************************************************/
static void beacon_counters_save(void) {
    static unsigned char warned = 0;

    if (!comms_loaded) {
        beacon_counters_restore();
        return;
    }

    if (sd_counters_save(beacon_seq, total_tx, total_rx)) {
        warned = 0;
        return;
    }

    /* Warn once per outage. */
    if (!warned) {
        warned = 1;
        user_debug_msg(STR_TASK_BEACON
                       "SD counter checkpoint FAILED -- counters will reset "
                       "on the next reboot (bc= still identifies it).");
    }
}


/******************************************************************************
 beacon_uplink_ack()

 Send "ACK <what> ..." if TX is allowed, then return the radio to RX.
******************************************************************************/
static void beacon_uplink_ack(const char *what) {
    int len;

    if (!beacon_compute_tx_allowed(OSGetTicks() / 100UL)) {
        return;
    }

    si4463_rx_disable();

    len = sprintf(ax25_info, "ACK %s seq=%lu bc=%u",
                  what, beacon_seq, (unsigned int)boot_count);
    if (len > 0 && len < (int)sizeof(ax25_info)) {
        si4463_tx_ax25((const unsigned char *)ax25_info, (unsigned int)len);
        total_tx++;
    }

    si4463_rx_enable();
}


/******************************************************************************
 up_prompt_handle()

 Validate a PROMPT field (raw bytes after "PROMPT ") and, if good, start the
 inference. Every outcome is ACKed or NAKed so the ground can tell "declined"
 from "not heard".
******************************************************************************/
static void up_prompt_handle(const unsigned char *field, unsigned int flen) {
    extern char   strTmp[];
    static char   text[UP_PROMPT_FIELD + 1];
    char          detail[40];
    unsigned int  len = 0;
    unsigned int  i;
    unsigned long h;

    if (flen != UP_PROMPT_FIELD) {
        sprintf(detail, "NAK PROMPT len=%u", flen);
        goto nak;
    }

    while (len < UP_PROMPT_FIELD && field[len] != 0x00) {
        if (field[len] < 0x20 || field[len] > 0x7E) {
            sprintf(detail, "NAK PROMPT char@%u", len);
            goto nak;
        }
        text[len] = (char)field[len];
        len++;
    }
    for (i = len; i < UP_PROMPT_FIELD; i++) {
        if (field[i] != 0x00) {
            sprintf(detail, "NAK PROMPT pad@%u", i);
            goto nak;
        }
    }
    if (len == 0) {
        sprintf(detail, "NAK PROMPT empty");
        goto nak;
    }
    text[len] = '\0';
    h = jetson_crc32(text, len);

    if (jetson_request_prompt_text(text)) {
        up_prompt_hash = h;
        up_prompt_live = 1;
        sprintf(detail, "PROMPT %uB h=%08lX", len, h);
    }
    else if (up_prompt_live && h == up_prompt_hash) {
        /* Repeat of the prompt already queued or running. */
        sprintf(detail, "PROMPT %uB h=%08lX dup", len, h);
    }
    else {
        sprintf(detail, "NAK PROMPT busy h=%08lX", h);
    }

    sprintf(strTmp, STR_TASK_BEACON "uplink: %s \"%.60s\"", detail, text);
    user_debug_msg(strTmp);
    beacon_uplink_ack(detail);
    return;

nak:
    sprintf(strTmp, STR_TASK_BEACON "uplink: %s -- not run.", detail);
    user_debug_msg(strTmp);
    beacon_uplink_ack(detail);
}


/******************************************************************************
 beacon_uplink_dispatch()

 Parse and execute one received uplink frame (n bytes in up_buf, which must
 have room for a terminating NUL).
******************************************************************************/
static void beacon_uplink_dispatch(unsigned char *up_buf, unsigned int n) {
    extern char   strTmp[];
    const char   *cmd;
    unsigned int  i;

    /* PROMPT is checked on the raw bytes, before the NUL padding is
       sanitised to '.'. */
    if (n >= UPLINK_PREFIX_LEN + UP_PROMPT_VERB_LEN
        && strncmp((const char *)up_buf, UPLINK_PREFIX, UPLINK_PREFIX_LEN) == 0
        && strncmp((const char *)up_buf + UPLINK_PREFIX_LEN,
                   UP_PROMPT_VERB, UP_PROMPT_VERB_LEN) == 0) {
        sprintf(strTmp, STR_TASK_BEACON "RX uplink %uB rssi=%u: PROMPT",
                n, (unsigned int)si4463_last_rssi());
        user_debug_msg(strTmp);
        up_prompt_handle(up_buf + UPLINK_PREFIX_LEN + UP_PROMPT_VERB_LEN,
                         n - UPLINK_PREFIX_LEN - UP_PROMPT_VERB_LEN);
        return;
    }

    /* Replace non-printables before logging. */
    for (i = 0; i < n; i++) {
        if (up_buf[i] < 0x20 || up_buf[i] > 0x7E) {
            up_buf[i] = '.';
        }
    }
    up_buf[n] = '\0';

    /* %.80s: frame can be 300 B, strTmp is 160. */
    sprintf(strTmp, STR_TASK_BEACON "RX uplink %uB rssi=%u: \"%.80s\"",
            n, (unsigned int)si4463_last_rssi(),
            (const char *)up_buf);
    user_debug_msg(strTmp);

    if (n < UPLINK_PREFIX_LEN
        || strncmp((const char *)up_buf, UPLINK_PREFIX, UPLINK_PREFIX_LEN) != 0) {
        user_debug_msg(STR_TASK_BEACON
                       "uplink: wrong callsign/auth prefix -- ignored.");
        return;
    }

    cmd = (const char *)up_buf + UPLINK_PREFIX_LEN;

    /* BCN OFF: saved to flash immediately. No ACK; the ground sees the
       carrier stop. */
    if (strncmp(cmd, "BCN OFF", 7) == 0) {
        comms_flags |= COMMSTATE_FLAG_BEACON_INHIBIT;
        if (!commstate_flags_set(comms_flags)) {
            user_debug_msg(STR_TASK_BEACON
                           "uplink: BCN OFF applied but FLASH SAVE FAILED "
                           "-- inhibit will not survive a reset!");
        } else {
            user_debug_msg(STR_TASK_BEACON
                           "uplink: BCN OFF -- transmitter inhibited "
                           "(persistent). Still listening.");
        }
        si4463_rx_enable();
        return;
    }

    if (strncmp(cmd, "BCN ON", 6) == 0) {
        comms_flags &= (unsigned int)~COMMSTATE_FLAG_BEACON_INHIBIT;
        if (!commstate_flags_set(comms_flags)) {
            user_debug_msg(STR_TASK_BEACON
                           "uplink: BCN ON applied but FLASH SAVE FAILED "
                           "-- inhibit may return after a reset!");
        } else {
            user_debug_msg(STR_TASK_BEACON
                           "uplink: BCN ON -- beacon re-enabled.");
        }
        beacon_uplink_ack("BCNON");
        return;
    }

    if (strncmp(cmd, "PING", 4) == 0) {
        user_debug_msg(STR_TASK_BEACON "uplink: PING.");
        beacon_uplink_ack("PING");
        return;
    }

    /* RUN [<id>]: library prompt <id>, or the next one. NAKed if the payload
       is busy. */
    if (strncmp(cmd, "RUN", 3) == 0) {
        const char *a = cmd + 3;
        char        detail[32];

        while (*a == ' ') a++;

        if (*a >= '0' && *a <= '9') {
            unsigned int id = 0;
            while (*a >= '0' && *a <= '9') {
                id = id * 10u + (unsigned int)(*a - '0');
                a++;
            }
            if (jetson_request_prompt_id((unsigned char)id)) {
                sprintf(detail, "RUN pid=%u", id);
            } else {
                sprintf(detail, "NAK RUN pid=%u", id);
            }
        }
        else {
            if (jetson_request_prompt_id(JETSON_PROMPT_NEXT)) {
                sprintf(detail, "RUN next");
            } else {
                sprintf(detail, "NAK RUN next");
            }
        }

        sprintf(strTmp, STR_TASK_BEACON "uplink: %s", detail);
        user_debug_msg(strTmp);
        beacon_uplink_ack(detail);
        return;
    }

    sprintf(strTmp, STR_TASK_BEACON "uplink: unknown verb \"%.40s\" -- ignored.",
            cmd);
    user_debug_msg(strTmp);
}


/******************************************************************************
 beacon_uplink_service()

 Poll the radio once and dispatch anything received. Called repeatedly
 during the listening window.
******************************************************************************/
static void beacon_uplink_service(void) {
    static unsigned char up_buf[SI4463_RX_MAX_PAYLOAD + 1];
    unsigned int n;

    /* PROMPT dup detection only lasts as long as that run. */
    if (up_prompt_live && !jetson_busy()) {
        up_prompt_live = 0;
    }

    n = si4463_rx_poll(up_buf, SI4463_RX_MAX_PAYLOAD);
    if (n == 0) {
        return;
    }

    /* Count every CRC-valid frame, addressed to us or not. */
    total_rx++;

    beacon_uplink_dispatch(up_buf, n);
}


/******************************************************************************
 beacon_uplink_inject()

 Bench aid ('u' console command): run a string through the uplink dispatcher
 as if received over the air. Safe to call from task_cmd since nothing it
 reaches yields.
******************************************************************************/
/* Shared by both injectors; task_cmd calls them one at a time. */
static unsigned char inj_buf[SI4463_RX_MAX_PAYLOAD + 1];

void beacon_uplink_inject(const char *payload) {
    extern char   strTmp[];
    unsigned int  n = 0;

    while (payload[n] != '\0' && n < SI4463_RX_MAX_PAYLOAD) {
        inj_buf[n] = (unsigned char)payload[n];
        n++;
    }

    sprintf(strTmp, STR_TASK_BEACON "uplink: INJECTING \"%.80s\" (bench only).",
            payload);
    user_debug_msg(strTmp);

    beacon_uplink_dispatch(inj_buf, n);
}


/******************************************************************************
 beacon_uplink_inject_prompt()

 Bench aid ('n' console command): build a full PROMPT frame around text,
 NUL-padded to 256 bytes like the ground does, and dispatch it. Text longer
 than 256 is truncated.
******************************************************************************/
void beacon_uplink_inject_prompt(const char *text) {
    extern char   strTmp[];
    unsigned int  n = 0;
    unsigned int  i;

    for (i = 0; i < UPLINK_PREFIX_LEN; i++)  inj_buf[n++] = UPLINK_PREFIX[i];
    for (i = 0; i < UP_PROMPT_VERB_LEN; i++) inj_buf[n++] = UP_PROMPT_VERB[i];
    for (i = 0; i < UP_PROMPT_FIELD; i++) {
        inj_buf[n++] = (text[0] != '\0') ? (unsigned char)*text++ : 0x00;
    }

    sprintf(strTmp, STR_TASK_BEACON "uplink: INJECTING PROMPT frame %uB (bench only).",
            n);
    user_debug_msg(strTmp);

    beacon_uplink_dispatch(inj_buf, n);
}


/******************************************************************************
 beacon_inhibited()

 1 if the transmitter is under the persistent ground-commanded inhibit.
******************************************************************************/
unsigned int beacon_inhibited(void) {
    return (comms_flags & COMMSTATE_FLAG_BEACON_INHIBIT) ? 1u : 0u;
}


/******************************************************************************
 task_beacon()

 Salvo task entry point.
******************************************************************************/
void task_beacon(void) {
    tlm_beacon_t tlm;
    int           ax25_len;
    int           tx_allowed;

    /* Burn-wire FET gates IO.24 (RF1) and IO.25 (RF0) low at boot. Only the
       deploy block below drives them high. */
    _TRISF1 = 0;            // RF1 (IO.24) -> output
    _TRISF0 = 0;            // RF0 (IO.25) -> output
    csk_io24_low();
    csk_io25_low();

#if DEPLOY_TEST_MODE
    /* Bench burn test: 10 s after boot, drive both gates high for 10 s, once
       per boot, ignoring the DEPLOYED flag. Must be 0 for flight. Energises
       the nichrome - disconnect the burn wires or fit a dummy load. */
    user_debug_msg(STR_TASK_BEACON "DEPLOY_TEST_MODE: burn-wire pulse in 10 s.");

    __builtin_clrwdt();
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);

    user_debug_msg(STR_TASK_BEACON "DEPLOY_TEST_MODE: IO.24+IO.25 HIGH (10 s burn).");
    _TRISF1 = 0;
    _TRISF0 = 0;
    csk_io24_high();
    csk_io25_high();

    __builtin_clrwdt();
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);

    csk_io24_low();
    csk_io25_low();
    user_debug_msg(STR_TASK_BEACON "DEPLOY_TEST_MODE: burn complete, IO.24+IO.25 LOW.");
#endif /* DEPLOY_TEST_MODE */

    /* Count the boot only after 1 s alive. A cold ICSP read runs init() for a
       few hundred ms before the programmer halts the core, so counting in
       init() made every memory read look like a boot. bootcount_store()
       masks interrupts around the NVM write. */
    OS_Delay(100);                       /* 1 s */
    boot_count = (unsigned char)bootcount_increment();
    sprintf(strTmp, STR_TASK_BEACON "Boot count = %u (recorded after 1 s alive).",
            (unsigned int)boot_count);
    user_debug_msg(strTmp);

    /* Rest of the ~5 s startup settle. */
    OS_Delay(200);
    OS_Delay(200);

    user_debug_msg(STR_TASK_BEACON "Beacon task started.");

    /* seq/tx/rx from SD, inhibit flag from program flash. */
    comms_flags = commstate_flags_get();
    beacon_counters_restore();

    sprintf(strTmp, STR_TASK_BEACON
            "comms state: seq=%lu tx=%lu rx=%lu flags=0x%04X%s",
            beacon_seq, total_tx, total_rx, comms_flags,
            (comms_flags & COMMSTATE_FLAG_BEACON_INHIBIT)
                ? "  [BEACON INHIBITED BY GROUND COMMAND]" : "");
    user_debug_msg(strTmp);

    /* task_si4463_rx is disabled, so this task is the only radio owner.
       Leaves the chip in READY with the TX path selected. */
    si4463_init_beacon();

    if (!jetson_last_telem.valid) {
        if (sd_read_last_telem(&jetson_last_telem)) {
            user_debug_msg(STR_TASK_BEACON "Restored telemetry from SD.");
        }
    }

    while (1) {
        __builtin_clrwdt();

        collect_telemetry(&tlm);
        tx_allowed = beacon_compute_tx_allowed(tlm.uptime_s);
        beacon_print_debug(&tlm, tx_allowed);

        /* Housekeeping beacon as an AX.25 UI frame (1k2 G3RUH, decodable by
           SatNOGS). Kept short for 1200 bps: the callsign is already in the
           AX.25 header, |w| is derivable from the axes, and tx= added
           nothing. bc= (boot count) and rst= (reset cause) tell a reboot
           apart from a missed pass. Uptime in minutes. */
        ax25_len = sprintf(ax25_info,
            "BCN seq=%lu bc=%u rst=%02X up=%lum "
            "gx=%d gy=%d gz=%d rx=%lu",
            beacon_seq,
            (unsigned int)boot_count,
            (unsigned int)reset_cause_code(),
            tlm.uptime_s / 60UL,
            tlm.gyro_x_dps, tlm.gyro_y_dps, tlm.gyro_z_dps,
            total_rx);

        if (tx_allowed && ax25_len > 0 && ax25_len < (int)sizeof(ax25_info)) {
            si4463_rx_disable();
            si4463_tx_ax25((const unsigned char *)ax25_info,
                           (unsigned int)ax25_len);
            beacon_seq++;
            total_tx++;
        }

        if (tx_allowed && jetson_last_telem.pending_tx) {
            si4463_rx_disable();
            beacon_tx_llm_frame();
        }

        /* One TXT chunk per cycle (see the LLM text downlink notes). */
        if (tx_allowed && jetson_infer_result.dl_pending) {
            si4463_rx_disable();
            beacon_tx_llm_text();
        }

        beacon_counters_save();

#if (FLIGHT_MODE && DEPLOY_ARMED)
        /* One-shot antenna deployment, built only with FLIGHT_MODE and
           DEPLOY_ARMED both set (main.h).

           Fires once uptime passes FLIGHT_HOLD_S, which resets each boot.
           Must stay after the TX block: on the first qualifying cycle the
           beacon finishes before the nichrome is energised, so the PA and
           burn wires never draw at the same time.

           State in program flash (deploy.h): skip if DEPLOYED is set or
           attempts >= DEPLOY_MAX_ATTEMPTS. The attempt is counted before the
           burn so a brownout mid-burn still counts; DEPLOYED is set only
           after the burn completes. */
        if (!deploy_get()
            && deploy_attempts_get() < DEPLOY_MAX_ATTEMPTS
            && tlm.uptime_s >= FLIGHT_HOLD_S) {

            unsigned int n = deploy_attempt_inc();   /* persist before burn */
            sprintf(strTmp, STR_TASK_BEACON
                    "Deploy attempt %u/%u: nichrome IO.24+IO.25 HIGH.",
                    n, DEPLOY_MAX_ATTEMPTS);
            user_debug_msg(strTmp);

            /* Two burn wires in parallel for redundancy. */
            _TRISF1 = 0;
            _TRISF0 = 0;
            csk_io24_high();
            csk_io25_high();

            /* 10 s burn */
            OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);

            csk_io24_low();
            csk_io25_low();

            deploy_set();
            user_debug_msg(STR_TASK_BEACON
                           "Nichrome OFF, antenna deployed (flag set).");
        }
#endif /* FLIGHT_MODE && DEPLOY_ARMED */

        /* Listening window: half-duplex, so sit in RX for the rest of the
           cycle and poll every 400 ms (unrolled, see the OS_Delay note at the
           top). After a packet arrives the chip holds it in READY until
           polled, so the poll period is also the deaf time. */
        si4463_rx_enable();

        OS_Delay(40); beacon_uplink_service();   /*  1 */
        OS_Delay(40); beacon_uplink_service();   /*  2 */
        OS_Delay(40); beacon_uplink_service();   /*  3 */
        OS_Delay(40); beacon_uplink_service();   /*  4 */
        OS_Delay(40); beacon_uplink_service();   /*  5 */
        OS_Delay(40); beacon_uplink_service();   /*  6 */
        OS_Delay(40); beacon_uplink_service();   /*  7 */
        OS_Delay(40); beacon_uplink_service();   /*  8 */
        OS_Delay(40); beacon_uplink_service();   /*  9 */
        OS_Delay(40); beacon_uplink_service();   /* 10 */
        OS_Delay(40); beacon_uplink_service();   /* 11 */
        OS_Delay(40); beacon_uplink_service();   /* 12 */
        OS_Delay(40); beacon_uplink_service();   /* 13 */
        OS_Delay(40); beacon_uplink_service();   /* 14 */
        OS_Delay(40); beacon_uplink_service();   /* 15 */
        OS_Delay(40); beacon_uplink_service();   /* 16 */
        OS_Delay(40); beacon_uplink_service();   /* 17 */
        OS_Delay(40); beacon_uplink_service();   /* 18 */
        OS_Delay(40); beacon_uplink_service();   /* 19 */
        OS_Delay(40); beacon_uplink_service();   /* 20 */
        OS_Delay(40); beacon_uplink_service();   /* 21 */
        OS_Delay(40); beacon_uplink_service();   /* 22 */
        OS_Delay(40); beacon_uplink_service();   /* 23 */
        OS_Delay(40); beacon_uplink_service();   /* 24 */
        OS_Delay(40); beacon_uplink_service();   /* 25: 25 x 40 = 10 s */
    }
}
