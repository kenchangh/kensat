/******************************************************************************
KENSAT - Telemetry Beacon Task

Salvo cooperative task. Wakes every BEACON_INTERVAL ticks (default 10s),
collects available housekeeping data, serializes into tlm_beacon_t, and
transmits via Si4463 GFSK TX.

This task owns the radio, so it is also where the rest of the link lives.
Every cycle it sends, in this order and all subject to the same TX gate
(RF-silence hold + commanded inhibit):

    "BCN ..."   housekeeping beacon
    "LLM ..."   how the last inference went, once per inference
    "TXT ..."   one chunk of the generated text, until the result is out
    "ACK ..."   acknowledgement of an uplink telecommand

and then spends the rest of the cycle in RX, polling for uplink
telecommands -- including the chunks of an uplinked LLM prompt.

EPS (I2C) and Jetson (UART1) fields are placeholder zeros until those
subsystem drivers are implemented.

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

/* Scratch for building the AX.25 info field (ASCII telemetry line).
   Sized to comfortably fit one line of "KEY=val KEY=val ..." for the
   beacon or LLM frame. */
static char ax25_info[180];

#include "csk_hw.h"
#include "csk_io.h"
#include "csk_uart.h"

#include "salvo.h"

#include <string.h>


/******************************************************************************
 Module state

 The counters below are CUMULATIVE OVER THE LIFE OF THE MISSION, not per
 boot.  They are restored from the SD card at task start and checkpointed
 back after every beacon (sd_counters_load/save), so the sequence number the
 ground sees is exactly the one last transmitted -- no rounding to a
 checkpoint interval, and no chance of a number being reused after a reset.

 32-bit because a 16-bit seq would wrap in 7.6 days at a 10 s cadence.

 Only the flag word lives in program flash: it changes a handful of times in
 the whole mission and must survive a missing or dead card.  See commstate.h.
******************************************************************************/
static unsigned long beacon_seq  = 0;
static unsigned char boot_count  = 0;

/* Persisted flag word (currently just the commanded beacon inhibit). */
static unsigned int  comms_flags = 0;

/* 0 until the counters have been restored from the card.  Until then we must
   not save, or a good checkpoint would be overwritten with zeros -- which
   matters because the card may mount late (see the hot-insert retry path in
   task_sdlog) or not at all. */
static unsigned char comms_loaded = 0;

/* Latest rail reading, updated each beacon cycle. task_sdlog reads this.
   (No VBATT: this board has no battery-sense hardware.) */
volatile unsigned int g_rail_3v3_mv = 0;

/* Salvo prebuilt library (OSLIBRARY_CONFIG=OST) uses OSBYTES_OF_DELAYS=1, so
   OS_Delay() takes uint8_t (max 255 ticks ≈ 2.55s at 100 Hz). For longer
   delays, issue multiple OS_Delay(200) calls *at task top level*. Yielding
   from inside a helper or a while-loop with a local counter corrupts the
   stack on resume on this build (chip resets or the task hangs); the only
   reliably-working pattern observed is unrolled top-level OS_Delays. */

/* Comms counters -- cumulative, persisted alongside beacon_seq. */
static unsigned long total_tx    = 0;
static unsigned long total_rx    = 0;

/* Uplinked-prompt assembly (see "WHY THE PROMPT IS CHUNKED" below).
   6 chunks x 40 chars = 240 = JETSON_PROMPT_MAX.  40 is what fits in one
   uplink frame: 63 B payload - 15 B command prefix - 7 B "PR k/m " = 41. */
#define UP_CHUNK_MAX      40
#define UP_CHUNKS         6
static char          up_chunk[UP_CHUNKS][UP_CHUNK_MAX + 1];
static unsigned char up_chunk_len[UP_CHUNKS];
static unsigned char up_have     = 0;   /* bitmap, bit k-1 = chunk k held */
static unsigned char up_total    = 0;   /* m from the last chunk seen, 0 = idle */

/* Text-downlink progress for the result currently being sent out.  The
   result itself stays in jetson_infer_result (512 B); copying it here would
   double the cost on a 16 KB part for no benefit, since task_jetson cannot
   overwrite it while a run is in progress. */
static unsigned char llm_txt_chunk = 0;  /* next chunk to send, 0-based */
static unsigned char llm_txt_pass  = 0;  /* completed passes over the set */


/******************************************************************************
 Uplink telecommand format

 Payload of one Si4463 packet-handler frame (preamble + sync 0x2DD4 + length
 + payload + CRC-16), ASCII:

     "KO6IKZ KENSAT1 <VERB>[ <ARG>]"

 The leading callsign keeps us from acting on anyone else's traffic, and the
 token after it is a shared telecommand code.  FCC Part 97.211(b) expressly
 permits a space telecommand station to use codes that obscure the meaning of
 telecommand messages, so a non-public token here is fine -- and the CRC alone
 is not remotely enough of a gate on a command that can silence the
 spacecraft.  Change UPLINK_AUTH before flight and keep it off the repo if you
 want it to be worth anything.

 Verbs:
     BCN OFF    inhibit the beacon (persistent, survives reset; no reply --
                the ground's confirmation is that the carrier stops)
     BCN ON     resume beaconing, acknowledged
     PING       liveness check, acknowledged
     PR <k>/<m> <text>
                chunk k of m of an uplinked LLM prompt, acknowledged with
                the set of chunks held so far
     PR?        report prompt-assembly state, acknowledged
     PRX        discard the partially assembled prompt, acknowledged
     RUN        run an inference now: on the assembled uplinked prompt if one
                is complete, else on the next prompt in the Jetson's library
     RUN <id>   run library prompt <id> (0-19) now

 There is deliberately NO auto-expiry on BCN OFF.  A licensing regime that
 requires a commandable shutdown expects the shutdown to hold until it is
 commanded otherwise, not to time out on its own.

 WHY THE PROMPT IS CHUNKED
 -------------------------
 The uplink is one Si4463 packet-handler frame, payload capped at
 SI4463_RX_MAX_PAYLOAD = 63 B, of which the prefix above eats 15.  A prompt
 worth sending does not fit, so it arrives as numbered pieces that are
 reassembled here and only handed to the payload once every piece is in.

 Chunks are stored in per-slot buffers rather than at a fixed stride in one
 flat buffer, so they may arrive in any order and may be of any length: the
 operator cannot be relied on to pad them, and an uplink pass is far too
 short to care about which repeat of which chunk got through first.  Each
 chunk is acknowledged with a bitmap of what is held, which is what lets the
 ground re-send only the missing pieces instead of starting over.

 Nothing about the assembly is persisted.  A reset drops the partial prompt
 and the spacecraft falls back to its own prompt library -- the mission keeps
 producing results without any uplink at all, which is the whole point of
 keeping the library on board.
******************************************************************************/
#define UPLINK_AUTH     "KENSAT1"
#define UPLINK_PREFIX   "KO6IKZ " UPLINK_AUTH " "
#define UPLINK_PREFIX_LEN  (sizeof(UPLINK_PREFIX) - 1)

/* Prompt assembly geometry: UP_CHUNKS / UP_CHUNK_MAX, declared with the
   assembly buffers further up this file. */



/******************************************************************************
 read_vcc_3v3_mv()

 Measures the PIC24 VDD rail (= VCC_SYS) using the internal band gap
 reference. No external hardware required.

 Method: enable internal V_BG (~1.2 V) via ANCFG.VBGEN, configure ADC1 with
 AVDD/AVSS as Vref and select the V_BG channel (CH0SA = 0b11110 on the
 PIC24FJ GB family). Then VDD = V_BG * 1023 / adc, since adc/1023 = V_BG/VDD.
 Averages 4 samples.

 Returns mV. 0 if the reading is implausibly low (likely wrong channel /
 band gap not yet on).
******************************************************************************/
#define VCC_3V3_VBG_MV   1200UL    /* nominal band gap, 1.16-1.24 V range */

static unsigned int adc_read_chan(unsigned char ch) {
    unsigned long sum = 0;
    unsigned char i;
    volatile unsigned int j;

    /* Configure ADC1 (held off until ADON=1). */
    AD1CON1 = 0x0000;
    AD1CON1bits.SSRC = 0b111;      /* internal counter ends sample → auto conv */
    AD1CON1bits.ASAM = 0;           /* manual sample start                      */
    AD1CON1bits.FORM = 0;           /* unsigned integer output                  */
    AD1CON2 = 0x0000;               /* AVDD/AVSS reference, single channel      */
    AD1CON3 = 0x1F02;               /* SAMC=31 Tad, ADCS=2  →  Tad ≈ 3·Tcy      */
    AD1CHS  = (unsigned int)ch & 0x001F;

    AD1CON1bits.ADON = 1;
    for (j = 0; j < 200; j++) { __builtin_nop(); }   /* ADC settling */

    /* Throwaway conversion: the first sample after ADON / an input-MUX change
       (e.g. switching from the internal band gap on CH25 to an external pin)
       does not fully charge the S/H cap from the new source, so its count
       reads low. Discard it before averaging the real samples. */
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

    /* Enable internal band gap. Empirically on this PIC24FJ256GB210 silicon,
       V_BG (1.2 V nominal) is selected by CH0SA = 25 (0b11001) — verified by
       sweeping channels 16-31 and finding the one that read ≈ 372 (= 1.2/3.3
       * 1023) at VDD = 3.3 V. */
    ANCFGbits.VBGEN  = 1;
    for (j = 0; j < 2000; j++) { __builtin_nop(); }

    adc = adc_read_chan(25);

    if (adc < 100) {
        return 0;                   /* implausibly low → don't trust it */
    }

    return (unsigned int)((VCC_3V3_VBG_MV * 1023UL) / (unsigned long)adc);
}


/******************************************************************************
 beacon_read_3v3()

 Take a fresh VCC_SYS (3V3) reading via the internal band gap and cache it in
 g_rail_3v3_mv, so the startup banner has a value before the first beacon cycle
 runs. Returns the rail in mV.

 (This board has no battery-sense hardware, so there is no VBATT/SoC read.)
******************************************************************************/
unsigned int beacon_read_3v3(void) {
    unsigned int v3 = read_vcc_3v3_mv();
    g_rail_3v3_mv = v3;
    return v3;
}


/******************************************************************************
 collect_telemetry()

 Fill the small beacon packet. All fields with no real source yet are
 zeroed via the initial memset; only the ones we actually have are set.
******************************************************************************/
static void collect_telemetry(tlm_beacon_t *tlm) {

    memset(tlm, 0, sizeof(tlm_beacon_t));

    /* Identification */
    memcpy(tlm->callsign, "KO6IKZ", TLM_CALLSIGN_LEN);
    tlm->frame_type   = TLM_FRAME_BEACON;
    /* The wire struct predates the 32-bit counters; it is only used
       internally (the downlink is the ASCII AX.25 line built below), so
       truncation here is harmless. */
    tlm->seq_num      = (unsigned int)beacon_seq;

    /* OBC */
    tlm->uptime_s     = OSGetTicks() / 100UL;
    tlm->reboot_count = boot_count;

    /* Power rails. VCC_SYS via internal band gap (real). This board has no
       battery-sense hardware, so VBATT/SoC is not read. (+5V_SYS also has no
       ADC path, so it is not reported.) */
    tlm->vcc_3v3_mv = read_vcc_3v3_mv();

    /* Cache for task_sdlog and debug consumers. */
    g_rail_3v3_mv = tlm->vcc_3v3_mv;

    /* Tumbling: per-axis angular rate + precomputed magnitude. */
    tlm->gyro_x_dps     = g_gyro_x_dps;
    tlm->gyro_y_dps     = g_gyro_y_dps;
    tlm->gyro_z_dps     = g_gyro_z_dps;
    tlm->tumble_mag_dps = g_tumble_mag_dps;

    /* Comms */
    tlm->last_rssi     = si4463_last_rssi();      /* raw; dBm ~= raw/2 - 134 */
    tlm->tx_count      = (unsigned int)total_tx;
    tlm->rx_count      = (unsigned int)total_rx;

    /* Operational */
    tlm->mission_mode  = TLM_MODE_NOMINAL;
}


/******************************************************************************
 beacon_rf_hold_cleared()

 RF-silence compliance gate: stay silent until the initial FLIGHT_HOLD_S
 post-power-on hold has been satisfied ONCE.  That fact is persisted
 (deploy_rf_unlock), so after the first time, later reboots transmit
 immediately without re-waiting.

 This is NOT gated on FLIGHT_MODE -- the hold runs on the bench too, so the
 bench exercises the exact timing that flies.  Consequence worth knowing: a
 bench session longer than the hold latches rf_unlocked in flash, and a board
 that ships with it latched will transmit from its very first orbital loop
 with no wait.  Clear it with the 'y' console command (deploy_reset) as the
 last step before flight; a reflash of the .hex clears it too.

 Deployment is deliberately NOT consulted here: it is GPIO, not RF, and it
 proceeds during this hold.  The beacon goes out whether or not the burn wires
 ever fired -- there is no deploy sense line to wait on, and a signal through
 a stowed antenna still beats silence.

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
static int beacon_rf_hold_cleared(unsigned long uptime_s) {
    if (deploy_rf_unlocked_get()) {
        return 1;                        /* hold already satisfied earlier */
    }
    else if (uptime_s >= FLIGHT_HOLD_S) {
        deploy_rf_unlock();              /* satisfied now -- persist once */
        return 1;
    }
    return 0;                            /* still in the one-time hold */
}


/******************************************************************************
 beacon_compute_tx_allowed()

 Everything that has to be true before ANY carrier goes out: the one-time
 power-on RF-silence hold must be cleared, and the beacon must not be under a
 commanded inhibit from the ground (BCN OFF).  The inhibit is checked first
 and gates every transmitter path -- beacon frames, LLM frames and uplink
 acknowledgements alike -- because a "stop transmitting" command that only
 stopped some of them would not satisfy anybody's licence.

 Contains no OS_Delay, so it is safe as a helper. Returns 1 if TX is allowed.
******************************************************************************/
static int beacon_compute_tx_allowed(unsigned long uptime_s) {
    if (comms_flags & COMMSTATE_FLAG_BEACON_INHIBIT) {
        return 0;                        /* commanded silent from the ground */
    }
    return beacon_rf_hold_cleared(uptime_s);
}


/******************************************************************************
 beacon_print_debug()

 Print the beacon sequence + rail + tumble readings to the debug console.
 Contains no OS_Delay, so it is safe as a helper.

 Terse mode (the default) prints only the first line -- the rails and tumble
 figures are already in TELEM.CSV and in the beacon frame itself, so paying
 ~0.3 s of busy-waited UART for them every cycle buys nothing.  'q' restores
 the full block.
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
 LLM inference-result text downlink

 The housekeeping frame below reports how an inference WENT; this is how the
 generated TEXT -- the actual mission product -- gets to the ground.

 A result is up to INFER_BUF_SIZE (512) bytes and one AX.25 info field holds
 180, so the text goes out as a numbered series of frames:

     "TXT s=<seq> c=<k>/<m> h=<crc32> <144 bytes of text>"

 s= and h= match the "LLM seq=... h=..." housekeeping frame for the same
 result, so chunks can be grouped and a reassembled result checked against
 the CRC-32 the spacecraft computed over exactly the bytes it transmitted.
 There is no ARQ on this link and no request-for-repeat, so those two fields
 are the entire reassembly protocol: group by s=, index by c=, verify h=.

 ONE CHUNK PER BEACON CYCLE, and the whole set LLM_TXT_PASSES times.

 Not back-to-back, for two reasons.  A 144 B chunk encodes to ~218 B, which
 at 1200 bps is ~1.5 s of airtime; sending all four in one cycle would eat
 6 s of a 10 s cycle, leaving almost no listening window and pushing the
 transmitter duty cycle past 60%.  Interleaving them one per cycle holds
 per-cycle airtime at beacon + one chunk (~2.1 s of 10 s) and keeps the
 uplink alive throughout.  It also spreads the result over ~40 s of sky, so a
 single fade cannot take out the whole thing the way it could a burst.

 The repeat pass exists for the same reason: a chunk lost to a fade is lost
 for good, and a station that acquires us mid-pass has missed whatever went
 out before it locked.  Two passes separated by ~40 s make a correlated loss
 of both copies of the same chunk much less likely, at a cost of ~80 s of
 result downlink per inference -- cheap against an inference the spacecraft
 only runs once a day.
******************************************************************************/
#define LLM_TXT_CHUNK     144   /* text bytes per TXT frame   */
#define LLM_TXT_PASSES    2     /* times the whole set is sent */


/******************************************************************************
 llm_txt_chunks()

 How many TXT frames the pending result needs.  Contains no OS_Delay, so it
 is safe as a helper.
******************************************************************************/
static unsigned char llm_txt_chunks(void) {
    unsigned int len = jetson_infer_result.response_len;
    return (unsigned char)((len + (LLM_TXT_CHUNK - 1)) / LLM_TXT_CHUNK);
}


/******************************************************************************
 beacon_tx_llm_frame()

 If task_jetson queued an LLM-inference frame, build and TX it now.  Only one
 Si4463-owning task (task_beacon), so no locking needed.  The caller has
 already confirmed tx_allowed && jetson_last_telem.pending_tx before calling.

 Contains no OS_Delay, so it is safe as a helper.
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
        /* txt=<bytes>/<frames>: how much generated text belongs to this
           result and how many TXT frames carry it, so a station that hears
           this frame knows what a complete reassembly looks like. */
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

 Transmit ONE chunk of the pending inference result, then advance.  Clears
 jetson_infer_result.dl_pending once LLM_TXT_PASSES complete passes have gone
 out.  The caller has already confirmed tx_allowed && dl_pending.

 Note the payload is memcpy'd in rather than sprintf'd: this is text a
 language model generated, so it can perfectly well contain a '%', and
 handing it to sprintf as part of a format string would make the frame
 contents decide how the frame is built.  The header is formatted, the text
 is copied -- never the other way round.  (task_jetson has already reduced
 the text to printable ASCII, so nothing here has to re-sanitise it.)

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
static void beacon_tx_llm_text(void) {
    extern char   strTmp[];
    static unsigned int llm_txt_seq = 0xFFFFu;   /* which result we are on */
    unsigned char chunks = llm_txt_chunks();
    unsigned int  off, n;
    int           hdr;

    if (chunks == 0) {                  /* nothing to send after all */
        jetson_infer_result.dl_pending = 0;
        return;
    }

    /* A newer inference finished while this one was still going out: abandon
       the remainder and start the new result from chunk 1.  The fresh result
       is the more interesting one, and sending the tail of a result the
       ground can no longer complete is wasted airtime. */
    if (jetson_infer_result.seq != llm_txt_seq) {
        llm_txt_seq   = jetson_infer_result.seq;
        llm_txt_chunk = 0;
        llm_txt_pass  = 0;
    }

    if (llm_txt_chunk >= chunks) {      /* result shrank under us */
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

    /* Advance: next chunk, or next pass, or done. */
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

 Try to restore the cumulative counters from the SD card.  Sets comms_loaded
 once the card has answered one way or the other; until then the caller must
 not checkpoint.

 Called at task start and then retried once per beacon cycle, because the
 card can mount late -- task_sdlog power-cycles the slot and retries on a
 hot-insert path, so "not mounted yet" is not the same as "no card".

 Contains no OS_Delay, so it is safe as a helper.
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
        comms_loaded = 1;               /* fresh card -- start from zero */
        user_debug_msg(STR_TASK_BEACON
                       "no counter checkpoint on SD; starting from zero.");
        break;

    default:                            /* SD_CTR_NO_CARD */
        break;                          /* stay unloaded; retry next cycle */
    }
}


/******************************************************************************
 beacon_counters_save()

 Checkpoint the counters to the card.  Called after every beacon: the write
 is a few dozen bytes to a pair of tiny alternating files, which the card
 absorbs without complaint, so there is no reason to batch it.  That is the
 whole advantage of keeping these off program flash.

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
static void beacon_counters_save(void) {
    static unsigned char warned = 0;

    if (!comms_loaded) {
        beacon_counters_restore();      /* card may have arrived late */
        return;
    }

    if (sd_counters_save(beacon_seq, total_tx, total_rx)) {
        warned = 0;
        return;
    }

    /* Report once per outage rather than every 10 s. */
    if (!warned) {
        warned = 1;
        user_debug_msg(STR_TASK_BEACON
                       "SD counter checkpoint FAILED -- counters will reset "
                       "on the next reboot (bc= still identifies it).");
    }
}


/******************************************************************************
 beacon_uplink_ack()

 Send a short acknowledgement frame for an uplink command.  Subject to the
 same TX gate as everything else, so an acknowledgement is never what breaks
 RF silence.  Swaps the radio out of RX for the transmission and puts it
 straight back afterwards.

 Contains no OS_Delay (si4463_tx_ax25 is all busy-wait), so it is safe as a
 helper.
******************************************************************************/
static void beacon_uplink_ack(const char *what) {
    int len;

    if (!beacon_compute_tx_allowed(OSGetTicks() / 100UL)) {
        return;
    }

    si4463_rx_disable();

    /* No "KO6IKZ " prefix -- see the BCN builder: the AX.25 source
       address already carries the callsign on every frame. */
    len = sprintf(ax25_info, "ACK %s seq=%lu bc=%u",
                  what, beacon_seq, (unsigned int)boot_count);
    if (len > 0 && len < (int)sizeof(ax25_info)) {
        si4463_tx_ax25((const unsigned char *)ax25_info, (unsigned int)len);
        total_tx++;
    }

    si4463_rx_enable();
}


/******************************************************************************
 up_prompt_reset()

 Drop any partially assembled uplinked prompt.  Contains no OS_Delay, so it
 is safe as a helper.
******************************************************************************/
static void up_prompt_reset(void) {
    unsigned char k;
    for (k = 0; k < UP_CHUNKS; k++) {
        up_chunk_len[k] = 0;
        up_chunk[k][0]  = '\0';
    }
    up_have  = 0;
    up_total = 0;
}


/******************************************************************************
 up_prompt_complete()

 1 if every chunk of the announced total is held.  Contains no OS_Delay, so
 it is safe as a helper.
******************************************************************************/
static unsigned char up_prompt_complete(void) {
    unsigned char want;

    if (up_total == 0) {
        return 0;
    }
    /* Low up_total bits all set, e.g. m=3 -> 0b111. */
    want = (unsigned char)((1u << up_total) - 1u);
    return ((up_have & want) == want) ? 1u : 0u;
}


/******************************************************************************
 up_prompt_store()

 Parse and store one "PR <k>/<m> <text>" chunk.  arg points just past the
 "PR ".  Returns 1 if the chunk was accepted.

 A chunk whose <m> disagrees with the assembly in progress starts a new one:
 the operator has moved on to a different prompt, and silently mixing pieces
 of two prompts would produce a plausible-looking sentence that was never
 sent.  Everything else about a malformed chunk is simply rejected -- this is
 attacker-facing data off the air, and there is nothing to gain by guessing
 what was meant.

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
static unsigned char up_prompt_store(const char *arg) {
    unsigned int  k = 0, m = 0;
    unsigned int  i;
    const char   *p = arg;

    while (*p >= '0' && *p <= '9') { k = k * 10u + (unsigned int)(*p - '0'); p++; }
    if (*p != '/' || k == 0 || k > UP_CHUNKS) {
        return 0;
    }
    p++;
    while (*p >= '0' && *p <= '9') { m = m * 10u + (unsigned int)(*p - '0'); p++; }
    if (m == 0 || m > UP_CHUNKS || k > m) {
        return 0;
    }
    if (*p == ' ') {
        p++;                            /* single separator before the text */
    }
    else if (*p != '\0') {
        return 0;
    }

    if (up_total != 0 && m != up_total) {
        up_prompt_reset();              /* different prompt -- start over */
    }
    up_total = (unsigned char)m;

    for (i = 0; p[i] != '\0' && i < UP_CHUNK_MAX; i++) {
        up_chunk[k - 1][i] = p[i];
    }
    up_chunk[k - 1][i] = '\0';
    up_chunk_len[k - 1] = (unsigned char)i;
    up_have |= (unsigned char)(1u << (k - 1));
    return 1;
}


/******************************************************************************
 up_prompt_assemble()

 Concatenate the held chunks in order into dst (always null-terminated) and
 return the length.  Only meaningful once up_prompt_complete().

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
static unsigned int up_prompt_assemble(char *dst, unsigned int cap) {
    unsigned int  len = 0;
    unsigned char k, i;

    for (k = 0; k < up_total; k++) {
        for (i = 0; i < up_chunk_len[k] && len + 1 < cap; i++) {
            dst[len++] = up_chunk[k][i];
        }
    }
    dst[len] = '\0';
    return len;
}


/******************************************************************************
 beacon_uplink_service()

 Non-blocking: check whether an uplink frame arrived, and if so validate and
 execute it.  Called repeatedly during the listening window between beacons.

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
static void beacon_uplink_dispatch(unsigned char *up_buf, unsigned char n) {
    extern char   strTmp[];
    const char   *cmd;
    unsigned char i;

    /* Sanitise before it goes anywhere near sprintf: this is attacker-facing
       data off the air, and control characters in the debug stream would
       corrupt the console. */
    for (i = 0; i < n; i++) {
        if (up_buf[i] < 0x20 || up_buf[i] > 0x7E) {
            up_buf[i] = '.';
        }
    }
    up_buf[n] = '\0';

    sprintf(strTmp, STR_TASK_BEACON "RX uplink %uB rssi=%u: \"%s\"",
            (unsigned int)n, (unsigned int)si4463_last_rssi(),
            (const char *)up_buf);
    user_debug_msg(strTmp);

    if (n < UPLINK_PREFIX_LEN
        || strncmp((const char *)up_buf, UPLINK_PREFIX, UPLINK_PREFIX_LEN) != 0) {
        user_debug_msg(STR_TASK_BEACON
                       "uplink: wrong callsign/auth prefix -- ignored.");
        return;
    }

    cmd = (const char *)up_buf + UPLINK_PREFIX_LEN;

    /* ---- BCN OFF: commanded RF silence ------------------------------
       Persisted immediately so it survives a reset (and so a brownout
       moments later cannot bring the transmitter back up).  No reply is
       sent: we have just been told to stop transmitting, and the ground
       confirms by observing that the carrier stopped. */
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
        /* Make sure we are listening for the command that undoes this. */
        si4463_rx_enable();
        return;
    }

    /* ---- BCN ON: resume beaconing ----------------------------------- */
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

    /* ---- PING: liveness check --------------------------------------- */
    if (strncmp(cmd, "PING", 4) == 0) {
        user_debug_msg(STR_TASK_BEACON "uplink: PING.");
        beacon_uplink_ack("PING");
        return;
    }

    /* ---- PR?: report prompt-assembly state --------------------------
       The acknowledgement carries the bitmap of chunks held, which is what
       the ground needs to re-send only the pieces that did not make it. */
    if (strncmp(cmd, "PR?", 3) == 0) {
        char detail[32];
        sprintf(detail, "PR? m=%u bm=%02X %s",
                (unsigned int)up_total, (unsigned int)up_have,
                up_prompt_complete() ? "complete" : "partial");
        user_debug_msg(STR_TASK_BEACON "uplink: PR? -- reporting state.");
        beacon_uplink_ack(detail);
        return;
    }

    /* ---- PRX: discard the partial prompt ---------------------------- */
    if (strncmp(cmd, "PRX", 3) == 0) {
        up_prompt_reset();
        user_debug_msg(STR_TASK_BEACON "uplink: PRX -- prompt assembly cleared.");
        beacon_uplink_ack("PRX");
        return;
    }

    /* ---- PR <k>/<m> <text>: one chunk of an uplinked prompt --------- */
    if (strncmp(cmd, "PR ", 3) == 0) {
        char detail[32];

        if (!up_prompt_store(cmd + 3)) {
            sprintf(strTmp, STR_TASK_BEACON
                    "uplink: malformed PR chunk \"%s\" -- ignored.", cmd);
            user_debug_msg(strTmp);
            return;
        }

        sprintf(strTmp, STR_TASK_BEACON
                "uplink: prompt chunk stored, m=%u bm=%02X%s",
                (unsigned int)up_total, (unsigned int)up_have,
                up_prompt_complete() ? " -- COMPLETE, awaiting RUN." : "");
        user_debug_msg(strTmp);

        sprintf(detail, "PR m=%u bm=%02X", (unsigned int)up_total,
                (unsigned int)up_have);
        beacon_uplink_ack(detail);
        return;
    }

    /* ---- RUN [<id>]: start an inference now -------------------------
       With an argument: that library prompt.  Without: the assembled
       uplinked prompt if one is complete, otherwise the next library prompt.

       The payload refuses while it is busy (jetson_request_* returns 0) and
       the refusal is acknowledged as NAK rather than silently dropped --
       repeating a telecommand is the normal way to get one through a fading
       uplink, and the operator needs to be able to tell "heard, declined"
       from "not heard". */
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
        else if (up_prompt_complete()) {
            static char up_prompt[JETSON_PROMPT_MAX + 1];
            unsigned int len = up_prompt_assemble(up_prompt, sizeof(up_prompt));

            if (jetson_request_prompt_text(up_prompt)) {
                sprintf(detail, "RUN pr=%uB", len);
            } else {
                sprintf(detail, "NAK RUN pr=%uB", len);
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

    sprintf(strTmp, STR_TASK_BEACON "uplink: unknown verb \"%s\" -- ignored.",
            cmd);
    user_debug_msg(strTmp);
}


/******************************************************************************
 beacon_uplink_service()

 Poll the radio once and hand anything it heard to the dispatcher above.
 Called repeatedly during the listening window between beacons.

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
static void beacon_uplink_service(void) {
    static unsigned char up_buf[SI4463_RX_MAX_PAYLOAD + 1];
    unsigned char n;

    n = si4463_rx_poll(up_buf, SI4463_RX_MAX_PAYLOAD);
    if (n == 0) {
        return;                          /* nothing heard */
    }

    /* Count every CRC-valid frame we receive, whether or not it turns out to
       be addressed to us -- it is still evidence the uplink works. */
    total_rx++;

    beacon_uplink_dispatch(up_buf, n);
}


/******************************************************************************
 beacon_uplink_inject()

 Bench aid: push a string through the real uplink dispatcher exactly as if it
 had arrived over the air, so the command grammar, the persistent inhibit and
 the acknowledgement path can be exercised without a ground transmitter.
 Everything downstream of the demodulator is the production code path; only
 si4463_rx_poll() is bypassed.

 Called from task_cmd (see the 'u' command).  Safe to call from another task:
 the radio helpers it can reach never yield, so a cooperative context switch
 cannot land in the middle of one.

 Contains no OS_Delay, so it is safe as a helper.
******************************************************************************/
void beacon_uplink_inject(const char *payload) {
    static unsigned char inj_buf[SI4463_RX_MAX_PAYLOAD + 1];
    extern char   strTmp[];
    unsigned char n = 0;

    while (payload[n] != '\0' && n < SI4463_RX_MAX_PAYLOAD) {
        inj_buf[n] = (unsigned char)payload[n];
        n++;
    }

    sprintf(strTmp, STR_TASK_BEACON "uplink: INJECTING \"%s\" (bench only).",
            payload);
    user_debug_msg(strTmp);

    beacon_uplink_dispatch(inj_buf, n);
}


/******************************************************************************
 beacon_inhibited()

 1 if the transmitter is under a commanded (persistent) beacon inhibit.
******************************************************************************/
unsigned int beacon_inhibited(void) {
    return (comms_flags & COMMSTATE_FLAG_BEACON_INHIBIT) ? 1u : 0u;
}


/******************************************************************************
 task_beacon()

 Salvo task entry point. Yields with OS_Delay between beacon transmissions.
******************************************************************************/
void task_beacon(void) {
    tlm_beacon_t tlm;
    int           ax25_len;
    int           tx_allowed;

    /* ===== Burn-wire gate FETs OFF at boot ===========================
       IO.24 (RF1) and IO.25 (RF0) gate the nichrome deploy FETs (Q1/Q2).
       Drive them LOW at startup so the burn wires are NEVER energised when
       the OBC comes online -- mandatory for vibration testing, and the safe
       default in every mode.  The flight deploy block in the loop below is
       the only place that drives them HIGH, and only during its timed burn
       (and only in a FLIGHT_MODE build with DEPLOY_ARMED set). */
    _TRISF1 = 0;            // RF1 (IO.24) -> output
    _TRISF0 = 0;            // RF0 (IO.25) -> output
    csk_io24_low();
    csk_io25_low();
    /* ================================================================= */

#if DEPLOY_TEST_MODE
    /* ===== DEPLOYMENT TEST MODE =====================================
       Bench burn test: 10 s after startup, drive the nichrome gate FETs on
       IO.24 (RF1) + IO.25 (RF0) HIGH for 10 s, then back LOW.  One-shot per
       boot, independent of the flight auto-deploy sequence and the persistent
       DEPLOYED flag -- it always pulses once.  Build flag in main.h; MUST be 0
       for flight.  !!! Energises nichrome -- disconnect the burn wires or fit
       a dummy load. !!!  OS_Delays sit at the task's top level (Salvo PIC24
       stack constraint -- no nested helper / while-loop). */
    user_debug_msg(STR_TASK_BEACON "DEPLOY_TEST_MODE: burn-wire pulse in 10 s.");

    /* 10 s pre-fire delay = 1000 ticks = 5 x OS_Delay(200). */
    __builtin_clrwdt();
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);

    user_debug_msg(STR_TASK_BEACON "DEPLOY_TEST_MODE: IO.24+IO.25 HIGH (10 s burn).");
    _TRISF1 = 0;
    _TRISF0 = 0;
    csk_io24_high();
    csk_io25_high();

    /* 10 s burn = 1000 ticks = 5 x OS_Delay(200). */
    __builtin_clrwdt();
    OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);

    csk_io24_low();
    csk_io25_low();
    user_debug_msg(STR_TASK_BEACON "DEPLOY_TEST_MODE: burn complete, IO.24+IO.25 LOW.");
#endif /* DEPLOY_TEST_MODE */

    /* ---- Boot-counter alive-window: 1 s ----------------------------------
       Record this power-up in the persistent boot counter ONCE, here, after a
       1 s alive-delay -- NOT in init().  Rationale: a cold ICSP read powers the
       chip from 0, and the core runs the boot prologue (init(), incl. the
       ~200 ms UART banner) for a few hundred ms before the programmer halts it.
       An increment placed in init() is reached inside that window, so every
       memory read counted as a boot and corrupted the pre/post vibration-test
       comparison.  1 s is comfortably past the programmer's
       halt window, so a read never reaches it, while every genuine boot does.
       Keep the OS_Delay at the task's top level (Salvo PIC24 stack constraint).
       bootcount_store() raises CPU IPL to 7 around the NVM unlock, so this is
       safe to call here with interrupts enabled.  (Also populates boot_count,
       which feeds tlm->reboot_count below.) */
    OS_Delay(100);                       /* 1 s @ 100 Hz tick */
    boot_count = (unsigned char)bootcount_increment();
    sprintf(strTmp, STR_TASK_BEACON "Boot count = %u (recorded after 1 s alive).",
            (unsigned int)boot_count);
    user_debug_msg(strTmp);

    /* Remainder of the original ~5 s startup settle before bringing up the
       radio and entering the beacon loop.  OS_Delay max is 255 ticks
       (OSBYTES_OF_DELAYS=1); keep each call at the task's top level. */
    OS_Delay(200);
    OS_Delay(200);

    user_debug_msg(STR_TASK_BEACON "Beacon task started.");

    /* Antenna deployment is handled INSIDE the beacon loop below (once uptime
       passes the FLIGHT_HOLD_S deployer-safety hold), not here, and only in a
       FLIGHT_MODE build.  The beacon loop is entered now either way: TX is
       gated ONLY by the RF-silence hold and the ground inhibit, never by
       deployment status, so a (faint, possibly pre-deploy) signal goes out
       regardless of whether the burn wires fired, succeeded, or were compiled
       out.  See beacon_rf_hold_cleared() and the deploy block in while(1). */

    /* ---- Restore persistent comms state --------------------------------
       seq/tx/rx used to live only in RAM, so every reset restarted them at
       zero and the ground could not tell "the spacecraft rebooted" from "we
       lost the pass".  They now come off the SD card, checkpointed after
       every beacon, so they resume exactly where they left off -- the card
       shrugs off a write every 10 s, which program flash could not.

       The commanded beacon-inhibit flag comes from program flash instead: it
       changes a handful of times in the whole mission, so there is no wear
       argument, and it must not depend on a card that could be missing or
       dead.  See commstate.h for that split. */
    comms_flags = commstate_flags_get();
    beacon_counters_restore();

    sprintf(strTmp, STR_TASK_BEACON
            "comms state: seq=%lu tx=%lu rx=%lu flags=0x%04X%s",
            beacon_seq, total_tx, total_rx, comms_flags,
            (comms_flags & COMMSTATE_FLAG_BEACON_INHIBIT)
                ? "  [BEACON INHIBITED BY GROUND COMMAND]" : "");
    user_debug_msg(strTmp);

    /* One-shot Si4463 init — task_si4463_rx is disabled, so no other task
       owns the chip. Leaves the chip in READY with TX path selected. */
    si4463_init_beacon();

    /* Try to restore last telemetry from SD card (survives reboot) */
    if (!jetson_last_telem.valid) {
        if (sd_read_last_telem(&jetson_last_telem)) {
            user_debug_msg(STR_TASK_BEACON "Restored telemetry from SD.");
        }
    }

    while (1) {
        __builtin_clrwdt();

        /* Collect telemetry */
        collect_telemetry(&tlm);

        /* RF-silence compliance gate + commanded inhibit (see helper). */
        tx_allowed = beacon_compute_tx_allowed(tlm.uptime_s);

        /* Debug: print beacon sequence + rail + tumble readings */
        beacon_print_debug(&tlm, tx_allowed);

        /* Build ASCII info field, transmit as AX.25 UI frame so SatNOGS
           + any 1k2 G3RUH ham receiver decodes it with no custom
           software. Prefix "BCN" identifies the housekeeping beacon.

           seq/rx are cumulative across reboots (restored from flash);
           bc= is the persistent boot count and rst= the condensed reset
           cause.  Together those two make a seq discontinuity readable: an
           unchanged bc means we simply were not heard for a while, an
           incremented bc means the spacecraft reset, and rst= says why.

           COMPACTED for 1200 bps -- airtime is now ~8x more expensive per
           byte, so every field has to earn its place.  Removed:
             - the "KO6IKZ " prefix: it duplicates the AX.25 source address
               (AX25_SRC_CALL), which is in the header of every single frame.
             - "|w|=": tumble_mag_dps is sqrt(gx^2+gy^2+gz^2) computed from
               the three axes that are still in the frame, so it was pure
               redundancy.  Axes kept in preference to the magnitude so the
               ground can do detumble analysis per-axis; the magnitude is
               recoverable from them, but not the reverse.
             - "tx=": total_tx increments on the same beat as beacon_seq
               (below) plus once per LLM and ACK frame, so tx-seq only ever
               counted LLM+ACK traffic -- not actionable from the ground.
               rx= is KEPT: it is the only evidence an uplink was received.
           Uptime is in minutes, not seconds: a 10 s beacon never needed 1 s
           resolution and minutes bounds the field width. */
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
            /* Leave RX and point the RF switch at the PA before keying up. */
            si4463_rx_disable();
            si4463_tx_ax25((const unsigned char *)ax25_info,
                           (unsigned int)ax25_len);
            /* Update counters only on an actual transmission. */
            beacon_seq++;
            total_tx++;
        }

        /* If task_jetson queued an LLM-inference frame, TX it now (subject to
           the same RF-silence hold).  Only one Si4463-owning task (this one),
           so no locking needed. */
        if (tx_allowed && jetson_last_telem.pending_tx) {
            si4463_rx_disable();
            beacon_tx_llm_frame();
        }

        /* Then one chunk of the generated text, if a result is still being
           downlinked.  One per cycle on purpose -- see the airtime and
           fade-margin argument above beacon_tx_llm_text().  The housekeeping
           frame above goes out first on the cycle after an inference, so a
           station that hears only that one still learns how many TXT frames
           to expect (txt=<bytes>/<frames>). */
        if (tx_allowed && jetson_infer_result.dl_pending) {
            si4463_rx_disable();
            beacon_tx_llm_text();
        }

        /* Checkpoint the counters to the card after every beacon. */
        beacon_counters_save();

#if (FLIGHT_MODE && DEPLOY_ARMED)
        /* ---------- One-shot antenna deployment ----------
           This block exists only in a build with BOTH flags set -- see
           FLIGHT_MODE and DEPLOY_ARMED in main.h.  DEPLOY_ARMED is the
           separate debug disarm: with it at 0 this whole block is compiled
           out even in a FLIGHT_MODE build, while everything else about flight
           mode stays live.  Check the values in main.h rather than trusting a
           comment here, and check the boot banner on the actual board.

           Fire the nichrome burn wires once the FLIGHT_HOLD_S deployer-safety
           hold has elapsed (uptime resets to 0 on every boot, so this
           naturally re-honours the full wait after a reset).  We keep
           beaconing the whole time, so a signal goes out regardless of deploy
           status.

           This block sits AFTER the transmit block above, and the RF hold uses
           the same FLIGHT_HOLD_S threshold, so on the first qualifying cycle
           the beacon frame is keyed out and finished BEFORE the nichrome is
           energised.  The PA and the burn wires therefore never draw at the
           same instant -- do not reorder these two blocks.

           Resilience (persistent state in program flash, see deploy.h):
             - DEPLOYED flag set            -> never re-fire.
             - attempts >= MAX_ATTEMPTS     -> give up, just keep beaconing.
             - else                         -> record attempt, then burn.

           The attempt counter is bumped just BEFORE energising, so a reboot
           during the burn (e.g. brownout from the high-current load) still
           counts toward the limit and we retry next boot -- up to
           DEPLOY_MAX_ATTEMPTS times.  The DEPLOYED flag is written only after
           the burn fully completes, so a half-finished burn is never recorded
           as done.

           OS_Delays here sit directly in the loop body (same as the beacon
           interval below), which is the pattern that works on this Salvo
           build -- no nested helper / return. */
        if (!deploy_get()
            && deploy_attempts_get() < DEPLOY_MAX_ATTEMPTS
            && tlm.uptime_s >= FLIGHT_HOLD_S) {

            unsigned int n = deploy_attempt_inc();   /* persist before burn */
            sprintf(strTmp, STR_TASK_BEACON
                    "Deploy attempt %u/%u: nichrome IO.24+IO.25 HIGH.",
                    n, DEPLOY_MAX_ATTEMPTS);
            user_debug_msg(strTmp);

            /* Drive burn wires HIGH on IO.24 (RF1) and IO.25 (RF0), two in
               parallel for redundancy. */
            _TRISF1 = 0;
            _TRISF0 = 0;
            csk_io24_high();
            csk_io25_high();

            /* Burn duration: 10 s = 1000 ticks = 5 × OS_Delay(200). */
            OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200); OS_Delay(200);

            csk_io24_low();
            csk_io25_low();

            deploy_set();   /* burn completed -- record DEPLOYED */
            user_debug_msg(STR_TASK_BEACON
                           "Nichrome OFF, antenna deployed (flag set).");
        }
#endif /* FLIGHT_MODE && DEPLOY_ARMED */

        /* ---------- Listening window (uplink) ----------
           The satellite is half-duplex with a single radio, so "having an
           uplink" simply means spending the whole inter-beacon gap parked in
           RX instead of idle in READY.  Arm RX once, then poll every 400 ms
           for the rest of the ~10 s interval.

           Polling rather than interrupts, and unrolled OS_Delays rather than
           a loop, because of the two standing constraints on this build:
           si4463_rx_poll() must not yield (it does not), and OS_Delay() only
           works reliably at the task's top level -- a for/while loop around
           it corrupts the task on resume (see the note at the top of this
           file).

           The poll period is also the deaf period: START_RX parks the chip
           in READY holding a good packet until we drain it.  400 ms against
           a ~136 ms uplink frame means a ground station repeating a command
           will land one within a couple of tries. */
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
        OS_Delay(40); beacon_uplink_service();   /* 25 -- 25 * 40 = 1000
                                                    ticks = 10 s */
    }
}
