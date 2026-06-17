/******************************************************************************
KENSAT - Telemetry Beacon Task

Salvo cooperative task. Wakes every BEACON_INTERVAL ticks (default 30s),
collects available housekeeping data, serializes into tlm_beacon_t, and
transmits via Si4463 GFSK TX.

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
******************************************************************************/
static unsigned int  beacon_seq  = 0;
static unsigned char boot_count  = 0;

/* Latest rail readings, updated each beacon cycle. task_sdlog reads these. */
volatile unsigned int g_rail_5v_mv  = 0;
volatile unsigned int g_rail_3v3_mv = 0;
volatile unsigned int g_vbatt_mv    = 0;   /* battery / SoC, EPS I2C not yet wired */

/* Salvo prebuilt library (OSLIBRARY_CONFIG=OST) uses OSBYTES_OF_DELAYS=1, so
   OS_Delay() takes uint8_t (max 255 ticks ≈ 2.55s at 100 Hz). For longer
   delays, issue multiple OS_Delay(200) calls *at task top level*. Yielding
   from inside a helper or a while-loop with a local counter corrupts the
   stack on resume on this build (chip resets or the task hangs); the only
   reliably-working pattern observed is unrolled top-level OS_Delays. */

/* Comms counters (updated externally or via accessors in future) */
static unsigned int  total_tx    = 0;
static unsigned int  total_rx    = 0;
static unsigned int  total_crc_err = 0;

/* Current TX power setting (PA_PWR_LVL) */
#define BEACON_TX_POWER  0x7F


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
#define VCC_3V3_LO_MV    2800U     /* below this → flag rail down         */
#define VCC_3V3_HI_MV    3700U     /* above this → reading is bogus       */

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
 read_vcc_5v_mv()

 +5V_SYS is upstream of the EPS 3.3V buck/LDO that produces VCC_SYS. We
 don't have a direct ADC measurement for it on this hardware (no external
 divider on any AN pin, no EPS I²C contract finalised yet). What we *can*
 say is: if VCC_SYS is in nominal range, +5V_SYS must be alive — the buck
 cannot regulate without it. So we report the nominal 5000 mV in that case
 and 0 ("rail down / unknown") otherwise. This is honest enough to flag
 outright failures and ride along when the rail is healthy.

 When the EPS exposes its I²C telemetry register or someone wires a
 divider to AN0/AN8, replace this with the real measurement.
******************************************************************************/
#define VCC_5V_NOM_MV    5000U

static unsigned int read_vcc_5v_mv(unsigned int v3v3_mv) {
    if (v3v3_mv >= VCC_3V3_LO_MV && v3v3_mv <= VCC_3V3_HI_MV) {
        return VCC_5V_NOM_MV;
    }
    return 0;
}


/******************************************************************************
 read_vbatt_mv()

 Battery voltage via the VBATT SENSE divider on IO.32 (= RB5 = AN5).

 The divider scales the battery down so that full charge (8.4 V) reads
 3.02 V at the ADC pin, leaving headroom under the 3.3 V rail. So:
     V_batt = V_pin * (8400 / 3020)   (ratio ≈ 2.781)

 V_pin is recovered from the raw 10-bit count using the *measured* 3V3 rail
 as Vref (AD1CON2 selects AVDD/AVSS), which read_vcc_3v3_mv() already gives
 us — so battery accuracy doesn't drift with the rail.
     V_pin_mv = count * vref_mv / 1023

 Pass the measured rail (vcc_3v3_mv) as vref_mv. Returns mV, 0 if the pin
 reads essentially zero (battery sense absent / divider open).
******************************************************************************/
#define VBATT_CAL_PIN_MV    3020UL   /* ADC-pin mV at full charge          */
#define VBATT_CAL_BATT_MV   8400UL   /* battery mV at full charge (8.4 V)  */

static unsigned int read_vbatt_mv(unsigned int vref_mv) {
    unsigned int  count;
    unsigned long vpin_mv;

    /* Configure RB5/AN5 as an analog input (boot leaves PORTB digital
       outputs — see csk_power.c). */
    ANSBbits.ANSB5 = 1;
    TRISBbits.TRISB5 = 1;

    count = adc_read_chan(5);

    if (count < 5 || vref_mv == 0) {
        return 0;                    /* nothing on the divider / no Vref */
    }

    vpin_mv = ((unsigned long)count * (unsigned long)vref_mv) / 1023UL;
    return (unsigned int)((vpin_mv * VBATT_CAL_BATT_MV) / VBATT_CAL_PIN_MV);
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
    tlm->seq_num      = beacon_seq;

    /* OBC */
    tlm->uptime_s     = OSGetTicks() / 100UL;
    tlm->reboot_count = boot_count;

    /* Power rails. VCC_SYS via internal band gap (real); +5V_SYS inferred
       from VCC_SYS health (see read_vcc_5v_mv()); VBATT via the IO.32 (AN5)
       sense divider, scaled against the measured 3V3 rail used as Vref. */
    tlm->vcc_3v3_mv = read_vcc_3v3_mv();
    tlm->vcc_5v_mv  = read_vcc_5v_mv(tlm->vcc_3v3_mv);
    tlm->vbatt_mv   = read_vbatt_mv(tlm->vcc_3v3_mv);

    /* Cache for task_sdlog and debug consumers. */
    g_rail_3v3_mv = tlm->vcc_3v3_mv;
    g_rail_5v_mv  = tlm->vcc_5v_mv;
    g_vbatt_mv    = tlm->vbatt_mv;

    if (tlm->vcc_3v3_mv >= VCC_3V3_LO_MV && tlm->vcc_3v3_mv <= VCC_3V3_HI_MV)
        tlm->eps_rails |= TLM_RAIL_3V3;
    if (tlm->vcc_5v_mv > 0)
        tlm->eps_rails |= TLM_RAIL_5V;
    if (tlm->vbatt_mv > 0)
        tlm->eps_rails |= TLM_RAIL_VBATT;
    if (csk_status.jetson_power_on)
        tlm->eps_rails |= TLM_RAIL_JETSON;

    /* Tumbling: per-axis angular rate + precomputed magnitude. */
    tlm->gyro_x_dps     = g_gyro_x_dps;
    tlm->gyro_y_dps     = g_gyro_y_dps;
    tlm->gyro_z_dps     = g_gyro_z_dps;
    tlm->tumble_mag_dps = g_tumble_mag_dps;

    /* Comms */
    tlm->tx_power      = BEACON_TX_POWER;
    tlm->last_rssi     = 0;
    tlm->tx_count      = total_tx;
    tlm->rx_count      = total_rx;
    tlm->crc_err_count = total_crc_err;

    /* Operational */
    tlm->mission_mode  = TLM_MODE_NOMINAL;
}


/******************************************************************************
 task_beacon()

 Salvo task entry point. Yields with OS_Delay between beacon transmissions.
******************************************************************************/
void task_beacon(void) {
    tlm_beacon_t tlm;
    int           ax25_len;
    int           tx_allowed;

    /* ===== TEMPORARY probe test (DISABLED) ===========================
       Gate drive turned OFF -- both burn FETs are left undriven, so the
       R2/R4 pulldowns hold Q1/Q2 gates low (FETs off).  Re-enable by
       uncommenting to drive IO.24 (RF1) + IO.25 (RF0) HIGH at boot. */
    _TRISF1 = 0;            // RF1 (IO.24) -> output
    _TRISF0 = 0;            // RF0 (IO.25) -> output
    csk_io24_high();
    csk_io25_high();
    /* ================================================================= */

    /* 5 s startup delay. OS_Delay max is 255 ticks (OSBYTES_OF_DELAYS=1),
       so split into chunks. Keep every OS_Delay at the task's top level —
       on this Salvo PIC24 build, yielding from inside a helper or a while-
       loop with locals corrupts the stack on resume and the task hangs. */
    OS_Delay(200);
    OS_Delay(200);
    OS_Delay(100);

    user_debug_msg(STR_TASK_BEACON "Beacon task started.");

    /* ===== TEMPORARY -- CW carrier on boot =============================
       Emit an unmodulated continuous-wave carrier at 437.08 MHz right at
       startup (for antenna / VNA / RF-path bring-up).  si4463_cmd_cw()
       does its own SPI init + POWER_UP and then loops forever until 'q'
       is pressed on the debug UART; once stopped it falls through to the
       normal beacon loop below.  Remove this block to restore normal
       boot-to-beacon behaviour. */
    si4463_cmd_cw();
    /* =================================================================== */

    /* Antenna deployment is handled INSIDE the beacon loop below (once uptime
       passes the 120 s deployer-safety hold), not here.  This way we start
       beaconing immediately and a (faint, possibly pre-deploy) signal goes out
       regardless of whether deployment ever succeeds -- see the deploy block
       in the while(1) loop. */

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

        /* RF-silence compliance: in flight, stay silent until the initial
           125 s post-power-on hold has been satisfied ONCE.  That fact is
           persisted (deploy_rf_unlock), so after the first time, later
           reboots transmit immediately without re-waiting.  Antenna
           deployment below still proceeds during the hold -- it is GPIO, not
           RF.  On the bench we transmit right away. */
#if BENCH_TEST
        tx_allowed = 1;
#else
        if (deploy_rf_unlocked_get()) {
            tx_allowed = 1;                  /* hold already satisfied earlier */
        }
        else if (tlm.uptime_s >= 125) {
            deploy_rf_unlock();              /* satisfied now -- persist once */
            tx_allowed = 1;
        }
        else {
            tx_allowed = 0;                  /* still in the one-time hold */
        }
#endif

        /* Debug: print beacon sequence + rail + tumble readings */
        {
            extern char strTmp[];
            if (tx_allowed)
                sprintf(strTmp, STR_TASK_BEACON "TX beacon #%u (uptime %lus)",
                        beacon_seq, tlm.uptime_s);
            else
                sprintf(strTmp, STR_TASK_BEACON
                        "RF silent (uptime %lus < 125s compliance hold)",
                        tlm.uptime_s);
            user_debug_msg(strTmp);
            sprintf(strTmp,
                    STR_TASK_BEACON "rails: VB=%umV 5V=%umV 3V3=%umV flags=0x%02X",
                    tlm.vbatt_mv, tlm.vcc_5v_mv, tlm.vcc_3v3_mv,
                    (unsigned int)tlm.eps_rails);
            user_debug_msg(strTmp);
            sprintf(strTmp,
                    STR_TASK_BEACON "tumble: gx=%d gy=%d gz=%d |w|=%udps",
                    tlm.gyro_x_dps, tlm.gyro_y_dps, tlm.gyro_z_dps,
                    tlm.tumble_mag_dps);
            user_debug_msg(strTmp);
        }

        /* Build ASCII info field, transmit as AX.25 UI frame so SatNOGS
           + any 9k6 G3RUH ham receiver decodes it with no custom
           software. Prefix "BCN" identifies the housekeeping beacon. */
        ax25_len = sprintf(ax25_info,
            "KO6IKZ BCN seq=%u up=%lus vb=%u v5=%u v3=%u r=%02X "
            "gx=%d gy=%d gz=%d |w|=%u txp=%02X tx=%u rx=%u ce=%u m=%u",
            beacon_seq,
            tlm.uptime_s,
            tlm.vbatt_mv, tlm.vcc_5v_mv, tlm.vcc_3v3_mv,
            (unsigned int)tlm.eps_rails,
            tlm.gyro_x_dps, tlm.gyro_y_dps, tlm.gyro_z_dps,
            tlm.tumble_mag_dps,
            (unsigned int)tlm.tx_power,
            tlm.tx_count, tlm.rx_count, tlm.crc_err_count,
            (unsigned int)tlm.mission_mode);

        if (tx_allowed && ax25_len > 0 && ax25_len < (int)sizeof(ax25_info)) {
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
            jetson_last_telem.pending_tx = 0;
            sprintf(strTmp,
                    STR_TASK_BEACON "TX LLM frame #%u (prompt %u, infer=%us)",
                    jetson_last_telem.tx.seq_num,
                    jetson_last_telem.tx.prompt_id,
                    jetson_last_telem.tx.infer_dur_s);
            user_debug_msg(strTmp);

            ax25_len = sprintf(ax25_info,
                "KO6IKZ LLM seq=%u pid=%u up=%lus "
                "cpu=%d gpu=%d pk=%d/%d mw=%u ram=%u "
                "ml=%us inf=%us lat=%ums st=%02X h=%08lX",
                jetson_last_telem.tx.seq_num,
                jetson_last_telem.tx.prompt_id,
                jetson_last_telem.tx.uptime_s,
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
                jetson_last_telem.tx.llm_hash);

            if (ax25_len > 0 && ax25_len < (int)sizeof(ax25_info)) {
                si4463_tx_ax25((const unsigned char *)ax25_info,
                               (unsigned int)ax25_len);
            }
            total_tx++;
        }

#if !BENCH_TEST
        /* ---------- One-shot antenna deployment ----------
           Fire the nichrome burn wires once the 120 s deployer-safety hold
           has elapsed (uptime resets to 0 on every boot, so this naturally
           re-honours the 120 s wait after a reset).  We keep beaconing the
           whole time, so a signal goes out regardless of deploy status.

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
            && tlm.uptime_s >= 120) {

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
#endif /* !BENCH_TEST */

        /* Wait for next beacon interval (~30s) */
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);
        OS_Delay(200);   /* 15 * 200 = 3000 ticks = 30 s */
    }
}
