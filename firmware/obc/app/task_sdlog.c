/******************************************************************************
KENSAT SD Card Telemetry Logger

Salvo cooperative task. Initializes the SD card via EFFS-THIN (SPI1),
then logs telemetry to TELEM.CSV every 60 seconds. Also provides
sd_read_last_telem() so task_beacon can restore Jetson telemetry after
a reboot.

SD card: SPI1, CS on RE5, power on RE4 (both active LOW via csk_sd API).
Filesystem: HCC EFFS-THIN library 4 (FAT16, superthin, F_MAXFILES=1).

******************************************************************************/
#include "main.h"
#include "events.h"
#include "task_beacon.h"
#include "task_sdlog.h"
#include "task_bmi088.h"
#include "task_jetson.h"
#include "telemetry.h"

#include "csk_sd.h"
#include "csk_uart.h"
#include "csk_io.h"
#include "bootcount.h"

#ifndef CSK_EFFS_THIN_LIB
#define CSK_EFFS_THIN_LIB 4
#endif
#include "thin_usr.h"

#include "salvo.h"

#include <string.h>


/* ---- Configuration ---- */
#define SDLOG_POLL_TICKS      200       /* 2 s per iteration (fits uint8) */
#define SDLOG_WRITE_EVERY     30        /* 30 × 2 s = 60 s between CSV rows */
#define SDLOG_MAX_INIT_RETRIES 3        /* give up (no card) after this many   *
                                         * failed re-mounts, then idle quietly */
#define SDLOG_FILE_NAME       "TELEM.CSV"
#define SDLOG_INFER_FILE      "INFER.LOG"
#define SDLOG_BOOT_FILE       "BOOT.LOG"

/* Comms counters are checkpointed to a PAIR of tiny files, written
   alternately (see sd_counters_save()).  EFFS-THIN superthin has no
   f_rename and can only hold one file open at a time, so a ping-pong pair
   is how we get an atomic update: the copy we are not writing is always a
   complete, previously-verified checkpoint, so a power failure part-way
   through a write can never destroy both. */
#define SDLOG_CTR_FILE_A      "COMMS0.DAT"
#define SDLOG_CTR_FILE_B      "COMMS1.DAT"
#define SDLOG_CTR_MAGIC       "KENSAT1"
#define SDLOG_LINE_MAX        128       /* must hold one whole CSV row + slack */

/* CSV header (must fit in strTmp).
   v3v3sys is the VCC_SYS rail voltage in mV (no vbatt: no battery-sense HW).
   tumble columns mirror the beacon: per-axis gyro dps + magnitude. */
#define SDLOG_CSV_HEADER  "uptime,cpu,gpu,cpu_pk,gpu_pk,pwr_mw,ram_mb,infer_s,load_s,jetson,v3v3sys,gx,gy,gz,tumble\r\n"


/* ---- Module state ---- */
static unsigned char sd_initialized = 0;
static unsigned char sd_gave_up     = 0;   /* set once no card found -> stop retrying */
static unsigned char sd_init_retries = 0;
static unsigned long sd_write_count = 0;
static unsigned char sd_boot_logged = 0;   /* boot count appended to BOOT.LOG once */

/* Which counter file to write next, and the serial stamped into it.  The
   serial is what tells the two copies apart at boot; it only ever grows. */
static unsigned char sd_ctr_next   = 0;
static unsigned long sd_ctr_serial = 0;




/******************************************************************************
 try_sd_mount()

 Mount FAT16 volume (call after power-on + delay). Returns 1 on success.
 Does NOT yield — safe to call from task context between OS_Delay calls.
******************************************************************************/
static unsigned char try_sd_mount(void) {

    unsigned char rc;

    rc = f_initvolume();
    if (rc == 0) {
        return 1;
    }

    sprintf(strTmp, STR_TASK_SDLOG "f_initvolume failed (err %u).", (unsigned int)rc);
    user_debug_msg(strTmp);
    return 0;
}


/******************************************************************************
 write_boot_log()

 Appends one line to BOOT.LOG recording the persistent boot count (from
 program flash) at the uptime this card was mounted. Lets you pull the SD
 card after a vibration test and confirm the OBC didn't reset: the boot
 count should match the last entry and no new lines should appear.

 Format:  boot #<n> @ <uptime>s\r\n
 Returns 1 on success, 0 on failure.
******************************************************************************/
static unsigned char write_boot_log(void) {

    F_FILE *file;
    unsigned int len;
    unsigned int written;

    /* Tick = 10 ms (100 Hz), so ms = ticks * 10.  ms (not s) so fast mounts
       aren't all "0". */
    sprintf(strTmp, "boot #%u @ %lums\r\n",
            bootcount_get(),
            OSGetTicks() * 10UL);
    len = strlen(strTmp);

    file = f_open(SDLOG_BOOT_FILE, "a+");
    if (!file) {
        user_debug_msg(STR_TASK_SDLOG "Cannot open " SDLOG_BOOT_FILE ".");
        return 0;
    }

    written = (unsigned int)f_write(strTmp, 1, len, file);
    f_close(file);

    return written > 0;
}


/******************************************************************************
 sd_counters_parse()

 Parse one checkpoint line: "KENSAT1 <serial> <seq> <tx> <rx> <sum>".
 Returns 1 if the line is well formed AND the checksum matches, else 0.

 The checksum is what makes a torn write detectable.  f_write is not atomic,
 so a power failure mid-update can leave a short or half-overwritten file;
 without the sum a truncated "KENSAT1 91 12" would parse as a plausible
 (wrong) checkpoint and silently roll the sequence number backwards.
******************************************************************************/
static unsigned char sd_counters_parse(const char *line,
                                       unsigned long *serial,
                                       unsigned long *seq,
                                       unsigned long *tx,
                                       unsigned long *rx) {

    const char   *p = line;
    unsigned long v[5];
    unsigned char i;

    if (strncmp(p, SDLOG_CTR_MAGIC, sizeof(SDLOG_CTR_MAGIC) - 1) != 0)
        return 0;
    p += sizeof(SDLOG_CTR_MAGIC) - 1;

    for (i = 0; i < 5; i++) {
        unsigned char digits = 0;
        while (*p == ' ') p++;
        v[i] = 0;
        while (*p >= '0' && *p <= '9') {
            v[i] = v[i] * 10UL + (unsigned long)(*p - '0');
            p++;
            digits++;
        }
        if (digits == 0)
            return 0;                    /* truncated / not a number */
    }

    if (v[4] != (v[0] + v[1] + v[2] + v[3]))
        return 0;                        /* torn or corrupted write */

    *serial = v[0];
    *seq    = v[1];
    *tx     = v[2];
    *rx     = v[3];
    return 1;
}


/******************************************************************************
 sd_counters_read_one()

 Read and validate a single checkpoint file.  Returns 1 on success.
******************************************************************************/
static unsigned char sd_counters_read_one(const char *name,
                                          unsigned long *serial,
                                          unsigned long *seq,
                                          unsigned long *tx,
                                          unsigned long *rx) {

    F_FILE *file;
    char    buf[64];
    long    nread;

    if (f_filelength(name) <= 0)
        return 0;                        /* missing or empty */

    file = f_open(name, "r");
    if (!file)
        return 0;

    nread = f_read(buf, 1, sizeof(buf) - 1, file);
    f_close(file);

    if (nread <= 0)
        return 0;

    buf[nread] = '\0';
    return sd_counters_parse(buf, serial, seq, tx, rx);
}


/******************************************************************************
 sd_counters_load()

 Restore the cumulative beacon counters from the SD card.  Reads both copies
 of the ping-pong pair and takes the one with the higher serial that still
 passes its checksum, so an update interrupted by a power failure falls back
 to the previous complete checkpoint instead of losing the counters.

 Returns:
   SD_CTR_LOADED    a valid checkpoint was found; seq/tx/rx are set
   SD_CTR_EMPTY     the card is up but holds no valid checkpoint yet
                    (fresh card, or both copies corrupt) -- start from zero
   SD_CTR_NO_CARD   the card is not mounted; try again later, and do NOT
                    save in the meantime or a good checkpoint would be
                    overwritten with zeros

 Does not yield -- safe to call from a Salvo helper.
******************************************************************************/
unsigned char sd_counters_load(unsigned long *seq,
                               unsigned long *tx,
                               unsigned long *rx) {

    unsigned long ser_a, seq_a, tx_a, rx_a;
    unsigned long ser_b, seq_b, tx_b, rx_b;
    unsigned char ok_a, ok_b;

    if (!sd_initialized)
        return SD_CTR_NO_CARD;

    ok_a = sd_counters_read_one(SDLOG_CTR_FILE_A, &ser_a, &seq_a, &tx_a, &rx_a);
    ok_b = sd_counters_read_one(SDLOG_CTR_FILE_B, &ser_b, &seq_b, &tx_b, &rx_b);

    if (!ok_a && !ok_b) {
        sd_ctr_serial = 0;
        sd_ctr_next   = 0;
        return SD_CTR_EMPTY;
    }

    /* Take the newer valid copy; write the OTHER one next, so the copy we
       just trusted stays intact until its replacement is complete. */
    if (ok_a && (!ok_b || ser_a >= ser_b)) {
        *seq = seq_a; *tx = tx_a; *rx = rx_a;
        sd_ctr_serial = ser_a;
        sd_ctr_next   = 1;
    } else {
        *seq = seq_b; *tx = tx_b; *rx = rx_b;
        sd_ctr_serial = ser_b;
        sd_ctr_next   = 0;
    }

    return SD_CTR_LOADED;
}


/******************************************************************************
 sd_counters_save()

 Write one checkpoint, alternating between the two copies.  Cheap enough to
 call on every beacon, which is the whole point of keeping these on the card
 rather than in program flash: the sequence number the ground sees is the
 exact one that was last transmitted, with no rounding to a checkpoint
 interval and no risk of a number ever being reused.

 Returns 1 on success.  Does not yield -- safe to call from a Salvo helper.
******************************************************************************/
unsigned char sd_counters_save(unsigned long seq,
                               unsigned long tx,
                               unsigned long rx) {

    F_FILE       *file;
    const char   *name;
    unsigned int  len;
    unsigned int  written;

    if (!sd_initialized)
        return 0;

    sd_ctr_serial++;
    name = sd_ctr_next ? SDLOG_CTR_FILE_B : SDLOG_CTR_FILE_A;

    /* Plain ASCII, one line, so the state is readable by eye when the card
       is pulled after a test. */
    sprintf(strTmp, SDLOG_CTR_MAGIC " %lu %lu %lu %lu %lu\r\n",
            sd_ctr_serial, seq, tx, rx,
            sd_ctr_serial + seq + tx + rx);
    len = strlen(strTmp);

    file = f_open(name, "w");            /* truncate: keeps the file tiny */
    if (!file)
        return 0;

    written = (unsigned int)f_write(strTmp, 1, len, file);
    f_close(file);

    if (written != len)
        return 0;

    sd_ctr_next = sd_ctr_next ? 0 : 1;   /* ping-pong for the next save */
    return 1;
}


/******************************************************************************
 write_csv_line()

 Appends one CSV row to TELEM.CSV. Returns bytes written (0 on error).
******************************************************************************/
static unsigned int write_csv_line(void) {

    F_FILE *file;
    unsigned int len;
    unsigned int written;
    unsigned long uptime_s;

    /* Compute uptime from Salvo ticks */
    uptime_s = OSGetTicks() / 100UL;

    /* Format CSV row. 17 fields would overflow strTmp (160 B) if formatted
       at once, so build it in two halves. */
    sprintf(strTmp, "%lu,%d,%d,%d,%d,%u,%u,%u,%u,%u,",
            uptime_s,
            jetson_last_telem.tx.cpu_temp,
            jetson_last_telem.tx.gpu_temp,
            jetson_last_telem.tx.cpu_temp_peak,
            jetson_last_telem.tx.gpu_temp_peak,
            jetson_last_telem.tx.power_mw,
            jetson_last_telem.tx.ram_mb,
            jetson_last_telem.tx.infer_dur_s,
            jetson_last_telem.tx.model_load_s,
            (unsigned int)csk_status.jetson_power_on);
    sprintf(strTmp + strlen(strTmp), "%u,%d,%d,%d,%u\r\n",
            g_rail_3v3_mv,
            g_gyro_x_dps,
            g_gyro_y_dps,
            g_gyro_z_dps,
            g_tumble_mag_dps);

    len = strlen(strTmp);

    file = f_open(SDLOG_FILE_NAME, "a+");
    if (!file) {
        user_debug_msg(STR_TASK_SDLOG "Cannot open " SDLOG_FILE_NAME ".");
        return 0;
    }

    /* Write CSV header on first write to a new file */
    if (sd_write_count == 0) {
        if (f_filelength(SDLOG_FILE_NAME) == 0) {
            f_write(SDLOG_CSV_HEADER, 1, strlen(SDLOG_CSV_HEADER), file);
        }
    }

    written = (unsigned int)f_write(strTmp, 1, len, file);
    f_close(file);

    if (written == 0) {
        user_debug_msg(STR_TASK_SDLOG "f_write failed (card full?).");
    }

    return written;
}


/******************************************************************************
 write_infer_log()

 Appends one inference result to INFER.LOG. Format:
   [<uptime>s] P<id> cpu=<c> gpu=<g> pwr=<p>mW infer=<i>s len=<n>\r\n
   <response text>\r\n
   ---\r\n
 Returns 1 on success, 0 on failure.
******************************************************************************/
static unsigned char write_infer_log(void) {

    F_FILE *file;
    unsigned int hdr_len;
    unsigned int written;

    /* Format the header line using strTmp (80 bytes) */
    sprintf(strTmp, "[%lus] P%02u cpu=%d gpu=%d pwr=%u infer=%us len=%u\r\n",
            jetson_infer_result.uptime_s,
            jetson_infer_result.prompt_id,
            jetson_last_telem.tx.cpu_temp,
            jetson_last_telem.tx.gpu_temp,
            jetson_last_telem.tx.power_mw,
            jetson_last_telem.tx.infer_dur_s,
            jetson_infer_result.response_len);

    hdr_len = strlen(strTmp);

    file = f_open(SDLOG_INFER_FILE, "a+");
    if (!file) {
        user_debug_msg(STR_TASK_SDLOG "Cannot open " SDLOG_INFER_FILE ".");
        return 0;
    }

    /* Write header line */
    written = (unsigned int)f_write(strTmp, 1, hdr_len, file);
    if (written == 0) {
        f_close(file);
        return 0;
    }

    /* Write response body */
    if (jetson_infer_result.response_len > 0) {
        f_write(jetson_infer_result.response, 1,
                jetson_infer_result.response_len, file);
        f_write("\r\n", 1, 2, file);
    }

    /* Write separator */
    f_write("---\r\n", 1, 5, file);

    f_close(file);
    return 1;
}


/******************************************************************************
 sd_dump_file()

 Reads <name> off the SD card and echoes its raw contents to the debug
 console (UART1). Lets the operator read TELEM.CSV / BOOT.LOG / INFER.LOG
 over the serial link without pulling the card (see the 'l' console cmd).

 Cooperative-safe: never yields while the file is open, so task_sdlog cannot
 run and grab the single EFFS-THIN handle (F_MAXFILES=1) mid-dump. Because we
 hold the CPU for the whole dump, task_wdt can't service the external TPS3430,
 so we kick both watchdogs each chunk (extra WDI falling edges are harmless).
******************************************************************************/
void sd_dump_file(const char *name) {

    F_FILE *file;
    long total;
    long n;
    char buf[64];

    if (!sd_initialized) {
        user_debug_msg(STR_TASK_SDLOG "SD not ready; cannot dump.");
        return;
    }

    total = f_filelength(name);
    if (total < 0)
        total = 0;

    sprintf(strTmp, STR_TASK_SDLOG "---- %s (%ld bytes) ----", name, total);
    user_debug_msg(strTmp);

    file = f_open(name, "r");
    if (!file) {
        sprintf(strTmp, STR_TASK_SDLOG "Cannot open %s.", name);
        user_debug_msg(strTmp);
        return;
    }

    while ((n = f_read(buf, 1, (long)(sizeof(buf) - 1), file)) > 0) {
        buf[n] = '\0';
        csk_uart1_puts(buf);
        __builtin_clrwdt();              /* internal PIC24 WDT */
        csk_io7_low(); csk_io7_high();   /* external TPS3430 WDI falling edge */
    }
    f_close(file);

    csk_uart1_puts("\r\n");
    user_debug_msg(STR_TASK_SDLOG "---- end ----");
}


/******************************************************************************
 parse_field()

 Parse a signed decimal integer from a CSV field, advancing past the
 trailing comma/CR/LF (if present). Returns as unsigned int (caller casts
 to int for signed fields).
******************************************************************************/
static unsigned int parse_field(const char **pp) {

    unsigned int val = 0;
    const char *p = *pp;
    int neg = 0;

    if (*p == '-') {
        neg = 1;
        p++;
    }

    while (*p >= '0' && *p <= '9') {
        val = val * 10U + (*p - '0');
        p++;
    }
    if (*p == ',')
        p++;
    if (*p == '\r')
        p++;
    if (*p == '\n')
        p++;
    *pp = p;

    return neg ? (unsigned int)(-(int)val) : val;
}


/******************************************************************************
 sd_read_last_telem()

 Reads the last CSV row from TELEM.CSV and parses it into the provided
 jetson_telem_t struct. Called by task_beacon at startup to restore
 telemetry across reboots.

 Returns 1 on success, 0 on failure (file missing, empty, parse error).
******************************************************************************/
unsigned char sd_read_last_telem(jetson_telem_t *telem) {

    F_FILE *file;
    long fsize;
    long read_start;
    long nread;
    char buf[SDLOG_LINE_MAX];
    char *line;
    int idx;
    const char *p;

    if (!sd_initialized)
        return 0;

    fsize = f_filelength(SDLOG_FILE_NAME);
    if (fsize <= 0)
        return 0;

    file = f_open(SDLOG_FILE_NAME, "r");
    if (!file)
        return 0;

    /* Read last SDLOG_LINE_MAX bytes of the file */
    read_start = fsize - SDLOG_LINE_MAX;
    if (read_start < 0)
        read_start = 0;

    f_seek(file, read_start, F_SEEK_SET);
    nread = f_read(buf, 1, SDLOG_LINE_MAX - 1, file);
    f_close(file);

    if (nread <= 0)
        return 0;

    buf[nread] = '\0';

    /* Find the start of the last complete line.
       Scan backwards from end for second-to-last '\n'. */
    line = NULL;
    for (idx = (int)nread - 2; idx >= 0; idx--) {
        if (buf[idx] == '\n') {
            line = &buf[idx + 1];
            break;
        }
    }
    if (!line) {
        /* Only one line (or partial) — try from start of buffer */
        line = buf;
    }

    /* Skip if it looks like the header */
    if (line[0] == 'u')  /* "uptime,..." */
        return 0;

    /* Parse: uptime,cpu,gpu,cpu_pk,gpu_pk,pwr,ram,infer,load,jetson */
    p = line;

    /* Skip uptime field */
    while (*p && *p != ',') p++;
    if (*p == ',') p++;

    telem->tx.cpu_temp      = (int)parse_field(&p);
    telem->tx.gpu_temp      = (int)parse_field(&p);
    telem->tx.cpu_temp_peak = (int)parse_field(&p);
    telem->tx.gpu_temp_peak = (int)parse_field(&p);
    telem->tx.power_mw      = parse_field(&p);
    telem->tx.ram_mb        = parse_field(&p);
    telem->tx.infer_dur_s   = parse_field(&p);
    telem->tx.model_load_s  = parse_field(&p);
    telem->timestamp_ticks  = 0;
    telem->valid            = 1;
    telem->pending_tx       = 0;   /* restored data isn't fresh enough to TX */

    return 1;
}


/******************************************************************************
****                                                                       ****
**                                                                           **
task_sdlog()

Salvo task entry point. Initializes SD card, then logs telemetry every
60 seconds.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_sdlog(void) {

    unsigned int written;
    unsigned int cycle;
    unsigned long t_mount0;          /* tick stamp when we begin mounting   */
    unsigned int  mount_ms;          /* how long the successful mount took  */

    user_debug_msg(STR_TASK_SDLOG "Starting.");
    OS_Delay(30);    /* brief: let other tasks init (was 2 s) */

    /* Power on SD card and give it a short, spec-adequate settle.
       SD needs only ~1 ms power ramp + a few hundred clocks; 250 ms is plenty
       (was 1 s). */
    csk_sd_pwr_on();
    csk_sd_open();
    OS_Delay(25);    /* 250 ms power stabilization */

    /* Mount filesystem.  Most of the old "warm-up" came from power-cycling the
       slot on every miss, which resets the card's init progress.  Instead try
       f_initvolume a few times back-to-back with short gaps WITHOUT cutting
       power — a card that just needs another attempt is caught in <1 s.
       These are unrolled at the task top level (no inner loop) per the Salvo
       OS_Delay rule. */
    t_mount0 = OSGetTicks();
    if (try_sd_mount())                  sd_initialized = 1;
    if (!sd_initialized) { OS_Delay(10); if (try_sd_mount()) sd_initialized = 1; }
    if (!sd_initialized) { OS_Delay(10); if (try_sd_mount()) sd_initialized = 1; }
    if (!sd_initialized) { OS_Delay(20); if (try_sd_mount()) sd_initialized = 1; }

    if (sd_initialized) {
        mount_ms = (unsigned int)((OSGetTicks() - t_mount0) * 10UL);
        sprintf(strTmp, STR_TASK_SDLOG "SD card initialized (mount %u ms).",
                mount_ms);
        user_debug_msg(strTmp);
        /* Log this boot immediately rather than waiting a 2 s loop tick.
           Boot count itself is reported by the startup banner (task_cmd); this
           is just a confirmation that the SD write succeeded. */
        if (write_boot_log()) {
            sd_boot_logged = 1;
            user_debug_msg(STR_TASK_SDLOG "Logged boot to " SDLOG_BOOT_FILE ".");
        }
    } else {
        user_debug_msg(STR_TASK_SDLOG "SD card init failed, will retry.");
    }

    /* Signal that SD card resource is available */
    OSSignalBinSem(RSRC_SD_CARD_P);

    cycle = SDLOG_WRITE_EVERY;   /* trigger first CSV write immediately */

    while (1) {

        OS_Delay(SDLOG_POLL_TICKS);  /* 2 s — fits in uint8 */

        /* Retry init if not initialized — full power cycle for hot-insert.
           Bounded: after SDLOG_MAX_INIT_RETRIES failures we assume no card is
           present (testing without SD) and stop retrying, so we don't spam the
           debug UART or power-cycle the SD slot forever. */
        if (!sd_initialized) {
            if (sd_gave_up)
                continue;            /* no card — idle quietly */

            csk_sd_close();
            csk_sd_pwr_off();
            OS_Delay(50);    /* 500 ms power-off settle */
            csk_sd_pwr_on();
            csk_sd_open();
            OS_Delay(100);   /* 1 s power stabilization */
            if (try_sd_mount()) {
                sd_initialized = 1;
                user_debug_msg(STR_TASK_SDLOG "SD card initialized (retry).");
            } else if (++sd_init_retries >= SDLOG_MAX_INIT_RETRIES) {
                sd_gave_up = 1;
                csk_sd_close();
                csk_sd_pwr_off();    /* leave the slot powered down */
                user_debug_msg(STR_TASK_SDLOG
                    "No SD card detected; logging disabled for this session.");
            }
            continue;
        }

        /* Record this boot to BOOT.LOG once, the first time the card is up.
           Survives across the initial mount and the hot-insert retry path. */
        if (!sd_boot_logged) {
            if (write_boot_log()) {
                sd_boot_logged = 1;
                user_debug_msg(STR_TASK_SDLOG "Logged boot to " SDLOG_BOOT_FILE ".");
            }
        }

        /* Write inference result to INFER.LOG if one is pending (check every 2s) */
        if (jetson_infer_result.pending) {
            jetson_infer_result.pending = 0;
            if (write_infer_log()) {
                sprintf(strTmp, STR_TASK_SDLOG "Logged inference P%02u (%u bytes).",
                        jetson_infer_result.prompt_id,
                        jetson_infer_result.response_len);
                user_debug_msg(strTmp);
            }
        }

        /* Only write telemetry CSV every ~60 s */
        cycle++;
        if (cycle < SDLOG_WRITE_EVERY)
            continue;
        cycle = 0;

        /* Skip if no real Jetson telemetry yet.
           timestamp_ticks is set to OSGetTicks() (>0) by task_jetson on
           real telemetry, but left at 0 by sd_read_last_telem(). */
        if (jetson_last_telem.timestamp_ticks == 0)
            continue;

        /* Write telemetry row */
        written = write_csv_line();
        if (written > 0) {
            sd_write_count++;
            sprintf(strTmp, STR_TASK_SDLOG "Wrote %u bytes (#%lu).",
                    written, sd_write_count);
            user_debug_msg(strTmp);
        }

    } /* while */

} /* task_sdlog */
