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

#ifndef CSK_EFFS_THIN_LIB
#define CSK_EFFS_THIN_LIB 4
#endif
#include "thin_usr.h"

#include "salvo.h"

#include <string.h>


/* ---- Configuration ---- */
#define SDLOG_POLL_TICKS      200       /* 2 s per iteration (fits uint8) */
#define SDLOG_WRITE_EVERY     30        /* 30 × 2 s = 60 s between CSV rows */
#define SDLOG_FILE_NAME       "TELEM.CSV"
#define SDLOG_INFER_FILE      "INFER.LOG"
#define SDLOG_LINE_MAX        128       /* must hold one whole CSV row + slack */

/* CSV header (must fit in strTmp).
   v5sys / v3v3sys / vbatt are EPS rail voltages in mV.
   tumble columns mirror the beacon: per-axis gyro dps + magnitude. */
#define SDLOG_CSV_HEADER  "uptime,cpu,gpu,cpu_pk,gpu_pk,pwr_mw,ram_mb,infer_s,load_s,jetson,vbatt,v5sys,v3v3sys,gx,gy,gz,tumble\r\n"


/* ---- Module state ---- */
static unsigned char sd_initialized = 0;
static unsigned long sd_write_count = 0;




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
    sprintf(strTmp + strlen(strTmp), "%u,%u,%u,%d,%d,%d,%u\r\n",
            g_vbatt_mv,
            g_rail_5v_mv,
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

    user_debug_msg(STR_TASK_SDLOG "Starting.");
    OS_Delay(200);   /* let other tasks init first */

    /* Power on SD card and wait for it to stabilize */
    csk_sd_pwr_on();
    csk_sd_open();
    user_debug_msg(STR_TASK_SDLOG "SD powered on, waiting...");
    OS_Delay(100);   /* 1 s for SD card power stabilization */

    /* Mount filesystem */
    if (try_sd_mount()) {
        sd_initialized = 1;
        user_debug_msg(STR_TASK_SDLOG "SD card initialized.");
    } else {
        user_debug_msg(STR_TASK_SDLOG "SD card init failed, will retry.");
    }

    /* Signal that SD card resource is available */
    OSSignalBinSem(RSRC_SD_CARD_P);

    cycle = SDLOG_WRITE_EVERY;   /* trigger first CSV write immediately */

    while (1) {

        OS_Delay(SDLOG_POLL_TICKS);  /* 2 s — fits in uint8 */

        /* Retry init if not initialized — full power cycle for hot-insert */
        if (!sd_initialized) {
            csk_sd_close();
            csk_sd_pwr_off();
            OS_Delay(50);    /* 500 ms power-off settle */
            csk_sd_pwr_on();
            csk_sd_open();
            OS_Delay(100);   /* 1 s power stabilization */
            if (try_sd_mount()) {
                sd_initialized = 1;
                user_debug_msg(STR_TASK_SDLOG "SD card initialized (retry).");
            }
            continue;
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
