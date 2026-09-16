/******************************************************************************
KENSAT SD Card Telemetry Logger

Mounts the SD card and logs telemetry to TELEM.CSV every 60 s, inference
results to INFER.LOG, boots to BOOT.LOG, and beacon counters to COMMS0/1.DAT.

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
#define SDLOG_WRITE_EVERY     30        /* 30 x 2 s = 60 s between CSV rows */
#define SDLOG_MAX_INIT_RETRIES 3        /* then assume no card and stop */
#define SDLOG_FILE_NAME       "TELEM.CSV"
#define SDLOG_INFER_FILE      "INFER.LOG"
#define SDLOG_BOOT_FILE       "BOOT.LOG"

/* Counter checkpoints alternate between two files.  EFFS-THIN superthin has
   no f_rename, so this is how a power loss mid-write leaves one good copy. */
#define SDLOG_CTR_FILE_A      "COMMS0.DAT"
#define SDLOG_CTR_FILE_B      "COMMS1.DAT"
#define SDLOG_CTR_MAGIC       "KENSAT1"
#define SDLOG_LINE_MAX        128       /* > one CSV row */

/* CSV header (must fit in strTmp).  v3v3sys in mV; gx/gy/gz/tumble in dps. */
#define SDLOG_CSV_HEADER  "uptime,cpu,gpu,cpu_pk,gpu_pk,pwr_mw,ram_mb,infer_s,load_s,jetson,v3v3sys,gx,gy,gz,tumble\r\n"


/* ---- Module state ---- */
static unsigned char sd_initialized = 0;
static unsigned char sd_gave_up     = 0;   /* no card found, stop retrying */
static unsigned char sd_init_retries = 0;
static unsigned long sd_write_count = 0;
static unsigned char sd_boot_logged = 0;

/* Next counter file to write (0 = A, 1 = B) and its serial.  The higher
   serial is the newer copy. */
static unsigned char sd_ctr_next   = 0;
static unsigned long sd_ctr_serial = 0;


/******************************************************************************
 try_sd_mount()

 Mount the FAT16 volume.  Returns 1 on success.
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

 Append "boot #<n> @ <ms>ms" to BOOT.LOG.  Used to check for OBC resets
 after vibration testing.  Returns 1 on success.
******************************************************************************/
static unsigned char write_boot_log(void) {

    F_FILE *file;
    unsigned int len;
    unsigned int written;

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

 Parse "KENSAT1 <serial> <seq> <tx> <rx> <sum>", sum = serial+seq+tx+rx.
 Returns 1 if well formed and the sum matches.  The sum catches a torn
 write.
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
            return 0;
    }

    if (v[4] != (v[0] + v[1] + v[2] + v[3]))
        return 0;

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
        return 0;

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

 Restore the beacon counters from the valid copy with the higher serial.
 Returns SD_CTR_LOADED, SD_CTR_EMPTY or SD_CTR_NO_CARD (see task_sdlog.h).
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

    /* Use the newer copy and overwrite the other one next. */
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

 Write one checkpoint to the older of the two files.  Called on every
 beacon.  Returns 1 on success.
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

    sprintf(strTmp, SDLOG_CTR_MAGIC " %lu %lu %lu %lu %lu\r\n",
            sd_ctr_serial, seq, tx, rx,
            sd_ctr_serial + seq + tx + rx);
    len = strlen(strTmp);

    file = f_open(name, "w");
    if (!file)
        return 0;

    written = (unsigned int)f_write(strTmp, 1, len, file);
    f_close(file);

    if (written != len)
        return 0;

    sd_ctr_next = sd_ctr_next ? 0 : 1;
    return 1;
}


/******************************************************************************
 write_csv_line()

 Append one row to TELEM.CSV.  Returns bytes written (0 on error).
******************************************************************************/
static unsigned int write_csv_line(void) {

    F_FILE *file;
    unsigned int len;
    unsigned int written;
    unsigned long uptime_s;

    uptime_s = OSGetTicks() / 100UL;

    /* Built in two sprintf calls to keep each one small. */
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

    /* Header on first write to an empty file */
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

 Append the inference result to INFER.LOG:
   [<uptime>s] P<id> cpu=<c> gpu=<g> pwr=<p>mW infer=<i>s len=<n>\r\n
   <response text>\r\n
   ---\r\n
 Returns 1 on success, 0 on failure.
******************************************************************************/
static unsigned char write_infer_log(void) {

    F_FILE *file;
    unsigned int hdr_len;
    unsigned int written;

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

    written = (unsigned int)f_write(strTmp, 1, hdr_len, file);
    if (written == 0) {
        f_close(file);
        return 0;
    }

    if (jetson_infer_result.response_len > 0) {
        f_write(jetson_infer_result.response, 1,
                jetson_infer_result.response_len, file);
        f_write("\r\n", 1, 2, file);
    }

    f_write("---\r\n", 1, 5, file);

    f_close(file);
    return 1;
}


/******************************************************************************
 sd_dump_file()

 Print a file from the SD card to the debug console ('l' command).
 Doesn't yield, so nothing else can take the single EFFS-THIN handle
 (F_MAXFILES=1); kicks both watchdogs each chunk instead.
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

 Parse a signed decimal CSV field and skip a trailing ',', CR, LF.
 Returned as unsigned; cast to int for signed fields.
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

 Load the last TELEM.CSV row into *telem.  Used by task_beacon at startup.
 Returns 1 on success.
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

    read_start = fsize - SDLOG_LINE_MAX;
    if (read_start < 0)
        read_start = 0;

    f_seek(file, read_start, F_SEEK_SET);
    nread = f_read(buf, 1, SDLOG_LINE_MAX - 1, file);
    f_close(file);

    if (nread <= 0)
        return 0;

    buf[nread] = '\0';

    /* Last line starts after the second-to-last '\n'. */
    line = NULL;
    for (idx = (int)nread - 2; idx >= 0; idx--) {
        if (buf[idx] == '\n') {
            line = &buf[idx + 1];
            break;
        }
    }
    if (!line) {
        line = buf;
    }

    if (line[0] == 'u')  /* header row */
        return 0;

    /* uptime,cpu,gpu,cpu_pk,gpu_pk,pwr,ram,infer,load,...; skip uptime */
    p = line;
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
    telem->pending_tx       = 0;   /* restored data is not re-sent */

    return 1;
}


/******************************************************************************
****                                                                       ****
**                                                                           **
task_sdlog()

Mount the SD card, then poll every 2 s and write a CSV row every 60 s.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_sdlog(void) {

    unsigned int written;
    unsigned int cycle;
    unsigned long t_mount0;
    unsigned int  mount_ms;

    user_debug_msg(STR_TASK_SDLOG "Starting.");
    OS_Delay(30);

    /* SD needs ~1 ms ramp plus a few hundred clocks; 250 ms is plenty. */
    csk_sd_pwr_on();
    csk_sd_open();
    OS_Delay(25);

    /* Retry the mount a few times without cutting power (power cycling
       resets the card's init).  Unrolled to keep OS_Delay out of an inner
       loop (Salvo). */
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
        if (write_boot_log()) {
            sd_boot_logged = 1;
            user_debug_msg(STR_TASK_SDLOG "Logged boot to " SDLOG_BOOT_FILE ".");
        }
    } else {
        user_debug_msg(STR_TASK_SDLOG "SD card init failed, will retry.");
    }

    OSSignalBinSem(RSRC_SD_CARD_P);

    cycle = SDLOG_WRITE_EVERY;   /* first CSV write immediately */

    while (1) {

        OS_Delay(SDLOG_POLL_TICKS);

        /* Power-cycle and remount (hot insert), up to SDLOG_MAX_INIT_RETRIES. */
        if (!sd_initialized) {
            if (sd_gave_up)
                continue;

            csk_sd_close();
            csk_sd_pwr_off();
            OS_Delay(50);    /* 500 ms off */
            csk_sd_pwr_on();
            csk_sd_open();
            OS_Delay(100);   /* 1 s settle */
            if (try_sd_mount()) {
                sd_initialized = 1;
                user_debug_msg(STR_TASK_SDLOG "SD card initialized (retry).");
            } else if (++sd_init_retries >= SDLOG_MAX_INIT_RETRIES) {
                sd_gave_up = 1;
                csk_sd_close();
                csk_sd_pwr_off();
                user_debug_msg(STR_TASK_SDLOG
                    "No SD card detected; logging disabled for this session.");
            }
            continue;
        }

        /* BOOT.LOG once per boot, if the card came up late */
        if (!sd_boot_logged) {
            if (write_boot_log()) {
                sd_boot_logged = 1;
                user_debug_msg(STR_TASK_SDLOG "Logged boot to " SDLOG_BOOT_FILE ".");
            }
        }

        if (jetson_infer_result.pending) {
            jetson_infer_result.pending = 0;
            if (write_infer_log()) {
                sprintf(strTmp, STR_TASK_SDLOG "Logged inference P%02u (%u bytes).",
                        jetson_infer_result.prompt_id,
                        jetson_infer_result.response_len);
                user_debug_msg(strTmp);
            }
        }

        cycle++;
        if (cycle < SDLOG_WRITE_EVERY)
            continue;
        cycle = 0;

        /* No fresh Jetson telemetry yet (sd_read_last_telem() leaves
           timestamp_ticks at 0). */
        if (jetson_last_telem.timestamp_ticks == 0)
            continue;

        written = write_csv_line();
        if (written > 0) {
            sd_write_count++;
            sprintf(strTmp, STR_TASK_SDLOG "Wrote %u bytes (#%lu).",
                    written, sd_write_count);
            user_debug_msg(strTmp);
        }

    } /* while */

} /* task_sdlog */
