/******************************************************************************
KENSAT - Si4463 Radio Transceiver RX Task

Si4463 driver over SPI2: init, RF config, AX.25 downlink, uplink RX.

SPI2 Pin Assignments (PIC24FJ256GB210 -> ESP32-S3 via CubeSat Kit Bus):
  SDO2 (MOSI) = RG8 / RP19  (IO.9,  H1-15) -> ESP32 GPIO 6
  SDI2 (MISO) = RG7 / RP26  (IO.10, H1-14) -> ESP32 GPIO 13
  SCK2 (CLK)  = RG6 / RP21  (IO.11, H1-13) -> ESP32 GPIO 5
  nSEL (CS)   = RG9         (IO.8,  H1-16) -> ESP32 GPIO 10, active low
  SDN          = RE8         (IO.30, H2-18) -> ESP32 GPIO 14, active high = shutdown
  nIRQ         = RE9         (IO.31, H2-17) -> ESP32 GPIO 9,  active low

******************************************************************************/
#include "main.h"
#include "task_si4463.h"
#include "ax25.h"

// Pumpkin CubeSat Kit headers
#include "csk_hw.h"
#include "csk_uart.h"
#include "csk_usb.h"

// KENSAT event definitions (for USB/MHX semaphore)
#include "events.h"

// Microchip PIC24F Peripheral Library
#include <spi.h>


// IOL1WAY=ON would lock PPS after the first unlock and block the SPI2 mapping.
_CONFIG3(IOL1WAY_OFF)

// Pumpkin Salvo headers
#include "salvo.h"


/******************************************************************************
 Si4463 Command Definitions
******************************************************************************/
#define SI4463_CMD_PART_INFO      0x01
#define SI4463_CMD_POWER_UP       0x02
#define SI4463_CMD_FUNC_INFO      0x10
#define SI4463_CMD_SET_PROPERTY   0x11
#define SI4463_CMD_GET_PROPERTY   0x12
#define SI4463_CMD_GPIO_PIN_CFG   0x13
#define SI4463_CMD_FIFO_INFO      0x15
#define SI4463_CMD_GET_INT_STATUS 0x20
#define SI4463_CMD_START_TX       0x31
#define SI4463_CMD_START_RX       0x32
#define SI4463_CMD_REQUEST_DEVICE_STATE 0x33
#define SI4463_CMD_CHANGE_STATE   0x34
#define SI4463_CMD_GET_MODEM_STATUS 0x22
#define SI4463_CMD_READ_CMD_BUFF  0x44
#define SI4463_CMD_FRR_A_READ    0x50
#define SI4463_CMD_FRR_B_READ    0x51
#define SI4463_CMD_WRITE_TX_FIFO  0x66
#define SI4463_CMD_READ_RX_FIFO   0x77

// Si4463 operating states
#define SI4463_STATE_NOCHANGE     0x00
#define SI4463_STATE_SLEEP        0x01
#define SI4463_STATE_SPI_ACTIVE   0x02
#define SI4463_STATE_READY        0x03
#define SI4463_STATE_TX_TUNE      0x05
#define SI4463_STATE_RX_TUNE      0x06
#define SI4463_STATE_TX           0x07
#define SI4463_STATE_RX           0x08

// CTS (Clear To Send) response byte
#define SI4463_CTS               0xFF

// Pin control macros  (matched to CubeSat Kit bus wiring)
#define SI4463_CS_LOW()           (_LATG9 = 0)    // IO.8, H1-16
#define SI4463_CS_HIGH()          (_LATG9 = 1)
#define SI4463_SDN_LOW()          (_LATE8 = 0)    // IO.30, H2-18
#define SI4463_SDN_HIGH()         (_LATE8 = 1)
#define SI4463_NIRQ_READ()        (_RE9)           // IO.31, H2-17, active low

// Timeout for CTS polling
#define SI4463_CTS_TIMEOUT        10000


/******************************************************************************
 UART Debug Output
******************************************************************************/
static void si4463_debug_msg(const char *msg) {
    user_debug_msg(msg);
}


static void spi2_init(void) {

    // init.c leaves CS and SDN low and RG7 as output; fix that first.
    _LATG9 = 1;    // CS HIGH (deselect)
    _LATE8 = 1;    // SDN HIGH (shutdown Si4463)
    _TRISG7 = 1;   // MISO as input (stop driving against Si4463)

    // RE8 (SDN) = AN24, RE9 (nIRQ) = AN25: force digital.
    ANSG = 0x0000;
    ANSE = 0x0000;

    // Push-pull, not open-drain
    ODCG = 0x0000;
    ODCE = 0x0000;

    SI4463_CS_HIGH();
    SI4463_SDN_HIGH();

    _TRISG9 = 0;    // nSEL (CS) as output (manual, not SPI SS)
    _TRISG7 = 1;    // MISO as input
    _TRISE8 = 0;    // SDN as output
    _TRISE9 = 1;    // nIRQ as input

    SPI2STATbits.SPIEN = 0;

    __builtin_write_OSCCONL(OSCCON & ~(1 << 6));   // Unlock PPS
    RPOR9bits.RP19R  = 10;   // RP19 (RG8) = SDO2 (MOSI)
    RPOR10bits.RP21R = 11;   // RP21 (RG6) = SCK2OUT (SCLK)
    RPINR22bits.SDI2R = 26;  // SDI2 input from RP26 (RG7)
    __builtin_write_OSCCONL(OSCCON | (1 << 6));    // Lock PPS

    // SPI2: master, mode 0, 8-bit, ~1 MHz (16 MHz / 4 / 4). Kept slow
    // because the bus crosses two connectors.
    SPI2CON1 = 0;
    SPI2CON1bits.MSTEN = 1;      // Master mode
    SPI2CON1bits.CKP   = 0;      // Clock idle LOW (CPOL=0)
    SPI2CON1bits.CKE   = 1;      // Data changes on idle->active (Mode 0)
    SPI2CON1bits.PPRE  = 0b10;   // Primary prescale 4:1
    SPI2CON1bits.SPRE  = 0b100;  // Secondary prescale 4:1
    SPI2CON2 = 0x0000;
    SPI2STATbits.SPIROV = 0;     // Clear overflow
    SPI2STATbits.SPIEN = 1;      // Enable SPI2

}


static unsigned char spi2_transfer(unsigned char data) {
    SPI2STATbits.SPIROV = 0;
    SPI2BUF = data;
    while (!SPI2STATbits.SPIRBF) {}
    return (unsigned char)SPI2BUF;
}



/******************************************************************************
 Si4463 Command Functions
******************************************************************************/
static void si4463_send_cmd(const unsigned char *cmd, unsigned int len) {
    unsigned int i;

    SI4463_CS_LOW();
    // CS setup time for the ESP32 SPI slave (bench setup) to see CS low
    // before the first SCLK edge.
    { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
    for (i = 0; i < len; i++) {
        spi2_transfer(cmd[i]);
    }
    SI4463_CS_HIGH();
}


static unsigned char si4463_wait_cts(void) {
    unsigned int timeout = SI4463_CTS_TIMEOUT;
    unsigned char cts;

    while (timeout--) {
        __builtin_clrwdt();

        { unsigned int d; for (d = 0; d < 100; d++) { __builtin_nop(); } }

        SI4463_CS_LOW();
        { unsigned int d; for (d = 0; d < 10; d++) { __builtin_nop(); } }
        spi2_transfer(SI4463_CMD_READ_CMD_BUFF);
        cts = spi2_transfer(0x00);
        SI4463_CS_HIGH();

        if (cts == SI4463_CTS) {
            return 1;
        }
    }
    return 0;
}


/******************************************************************************
 si4463_read_response()

 Polls READ_CMD_BUFF for CTS, then reads resp_len response bytes in the same
 CS transaction (the response is lost once CS goes high).
 Returns 1 on success, 0 on CTS timeout.
******************************************************************************/
static unsigned char si4463_read_response(unsigned char *resp,
                                          unsigned char resp_len) {
    unsigned int timeout = SI4463_CTS_TIMEOUT;
    unsigned char cts;
    unsigned char i;

    while (timeout--) {
        __builtin_clrwdt();

        { unsigned int d; for (d = 0; d < 1000; d++) { __builtin_nop(); } }

        SI4463_CS_LOW();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
        spi2_transfer(SI4463_CMD_READ_CMD_BUFF);
        cts = spi2_transfer(0x00);

        if (cts == SI4463_CTS) {
            for (i = 0; i < resp_len; i++) {
                resp[i] = spi2_transfer(0x00);
            }
            SI4463_CS_HIGH();
            return 1;
        }

        SI4463_CS_HIGH();
    }
    return 0;
}


/******************************************************************************
 si4463_read_frr()

 Reads Fast Response Registers A and B. No CTS needed, so much faster
 than GET_INT_STATUS.
******************************************************************************/
static void si4463_read_frr(unsigned char *frr_a, unsigned char *frr_b) {
    SI4463_CS_LOW();
    { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
    spi2_transfer(SI4463_CMD_FRR_A_READ);
    *frr_a = spi2_transfer(0x00);
    *frr_b = spi2_transfer(0x00);
    SI4463_CS_HIGH();
}


static void si4463_reset(void) {
    unsigned int i;
    unsigned char j;

    __builtin_clrwdt();

    // Flush any stale SPI state
    for (j = 0; j < 16; j++) {
        SI4463_CS_LOW();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
        spi2_transfer(0xFF);
        SI4463_CS_HIGH();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
    }

    // Hold SDN high (shutdown) for >1 ms
    SI4463_SDN_HIGH();
    __builtin_clrwdt();
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }

    SI4463_SDN_LOW();

    // Wait for POR
    __builtin_clrwdt();
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    __builtin_clrwdt();
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    __builtin_clrwdt();
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    __builtin_clrwdt();
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }
}


// Si4463 Rev C2 (ROMID=0x06) patch, from the tinySA project
// (PATCHID=0xCA90, 512 B). Not applied: see si4463_power_up().
static const unsigned char si4463_patch_cmds[][9] = {
    {0x04,0x21,0x71,0x4B,0x00,0x00,0xDC,0x95,0x00},
    {0x05,0xA6,0x22,0x21,0xF0,0x41,0x5B,0x26,0x00},
    {0x05,0x1E,0x12,0xBD,0x5A,0xC2,0x52,0x41,0x00},  // dummy last byte
};
#define SI4463_PATCH_CMD_COUNT  64

static unsigned char si4463_apply_patch(void) {
    // 64 patch commands of 8 bytes; each is sent with a 0x08 length prefix
    static const unsigned char patch[] = {
        0x04,0x21,0x71,0x4B,0x00,0x00,0xDC,0x95,
        0x05,0xA6,0x22,0x21,0xF0,0x41,0x5B,0x26,
        0xE2,0x2F,0x1C,0xBB,0x0A,0xA8,0x94,0x28,
        0x05,0x87,0x67,0xE2,0x58,0x1A,0x07,0x5B,
        0xE1,0xD0,0x72,0xD8,0x8A,0xB8,0x5B,0x7D,
        0x05,0x11,0xEC,0x9E,0x28,0x23,0x1B,0x6D,
        0xE2,0x4F,0x8A,0xB2,0xA9,0x29,0x14,0x13,
        0x05,0xD1,0x2E,0x71,0x6A,0x51,0x4C,0x2C,
        0xE5,0x80,0x27,0x42,0xA4,0x69,0xB0,0x7F,
        0x05,0xAA,0x81,0x2A,0xBD,0x45,0xE8,0xA8,
        0xEA,0xE4,0xF0,0x24,0xC9,0x9F,0xCC,0x3C,
        0x05,0x08,0xF5,0x05,0x04,0x27,0x62,0x98,
        0xEA,0x6B,0x62,0x84,0xA1,0xF9,0x4A,0xE2,
        0x05,0xE9,0x77,0x05,0x4F,0x84,0xEE,0x35,
        0xE2,0x43,0xC3,0x8D,0xFB,0xAD,0x54,0x25,
        0x05,0x14,0x06,0x5E,0x39,0x36,0x2F,0x45,
        0xEA,0x0C,0x1C,0x74,0xD0,0x11,0xFC,0x32,
        0x05,0xDA,0x38,0xBA,0x0E,0x3C,0xE7,0x8B,
        0xEA,0xB0,0x09,0xE6,0xFF,0x94,0xBB,0xA9,
        0x05,0xD7,0x11,0x29,0xFE,0xDC,0x71,0xD5,
        0xEA,0x7F,0x83,0xA7,0x60,0x90,0x62,0x18,
        0x05,0x84,0x7F,0x6A,0xD1,0x91,0xC6,0x52,
        0xEA,0x2A,0xD8,0x7B,0x8E,0x4A,0x9F,0x91,
        0x05,0xBD,0xAA,0x9D,0x16,0x18,0x06,0x15,
        0xE2,0x55,0xAD,0x2D,0x0A,0x14,0x1F,0x5D,
        0x05,0xD3,0xE0,0x7C,0x39,0xCF,0x01,0xF0,
        0xEF,0x3A,0x91,0x72,0x6A,0x03,0xBB,0x96,
        0xE7,0x83,0x6D,0xA4,0x92,0xFC,0x13,0xA7,
        0xEF,0xF8,0xFD,0xCF,0x62,0x07,0x6F,0x1E,
        0xE7,0x4C,0xEA,0x4A,0x75,0x4F,0xD6,0xCF,
        0xE2,0xF6,0x11,0xE4,0x26,0x0D,0x4D,0xC6,
        0x05,0xFB,0xBF,0xE8,0x07,0x89,0xC3,0x51,
        0xEF,0x82,0x27,0x04,0x3F,0x96,0xA8,0x58,
        0xE7,0x41,0x29,0x3C,0x75,0x2A,0x03,0x1C,
        0xEF,0xAF,0x59,0x98,0x36,0xAA,0x0F,0x06,
        0xE6,0xF6,0x93,0x41,0x2D,0xEC,0x0E,0x99,
        0x05,0x29,0x19,0x90,0xE5,0xAA,0x36,0x40,
        0xE7,0xFB,0x68,0x10,0x7D,0x77,0x5D,0xC0,
        0xE7,0xCB,0xB4,0xDD,0xCE,0x90,0x54,0xBE,
        0xE7,0x72,0x8A,0xD6,0x02,0xF4,0xDD,0xCC,
        0xE7,0x6A,0x21,0x0B,0x02,0x86,0xEC,0x15,
        0xE7,0x7B,0x7C,0x3D,0x6B,0x81,0x03,0xD0,
        0xEF,0x7D,0x61,0x36,0x94,0x7C,0xA0,0xDF,
        0xEF,0xCC,0x85,0x3B,0xDA,0xE0,0x5C,0x1C,
        0xE7,0xE3,0x75,0xBB,0x39,0x22,0x4B,0xA8,
        0xEF,0xF9,0xCE,0xE0,0x5E,0xEB,0x1D,0xCB,
        0xE7,0xBD,0xE2,0x70,0xD5,0xAB,0x4E,0x3F,
        0xE7,0xB7,0x8D,0x20,0x68,0x6B,0x09,0x52,
        0xEF,0xA1,0x1B,0x90,0xCD,0x98,0x00,0x63,
        0xEF,0x54,0x67,0x5D,0x9C,0x11,0xFC,0x45,
        0xE7,0xD4,0x9B,0xC8,0x97,0xBE,0x8A,0x07,
        0xEF,0x52,0x8D,0x90,0x63,0x73,0xD5,0x2A,
        0xEF,0x03,0xBC,0x6E,0x1C,0x76,0xBE,0x4A,
        0xE7,0xC2,0xED,0x67,0xBA,0x5E,0x66,0x21,
        0xEF,0xE7,0x3F,0x87,0xBE,0xE0,0x7A,0x6D,
        0xE7,0xC9,0x70,0x93,0x1D,0x64,0xF5,0x6C,
        0xEF,0xF5,0x28,0x08,0x34,0xB3,0xB6,0x2C,
        0xEF,0x3A,0x0A,0xEC,0x0F,0xDB,0x56,0xCA,
        0xEF,0x39,0xA0,0x6E,0xED,0x79,0xD0,0x24,
        0xE7,0x6C,0x0B,0xAF,0xA9,0x4E,0x40,0xB5,
        0xE9,0xB9,0xAF,0xBF,0x25,0x50,0xD1,0x37,
        0x05,0x9E,0xDB,0xDE,0x3F,0x94,0xE9,0x6B,
        0xEC,0xC5,0x05,0xAA,0x57,0xDC,0x8A,0x5E,
        0x05,0x70,0xDA,0x84,0x84,0xDD,0xCA,0x90
    };
    unsigned char i;
    unsigned char cmd[9];

    for (i = 0; i < 64; i++) {
        cmd[0] = 0x08;
        {
            unsigned char j;
            for (j = 0; j < 8; j++) cmd[1+j] = patch[i*8 + j];
        }
        si4463_send_cmd(cmd, 9);
        if (!si4463_wait_cts()) return 0;
        if ((i & 0x0F) == 0) __builtin_clrwdt();
    }
    return 1;
}

static unsigned char si4463_power_up(void) {
    unsigned char cmd[7];

    si4463_reset();

    si4463_wait_cts();

    // POWER_UP: FUNC=1, crystal, 30 MHz. No patch: the test patch for
    // ROMID=0x06 broke the chip; a production patch from WDS is needed.
    cmd[0] = SI4463_CMD_POWER_UP;
    cmd[1] = 0x01;        // FUNC=1 (no patch)
    cmd[2] = 0x00;        // XTAL_OPTIONS: crystal mode
    cmd[3] = 0x01;        // XO_FREQ[31:24] = 30 MHz (0x01C9C380)
    cmd[4] = 0xC9;        // XO_FREQ[23:16]
    cmd[5] = 0xC3;        // XO_FREQ[15:8]
    cmd[6] = 0x80;        // XO_FREQ[7:0]

    si4463_send_cmd(cmd, 7);
    return si4463_wait_cts();
}


/******************************************************************************
 si4463_part_info()

 Prints PART_INFO to UART. A real Si4463 reports part=0x4463; 0xFFFF means
 nothing is answering on SPI.
******************************************************************************/
static void si4463_part_info(void) {
    unsigned char cmd[1];
    unsigned char resp[8];

    cmd[0] = SI4463_CMD_PART_INFO;
    si4463_send_cmd(cmd, 1);

    if (!si4463_read_response(resp, 8)) {
        si4463_debug_msg(STR_TASK_SI4463 "PART_INFO: CTS timeout");
        return;
    }

    // resp: [0]=CHIPREV [1..2]=PART [3]=PBUILD [4..5]=ID [6]=CUSTOMER [7]=ROMID
    sprintf(strTmp, STR_TASK_SI4463 "PART_INFO: part=0x%02X%02X, rev=0x%02X, id=0x%02X%02X",
            resp[1], resp[2], resp[0], resp[3], resp[4]);
    si4463_debug_msg(strTmp);
}


static unsigned char si4463_start_rx(void) {
    unsigned char cmd[8];

    cmd[0] = SI4463_CMD_START_RX;
    cmd[1] = 0x00;        // Channel 0
    cmd[2] = 0x00;        // Condition: start immediately
    cmd[3] = 0x00;        // RX_LEN[15:8] = 0 (use packet handler)
    cmd[4] = 0x00;        // RX_LEN[7:0]
    cmd[5] = SI4463_STATE_NOCHANGE;  // RXTIMEOUT_STATE
    cmd[6] = SI4463_STATE_READY;     // RXVALID_STATE, host re-arms RX
    cmd[7] = SI4463_STATE_READY;     // RXINVALID_STATE, host re-arms RX

    si4463_send_cmd(cmd, 8);
    return si4463_wait_cts();
}


static unsigned char si4463_start_tx(void) {
    unsigned char cmd[7];

    cmd[0] = SI4463_CMD_START_TX;
    cmd[1] = 0x00;        // Channel 0
    cmd[2] = 0x30;        // Condition: start immediately, READY on complete
    cmd[3] = 0x00;        // TX_LEN[15:8] = 0 (use packet handler)
    cmd[4] = 0x00;        // TX_LEN[7:0]
    cmd[5] = 0x00;        // TX_DELAY[15:8]
    cmd[6] = 0x00;        // TX_DELAY[7:0]

    si4463_send_cmd(cmd, 7);
    return si4463_wait_cts();
}


static unsigned char si4463_change_state(unsigned char new_state) {
    unsigned char cmd[2];

    cmd[0] = SI4463_CMD_CHANGE_STATE;
    cmd[1] = new_state;

    si4463_send_cmd(cmd, 2);
    return si4463_wait_cts();
}


/******************************************************************************
 si4463_set_property() / si4463_set_properties()

 SET_PROPERTY: one property, or up to 12 consecutive ones.
******************************************************************************/
static unsigned char si4463_set_property(unsigned char group,
                                         unsigned char index,
                                         unsigned char value) {
    unsigned char cmd[5];

    cmd[0] = SI4463_CMD_SET_PROPERTY;
    cmd[1] = group;
    cmd[2] = 1;       // number of properties
    cmd[3] = index;
    cmd[4] = value;

    si4463_send_cmd(cmd, 5);
    return si4463_wait_cts();
}


static unsigned char si4463_set_properties(unsigned char group,
                                           unsigned char start_index,
                                           const unsigned char *values,
                                           unsigned char count) {
    unsigned char cmd[16];  // max 12 properties per SET_PROPERTY call
    unsigned char i;

    if (count > 12) count = 12;

    cmd[0] = SI4463_CMD_SET_PROPERTY;
    cmd[1] = group;
    cmd[2] = count;
    cmd[3] = start_index;
    for (i = 0; i < count; i++) {
        cmd[4 + i] = values[i];
    }

    si4463_send_cmd(cmd, 4 + count);
    return si4463_wait_cts();
}


/******************************************************************************
 si4463_configure_rf()

 RF configuration: 437.08 MHz 2-GFSK, from WDS 3.2.11.0, ported from
 radio/firmware/esp32_si4463/si4463.cpp.

 Link rates are asymmetric: TX (downlink) 1200 bps +/-601 Hz, RX (uplink)
 9600 bps +/-3004 Hz. The downlink was slowed for ~9 dB of margin because
 the PA measures +12 dBm at PA_PWR_LVL=0x7F (match network loss), and
 9600 bps does not close at 520 km. The uplink is not power-limited.

 TX rate/deviation are MODEM_DATA_RATE (0x20/0x03..05), MODEM_FREQ_DEV
 (0x20/0x0C) and TX_NCO_MODE (0x20/0x06..09). RX recovery is the BCR/AFC
 blocks (0x20/0x18.., 0x20/0x24, 0x20/0x30) and CHFLT taps (0x21/0x00..23),
 so the TX settings do not affect uplink demodulation. To move the uplink
 to 1200 bps, regenerate BCR/AFC/CHFLT in WDS as a set (CHFLT are FIR taps;
 don't hand-edit) and change ground-station/pluto_tx.py to match.
******************************************************************************/
static unsigned char si4463_configure_rf(void) {

    __builtin_clrwdt();

    // Changes from the WDS config for the AX.25/G3RUH downlink:
    //   TX rate 9600 -> 1200 bps (DATA_RATE 0x05DC00 -> 0x00BB80)
    //   TX deviation +/-4.8 kHz -> +/-601 Hz (FREQ_DEV 0x150 -> 0x2A)
    //   PREAMBLE_TX_LENGTH = 0, SYNC SKIP_TX = 1, FIELD_1_LENGTH = 0 and
    //   no chip CRC: the AX.25 encoder supplies all framing in the FIFO.
    // The sync word is still used for uplink RX.

    // GLOBAL
    si4463_set_property(0x00, 0x00, 0x7F);  // XO_TUNE
    si4463_set_property(0x00, 0x03, 0x20);  // GLOBAL_CONFIG: high-perf

    // INT_CTL: enable PH + MODEM interrupts
    {
        static const unsigned char d[] = { 0x03, 0x38, 0x30 };
        // ENABLE: PH + MODEM
        // PH:     PACKET_SENT + PACKET_RX + CRC_ERROR
        // MODEM:  INVALID_SYNC + RSSI_JUMP (si4463_pkt_cfg_rx() swaps in
        //         PREAMBLE/SYNC_DETECT while listening)
        si4463_set_properties(0x01, 0x00, d, 3);
    }

    // FRR_CTL: FRR_A = PH_PEND (0x04), FRR_B = MODEM_PEND (0x07)
    {
        static const unsigned char d[] = { 0x04, 0x07, 0x00, 0x00 };
        si4463_set_properties(0x02, 0x00, d, 4);
    }

    // PREAMBLE: TX length 0; AX.25 sends its own flag preamble.
    {
        static const unsigned char d[] = { 0x00, 0x14, 0x00, 0x0F, 0x31, 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x10, 0x00, d, 9);
    }

    __builtin_clrwdt();

    // SYNC: SKIP_TX = 1, no sync word on TX.
    {
        static const unsigned char d[] = { 0xB1, 0xB4, 0x2B, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x11, 0x00, d, 6);
    }

    // PKT 0x00-0x0B. PKT_CRC_CONFIG=0x05: CRC-16 CCITT (poly 0x1021),
    // seed 0, same as crc16_itu_t in pluto_rx.py.
    {
        static const unsigned char d[] = { 0x05, 0x01, 0x08, 0xFF, 0xFF, 0x20, 0x02, 0x00, 0x2A, 0x01, 0x00, 0x30 };
        si4463_set_properties(0x12, 0x00, d, 12);
    }

    // PKT 0x0C-0x17 (fields): FIELD_1_LENGTH = 0, no CRC, raw FIFO bytes.
    // si4463_tx_ax25() sets FIELD_2_LENGTH per frame.
    {
        static const unsigned char d[] = { 0x30, 0x00, 0x00, 0x04, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x12, 0x0C, d, 12);
    }

    // PKT 0x18-0x23
    {
        static const unsigned char d[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x12, 0x18, d, 12);
    }

    __builtin_clrwdt();

    // PKT 0x24-0x2F
    {
        static const unsigned char d[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x12, 0x24, d, 12);
    }

    // PKT 0x30-0x34
    {
        static const unsigned char d[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x12, 0x30, d, 5);
    }

    // PKT CRC seed 0x36-0x39
    {
        static const unsigned char d[] = { 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x12, 0x36, d, 4);
    }

    __builtin_clrwdt();

    // MODEM 0x00-0x0B (TX side). With TXOSR = 40 and NCOMOD = 30 MHz,
    // Rb = DATA_RATE / 40, so 1200 bps is DATA_RATE = 48000 = 0x00BB80
    // (WDS had 0x05DC00 for 9600).
    {
        static const unsigned char d[] = { 0x03, 0x00, 0x07, 0x00, 0xBB, 0x80, 0x05, 0xC9, 0xC3, 0x80, 0x00, 0x00 };
        si4463_set_properties(0x20, 0x00, d, 12);
    }
    // FREQ_DEV: 14.31 Hz/unit on this board. 0x2A = 42 -> +/-601 Hz,
    // h = 1.0 at 1200 bps. h = 1.0 beat h = 0.6 (+/-360 Hz) through the
    // satnogs_fsk_ax25 flowgraph (frames decoded out of 20 / out of 10):
    //
    //   Eb/N0        16 dB    13 dB    11 dB      carrier offset  0 Hz  500 Hz
    //   +/-360 Hz    16/20    11/20     3/20                     10/10   1/10
    //   +/-600 Hz    19/20    18/20     7/20                     10/10   7/10
    //
    // DELFI-PQ (1200 bps G3RUH) uses the same deviation. Keep this in step
    // with `deviation:` in KENSAT's SatNOGS SatYAML entry, which sets
    // gr-satellites' pre-demod filter and demod gain.
    si4463_set_property(0x20, 0x0C, 0x2A);

    // MODEM 0x18-0x23
    {
        static const unsigned char d[] = { 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0x20, 0x20, 0x00, 0xE8, 0x01, 0x87 };
        si4463_set_properties(0x20, 0x18, d, 12);
    }

    // MODEM 0x24-0x2F
    {
        static const unsigned char d[] = { 0x01, 0x4F, 0x8B, 0x01, 0x4F, 0x02, 0x00, 0x00, 0x00, 0x12, 0x80, 0x54 };
        si4463_set_properties(0x20, 0x24, d, 12);
    }

    __builtin_clrwdt();

    // MODEM AFC limiter + misc
    {
        static const unsigned char d[] = { 0x09, 0x54, 0xE0 };
        si4463_set_properties(0x20, 0x30, d, 3);
    }

    si4463_set_property(0x20, 0x35, 0xE0);

    // MODEM AGC + FSK4 + OOK
    {
        static const unsigned char d[] = { 0x11, 0x56, 0x56, 0x80, 0x1A, 0x40, 0x00, 0x00, 0x2A, 0x0C, 0xA4, 0x23 };
        si4463_set_properties(0x20, 0x38, d, 12);
    }

    __builtin_clrwdt();

    // MODEM RAW + RSSI
    {
        static const unsigned char d[] = { 0x03, 0x00, 0x35, 0x02, 0x00, 0x14, 0x06, 0x00, 0x18, 0x40 };
        si4463_set_properties(0x20, 0x45, d, 10);
    }

    {
        static const unsigned char d[] = { 0x84, 0x0A };
        si4463_set_properties(0x20, 0x50, d, 2);
    }
    {
        static const unsigned char d[] = { 0x03, 0x07 };
        si4463_set_properties(0x20, 0x54, d, 2);
    }
    si4463_set_property(0x20, 0x57, 0x00);

    __builtin_clrwdt();

    // MODEM DSA: enabled, fast decay for inter-packet reset
    {
        static const unsigned char d[] = { 0x40, 0x04, 0x06, 0x20, 0x20 };
        si4463_set_properties(0x20, 0x5B, d, 5);
    }

    // CHFLT RX1 COE13-2
    {
        static const unsigned char d[] = { 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C };
        si4463_set_properties(0x21, 0x00, d, 12);
    }

    // CHFLT RX1 COE1-0 + COEM + RX2 COE13-8
    {
        static const unsigned char d[] = { 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5 };
        si4463_set_properties(0x21, 0x0C, d, 12);
    }

    __builtin_clrwdt();

    // CHFLT RX2 COE7-0 + COEM
    {
        static const unsigned char d[] = { 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00 };
        si4463_set_properties(0x21, 0x18, d, 12);
    }

    // PA
    {
        static const unsigned char d[] = { 0x08, 0x7F, 0x00, 0x1D };
        si4463_set_properties(0x22, 0x00, d, 4);
    }

    // SYNTH
    {
        static const unsigned char d[] = { 0x2C, 0x0E, 0x0B, 0x04, 0x0C, 0x73, 0x03 };
        si4463_set_properties(0x23, 0x00, d, 7);
    }

    __builtin_clrwdt();

    // MATCH
    {
        static const unsigned char d[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x30, 0x00, d, 12);
    }

    // FREQ_CONTROL: FRAC corrects for the crystal running high (no load
    // caps). Measured CW on this board:
    //   FRAC 0x09C457 (640087) -> 436.6492 MHz
    //   FRAC 0x09FF76 (655222) -> 436.8658 MHz
    // Slope ~14.31 Hz/unit, so 437.080 MHz is FRAC = 670191 = 0x0A39EF.
    // Board-specific: on a new board re-measure CW and adjust by
    // (437.080 - meas_MHz) * 69900.
    {
        static const unsigned char d[] = { 0x39, 0x0A, 0x39, 0xEF, 0x05, 0x76, 0x20, 0xFE };
        si4463_set_properties(0x40, 0x00, d, 8);
    }

    return 1;
}


/******************************************************************************
 si4463_gpio_cfg_rx() / si4463_gpio_cfg_tx()

 AS179 RF switch via Si4463 GPIO2/GPIO3.
 RX: GPIO2=high, GPIO3=low
 TX: GPIO2=low,  GPIO3=high
******************************************************************************/
static unsigned char si4463_gpio_cfg_rx(void) {
    unsigned char cmd[8];
    unsigned char resp[7];

    cmd[0] = SI4463_CMD_GPIO_PIN_CFG;
    cmd[1] = 0x00;   // GPIO0 - no change
    cmd[2] = 0x00;   // GPIO1 - no change
    cmd[3] = 0x03;   // GPIO2 - drive high
    cmd[4] = 0x02;   // GPIO3 - drive low
    cmd[5] = 0x00;   // NIRQ - default
    cmd[6] = 0x00;   // SDO - default
    cmd[7] = 0x00;   // GEN_CONFIG

    si4463_send_cmd(cmd, 8);
    return si4463_read_response(resp, 7);
}


static unsigned char si4463_gpio_cfg_tx(void) {
    unsigned char cmd[8];
    unsigned char resp[7];

    cmd[0] = SI4463_CMD_GPIO_PIN_CFG;
    cmd[1] = 0x00;   // GPIO0 - no change
    cmd[2] = 0x00;   // GPIO1 - no change
    cmd[3] = 0x02;   // GPIO2 - drive low
    cmd[4] = 0x03;   // GPIO3 - drive high
    cmd[5] = 0x00;   // NIRQ - default
    cmd[6] = 0x00;   // SDO - default
    cmd[7] = 0x00;   // GEN_CONFIG

    si4463_send_cmd(cmd, 8);
    return si4463_read_response(resp, 7);
}


/******************************************************************************
 si4463_set_mode()

 Switch between RX, TX and READY. Returns 1 on CTS, 0 on timeout.
******************************************************************************/
unsigned char si4463_set_mode(unsigned char mode) {
    __builtin_clrwdt();

    switch (mode) {
        case SI4463_MODE_RX:
            return si4463_start_rx();

        case SI4463_MODE_TX:
            return si4463_start_tx();

        case SI4463_MODE_READY:
        default:
            return si4463_change_state(SI4463_STATE_READY);
    }
}


/******************************************************************************
 si4463_get_device_state()

 Prints the current device state and channel to UART.
******************************************************************************/
static void si4463_get_device_state(void) {
    unsigned char cmd[1];
    unsigned char resp[2];
    unsigned char state;
    unsigned char channel;
    const char *state_name;

    cmd[0] = SI4463_CMD_REQUEST_DEVICE_STATE;
    si4463_send_cmd(cmd, 1);

    if (!si4463_read_response(resp, 2)) {
        si4463_debug_msg(STR_TASK_SI4463 "REQUEST_DEVICE_STATE: CTS timeout");
        return;
    }

    state   = resp[0];
    channel = resp[1];

    switch (state) {
        case SI4463_STATE_SLEEP:      state_name = "SLEEP";      break;
        case SI4463_STATE_SPI_ACTIVE: state_name = "SPI_ACTIVE"; break;
        case SI4463_STATE_READY:      state_name = "READY";      break;
        case SI4463_STATE_TX_TUNE:    state_name = "TX_TUNE";    break;
        case SI4463_STATE_RX_TUNE:    state_name = "RX_TUNE";    break;
        case SI4463_STATE_TX:         state_name = "TX";         break;
        case SI4463_STATE_RX:         state_name = "RX";         break;
        default:                      state_name = "UNKNOWN";    break;
    }

    sprintf(strTmp, STR_TASK_SI4463 "Device state: %s (0x%02X), channel: %u",
            state_name, state, (unsigned int)channel);
    si4463_debug_msg(strTmp);
}


/******************************************************************************
 si4463_get_int_status()

 GET_INT_STATUS, clearing all pending interrupts. Returns PH_PEND and
 saves MODEM_PEND in last_modem_pend. Returns 0 on CTS timeout.
******************************************************************************/
/* Bit positions per AN625 GET_INT_STATUS */
#define SI4463_PH_PEND_PACKET_RX  0x10   // bit 4 of PH_PEND
#define SI4463_PH_PEND_CRC_ERROR  0x08   // bit 3 of PH_PEND
#define SI4463_MODEM_PEND_SYNC_DETECT      0x01   // bit 0 of MODEM_PEND
#define SI4463_MODEM_PEND_PREAMBLE_DETECT  0x02   // bit 1 of MODEM_PEND

static unsigned char last_modem_pend;   // MODEM_PEND from last GET_INT_STATUS

static unsigned char si4463_get_int_status(void) {
    unsigned char cmd[4];
    unsigned char resp[8];

    cmd[0] = SI4463_CMD_GET_INT_STATUS;
    cmd[1] = 0x00;   // PH_CLR_PEND
    cmd[2] = 0x00;   // MODEM_CLR_PEND
    cmd[3] = 0x00;   // CHIP_CLR_PEND

    si4463_send_cmd(cmd, 4);

    if (!si4463_read_response(resp, 8)) {
        return 0x00;
    }

    // resp: [0]=INT_PEND [1]=INT_STATUS [2]=PH_PEND [3]=PH_STATUS
    //       [4]=MODEM_PEND [5]=MODEM_STATUS [6]=CHIP_PEND [7]=CHIP_STATUS
    last_modem_pend = resp[4];
    return resp[2];  // PH_PEND
}


/******************************************************************************
 si4463_fifo_info()

 FIFO_INFO. Non-zero reset clears the RX FIFO (TX is never reset here).
 Returns the RX FIFO byte count (0 on failure).
******************************************************************************/
static unsigned char si4463_fifo_info(unsigned char reset) {
    unsigned char cmd[2];
    unsigned char resp[2];

    cmd[0] = SI4463_CMD_FIFO_INFO;
    cmd[1] = reset ? 0x02 : 0x00;  // bit 1 = reset RX FIFO

    si4463_send_cmd(cmd, 2);

    // resp[0] = RX_FIFO_COUNT, resp[1] = TX_FIFO_SPACE
    if (!si4463_read_response(resp, 2)) {
        return 0;
    }
    return resp[0];
}


/******************************************************************************
 si4463_read_rx_fifo()

 Reads count bytes from the RX FIFO into buf. Returns count.
******************************************************************************/
static unsigned char si4463_read_rx_fifo(unsigned char *buf,
                                         unsigned char count) {
    unsigned char i;

    SI4463_CS_LOW();
    { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
    spi2_transfer(SI4463_CMD_READ_RX_FIFO);
    for (i = 0; i < count; i++) {
        buf[i] = spi2_transfer(0x00);
    }
    SI4463_CS_HIGH();

    return count;
}


/******************************************************************************
 si4463_cmd_rx()

 Bench command 's': init the radio and print received packets to UART.
 Blocks the scheduler until 'q' is received on the debug UART.
******************************************************************************/
#define RX_FIFO_BUF_SIZE  64

void si4463_cmd_rx(void) {
    unsigned char rx_buf[RX_FIFO_BUF_SIZE];
    unsigned char ph_pend;
    unsigned char n, i;
    unsigned int poll_count;
    static const char hex[] = "0123456789ABCDEF";

    si4463_debug_msg(STR_TASK_SI4463 "s: Initializing SPI2...");
    spi2_init();

    // MISO diagnostic: raw pin, PPS mapping, one SPI read
    {
        unsigned char raw_pin, rpinr_val, spi_byte;
        raw_pin = _RG7;
        rpinr_val = (unsigned char)(RPINR22 & 0x3F);
        SI4463_CS_LOW();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
        spi_byte = spi2_transfer(0x00);
        SI4463_CS_HIGH();
        sprintf(strTmp, STR_TASK_SI4463 "MISO diag: RG7=%u SDI2R=%u spi=0x%02X SPI2STAT=0x%04X",
                (unsigned int)raw_pin, (unsigned int)rpinr_val,
                (unsigned int)spi_byte, (unsigned int)SPI2STAT);
        si4463_debug_msg(strTmp);
    }

    si4463_debug_msg(STR_TASK_SI4463 "s: Sending POWER_UP...");
    if (!si4463_power_up()) {
        si4463_debug_msg(STR_TASK_SI4463 "s: POWER_UP CTS timeout!");
        return;
    }

    si4463_part_info();

    si4463_debug_msg(STR_TASK_SI4463 "s: Configuring RF (437.08 MHz, GFSK TX1200/RX9600)...");
    si4463_configure_rf();

    si4463_gpio_cfg_rx();
    si4463_debug_msg(STR_TASK_SI4463 "s: GPIO cfg0 (G2=H G3=L), entering RX...");

    if (0) { /* GPIO sweep, disabled */
        unsigned char cmd_ms[2];
        unsigned char ms_resp[8];
        unsigned char best_rssi, rssi_val, overall_best, best_cfg;
        unsigned int j;
        unsigned char cfg;

        overall_best = 0;
        best_cfg = 0;

        // cfg0: G2=H G3=L, cfg1: G2=L G3=H, cfg2: both low
        for (cfg = 0; cfg < 3; cfg++) {
            unsigned char cmd_gpio[8];
            cmd_gpio[0] = SI4463_CMD_GPIO_PIN_CFG;
            cmd_gpio[1] = 0x00;
            cmd_gpio[2] = 0x00;
            cmd_gpio[5] = 0x00;
            cmd_gpio[6] = 0x00;
            cmd_gpio[7] = 0x00;

            if (cfg == 0) {
                cmd_gpio[3] = 0x03; cmd_gpio[4] = 0x02;  // G2=H, G3=L
            } else if (cfg == 1) {
                cmd_gpio[3] = 0x02; cmd_gpio[4] = 0x03;  // G2=L, G3=H
            } else {
                cmd_gpio[3] = 0x02; cmd_gpio[4] = 0x02;  // G2=L, G3=L
            }
            si4463_send_cmd(cmd_gpio, 8);
            {   unsigned char gpio_resp[7];
                si4463_read_response(gpio_resp, 7);
            }

            si4463_set_mode(SI4463_MODE_RX);

            // Sample RSSI 5 times over ~2 seconds, keep peak
            best_rssi = 0;
            for (j = 0; j < 5; j++) {
                unsigned int d;
                __builtin_clrwdt();
                for (d = 0; d < 50000u; d++) { __builtin_nop(); }
                for (d = 0; d < 50000u; d++) { __builtin_nop(); }
                for (d = 0; d < 50000u; d++) { __builtin_nop(); }
                __builtin_clrwdt();
                for (d = 0; d < 50000u; d++) { __builtin_nop(); }
                for (d = 0; d < 50000u; d++) { __builtin_nop(); }
                for (d = 0; d < 50000u; d++) { __builtin_nop(); }
                __builtin_clrwdt();

                cmd_ms[0] = SI4463_CMD_GET_MODEM_STATUS;
                cmd_ms[1] = 0x00;
                si4463_send_cmd(cmd_ms, 2);
                si4463_read_response(ms_resp, 8);
                rssi_val = ms_resp[2];
                if (rssi_val > best_rssi) best_rssi = rssi_val;
            }

            sprintf(strTmp, STR_TASK_SI4463 "  cfg%u RSSI=%u (~%d dBm)",
                    (unsigned int)cfg, (unsigned int)best_rssi,
                    (int)best_rssi / 2 - 134);
            si4463_debug_msg(strTmp);

            if (best_rssi > overall_best) {
                overall_best = best_rssi;
                best_cfg = cfg;
            }

            si4463_set_mode(SI4463_MODE_READY);
        }

        // Apply the best GPIO config
        {
            unsigned char cmd_gpio[8];
            cmd_gpio[0] = SI4463_CMD_GPIO_PIN_CFG;
            cmd_gpio[1] = 0x00;
            cmd_gpio[2] = 0x00;
            cmd_gpio[5] = 0x00;
            cmd_gpio[6] = 0x00;
            cmd_gpio[7] = 0x00;
            if (best_cfg == 0) {
                cmd_gpio[3] = 0x03; cmd_gpio[4] = 0x02;
            } else if (best_cfg == 1) {
                cmd_gpio[3] = 0x02; cmd_gpio[4] = 0x03;
            } else {
                cmd_gpio[3] = 0x02; cmd_gpio[4] = 0x02;
            }
            si4463_send_cmd(cmd_gpio, 8);
            {   unsigned char gpio_resp[7];
                si4463_read_response(gpio_resp, 7);
            }
        }

        sprintf(strTmp, STR_TASK_SI4463 "s: Using cfg%u (best RSSI=%u), entering RX...",
                (unsigned int)best_cfg, (unsigned int)overall_best);
        si4463_debug_msg(strTmp);
    } /* end if(0) GPIO sweep */

    if (!si4463_set_mode(SI4463_MODE_RX)) {
        si4463_debug_msg(STR_TASK_SI4463 "s: START_RX CTS timeout!");
        return;
    }

    si4463_get_device_state();

    // Drain stale UART1 RX characters
    while (csk_uart1_getchar()) { }

    si4463_debug_msg(STR_TASK_SI4463 "s: Polling nIRQ... ('q' to exit)");

    poll_count = 0;
    while (1) {
        __builtin_clrwdt();

        if (csk_uart1_getchar() == 'q') {
            si4463_set_mode(SI4463_MODE_READY);
            si4463_debug_msg(STR_TASK_SI4463 "s: Exited RX mode.");
            return;
        }

        if (SI4463_NIRQ_READ() == 0) {

            // Quick check of PH_PEND via FRR
            {
                unsigned char frr_ph, frr_mdm;
                si4463_read_frr(&frr_ph, &frr_mdm);

                if (!(frr_ph & (SI4463_PH_PEND_PACKET_RX | SI4463_PH_PEND_CRC_ERROR))) {
                    // Modem-only event; chip stays in RX
                    goto skip_nirq;
                }
            }

            ph_pend = si4463_get_int_status();

            if (ph_pend & SI4463_PH_PEND_CRC_ERROR) {
                n = si4463_fifo_info(0);
                if (n > 0 && n <= RX_FIFO_BUF_SIZE)
                    si4463_read_rx_fifo(rx_buf, n);
                si4463_fifo_info(0x03);
                si4463_set_mode(SI4463_MODE_RX);
                // Print after re-arming RX
                sprintf(strTmp, STR_TASK_SI4463 "CRC_ERR %u", (unsigned int)n);
                si4463_debug_msg(strTmp);
            }

            if (ph_pend & SI4463_PH_PEND_PACKET_RX) {
                // Re-arm RX before printing
                n = si4463_fifo_info(0);
                if (n > RX_FIFO_BUF_SIZE) n = RX_FIFO_BUF_SIZE;
                if (n > 0) {
                    si4463_read_rx_fifo(rx_buf, n);
                }
                si4463_set_mode(SI4463_MODE_RX);

                // "RX: <payload as ASCII>"
                if (n > 1) {
                    unsigned char payload_len = rx_buf[0];
                    unsigned char j;
                    char *p = strTmp;
                    unsigned char end;

                    if (payload_len > n - 1) payload_len = n - 1;

                    *p++ = 'R'; *p++ = 'X'; *p++ = ':'; *p++ = ' ';
                    end = payload_len;
                    if (end > 30) end = 30;
                    for (j = 0; j < end; j++) {
                        unsigned char c = rx_buf[1 + j];
                        *p++ = (c >= 0x20 && c <= 0x7E) ? c : '.';
                    }
                    *p = '\0';
                    si4463_debug_msg(strTmp);
                }
            }
            skip_nirq: ;
        }

        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }

        // Heartbeat
        poll_count++;
        if (poll_count >= 100000u) {
            si4463_debug_msg(STR_TASK_SI4463 "RX: listening...");
            poll_count = 0;
        }
    }
}


/******************************************************************************
 si4463_cmd_tx()

 Bench command 'x': init the radio and transmit numbered test packets
 until 'q'. Blocks the scheduler.
******************************************************************************/
void si4463_cmd_tx(void) {
    unsigned int pkt_num;
    unsigned char tx_buf[16];
    unsigned char payload_len;
    unsigned char i;

    si4463_debug_msg(STR_TASK_SI4463 "x: Initializing SPI2...");
    spi2_init();

    si4463_debug_msg(STR_TASK_SI4463 "x: Resetting Si4463...");
    si4463_reset();

    si4463_debug_msg(STR_TASK_SI4463 "x: Sending POWER_UP...");
    if (!si4463_power_up()) {
        si4463_debug_msg(STR_TASK_SI4463 "x: POWER_UP CTS timeout!");
        return;
    }

    si4463_part_info();

    si4463_debug_msg(STR_TASK_SI4463 "x: Configuring RF (437.08 MHz, GFSK TX1200/RX9600)...");
    si4463_configure_rf();

    si4463_set_property(0x22, 0x01, 0x7F);  // PA_PWR_LVL max

    si4463_debug_msg(STR_TASK_SI4463 "x: Setting RF switch to TX...");
    si4463_gpio_cfg_tx();

    si4463_debug_msg(STR_TASK_SI4463 "x: Transmitting packets ('q' to stop)...");

    for (pkt_num = 0; ; pkt_num++) {
        __builtin_clrwdt();

        if (csk_uart1_getchar() == 'q') {
            si4463_debug_msg(STR_TASK_SI4463 "x: Stopped by user.");
            break;
        }

        // Reset TX FIFO
        {
            unsigned char cmd[2];
            cmd[0] = SI4463_CMD_FIFO_INFO;
            cmd[1] = 0x01;
            si4463_send_cmd(cmd, 2);
            si4463_wait_cts();
        }

        // Payload "KO6IKZ NNNN"
        payload_len = 11;

        // TX sends exactly FIELD_2_LENGTH bytes (variable length is RX-only),
        // so set it to the payload length or the FIFO underflows.
        si4463_set_property(0x12, 0x11, 0x00);          // FIELD_2_LENGTH_12_8
        si4463_set_property(0x12, 0x12, payload_len);   // FIELD_2_LENGTH_7_0

        tx_buf[0] = 'K'; tx_buf[1] = 'O'; tx_buf[2] = '6';
        tx_buf[3] = 'I'; tx_buf[4] = 'K'; tx_buf[5] = 'Z';
        tx_buf[6] = ' ';
        tx_buf[7] = '0' + ((pkt_num / 1000) % 10);
        tx_buf[8] = '0' + ((pkt_num / 100) % 10);
        tx_buf[9] = '0' + ((pkt_num / 10) % 10);
        tx_buf[10] = '0' + (pkt_num % 10);

        // TX FIFO: [length] [payload]
        SI4463_CS_LOW();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
        spi2_transfer(SI4463_CMD_WRITE_TX_FIFO);
        spi2_transfer(payload_len);
        for (i = 0; i < payload_len; i++) {
            spi2_transfer(tx_buf[i]);
        }
        SI4463_CS_HIGH();

        if (!si4463_start_tx()) {
            si4463_debug_msg(STR_TASK_SI4463 "x: START_TX CTS timeout!");
            break;
        }

        sprintf(strTmp, STR_TASK_SI4463 "TX [%u]: KO6IKZ %04u (%u bytes)",
                pkt_num, pkt_num, (unsigned int)payload_len);
        si4463_debug_msg(strTmp);

        // Wait for TX: ~640 bits = 533 ms at 1200 bps. 50000 NOPs measure
        // ~102 ms, so this is ~2.45 s.
        {
            unsigned int d;
            unsigned char w;
            for (w = 0; w < 24; w++) {
                for (d = 0; d < 50000u; d++) { __builtin_nop(); }
                __builtin_clrwdt();
            }
        }
    }

    si4463_set_mode(SI4463_MODE_READY);
    si4463_debug_msg(STR_TASK_SI4463 "x: TX done.");
}


/******************************************************************************
 si4463_start_cw()

 Init the chip and start an unmodulated carrier at 437.08 MHz, leaving it
 on (chip parked in TX). Used by si4463_cmd_cw() and task_beacon.
 Returns 1 on success, 0 if an init step times out.
******************************************************************************/
static unsigned char si4463_get_property(unsigned char group,
                                         unsigned char start,
                                         unsigned char count,
                                         unsigned char *resp);

unsigned char si4463_start_cw(void) {
    unsigned char cmd[7];

    si4463_debug_msg(STR_TASK_SI4463 "cw: Initializing SPI2...");
    spi2_init();

    si4463_debug_msg(STR_TASK_SI4463 "cw: Resetting Si4463...");
    si4463_reset();

    si4463_debug_msg(STR_TASK_SI4463 "cw: Sending POWER_UP...");
    if (!si4463_power_up()) {
        si4463_debug_msg(STR_TASK_SI4463 "cw: POWER_UP CTS timeout!");
        return 0;
    }

    si4463_part_info();

    si4463_debug_msg(STR_TASK_SI4463 "cw: Configuring RF (437.08 MHz)...");
    si4463_configure_rf();

    /* MODEM_MOD_TYPE = CW */
    si4463_set_property(0x20, 0x00, 0x00);

    /* PA max */
    si4463_set_property(0x22, 0x01, 0x7F);

    si4463_debug_msg(STR_TASK_SI4463 "cw: Setting RF switch to TX...");
    si4463_gpio_cfg_tx();

    /* START_TX with NEXT_STATE = NOCHANGE so the carrier stays on until
       CHANGE_STATE. */
    cmd[0] = SI4463_CMD_START_TX;
    cmd[1] = 0x00;
    cmd[2] = 0x00;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    si4463_send_cmd(cmd, 7);
    if (!si4463_wait_cts()) {
        si4463_debug_msg(STR_TASK_SI4463 "cw: START_TX CTS timeout!");
        return 0;
    }

    /* Confirm the chip is in TX (0x07) and MOD_TYPE is CW (0x00) */
    {
        unsigned char dcmd[1];
        unsigned char dresp[2];
        unsigned char mod[1];

        dcmd[0] = SI4463_CMD_REQUEST_DEVICE_STATE;
        si4463_send_cmd(dcmd, 1);
        if (si4463_read_response(dresp, 2)) {
            sprintf(strTmp, STR_TASK_SI4463 "cw: device state=0x%02X chan=%u "
                    "(0x07=TX expected)",
                    dresp[0], (unsigned int)dresp[1]);
            si4463_debug_msg(strTmp);
        }
        if (si4463_get_property(0x20, 0x00, 1, mod)) {
            sprintf(strTmp, STR_TASK_SI4463 "cw: MOD_TYPE=0x%02X (0x00=CW)",
                    mod[0]);
            si4463_debug_msg(strTmp);
        }
    }

    si4463_debug_msg(STR_TASK_SI4463 "cw: CW carrier ON at 437.08 MHz.");
    return 1;
}


/******************************************************************************
 si4463_cw_keepalive()

 Re-keys the CW carrier if the chip has dropped out of TX. Returns the
 state read before any re-key (0x07 = TX, 0xFF = read failed).
******************************************************************************/
unsigned char si4463_cw_keepalive(void) {
    unsigned char dcmd[1];
    unsigned char dresp[2];
    unsigned char cmd[7];

    dcmd[0] = SI4463_CMD_REQUEST_DEVICE_STATE;
    si4463_send_cmd(dcmd, 1);
    if (!si4463_read_response(dresp, 2)) {
        return 0xFF;
    }

    if (dresp[0] != SI4463_STATE_TX) {
        cmd[0] = SI4463_CMD_START_TX;
        cmd[1] = 0x00;
        cmd[2] = 0x00;
        cmd[3] = 0x00;
        cmd[4] = 0x00;
        cmd[5] = 0x00;
        cmd[6] = 0x00;
        si4463_send_cmd(cmd, 7);
        si4463_wait_cts();
    }

    return dresp[0];
}


/******************************************************************************
 si4463_cmd_cw()

 Bench command: CW carrier at 437.08 MHz until 'q' on the debug UART.
******************************************************************************/
void si4463_cmd_cw(void) {
    unsigned int hb;

    if (!si4463_start_cw()) {
        return;
    }

    si4463_debug_msg(STR_TASK_SI4463 "c: CW carrier ON at 437.08 MHz. 'q' to stop.");

    hb = 0;
    for (;;) {
        __builtin_clrwdt();

        if (csk_uart1_getchar() == 'q') {
            si4463_debug_msg(STR_TASK_SI4463 "c: Stopped by user.");
            break;
        }

        /* Heartbeat */
        { unsigned int d; for (d = 0; d < 50000u; d++) { __builtin_nop(); } }
        hb++;
        if (hb >= 300u) {
            si4463_debug_msg(STR_TASK_SI4463 "c: CW on...");
            hb = 0;
        }
    }

    si4463_change_state(SI4463_STATE_READY);
    si4463_debug_msg(STR_TASK_SI4463 "c: CW off.");
}


static unsigned char si4463_get_property(unsigned char group,
                                         unsigned char start,
                                         unsigned char count,
                                         unsigned char *resp) {
    unsigned char cmd[4];
    cmd[0] = SI4463_CMD_GET_PROPERTY;
    cmd[1] = group;
    cmd[2] = count;
    cmd[3] = start;
    si4463_send_cmd(cmd, 4);
    return si4463_read_response(resp, count);
}

/* Writes straight to UART1, bypassing csk_uart1_msg_ts(), which misbehaves
   when called back-to-back (see "Known issues" in firmware/obc/README.md). */
static void dump_props_line(const char *s) {
    csk_uart1_puts(s);
    csk_uart1_puts(STR_CRLF);
}


/******************************************************************************
 si4463_cmd_get_props()

 Diagnostic 'g' command: dumps PIC24 clock registers and reads back key
 Si4463 properties. Assumes the chip is already initialised.
******************************************************************************/

void si4463_cmd_get_props(void) {
    unsigned char cmd[1];
    unsigned char resp[16];
    unsigned long fcy_est;

    /* PIC24 clock + timer state */
    sprintf(strTmp, STR_TASK_SI4463 "g: OSCCON=0x%04X CLKDIV=0x%04X OSCTUN=0x%04X",
            (unsigned int)OSCCON, (unsigned int)CLKDIV, (unsigned int)OSCTUN);
    dump_props_line(strTmp);
    sprintf(strTmp, STR_TASK_SI4463 "g: T2CON=0x%04X PR2=%u U2BRG=%u",
            (unsigned int)T2CON, (unsigned int)PR2, (unsigned int)U2BRG);
    dump_props_line(strTmp);

    /* COSC = OSCCON[14:12], NOSC = OSCCON[10:8] */
    {
        unsigned char cosc = (OSCCON >> 12) & 0x07;
        unsigned char nosc = (OSCCON >> 8) & 0x07;
        sprintf(strTmp, STR_TASK_SI4463 "g: COSC=%u NOSC=%u (0=FRC 1=FRCPLL 2=PRI 3=PRIPLL 4=SOSC 5=LPRC 6=- 7=FRCDIV)",
                (unsigned int)cosc, (unsigned int)nosc);
        dump_props_line(strTmp);
    }

    /* Fcy estimate assuming a 100 Hz tick */
    fcy_est = (unsigned long)PR2 * 100UL;
    sprintf(strTmp, STR_TASK_SI4463 "g: Fcy estimate (PR2*100Hz)=%lu Hz", fcy_est);
    dump_props_line(strTmp);

    /* PART_INFO */
    cmd[0] = SI4463_CMD_PART_INFO;
    si4463_send_cmd(cmd, 1);
    if (si4463_read_response(resp, 8)) {
        sprintf(strTmp, STR_TASK_SI4463 "g: PART_INFO chiprev=%02X part=%02X%02X pbuild=%02X id=%02X%02X cust=%02X romid=%02X",
                resp[0], resp[1], resp[2], resp[3], resp[4], resp[5], resp[6], resp[7]);
        dump_props_line(strTmp);
    } else {
        dump_props_line(STR_TASK_SI4463 "g: PART_INFO CTS timeout");
    }

    /* FUNC_INFO */
    cmd[0] = SI4463_CMD_FUNC_INFO;
    si4463_send_cmd(cmd, 1);
    if (si4463_read_response(resp, 6)) {
        sprintf(strTmp, STR_TASK_SI4463 "g: FUNC_INFO revext=%02X revbranch=%02X revint=%02X patch=%02X%02X func=%02X",
                resp[0], resp[1], resp[2], resp[3], resp[4], resp[5]);
        dump_props_line(strTmp);
    }

    /* GLOBAL XO_TUNE 0x00/0x00 */
    if (si4463_get_property(0x00, 0x00, 1, resp)) {
        sprintf(strTmp, STR_TASK_SI4463 "g: XO_TUNE=%02X", resp[0]);
        dump_props_line(strTmp);
    }

    /* MODEM_MOD_TYPE 0x20/0x00 */
    if (si4463_get_property(0x20, 0x00, 1, resp)) {
        sprintf(strTmp, STR_TASK_SI4463 "g: MOD_TYPE=%02X", resp[0]);
        dump_props_line(strTmp);
    }

    /* MODEM_DATA_RATE 0x20/0x03..0x05 (3 bytes, MSB first) */
    if (si4463_get_property(0x20, 0x03, 3, resp)) {
        unsigned long dr = ((unsigned long)resp[0] << 16) |
                           ((unsigned long)resp[1] << 8)  |
                            (unsigned long)resp[2];
        sprintf(strTmp, STR_TASK_SI4463 "g: DATA_RATE=%02X%02X%02X (%lu)",
                resp[0], resp[1], resp[2], dr);
        dump_props_line(strTmp);
    }

    /* MODEM_TX_NCO_MODE 0x20/0x06..0x09 (4 bytes) */
    if (si4463_get_property(0x20, 0x06, 4, resp)) {
        unsigned long ncomod = (((unsigned long)(resp[0] & 0x03)) << 24) |
                               ((unsigned long)resp[1] << 16) |
                               ((unsigned long)resp[2] << 8)  |
                                (unsigned long)resp[3];
        unsigned char txosr = (resp[0] >> 6) & 0x03;
        sprintf(strTmp, STR_TASK_SI4463 "g: TX_NCO_MODE=%02X%02X%02X%02X txosr=%u ncomod=%lu",
                resp[0], resp[1], resp[2], resp[3], (unsigned int)txosr, ncomod);
        dump_props_line(strTmp);
    }

    /* MODEM_FREQ_DEV 0x20/0x0A..0x0C (3 bytes) */
    if (si4463_get_property(0x20, 0x0A, 3, resp)) {
        sprintf(strTmp, STR_TASK_SI4463 "g: FREQ_DEV=%02X%02X%02X",
                resp[0], resp[1], resp[2]);
        dump_props_line(strTmp);
    }

    /* MODEM_CLKGEN_BAND 0x20/0x51 */
    if (si4463_get_property(0x20, 0x51, 1, resp)) {
        sprintf(strTmp, STR_TASK_SI4463 "g: CLKGEN_BAND=%02X (band=%u sy_sel=%u)",
                resp[0], resp[0] & 0x07, (resp[0] >> 3) & 0x01);
        dump_props_line(strTmp);
    }

    /* FREQ_CONTROL 0x40/0x00..0x07 (8 bytes) */
    if (si4463_get_property(0x40, 0x00, 8, resp)) {
        unsigned long inte = resp[0];
        unsigned long frac = ((unsigned long)resp[1] << 16) |
                             ((unsigned long)resp[2] << 8)  |
                              (unsigned long)resp[3];
        sprintf(strTmp, STR_TASK_SI4463 "g: FREQ_CTL inte=%lu frac=%lu (raw %02X %02X%02X%02X)",
                inte, frac, resp[0], resp[1], resp[2], resp[3]);
        dump_props_line(strTmp);
    }

    /* PA group 0x22/0x00..0x03. Expected after si4463_init_beacon():
       PA_MODE=08 PA_PWR_LVL=7F PA_BIAS_CLKDUTY=00 PA_TC=1D. */
    if (si4463_get_property(0x22, 0x00, 4, resp)) {
        unsigned char pa_mode_f = resp[0] & 0x03;
        unsigned char pa_sel_f  = (resp[0] >> 2) & 0x0F;
        sprintf(strTmp, STR_TASK_SI4463 "g: PA_MODE=%02X PA_PWR_LVL=%02X PA_BIAS_CLKDUTY=%02X PA_TC=%02X",
                resp[0], resp[1], resp[2], resp[3]);
        dump_props_line(strTmp);
        sprintf(strTmp, STR_TASK_SI4463 "g:   pa_mode=%u (0=CLASS_E 1=SWC 2=SWC_HQ) pa_sel=%u (1=HP_COARSE 2=HP_FINE 4=LP)",
                (unsigned int)pa_mode_f, (unsigned int)pa_sel_f);
        dump_props_line(strTmp);
    } else {
        dump_props_line(STR_TASK_SI4463 "g: PA group readback CTS timeout");
    }

    /* MOD_TYPE decoded */
    if (si4463_get_property(0x20, 0x00, 1, resp)) {
        unsigned char mt = resp[0] & 0x07;
        unsigned char ms = (resp[0] >> 5) & 0x03;
        sprintf(strTmp, STR_TASK_SI4463 "g: MOD_TYPE bits: type=%u (0=CW 3=2GFSK 5=4GFSK) src=%u (0=FIFO 1=DIRECT 2=PRBS)",
                (unsigned int)mt, (unsigned int)ms);
        dump_props_line(strTmp);
    }
}


/******************************************************************************
****                                                                       ****
**                                                                           **
task_si4463_rx()

Disabled; the radio is owned by task_beacon. Bench RX/TX via the 's'/'x'
commands in task_cmd.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_si4463_rx(void) {

#if 0
    spi2_init();
    si4463_reset();
    si4463_power_up();

    while (1) {

        si4463_set_mode(SI4463_MODE_RX);

        // ~10 s. Salvo delays are 8-bit (OSBYTES_OF_DELAYS=1), so split
        // into <=255-tick chunks.
        OS_Delay(250);
        OS_Delay(250);
        OS_Delay(250);
        OS_Delay(250);

        si4463_set_mode(SI4463_MODE_READY);

        OS_Delay(2);

    } /* while */
#endif

} /* task_si4463_rx() */


/******************************************************************************
****                                                                       ****
**                                                                           **
Half-duplex uplink support

task_beacon owns the radio and listens between beacons. Switching direction
changes:
  - the AS179 RF switch (si4463_gpio_cfg_rx/tx)
  - the packet-handler FIELD config (0x12/0x0C..0x17), PKT_LEN and
    INT_CTL_MODEM_ENABLE (si4463_pkt_cfg_rx/tx)

Downlink is AX.25/G3RUH with the packet handler bypassed (raw FIFO bytes).
Uplink uses the chip's own framing, as sent by ground-station/pluto_tx.py:

  preamble | sync 0x2DD4 | length (2 B, big-endian) | payload | CRC-16 CCITT

with the CRC covering length + payload. The 2-byte length is needed for
the ~280 B PROMPT frame.

**                                                                           **
****                                                                       ****
******************************************************************************/

/* Latched RSSI of the last received packet, raw (dBm ~= raw/2 - 134) */
static unsigned char si4463_rx_rssi = 0;

/* TX values match si4463_configure_rf() */
#define SI4463_PKT_LEN_TX          0x2A  /* MSB-first, 1-byte len in FIFO, field 2 variable */
#define SI4463_PKT_LEN_RX          0x3A  /* same, but SIZE=1: 2-byte length (AN625 3.4.40) */
#define SI4463_INT_MODEM_TX        0x30  /* INVALID_SYNC + RSSI_JUMP */
#define SI4463_INT_MODEM_RX        (SI4463_MODEM_PEND_PREAMBLE_DETECT | SI4463_MODEM_PEND_SYNC_DETECT)


/* Downlink field config: raw FIFO bytes, no chip CRC */
static void si4463_pkt_cfg_tx(void) {
    static const unsigned char d[] =
        { 0x30, 0x00, 0x00, 0x04, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
    si4463_set_properties(0x12, 0x0C, d, 12);
    si4463_set_property(0x12, 0x08, SI4463_PKT_LEN_TX);
    si4463_set_property(0x01, 0x02, SI4463_INT_MODEM_TX);
}


/* Uplink field config:
     field 1 = 2-byte length, CRC_START + CRC_ENABLE (0x82)
     field 2 = payload, variable, CRC_ENABLE + CHECK_CRC + SEND_CRC (0x2A).
               No CRC_START, so the CRC covers length + payload.
   PH_FIELD_SPLIT = 0, so RX uses the TX field properties. */
static void si4463_pkt_cfg_rx(void) {
    unsigned char d[12];
    d[0]  = 0x30;                                        /* PKT_RX_THRESHOLD */
    d[1]  = 0x00;  d[2] = 0x02;                          /* FIELD_1_LENGTH = 2 */
    d[3]  = 0x04;                                        /* FIELD_1_CONFIG */
    d[4]  = 0x82;                                        /* FIELD_1_CRC_CONFIG */
    d[5]  = (unsigned char)((SI4463_RX_MAX_PAYLOAD >> 8) & 0x1F);  /* FIELD_2 max */
    d[6]  = (unsigned char)(SI4463_RX_MAX_PAYLOAD & 0xFF);
    d[7]  = 0x00;                                        /* FIELD_2_CONFIG */
    d[8]  = 0x2A;                                        /* FIELD_2_CRC_CONFIG */
    d[9]  = 0x00;  d[10] = 0x00;  d[11] = 0x00;          /* FIELD_3 unused */
    si4463_set_properties(0x12, 0x0C, d, 12);
    si4463_set_property(0x12, 0x08, SI4463_PKT_LEN_RX);
    si4463_set_property(0x01, 0x02, SI4463_INT_MODEM_RX);
}


/******************************************************************************
 si4463_rx_enable()

 Switch to RX, install the uplink config, flush, clear interrupts and arm
 RX. Called at the start of every listening window.
 Returns 1 if START_RX was accepted.
******************************************************************************/
unsigned char si4463_rx_enable(void) {
    si4463_change_state(SI4463_STATE_READY);
    si4463_pkt_cfg_rx();
    si4463_gpio_cfg_rx();
    si4463_fifo_info(0x03);          /* flush RX FIFO */
    si4463_get_int_status();         /* clear pending interrupts */
    return si4463_start_rx();
}


/******************************************************************************
 si4463_rx_disable()

 Leave RX, restore the downlink config and switch to the PA.
******************************************************************************/
void si4463_rx_disable(void) {
    si4463_change_state(SI4463_STATE_READY);
    si4463_pkt_cfg_tx();
    si4463_gpio_cfg_tx();
}


/******************************************************************************
 si4463_rx_take()

 Drain the RX FIFO into the frame being assembled. The first two bytes
 (big-endian length) go to rx_len_hdr[], the payload to buf[] up to maxlen.
 *have counts every byte read, including any dropped past maxlen.
******************************************************************************/
static unsigned char rx_len_hdr[2];

static void si4463_rx_take(unsigned char *buf, unsigned int maxlen,
                           unsigned int *have) {
    unsigned char n, i, b;

    n = si4463_fifo_info(0);
    if (n == 0) {
        return;
    }
    if (n > RX_FIFO_BUF_SIZE) n = RX_FIFO_BUF_SIZE;

    SI4463_CS_LOW();
    { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
    spi2_transfer(SI4463_CMD_READ_RX_FIFO);
    for (i = 0; i < n; i++) {
        b = spi2_transfer(0x00);
        if (*have < 2) {
            rx_len_hdr[*have] = b;
        } else if (*have - 2u < maxlen) {
            buf[*have - 2u] = b;
        }
        (*have)++;
    }
    SI4463_CS_HIGH();
}


/******************************************************************************
 si4463_rx_poll()

 Check for a received uplink frame. Returns the payload length copied into
 buf (0 if no complete, CRC-valid frame) and re-arms RX. A quiet poll is
 just a GPIO read of nIRQ.

 Called every 400 ms, but the 64 B RX FIFO fills 53 ms after sync at
 9600 bps, so a long frame (PROMPT, ~280 B) must be drained as it arrives.
 nIRQ is also raised on PREAMBLE/SYNC_DETECT; when seen, this busy-waits
 draining the FIFO until PACKET_RX or CRC_ERROR, capped at 0.8 s for data
 and 0.5 s for the frame (Salvo ticks keep counting from the Timer2 ISR).
 pluto_tx.py sends a 640 ms preamble so it outlasts one poll period.

 A frame is accepted only if exactly 2 + length bytes were read, so a FIFO
 overflow is rejected rather than returned truncated.
******************************************************************************/
#define RX_WAIT_DATA_TICKS   80UL   /* preamble -> first FIFO byte, 0.8 s */
#define RX_WAIT_FRAME_TICKS  50UL   /* first byte -> PACKET_RX, 0.5 s */

unsigned int si4463_rx_poll(unsigned char *buf, unsigned int maxlen) {
    unsigned char ph_pend;
    unsigned int  have = 0;
    unsigned int  plen;

    if (SI4463_NIRQ_READ() != 0) {
        return 0;
    }

    ph_pend = si4463_get_int_status();

    if (!(ph_pend & (SI4463_PH_PEND_PACKET_RX | SI4463_PH_PEND_CRC_ERROR))) {
        unsigned long t0    = OSGetTicks();
        unsigned long limit = RX_WAIT_DATA_TICKS;

        if (!(last_modem_pend & (SI4463_MODEM_PEND_PREAMBLE_DETECT
                                 | SI4463_MODEM_PEND_SYNC_DETECT))) {
            return 0;                /* still in RX */
        }

        /* Frame incoming: drain until done */
        while ((OSGetTicks() - t0) < limit) {
            __builtin_clrwdt();

            if (have == 0) {
                si4463_rx_take(buf, maxlen, &have);
                if (have > 0) {      /* data started: time the frame */
                    t0    = OSGetTicks();
                    limit = RX_WAIT_FRAME_TICKS;
                }
            } else {
                si4463_rx_take(buf, maxlen, &have);
            }

            if (SI4463_NIRQ_READ() == 0) {
                ph_pend = si4463_get_int_status();
                if (ph_pend & (SI4463_PH_PEND_PACKET_RX
                               | SI4463_PH_PEND_CRC_ERROR)) {
                    break;
                }
            }
        }

        if (!(ph_pend & (SI4463_PH_PEND_PACKET_RX | SI4463_PH_PEND_CRC_ERROR))) {
            /* False preamble or incomplete frame. The chip may be mid-packet,
               so restart from READY. */
            si4463_change_state(SI4463_STATE_READY);
            si4463_fifo_info(0x02);
            si4463_get_int_status();
            si4463_start_rx();
            return 0;
        }
    }

    if (ph_pend & SI4463_PH_PEND_CRC_ERROR) {
        si4463_fifo_info(0x02);
        si4463_start_rx();
        return 0;
    }

    /* PACKET_RX: collect what is left in the FIFO */
    si4463_rx_take(buf, maxlen, &have);

    /* Latched RSSI (GET_MODEM_STATUS byte 3); read before re-arming RX */
    {
        unsigned char cmd[2];
        unsigned char resp[8];
        cmd[0] = SI4463_CMD_GET_MODEM_STATUS;
        cmd[1] = 0x00;
        si4463_send_cmd(cmd, 2);
        if (si4463_read_response(resp, 8)) {
            si4463_rx_rssi = resp[3];
        }
    }

    si4463_fifo_info(0x02);
    si4463_start_rx();

    /* Frame = [len_hi][len_lo][payload...]; CRC is checked by the chip */
    if (have < 2) {
        return 0;
    }
    plen = ((unsigned int)rx_len_hdr[0] << 8) | rx_len_hdr[1];
    if (plen == 0 || plen > maxlen || have != plen + 2u) {
        return 0;                    /* overflow or oversize */
    }
    return plen;
}

/******************************************************************************
 si4463_last_rssi()

 Raw RSSI of the last received uplink frame (dBm ~= raw/2 - 134).
******************************************************************************/
unsigned char si4463_last_rssi(void) {
    return si4463_rx_rssi;
}


/******************************************************************************
 si4463_init_beacon()

 Radio init, called once at task_beacon startup. Leaves the chip in READY
 with the RF switch on the TX port.
******************************************************************************/
void si4463_init_beacon(void) {
    si4463_debug_msg(STR_TASK_SI4463 "beacon: init SPI2...");
    spi2_init();

    si4463_debug_msg(STR_TASK_SI4463 "beacon: reset...");
    si4463_reset();

    si4463_debug_msg(STR_TASK_SI4463 "beacon: POWER_UP...");
    if (!si4463_power_up()) {
        si4463_debug_msg(STR_TASK_SI4463 "beacon: POWER_UP CTS timeout!");
        return;
    }

    si4463_part_info();

    si4463_debug_msg(STR_TASK_SI4463 "beacon: configure RF...");
    si4463_configure_rf();

    /* PA max */
    si4463_set_property(0x22, 0x01, 0x7F);

    si4463_debug_msg(STR_TASK_SI4463 "beacon: RF switch -> TX path.");
    si4463_gpio_cfg_tx();

    si4463_change_state(SI4463_STATE_READY);

    si4463_debug_msg(STR_TASK_SI4463 "beacon: init done.");
}


/******************************************************************************
 si4463_tx_ax25()

 Transmit one AX.25 UI frame, 1200 bps G3RUH GFSK. ax25_build_ui() produces
 the complete encoded bit stream; the chip sends the FIFO bytes as-is.
 Expects si4463_init_beacon() to have run and the RF switch on TX. Leaves
 the chip in READY. Blocks for the frame's airtime.
******************************************************************************/
void si4463_tx_ax25(const unsigned char *info, unsigned int info_len) {
    static unsigned char ax25_buf[512];
    unsigned int  enc_len;
    unsigned int  i;
    unsigned int  row;

    __builtin_clrwdt();

    /* Flags + frame, stuffed, NRZI, scrambled, packed MSB-first */
    enc_len = ax25_build_ui(AX25_DST_CALL, AX25_DST_SSID,
                            AX25_SRC_CALL, AX25_SRC_SSID,
                            info, info_len,
                            ax25_buf, sizeof(ax25_buf));
    if (enc_len == 0) {
        si4463_debug_msg(STR_TASK_SI4463 "ax25: build failed (info too large?)");
        return;
    }

    /* Make sure we are out of RX and the downlink field config is in */
    si4463_change_state(SI4463_STATE_READY);
    si4463_pkt_cfg_tx();

    /* Hex dump, verbose console only (~0.5 s of UART time per frame) */
    if (g_console_verbose) {
    for (row = 0; row < enc_len; row += 16) {
        char *p = strTmp;
        unsigned int end = row + 16;
        if (end > enc_len) end = enc_len;

        *p++ = 'a'; *p++ = 'x'; *p++ = '2'; *p++ = '5';
        *p++ = ' '; *p++ = 'h'; *p++ = 'e'; *p++ = 'x'; *p++ = ' ';
        {
            unsigned char h2 = (row >> 8) & 0x0F;
            unsigned char h1 = (row >> 4) & 0x0F;
            unsigned char h0 = row & 0x0F;
            *p++ = h2 < 10 ? '0' + h2 : 'A' + h2 - 10;
            *p++ = h1 < 10 ? '0' + h1 : 'A' + h1 - 10;
            *p++ = h0 < 10 ? '0' + h0 : 'A' + h0 - 10;
        }
        *p++ = ':'; *p++ = ' ';

        for (i = row; i < end; i++) {
            unsigned char b = ax25_buf[i];
            unsigned char hi = (b >> 4) & 0x0F;
            unsigned char lo = b & 0x0F;
            *p++ = hi < 10 ? '0' + hi : 'A' + hi - 10;
            *p++ = lo < 10 ? '0' + lo : 'A' + lo - 10;
            *p++ = ' ';
        }
        *p = '\0';
        si4463_debug_msg(strTmp);
    }
    }

    /* FIELD_2_LENGTH = encoded length */
    si4463_set_property(0x12, 0x11, (unsigned char)((enc_len >> 8) & 0x1F));
    si4463_set_property(0x12, 0x12, (unsigned char)(enc_len & 0xFF));

    /* Reset TX FIFO */
    {
        unsigned char cmd[2];
        cmd[0] = SI4463_CMD_FIFO_INFO;
        cmd[1] = 0x01;
        si4463_send_cmd(cmd, 2);
        si4463_wait_cts();
    }

    /* The TX FIFO is 64 B and every encoded frame is larger (a typical
       beacon is ~117 B, max ~218 B). Prime 60 B, start TX, then refill. */
    SI4463_CS_LOW();
    { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
    spi2_transfer(SI4463_CMD_WRITE_TX_FIFO);
    {
        unsigned int chunk;
        unsigned int sent = 0;
        chunk = enc_len <= 60 ? enc_len : 60;
        for (i = 0; i < chunk; i++) spi2_transfer(ax25_buf[i]);
        sent = chunk;
        SI4463_CS_HIGH();

        /* TX_LEN = enc_len explicitly, since packet-handler framing is off */
        {
            unsigned char tx_cmd[7];
            tx_cmd[0] = SI4463_CMD_START_TX;
            tx_cmd[1] = 0x00;                       /* channel 0 */
            tx_cmd[2] = 0x30;                       /* immediate; READY on done */
            tx_cmd[3] = (unsigned char)((enc_len >> 8) & 0x1F);
            tx_cmd[4] = (unsigned char)(enc_len & 0xFF);
            tx_cmd[5] = 0x00;
            tx_cmd[6] = 0x00;
            si4463_send_cmd(tx_cmd, 7);
            if (!si4463_wait_cts()) {
                si4463_debug_msg(STR_TASK_SI4463 "ax25: START_TX CTS timeout!");
                return;
            }
        }

        /* ~2 ms for the chip to settle into TX before polling FIFO_INFO */
        { unsigned int d; for (d = 0; d < 1000u; d++) { __builtin_nop(); } }

        /* Refill as the chip drains (64 B takes ~427 ms at 1200 bps). NOP
           loops measure ~2.05 us per iteration on this board. Keep waits
           here short relative to the drain time: a too-long back-off
           underflowed the FIFO and cut off the FCS. A FIFO_INFO CTS
           timeout is retried, not treated as the end of the frame. */
        {
            /* Retry budget: ~2.2 ms per spin. A 218 B frame needs ~470
               spins; 2000 also caps a stuck radio at ~4.4 s, inside
               task_wdt's ~9 s margin. */
            unsigned int safety = 2000;
            unsigned int read_fails = 0;
            while (sent < enc_len && safety-- > 0) {
                unsigned char cmd[2];
                unsigned char resp[2];
                unsigned int  free_space;

                __builtin_clrwdt();

                cmd[0] = SI4463_CMD_FIFO_INFO;
                cmd[1] = 0x00;
                si4463_send_cmd(cmd, 2);
                if (!si4463_read_response(resp, 2)) {
                    read_fails++;
                    /* ~2 ms, then retry */
                    unsigned int d;
                    for (d = 0; d < 1000u; d++) { __builtin_nop(); }
                    continue;
                }
                free_space = resp[1];

                if (free_space < 16) {
                    /* ~2 ms back-off */
                    unsigned int d;
                    for (d = 0; d < 1000u; d++) { __builtin_nop(); }
                    continue;
                }

                chunk = enc_len - sent;
                if (chunk > free_space) chunk = free_space;

                SI4463_CS_LOW();
                { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
                spi2_transfer(SI4463_CMD_WRITE_TX_FIFO);
                for (i = 0; i < chunk; i++) spi2_transfer(ax25_buf[sent + i]);
                SI4463_CS_HIGH();
                sent += chunk;
            }
            if (sent < enc_len) {
                sprintf(strTmp, STR_TASK_SI4463 "ax25: FIFO refill timeout (%u/%u, %u read fails)",
                        sent, enc_len, read_fails);
                si4463_debug_msg(strTmp);
            }
        }
    }

    sprintf(strTmp, STR_TASK_SI4463 "ax25: TX %u bytes", enc_len);
    si4463_debug_msg(strTmp);

    /* Wait for the last FIFO load to go out: at most 64 B = 427 ms.
       Each outer iteration is ~102 ms, so 12 is ~1.2 s. This always runs
       in full; with up to three frames per beacon cycle, keep the total
       well inside task_wdt's ~9 s margin. If frames come out truncated
       (FCS missing), raise it and re-check the watchdog margin. */
    {
        unsigned int d;
        unsigned int w;
        for (w = 0; w < 12; w++) {
            for (d = 0; d < 50000u; d++) { __builtin_nop(); }
            __builtin_clrwdt();
        }
    }
}
