/******************************************************************************
KENSAT - Si4463 Radio Transceiver RX Task

Interfaces with Si4463 transceiver via SPI2 peripheral.
Cycles RX mode: enter RX -> wait 1 minute -> exit RX -> re-enter RX.

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


// Allow multiple PPS reconfigurations (default IOL1WAY=ON locks PPS
// permanently after first config, blocking our SPI2 PPS mapping).
// CONFIG3 is not defined elsewhere, so this is safe to add here.
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
 Sends a debug message directly to UART0 (RS-232) for manual testing.
******************************************************************************/
static void si4463_debug_msg(const char *msg) {
    user_debug_msg(msg);
}


static void spi2_init(void) {

    // Fix bus hazard from init.c: RG9(CS)=LOW, RE8(SDN)=LOW, RG7 output.
    // Deassert CS and shut down Si4463 before any SPI config.
    _LATG9 = 1;    // CS HIGH (deselect)
    _LATE8 = 1;    // SDN HIGH (shutdown Si4463)
    _TRISG7 = 1;   // MISO as input (stop driving against Si4463)

    // Ensure all PORTG and PORTE pins used are in digital mode.
    // RE8 (SDN) = AN24, RE9 (nIRQ) = AN25 — must clear analog bits.
    ANSG = 0x0000;
    ANSE = 0x0000;

    // Ensure push-pull (not open-drain) on SPI and control pins
    ODCG = 0x0000;
    ODCE = 0x0000;

    // Configure CS and SDN as GPIO outputs, inactive
    SI4463_CS_HIGH();
    SI4463_SDN_HIGH();

    // Pin directions for control pins (SPI pins set by peripheral)
    _TRISG9 = 0;    // nSEL (CS) as output (manual, not SPI SS)
    _TRISG7 = 1;    // MISO as input
    _TRISE8 = 0;    // SDN as output
    _TRISE9 = 1;    // nIRQ as input

    // Disable SPI2 before reconfiguring
    SPI2STATbits.SPIEN = 0;

    // Map PPS for SPI2:
    //   SDO2 (MOSI) -> RP19 (RG8)
    //   SCK2 (SCLK) -> RP21 (RG6)
    //   SDI2 (MISO) <- RP26 (RG7)
    __builtin_write_OSCCONL(OSCCON & ~(1 << 6));   // Unlock PPS
    RPOR9bits.RP19R  = 10;   // RP19 (RG8) = SDO2 (MOSI)
    RPOR10bits.RP21R = 11;   // RP21 (RG6) = SCK2OUT (SCLK)
    RPINR22bits.SDI2R = 26;  // SDI2 input from RP26 (RG7)
    __builtin_write_OSCCONL(OSCCON | (1 << 6));    // Lock PPS

    // Configure SPI2: Master, Mode 0 (CKP=0, CKE=1), 8-bit
    // Fcy assumed ~16 MHz. PPRE=01 (4:1), SPRE=110 (2:1) -> 16/(4*2) = 2 MHz
    // But bus has 2 connector hops, so slow down: PPRE=10 (4:1), SPRE=100 (4:1)
    // -> 16/(4*4) = 1 MHz
    SPI2CON1 = 0;
    SPI2CON1bits.MSTEN = 1;      // Master mode
    SPI2CON1bits.CKP   = 0;      // Clock idle LOW (CPOL=0)
    SPI2CON1bits.CKE   = 1;      // Data changes on idle->active (Mode 0)
    SPI2CON1bits.PPRE  = 0b10;   // Primary prescale 4:1 (16MHz/4/4 = 1MHz)
    SPI2CON1bits.SPRE  = 0b100;  // Secondary prescale 4:1 -> ~1 MHz
    SPI2CON2 = 0x0000;
    SPI2STATbits.SPIROV = 0;     // Clear overflow
    SPI2STATbits.SPIEN = 1;      // Enable SPI2

}


static unsigned char spi2_transfer(unsigned char data) {
    // Hardware SPI2 transfer
    SPI2STATbits.SPIROV = 0;        // Clear overflow flag
    SPI2BUF = data;                 // Start transfer
    while (!SPI2STATbits.SPIRBF) {} // Wait for receive complete
    return (unsigned char)SPI2BUF;  // Read received byte
}



/******************************************************************************
 Si4463 Command Functions
******************************************************************************/
static void si4463_send_cmd(const unsigned char *cmd, unsigned int len) {
    unsigned int i;

    SI4463_CS_LOW();
    // CS setup time: long delay to let ESP32 slave detect CS LOW and
    // enter spi_slave_byte() before first SCLK edge.
    // 5000 NOPs @ 4 MIPS = ~1.25 ms
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

 Polls READ_CMD_BUFF for CTS, then reads resp_len response bytes in the
 same CS transaction.  This is required because the Si4463 only makes
 response data available during the CTS read — once CS goes high, it's gone.
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
            // CTS received — read response bytes in same CS transaction
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

 Read Fast Response Registers A and B in a single SPI transaction.
 No CTS polling needed. ~3 µs vs ~100 µs for GET_INT_STATUS.
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

    // Flush SPI bus — toggle CS with 0xFF bytes to clear any stale state
    for (j = 0; j < 16; j++) {
        SI4463_CS_LOW();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
        spi2_transfer(0xFF);
        SI4463_CS_HIGH();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
    }

    // Assert SDN (shutdown) for >1 ms
    SI4463_SDN_HIGH();
    __builtin_clrwdt();
    for (i = 0; i < 50000u; i++) { __builtin_nop(); }

    // Release SDN to power up the chip
    SI4463_SDN_LOW();

    // Wait for POR (~200 ms for margin)
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


// Si4463 Rev C2 (ROMID=0x06) patch — fixes modem errata.
// From tinySA project, PATCHID=0xCA90, SIZE=512 bytes.
static const unsigned char si4463_patch_cmds[][9] = {
    {0x04,0x21,0x71,0x4B,0x00,0x00,0xDC,0x95,0x00},
    {0x05,0xA6,0x22,0x21,0xF0,0x41,0x5B,0x26,0x00},
    {0x05,0x1E,0x12,0xBD,0x5A,0xC2,0x52,0x41,0x00},  // dummy last byte
};
// Full 64-command patch stored as raw SPI data
#define SI4463_PATCH_CMD_COUNT  64

static unsigned char si4463_apply_patch(void) {
    // 64 patch commands, each 8 data bytes + 1 length prefix
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

    // Each patch command is 8 bytes, prefixed with length 0x08
    for (i = 0; i < 64; i++) {
        cmd[0] = 0x08;  // length prefix
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

    // Wait for CTS before sending command (matches ESP32 driver flow)
    si4463_wait_cts();

    // POWER_UP: FUNC=1, crystal mode, 30 MHz (no patch — test patch
    // for ROMID=0x06 broke the chip; need production patch from WDS)
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

 Sends PART_INFO (0x01) and prints the chip part number to UART.
 A real Si4463 returns part=0x4463. Returns 0xFFFF if SPI not connected.
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

    // resp[1]=CHIPREV, resp[2]=PART[15:8], resp[3]=PART[7:0],
    // resp[4]=PBUILD, resp[5]=ID[15:8], resp[6]=ID[7:0],
    // resp[7]=CUSTOMER, resp[0]=CHIPREV
    sprintf(strTmp, STR_TASK_SI4463 "PART_INFO: part=0x%02X%02X, rev=0x%02X, id=0x%02X%02X",
            resp[1], resp[2], resp[0], resp[3], resp[4]);
    si4463_debug_msg(strTmp);
}


static unsigned char si4463_start_rx(void) {
    // START_RX command: channel 0, start immediately, no timeout
    unsigned char cmd[8];

    cmd[0] = SI4463_CMD_START_RX;
    cmd[1] = 0x00;        // Channel 0
    cmd[2] = 0x00;        // Condition: start immediately
    cmd[3] = 0x00;        // RX_LEN[15:8] = 0 (use packet handler)
    cmd[4] = 0x00;        // RX_LEN[7:0]
    cmd[5] = SI4463_STATE_NOCHANGE;  // RXTIMEOUT_STATE
    cmd[6] = SI4463_STATE_READY;     // RXVALID_STATE — host re-enters RX
    cmd[7] = SI4463_STATE_READY;     // RXINVALID_STATE — host re-enters RX

    si4463_send_cmd(cmd, 8);
    return si4463_wait_cts();
}


static unsigned char si4463_start_tx(void) {
    // START_TX command: channel 0, start immediately, use packet handler
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

 SET_PROPERTY command to configure Si4463 registers.
 Property address is 16-bit: high byte = group, low byte = index.
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

 Full RF configuration for 437.08 MHz GFSK.
 Matches pluto_tx.py packet format:
   [Preamble 4B] [Sync 0x2DD4] [Len 1B] [Payload ≤64B] [CRC-16 2B]

 ASYMMETRIC LINK RATES -- READ BEFORE CHANGING ANY MODEM PROPERTY
 ----------------------------------------------------------------
 TX (downlink) runs at 1200 bps, ±601 Hz.  RX (uplink) stays at 9600 bps,
 ±3004 Hz.  This is deliberate, not an oversight.

 The downlink was dropped to 1200 bps to buy ~9 dB of link margin: the PA
 on this board measures +12 dBm, not the +20 dBm the datasheet reference
 match implies (PA_PWR_LVL is already 0x7F = max, so the shortfall is in
 the match network, not the drive level), and at 9600 bps the 520 km link
 does not close.  The uplink has no such problem -- it is limited by the
 *ground* station's TX power, which is not the constraint -- so there is
 nothing to gain by slowing it, and a great deal to lose.

 The two directions are configured by DISJOINT property groups:
   TX rate/deviation : MODEM_DATA_RATE (0x20/0x03..0x05), MODEM_FREQ_DEV
                       (0x20/0x0C), with TX_NCO_MODE (0x20/0x06..0x09).
   RX recovery       : MODEM_BCR_* (0x20/0x18..), MODEM_AFC_* (0x20/0x24,
                       0x20/0x30), and the CHFLT taps (0x21/0x00..0x23).
 Changing the TX pair therefore does NOT disturb uplink demodulation.

 If you ever DO want the uplink at 1200 as well, the BCR/AFC/CHFLT blocks
 must be regenerated in WDS for 1200 bps / ±360 Hz and replaced as a set.
 Do not hand-edit them: the CHFLT values are FIR filter taps, not scalars,
 and a partial edit yields a radio that still transmits but silently stops
 hearing commands.  Keep ground-station/pluto_tx.py at 9600 to match this
 block.
 ----------------------------------------------------------------

 Ported from radio/firmware/esp32_si4463/si4463.cpp.
******************************************************************************/
static unsigned char si4463_configure_rf(void) {

    __builtin_clrwdt();

    // =====================================================================
    // Based on WDS 3.2.11.0 config, MODIFIED for AX.25 / G3RUH:
    //   - TX rate 9600 -> 1200 bps (DATA_RATE 0x05DC00 -> 0x00BB80)
    //   - TX deviation ±4.8 kHz -> ±601 Hz (FREQ_DEV 0x150 -> 0x2A)
    //   - PREAMBLE_TX_LENGTH = 0 (we supply scrambled flag bytes in-band)
    //   - SYNC SKIP_TX = 1 (no chip-generated sync word)
    //   - FIELD_1_LENGTH = 0, FIELD_1/2_CRC_CONFIG = 0 (no chip framing/CRC)
    // 437.08 MHz, 2-GFSK, TX 1200 bps / RX 9600 bps, source = FIFO.
    // Sync word bytes are still loaded in SYNC_BITS but never transmitted.
    // =====================================================================

    // GLOBAL
    si4463_set_property(0x00, 0x00, 0x7F);  // XO_TUNE
    si4463_set_property(0x00, 0x03, 0x20);  // GLOBAL_CONFIG: high-perf

    // INT_CTL: enable PH + MODEM interrupts
    {
        static const unsigned char d[] = { 0x03, 0x38, 0x30 };
        //                                  ^     ^     ^
        //                                  |     |     +-- MODEM_INT: PREAMBLE_DETECT(4) + SYNC_DETECT(5)
        //                                  |     +-- PH_INT: RX_FIFO_ALMOST_FULL + PACKET_RX + CRC_ERROR
        //                                  +-- INT_CTL_ENABLE: PH(0) + MODEM(1)
        si4463_set_properties(0x01, 0x00, d, 3);
    }

    // FRR_CTL: map FRR_A=PH_PEND(0x04), FRR_B=MODEM_PEND(0x07)
    // Si4463 API: 0x04=PH_PEND, 0x07=MODEM_PEND, 0x0A=CHIP_PEND
    {
        static const unsigned char d[] = { 0x04, 0x07, 0x00, 0x00 };
        si4463_set_properties(0x02, 0x00, d, 4);
    }

    // PREAMBLE — TX length 0 nibbles. AX.25 supplies its own scrambled
    // flag-byte preamble inside the FIFO data.
    {
        static const unsigned char d[] = { 0x00, 0x14, 0x00, 0x0F, 0x31, 0x00, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x10, 0x00, d, 9);
    }

    __builtin_clrwdt();

    // SYNC. SYNC_CONFIG bit 7 SKIP_TX = 1 → chip does NOT emit a sync word
    // on TX; AX.25 framing handles its own start-of-frame (0x7E flags).
    {
        static const unsigned char d[] = { 0xB1, 0xB4, 0x2B, 0x00, 0x00, 0x00 };
        si4463_set_properties(0x11, 0x00, d, 6);
    }

    // PKT 0x00-0x0B. PKT_CRC_CONFIG=0x05 selects polynomial X16+X12+X5+1
    // (CCITT-16) per AN626; pluto_rx.py crc16_itu_t uses the same poly 0x1021
    // with seed 0x0000.
    {
        static const unsigned char d[] = { 0x05, 0x01, 0x08, 0xFF, 0xFF, 0x20, 0x02, 0x00, 0x2A, 0x01, 0x00, 0x30 };
        si4463_set_properties(0x12, 0x00, d, 12);
    }

    // PKT 0x0C-0x17. AX.25 path: no length byte, no chip-side CRC.
    //   FIELD_1_LENGTH = 0    → field 1 emits no bytes
    //   FIELD_1_CRC_CONFIG = 0
    //   FIELD_2_CRC_CONFIG = 0 → chip serializes raw FIFO bytes, no CRC
    // FIELD_2_LENGTH is updated per-packet by si4463_tx_ax25() to the actual
    // encoded byte count (see PKT_FIELD_2_LENGTH_7_0 writes there).
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

    // MODEM 0x00-0x0B.  TX side only -- these two properties set the
    // *transmit* symbol rate and deviation.  RX bit-clock recovery lives in
    // the BCR/AFC/CHFLT blocks below and is deliberately left at 9600 (see
    // the ASYMMETRIC LINK RATES note at the top of this function).
    //
    // DATA_RATE — WDS-original was 0x05DC00 (384000) for 9600 bps.  With
    // TX_NCO_MODE unchanged (TXOSR = 40, NCOMOD = 30 MHz):
    //     Rb = DATA_RATE * F_xtal / (TXOSR * NCOMOD) = DATA_RATE / 40
    // so 1200 bps needs DATA_RATE = 1200 * 40 = 48000 = 0x00BB80.
    {
        static const unsigned char d[] = { 0x03, 0x00, 0x07, 0x00, 0xBB, 0x80, 0x05, 0xC9, 0xC3, 0x80, 0x00, 0x00 };
        si4463_set_properties(0x20, 0x00, d, 12);
    }
    // FREQ_DEV — TX deviation.  Scale is 14.31 Hz/unit on this board (the
    // same slope as the FREQ_CONTROL FRAC shim below): 0xD2 = 210 gave
    // ±3004 Hz, which is h = 2*3004/9600 = 0.63 at 9600 bps.
    // 0x2A = 42 => ±601 Hz, i.e. h = 2*601/1200 = 1.00 at 1200 bps.
    //
    // WHY h = 1.0 AND NOT 0.6.  ±360 Hz (h = 0.6) was the first choice, to
    // mirror the old 9600 modulation index.  Measured through the real
    // satnogs_fsk_ax25 flowgraph, ±600 Hz is strictly better on both axes:
    //
    //   Eb/N0        16 dB    13 dB    11 dB      carrier offset  0 Hz  500 Hz
    //   ±360 (h=0.6) 16/20    11/20     3/20                     10/10   1/10
    //   ±600 (h=1.0) 19/20    18/20     7/20                     10/10   7/10
    //
    // ~2-3 dB of sensitivity and roughly double the usable carrier offset,
    // for 2.4 kHz of occupied bandwidth instead of 1.9 kHz -- irrelevant in
    // a 70 cm amateur allocation.  It also matches DELFI-PQ, the only flown
    // 1200 bps AX.25/G3RUH satellite in the gr-satellites catalogue that
    // declares a deviation at all (BINAR-1, KAFASAT and UmKA-1 omit it).
    //
    // Keep this in step with the `deviation:` field of the SatNOGS SatYAML
    // entry published for KENSAT: that field sets gr-satellites' Carson
    // pre-demod cutoff (|dev| + baud/2) AND
    // its quadrature-demod gain, so a mismatch between the two costs frames.
    // Leaving this at the 9600 value would give h = 2*3004/1200 = 5.0, far
    // outside that filter.
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

    // MODEM DSA — enabled, fast decay for inter-packet reset
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

    // FREQ_CONTROL — FRAC compensates for the crystal running high (no load
    // caps). Tuned empirically from two measured CW points on THIS board:
    //   FRAC 0x09C457 (640087) -> 436.6492 MHz
    //   FRAC 0x09FF76 (655222) -> 436.8658 MHz
    // => slope ~14.31 Hz/unit (~70 units/kHz; band outdiv=8).
    // Interpolating to 437.080 MHz: FRAC = 670191 = 0x0A39EF.
    // INTE=0x39 unchanged. FRAC is board-specific (per-crystal) -- re-measure
    // CW and recompute (ΔFRAC = (437.080 - meas_MHz) * 69900) on a new board.
    {
        static const unsigned char d[] = { 0x39, 0x0A, 0x39, 0xEF, 0x05, 0x76, 0x20, 0xFE };
        si4463_set_properties(0x40, 0x00, d, 8);
    }

    return 1;
}


/******************************************************************************
 si4463_gpio_cfg_rx() / si4463_gpio_cfg_tx()

 Configure AS179 RF switch via Si4463 GPIO2/GPIO3.
 RX: GPIO2=high, GPIO3=low   (RFC to RX antenna port)
 TX: GPIO2=low,  GPIO3=high  (RFC to TX/PA port)
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

 Public API to switch the Si4463 between RX, TX, and READY modes.
 Returns 1 on success (CTS received), 0 on failure (CTS timeout).
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

 Sends REQUEST_DEVICE_STATE (0x33), reads back current state and channel.
 Prints the result to UART.
******************************************************************************/
static void si4463_get_device_state(void) {
    unsigned char cmd[1];
    unsigned char resp[2];
    unsigned char state;
    unsigned char channel;
    const char *state_name;

    cmd[0] = SI4463_CMD_REQUEST_DEVICE_STATE;
    si4463_send_cmd(cmd, 1);

    // Read CTS + 2 response bytes (state, channel) in one CS transaction
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

 Sends GET_INT_STATUS (0x20) with all-zero args (don't clear pending IRQs
 selectively). Reads back 8 response bytes.  Returns the PH_PEND byte
 (response byte index 2) which contains PACKET_RX_PEND in bit 4.
******************************************************************************/
#define SI4463_PH_PEND_PACKET_RX  0x10   // bit 4 of PH_PEND
#define SI4463_PH_PEND_CRC_ERROR  0x20  // bit 5 of PH_PEND

static unsigned char last_modem_pend;   // MODEM_PEND from last GET_INT_STATUS

static unsigned char si4463_get_int_status(void) {
    unsigned char cmd[4];
    unsigned char resp[8];

    cmd[0] = SI4463_CMD_GET_INT_STATUS;
    cmd[1] = 0x00;   // PH_CLR_PEND  — don't clear
    cmd[2] = 0x00;   // MODEM_CLR_PEND — don't clear
    cmd[3] = 0x00;   // CHIP_CLR_PEND

    si4463_send_cmd(cmd, 4);

    // Read CTS + 8 response bytes in one CS transaction
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

 Queries FIFO_INFO (0x15) to get the number of bytes in the RX FIFO.
 Pass reset=1 to reset the RX FIFO after reading, 0 to leave it.
 Returns RX FIFO count (0 on failure).
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

 Reads exactly count bytes from the Si4463 RX FIFO via READ_RX_FIFO (0x77).
 Returns the number of bytes read.  Caller must ensure buf has room.
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

 Called from task_cmd to manually init the Si4463 and set RX mode.
 After init, enters a blocking poll loop watching nIRQ for incoming packets.
 On packet RX: reads FIFO, hex-dumps to UART, re-enters RX.
 Loop runs until reset ('r') or power cycle.
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

    // SPI2 MISO diagnostic: check raw pin, PPS, and read bytes
    {
        unsigned char raw_pin, rpinr_val, spi_byte;
        raw_pin = _RG7;
        rpinr_val = (unsigned char)(RPINR22 & 0x3F);
        // Do a dummy SPI read with CS asserted
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

    // Set GPIO for RX path: GPIO2=HIGH, GPIO3=LOW (cfg0, board default)
    si4463_gpio_cfg_rx();
    si4463_debug_msg(STR_TASK_SI4463 "s: GPIO cfg0 (G2=H G3=L), entering RX...");

    if (0) { /* GPIO sweep disabled — hardcoded cfg2 above */
        unsigned char cmd_ms[2];
        unsigned char ms_resp[8];
        unsigned char best_rssi, rssi_val, overall_best, best_cfg;
        unsigned int j;
        unsigned char cfg;

        overall_best = 0;
        best_cfg = 0;

        // Config 0: GPIO2=HIGH, GPIO3=LOW (original RX config)
        // Config 1: GPIO2=LOW,  GPIO3=HIGH (swapped)
        // Config 2: GPIO2=LOW,  GPIO3=LOW  (both off / direct path)
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

    // Drain stale UART1 RX characters before entering poll loop
    while (csk_uart1_getchar()) { }

    si4463_debug_msg(STR_TASK_SI4463 "s: Polling nIRQ... ('q' to exit)");

    poll_count = 0;
    while (1) {
        __builtin_clrwdt();

        // Check for 'q' on debug UART to exit; ignore other chars
        if (csk_uart1_getchar() == 'q') {
            si4463_set_mode(SI4463_MODE_READY);
            si4463_debug_msg(STR_TASK_SI4463 "s: Exited RX mode.");
            return;
        }

        // Check nIRQ — active low means an interrupt is pending
        if (SI4463_NIRQ_READ() == 0) {

            // Fast FRR triage: read PH_PEND without full GET_INT_STATUS (~3 µs)
            {
                unsigned char frr_ph, frr_mdm;
                si4463_read_frr(&frr_ph, &frr_mdm);

                // Only process packet handler events (PACKET_RX=0x10, CRC_ERROR=0x20)
                if (!(frr_ph & 0x30)) {
                    // Modem-only event (preamble, sync) — skip, chip stays in RX
                    goto skip_nirq;
                }
            }

            // PH event detected — do full GET_INT_STATUS to clear and read details
            ph_pend = si4463_get_int_status();

            if (ph_pend & 0x20) {
                // CRC error — flush FIFO and re-enter RX immediately
                n = si4463_fifo_info(0);
                if (n > 0 && n <= RX_FIFO_BUF_SIZE)
                    si4463_read_rx_fifo(rx_buf, n);
                si4463_fifo_info(0x03);
                si4463_set_mode(SI4463_MODE_RX);
                // Print AFTER re-entering RX (non-blocking for next packet)
                sprintf(strTmp, STR_TASK_SI4463 "CRC_ERR %u", (unsigned int)n);
                si4463_debug_msg(strTmp);
            }

            if (ph_pend & SI4463_PH_PEND_PACKET_RX) {
                // Read FIFO and re-enter RX FIRST, print AFTER
                n = si4463_fifo_info(0);
                if (n > RX_FIFO_BUF_SIZE) n = RX_FIFO_BUF_SIZE;
                if (n > 0) {
                    si4463_read_rx_fifo(rx_buf, n);
                }
                si4463_set_mode(SI4463_MODE_RX);

                // Single compact output line: "RX: KO6IKZ 0001 [12]"
                if (n > 1) {
                    unsigned char payload_len = rx_buf[0];
                    unsigned char j;
                    char *p = strTmp;
                    unsigned char end;

                    if (payload_len > n - 1) payload_len = n - 1;

                    // "RX: "
                    *p++ = 'R'; *p++ = 'X'; *p++ = ':'; *p++ = ' ';
                    // Payload as ASCII
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
            // Modem-only events (preamble, sync): chip stays in RX, no action needed
            skip_nirq: ;
        }

        // Pace the poll (~12 µs per iteration at 4 MIPS)
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }

        // Periodic heartbeat (no SPI — just a counter so user knows RX is alive)
        poll_count++;
        if (poll_count >= 100000u) {
            si4463_debug_msg(STR_TASK_SI4463 "RX: listening...");
            poll_count = 0;
        }
    }
}


/******************************************************************************
 si4463_cmd_tx()

 Called from task_cmd to manually init the Si4463 and set TX mode.
 Prints status to UART at each step.
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

    // PA power for OTA testing
    si4463_set_property(0x22, 0x01, 0x7F);  // PA_PWR_LVL = high for OTA

    si4463_debug_msg(STR_TASK_SI4463 "x: Setting RF switch to TX...");
    si4463_gpio_cfg_tx();

    si4463_debug_msg(STR_TASK_SI4463 "x: Transmitting packets ('q' to stop)...");

    for (pkt_num = 0; ; pkt_num++) {
        __builtin_clrwdt();

        // Check for 'q' on debug UART to exit; ignore other chars
        if (csk_uart1_getchar() == 'q') {
            si4463_debug_msg(STR_TASK_SI4463 "x: Stopped by user.");
            break;
        }

        // Reset TX FIFO
        {
            unsigned char cmd[2];
            cmd[0] = SI4463_CMD_FIFO_INFO;
            cmd[1] = 0x01;  // bit 0 = reset TX FIFO
            si4463_send_cmd(cmd, 2);
            si4463_wait_cts();
        }

        // Build payload: "KO6IKZ NNNN" (11 bytes)
        payload_len = 11;

        // Match PKT_FIELD_2_LENGTH to the actual payload bytes about to be
        // written to the FIFO. The chip's TX packet handler walks each field
        // for exactly FIELD_n_LENGTH bytes; the variable-length feature is
        // RX-only. Without this, FIELD_2_LENGTH stays at the configured 63B
        // max, the FIFO underflows after the real payload, and SEND_CRC for
        // field 2 never fires (chip stops modulating before the 16 CRC bits).
        si4463_set_property(0x12, 0x11, 0x00);          // FIELD_2_LENGTH_12_8
        si4463_set_property(0x12, 0x12, payload_len);   // FIELD_2_LENGTH_7_0

        tx_buf[0] = 'K'; tx_buf[1] = 'O'; tx_buf[2] = '6';
        tx_buf[3] = 'I'; tx_buf[4] = 'K'; tx_buf[5] = 'Z';
        tx_buf[6] = ' ';
        tx_buf[7] = '0' + ((pkt_num / 1000) % 10);
        tx_buf[8] = '0' + ((pkt_num / 100) % 10);
        tx_buf[9] = '0' + ((pkt_num / 10) % 10);
        tx_buf[10] = '0' + (pkt_num % 10);

        // Write to TX FIFO: [length byte] [payload]
        // The packet handler adds preamble, sync, and CRC automatically
        SI4463_CS_LOW();
        { unsigned int d; for (d = 0; d < 50; d++) { __builtin_nop(); } }
        spi2_transfer(SI4463_CMD_WRITE_TX_FIFO);
        spi2_transfer(payload_len);  // field 1: length byte
        for (i = 0; i < payload_len; i++) {
            spi2_transfer(tx_buf[i]);  // field 2: payload
        }
        SI4463_CS_HIGH();

        // Start TX
        if (!si4463_start_tx()) {
            si4463_debug_msg(STR_TASK_SI4463 "x: START_TX CTS timeout!");
            break;
        }

        sprintf(strTmp, STR_TASK_SI4463 "TX [%u]: KO6IKZ %04u (%u bytes)",
                pkt_num, pkt_num, (unsigned int)payload_len);
        si4463_debug_msg(strTmp);

        // Wait for TX complete.  Bench-only test path (si4463_cmd_tx), not
        // the flight beacon -- see si4463_tx_ax25() for that.
        // Packet = 64B preamble + 2B sync + 1B len + 11B payload + 2B CRC
        //        = 640 bits, which at the new 1200 bps TX rate is 533 ms.
        // Each loop is 50000 NOPs = ~102 ms measured (NOT the 3.125 ms the
        // original comment assumed), so 24 loops = ~2.45 s.  Comfortably
        // covers 533 ms; left as-is because this runs only on an operator
        // command, but note it blocks the scheduler for ~2.4 s when used.
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

 One-shot: init the chip and bring up an unmodulated continuous-wave carrier
 at 437.08 MHz, then return with the carrier still ON (chip parked in TX).
 Shared by si4463_cmd_cw() (interactive) and task_beacon (boot-time SA test).
 Returns 1 on success, 0 if any init step times out (carrier not running).
******************************************************************************/
/* Defined later (used here for the CW diagnostic readback). */
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

    /* Override modulation: MODEM_MOD_TYPE (0x20/0x00) = 0x00
       bits[2:0]=0 (CW), bits[5:3]=0 (PACKET source, unused in CW).
       This makes the PA emit a pure unmodulated carrier. */
    si4463_set_property(0x20, 0x00, 0x00);

    /* PA at max for SA/VNA visibility */
    si4463_set_property(0x22, 0x01, 0x7F);

    si4463_debug_msg(STR_TASK_SI4463 "cw: Setting RF switch to TX...");
    si4463_gpio_cfg_tx();

    /* START_TX: channel 0, condition=0x00 (start immediately, NEXT_STATE=
       NOCHANGE so chip stays in TX), TX_LEN=0, no delay. With MOD_TYPE=CW
       the modem emits continuous carrier until we CHANGE_STATE. */
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

    /* Diagnostic: confirm the chip is actually parked in TX (0x07) and that
       MOD_TYPE really latched to CW (0x00). If state reads back READY/other,
       the PH completed and dropped us out of TX -> no continuous carrier. */
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

 Reads the current device state and, if the chip has fallen out of TX (e.g.
 the packet handler completed its configured field length and transitioned to
 READY), re-issues START_TX to re-key the CW carrier. Returns the state read
 BEFORE any re-key (0x07 = was still in TX; 0xFF = state read failed).
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
        /* Carrier dropped — re-key. */
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

 Unmodulated continuous-wave carrier at 437.08 MHz for VNA S11 measurement.
 Press 'q' on the debug UART to stop.
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

        /* Pace + heartbeat so the user knows it's alive */
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


/******************************************************************************
 si4463_cmd_get_props()

 Diagnostic 'g' command: dumps PIC24 clock state and reads back Si4463
 properties via GET_PROPERTY so we can verify what the chip is actually
 programmed with (vs what we think we wrote). Does not reset or reconfigure
 the chip — assumes task_beacon already initialised it.
******************************************************************************/
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

/* Direct UART writer used only by si4463_cmd_get_props — sidesteps
   csk_uart1_msg_ts's tx_free busy-wait, which destabilises the system
   when called rapidly back-to-back (see "Known issues" in
   firmware/obc/README.md). uart2_puts blocks per-char on TX-empty, so
   back-to-back calls are self-throttling. */
static void dump_props_line(const char *s) {
    csk_uart1_puts(s);
    csk_uart1_puts(STR_CRLF);
}

void si4463_cmd_get_props(void) {
    unsigned char cmd[1];
    unsigned char resp[16];
    unsigned long fcy_est;

    /* PIC24 clock + timer state. We can compute Fcy from PR2 and the
       previously-observed tick rate, but reading OSCCON/CLKDIV tells us
       what oscillator is actually selected. */
    sprintf(strTmp, STR_TASK_SI4463 "g: OSCCON=0x%04X CLKDIV=0x%04X OSCTUN=0x%04X",
            (unsigned int)OSCCON, (unsigned int)CLKDIV, (unsigned int)OSCTUN);
    dump_props_line(strTmp);
    sprintf(strTmp, STR_TASK_SI4463 "g: T2CON=0x%04X PR2=%u U2BRG=%u",
            (unsigned int)T2CON, (unsigned int)PR2, (unsigned int)U2BRG);
    dump_props_line(strTmp);

    /* Decode COSC/NOSC bits: bits[14:12]=COSC, bits[10:8]=NOSC */
    {
        unsigned char cosc = (OSCCON >> 12) & 0x07;
        unsigned char nosc = (OSCCON >> 8) & 0x07;
        sprintf(strTmp, STR_TASK_SI4463 "g: COSC=%u NOSC=%u (0=FRC 1=FRCPLL 2=PRI 3=PRIPLL 4=SOSC 5=LPRC 6=- 7=FRCDIV)",
                (unsigned int)cosc, (unsigned int)nosc);
        dump_props_line(strTmp);
    }

    /* Estimate Fcy from PR2 if Salvo tick was meant to be 100 Hz.
       Real-world: empirical Fcy = PR2 * actual_tick_rate. */
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

    /* PA group 0x22/0x00..0x03 — confirm what the chip is actually running:
       PA_MODE, PA_PWR_LVL, PA_BIAS_CLKDUTY, PA_TC.
       Expected after init_beacon(): PA_MODE=08 PA_PWR_LVL=7F PA_BIAS=00 PA_TC=1D. */
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

    /* Re-decode MOD_TYPE so we can see modulation source (CW vs FIFO GFSK)
       at a glance without consulting the datasheet. */
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

Disabled for manual radio board testing. Use 's' command to set RX mode
and 'x' command to set TX mode via task_cmd instead.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_si4463_rx(void) {

#if 0  // Disabled for manual testing — use 's'/'x' commands in task_cmd
    // Initialize SPI2 peripheral and pin directions FIRST.
    spi2_init();

    // Reset and power up Si4463 (no USB messages to avoid RTOS yield issues)
    si4463_reset();
    si4463_power_up();

    // Main loop: cycle RX mode with WDT clearing
    while (1) {

        si4463_set_mode(SI4463_MODE_RX);

        // Wait ~10 seconds, yielding to scheduler. Split into <=255-tick chunks
        // because Salvo Pro-lib delays are 8-bit (OSBYTES_OF_DELAYS=1) and
        // OS_Delay(1000) would truncate mod 256 to 232 ticks. 4 x 250 = 1000.
        OS_Delay(250);
        OS_Delay(250);
        OS_Delay(250);
        OS_Delay(250);

        si4463_set_mode(SI4463_MODE_READY);

        // Brief pause
        OS_Delay(2);

    } /* while */
#endif

} /* task_si4463_rx() */


/******************************************************************************
****                                                                       ****
**                                                                           **
Half-duplex uplink support

There is one radio and one owner (task_beacon), so listening simply means
parking the chip in RX during the gap between beacons and dropping back to
READY/TX when it is time to transmit.  Two things have to be switched every
time the direction changes:

  1. The AS179 RF switch (Si4463 GPIO2/GPIO3) -- si4463_gpio_cfg_rx/tx().

  2. The packet-handler FIELD config, group 0x12 offsets 0x0C..0x17.  The
     downlink is AX.25/G3RUH, which needs the packet handler OUT of the way:
     si4463_configure_rf() leaves FIELD_1_LENGTH = 0 and both CRC configs at
     0 so the chip serialises raw FIFO bytes and si4463_tx_ax25() supplies
     its own flags, bit-stuffing, NRZI, scrambling and FCS.  Receiving that
     same format would mean descrambling and running HDLC over a raw bit
     stream in software, which the Si4463's packet handler cannot do for us.

     So the UPLINK deliberately uses the chip's native framing instead --
     preamble + sync 0x2DD4 + 1-byte length + payload + CRC-16 CCITT -- which
     is the format si4463_cmd_rx() has always used and that
     ground-station/pluto_tx.py already generates.  That needs
     FIELD_1_LENGTH = 1 with CRC_START, and FIELD_2 with SEND_CRC/CHECK_CRC,
     so the two field configs are swapped in and out around every
     transmission.

Downlink stays AX.25 (SatNOGS-decodable); uplink is the simple Si4463 frame.

**                                                                           **
****                                                                       ****
******************************************************************************/

/* Latched RSSI of the most recently received packet (raw Si4463 units;
   dBm ~= raw/2 - 134).  Zero until something is heard. */
static unsigned char si4463_rx_rssi = 0;


/* Packet-handler FIELD config for the AX.25 downlink: field 1 emits nothing,
   field 2 is raw FIFO bytes with no chip-side CRC.  si4463_tx_ax25() still
   overwrites FIELD_2_LENGTH per frame. */
static void si4463_pkt_cfg_tx(void) {
    static const unsigned char d[] =
        { 0x30, 0x00, 0x00, 0x04, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00 };
    si4463_set_properties(0x12, 0x0C, d, 12);
}


/* Packet-handler FIELD config for the uplink: field 1 = 1-byte length
   (CRC_START + CRC_ENABLE, not sent), field 2 = up to 63 B payload
   (CRC_START + SEND_CRC + CRC_ENABLE, i.e. CHECK_CRC on RX). */
static void si4463_pkt_cfg_rx(void) {
    static const unsigned char d[] =
        { 0x30, 0x00, 0x01, 0x04, 0x82, 0x00, 0x3F, 0x00, 0xA2, 0x00, 0x00, 0x00 };
    si4463_set_properties(0x12, 0x0C, d, 12);
}


/******************************************************************************
 si4463_rx_enable()

 Point the RF switch at the RX port, install the uplink packet-handler
 config, flush both FIFOs, clear stale interrupts and arm RX.  Called at the
 top of every listening window so the chip's state is deterministic even if a
 previous packet or a glitch left it somewhere unexpected.

 Contains no OS_Delay -- safe to call from a Salvo helper.
 Returns 1 if START_RX was accepted.
******************************************************************************/
unsigned char si4463_rx_enable(void) {
    si4463_change_state(SI4463_STATE_READY);
    si4463_pkt_cfg_rx();
    si4463_gpio_cfg_rx();
    si4463_fifo_info(0x03);          /* reset TX + RX FIFOs */
    si4463_get_int_status();         /* clear anything still pending */
    return si4463_start_rx();
}


/******************************************************************************
 si4463_rx_disable()

 Leave RX, restore the AX.25 field config and point the RF switch back at
 the PA.  Call before any si4463_tx_ax25().

 Contains no OS_Delay -- safe to call from a Salvo helper.
******************************************************************************/
void si4463_rx_disable(void) {
    si4463_change_state(SI4463_STATE_READY);
    si4463_pkt_cfg_tx();
    si4463_gpio_cfg_tx();
}


/******************************************************************************
 si4463_rx_poll()

 Non-blocking check for one received uplink frame.  Returns the payload
 length copied into buf (0 if nothing arrived), and re-arms RX itself.

 Cheap in the common case: nIRQ is a plain GPIO read, so a quiet poll costs
 no SPI traffic at all.  START_RX was issued with RXVALID_STATE = READY, so
 the chip parks after a good packet and has to be re-armed -- which is done
 here, before the payload is copied out, to minimise the deaf window.

 A modem-only interrupt (preamble / sync detect, i.e. a packet part-way in)
 is NOT re-armed: the chip is still in RX and restarting it would throw away
 the frame currently being demodulated.  GET_INT_STATUS has already cleared
 the pending bits, so nIRQ deasserts and the next poll sees a clean state.

 Contains no OS_Delay -- safe to call from a Salvo helper.
******************************************************************************/
unsigned char si4463_rx_poll(unsigned char *buf, unsigned char maxlen) {
    static unsigned char fifo_buf[RX_FIFO_BUF_SIZE];
    unsigned char ph_pend;
    unsigned char n, plen, i;

    if (SI4463_NIRQ_READ() != 0) {
        return 0;                    /* quiet -- no SPI transaction at all */
    }

    ph_pend = si4463_get_int_status();     /* also clears all pending IRQs */

    if (ph_pend & SI4463_PH_PEND_CRC_ERROR) {
        /* Corrupt frame: bin it and listen again. */
        si4463_fifo_info(0x02);
        si4463_start_rx();
        return 0;
    }

    if (!(ph_pend & SI4463_PH_PEND_PACKET_RX)) {
        return 0;                    /* modem-only event; still in RX */
    }

    n = si4463_fifo_info(0);
    if (n > RX_FIFO_BUF_SIZE) n = RX_FIFO_BUF_SIZE;
    if (n > 0) {
        si4463_read_rx_fifo(fifo_buf, n);
    }

    /* Latched RSSI for this packet: GET_MODEM_STATUS response byte 3.
       Must be read before RX is re-armed or it is overwritten. */
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

    si4463_fifo_info(0x02);          /* drop any residue */
    si4463_start_rx();               /* back to listening straight away */

    /* FIFO layout after a good packet: [length byte][payload...].
       The CRC is checked by the chip and not stored. */
    if (n < 2) {
        return 0;
    }
    plen = fifo_buf[0];
    if (plen > (unsigned char)(n - 1)) plen = (unsigned char)(n - 1);
    if (plen > maxlen)                plen = maxlen;

    for (i = 0; i < plen; i++) {
        buf[i] = fifo_buf[1 + i];
    }
    return plen;
}


/******************************************************************************
 si4463_last_rssi()

 Raw latched RSSI of the last received uplink frame (dBm ~= raw/2 - 134).
******************************************************************************/
unsigned char si4463_last_rssi(void) {
    return si4463_rx_rssi;
}


/******************************************************************************
 si4463_init_beacon()

 One-shot radio init for the beacon task. Called once at task_beacon
 startup because task_si4463_rx is disabled and no other task owns the
 chip. Matches the init sequence used by si4463_cmd_tx():
   spi2_init -> reset -> power_up -> configure_rf -> PA 0x7F -> gpio_cfg_tx.
 Leaves the chip in READY with the RF switch pointed at the TX/PA port.
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

    /* Full PA power for OTA reach */
    si4463_set_property(0x22, 0x01, 0x7F);

    si4463_debug_msg(STR_TASK_SI4463 "beacon: RF switch -> TX path.");
    si4463_gpio_cfg_tx();

    /* Chip is now in READY with TX path selected. */
    si4463_change_state(SI4463_STATE_READY);

    si4463_debug_msg(STR_TASK_SI4463 "beacon: init done.");
}


/******************************************************************************
 si4463_tx_ax25()

 Transmits one AX.25 UI frame over G3RUH 1k2 GFSK. info[] is the AX.25
 info field (typically an ASCII telemetry line). The full frame
 (preamble flags + addr + ctrl/PID + info + FCS + tail flags, NRZI
 + G3RUH scrambled) is rendered in a static scratch buffer and shoved
 into the Si4463 TX FIFO; the chip serializes the bytes with no
 framing of its own (preamble/sync/CRC all disabled in configure_rf).

 Assumes the chip has been initialised by si4463_init_beacon() and the
 RF switch is on the TX port. Coerces RX→READY first if necessary.
 Leaves the chip in READY after TX (START_TX condition 0x30).

 In verbose console mode ('q') a hex dump of the encoded bytes is printed
 to the debug UART so the OTA capture can be correlated against what was
 actually shifted out.
******************************************************************************/
void si4463_tx_ax25(const unsigned char *info, unsigned int info_len) {
    static unsigned char ax25_buf[512];
    unsigned int  enc_len;
    unsigned int  i;
    unsigned int  row;

    __builtin_clrwdt();

    /* Build the AX.25 bit stream (preamble flags + frame + tail flags,
       stuffed + NRZI + G3RUH scrambled, packed MSB-first to match the
       chip's MSB-first FIFO shift-out). */
    enc_len = ax25_build_ui(AX25_DST_CALL, AX25_DST_SSID,
                            AX25_SRC_CALL, AX25_SRC_SSID,
                            info, info_len,
                            ax25_buf, sizeof(ax25_buf));
    if (enc_len == 0) {
        si4463_debug_msg(STR_TASK_SI4463 "ax25: build failed (info too large?)");
        return;
    }

    /* If something left the chip in RX, drop to READY before TX, and make
       sure the packet handler is in raw-FIFO (AX.25) mode -- the uplink
       listening window swaps in a different FIELD config (see
       si4463_pkt_cfg_rx()) and must never be left in place for a downlink. */
    si4463_change_state(SI4463_STATE_READY);
    si4463_pkt_cfg_tx();

    /* Hex dump the encoded bytes, 16 per line.  ~6 lines of ~70 chars is
       ~500 chars; at 9600 baud that is ~0.5 s per frame in which
       si4463_debug_msg busy-waits on TX ring space and the CPU cannot idle,
       so it is off unless the console is in verbose mode ('q').  The ground
       station decodes the actual RF, so this is a correlation aid, not the
       record. */
    if (g_console_verbose) {
    for (row = 0; row < enc_len; row += 16) {
        char *p = strTmp;
        unsigned int end = row + 16;
        if (end > enc_len) end = enc_len;

        /* "ax25 hex NNN: " */
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

    /* Set FIELD_2_LENGTH to the encoded byte count (can exceed 255 for
       large frames, so program both halves). FIELD_1_LENGTH is already
       0 from configure_rf, so the chip emits only field-2 bytes. */
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

    /* Stream encoded bytes into the FIFO. The Si4463 holds only 64 B in
       the TX FIFO, and an encoded frame always exceeds that: the 16
       leading flags alone are 16 B, and a routine ~78 B ASCII beacon
       encodes to ~117 B (a full 179 B info field reaches ~218 B). So the
       refill loop below runs on EVERY frame, not just oversized ones --
       seed the FIFO with the first 60 B here, then top it up as the chip
       drains it. */
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

        /* Kick off TX. With PH framing disabled (SKIP_TX sync, length=0
           field 1, no CRC), TX_LEN=0 (let-PH-decide) is ambiguous — pass
           enc_len explicitly so the chip TXes exactly that many bytes
           from the FIFO and then returns to READY. */
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

        /* Let the chip settle into TX before we start polling FIFO_INFO.
           Without this, the first FIFO_INFO can return CTS-late and used
           to exit the loop prematurely.  ~2 ms is ample (see the timing
           note below); this was 16000 iterations = ~33 ms, which burned
           most of the FIFO's 53 ms of buffered airtime before the first
           top-up even happened. */
        { unsigned int d; for (d = 0; d < 1000u; d++) { __builtin_nop(); } }

        /* Top up the FIFO as the chip drains it. At 1200 bps each byte
           takes ~6.67 ms, so the 64 B FIFO drains in ~427 ms (it was ~53 ms
           at 9600). Safety budget
           bounds retries; a transient FIFO_INFO CTS timeout does not break
           out of the loop (that used to stop the frame after the initial
           60 bytes).

           NOP-LOOP TIMING: measured on this board, `for (d=0; d<50000u;
           d++) nop;` takes ~102 ms -- about 2.05 us per iteration, ~33x
           slower than the 3.125 ms/50k that the original waits here
           assumed.  The back-off below was 48000 iterations, i.e. ~98 ms:
           nearly twice the time it takes the chip to drain the entire
           FIFO.  The result was a deterministic underflow -- Pluto capture
           showed every frame transmitting 96.0 ms of real data followed by
           ~9 ms of unmodulated carrier, so the last ~10 bytes (including
           the FCS) never went out and no frame ever validated on the
           ground.  Keep every wait in this loop small relative to the
           427 ms drain time. */
        {
            /* SAFETY BUDGET at 1200 bps.  The worst frame this encoder can
               emit is 218 B; after the initial 64 B fill, the remaining
               154 B drain in 154*8/1200 = 1.03 s.  Each back-off spin is
               ~2.2 ms (1000 nops @ ~2.05 us + one FIFO_INFO transaction),
               so a healthy worst-case frame needs ~470 spins -- the old
               500 was only 1.1x that and would have tripped the "FIFO
               refill timeout" path on long frames.

               2000 gives 4.3x headroom.  Do NOT simply scale this with the
               baud drop (500 -> 4000): this counter also bounds how long a
               *sick* radio can block the cooperative scheduler, and
               4000 * 2.2 ms = 8.8 s would consume almost the entire 9.0 s
               of late margin task_wdt has against the TPS3430's ~55 s
               window.  2000 caps a pathological stall at ~4.4 s. */
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
                    /* short wait then retry, don't bail (~2 ms) */
                    unsigned int d;
                    for (d = 0; d < 1000u; d++) { __builtin_nop(); }
                    continue;
                }
                free_space = resp[1];

                if (free_space < 16) {
                    /* ~2 ms back-off: short against the 53 ms it takes the
                       chip to drain a full FIFO, so we never sleep through
                       the tail of a frame. */
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

    /* Wait for TX complete.

       CALIBRATION: the inner loop was assumed to run at ~3.125 ms per
       outer iteration, so 200 iterations were meant to be ~625 ms.  Timed
       against the console (tick stamps + host clock), 200 iterations
       actually take ~20.4 s -- ~102 ms per outer iteration, 33x the
       estimate.  That blocked the cooperative scheduler for 20 s per
       beacon, which starved task_wdt of its WDI kick and let the external
       TPS3430 reset the OBC every ~55 s.

       SIZING AT 1200 bps.  This wait is UNCONDITIONAL -- it always burns
       its full duration, so oversizing it is not free.  What it has to
       cover is not the whole frame's airtime but only what is still
       sitting in the FIFO when the refill loop above exits, which is at
       most one full FIFO: 64 B * 8 / 1200 = 427 ms, regardless of frame
       length.  (The refill loop itself already spans the rest of the
       airtime, since it blocks waiting for drain.)

       12 iterations ~= 1.22 s = 2.9x that 427 ms residual.  Going to the
       ~20 that a naive "scale it with the baud rate" argument suggests
       would cost 2.04 s per frame; with up to three frames per beacon
       cycle (beacon + LLM + ACK) that is ~7.1 s of blocking against
       task_wdt's 9.0 s of late margin, leaving under 2 s.  At 12 the same
       worst case is ~4.7 s and the kick still lands near 50.7 s, well
       inside the ~55 s TPS3430 deadline.

       If the console ever shows truncated frames (real data followed by
       unmodulated carrier, FCS missing), raise this -- but re-check the
       watchdog margin in the same pass. */
    {
        unsigned int d;
        unsigned int w;
        for (w = 0; w < 12; w++) {
            for (d = 0; d < 50000u; d++) { __builtin_nop(); }
            __builtin_clrwdt();
        }
    }
}
