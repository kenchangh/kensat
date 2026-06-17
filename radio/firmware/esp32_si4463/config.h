#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// ESP32-S3 to Si4463 Pin Mapping (via J1 header)
// Adjust these to match your wiring
// ============================================================

// SPI pins (ESP32-S3 SPI2/HSPI defaults)
#define PIN_SCLK  12
#define PIN_MOSI  11
#define PIN_MISO  13
#define PIN_NSEL  10  // Chip select (active low)

// Control pins
#define PIN_SDN   9   // Shutdown (active high)
#define PIN_NIRQ  14  // Interrupt (active low)

// ============================================================
// RF Configuration
// ============================================================

#define XTAL_FREQ_HZ      30000000UL  // 30 MHz crystal (Y1 on board)
#define RF_FREQ_HZ        437080000UL // 437.08 MHz
#define DATA_RATE_BPS      9600       // 9600 bps GFSK
#define FREQ_DEVIATION_HZ  4800       // +/- 4.8 kHz deviation
#define RX_BANDWIDTH_HZ    100000     // 100 kHz RX bandwidth

// Packet config
#define MAX_PACKET_LEN     64         // Max payload bytes
#define PREAMBLE_LEN       8          // Preamble length in nibbles (4 bytes)
#define SYNC_WORD          0x2DD4     // Sync word (16-bit)

// Mode: set to true for TX, false for RX
#define RF_MODE_TX         false

// TX settings
#define TX_POWER           0x00       // PA power level (0x00–0x7F)
#define TX_INTERVAL_MS     1000       // Delay between TX packets (ms)

// SPI speed
#define SPI_CLOCK_HZ       4000000    // 4 MHz SPI clock

// Serial
#define SERIAL_BAUD        115200

#endif // CONFIG_H
