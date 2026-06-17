/******************************************************************************
  ESP32-S3 SPI Slave Mock - Si4463 Radio Board Stub

  Pure GPIO bit-bang SPI slave — no ESP-IDF SPI driver.
  Polls CS, reads bits on SCLK rising edge, responds with CTS on MISO.

  Wiring (CubeSat Kit Bus -> ESP32-S3-WROOM):
    SCLK  (IO.11, H1-13)  -> GPIO 5   (avoid GPIO 12 - flash conflict)
    MISO  (IO.10, H1-14)  -> GPIO 13
    MOSI  (IO.9,  H1-15)  -> GPIO 6   (avoid GPIO 11 - flash conflict)
    nSEL  (IO.8,  H1-16)  -> GPIO 10   (CS, active low)
    nIRQ  (IO.31, H2-17)  -> GPIO 9    (interrupt output, active low)
    SDN   (IO.30, H2-18)  -> GPIO 14   (shutdown input, active high = off)
    DGND  (H2-29)         -> GND

  Board: ESP32-S3 Dev Module
  Framework: Arduino (ESP32 core >= 2.0)
******************************************************************************/

#include <string.h>
#include "soc/gpio_reg.h"

// Pin definitions
#define PIN_MOSI    6
#define PIN_MISO   13
#define PIN_SCLK    5
#define PIN_CS     10
#define PIN_NIRQ    9
#define PIN_SDN    14

// Fast GPIO reads via register (all pins < 32, so GPIO_IN_REG works)
#define FAST_READ(pin) ((REG_READ(GPIO_IN_REG) >> (pin)) & 1)
#define FAST_CS()      FAST_READ(PIN_CS)
#define FAST_SCLK()    FAST_READ(PIN_SCLK)
#define FAST_MOSI()    FAST_READ(PIN_MOSI)

// Si4463 command bytes
#define CMD_POWER_UP       0x02
#define CMD_GET_INT_STATUS 0x20
#define CMD_START_RX       0x32
#define CMD_CHANGE_STATE   0x34
#define CMD_READ_CMD_BUFF  0x44

// RX buffer
#define BUF_LEN  16
static uint8_t rx_buf[BUF_LEN];

// SDN state tracking
static bool sdn_prev = false;
static unsigned long trans_count = 0;

// Diagnostic: capture raw GPIO register snapshots during transactions
#define DIAG_SAMPLES 64
static uint32_t diag_gpio[DIAG_SAMPLES];
static volatile int diag_idx = 0;


/******************************************************************************
  Print hex bytes helper
******************************************************************************/
static void print_hex(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        if (i < len - 1) Serial.print(' ');
    }
}


/******************************************************************************
  Parse and print a received Si4463 command
******************************************************************************/
static void parse_command(const uint8_t *buf, size_t len) {
    if (len == 0) return;

    uint8_t cmd = buf[0];

    switch (cmd) {
        case CMD_POWER_UP:
            Serial.print("CMD: POWER_UP");
            if (len > 1) {
                Serial.print(" [");
                print_hex(buf + 1, len - 1);
                Serial.print("]");
            }
            Serial.println();
            break;

        case CMD_GET_INT_STATUS:
            Serial.println("CMD: GET_INT_STATUS");
            break;

        case CMD_START_RX:
            Serial.print("CMD: START_RX");
            if (len > 1) {
                Serial.print(" [");
                print_hex(buf + 1, len - 1);
                Serial.print("]");
            }
            Serial.println();
            break;

        case CMD_CHANGE_STATE:
            Serial.print("CMD: CHANGE_STATE to 0x");
            if (len > 1) {
                if (buf[1] < 0x10) Serial.print('0');
                Serial.print(buf[1], HEX);
                switch (buf[1]) {
                    case 0x03: Serial.print(" (READY)"); break;
                    case 0x08: Serial.print(" (RX)"); break;
                }
            }
            Serial.println();
            break;

        case CMD_READ_CMD_BUFF:
            Serial.println("CMD: READ_CMD_BUFF -> CTS");
            break;

        default:
            Serial.print("CMD: 0x");
            if (cmd < 0x10) Serial.print('0');
            Serial.print(cmd, HEX);
            if (len > 1) {
                Serial.print(" [");
                print_hex(buf + 1, len - 1);
                Serial.print("]");
            }
            Serial.println();
            break;
    }
}


/******************************************************************************
  Bit-bang SPI slave: receive one byte on SCLK rising edges.
  Outputs tx_byte on MISO simultaneously.
  Returns received byte. Assumes CS is already LOW.
******************************************************************************/
static inline uint8_t IRAM_ATTR spi_slave_byte(uint8_t tx_byte) {
    uint8_t rx_byte = 0;
    uint32_t gpio_val;

    for (int i = 7; i >= 0; i--) {
        // Set MISO for this bit (fast write)
        if ((tx_byte >> i) & 1) {
            REG_WRITE(GPIO_OUT_W1TS_REG, 1 << PIN_MISO);
        } else {
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << PIN_MISO);
        }

        // Wait for SCLK rising edge (with CS abort)
        do {
            gpio_val = REG_READ(GPIO_IN_REG);
            // Capture diagnostic samples
            if (diag_idx < DIAG_SAMPLES) {
                diag_gpio[diag_idx++] = gpio_val;
            }
            if (gpio_val & (1 << PIN_CS)) return rx_byte;
        } while (!(gpio_val & (1 << PIN_SCLK)));

        // Sample MOSI on rising edge
        if (gpio_val & (1 << PIN_MOSI)) {
            rx_byte |= (1 << i);
        }

        // Wait for SCLK falling edge (with CS abort)
        do {
            gpio_val = REG_READ(GPIO_IN_REG);
            if (gpio_val & (1 << PIN_CS)) return rx_byte;
        } while (gpio_val & (1 << PIN_SCLK));
    }

    return rx_byte;
}


/******************************************************************************
  Print diagnostic GPIO captures
******************************************************************************/
static void print_diag(void) {
    if (diag_idx == 0) return;

    int count = diag_idx;
    if (count > DIAG_SAMPLES) count = DIAG_SAMPLES;

    // Count SCLK high/low samples for summary
    int sclk_high = 0;
    for (int i = 0; i < count; i++) {
        if (diag_gpio[i] & (1 << PIN_SCLK)) sclk_high++;
    }

    Serial.print("--- GPIO DIAG (");
    Serial.print(count);
    Serial.print(" samples, SCLK HIGH in ");
    Serial.print(sclk_high);
    Serial.println(") ---");
    Serial.println("  #   RAW_HEX    CS SCLK MOSI MISO SDN");

    // Print first 32 and last 8 samples to keep output manageable
    int print_limit = (count <= 40) ? count : 32;
    for (int i = 0; i < print_limit; i++) {
        uint32_t v = diag_gpio[i];
        int cs   = (v >> PIN_CS)   & 1;
        int sclk = (v >> PIN_SCLK) & 1;
        int mosi = (v >> PIN_MOSI) & 1;
        int miso = (v >> PIN_MISO) & 1;
        int sdn  = (v >> PIN_SDN)  & 1;

        char line[64];
        snprintf(line, sizeof(line), "  %2d  %08lX   %d    %d    %d    %d   %d",
                 i, (unsigned long)v, cs, sclk, mosi, miso, sdn);
        Serial.println(line);
    }
    if (count > 40) {
        Serial.println("  ... (truncated) ...");
        for (int i = count - 8; i < count; i++) {
            uint32_t v = diag_gpio[i];
            int cs   = (v >> PIN_CS)   & 1;
            int sclk = (v >> PIN_SCLK) & 1;
            int mosi = (v >> PIN_MOSI) & 1;
            int miso = (v >> PIN_MISO) & 1;
            int sdn  = (v >> PIN_SDN)  & 1;

            char line[64];
            snprintf(line, sizeof(line), "  %2d  %08lX   %d    %d    %d    %d   %d",
                     i, (unsigned long)v, cs, sclk, mosi, miso, sdn);
            Serial.println(line);
        }
    }
    Serial.println("--- END DIAG ---");
}


/******************************************************************************
  setup()
******************************************************************************/
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("=== ESP32-S3 Si4463 Mock Radio (GPIO diag) ===");

    // nIRQ: output, idle HIGH
    pinMode(PIN_NIRQ, OUTPUT);
    digitalWrite(PIN_NIRQ, HIGH);

    // SDN: input
    pinMode(PIN_SDN, INPUT_PULLDOWN);
    sdn_prev = digitalRead(PIN_SDN);

    // SPI pins: all GPIO, no SPI peripheral
    pinMode(PIN_CS, INPUT_PULLUP);
    pinMode(PIN_SCLK, INPUT);
    pinMode(PIN_MOSI, INPUT);
    pinMode(PIN_MISO, OUTPUT);
    digitalWrite(PIN_MISO, HIGH);  // Default MISO = 0xFF (CTS ready)

    Serial.print("SDN="); Serial.print(digitalRead(PIN_SDN));
    Serial.print(" CS="); Serial.print(digitalRead(PIN_CS));
    Serial.print(" SCLK="); Serial.print(digitalRead(PIN_SCLK));
    Serial.print(" MOSI="); Serial.println(digitalRead(PIN_MOSI));

    // Print GPIO_IN_REG address and pin bit positions for verification
    Serial.print("GPIO_IN_REG raw: 0x");
    Serial.println(REG_READ(GPIO_IN_REG), HEX);
    Serial.print("Pin bits: CS=bit"); Serial.print(PIN_CS);
    Serial.print(" SCLK=bit"); Serial.print(PIN_SCLK);
    Serial.print(" MOSI=bit"); Serial.print(PIN_MOSI);
    Serial.print(" MISO=bit"); Serial.println(PIN_MISO);

    // Monitor all SPI pins for 10 seconds to catch PIC24 pin test
    // PIC24 holds SCLK HIGH for 2s, then MOSI HIGH for 2s
    Serial.println("Monitoring SPI pins for 10 sec...");
    {
        unsigned long start = millis();
        int sclk_high = 0, mosi_high = 0, cs_low = 0, sdn_high = 0;
        int total = 0;
        unsigned long last_print = start;
        while (millis() - start < 10000) {
            uint32_t reg = REG_READ(GPIO_IN_REG);
            if (reg & (1 << PIN_SCLK)) sclk_high++;
            if (reg & (1 << PIN_MOSI)) mosi_high++;
            if (!(reg & (1 << PIN_CS))) cs_low++;
            if (reg & (1 << PIN_SDN))  sdn_high++;
            total++;

            // Print status every 2 seconds
            if (millis() - last_print >= 2000) {
                Serial.print("  @");
                Serial.print((millis() - start) / 1000);
                Serial.print("s: SCLK_H=");
                Serial.print(sclk_high);
                Serial.print(" MOSI_H=");
                Serial.print(mosi_high);
                Serial.print(" CS_L=");
                Serial.print(cs_low);
                Serial.print(" SDN_H=");
                Serial.print(sdn_high);
                Serial.print(" /");
                Serial.println(total);
                last_print = millis();
            }

            delayMicroseconds(10);
        }
        Serial.print("Final: SCLK_H=");
        Serial.print(sclk_high);
        Serial.print(" MOSI_H=");
        Serial.print(mosi_high);
        Serial.print(" CS_L=");
        Serial.print(cs_low);
        Serial.print(" SDN_H=");
        Serial.print(sdn_high);
        Serial.print(" /");
        Serial.println(total);
    }

    Serial.println("Waiting for PIC24 commands...");
    Serial.println();
}


/******************************************************************************
  loop()

  Polls for CS LOW, then bit-bangs SPI slave receive.
  Sends 0xFF (CTS) on MISO for all bytes.
******************************************************************************/
void loop() {
    // Monitor SDN transitions
    bool sdn_now = digitalRead(PIN_SDN);
    if (sdn_now != sdn_prev) {
        Serial.print("[SDN] ");
        Serial.println(sdn_now ? "HIGH (shutdown)" : "LOW (power up)");
        sdn_prev = sdn_now;
    }

    // Wait for CS LOW (transaction start)
    if (!FAST_CS()) {
        // Debounce: brief check that CS is still LOW (not a glitch)
        delayMicroseconds(1);
        if (FAST_CS()) return;

        // Reset diagnostic capture for this transaction
        diag_idx = 0;

        // CS is LOW — receive bytes until CS goes HIGH
        size_t rx_len = 0;
        memset(rx_buf, 0, BUF_LEN);

        while (!FAST_CS() && rx_len < BUF_LEN) {
            // Send 0xFF on MISO (CTS response for any byte position)
            rx_buf[rx_len] = spi_slave_byte(0xFF);
            rx_len++;
        }

        if (rx_len > 0) {
            trans_count++;

            // Count SCLK high samples in diagnostic
            int sclk_hi = 0;
            int count = diag_idx;
            if (count > DIAG_SAMPLES) count = DIAG_SAMPLES;
            for (int j = 0; j < count; j++) {
                if (diag_gpio[j] & (1 << PIN_SCLK)) sclk_hi++;
            }

            // Minimal one-line output to avoid blocking Serial
            Serial.print("[#");
            Serial.print(trans_count);
            Serial.print("] ");
            parse_command(rx_buf, rx_len);
            Serial.print("  RAW(");
            Serial.print(rx_len);
            Serial.print("B): ");
            print_hex(rx_buf, rx_len);
            Serial.print(" SCLK_HI=");
            Serial.print(sclk_hi);
            Serial.print("/");
            Serial.println(count);

            // Only print full diag for first transaction with SCLK activity
            if (sclk_hi > 0 && trans_count <= 5) {
                print_diag();
            }
        }
    }
}
