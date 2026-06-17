/*
 * ESP32-S3 + Si4463 RX/TX
 *
 * Initializes the Si4463 transceiver on the kensat_radio board.
 * Set RF_MODE_TX in config.h to switch between RX and TX mode.
 *
 * Wiring: ESP32-S3 connects to Si4463 via J1 header.
 * See config.h for pin mapping.
 */

#include "si4463.h"
#include "config.h"

static uint8_t rx_buf[MAX_PACKET_LEN + 1];
static uint16_t tx_counter = 0;

static void send_test_packet(void) {
  char msg[MAX_PACKET_LEN];
  uint8_t len = snprintf(msg, sizeof(msg), "KENSAT %04u", tx_counter++);

  // Reset TX FIFO
  si4463_fifo_info(nullptr, true);

  // Write length byte + payload
  si4463_write_tx_fifo((const uint8_t *)&len, 1);
  si4463_write_tx_fifo((const uint8_t *)msg, len);

  // Start TX: length = 1 (length byte) + payload
  si4463_start_tx(0, len + 1);

  Serial.print("[TX] ");
  Serial.print(len);
  Serial.print(" bytes: ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000);

  Serial.println("=== kensat_radio Si4463 ===");

  si4463_init();
  Serial.println("SPI initialized");

  if (!si4463_reset()) {
    Serial.println("ERROR: Reset failed");
    while (1) delay(1000);
  }
  Serial.println("Reset complete");

  if (!si4463_power_up()) {
    Serial.println("ERROR: Power-up failed (check wiring)");
    while (1) delay(1000);
  }
  Serial.println("Power-up complete");

  uint16_t part = 0;
  if (si4463_get_part_info(&part)) {
    Serial.print("Part: Si");
    Serial.println(part, HEX);
    if (part != 0x4463) {
      Serial.println("WARNING: Expected Si4463");
    }
  } else {
    Serial.println("ERROR: Could not read part info");
    while (1) delay(1000);
  }

  if (!si4463_configure_rf()) {
    Serial.println("ERROR: RF configuration failed");
    while (1) delay(1000);
  }
  Serial.println("RF configured: 437.08 MHz, GFSK 9600 bps");

  si4463_clear_int_status();
  si4463_fifo_info(nullptr, true);

  if (RF_MODE_TX) {
    si4463_set_rf_switch_tx();
    Serial.println("TX mode active (AS179 → TX path)");
  } else {
    si4463_set_rf_switch_rx();
    if (!si4463_start_rx(0, 0)) {
      Serial.println("ERROR: Start RX failed");
      while (1) delay(1000);
    }
    Serial.println("RX mode active (AS179 → RX path)");
  }

  // Read and print chip state
  uint8_t state = si4463_get_device_state();
  Serial.print("Chip state: 0x0");
  Serial.print(state, HEX);
  Serial.print(" (");
  switch (state) {
    case 0x01: Serial.print("Sleep");    break;
    case 0x02: Serial.print("SPI Active"); break;
    case 0x03: Serial.print("Ready");    break;
    case 0x05: Serial.print("TX Tune");  break;
    case 0x06: Serial.print("RX Tune");  break;
    case 0x07: Serial.print("TX");       break;
    case 0x08: Serial.print("RX");       break;
    default:   Serial.print("Unknown");  break;
  }
  Serial.println(")");
  Serial.println();
}

void loop() {
  if (RF_MODE_TX) {
    send_test_packet();
    delay(TX_INTERVAL_MS);
    return;
  }

  // RX mode: check nIRQ
  if (digitalRead(PIN_NIRQ) == LOW) {
    uint8_t ph_pend = 0, modem_pend = 0, chip_pend = 0;

    if (si4463_get_int_status(&ph_pend, &modem_pend, &chip_pend)) {

      if (ph_pend & INT_PH_PACKET_RX) {
        uint8_t len = si4463_read_rx_fifo(rx_buf, sizeof(rx_buf));
        if (len > 0) {
          Serial.print("[RX] ");
          Serial.print(len);
          Serial.print(" bytes: ");
          for (uint8_t i = 0; i < len; i++) {
            if (rx_buf[i] < 0x10) Serial.print("0");
            Serial.print(rx_buf[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
          Serial.print("     ASCII: ");
          for (uint8_t i = 0; i < len; i++) {
            Serial.print(isPrintable(rx_buf[i]) ? (char)rx_buf[i] : '.');
          }
          Serial.println();
          Serial.println();
        }
        si4463_fifo_info(nullptr, true);
        si4463_start_rx(0, 0);
      }

      if (ph_pend & INT_PH_CRC_ERROR) {
        Serial.println("[RX] CRC error — packet discarded");
        si4463_fifo_info(nullptr, true);
        si4463_start_rx(0, 0);
      }
    }
  }

  delay(1);
}
