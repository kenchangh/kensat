#include "si4463.h"

static SPIClass *spi = nullptr;

// ============================================================
// Low-level SPI helpers
// ============================================================

static void spi_select(void) {
  digitalWrite(PIN_NSEL, LOW);
}

static void spi_deselect(void) {
  digitalWrite(PIN_NSEL, HIGH);
}

void si4463_init(void) {
  // Configure pins
  pinMode(PIN_SDN, OUTPUT);
  pinMode(PIN_NSEL, OUTPUT);
  pinMode(PIN_NIRQ, INPUT_PULLUP);

  digitalWrite(PIN_SDN, HIGH);   // Keep in shutdown initially
  digitalWrite(PIN_NSEL, HIGH);  // Deselect

  // Init SPI
  spi = new SPIClass(HSPI);
  spi->begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_NSEL);
}

bool si4463_reset(void) {
  // SDN high for at least 10us, then low
  digitalWrite(PIN_SDN, HIGH);
  delay(10);
  digitalWrite(PIN_SDN, LOW);
  delay(15);  // Wait for POR (typ 6ms)
  return true;
}

bool si4463_wait_cts(void) {
  // Poll CTS via READ_CMD_BUFF
  uint32_t timeout = millis() + 500;
  while (millis() < timeout) {
    spi_select();
    spi->beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
    spi->transfer(CMD_READ_CMD_BUFF);
    uint8_t cts = spi->transfer(0xFF);
    spi->endTransaction();
    spi_deselect();

    if (cts == SI4463_CTS) {
      return true;
    }
    delayMicroseconds(100);
  }
  return false;  // Timeout
}

bool si4463_send_command(const uint8_t *cmd, uint8_t len) {
  if (!si4463_wait_cts()) return false;

  spi_select();
  spi->beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  for (uint8_t i = 0; i < len; i++) {
    spi->transfer(cmd[i]);
  }
  spi->endTransaction();
  spi_deselect();
  return true;
}

bool si4463_read_response(uint8_t *resp, uint8_t len) {
  uint32_t timeout = millis() + 500;
  while (millis() < timeout) {
    spi_select();
    spi->beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
    spi->transfer(CMD_READ_CMD_BUFF);
    uint8_t cts = spi->transfer(0xFF);

    if (cts == SI4463_CTS) {
      for (uint8_t i = 0; i < len; i++) {
        resp[i] = spi->transfer(0xFF);
      }
      spi->endTransaction();
      spi_deselect();
      return true;
    }

    spi->endTransaction();
    spi_deselect();
    delayMicroseconds(100);
  }
  return false;
}

// ============================================================
// POWER_UP command
// ============================================================

bool si4463_power_up(void) {
  // POWER_UP: boot from internal, XTAL, 30 MHz
  uint8_t cmd[] = {
    CMD_POWER_UP,
    0x01,       // Function: PRO (full functionality)
    0x01,       // XTAL options: TCXO = 0, use crystal
    (uint8_t)((XTAL_FREQ_HZ >> 24) & 0xFF),
    (uint8_t)((XTAL_FREQ_HZ >> 16) & 0xFF),
    (uint8_t)((XTAL_FREQ_HZ >> 8) & 0xFF),
    (uint8_t)(XTAL_FREQ_HZ & 0xFF)
  };

  if (!si4463_send_command(cmd, sizeof(cmd))) return false;
  delay(20);  // POR completion
  return si4463_wait_cts();
}

// ============================================================
// Property access
// ============================================================

bool si4463_set_property(uint16_t prop, uint8_t value) {
  uint8_t group = (prop >> 8) & 0xFF;
  uint8_t index = prop & 0xFF;

  uint8_t cmd[] = {
    CMD_SET_PROPERTY,
    group,
    1,      // Number of properties
    index,
    value
  };
  return si4463_send_command(cmd, sizeof(cmd));
}

bool si4463_set_properties(uint16_t start_prop, const uint8_t *values, uint8_t count) {
  uint8_t group = (start_prop >> 8) & 0xFF;
  uint8_t index = start_prop & 0xFF;

  uint8_t cmd[4 + count];
  cmd[0] = CMD_SET_PROPERTY;
  cmd[1] = group;
  cmd[2] = count;
  cmd[3] = index;
  memcpy(&cmd[4], values, count);
  return si4463_send_command(cmd, 4 + count);
}

// ============================================================
// Part info
// ============================================================

bool si4463_get_part_info(uint16_t *part) {
  uint8_t cmd[] = { CMD_PART_INFO };
  if (!si4463_send_command(cmd, 1)) return false;

  uint8_t resp[8];
  if (!si4463_read_response(resp, 8)) return false;

  *part = ((uint16_t)resp[1] << 8) | resp[2];
  return true;
}

// ============================================================
// Interrupt status
// ============================================================

bool si4463_get_int_status(uint8_t *ph_pend, uint8_t *modem_pend, uint8_t *chip_pend) {
  // GET_INT_STATUS with no clearing mask (read all, clear all)
  uint8_t cmd[] = { CMD_GET_INT_STATUS, 0x00, 0x00, 0x00 };
  if (!si4463_send_command(cmd, sizeof(cmd))) return false;

  uint8_t resp[8];
  if (!si4463_read_response(resp, 8)) return false;

  if (ph_pend)    *ph_pend    = resp[2];
  if (modem_pend) *modem_pend = resp[4];
  if (chip_pend)  *chip_pend  = resp[6];
  return true;
}

bool si4463_clear_int_status(void) {
  return si4463_get_int_status(nullptr, nullptr, nullptr);
}

// ============================================================
// FIFO info
// ============================================================

bool si4463_fifo_info(uint8_t *rx_count, bool reset_fifos) {
  uint8_t cmd[] = { CMD_FIFO_INFO, (uint8_t)(reset_fifos ? 0x03 : 0x00) };
  if (!si4463_send_command(cmd, sizeof(cmd))) return false;

  uint8_t resp[2];
  if (!si4463_read_response(resp, 2)) return false;

  if (rx_count) *rx_count = resp[0];
  return true;
}

// ============================================================
// RF configuration for 435 MHz GFSK 9600 bps
// ============================================================

bool si4463_configure_rf(void) {
  // --- Interrupt control: enable packet handler interrupts ---
  si4463_set_property(PROP_INT_CTL_ENABLE, 0x03);       // Enable PH and modem interrupts
  si4463_set_property(PROP_INT_CTL_PH_ENABLE, 0x38);    // PACKET_RX + PACKET_SENT + CRC_ERROR

  // --- Preamble: 4 bytes (8 nibbles), standard pattern ---
  si4463_set_property(PROP_PREAMBLE_TX_LENGTH, PREAMBLE_LEN);
  si4463_set_property(PROP_PREAMBLE_CONFIG, 0x31);       // Standard preamble

  // --- Sync word: 2 bytes ---
  si4463_set_property(PROP_SYNC_CONFIG, 0x01);            // 2-byte sync word
  si4463_set_property(PROP_SYNC_BITS_31_24, (SYNC_WORD >> 8) & 0xFF);
  si4463_set_property(PROP_SYNC_BITS_23_16, SYNC_WORD & 0xFF);

  // --- Packet config: variable length, CRC-16 ---
  si4463_set_property(PROP_PKT_CRC_CONFIG, 0x05);        // CRC-16 ITU-T seed 0xFFFF
  si4463_set_property(PROP_PKT_CONFIG1, 0x80);            // PH_FIELD_SPLIT
  si4463_set_property(PROP_PKT_LEN, 0x0A);               // Variable length, field 1 = length byte
  si4463_set_property(PROP_PKT_LEN_FIELD_SOURCE, 0x01);  // Length from field 1

  // Field 1: 1 byte (length byte)
  si4463_set_property(PROP_PKT_FIELD_1_LENGTH_12_8, 0x00);
  si4463_set_property(PROP_PKT_FIELD_1_LENGTH_7_0, 0x01);
  si4463_set_property(PROP_PKT_FIELD_1_CONFIG, 0x00);
  si4463_set_property(PROP_PKT_FIELD_1_CRC_CONFIG, 0x82); // CRC start, enable

  // Field 2: up to MAX_PACKET_LEN bytes (payload)
  si4463_set_property(PROP_PKT_FIELD_2_LENGTH_12_8, 0x00);
  si4463_set_property(PROP_PKT_FIELD_2_LENGTH_7_0, MAX_PACKET_LEN);
  si4463_set_property(PROP_PKT_FIELD_2_CONFIG, 0x00);
  si4463_set_property(PROP_PKT_FIELD_2_CRC_CONFIG, 0x02); // CRC check on RX

  // RX field config (mirrors TX fields)
  si4463_set_property(PROP_PKT_RX_FIELD_1_LENGTH_12_8, 0x00);
  si4463_set_property(PROP_PKT_RX_FIELD_1_LENGTH_7_0, 0x01);
  si4463_set_property(PROP_PKT_RX_FIELD_1_CONFIG, 0x00);
  si4463_set_property(PROP_PKT_RX_FIELD_1_CRC_CONFIG, 0x82);
  si4463_set_property(PROP_PKT_RX_FIELD_2_LENGTH_12_8, 0x00);
  si4463_set_property(PROP_PKT_RX_FIELD_2_LENGTH_7_0, MAX_PACKET_LEN);
  si4463_set_property(PROP_PKT_RX_FIELD_2_CONFIG, 0x00);
  si4463_set_property(PROP_PKT_RX_FIELD_2_CRC_CONFIG, 0x02);

  // --- CLKGEN band: 420–470 MHz ---
  si4463_set_property(PROP_MODEM_CLKGEN_BAND, 0x0A);  // BAND=2 (outdiv=8, 420-470 MHz) + SY_SEL=1

  // --- Modem: GFSK, 9600 bps ---
  si4463_set_property(PROP_MODEM_MOD_TYPE, 0x03);         // GFSK, source from packet handler
  si4463_set_property(PROP_MODEM_MAP_CONTROL, 0x00);      // Normal mapping

  // Data rate: 9600 bps
  // DATA_RATE = data_rate * 2^20 / XTAL
  // 9600 * 1048576 / 30000000 = 335544 = 0x051EB8
  uint8_t dr[] = { 0x00, 0x25, 0x80 };  // ~9600 bps
  si4463_set_properties(PROP_MODEM_DATA_RATE_2, dr, 3);

  // TX NCO modulator: XTAL / 10 = 3 MHz
  // NCO_MODE = XTAL / 10 = 3000000 = 0x002DC6C0
  uint8_t nco[] = { 0x05, 0x2D, 0xC6, 0xC0 };  // NCOMOD[25:24]=0, TXOSR=10x
  si4463_set_properties(PROP_MODEM_TX_NCO_MODE_3, nco, 4);

  // Frequency deviation: 4800 Hz
  // FREQ_DEV = deviation * 2^19 / XTAL * outdiv
  // For 435 MHz, outdiv=8: 4800 * 524288 * 8 / 30000000 = 671 = 0x00029F
  uint8_t dev[] = { 0x00, 0x02, 0x9F };
  si4463_set_properties(PROP_MODEM_FREQ_DEV_2, dev, 3);

  // BCR/OSR for 9600 bps RX
  // OSR = XTAL / (data_rate * 10) for standard decimation
  // Simplified: use oversampling ratio ~312
  uint8_t bcr_osr[] = { 0x01, 0x38 };  // OSR = 312
  si4463_set_properties(PROP_MODEM_BCR_OSR_1, bcr_osr, 2);

  // BCR NCO offset: 2^22 / OSR = 4194304 / 312 = 13443 = 0x003483
  uint8_t bcr_nco[] = { 0x00, 0x34, 0x83 };
  si4463_set_properties(PROP_MODEM_BCR_NCO_OFFSET_2, bcr_nco, 3);

  // BCR gain
  uint8_t bcr_gain[] = { 0x00, 0xD2 };
  si4463_set_properties(PROP_MODEM_BCR_GAIN_1, bcr_gain, 2);

  // AFC
  si4463_set_property(PROP_MODEM_AFC_GEAR, 0x54);
  si4463_set_property(PROP_MODEM_AFC_WAIT, 0x36);
  uint8_t afc_gain[] = { 0x80, 0x02 };
  si4463_set_properties(PROP_MODEM_AFC_GAIN_1, afc_gain, 2);
  uint8_t afc_lim[] = { 0x00, 0x30 };
  si4463_set_properties(PROP_MODEM_AFC_LIMITER_1, afc_lim, 2);

  // AGC
  si4463_set_property(PROP_MODEM_AGC_CONTROL, 0xE2);
  si4463_set_property(PROP_MODEM_AGC_WINDOW_SIZE, 0x11);

  // RSSI threshold
  si4463_set_property(PROP_MODEM_RSSI_THRESH, 0x27);

  // --- Frequency control: 435 MHz ---
  // INTE = floor(freq / (XTAL/outdiv * 2)) - 1
  // outdiv for 420-470 MHz band = 8
  // freq_vco = freq * outdiv = 435e6 * 8 = 3.48 GHz
  // INTE = floor(3.48e9 / (2 * 30e6)) - 1 = floor(58) - 1 = 57
  // FRAC = (freq_vco / (2 * XTAL) - INTE - 1) * 2^19
  // 437.08 MHz with outdiv=8, 30 MHz XTAL:
  //   f = (INTE + FRAC/2^19) * 2 * XTAL / outdiv
  //   INTE=57 (0x39), FRAC=669690 (0x0A37FA)
  // BUG FIX: was 0x02380C (~429.6 MHz, wrong formula)
  si4463_set_property(PROP_FREQ_CONTROL_INTE, 0x39);
  uint8_t frac[] = { 0x0A, 0x37, 0xFA };
  si4463_set_properties(PROP_FREQ_CONTROL_FRAC_2, frac, 3);

  // Channel step size: 0 (single channel)
  uint8_t step[] = { 0x00, 0x00 };
  si4463_set_properties(PROP_FREQ_CONTROL_CHANNEL_STEP_SIZE_1, step, 2);

  // --- PA config ---
  si4463_set_property(PROP_PA_MODE, 0x08);       // Class-E match, single-ended
  si4463_set_property(PROP_PA_PWR_LVL, TX_POWER);

  return true;
}

// ============================================================
// RF switch (AS179) control via GPIO2/GPIO3
// GPIO3 → V1, GPIO2 → V2
// ============================================================

bool si4463_set_rf_switch_rx(void) {
  // RX: GPIO2=high (V2), GPIO3=low (V1) → RFC to J2
  uint8_t cmd[] = {
    CMD_GPIO_PIN_CFG,
    0x00,   // GPIO0 - no change
    0x00,   // GPIO1 - no change
    0x03,   // GPIO2 - drive high
    0x02,   // GPIO3 - drive low
    0x00,   // NIRQ - default
    0x00,   // SDO - default
    0x00    // GEN_CONFIG
  };
  if (!si4463_send_command(cmd, sizeof(cmd))) return false;

  // Read response (7 bytes) to complete the command
  uint8_t resp[7];
  if (!si4463_read_response(resp, 7)) return false;

  Serial.print("GPIO_PIN_CFG resp: ");
  for (int i = 0; i < 7; i++) {
    if (resp[i] < 0x10) Serial.print("0");
    Serial.print(resp[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  return true;
}

bool si4463_set_rf_switch_tx(void) {
  // TX: GPIO2=low (V2), GPIO3=high (V1) → RFC to J3
  uint8_t cmd[] = {
    CMD_GPIO_PIN_CFG,
    0x00,   // GPIO0 - no change
    0x00,   // GPIO1 - no change
    0x02,   // GPIO2 - drive low
    0x03,   // GPIO3 - drive high
    0x00,   // NIRQ - default
    0x00,   // SDO - default
    0x00    // GEN_CONFIG
  };
  if (!si4463_send_command(cmd, sizeof(cmd))) return false;

  uint8_t resp[7];
  if (!si4463_read_response(resp, 7)) return false;

  Serial.print("GPIO_PIN_CFG resp: ");
  for (int i = 0; i < 7; i++) {
    if (resp[i] < 0x10) Serial.print("0");
    Serial.print(resp[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  return true;
}

// ============================================================
// Start RX
// ============================================================

bool si4463_start_rx(uint8_t channel, uint16_t rx_len) {
  uint8_t cmd[] = {
    CMD_START_RX,
    channel,
    0x00,                         // Condition: start immediately
    (uint8_t)((rx_len >> 8) & 0xFF),
    (uint8_t)(rx_len & 0xFF),
    0x08,                         // RX timeout: no change (stay in RX)
    0x03,                         // RX valid: go to READY
    0x08                          // RX invalid (CRC error): stay in RX
  };
  return si4463_send_command(cmd, sizeof(cmd));
}

// ============================================================
// Start TX
// ============================================================

bool si4463_start_tx(uint8_t channel, uint16_t tx_len) {
  uint8_t cmd[] = {
    CMD_START_TX,
    channel,
    0x30,                         // Condition: start immediately, return to READY on complete
    (uint8_t)((tx_len >> 8) & 0xFF),
    (uint8_t)(tx_len & 0xFF),
    0x00                          // TX delay: none
  };
  return si4463_send_command(cmd, sizeof(cmd));
}

// ============================================================
// Write TX FIFO
// ============================================================

bool si4463_write_tx_fifo(const uint8_t *buf, uint8_t len) {
  if (!si4463_wait_cts()) return false;

  spi_select();
  spi->beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  spi->transfer(CMD_WRITE_TX_FIFO);
  for (uint8_t i = 0; i < len; i++) {
    spi->transfer(buf[i]);
  }
  spi->endTransaction();
  spi_deselect();
  return true;
}

// ============================================================
// Read RX FIFO
// ============================================================

uint8_t si4463_read_rx_fifo(uint8_t *buf, uint8_t max_len) {
  // Get FIFO byte count
  uint8_t rx_count = 0;
  if (!si4463_fifo_info(&rx_count, false)) return 0;
  if (rx_count == 0) return 0;
  if (rx_count > max_len) rx_count = max_len;

  spi_select();
  spi->beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  spi->transfer(CMD_READ_RX_FIFO);
  for (uint8_t i = 0; i < rx_count; i++) {
    buf[i] = spi->transfer(0xFF);
  }
  spi->endTransaction();
  spi_deselect();

  return rx_count;
}

// ============================================================
// Device state
// ============================================================

uint8_t si4463_get_device_state(void) {
  uint8_t cmd[] = { CMD_REQUEST_DEVICE_STATE };
  if (!si4463_send_command(cmd, 1)) return 0;

  uint8_t resp[2];
  if (!si4463_read_response(resp, 2)) return 0;

  return resp[0] & 0x0F;  // Lower nibble is the state
}
