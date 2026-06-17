#ifndef SI4463_H
#define SI4463_H

#include <Arduino.h>
#include <SPI.h>
#include "config.h"

// ============================================================
// Si4463 API Commands
// ============================================================
#define CMD_NOP             0x00
#define CMD_POWER_UP        0x02
#define CMD_PART_INFO       0x01
#define CMD_SET_PROPERTY    0x11
#define CMD_GET_PROPERTY    0x12
#define CMD_GPIO_PIN_CFG    0x13
#define CMD_FIFO_INFO       0x15
#define CMD_GET_INT_STATUS  0x20
#define CMD_START_RX        0x32
#define CMD_START_TX        0x31
#define CMD_CHANGE_STATE    0x34
#define CMD_READ_CMD_BUFF   0x44
#define CMD_READ_RX_FIFO    0x77
#define CMD_WRITE_TX_FIFO   0x66
#define CMD_REQUEST_DEVICE_STATE 0x33

// ============================================================
// Si4463 Property Groups
// ============================================================
#define PROP_GLOBAL         0x00
#define PROP_INT_CTL        0x01
#define PROP_FRR_CTL        0x02
#define PROP_PREAMBLE       0x10
#define PROP_SYNC           0x11
#define PROP_PKT            0x12
#define PROP_MODEM          0x20
#define PROP_PA             0x22
#define PROP_FREQ_CONTROL   0x40
#define PROP_PA_MODE                 0x2200
#define PROP_PA_PWR_LVL              0x2201

// Key property indices
#define PROP_GLOBAL_XO_TUNE          0x0000
#define PROP_GLOBAL_CONFIG           0x0003
#define PROP_INT_CTL_ENABLE          0x0100
#define PROP_INT_CTL_PH_ENABLE       0x0101
#define PROP_INT_CTL_MODEM_ENABLE    0x0102
#define PROP_PREAMBLE_TX_LENGTH      0x1000
#define PROP_PREAMBLE_CONFIG         0x1004
#define PROP_SYNC_CONFIG             0x1100
#define PROP_SYNC_BITS_31_24         0x1101
#define PROP_SYNC_BITS_23_16         0x1102
#define PROP_PKT_CRC_CONFIG          0x1200
#define PROP_PKT_CONFIG1             0x1206
#define PROP_PKT_LEN                 0x1208
#define PROP_PKT_LEN_FIELD_SOURCE    0x1209
#define PROP_PKT_FIELD_1_LENGTH_12_8 0x120D
#define PROP_PKT_FIELD_1_LENGTH_7_0  0x120E
#define PROP_PKT_FIELD_1_CONFIG      0x120F
#define PROP_PKT_FIELD_1_CRC_CONFIG  0x1210
#define PROP_PKT_FIELD_2_LENGTH_12_8 0x1211
#define PROP_PKT_FIELD_2_LENGTH_7_0  0x1212
#define PROP_PKT_FIELD_2_CONFIG      0x1213
#define PROP_PKT_FIELD_2_CRC_CONFIG  0x1214
#define PROP_PKT_RX_FIELD_1_LENGTH_12_8 0x1221
#define PROP_PKT_RX_FIELD_1_LENGTH_7_0  0x1222
#define PROP_PKT_RX_FIELD_1_CONFIG      0x1223
#define PROP_PKT_RX_FIELD_1_CRC_CONFIG  0x1224
#define PROP_PKT_RX_FIELD_2_LENGTH_12_8 0x1225
#define PROP_PKT_RX_FIELD_2_LENGTH_7_0  0x1226
#define PROP_PKT_RX_FIELD_2_CONFIG      0x1227
#define PROP_PKT_RX_FIELD_2_CRC_CONFIG  0x1228
#define PROP_MODEM_MOD_TYPE          0x2000
#define PROP_MODEM_MAP_CONTROL       0x2001
#define PROP_MODEM_DATA_RATE_2       0x2003
#define PROP_MODEM_DATA_RATE_1       0x2004
#define PROP_MODEM_DATA_RATE_0       0x2005
#define PROP_MODEM_TX_NCO_MODE_3     0x2006
#define PROP_MODEM_TX_NCO_MODE_2     0x2007
#define PROP_MODEM_TX_NCO_MODE_1     0x2008
#define PROP_MODEM_TX_NCO_MODE_0     0x2009
#define PROP_MODEM_FREQ_DEV_2        0x200A
#define PROP_MODEM_FREQ_DEV_1        0x200B
#define PROP_MODEM_FREQ_DEV_0        0x200C
#define PROP_MODEM_FREQ_OFFSET_1     0x200D
#define PROP_MODEM_FREQ_OFFSET_0     0x200E
#define PROP_MODEM_BCR_OSR_1         0x2022
#define PROP_MODEM_BCR_OSR_0         0x2023
#define PROP_MODEM_BCR_NCO_OFFSET_2  0x2024
#define PROP_MODEM_BCR_NCO_OFFSET_1  0x2025
#define PROP_MODEM_BCR_NCO_OFFSET_0  0x2026
#define PROP_MODEM_BCR_GAIN_1        0x2027
#define PROP_MODEM_BCR_GAIN_0        0x2028
#define PROP_MODEM_AFC_GEAR          0x202C
#define PROP_MODEM_AFC_WAIT          0x202E
#define PROP_MODEM_AFC_GAIN_1        0x2030
#define PROP_MODEM_AFC_GAIN_0        0x2031
#define PROP_MODEM_AFC_LIMITER_1     0x2032
#define PROP_MODEM_AFC_LIMITER_0     0x2033
#define PROP_MODEM_AGC_CONTROL       0x2035
#define PROP_MODEM_AGC_WINDOW_SIZE   0x2038
#define PROP_MODEM_OOK_CNT1         0x2042
#define PROP_MODEM_RSSI_THRESH       0x204A
#define PROP_MODEM_CLKGEN_BAND       0x2051
#define PROP_FREQ_CONTROL_INTE       0x4000
#define PROP_FREQ_CONTROL_FRAC_2     0x4001
#define PROP_FREQ_CONTROL_FRAC_1     0x4002
#define PROP_FREQ_CONTROL_FRAC_0     0x4003
#define PROP_FREQ_CONTROL_CHANNEL_STEP_SIZE_1 0x4004
#define PROP_FREQ_CONTROL_CHANNEL_STEP_SIZE_0 0x4005

// CTS value
#define SI4463_CTS  0xFF

// Interrupt flags
#define INT_PH_RX_FIFO_ALMOST_FULL  0x01
#define INT_PH_PACKET_RX            0x10
#define INT_PH_PACKET_SENT          0x20
#define INT_PH_CRC_ERROR            0x08

// ============================================================
// Function Declarations
// ============================================================

void si4463_init(void);
bool si4463_reset(void);
bool si4463_power_up(void);
bool si4463_send_command(const uint8_t *cmd, uint8_t len);
bool si4463_read_response(uint8_t *resp, uint8_t len);
bool si4463_wait_cts(void);
bool si4463_set_property(uint16_t prop, uint8_t value);
bool si4463_set_properties(uint16_t start_prop, const uint8_t *values, uint8_t count);
bool si4463_get_part_info(uint16_t *part);
bool si4463_get_int_status(uint8_t *ph_pend, uint8_t *modem_pend, uint8_t *chip_pend);
bool si4463_clear_int_status(void);
bool si4463_configure_rf(void);
bool si4463_set_rf_switch_rx(void);
bool si4463_set_rf_switch_tx(void);
bool si4463_start_rx(uint8_t channel, uint16_t rx_len);
bool si4463_start_tx(uint8_t channel, uint16_t tx_len);
uint8_t si4463_read_rx_fifo(uint8_t *buf, uint8_t max_len);
bool si4463_write_tx_fifo(const uint8_t *buf, uint8_t len);
bool si4463_fifo_info(uint8_t *rx_count, bool reset_fifos);
uint8_t si4463_get_device_state(void);

#endif // SI4463_H
