/******************************************************************************
KENSAT - Si4463 Radio Transceiver RX Task

Si4463 transceiver control via SPI2. Cycles RX mode every 1 minute.

******************************************************************************/
#ifndef __task_si4463_h
#define __task_si4463_h


// Si4463 operating modes for si4463_set_mode()
#define SI4463_MODE_RX      0
#define SI4463_MODE_TX      1
#define SI4463_MODE_READY   2

// Function declarations
extern void task_si4463_rx(void);
extern unsigned char si4463_set_mode(unsigned char mode);
extern void si4463_cmd_rx(void);
extern void si4463_cmd_tx(void);
extern void si4463_cmd_cw(void);
extern unsigned char si4463_start_cw(void);
extern unsigned char si4463_cw_keepalive(void);
extern void si4463_cmd_get_props(void);
extern void si4463_init_beacon(void);
extern void si4463_tx_ax25(const unsigned char *info, unsigned int info_len);

/* Half-duplex uplink.  The radio listens in the gap between downlinks:
   si4463_rx_enable() at the top of a listening window, si4463_rx_poll()
   as often as convenient, si4463_rx_disable() before transmitting.
   None of these yield, so they are safe to call from a Salvo helper. */
extern unsigned char si4463_rx_enable(void);
extern void          si4463_rx_disable(void);
extern unsigned char si4463_rx_poll(unsigned char *buf, unsigned char maxlen);
extern unsigned char si4463_last_rssi(void);

/* Largest uplink payload the Si4463 packet handler will deliver. */
#define SI4463_RX_MAX_PAYLOAD   63

// Symbols, etc.
#define STR_TASK_SI4463 "task_si4463: " STR_1TAB


#endif /* __task_si4463_h */
