/******************************************************************************
KENSAT - Si4463 Radio Transceiver RX Task

Si4463 transceiver control via SPI2.

******************************************************************************/
#ifndef __task_si4463_h
#define __task_si4463_h


// Si4463 operating modes for si4463_set_mode()
#define SI4463_MODE_RX      0
#define SI4463_MODE_TX      1
#define SI4463_MODE_READY   2

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

/* Half-duplex uplink: si4463_rx_enable() at the start of a listening window,
   si4463_rx_poll() periodically, si4463_rx_disable() before transmitting.
   None of these yield. si4463_rx_poll() busy-waits up to ~1.3 s while a
   frame is arriving. */
extern unsigned char si4463_rx_enable(void);
extern void          si4463_rx_disable(void);
extern unsigned int  si4463_rx_poll(unsigned char *buf, unsigned int maxlen);
extern unsigned char si4463_last_rssi(void);

/* Max uplink payload. PROMPT needs 15 B "KO6IKZ KENSAT1 " + 7 B "PROMPT "
   + 256 B prompt = 278 B, plus headroom for a longer UPLINK_AUTH token.
   This exceeds the 64 B RX FIFO, so uplink frames use a 2-byte length and
   si4463_rx_poll() drains the FIFO while the frame arrives. */
#define SI4463_RX_MAX_PAYLOAD   300

// Symbols, etc.
#define STR_TASK_SI4463 "task_si4463: " STR_1TAB


#endif /* __task_si4463_h */
