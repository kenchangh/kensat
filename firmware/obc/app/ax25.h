/******************************************************************************
KENSAT - AX.25 UI frame encoder (G3RUH 9k6 GFSK over Si4463)

Builds the Si4463 TX FIFO bit stream for one AX.25 UI frame:

  [0x7E x N] [addr][ctrl=0x03][pid=0xF0][info][FCS-16] [0x7E x M]

bit-stuffed, NRZI, G3RUH scrambled (x^17 + x^12 + 1), packed MSB-first,
padded to a whole byte with flag bits.

Worst case is ~285 bytes for a 200-byte info field; callers use 512.

******************************************************************************/
#ifndef __ax25_h
#define __ax25_h

/* Default callsigns */
#define AX25_DST_CALL   "BEACON"
#define AX25_DST_SSID   0
#define AX25_SRC_CALL   "KO6IKZ"
#define AX25_SRC_SSID   0

/* Preamble and tail, in flag bytes.  16 leading flags = ~107 ms at
   1200 bps.  Don't reduce it: gr-satellites (what SatNOGS runs) needs the
   preamble to lock symbol sync, even though pluto_rx_ax25.py decodes fine
   with less.  Measured with satnogs_fsk_ax25, clean synthetic frames:

       leading flags :  4     8    10    12    16    24    32
       frames decoded: 3/6   4/6   6/6   6/6   6/6   6/6   6/6

   Keep AX25_TX_TRAILING_FLAGS >= 2.
   See ground-station/test_ax25_preamble.py. */
#define AX25_TX_LEADING_FLAGS   16
#define AX25_TX_TRAILING_FLAGS  4

/* Build fails if AX25_TX_LEADING_FLAGS < 16.  typedef instead of
   _Static_assert for XC16. */
typedef char ax25_leading_flags_floor_check[
    (AX25_TX_LEADING_FLAGS >= 16) ? 1 : -1];

/* Maximum AX.25 info field this encoder accepts (bytes). */
#define AX25_MAX_INFO           200


/******************************************************************************
 ax25_build_ui()

 Build a UI frame from src_call-src_ssid to dst_call-dst_ssid into
 out_bytes.  Returns the byte length, or 0 if info is too long or out_bytes
 too small.  Callsigns are space-padded/truncated to 6 chars.
******************************************************************************/
extern unsigned int ax25_build_ui(
    const char         *dst_call,
    unsigned char       dst_ssid,
    const char         *src_call,
    unsigned char       src_ssid,
    const unsigned char *info,
    unsigned int        info_len,
    unsigned char      *out_bytes,
    unsigned int        out_cap);


#endif /* __ax25_h */
