/******************************************************************************
KENSAT - AX.25 UI frame encoder (G3RUH 9k6 GFSK over Si4463)

Builds a ready-to-FIFO bit stream for an AX.25 UI frame, fully formatted
for G3RUH 9600 bps reception by Direwolf / SatNOGS:

  [leading 0x7E flags] [addr][ctrl=0x03][pid=0xF0][info][FCS-16]
  [trailing 0x7E flags]

  - bit-stuffed (insert 0 after five consecutive 1s in the data portion)
  - NRZI line-coded (0 = transition, 1 = no transition)
  - G3RUH scrambled (LFSR x^17 + x^12 + 1, self-synchronizing)
  - packed LSB-first into bytes for the Si4463 TX FIFO

Final byte is padded with extra scrambled flag bits — never half-byte
truncated.

Output buffer should be sized for worst case: ~200 B info + 20 B
overhead → ~220 B pre-stuff → ~265 B post-stuff → ~285 B incl flags.
Caller passes a buffer of 512 B to leave headroom.

******************************************************************************/
#ifndef __ax25_h
#define __ax25_h

/* Default callsigns. Override per call if needed. */
#define AX25_DST_CALL   "BEACON"
#define AX25_DST_SSID   0
#define AX25_SRC_CALL   "KO6IKZ"
#define AX25_SRC_SSID   0

/* TX preamble / tail length (in HDLC flag bytes).
   16 leading flags = 128 bits = ~13 ms at 9600 — gives the receiver's
   AGC, BCR, and scrambler enough lead-in to lock before the address
   field starts. 4 trailing flags lets the receiver close the frame
   cleanly. */
#define AX25_TX_LEADING_FLAGS   16
#define AX25_TX_TRAILING_FLAGS  4

/* Maximum AX.25 info field this encoder accepts (bytes). */
#define AX25_MAX_INFO           200


/******************************************************************************
 ax25_build_ui()

 Build an AX.25 UI frame addressed from src_call (with src_ssid) to
 dst_call (with dst_ssid), with the supplied info field. Renders the
 entire bit stream (preamble flags + stuffed/NRZI/G3RUH-scrambled frame
 + tail flags), packs LSB-first into out_bytes, and returns the byte
 length written.

 Returns 0 on error (info too large, output buffer too small, or any
 internal overflow).

 Callsigns are space-padded to 6 characters; passing a string longer
 than 6 chars truncates silently.
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
