/******************************************************************************
KENSAT - AX.25 UI frame encoder (G3RUH 9k6 GFSK over Si4463)

Builds a ready-to-FIFO bit stream for an AX.25 UI frame, fully formatted
for G3RUH 9600 bps reception by Direwolf / SatNOGS:

  [leading 0x7E flags] [addr][ctrl=0x03][pid=0xF0][info][FCS-16]
  [trailing 0x7E flags]

  - bit-stuffed (insert 0 after five consecutive 1s in the data portion)
  - NRZI line-coded (0 = transition, 1 = no transition)
  - G3RUH scrambled (LFSR x^17 + x^12 + 1, self-synchronizing)
  - packed MSB-first into bytes for the Si4463 TX FIFO (the chip shifts
    FIFO bytes out bit 7 first, so the first bit we emit must land in
    bit 7 for the on-air order to match the AX.25 spec)

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
   16 leading flags = 128 bits = ~107 ms at 1200 bps — gives the receiver's
   AGC, BCR, and scrambler enough lead-in to lock before the address field
   starts. 4 trailing flags lets the receiver close the frame cleanly.

   DO NOT REDUCE THIS. It was cut to 8 during the 1200 bps changeover to save
   airtime (8 flag bytes = 53 ms once each flag costs 6.67 ms instead of
   0.83 ms) on the reasoning that the G3RUH descrambler self-syncs in ~17 bits
   and the BCR needs only a few tens more. That reasoning is wrong in
   practice, and the way it is wrong is instructive: it holds for a decoder
   that brute-forces every symbol-timing phase, which is exactly what
   ground-station/pluto_rx_ax25.py does, so the project's own reference decoder
   showed no difference at all between 8 and 16. gr-satellites -- what actual
   SatNOGS observers run -- instead has to LOCK a symbol-sync loop, and it
   needs a real preamble to do it.

   Measured through the real satnogs_fsk_ax25 flowgraph on clean synthetic
   frames at 48 kSps, zero carrier offset (so this is a best case, not a
   marginal-SNR effect):

       leading flags :  4     8    10    12    16    24    32
       frames decoded: 3/6   4/6   6/6   6/6   6/6   6/6   6/6

   At 8 flags a third of all frames are lost. The 53 ms saved is ~10% of a
   550 ms beacon, so trading it for a 33% frame loss is strictly negative:
   you would have to transmit ~1.5x as often to land the same number of
   decodes. The cliff sits between 8 and 10, and 16 keeps real margin above
   it -- worth having, since the synthetic test above is noise-free and a
   real pass will not be.

   Do not take AX25_TX_TRAILING_FLAGS below 2.

   See also ground-station/test_ax25_preamble.py, which re-runs the table
   above against the reference decoder without any hardware. */
#define AX25_TX_LEADING_FLAGS   16
#define AX25_TX_TRAILING_FLAGS  4

/* Compile-time floor on the preamble. Fails the build (negative array
   bound) if anyone trims AX25_TX_LEADING_FLAGS below 16 -- deliberately
   loud, because the cost of a trim is silent everywhere else: our own
   decoder will not notice it, only third-party stations will. Written as a
   typedef rather than _Static_assert so it holds under XC16's C89/C99 front
   end. Raising the value above 16 is allowed; it only costs airtime. */
typedef char ax25_leading_flags_floor_check[
    (AX25_TX_LEADING_FLAGS >= 16) ? 1 : -1];

/* Maximum AX.25 info field this encoder accepts (bytes). */
#define AX25_MAX_INFO           200


/******************************************************************************
 ax25_build_ui()

 Build an AX.25 UI frame addressed from src_call (with src_ssid) to
 dst_call (with dst_ssid), with the supplied info field. Renders the
 entire bit stream (preamble flags + stuffed/NRZI/G3RUH-scrambled frame
 + tail flags), packs MSB-first into out_bytes, and returns the byte
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
