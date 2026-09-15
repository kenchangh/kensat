/******************************************************************************
KENSAT - AX.25 UI frame encoder (G3RUH 9k6 GFSK)

See ax25.h for the high-level pipeline. This file implements:

  1. Address field: callsign chars shifted left by 1, SSID byte
     0x60 | (ssid << 1) with the last-address bit set on the src SSID.
  2. CRC-16-CCITT FCS, reflected (poly 0x8408, init 0xFFFF, final XOR
     0xFFFF). Appended low-byte first, high-byte second.
  3. Bit stuffing: a 0 bit is inserted after every five consecutive 1s
     in the (addr+ctrl+pid+info+fcs) bit stream. Bits are processed
     LSB-first per byte. Flags are never stuffed.
  4. NRZI: 0 = transition, 1 = no transition. Initial line state = 1.
  5. G3RUH scrambler: out = in XOR sr[12] XOR sr[17] with sr shifting
     in the scrambled output. Self-synchronizing — receiver locks
     within 17 bits once data starts.
  6. Pack MSB-first into output bytes (bit 7 of byte 0 is the first
     bit on the air, matching the Si4463's MSB-first FIFO shift-out).
     The final byte is padded out to a byte boundary with additional
     scrambled flag bits.

******************************************************************************/
#include "ax25.h"

#include <string.h>


/******************************************************************************
 Bit-stream packer state
******************************************************************************/
typedef struct {
    unsigned char *out;
    unsigned int   cap;
    unsigned int   byte_idx;
    unsigned char  bit_idx;     /* 0..7, MSB-first within byte */
    unsigned char  nrzi_last;   /* current NRZI line level (0/1) */
    unsigned long  sr;          /* 17-bit G3RUH scrambler shift reg */
    unsigned char  overflow;
} ax25_bs_t;


/******************************************************************************
 fill_call()

 Encode one AX.25 address subfield (6 chars + SSID byte) into p.
 is_last sets the HDLC extension bit (0 = more addresses follow,
 1 = end of address chain).
******************************************************************************/
static unsigned int fill_call(unsigned char *p,
                              const char *call,
                              unsigned char ssid,
                              unsigned char is_last)
{
    unsigned char i;

    for (i = 0; i < 6; i++) {
        char c = call[i];
        if (c == 0) break;
        p[i] = (unsigned char)(((unsigned char)c) << 1);
    }
    for (; i < 6; i++) {
        p[i] = (unsigned char)(' ' << 1);
    }
    /* SSID byte: 0b CRR_SSID_X. C=0 (response), RR=11 (reserved),
       X=1 only on the final address byte. */
    p[6] = (unsigned char)(0x60 | ((ssid & 0x0F) << 1) | (is_last ? 0x01 : 0x00));
    return 7;
}


/******************************************************************************
 ax25_fcs()

 CRC-16-CCITT (reflected) over the given bytes. Standard HDLC FCS:
 init 0xFFFF, process each byte LSB-first with reflected poly 0x8408,
 final XOR 0xFFFF. Returned value is appended low-byte-first to the
 frame before bit stuffing.
******************************************************************************/
static unsigned int ax25_fcs(const unsigned char *buf, unsigned int len)
{
    unsigned int  crc = 0xFFFF;
    unsigned int  i;
    unsigned char j;

    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x8408;
            else         crc >>= 1;
        }
    }
    return crc ^ 0xFFFF;
}


/******************************************************************************
 bs_emit_bit()

 Push one logical bit through the NRZI encoder + G3RUH scrambler +
 byte packer. This is the single chokepoint for the bit stream.
******************************************************************************/
static void bs_emit_bit(ax25_bs_t *s, unsigned char in_bit)
{
    unsigned char nrzi_bit;
    unsigned char scrambled;

    /* NRZI: a 0 forces a level transition, a 1 holds the level. */
    if (!in_bit) s->nrzi_last ^= 1;
    nrzi_bit = s->nrzi_last;

    /* G3RUH self-synchronizing scrambler: x^17 + x^12 + 1.
       The shift register holds the last 17 scrambled output bits
       (LSB = most recent). */
    scrambled = (unsigned char)(
        nrzi_bit ^
        (unsigned char)((s->sr >> 11) & 1) ^
        (unsigned char)((s->sr >> 16) & 1));
    s->sr = ((s->sr << 1) | scrambled) & 0x1FFFFUL;

    /* Pack MSB-first into the output buffer. Si4463 hardware transmits
       FIFO bytes MSB-first (bit 7 first), so to make the on-air bit
       order match the order we emit bits, the first bit of a byte goes
       into bit 7. This is the same convention pluto_rx.py assumes for
       the legacy packet format — confirmed empirically: with LSB-first
       packing the on-air bit stream was byte-reversed relative to the
       AX.25 spec, and Direwolf / our Pluto decoder both failed. */
    if (s->byte_idx >= s->cap) { s->overflow = 1; return; }
    if (s->bit_idx == 0) s->out[s->byte_idx] = 0;
    s->out[s->byte_idx] |= (unsigned char)(scrambled << (7 - s->bit_idx));
    s->bit_idx++;
    if (s->bit_idx == 8) {
        s->bit_idx = 0;
        s->byte_idx++;
    }
}


/******************************************************************************
 bs_emit_flag()

 Emit one 0x7E flag, LSB-first on the wire, with no bit stuffing. Used for the
 leading preamble flags, the closing flag(s), and any byte-boundary
 padding at the very end of the bit stream.
******************************************************************************/
static void bs_emit_flag(ax25_bs_t *s)
{
    unsigned char b;
    for (b = 0; b < 8; b++) {
        bs_emit_bit(s, (unsigned char)((0x7E >> b) & 1));
    }
}


/******************************************************************************
 bs_emit_byte_stuffed()

 Emit one data byte LSB-first, applying HDLC bit stuffing: after every
 five consecutive 1 bits seen so far, force a 0 into the stream and
 reset the run counter. *ones is the running 1-count carried across
 byte boundaries; reset it externally before/after the stuffed region.
******************************************************************************/
static void bs_emit_byte_stuffed(ax25_bs_t *s,
                                 unsigned char byte,
                                 unsigned char *ones)
{
    unsigned char b;
    unsigned char bit;

    for (b = 0; b < 8; b++) {
        bit = (unsigned char)((byte >> b) & 1);
        bs_emit_bit(s, bit);
        if (bit) {
            (*ones)++;
            if (*ones == 5) {
                bs_emit_bit(s, 0);
                *ones = 0;
            }
        } else {
            *ones = 0;
        }
    }
}


/******************************************************************************
 ax25_build_ui()  -- public API
******************************************************************************/
unsigned int ax25_build_ui(
    const char         *dst_call,
    unsigned char       dst_ssid,
    const char         *src_call,
    unsigned char       src_ssid,
    const unsigned char *info,
    unsigned int        info_len,
    unsigned char      *out_bytes,
    unsigned int        out_cap)
{
    unsigned char frame[14 + 2 + AX25_MAX_INFO + 2];   /* addr+ctl/pid+info+fcs */
    unsigned int  flen = 0;
    unsigned int  fcs;
    unsigned int  i;
    unsigned char ones = 0;
    ax25_bs_t     st;

    if (info_len > AX25_MAX_INFO) return 0;
    if (out_cap < 64)             return 0;   /* sanity */

    /* ---- Build the un-stuffed frame bytes ---- */

    /* Destination + source addresses (14 bytes total). The HDLC
       end-of-address bit is set on the src SSID byte only. */
    flen += fill_call(frame + flen, dst_call, dst_ssid, 0);
    flen += fill_call(frame + flen, src_call, src_ssid, 1);

    /* UI control + no L3 PID */
    frame[flen++] = 0x03;
    frame[flen++] = 0xF0;

    /* Info */
    for (i = 0; i < info_len; i++) {
        frame[flen++] = info[i];
    }

    /* FCS — computed over addr+ctrl+pid+info, appended low byte first. */
    fcs = ax25_fcs(frame, flen);
    frame[flen++] = (unsigned char)(fcs & 0xFF);
    frame[flen++] = (unsigned char)((fcs >> 8) & 0xFF);

    /* ---- Serialize ---- */

    st.out       = out_bytes;
    st.cap       = out_cap;
    st.byte_idx  = 0;
    st.bit_idx   = 0;
    st.nrzi_last = 1;
    st.sr        = 0;
    st.overflow  = 0;
    memset(out_bytes, 0, out_cap);

    /* Leading flags (preamble): no stuffing, but they DO go through
       the NRZI + scrambler so the receiver's descrambler self-syncs
       on them. */
    for (i = 0; i < AX25_TX_LEADING_FLAGS; i++) bs_emit_flag(&st);

    /* Frame bytes, bit-stuffed. */
    for (i = 0; i < flen; i++) bs_emit_byte_stuffed(&st, frame[i], &ones);

    /* Trailing flags. */
    for (i = 0; i < AX25_TX_TRAILING_FLAGS; i++) bs_emit_flag(&st);

    /* Pad final byte with additional flag bits so the FIFO sees only
       whole bytes. Keep going until we're byte-aligned again. */
    {
        unsigned char fb = 0;
        while (st.bit_idx != 0) {
            bs_emit_bit(&st, (unsigned char)((0x7E >> (fb & 7)) & 1));
            fb++;
        }
    }

    if (st.overflow) return 0;
    return st.byte_idx;
}
