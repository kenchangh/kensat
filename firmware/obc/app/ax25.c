/******************************************************************************
KENSAT - AX.25 UI frame encoder (G3RUH 9k6 GFSK)

See ax25.h for the frame format.  Bits go out LSB-first per byte, then
bit stuffing (data only), NRZI (initial level 1), G3RUH scrambling, and
MSB-first packing for the Si4463 FIFO.

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

 Encode one address subfield (6 chars + SSID byte) into p.  is_last sets
 the end-of-address bit.  Returns 7.
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
    /* SSID byte CRRSSSSX: C=0, RR=11, X=1 on the last address. */
    p[6] = (unsigned char)(0x60 | ((ssid & 0x0F) << 1) | (is_last ? 0x01 : 0x00));
    return 7;
}


/******************************************************************************
 ax25_fcs()

 HDLC FCS: CRC-16-CCITT reflected, poly 0x8408, init/final 0xFFFF.
 Sent low byte first.
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

 NRZI-encode, scramble and pack one bit.
******************************************************************************/
static void bs_emit_bit(ax25_bs_t *s, unsigned char in_bit)
{
    unsigned char nrzi_bit;
    unsigned char scrambled;

    /* NRZI: 0 = transition, 1 = hold */
    if (!in_bit) s->nrzi_last ^= 1;
    nrzi_bit = s->nrzi_last;

    /* G3RUH scrambler x^17 + x^12 + 1; sr holds the last 17 output bits,
       LSB = newest. */
    scrambled = (unsigned char)(
        nrzi_bit ^
        (unsigned char)((s->sr >> 11) & 1) ^
        (unsigned char)((s->sr >> 16) & 1));
    s->sr = ((s->sr << 1) | scrambled) & 0x1FFFFUL;

    /* MSB-first: the Si4463 shifts FIFO bytes out bit 7 first.  LSB-first
       packing was tried and Direwolf could not decode it. */
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

 Emit one 0x7E flag, unstuffed.
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

 Emit one data byte LSB-first with bit stuffing (0 after five 1s).
 *ones carries the run of 1s across bytes.
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
 ax25_build_ui()
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
    if (out_cap < 64)             return 0;

    /* ---- Build the un-stuffed frame ---- */
    flen += fill_call(frame + flen, dst_call, dst_ssid, 0);
    flen += fill_call(frame + flen, src_call, src_ssid, 1);

    /* UI control, no layer 3 */
    frame[flen++] = 0x03;
    frame[flen++] = 0xF0;

    for (i = 0; i < info_len; i++) {
        frame[flen++] = info[i];
    }

    /* FCS, low byte first */
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

    /* Preamble flags are scrambled too, so the receiver syncs on them. */
    for (i = 0; i < AX25_TX_LEADING_FLAGS; i++) bs_emit_flag(&st);

    for (i = 0; i < flen; i++) bs_emit_byte_stuffed(&st, frame[i], &ones);

    for (i = 0; i < AX25_TX_TRAILING_FLAGS; i++) bs_emit_flag(&st);

    /* Pad to a whole byte with more flag bits. */
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
