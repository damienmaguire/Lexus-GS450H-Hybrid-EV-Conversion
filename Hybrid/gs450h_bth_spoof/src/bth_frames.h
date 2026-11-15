/* bth_frames.h -- builds each 16-byte BTH frame that goes to the HV ECU.
 *
 * bth5_buildFrame() is the one function that makes a frame:
 *   1. copy the OEM template frame for this counter index (oem_template.h)
 *   2. set the counter (b0) and its complement (b1)
 *   3. write the current into b2 and b8; in slot 0 also write X13 (b13),
 *      the average current of the previous burst
 *   4. once all 5 modules are live, write the 20 block voltages
 *   5. write aux V, intake air and TB1-4 (temps_aux.h)
 *   6. solve the two-part checksum
 *
 * Two-part checksum (the fix that ended the trips). With S = sum(b0..b14):
 *   b14[3:0] = floor(S / 257)
 *   b15      = S mod 257
 * Every OEM frame we have (4618 car/bench frames) follows this. b14[3:0]
 * itself is part of S, so it is solved: let S0 = S - b14[3:0], then
 * b14[3:0] = S0 >> 8 and b15 = S0 & 0xFF, i.e. b14[3:0]:b15 is the 12-bit
 * sum S0. That answer always exists, is unique, fits the nibble
 * (S0 <= 3300 so q <= 12) and never gives b15 = 256.
 *
 * Function names keep the "bth5_" prefix from development so the logs,
 * tests and history notes still match.
 *
 * Pure C, shared with the host tests.
 */
#ifndef BTH_FRAMES_H
#define BTH_FRAMES_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "bth_link.h"
#include "oem_template.h"
#include "current_sensor.h"
#include "temps_aux.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Block volts -> raw, round to nearest, 25/1023 V per count. */
static inline uint16_t bth5_encodeBlock(float v) {
    long r = lroundf(v * 1023.0f / 25.0f);
    if (r < 0) r = 0;
    if (r > 1023) r = 1023;
    return (uint16_t)r;
}

/* Everything the BTH sender needs for one burst. Core 1 (CAN) fills the
 * block, temperature and diagnostic fields; core 0 adds the current just
 * before building the burst. A tb_raw / intake_raw / aux_v_raw of 0 means
 * "leave the template value". */
typedef struct {
    uint16_t i_raw;            /* 10-bit current, b2/b8 (rounded) */
    uint16_t i_raw13;          /* 10-bit current, slot-0 b13 (OEM: floor of the filtered value) */
    uint16_t raw_block[20];    /* 10-bit, valid when live */
    uint16_t raw_uniform;      /* 10-bit packV/20 */
    uint8_t  live;             /* 0 = template blocks (boot), 1 = module data */
    uint16_t aux_v_raw;        /* aux battery V: slot-3 X13 */
    uint16_t tb_raw[4];        /* TB1..TB4 (slot-4 A5, A11, X13; slot-5 A5); 0 = template */
    uint16_t intake_raw;       /* intake air temp: slot-3 A5 (b5 + b6[1:0]); 0 = template */
    int8_t   mod_tmin, mod_tmax; /* module temps seen (degC, -128 = none); diagnostics only */
    uint8_t  tb_src;            /* BTH5_TB_TEMPLATE / LIVE / STALE */
    /* diagnostics (core 1 -> core 0 -> ring log); not sent on BTH */
    uint16_t cmin_mv, cmax_mv; /* min/max cell over the fresh modules, mV */
    uint8_t  n_vmod;           /* modules with all 16 cells seen */
    uint32_t vnew_ms, vold_ms; /* newest / oldest last cell-frame time over those modules */
    uint32_t blk_chg_ms;       /* core-1 time raw_block[] last changed */
    uint32_t cell_rej, cell_flag;  /* cell-guard counters (core 1) */
} bth5_pub_t;

/* Voltage field number (see bth_link.h) -> block 0..19, or uniform.
 * PLACEHOLDER order: blocks in field order. Which field the ECU treats as
 * which physical block is not confirmed. Any 20 of the 22 fields still add
 * up to within one block of the real pack voltage. */
static const uint8_t BTH5_FIELD_MAP[BTH_N_VFIELDS] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    BTH_FIELD_UNIFORM, BTH_FIELD_UNIFORM
};

/* Older checksum handling, used only by the B14Q_KEEP_LEGACY path:
 * if S mod 257 would be 256 (cannot be stored in b15), nudge the b10 block
 * voltage by -1, +1, -2, +2 ... counts (0.024 V each) until it fits.
 * Returns the nudge applied (in practice 0 or +-1). */
static inline int bth5_fix256(uint8_t *f) {
    static const int tries[] = {0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5, -6, 6,
                                -7, 7, -8, 8, -9, 9, -10, 10, -11, 11, -12, 12};
    /* borrow band (S&255) < (S>>8) is at most 9 sums wide (S <= 2550), so +-12 always escapes */
    uint16_t b10 = bth_get10(f, 10, 12, 2);
    unsigned k;
    for (k = 0; k < sizeof tries / sizeof tries[0]; k++) {
        int v = (int)b10 + tries[k];
        uint32_t s = 0; int i;
        if (v < 0 || v > 1023) continue;
        bth_put10(f, 10, 12, 2, (uint16_t)v);
        for (i = 0; i < 15; i++) s += f[i];
        if (s % 257 != 256
           ) return tries[k];
    }
    return 99; /* unreachable in practice */
}

/* Solve the two-part checksum nibble in place: b14[3:0] = S0 >> 8 where
 * S0 = sum(b0..b13) + (b14 & 0xF0). Returns the nibble. b15 is written by
 * the caller (bth_checksum257). */
static inline uint8_t bth5_solve_b14q(uint8_t *f) {
    uint32_t s0 = 0; int i;
    for (i = 0; i < 14; i++) s0 += f[i];
    s0 += (uint32_t)(f[14] & 0xF0u);
    f[14] = (uint8_t)((f[14] & 0xF0u) | ((s0 >> 8) & 0x0Fu));
    return (uint8_t)(s0 >> 8);
}

/* 1 if b14[3:0] == floor(sum(b0..b14) / 257), i.e. the frame already
 * passes the two-part checksum. */
static inline int bth5_b14q_ok(const uint8_t *f) {
    uint32_t s = 0; int i;
    for (i = 0; i < 15; i++) s += f[i];
    return (uint32_t)(f[14] & 0x0Fu) == s / 257u;
}

/* Solve the checksum nibble on the frame as built. The loop over b10
 * nudges is left over from an option that is off in this build; with it
 * off the first try (nudge 0) is always taken, so b10 is never changed
 * here. Returns the nudge (0). */
static inline int bth5_fixq(uint8_t *f) {
    static const int tries[] = {0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5, -6, 6,
                                -7, 7, -8, 8, -9, 9, -10, 10, -11, 11, -12, 12};
    uint16_t b10 = bth_get10(f, 10, 12, 2);
    unsigned k;
    for (k = 0; k < sizeof tries / sizeof tries[0]; k++) {
        int v = (int)b10 + tries[k];
        if (v < 0 || v > 1023) continue;
        bth_put10(f, 10, 12, 2, (uint16_t)v);
        bth5_solve_b14q(f);
        return tries[k];
    }
    /* unreachable in practice; fall back to the un-nudged frame, still quotient-correct */
    bth_put10(f, 10, 12, 2, b10);
    bth5_solve_b14q(f);
    return 99;
}

/* X13 value packed into slot 0 of the burst that frame idx belongs to.
 * main.c sets p->i_raw13 to the floor of the mean of the 12 current fields
 * (b2/b8) sent in the previous burst, which is what the OEM board does
 * (728/728 OEM frames). */
static inline uint16_t bth5_burst_x13(uint8_t idx, const bth5_pub_t *p) {
    (void)idx;
    return p->i_raw13;                          /* always <= 1023: encode clamps, X13 = mean of 10-bit fields */
}

/* Older b14 low-bit rule (before the two-part checksum was understood),
 * used only by the B14Q_KEEP_LEGACY path. When slot-0 X13 is outside
 * B14_X13_LO..511: slot-0 b14[5:0] = 38, and slots 2 and 5 b14[5:0] - 1.
 * At normal car sums this gives the same nibble as the quotient rule. */
static inline void bth5_legacy_b14(uint8_t *out, uint8_t idx, const bth5_pub_t *p) {
    /* After current packing so slot-0 b14[7:6] already holds X13 high bits.
     * Applied from the sent b2/b8 raw, to every frame of the burst. */
    /* Keyed to the X13 sent in slot 0 of this burst (previous burst's mean),
     * both sides: charge (X13 > 511) and heavy discharge (X13 < B14_X13_LO). */
    const uint16_t x13 = bth5_burst_x13(idx, p);   /* = the X13 packed in this burst's slot 0 */
    if (x13 > 511 || x13 < B14_X13_LO) {
        uint8_t s = (uint8_t)(idx % 6);
        if (s == 0) {
            out[14] = (uint8_t)((out[14] & 0xC0) | (CHARGE_B14_SLOT0_LO & 0x3F));
        } else if (s == 2 || s == 5) {
            uint8_t lo = (uint8_t)(out[14] & 0x3F);
            if (lo > 0) lo--;
            out[14] = (uint8_t)((out[14] & 0xC0) | lo);
        }
    }
}

/* Build frame idx (0..29 = counter 0x20..0x3D) into out[16].
 * Steps are listed at the top of this file.
 * With B14Q_KEEP_LEGACY = 1 (car build) the frame is first built with the
 * older b14 rule; if that already passes the two-part checksum it is sent
 * as is (so frames match the earlier tested builds), otherwise the nibble
 * is solved directly. Either way the frame sent always passes.
 * Returns the b10 nudge used (0 or +-1, for the tests). */
static inline int bth5_buildFrame(uint8_t *out, uint8_t idx, const bth5_pub_t *p) {
    memcpy(out, BTH_H1252[idx], 16);
    out[0] = (uint8_t)(0x20 + idx);
    out[1] = (uint8_t)(0xFF - out[0]);
    bth_put10(out, 2, 6, 6, p->i_raw);
    bth_put10(out, 8, 12, 6, p->i_raw);
    if (idx % 6 == 0) bth_put10(out, 13, 14, 6, bth5_burst_x13(idx, p));
    if (p->live) {
        const int bytes[4] = {3, 4, 9, 10};
        uint16_t v[4];
        int i;
        for (i = 0; i < 4; i++) {
            int fi = bth_voltage_field_index(idx, bytes[i]);
            if (fi < 0) v[i] = p->raw_uniform;   /* -2: 0x3D b9; -1: not written */
            else v[i] = (BTH5_FIELD_MAP[fi] < 20) ? p->raw_block[BTH5_FIELD_MAP[fi]] : p->raw_uniform;
        }
        bth_putVoltages(out, idx, v);
    }
    {
        uint8_t s = (uint8_t)(idx % 6);
        if (s == 3 && p->aux_v_raw) bth_put10(out, 13, 14, 6, p->aux_v_raw);
        if (s == 3 && p->intake_raw) bth_put10(out, 5, 6, 0, p->intake_raw);
        if (s == 4) {
            if (p->tb_raw[0]) bth_put10(out, 5, 6, 0, p->tb_raw[0]);
            if (p->tb_raw[1]) bth_put10(out, 11, 12, 0, p->tb_raw[1]);
            if (p->tb_raw[2]) bth_put10(out, 13, 14, 6, p->tb_raw[2]);
        }
        if (s == 5 && p->tb_raw[3]) bth_put10(out, 5, 6, 0, p->tb_raw[3]);
    }
    {
        /* Checksum: b14[3:0] = S div 257 (always exists, unique), b15 = S mod 257.
         * B14Q_KEEP_LEGACY: first build the frame with the older rule (legacy b14
         * + fix256). If that frame already satisfies the quotient rule it is sent
         * as is, so frames the older build got right stay byte-identical
         * (including its b10 borrow-nudge choice). Otherwise solve q on the
         * un-nudged frame (bth5_fixq). */
        int n;
#if B14Q_KEEP_LEGACY
        {
            uint8_t leg[16];
            memcpy(leg, out, 16);
            bth5_legacy_b14(leg, idx, p);
            n = bth5_fix256(leg);
            if (bth5_b14q_ok(leg)) {
                memcpy(out, leg, 15);
                out[15] = bth_checksum257(out);
                return n;
            }
        }
#endif
        n = bth5_fixq(out);
        out[15] = bth_checksum257(out);
        return n;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BTH_FRAMES_H */
