/* bth_link.h -- low-level BTH frame layout helpers.
 *
 * The BTH link carries 16-byte frames, 6 per burst, from the battery board
 * to the HV ECU. Frames count 0x20..0x3D (30 frames, 5 bursts) and repeat.
 * "Slot" = position of a frame inside its burst (counter index % 6).
 *
 * Every value field is 10 bits: the low 8 bits sit in a value byte and the
 * top 2 bits are packed into a shared "high-bits" byte (b6, b12 or b14):
 *   b2  current     hi = b6[7:6]      b8  current     hi = b12[7:6]
 *   b3  block V     hi = b6[5:4]      b9  block V *   hi = b12[5:4]
 *   b4  block V     hi = b6[3:2]      b10 block V     hi = b12[3:2]
 *   b5  temp / aux  hi = b6[1:0]      b11 temp / aux  hi = b12[1:0]
 *   b13 ("X13")     hi = b14[7:6]
 *   * b9 is not a voltage in slots 2 and 5, except in frame 0x3D.
 * Fixed bytes: b1 = 0xFF - b0, b7 = 0.
 *
 * Block voltage = raw * 25/1023 V (no offset).
 *
 * Pure C, no Pico SDK, so the host tests use the same code as the firmware.
 */
#ifndef BTH_LINK_H
#define BTH_LINK_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* b15 = (sum of b0..b14) mod 257. The full two-part checksum (b14 low
 * nibble + b15) is solved in bth_frames.h; once that is done this mod-257
 * value always fits in a byte. */
static inline uint8_t bth_checksum257(const uint8_t *f) {
    uint32_t s = 0;
    for (int i = 0; i < 15; i++) s += f[i];
    return (uint8_t)(s % 257);
}

#define BTH_RAW10_MAX       1023u
#define BTH_V_PER_COUNT     (25.0f / 1023.0f)   /* block volts per count */

/* Round a float to the nearest 10-bit count, clamped to 0..1023. */
static inline uint16_t bth_clamp10(float r) {
    if (r < 0.0f) r = 0.0f;
    if (r > 1023.0f) r = 1023.0f;
    return (uint16_t)(r + 0.5f);
}

/* One block voltage (V) -> 10-bit raw. */
static inline uint16_t bth_encodeBlock(float block_v) {
    return bth_clamp10(block_v / BTH_V_PER_COUNT);
}

/* Whole-pack voltage -> raw for one of 20 equal blocks. Used for the
 * "uniform" value (pack V / 20). */
static inline uint16_t bth_encodeVoltage(float pack_v) {
    return bth_encodeBlock(pack_v / 20.0f);
}

/* 10-bit raw -> block volts. */
static inline float bth_decodeBlock(uint16_t raw10) { return raw10 * BTH_V_PER_COUNT; }

/* Write a 10-bit value: low byte to f[lo], top 2 bits to bits
 * [shift+1:shift] of f[hi]. All other bits of f[hi] are kept. */
static inline void bth_put10(uint8_t *f, int lo, int hi, int shift, uint16_t v) {
    v &= 0x3FF;
    f[lo] = (uint8_t)(v & 0xFF);
    f[hi] = (uint8_t)((f[hi] & ~(3u << shift)) | (((unsigned)(v >> 8) & 3u) << shift));
}

/* Read a 10-bit value back (inverse of bth_put10). */
static inline uint16_t bth_get10(const uint8_t *f, int lo, int hi, int shift) {
    return (uint16_t)(f[lo] | (((f[hi] >> shift) & 3u) << 8));
}

/* b9 carries a block voltage except in slots 2 and 5. Frame 0x3D
 * (index 29) is the exception: it has a voltage in b9 too. */
static inline int bth_byte9IsVoltage(uint8_t idx) {
    uint8_t slot = (uint8_t)(idx % 6);
    return (slot != 2 && slot != 5) || (idx == 29);
}

/* Write the 4 block-voltage fields of a frame: v[0..3] -> b3, b4, b9, b10.
 * b9 is only written where it is a voltage. */
static inline void bth_putVoltages(uint8_t *f, uint8_t idx, const uint16_t v[4]) {
    bth_put10(f, 3, 6, 4, v[0]);
    bth_put10(f, 4, 6, 2, v[1]);
    if (bth_byte9IsVoltage(idx)) bth_put10(f, 9, 12, 4, v[2]);
    bth_put10(f, 10, 12, 2, v[3]);
}

/* A burst has 22 block-voltage fields, numbered in this order:
 *   slot0: b3,b4,b9,b10  (0-3)      slot3: b3,b4,b9,b10  (11-14)
 *   slot1: b3,b4,b9,b10  (4-7)      slot4: b3,b4,b9,b10  (15-18)
 *   slot2: b3,b4,b10     (8-10)     slot5: b3,b4,b10     (19-21)
 * Frame 0x3D's extra b9 voltage gets the uniform value, not a 23rd field.
 * BTH5_FIELD_MAP (bth_frames.h) maps field number -> block 0..19, or
 * BTH_FIELD_UNIFORM for "pack V / 20". */
#define BTH_N_VFIELDS 22
#define BTH_FIELD_UNIFORM 0xFF

/* Field number (0..21) for value byte byte_no (3, 4, 9 or 10) of frame idx.
 * Returns -1 if that byte is not a voltage here, -2 for frame 0x3D's b9. */
static inline int bth_voltage_field_index(uint8_t idx, int byte_no) {
    static const int start[6] = {0, 4, 8, 11, 15, 19};
    uint8_t slot = (uint8_t)(idx % 6);
    int base = start[slot];
    int has9 = bth_byte9IsVoltage(idx);
    if (byte_no == 3) return base;
    if (byte_no == 4) return base + 1;
    if (byte_no == 9) {
        if (!has9) return -1;
        if (idx == 29) return -2; /* extra / uniform */
        return base + 2;
    }
    if (byte_no == 10) return base + ((has9 && idx != 29) ? 3 : 2);
    return -1;
}

#ifdef __cplusplus
}
#endif

#endif /* BTH_LINK_H */
