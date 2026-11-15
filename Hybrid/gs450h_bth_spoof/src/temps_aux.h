/* temps_aux.h -- the slot-3/4/5 "aux" fields: battery temperatures TB1-4,
 * battery intake air temperature, and the 12 V aux battery voltage.
 *
 * Field map (10-bit fields, see bth_link.h for the byte layout):
 *   TB1     slot 4 A5  (b5  + b6[1:0])      template raw 666
 *   TB2     slot 4 A11 (b11 + b12[1:0])     template raw 673
 *   TB3     slot 4 X13 (b13 + b14[7:6])     template raw 668
 *   TB4     slot 5 A5  (b5  + b6[1:0])      template raw 650
 *   intake  slot 3 A5  (b5  + b6[1:0])      template raw 605
 *   aux V   slot 3 X13 (b13 + b14[7:6])     fixed 12.6 V = raw 638
 *
 * Temperatures come from the BMW module thermistors (byte - 40 C, see
 * bmw_pack.c). Targets: TB1 = TB3 = average, TB2 = coldest, TB4 = warmest
 * (TB2/TB4 limited to +-5 C from the average), intake = TB2. Sent values
 * are clamped to 5..40 C and slewed at 0.05 C/s. With no valid readings
 * for 5 s they slew back to the template values.
 *
 * Temperature law (NTC beta, fitted to 5 X431 readings, max error 0.008 C):
 *   ln(raw / (1023 - raw)) = -10.389791 + 3297.8588 / T_kelvin
 * Higher raw = colder. The high bits change from 2 to 3 at raw 768 (13.81 C).
 *
 * Pure C, shared with the host tests.
 */
#ifndef TEMPS_AUX_H
#define TEMPS_AUX_H

#include <stdint.h>
#include <math.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Aux battery volts -> raw (AUX_V_PER_COUNT = 13.3/673 V per count),
 * clamped to 1..1022 so it is never 0 ("leave template") or 1023. */
static inline uint16_t bth5_encodeAuxV(float v) {
    long r = lroundf(v / AUX_V_PER_COUNT);
    if (r < 1) r = 1;
    if (r > 1022) r = 1022;
    return (uint16_t)r;
}

/* Raw -> degrees C with the beta law:
 *   T = BETA / (ln(raw / (N - raw)) - LNK) - 273.15 */
static inline float bth5_decodeTemp(uint16_t raw) {
    float r = (float)raw;
    return TEMP_BETA_K / (logf(r / (TEMP_ADC_N - r)) - TEMP_BETA_LNK) - 273.15f;
}

/* Degrees C -> raw. Clamps to TEMP_SEND_MIN_C..MAX_C (5..40 C) first, then
 *   x = exp(LNK + BETA / (T + 273.15)),  raw = N * x / (1 + x),
 * rounded and limited to 1..1022. */
static inline uint16_t bth5_encodeTemp(float c) {
    long r;
    if (c < TEMP_SEND_MIN_C) c = TEMP_SEND_MIN_C;
    if (c > TEMP_SEND_MAX_C) c = TEMP_SEND_MAX_C;
    {   float x = expf(TEMP_BETA_LNK + TEMP_BETA_K / (c + 273.15f));
        r = lroundf(TEMP_ADC_N * x / (1.0f + x)); }
    if (r < 1) r = 1;
    if (r > 1022) r = 1022;
    return (uint16_t)r;
}

/* Where the sent temperatures came from (bth5_tb_t.src, logged):
 * TEMPLATE = never had a valid reading, LIVE = from the modules,
 * STALE = lost the modules, slewing back to / holding the template.
 * Channels: 0..3 = TB1..TB4, 4 = intake air. */
#define BTH5_TB_TEMPLATE  0
#define BTH5_TB_LIVE      1
#define BTH5_TB_STALE     2         /* slewing back to / holding template */
#define BTH5_TB_NCH 5
/* Template raws per channel (TB1, TB2, TB3, TB4, intake), = oem_template.h. */
static const uint16_t BTH5_TB_TPL_RAW[5] = { 666, 673, 668, 650, 605 };
/* Filter state. t[] = temperatures currently sent (C). init = had a valid
 * reading. last_ms = last step, good_ms = last valid reading. */
typedef struct {
    float t[BTH5_TB_NCH]; /* degC currently sent */
    uint8_t init, src;
    uint32_t last_ms, good_ms;
} bth5_tb_t;
/* Template temperature (C) of channel i. */
static inline float bth5_tb_tpl_c(int i) { return bth5_decodeTemp(BTH5_TB_TPL_RAW[i]); }

/* A module reading is usable if at least one thermistor was used, all
 * values are within TB_VALID_MIN_C..MAX_C (-30..53 C, which rejects the
 * -40 "missing" and 55 C "unused" codes) and min <= avg <= max. */
static inline int bth5_tb_valid(int n_used, float tavg, float tmin, float tmax) {
    return n_used > 0 && tmin >= TB_VALID_MIN_C && tmax <= TB_VALID_MAX_C && tavg >= tmin && tavg <= tmax;
}

/* One filter step (core 1, every 10 ms poll). Inputs are the module temps
 * from bmw_pack.c. out[i] = raw to send for channel i, or 0 = leave the
 * template byte untouched.
 *   - valid reading: targets as above, clamped to 5..40 C. The first one
 *     is taken directly; after that each channel moves at most
 *     TB_SLEW_C_PER_S (dt capped at 1 s).
 *   - no valid reading for < TB_STALE_MS: hold the current values.
 *   - longer: slew towards the template temperatures; once every channel
 *     encodes to its exact template raw, send the template (out = 0). */
static inline void bth5_tb_step(bth5_tb_t *s, uint16_t out[BTH5_TB_NCH], int n_used, float tavg, float tmin, float tmax, uint32_t now) {
    int i, ok = bth5_tb_valid(n_used, tavg, tmin, tmax);
    float tgt[BTH5_TB_NCH], dt = s->init ? (float)(uint32_t)(now - s->last_ms) * 0.001f : 0.0f, step;
    if (dt > 1.0f) dt = 1.0f;
    if (dt < 0.0f) dt = 0.0f;
    step = TB_SLEW_C_PER_S * dt;
    s->last_ms = now;
    if (ok) {
        float lo = tavg - TB_SPREAD_MAX_C, hi = tavg + TB_SPREAD_MAX_C;
        tgt[0] = tavg; tgt[1] = tmin < lo ? lo : tmin; tgt[2] = tavg; tgt[3] = tmax > hi ? hi : tmax;
        tgt[4] = tgt[1];   /* intake = spread-limited module min (= TB2) */
        for (i = 0; i < BTH5_TB_NCH; i++) {
            if (tgt[i] < TEMP_SEND_MIN_C) tgt[i] = TEMP_SEND_MIN_C;
            if (tgt[i] > TEMP_SEND_MAX_C) tgt[i] = TEMP_SEND_MAX_C;
        }
        s->good_ms = now;
        if (!s->init) { for (i = 0; i < BTH5_TB_NCH; i++) s->t[i] = tgt[i]; s->init = 1; s->src = BTH5_TB_LIVE; }
        else s->src = BTH5_TB_LIVE;
    } else {
        if (!s->init) { for (i = 0; i < BTH5_TB_NCH; i++) out[i] = 0; s->src = BTH5_TB_TEMPLATE; return; }
        if ((uint32_t)(now - s->good_ms) < TB_STALE_MS) {            /* brief gap: hold */
            for (i = 0; i < BTH5_TB_NCH; i++) out[i] = bth5_encodeTemp(s->t[i]);
            return;
        }
        for (i = 0; i < BTH5_TB_NCH; i++) tgt[i] = bth5_tb_tpl_c(i);
        s->src = BTH5_TB_STALE;
    }
    for (i = 0; i < BTH5_TB_NCH; i++) {
        float d = tgt[i] - s->t[i];
        if (d > step) d = step;
        if (d < -step) d = -step;
        s->t[i] += d;
        out[i] = bth5_encodeTemp(s->t[i]);
    }
    if (s->src == BTH5_TB_STALE) {   /* converged: send the exact template raws */
        int done = 1;
        for (i = 0; i < BTH5_TB_NCH; i++) if (out[i] != BTH5_TB_TPL_RAW[i]) done = 0;
        if (done) for (i = 0; i < BTH5_TB_NCH; i++) out[i] = 0;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* TEMPS_AUX_H */
