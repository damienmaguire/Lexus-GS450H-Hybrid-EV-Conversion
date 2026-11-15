/* current_sensor.h -- pack current: Hall sensor reading, BTH encoding,
 * deadband policy and the engine-start detector.
 *
 * Sign convention everywhere in this firmware: amps are DISCHARGE-positive.
 *
 * Flow, once per burst (main.c sample_current()):
 *   ADC mean -> bth5_isense_step() -> amps
 *   amps -> bth5_encodeCurrent() (b2/b8) and bth5_encodeCurrentFloor()
 *   bth5_eng_step()        tracks PRE -> CRANK -> SETTLE -> RUN
 *   bth5_applyCurrentPolicy() forces 0x1FF inside the deadband, and for any
 *                          charge before RUN
 *
 * Encoding: raw = 511 - I / 0.49 (X431_I_ZERO_RAW, X431_A_PER_COUNT in
 * config.h), clamped to 0..1023. 511 (0x1FF) is 0 A, the only rest value
 * the ECU has accepted with the contactors open.
 *
 * Pure C, shared with the host tests.
 */
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Amps -> raw, rounded DOWN (floor). The OEM slot-0 b13 copy of the current
 * is the floor of the filtered value, so it equals b2/b8 or is 1 count
 * lower. (Kept for the tests; the firmware now sends a burst average in
 * slot-0 b13, see main.c gx13_from_history().) */
static inline uint16_t bth5_encodeCurrentFloor(float amps) {
    long r = (long)floorf((float)X431_I_ZERO_RAW - amps / X431_A_PER_COUNT + 1e-4f) + CURRENT_RAW_OFFSET;
    if (r < 0) r = 0;
    if (r > 1023) r = 1023;
    return (uint16_t)r;
}

/* Amps -> raw, rounded to nearest. This is what goes in b2 and b8. */
static inline uint16_t bth5_encodeCurrent(float amps) {
    long r = lroundf((float)X431_I_ZERO_RAW - amps / X431_A_PER_COUNT) + CURRENT_RAW_OFFSET;
    if (r < 0) r = 0;
    if (r > 1023) r = 1023;
    return (uint16_t)r;
}

/* Current state per burst, kept for the log ("latch" column):
 *   DB   inside the deadband, 0x1FF sent
 *   DIS  live discharge
 *   CHG  live charge (RUN only)
 *   ZERO auto-zero window at power-up, 0x1FF sent
 *   CHGH charge measured before RUN, held at 0x1FF */
#define BTH5_IST_DB   2
#define BTH5_IST_DIS  3
#define BTH5_IST_CHG  4
#define BTH5_IST_ZERO 5   /* auto-zero window, 0x1FF is sent */
#define BTH5_IST_CHGH 6

/* Auto-zero: use the window mean as the sensor zero if it is inside
 * AUTO_ZERO_MIN_MV..MAX_MV, else the fixed CURRENT_ZERO_MV.
 * *accepted (optional) is set to 1 if the mean was used. */
static inline float bth5_autozero_pick(float mean_mv, int *accepted) {
    int ok = (mean_mv >= AUTO_ZERO_MIN_MV) && (mean_mv <= AUTO_ZERO_MAX_MV);
    if (accepted) *accepted = ok;
    return ok ? mean_mv : CURRENT_ZERO_MV;
}

/* Engine-start detector, fed one current sample per burst.
 *   PRE    -> CRANK   discharge >= 25 A (ENG_CRANK_DA) for ENG_CRANK_N bursts
 *   CRANK  -> SETTLE  discharge < 15 A (ENG_CRANK_END_DA)
 *   SETTLE -> RUN     5 bursts (~0.5 s) without another crank
 *   SETTLE -> CRANK   discharge >= 25 A again
 * RUN stays for the rest of the boot. "sent" holds the last b2/b8 raw. */
#define BTH5_ENG_PRE    0
#define BTH5_ENG_CRANK  1
#define BTH5_ENG_SETTLE 2
#define BTH5_ENG_RUN    3
typedef struct {
    int st;
    int n;
    uint16_t sent;      /* last sent b2/b8 raw (for the charge slew) */
} bth5_eng_t;
/* Start in PRE with 0x1FF as the last sent value. */
static inline void bth5_eng_init(bth5_eng_t *e) { e->st = BTH5_ENG_PRE; e->n = 0; e->sent = 0x1FF; }

/* Advance the detector by one burst. amps = discharge-positive. */
static inline void bth5_eng_step(bth5_eng_t *e, float amps) {
    switch (e->st) {
    case BTH5_ENG_PRE:
        if (amps >= ENG_CRANK_DA * 0.1f) { if (++e->n >= ENG_CRANK_N) { e->st = BTH5_ENG_CRANK; e->n = 0; } }
        else e->n = 0;
        break;
    case BTH5_ENG_CRANK:
        if (amps < ENG_CRANK_END_DA * 0.1f) { e->st = BTH5_ENG_SETTLE; e->n = 0; }
        break;
    case BTH5_ENG_SETTLE:
        if (amps >= ENG_CRANK_DA * 0.1f) { e->st = BTH5_ENG_CRANK; e->n = 0; }
        else if (++e->n >= ENG_SETTLE_N) e->st = BTH5_ENG_RUN;
        break;
    default: break; /* RUN is sticky for the rest of the boot */
    }
}

/* Deadband / charge-hold policy. r and r13 hold the encoded current on
 * entry and are overwritten with 0x1FF where needed.
 *   Before RUN: discharge >= 0.5 A live; any charge >= 0.5 A held at 0x1FF
 *               (CHGH); otherwise 0x1FF (DB).
 *   In RUN:     |I| >= 0.5 A live in both directions; otherwise 0x1FF.
 * There is no charge slew limit in this build, so *slewed is always 0.
 * Returns the BTH5_IST_* state. */
static inline int bth5_applyCurrentPolicy(uint16_t *r, uint16_t *r13, float amps, bth5_eng_t *e, int *slewed) {
    int run = (e->st == BTH5_ENG_RUN);
    float db_dis = (run ? RUN_DB_DA : CURRENT_DIS_DB_DA) * 0.1f;
    float db_chg = (run ? RUN_DB_DA : CURRENT_CHG_DB_DA) * 0.1f;
    int st;
    if (amps >= db_dis) st = BTH5_IST_DIS;
    else if (-amps >= db_chg) st = run ? BTH5_IST_CHG : BTH5_IST_CHGH;
    else st = BTH5_IST_DB;
    if (st == BTH5_IST_DB || st == BTH5_IST_CHGH) { *r = 0x1FF; *r13 = 0x1FF; }
    *slewed = 0;
    e->sent = *r;
    return st;
}

/* Hall sensor state.
 *   filt_amps      value returned (no filtering: CURRENT_FILTER_ALPHA = 1)
 *   last_pin_mv    last ADC pin voltage, last_raw_amps the unfiltered amps
 *   bad_run/good_run  consecutive out-of-range / in-range samples
 *   fault          1 = sensor fault, 0 A is sent
 *   primed         0 until the first good sample
 *   zero_mv        sensor zero in use (set by the auto-zero) */
typedef struct {
    float filt_amps;
    float last_pin_mv;
    float last_raw_amps;
    int bad_run, good_run;
    int fault;          /* 1 = sensor fault, sending 0 A */
    int primed;
    float zero_mv;      /* sensor zero in use (CURRENT_ZERO_MV until the auto-zero sets it) */
} bth5_isense_t;

/* Reset the sensor state; zero starts at CURRENT_ZERO_MV. */
static inline void bth5_isense_init(bth5_isense_t *s) {
    memset(s, 0, sizeof *s);
    s->zero_mv = CURRENT_ZERO_MV;
    s->fault = 0;       /* the first in-range sample is used at once */
}

/* Feed one averaged ADC reading (12-bit counts). Returns amps,
 * discharge positive: I = (zero_mv - sensor_mv) / MV_PER_AMP.
 * Sensor output outside 0.4..4.6 V for 5 samples = fault, 0 A until it is
 * back in range for 5 samples. A shorter glitch holds the last value. */
static inline float bth5_isense_step(bth5_isense_t *s, float adc_counts) {
    float pin_mv = adc_counts * (ADC_VREF_MV / 4096.0f);
    float sens_mv = pin_mv * DIVIDER_RATIO;
    int bad = (sens_mv < SENSOR_FAULT_LO_MV) || (sens_mv > SENSOR_FAULT_HI_MV);
    /* Amps are discharge-positive. With CURRENT_SIGN = +1 (checked on the
     * car with a 12 V load test): sensor above zero -> negative I (charge);
     * more load (sensor output drops) -> I more positive (discharge) -> raw
     * further below 511 (raw = 511 - I/0.49). */
    float a = ((s->zero_mv - sens_mv) / MV_PER_AMP) * (float)CURRENT_SIGN;
    s->last_pin_mv = pin_mv;
    s->last_raw_amps = a;
    if (bad) { s->bad_run++; s->good_run = 0; }
    else     { s->good_run++; s->bad_run = 0; }
    if (!s->fault && s->bad_run >= SENSOR_FAULT_SET_N) s->fault = 1;
    if (s->fault && s->good_run >= SENSOR_FAULT_CLR_N) { s->fault = 0; s->primed = 0; }
    if (s->fault) { s->filt_amps = 0.0f; return 0.0f; }
    if (bad) return s->filt_amps;               /* hold filter through a short glitch */
    if (!s->primed) { s->filt_amps = a; s->primed = 1; }
    else s->filt_amps += CURRENT_FILTER_ALPHA * (a - s->filt_amps);
    return s->filt_amps;
}

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSOR_H */
