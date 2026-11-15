/* config.h -- every tunable value of the firmware, in one place.
 *
 * These are the values of the build that was tested on the car on
 * 15 Nov 2026 (development name i18qnb_aux_db05_tb5i): READY, D and R,
 * hard launches, zero DTCs. A plain build uses them, no extra flags needed.
 *
 * Each value is wrapped in #ifndef so a test or a one-off build can still
 * override it with -DNAME=value. Do not change them for the car without
 * testing; several are tied to measurements on this car.
 *
 * Units: "_DA" values are tenths of an amp (5 = 0.5 A). "_MV" values are
 * millivolts at the Hall sensor output (after undoing the divider).
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---------------- Hall current sensor on GP26 / ADC0 ---------------- */

#ifndef ADC_SAMPLES
#define ADC_SAMPLES           256      /* ADC reads averaged per burst (~0.5 ms) */
#endif
#ifndef ADC_VREF_MV
#define ADC_VREF_MV           3300.0f  /* RP2350 ADC full scale */
#endif
#ifndef DIVIDER_RATIO
#define DIVIDER_RATIO         1.5f     /* 10k/20k divider: sensor mV = pin mV * 1.5 */
#endif
#ifndef MV_PER_AMP
#define MV_PER_AMP            10.0f    /* sensor gain, 10 mV per amp */
#endif
#ifndef CURRENT_ZERO_MV
#define CURRENT_ZERO_MV       2565.0f  /* fallback sensor zero if the auto-zero is out of range */
#endif
#ifndef CURRENT_SIGN
#define CURRENT_SIGN          1        /* +1: sensor below zero = discharge = positive amps
                                          (sign checked on the car 15 Nov: pack V falls as I rises) */
#endif
#ifndef CURRENT_FILTER_ALPHA
#define CURRENT_FILTER_ALPHA  1.0f     /* 1.0 = no IIR filtering; the 256-sample mean is enough */
#endif
#ifndef SENSOR_FAULT_LO_MV
#define SENSOR_FAULT_LO_MV    400.0f   /* sensor output below this = unplugged/short */
#endif
#ifndef SENSOR_FAULT_HI_MV
#define SENSOR_FAULT_HI_MV    4600.0f  /* sensor output above this = fault */
#endif
#ifndef SENSOR_FAULT_SET_N
#define SENSOR_FAULT_SET_N    5        /* bad samples in a row before we send 0 A */
#endif
#ifndef SENSOR_FAULT_CLR_N
#define SENSOR_FAULT_CLR_N    5        /* good samples in a row before live current returns */
#endif

/* Auto-zero: the first AUTO_ZERO_N samples after power-up (one per burst,
 * ~0.5 s) are averaged and used as the sensor zero, if the mean is inside
 * MIN..MAX. Otherwise CURRENT_ZERO_MV is used. 0 A (0x1FF) is sent meanwhile. */
#ifndef AUTO_ZERO_N
#define AUTO_ZERO_N           5
#endif
#ifndef AUTO_ZERO_MIN_MV
#define AUTO_ZERO_MIN_MV      2400.0f
#endif
#ifndef AUTO_ZERO_MAX_MV
#define AUTO_ZERO_MAX_MV      2700.0f
#endif

/* ---------------- Current as sent on BTH ----------------
 * raw = X431_I_ZERO_RAW - I / X431_A_PER_COUNT, I in amps, discharge positive.
 * So 0 A = 511 (0x1FF), discharge goes below 511, charge above.
 * The X431 shows (512 - raw) * 0.49, so 0x1FF reads as 0.49 A on it. */
#ifndef X431_I_ZERO_RAW
#define X431_I_ZERO_RAW       511
#endif
#ifndef X431_A_PER_COUNT
#define X431_A_PER_COUNT      0.49f    /* amps per count */
#endif
#ifndef CURRENT_RAW_OFFSET
#define CURRENT_RAW_OFFSET    0        /* extra counts added after encoding (unused, 0) */
#endif

/* Deadbands (tenths of an amp). Inside the deadband exactly 0x1FF is sent.
 *   Before the engine-start detector reaches RUN: discharge >= 0.5 A is
 *   sent live, charge is always held at 0x1FF.
 *   In RUN: both directions live from 0.5 A. */
#ifndef CURRENT_DIS_DB_DA
#define CURRENT_DIS_DB_DA     5        /* pre-RUN discharge threshold, 0.5 A */
#endif
#ifndef CURRENT_CHG_DB_DA
#define CURRENT_CHG_DB_DA     5        /* pre-RUN charge threshold (charge is held anyway) */
#endif
#ifndef RUN_DB_DA
#define RUN_DB_DA             5        /* RUN deadband, both sides, 0.5 A */
#endif

/* Engine-start detector (one current sample per burst, discharge positive). */
#ifndef ENG_CRANK_DA
#define ENG_CRANK_DA          250      /* >= 25.0 A discharge = cranking */
#endif
#ifndef ENG_CRANK_N
#define ENG_CRANK_N           1        /* bursts above ENG_CRANK_DA to enter CRANK */
#endif
#ifndef ENG_CRANK_END_DA
#define ENG_CRANK_END_DA      150      /* crank over when discharge < 15.0 A */
#endif
#ifndef ENG_SETTLE_N
#define ENG_SETTLE_N          5        /* bursts (~0.53 s) after the crank before RUN */
#endif

/* ---------------- Aux (12 V) battery voltage ----------------
 * The Pico does not measure the 12 V battery. A fixed value is sent in
 * slot 3 X13. Scale from the car: raw 673 = 13.3 V, so 12.6 V = raw 638. */
#ifndef AUX_V_FIXED
#define AUX_V_FIXED           12.6f
#endif
#ifndef AUX_V_PER_COUNT
#define AUX_V_PER_COUNT       (13.3f / 673.0f)
#endif

/* ---------------- Battery temperatures TB1-4 and intake air ----------------
 * NTC beta law fitted to 5 X431 readings (max error 0.008 C):
 *   ln(raw / (TEMP_ADC_N - raw)) = TEMP_BETA_LNK + TEMP_BETA_K / T_kelvin
 * Points: 650 -> 28.15, 666 -> 26.29, 668 -> 26.06, 673 -> 25.48, 763 -> 14.46 C. */
#ifndef TEMP_BETA_K
#define TEMP_BETA_K           3297.8588f
#endif
#ifndef TEMP_BETA_LNK
#define TEMP_BETA_LNK         (-10.389791f)
#endif
#ifndef TEMP_ADC_N
#define TEMP_ADC_N            1023.0f
#endif
#ifndef TEMP_SEND_MIN_C
#define TEMP_SEND_MIN_C       5.0f     /* sent temps clamped to 5..40 C (raw 831..548) */
#endif
#ifndef TEMP_SEND_MAX_C
#define TEMP_SEND_MAX_C       40.0f
#endif
#ifndef TB_SLEW_C_PER_S
#define TB_SLEW_C_PER_S       0.05f    /* max change of a sent temp: 3 C per minute */
#endif
#ifndef TB_STALE_MS
#define TB_STALE_MS           5000u    /* no valid module temps this long -> slew back to template */
#endif
#ifndef TB_SPREAD_MAX_C
#define TB_SPREAD_MAX_C       5.0f     /* TB2 / TB4 kept within +-5 C of the average */
#endif
#ifndef TB_VALID_MIN_C
#define TB_VALID_MIN_C        (-30.0f) /* module temps outside this range = reading rejected */
#endif
#ifndef TB_VALID_MAX_C
#define TB_VALID_MAX_C        53.0f
#endif

/* ---------------- Checksum / b14 ----------------
 * The checksum itself is fixed (see bth_frames.h). These only control how
 * a frame is chosen when two encodings are both valid. */
#ifndef B14Q_KEEP_LEGACY
#define B14Q_KEEP_LEGACY      1        /* 1 (car build): if the older b14 rule already gives a
                                          frame that passes the two-part checksum, send that one
                                          unchanged. 0: always solve the nibble directly.
                                          The host tests run both. */
#endif
#ifndef B14_X13_LO
#define B14_X13_LO            482      /* older b14 rule: slot-0 low bits change when X13 is
                                          outside 482..511 (from the OEM car capture) */
#endif
#ifndef CHARGE_B14_SLOT0_LO
#define CHARGE_B14_SLOT0_LO   38       /* older b14 rule: slot-0 b14[5:0] value on the charge side */
#endif

/* ---------------- BMW modules ---------------- */
#ifndef EXPECTED_MODULES
#define EXPECTED_MODULES      5        /* 5 x 16 cells = 80 cells = 20 BTH blocks of 4 */
#endif
#ifndef PACK_AH
#define PACK_AH               26.0f    /* only used by the internal SOC estimate (diagnostics) */
#endif

/* ---------------- BTH timing ----------------
 * 1020 us per byte and an 8.18 ms gap after each 6-frame burst give the
 * OEM 106.1 ms burst period. flash_log.h checks these at compile time. */
#ifndef BYTE_PERIOD_US
#define BYTE_PERIOD_US        1020
#endif
#ifndef BURST_GAP_US
#define BURST_GAP_US          8180
#endif

/* ---------------- Flash log ---------------- */
#ifndef ZERO_META_KEEP
#define ZERO_META_KEEP        6u       /* auto-zero records kept when the meta sector is compacted */
#endif
#ifndef ERASE_DEFER_A
#define ERASE_DEFER_A         10.0f    /* put off a flash sector erase while |I| > 10 A */
#endif

#endif /* CONFIG_H */
