/* bmw_pack.h -- model of the BMW Gen1 PHEV battery pack built from the
 * modules' CAN frames: cell voltages, temperatures and a health state.
 *
 * Runs on core 1 (main.c core1_can()). pack_decode_can() takes each CAN
 * frame, pack_update() is called every 10 ms to recompute pack totals.
 *
 * CAN IDs (N = module address nibble, 0..6 accepted):
 *   0x12N..0x17N  cell voltages, 3 cells per frame (1 in 0x17N),
 *                 16-bit little-endian mV, top 2 bits are status flags
 *   0x18N         temperatures, 4 bytes, value = byte - 40 C
 *                 (same as stm32_betty, src/stm32_betty.cpp:414)
 * Other IDs are ignored.
 */
#ifndef BMW_PACK_H
#define BMW_PACK_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Module address nibbles accepted: 0..6. Exactly EXPECTED_MODULES (5) of
 * them must be present. */
#define CSC_ADDR_MIN          0
#define CSC_ADDR_MAX          6
#define CSC_ADDR_SLOTS        (CSC_ADDR_MAX - CSC_ADDR_MIN + 1)
#define CSC_CELLS_PER_MOD     16
#define CSC_TEMPS_PER_MOD     4

#define PACK_CELLS            (EXPECTED_MODULES * CSC_CELLS_PER_MOD) /* 80 */

#define STALE_MS              500     /* module silent this long = not fresh */

/* Cell guard. A cell word is 14-bit mV + 2 status bits (SimpBMS masks
 * the high byte with 0x3F); bit 14 was seen set at current peaks
 * (0x4DE3 = 3555 mV + 0x4000). A new value is rejected if it is outside
 * 2.5..4.3 V, or if it jumps more than 300 mV on its own (fewer than 2 other
 * cells of the module moved to within 150 mV of it in the last 250 ms).
 * A cell with no accepted value for 1 s takes its module's average. */
#define GUARD_RAW_MASK        0x3FFFu
#define GUARD_MV_MIN          2500u
#define GUARD_MV_MAX          4300u
#define GUARD_JUMP_MV         300.0f
#define GUARD_HOLD_MS         1000u
#define GUARD_CORR_MV         150
#define GUARD_CORR_N          2
#define GUARD_CORR_MS         250u

#define RESUME_HEALTHY_MS     2000    /* after a dropout, healthy this long before RUN again */
#define CELL_IMPLAUS_MIN      2.5f    /* health check limits (BTH_REASON_CELL) */
#define CELL_IMPLAUS_MAX      4.3f
#define CELL_SEEN_MIN         0.5f    /* a cell reading must be inside this to count as seen */
#define CELL_SEEN_MAX         5.0f
#define TEMP_UNUSED_LO        54.5f   /* 54.5..55.5 C = unused thermistor channel, skipped */
#define TEMP_UNUSED_HI        55.5f
#define TEMP_MISSING          (-39.0f) /* <= -39 C (byte 0 -> -40) = missing, skipped */
#define REST_AMPS             1.5f    /* SOC estimate: |I| below this = resting */
#define OCV_SEED_MS           2000    /* SOC estimate: rest time before seeding from OCV */
#define OCV_RESYNC_MS         8000
#define OCV_SEED_TIMEOUT_MS   5000

/* Why the pack is not healthy (pack_state_t.reason). */
typedef enum {
    BTH_REASON_OK = 0,
    BTH_REASON_BOOT,    /* have not yet seen all expected modules */
    BTH_REASON_STALE,   /* a locked module was silent for STALE_MS */
    BTH_REASON_CELL,    /* a cell is outside 2.5-4.3 V */
    BTH_REASON_CAN,     /* controller bus-off / SPI / mode error */
    BTH_REASON_COUNT    /* present != EXPECTED_MODULES */
} bth_reason_t;

/* One module (one CAN address). */
typedef struct {
    bool exists;
    bool complete;                 /* all 16 cells have been seen in range */
    uint32_t last_ms;              /* last voltage or temp frame */
    uint32_t vlast_ms;             /* time of the last cell-voltage frame (0x12N-0x17N) */
    uint32_t good_ms[CSC_CELLS_PER_MOD]; /* cell guard: time of each cell's last accepted value */
    uint16_t raw_mv[CSC_CELLS_PER_MOD];  /* latest in-range masked word, accepted or not */
    uint32_t raw_ms[CSC_CELLS_PER_MOD];
    float cell[CSC_CELLS_PER_MOD];
    uint16_t cell_seen_mask;       /* bit i set once cell i was valid */
    float temp[CSC_TEMPS_PER_MOD];
    uint8_t n_temps;
} csc_module_t;

/* The whole pack. Only some fields feed BTH: cell[], present_mask,
 * n_present, min/max cell and the temperature summary (tmin/tmax/tavg,
 * n_temp_used). SOC and reason are internal / diagnostic. */
typedef struct {
    csc_module_t mod[CSC_ADDR_SLOTS];
    uint8_t locked_mask;           /* bit = addr, frozen on transition to RUN */
    uint8_t present_mask;          /* currently complete + not stale */
    uint8_t n_present;
    bool can_error;
    bool current_warn;
    float amps;
    float pack_v;
    float min_cell, max_cell;
    uint8_t min_addr, min_idx, max_addr, max_idx;
    float tmin, tmax, tavg;
    uint8_t n_temp_used;
    float soc_ah, soc_real, soc_ocv;
    bool soc_seeded;
    uint32_t rest_since_ms;
    uint32_t born_ms;
    bool have_born;
    bool running;
    bool ever_started;
    bth_reason_t reason;
    uint32_t healthy_since_ms;
    bool healthy_timing;
    uint16_t v_raw;                /* packV/20 encoded, 10-bit */
    uint16_t raw_block[20];        /* 4-cell groups, address-ascending cells, 10-bit */
    uint32_t last_update_ms;
    uint32_t cell_rej;             /* cell guard: values rejected (range / lone jump) */
    uint32_t cell_flag;            /* cell words with status bits 14/15 set (masked off) */
} pack_state_t;

/* Clear the model; temps start at -40 (missing). */
void pack_init(pack_state_t *p);
/* Decode one received standard-ID CAN frame. */
void pack_decode_can(pack_state_t *p, uint32_t id, const uint8_t *d, uint8_t dlc,
                     uint32_t now_ms);
/* Store the pack current (used only by the SOC estimate). */
void pack_set_current(pack_state_t *p, float amps, bool warn);
/* Flag a CAN controller error (bus-off, SPI or mode failure). */
void pack_set_can_error(pack_state_t *p, bool err);
/* Recompute totals, SOC estimate and health. Call every 10 ms. */
void pack_update(pack_state_t *p, uint32_t now_ms, float dt);
/* Text for a bth_reason_t, for debug prints. */
const char *pack_reason_str(bth_reason_t r);

/* State of charge (%) from one cell's open-circuit voltage, 11-point NMC
 * table from stm32_betty (ocvSocFromCell). */
float pack_ocv_soc(float cell_v);

#ifdef __cplusplus
}
#endif

#endif /* BMW_PACK_H */
