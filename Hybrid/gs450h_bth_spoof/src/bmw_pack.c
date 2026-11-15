/* bmw_pack.c -- decodes the BMW Gen1 PHEV module CAN frames into the pack
 * model (bmw_pack.h): cell voltages with a glitch guard, temperatures, pack
 * totals, the 20 BTH block raws, and an internal health / SOC estimate.
 * Runs on core 1.
 */
#include "config.h"
#include "bmw_pack.h"
#include "bth_link.h"

#include <string.h>

/* Short name of a health reason, for debug output. */
const char *pack_reason_str(bth_reason_t r) {
    switch (r) {
    case BTH_REASON_OK:    return "OK";
    case BTH_REASON_BOOT:  return "BOOT";
    case BTH_REASON_STALE: return "STALE";
    case BTH_REASON_CELL:  return "CELL";
    case BTH_REASON_CAN:   return "CAN";
    case BTH_REASON_COUNT: return "COUNT";
    }
    return "?";
}

/* Cell OCV -> SOC %, linear between the 11 table points (stm32_betty.cpp
 * lines 107-123). Used only by the internal SOC estimate. */
float pack_ocv_soc(float v) {
    /* stm32_betty.cpp:107-123 */
    static const float vv[] = {3.20f, 3.40f, 3.50f, 3.55f, 3.62f, 3.70f,
                               3.78f, 3.85f, 3.92f, 4.00f, 4.10f};
    static const float ss[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    const int n = 11;
    int i;
    if (v <= vv[0]) return ss[0];
    if (v >= vv[n - 1]) return ss[n - 1];
    for (i = 1; i < n; i++) {
        if (v <= vv[i]) {
            float f = (v - vv[i - 1]) / (vv[i] - vv[i - 1]);
            return ss[i - 1] + f * (ss[i] - ss[i - 1]);
        }
    }
    return 50.0f;
}

/* Cell guard, pass 1 (per frame, before guard_cell): remember every
 * in-range word and when it came, so cells of the same frame can back each
 * other up in the jump check. */
static void guard_note_raw(csc_module_t *m, int idx, uint16_t raw, uint32_t now_ms) {
    uint16_t mv = (uint16_t)(raw & GUARD_RAW_MASK);
    if (mv >= GUARD_MV_MIN && mv <= GUARD_MV_MAX) { m->raw_mv[idx] = mv; m->raw_ms[idx] = now_ms; }
}

/* Cell guard, pass 2: accept or reject one cell word. Counts words with
 * status bits set (cell_flag) and rejected values (cell_rej). A rejected
 * value leaves the last good one in place. Always returns 1 (handled). */
static int guard_cell(pack_state_t *p, csc_module_t *m, int idx, uint16_t raw, uint32_t now_ms) {
    uint16_t mv;
    float v;
    int ok;
    if (raw & ~GUARD_RAW_MASK) p->cell_flag++;
    mv = (uint16_t)(raw & GUARD_RAW_MASK);
    v = (float)mv * 0.001f;
    ok = (mv >= GUARD_MV_MIN && mv <= GUARD_MV_MAX);
    if (ok && (m->cell_seen_mask & (1u << idx))) {
        float dj = (v - m->cell[idx]) * 1000.0f;
        if (dj > GUARD_JUMP_MV || dj < -GUARD_JUMP_MV) {
            /* Real load steps move every cell together: accept the jump only
             * if >= GUARD_CORR_N other cells of this module recently reported
             * a word within GUARD_CORR_MV of the new value. */
            int c, n = 0;
            for (c = 0; c < CSC_CELLS_PER_MOD; c++) {
                int dc;
                if (c == idx || !(m->cell_seen_mask & (1u << c))) continue;
                if ((uint32_t)(now_ms - m->raw_ms[c]) > GUARD_CORR_MS) continue;
                dc = (int)m->raw_mv[c] - (int)mv;
                if (dc <= (int)GUARD_CORR_MV && dc >= -(int)GUARD_CORR_MV) n++;
            }
            if (n < GUARD_CORR_N) ok = 0;
        }
    }
    if (!ok) { p->cell_rej++; return 1; }   /* keep the last good value */
    m->cell[idx] = v;
    m->cell_seen_mask |= (uint16_t)(1u << idx);
    m->good_ms[idx] = now_ms;
    return 1;
}

/* Cells of a complete module with no accepted value for GUARD_HOLD_MS (1 s)
 * take the average of that module's good cells. */
static void guard_tick(pack_state_t *p, uint32_t now_ms) {
    int a, c;
    for (a = 0; a < CSC_ADDR_SLOTS; a++) {
        csc_module_t *m = &p->mod[a];
        float sum = 0.0f;
        int n = 0;
        uint16_t stale = 0;
        if (!m->complete) continue;
        for (c = 0; c < CSC_CELLS_PER_MOD; c++) {
            if ((uint32_t)(now_ms - m->good_ms[c]) > GUARD_HOLD_MS) stale |= (uint16_t)(1u << c);
            else { sum += m->cell[c]; n++; }
        }
        if (!stale || !n) continue;
        for (c = 0; c < CSC_CELLS_PER_MOD; c++)
            if (stale & (1u << c)) m->cell[c] = sum / (float)n;
    }
}

/* Clear everything. SOC starts at 50 %, temps at -40 C (missing), the
 * uniform voltage at 296 V / 20. */
void pack_init(pack_state_t *p) {
    int a, c, t;
    memset(p, 0, sizeof(*p));
    p->reason = BTH_REASON_BOOT;
    p->soc_ah = PACK_AH * 0.5f;
    p->soc_real = 50.0f;
    p->soc_ocv = 50.0f;
    p->v_raw = bth_encodeVoltage(296.0f);
    for (a = 0; a < CSC_ADDR_SLOTS; a++) {
        for (c = 0; c < CSC_CELLS_PER_MOD; c++) p->mod[a].cell[c] = 0.0f;
        for (t = 0; t < CSC_TEMPS_PER_MOD; t++) p->mod[a].temp[t] = -40.0f;
    }
}

/* Module address nibble inside 0..6? */
static int addr_ok(int addr) {
    return addr >= CSC_ADDR_MIN && addr <= CSC_ADDR_MAX;
}

/* Decode one CAN frame (see the ID table in bmw_pack.h).
 *   0x18N: temps, byte - 40 C, up to 4 per module.
 *   0x12N..0x17N: cells (mid - 2) * 3 .. +2, masked to 14 bits and passed
 *   through the cell guard. guard_cell() always returns 1, so the plain
 *   store after it is never reached in this build.
 * A module only counts as fresh from cell or temp frames, never from other
 * IDs, so a module that stops answering polls goes stale. */
void pack_decode_can(pack_state_t *p, uint32_t id, const uint8_t *d, uint8_t dlc,
                     uint32_t now_ms) {
    int addr, mid, i;
    csc_module_t *m;
    if (id > 0x7FF || d == 0) return;
    addr = (int)(id & 0xF);
    if (!addr_ok(addr)) return;
    m = &p->mod[addr - CSC_ADDR_MIN];
    mid = (int)((id >> 4) & 0xF);

    /* Solicited temps: 0x18N only, not 0x19N / 0x1AN / 0x20N. */
    if ((id & 0xFF0) == 0x180) {
        m->exists = true;
        m->last_ms = now_ms;
        m->n_temps = 0;
        for (i = 0; i < 4 && i < dlc; i++) {
            m->temp[i] = (float)d[i] - 40.0f;
            m->n_temps++;
        }
        return;
    }

    /* Cell voltages, mid 2-7 → 0x12N-0x17N. Full 16-bit LE millivolts. */
    if (mid >= 2 && mid <= 7) {
        int base = (mid - 2) * 3;
        int n = (mid == 7) ? 1 : 3;
        m->exists = true;
        m->last_ms = now_ms;
        m->vlast_ms = now_ms;
        if (dlc < (uint8_t)(n * 2)) return;
        for (i = 0; i < n && base + i < CSC_CELLS_PER_MOD; i++)
            guard_note_raw(m, base + i, (uint16_t)((uint16_t)d[i * 2] | ((uint16_t)d[i * 2 + 1] << 8)), now_ms);
        for (i = 0; i < n; i++) {
            int idx = base + i;
            uint16_t mv;
            float v;
            if (idx >= CSC_CELLS_PER_MOD) break;
            mv = (uint16_t)d[i * 2] | ((uint16_t)d[i * 2 + 1] << 8);
            /* 14-bit mV + 2 status bits (mask the top 2 bits), on every path */
            if (mv & ~0x3FFFu) p->cell_flag++;   /* count here; the guard then sees the masked word */
            mv &= 0x3FFFu;
            if (guard_cell(p, m, idx, mv, now_ms)) continue;
            v = (float)mv * 0.001f;
            if (v > CELL_SEEN_MIN && v < CELL_SEEN_MAX) {
                m->cell[idx] = v;
                m->cell_seen_mask |= (uint16_t)(1u << idx);
            } else if (mv != 0) {
                /* Out-of-window but non-zero: still store so the implausible
                 * check can see a <2.5 or >4.3 reading (e.g. 2.4 V is
                 * inside SEEN but 2.0 V is also inside SEEN). Anything
                 * between 0.5 and 5.0 is stored above. Below 0.5 or above
                 * 5.0 we still record it so CELL faults trigger. */
                m->cell[idx] = v;
                m->cell_seen_mask |= (uint16_t)(1u << idx);
            }
        }
        m->complete = (m->cell_seen_mask == 0xFFFFu);
        return;
    }

    /* Ignore heartbeats / status / unsolicited 0x20N for last_ms so a
     * module that is no longer being polled cannot look fresh. */
    (void)now_ms;
}

/* Current for the SOC estimate (charge positive, here always 0). */
void pack_set_current(pack_state_t *p, float amps, bool warn) {
    p->amps = amps;
    p->current_warn = warn;
}

/* Record whether the CAN controller is in error. */
void pack_set_can_error(pack_state_t *p, bool err) {
    p->can_error = err;
}

/* Skip the -40 C "missing" code and the ~55 C "unused channel" code. */
static bool temp_usable(float tp) {
    if (tp <= TEMP_MISSING) return false;
    if (tp >= TEMP_UNUSED_LO && tp <= TEMP_UNUSED_HI) return false;
    return true;
}

/* Rebuild the pack totals from the fresh, complete modules: pack voltage
 * (sum of cells), min/max cell, temperature min/max/average over the
 * usable thermistors, and the 20 block raws (4 cells each). If no
 * thermistor is usable, every non-missing reading is used instead so
 * there is still something to show. */
static void recompute(pack_state_t *p, uint32_t now_ms) {
    int a, c, t;
    float sum = 0.0f, cmin = 99.0f, cmax = 0.0f;
    float tmin = 200.0f, tmax = -100.0f, tsum = 0.0f;
    int n_temp = 0;
    int n_cells = 0;
    uint8_t present = 0;
    uint8_t n_present = 0;
    uint8_t min_a = 0, min_i = 0, max_a = 0, max_i = 0;
    float cells_flat[EXPECTED_MODULES * CSC_CELLS_PER_MOD];
    int n_flat = 0;

    p->last_update_ms = now_ms;
    guard_tick(p, now_ms);

    for (a = 0; a < CSC_ADDR_SLOTS; a++) {
        csc_module_t *m = &p->mod[a];
        bool fresh;
        if (!m->complete) continue;
        fresh = (now_ms - m->last_ms) <= STALE_MS;
        /* unsigned wrap: if last_ms is in the future, treat as fresh */
        if (now_ms < m->last_ms) fresh = true;
        if (!fresh) continue;
        present |= (uint8_t)(1u << a);
        n_present++;
        for (c = 0; c < CSC_CELLS_PER_MOD; c++) {
            float v = m->cell[c];
            sum += v;
            n_cells++;
            if (n_flat < (int)sizeof(cells_flat) / (int)sizeof(cells_flat[0])) {
                cells_flat[n_flat++] = v;
            }
            if (v < cmin) { cmin = v; min_a = (uint8_t)(a + CSC_ADDR_MIN); min_i = (uint8_t)c; }
            if (v > cmax) { cmax = v; max_a = (uint8_t)(a + CSC_ADDR_MIN); max_i = (uint8_t)c; }
        }
        for (t = 0; t < m->n_temps && t < CSC_TEMPS_PER_MOD; t++) {
            float tp = m->temp[t];
            if (!temp_usable(tp)) continue;
            n_temp++;
            tsum += tp;
            if (tp < tmin) tmin = tp;
            if (tp > tmax) tmax = tp;
        }
    }

    /* If every channel looked unused, report all temps that are not the
     * missing-sensor floor so the operator can still see something. */
    if (n_temp == 0) {
        for (a = 0; a < CSC_ADDR_SLOTS; a++) {
            csc_module_t *m = &p->mod[a];
            if (!(present & (1u << a))) continue;
            for (t = 0; t < m->n_temps && t < CSC_TEMPS_PER_MOD; t++) {
                float tp = m->temp[t];
                if (tp <= TEMP_MISSING) continue;
                n_temp++;
                tsum += tp;
                if (tp < tmin) tmin = tp;
                if (tp > tmax) tmax = tp;
            }
        }
    }

    p->present_mask = present;
    p->n_present = n_present;
    p->pack_v = (n_cells > 0) ? sum : 0.0f;
    p->min_cell = (cmin < 99.0f) ? cmin : 0.0f;
    p->max_cell = cmax;
    p->min_addr = min_a;
    p->min_idx = min_i;
    p->max_addr = max_a;
    p->max_idx = max_i;
    p->n_temp_used = (uint8_t)n_temp;
    if (n_temp > 0) {
        p->tmin = tmin;
        p->tmax = tmax;
        p->tavg = tsum / (float)n_temp;
    } else {
        p->tmin = p->tmax = p->tavg = 0.0f;
    }

    p->v_raw = bth_encodeVoltage(p->pack_v);

    /* 20 blocks of 4 consecutive cells, modules in ascending CAN address. */
    {
        int b;
        for (b = 0; b < 20; b++) p->raw_block[b] = p->v_raw;
        if (n_flat == PACK_CELLS) {
            for (b = 0; b < 20; b++) {
                float bv = cells_flat[4 * b] + cells_flat[4 * b + 1]
                         + cells_flat[4 * b + 2] + cells_flat[4 * b + 3];
                /* Block voltage straight to the 10-bit BTH value (25/1023 V). */
                p->raw_block[b] = bth_encodeBlock(bv);
            }
        }
    }
}

/* Any cell of the modules in mask outside 2.5..4.3 V? */
static bool cells_implausible(const pack_state_t *p, uint8_t mask) {
    int a, c;
    for (a = 0; a < CSC_ADDR_SLOTS; a++) {
        const csc_module_t *m;
        if (!(mask & (1u << a))) continue;
        m = &p->mod[a];
        for (c = 0; c < CSC_CELLS_PER_MOD; c++) {
            float v = m->cell[c];
            if (v < CELL_IMPLAUS_MIN || v > CELL_IMPLAUS_MAX) return true;
        }
    }
    return false;
}

/* All modules in mask complete and heard from within STALE_MS? */
static bool mask_fresh(const pack_state_t *p, uint8_t mask, uint32_t now_ms) {
    int a;
    for (a = 0; a < CSC_ADDR_SLOTS; a++) {
        const csc_module_t *m;
        if (!(mask & (1u << a))) continue;
        m = &p->mod[a];
        if (!m->complete) return false;
        if (now_ms >= m->last_ms) {
            if ((now_ms - m->last_ms) > STALE_MS) return false;
        }
    }
    return mask != 0;
}

/* Number of bits set in a byte. */
static int popcount8(uint8_t x) {
    int n = 0;
    while (x) { n += x & 1; x >>= 1; }
    return n;
}

/* Internal SOC estimate (Ah counting, seeded and re-synced from the cell
 * OCV after a rest). Not sent on BTH; the HV ECU keeps its own SOC. */
static void update_soc(pack_state_t *p, uint32_t now_ms, float dt) {
    float avg;
    int n_cells = (int)p->n_present * CSC_CELLS_PER_MOD;
    bool resting;

    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;

    /* Positive current is charge. */
    if (p->soc_seeded) {
        if (!p->current_warn) p->soc_ah += p->amps * dt / 3600.0f;
        if (p->soc_ah < 0.0f) p->soc_ah = 0.0f;
        if (p->soc_ah > PACK_AH) p->soc_ah = PACK_AH;
        p->soc_real = 100.0f * p->soc_ah / PACK_AH;
    }

    if (n_cells > 0 && p->pack_v > 0.0f) {
        avg = p->pack_v / (float)n_cells;
        p->soc_ocv = pack_ocv_soc(avg);
    }

    resting = (p->amps > -REST_AMPS && p->amps < REST_AMPS && n_cells > 0);
    if (!p->have_born) {
        p->born_ms = now_ms;
        p->have_born = true;
        p->rest_since_ms = now_ms;
    }
    if (!resting) p->rest_since_ms = now_ms;

    if (!p->soc_seeded && n_cells > 0) {
        if ((resting && (now_ms - p->rest_since_ms) >= OCV_SEED_MS) ||
            (now_ms - p->born_ms) >= OCV_SEED_TIMEOUT_MS) {
            p->soc_ocv = pack_ocv_soc(p->pack_v / (float)n_cells);
            p->soc_real = p->soc_ocv;
            p->soc_ah = PACK_AH * (p->soc_real / 100.0f);
            p->soc_seeded = true;
        }
    } else if (p->soc_seeded && resting &&
               (now_ms - p->rest_since_ms) >= OCV_RESYNC_MS) {
        p->soc_ocv = pack_ocv_soc(p->pack_v / (float)n_cells);
        p->soc_real = p->soc_ocv;
        p->soc_ah = PACK_AH * (p->soc_real / 100.0f);
        p->rest_since_ms = now_ms;
    }
}

/* Recompute, update the SOC estimate, then decide health:
 * before the first start all 5 modules must be present and plausible; after
 * that the modules seen at start (locked_mask) must stay fresh and
 * plausible. After a dropout the pack must be healthy for 2 s before it is
 * "running" again. Note: BTH uses bth5_modules_live() (pack_to_bth.h), not
 * this running flag, to decide whether to send live blocks. */
void pack_update(pack_state_t *p, uint32_t now_ms, float dt) {
    bth_reason_t why = BTH_REASON_OK;
    uint8_t need;
    bool healthy;

    recompute(p, now_ms);
    update_soc(p, now_ms, dt);

    if (p->can_error) {
        why = BTH_REASON_CAN;
    } else if (!p->ever_started) {
        if (p->n_present < EXPECTED_MODULES) why = BTH_REASON_BOOT;
        else if (p->n_present > EXPECTED_MODULES) why = BTH_REASON_COUNT;
        else if (cells_implausible(p, p->present_mask)) why = BTH_REASON_CELL;
        else why = BTH_REASON_OK;
    } else {
        need = p->locked_mask ? p->locked_mask : p->present_mask;
        if (p->n_present != EXPECTED_MODULES) why = BTH_REASON_COUNT;
        else if (!mask_fresh(p, need, now_ms)) why = BTH_REASON_STALE;
        else if (cells_implausible(p, need)) why = BTH_REASON_CELL;
        else if (p->present_mask != need && popcount8(p->present_mask) != EXPECTED_MODULES)
            why = BTH_REASON_COUNT;
        else why = BTH_REASON_OK;
    }

    /* If we already started, a locked module going stale wins over count. */
    if (p->ever_started && p->locked_mask && !p->can_error) {
        if (!mask_fresh(p, p->locked_mask, now_ms)) why = BTH_REASON_STALE;
        else if (cells_implausible(p, p->locked_mask)) why = BTH_REASON_CELL;
    }

    p->reason = (why == BTH_REASON_OK) ? BTH_REASON_OK : why;
    healthy = (why == BTH_REASON_OK);

    if (healthy) {
        if (!p->healthy_timing) {
            p->healthy_since_ms = now_ms;
            p->healthy_timing = true;
        }
        if (!p->running) {
            /* First start: as soon as 5 modules are healthy.
             * After a dropout: require RESUME_HEALTHY_MS of continuous health. */
            uint32_t need_ms = p->ever_started ? RESUME_HEALTHY_MS : 0;
            if ((now_ms - p->healthy_since_ms) >= need_ms) {
                p->running = true;
                p->ever_started = true;
                p->locked_mask = p->present_mask;
                p->reason = BTH_REASON_OK;
            }
        }
    } else {
        p->healthy_timing = false;
        p->running = false;
    }
}

