/* pack_to_bth.h -- turns the BMW module data (bmw_pack.h) into the block
 * voltages sent on BTH, plus a few numbers for the flash log.
 *
 * Block layout: 20 blocks of 4 consecutive cells. Modules are taken in
 * ascending CAN address, cells 1..16 within each, so block 0 = cells 1-4
 * of the lowest-address module and block 19 = cells 13-16 of the highest.
 *
 * Pure C, shared with the host tests.
 */
#ifndef PACK_TO_BTH_H
#define PACK_TO_BTH_H
#include "bth_frames.h"
#include "bmw_pack.h"

/* Module data is usable when exactly EXPECTED_MODULES (5) modules are
 * complete and fresh and every cell reads 0.5..5.0 V. Cells outside
 * 2.5..4.3 V are still sent as measured, so the HV ECU's own block limits
 * can act on them. */
static inline int bth5_modules_live(const pack_state_t *p) {
    return p->n_present == EXPECTED_MODULES &&
           p->min_cell >= CELL_SEEN_MIN && p->max_cell <= CELL_SEEN_MAX;
}

/* Fill pub->raw_block[0..19] and pub->raw_uniform (average block) from the
 * modules and set pub->live = 1. Returns 1 on success. If the modules are
 * not usable it returns 0 and leaves pub untouched, so the last good
 * blocks (or the template, before the first good reading) keep going out. */
static inline int bth5_blocks_from_pack(const pack_state_t *p, bth5_pub_t *pub) {
    int a, c, n = 0, b = 0;
    float acc = 0.0f, total = 0.0f;
    uint16_t blk[20];
    if (!bth5_modules_live(p)) return 0;
    for (a = 0; a < CSC_ADDR_SLOTS && b < 20; a++) {
        if (!(p->present_mask & (1u << a))) continue;
        for (c = 0; c < CSC_CELLS_PER_MOD && b < 20; c++) {
            acc += p->mod[a].cell[c];
            if (++n == 4) { blk[b++] = bth5_encodeBlock(acc); total += acc; acc = 0.0f; n = 0; }
        }
    }
    if (b != 20) return 0;
    memcpy(pub->raw_block, blk, sizeof blk);
    pub->raw_uniform = bth5_encodeBlock(total / 20.0f);
    pub->live = 1;
    return 1;
}

/* Log-only statistics: min/max cell (mV), number of complete modules, and
 * the newest/oldest time a module sent cell voltages. Not sent on BTH. */
static inline void bth5_vlog_stats(const pack_state_t *p, bth5_pub_t *pub) {
    int a, n = 0;
    uint32_t vn = 0, vo = 0;
    for (a = 0; a < CSC_ADDR_SLOTS; a++) {
        const csc_module_t *m = &p->mod[a];
        if (!m->complete) continue;
        if (n == 0 || (int32_t)(m->vlast_ms - vn) > 0) vn = m->vlast_ms;
        if (n == 0 || (int32_t)(m->vlast_ms - vo) < 0) vo = m->vlast_ms;
        n++;
    }
    pub->n_vmod = (uint8_t)n;
    pub->vnew_ms = vn;
    pub->vold_ms = vo;
    pub->cmin_mv = (p->n_present && p->min_cell < 65.0f) ? (uint16_t)(p->min_cell * 1000.0f + 0.5f) : 0;
    pub->cmax_mv = (p->n_present && p->max_cell < 65.0f) ? (uint16_t)(p->max_cell * 1000.0f + 0.5f) : 0;
}
#endif /* PACK_TO_BTH_H */
