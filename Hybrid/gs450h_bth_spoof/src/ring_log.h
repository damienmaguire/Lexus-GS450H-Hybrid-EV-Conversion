/* ring_log.h -- rolling flash log of the last ~7 minutes: the pure,
 * host-testable parts (record layout, write-pointer search, page filling,
 * erase deferral, CSV). The flash I/O is in flash_log.c.
 *
 * One 32-byte record per BTH burst (106.1 ms) goes into a ring of
 * RING_SECTORS (32) 4 KB sectors = 4096 slots. Each slot is written once,
 * by programming a 256-byte page that is 0xFF except for the new slots, so
 * bytes already written are not touched. Before the writer enters a
 * sector, that sector is erased. The ring always holds 30..31 sectors of
 * history (3840..3968 rows, ~407..421 s).
 *
 * rec.b.pad holds the boot id (meta boot_count mod 0xFFFF) so a dump can
 * separate boots; rec.b.t_ms is ms since that boot. A marker record
 * (latch = RING_MARK_ZERO) stores the auto-zero: adc_mv = zero used,
 * adc_counts = window mean.
 *
 * Nothing is written in the first 3 s of a boot, or at all once a USB host
 * is connected, so plugging in a laptop never overwrites the car log.
 */
#ifndef RING_LOG_H
#define RING_LOG_H
#include <stdint.h>
#include <string.h>
#include "flash_log.h"

/* 32-byte ring record = the 16-byte bootlog_rec_t + 16 bytes of block and
 * temperature data. Some fields were re-used as the code developed:
 *   blk0w:      [9:0] block-0 raw sent, [12:10] complete modules (0..7),
 *               [13] a cell word with status bits was seen since the last
 *               burst, [14] a sector erase ran in the gap before this burst,
 *               [15] live module data (pub.live)
 *   age_new_ms: cumulative cell-guard rejects (not an age any more)
 *   age_old_ms: ms since the oldest module's last cell frame
 *               (0xFFFF = no module yet)
 *   blk_age_ms: module temps, (tmin + 40) | (tmax + 40) << 8, 0xFFFF = none */
typedef struct __attribute__((packed)) {
    bootlog_rec_t b;
    uint16_t pack_raw;     /* sum of the 20 block raws sent (V = raw * 25/1023) */
    uint16_t blk0w;        /* see above */
    uint16_t blk19_raw;    /* block 19 raw sent [9:0]; [14:12] modules estimated (stale) */
    uint16_t cmin_mv, cmax_mv;
    uint16_t age_new_ms;   /* newest module cell data */
    uint16_t age_old_ms;   /* oldest module cell data */
    uint16_t blk_age_ms;   /* since core 1 last changed any block raw */
} ring_rec_t;
_Static_assert(sizeof(ring_rec_t) == 32, "ring_rec_t packing");
/* The 16-byte head of a ring record. */
#define RR_B(r) (&(r)->b)
#define RING_SECTORS_DEFAULT 32u   /* 128 KB: 30..31 sectors of history = 3840..3968 rows (~407..421 s) */

#ifndef RING_SECTORS
#define RING_SECTORS      RING_SECTORS_DEFAULT
#endif
#define RING_SECTOR_BYTES 4096u
#define RING_SPS          (RING_SECTOR_BYTES / sizeof(ring_rec_t))  /* 128 slots per sector */
#define RING_SPP          (256u / sizeof(ring_rec_t))               /* 8 slots per page */
#define RING_SLOTS        (RING_SECTORS * RING_SPS)                    /* 4096 */
#define RING_FIFO         128u      /* RAM backlog, 13.6 s of bursts */
#define RING_HOLDOFF_MS   3000u     /* no flash writes before this; lets a laptop enumerate first */
#define RING_MARK_ZERO    0xFEu
#define RING_TAIL_N       100u      /* 't' dump: last ~10.6 s */
#define RING_BATCH        4u        /* program when >= 4 records pending: <= 4 partial programs
                                       per page; <= ~0.4 s (+ the burst in flight) lost at power-off */

/* RAM queue of records waiting to be written to flash. */
typedef struct {
    ring_rec_t r[RING_FIFO];
    uint16_t head, n;
    uint32_t dropped;
} ring_fifo_t;

/* Add a record; if the queue is full the oldest one is dropped (counted). */
static inline void ring_fifo_push(ring_fifo_t *f, const ring_rec_t *r) {
    if (f->n == RING_FIFO) { f->head = (uint16_t)((f->head + 1) % RING_FIFO); f->n--; f->dropped++; }
    f->r[(f->head + f->n) % RING_FIFO] = *r;
    f->n++;
}

/* 1 if every byte of the slot is 0xFF. */
static inline int ring_slot_erased(const ring_rec_t *r) {
    const uint8_t *p = (const uint8_t *)r;
    for (unsigned i = 0; i < sizeof *r; i++) if (p[i] != 0xFF) return 0;
    return 1;
}
/* 1 if the slot holds a complete record (not erased, not torn). */
static inline int ring_slot_valid(const ring_rec_t *r) {
    return !ring_slot_erased(r) && RR_B(r)->t_ms != 0xFFFFFFFFu && RR_B(r)->pad != 0xFFFFu;
}
/* 1 if the whole sector is 0xFF. */
static inline int ring_sector_erased(const ring_rec_t *flash, unsigned sec) {
    const uint32_t *w = (const uint32_t *)(const void *)&flash[sec * RING_SPS];
    for (unsigned i = 0; i < RING_SECTOR_BYTES / 4; i++) if (w[i] != 0xFFFFFFFFu) return 0;
    return 1;
}
/* Boot id stored in each record. */
static inline uint16_t ring_boot_id(uint32_t boot_count) {
    return (uint16_t)(boot_count % 0xFFFFu);   /* never 0xFFFF (= erased) */
}

/* Newest valid slot by (boot id, t_ms); -1 if the ring is empty. */
static inline int ring_find_newest(const ring_rec_t *flash) {
    int best = -1; uint64_t bk = 0;
    for (unsigned i = 0; i < RING_SLOTS; i++) {
        if (!ring_slot_valid(&flash[i])) continue;
        uint64_t k = ((uint64_t)RR_B(&flash[i])->pad << 32) | RR_B(&flash[i])->t_ms;
        if (best < 0 || k >= bk) { best = (int)i; bk = k; }
    }
    return best;
}

/* Boot: write pointer and the one sector that must be erased first (-1 none). */
static inline int ring_init_plan(const ring_rec_t *flash, uint32_t *wr) {
    int nw = ring_find_newest(flash);
    unsigned sec;
    *wr = (nw < 0) ? 0u : (uint32_t)((nw + 1) % RING_SLOTS);
    sec = *wr / RING_SPS;
    if (*wr % RING_SPS == 0)
        return ring_sector_erased(flash, sec) ? -1 : (int)sec;
    sec = (sec + 1) % RING_SECTORS;
    return ring_sector_erased(flash, sec) ? -1 : (int)sec;
}

/* Result of ring_fill_page(). */
typedef struct {
    int n;            /* records placed in page_buf (0 = nothing to program) */
    unsigned page;    /* ring page index (0..255) to program */
    int need_erase;   /* sector to erase next (-1 none) */
} ring_fill_t;

/* Move as many FIFO records as fit into the page at *wr. Slots that are not
 * erased (torn write after power loss) are skipped. Never places a record in
 * a sector that is not erased at its first slot. */
static inline ring_fill_t ring_fill_page(const ring_rec_t *flash, uint32_t *wr,
                                         ring_fifo_t *f, uint8_t *page_buf) {
    ring_fill_t o; int entered = -1;
    o.n = 0; o.page = *wr / RING_SPP; o.need_erase = -1;
    memset(page_buf, 0xFF, 256);
    while (f->n && *wr / RING_SPP == o.page) {
        if (*wr % RING_SPS == 0 && !ring_sector_erased(flash, *wr / RING_SPS)) {
            if (o.n == 0) o.need_erase = (int)(*wr / RING_SPS);
            break;
        }
        if (!ring_slot_erased(&flash[*wr])) { *wr = (*wr + 1) % RING_SLOTS; continue; }
        memcpy(page_buf + (*wr % RING_SPP) * sizeof(ring_rec_t), &f->r[f->head], sizeof(ring_rec_t));
        f->head = (uint16_t)((f->head + 1) % RING_FIFO); f->n--;
        if (*wr % RING_SPS == 0) entered = (int)(*wr / RING_SPS);
        o.n++;
        *wr = (*wr + 1) % RING_SLOTS;
    }
    if (entered >= 0) {
        unsigned nx = ((unsigned)entered + 1) % RING_SECTORS;
        if (!ring_sector_erased(flash, nx)) o.need_erase = (int)nx;
    }
    return o;
}

/* Dump target: boot id of the newest valid record that is not this boot
 * (the car run, when dumping from a laptop). Falls back to cur_boot. */
static inline uint16_t ring_pick_target(const ring_rec_t *flash, uint32_t wr, uint16_t cur_boot) {
    int have = 0; uint16_t t = cur_boot;
    for (unsigned k = 0; k < RING_SLOTS; k++) {
        const ring_rec_t *r = &flash[(wr + k) % RING_SLOTS];
        if (ring_slot_valid(r) && RR_B(r)->pad != cur_boot) { t = RR_B(r)->pad; have = 1; }
    }
    (void)have;
    return t;
}
/* Should a pending sector erase wait? During an erase the bursts sent
 * all repeat one frozen snapshot (current included), so the erase is put
 * off while |I| > ERASE_DEFER_A (10 A) or the engine is cranking/settling.
 * If the RAM backlog reaches 3/4 of the queue it erases anyway, but only
 * outside CRANK/SETTLE and only if the current moved < 3 A since the
 * previous gap. Returns 1 = wait. */
static inline int ring_erase_should_defer(float amps, float amps_prev, int eng_busy, unsigned fifo_n) {
    float a = amps < 0 ? -amps : amps;
    float da = amps - amps_prev;
    if (da < 0) da = -da;
    if (eng_busy) return 1;
    if (a <= ERASE_DEFER_A) return 0;
    return !(fifo_n >= (RING_FIFO * 3u) / 4u && da < 3.0f);
}

/* CSV columns added by ring_vlog_csv_ext(). mod_tmin_c / mod_tmax_c come
 * from the blk_age_ms slot; cell_flag and erase are blk0w bits 13 and 14. */
#define RING_VLOG_CSV_EXT \
    ",pack_v,blk0_mv,blk19_mv,cell_min_mv,cell_max_mv,cell_rej,age_old_ms,mod_tmin_c,mod_tmax_c,n_vmod,blk_live,cell_flag,erase"

/* Block raw -> mV exactly as the HV ECU decodes it (raw * 25/1023 V). */
static inline unsigned ring_blk_mv(unsigned raw) { return (raw * 25000u + 511u) / 1023u; }

/* Append the 32-byte-record columns to a row made by bootlog_csv_row(). */
static inline int ring_vlog_csv_ext(char *dst, size_t dst_sz, const ring_rec_t *r) {
    size_t n = strlen(dst);
    if (n >= dst_sz) return -1;
    {
        int k;
        if (r->blk_age_ms == 0xFFFFu)
            k = snprintf(dst + n, dst_sz - n, ",%.3f,%u,%u,%u,%u,%u,%u,,,%u,%u",
                         (double)r->pack_raw * 25.0 / 1023.0,
                         ring_blk_mv(r->blk0w & 0x3FFu), ring_blk_mv(r->blk19_raw & 0x3FFu),
                         (unsigned)r->cmin_mv, (unsigned)r->cmax_mv,
                         (unsigned)r->age_new_ms, (unsigned)r->age_old_ms,
                         (unsigned)((r->blk0w >> 10) & 7u), (unsigned)(r->blk0w >> 15));
        else
            k = snprintf(dst + n, dst_sz - n, ",%.3f,%u,%u,%u,%u,%u,%u,%d,%d,%u,%u",
                         (double)r->pack_raw * 25.0 / 1023.0,
                         ring_blk_mv(r->blk0w & 0x3FFu), ring_blk_mv(r->blk19_raw & 0x3FFu),
                         (unsigned)r->cmin_mv, (unsigned)r->cmax_mv,
                         (unsigned)r->age_new_ms, (unsigned)r->age_old_ms,
                         (int)(r->blk_age_ms & 0xFFu) - 40, (int)(r->blk_age_ms >> 8) - 40,
                         (unsigned)((r->blk0w >> 10) & 7u), (unsigned)(r->blk0w >> 15));
        if (k < 0 || (size_t)k >= dst_sz - n) return -1;
        n += (size_t)k;
        return snprintf(dst + n, dst_sz - n, ",%u,%u", (unsigned)((r->blk0w >> 13) & 1u), (unsigned)((r->blk0w >> 14) & 1u));
    }
}
#endif /* RING_LOG_H */
