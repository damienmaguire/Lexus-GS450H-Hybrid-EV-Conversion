/* flash_log.h -- shared definitions for the flash log (flash_log.c,
 * ring_log.h): record layouts, the "meta" sector, CSV helpers and the
 * timing budget that lets flash writes fit between BTH bursts.
 *
 * Flash layout, top of the 4 MB chip:
 *   last sector        old 10 s boot-log bank 1 (not written by this build)
 *   last - 1           old 10 s boot-log bank 0 (not written by this build)
 *   last - 2           meta sector: 16 x 256-byte records (boot counter,
 *                      last attempt, and the auto-zero "ZERO" records)
 *   below that         16 sectors left free for older 16-byte ring logs,
 *                      then the 32-sector ring log (see ring_log.h)
 *
 * The portable parts (layouts, pickers, CSV) build on the host for the
 * tests; the flash I/O in flash_log.c is Pico-only (PICO_ON_DEVICE).
 */
#ifndef BTH_BOOTLOG_H
#define BTH_BOOTLOG_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"

/* ---- BTH burst timing (values in config.h) ----
 * 6 frames x 16 bytes x 1020 us = 97.92 ms of data, then an 8.18 ms gap:
 * 106.1 ms per burst, as the OEM board. Flash writes only happen in the gap
 * (or, for a sector erase, while RAM-resident code keeps sending bursts). */
#define BTH_BURST_TX_US      (6 * 16 * BYTE_PERIOD_US) /* 97920 */
#define BTH_BURST_PERIOD_US  (BTH_BURST_TX_US + BURST_GAP_US) /* 106100 */

#if BYTE_PERIOD_US != 1020
#error BYTE_PERIOD_US must stay 1020 (OEM 1020 us/byte)
#endif
#if BURST_GAP_US != 8180
#error BURST_GAP_US must stay 8180 (8.18 ms idle)
#endif
#if BTH_BURST_PERIOD_US != 106100
#error burst period is not 106.1 ms
#endif

/* 4 MB NOR flash on the Pico 2. A 256-byte page program (max 3 ms) fits in
 * the 8.18 ms gap. A 4 KB sector erase (max 400 ms) does not, so during an
 * erase RAM-resident code keeps sending up to 8 pre-built bursts
 * (8 x 106.1 ms > 400 ms). Checked at compile time below. */
#define BOOTLOG_PAGE_PROG_TYP_US     400u
#define BOOTLOG_PAGE_PROG_MAX_US     3000u
#define BOOTLOG_SECTOR_ERASE_TYP_US  45000u
#define BOOTLOG_SECTOR_ERASE_MAX_US  400000u
#define BOOTLOG_ERASE_BURST_BUDGET   8u

#if BOOTLOG_PAGE_PROG_MAX_US + 2000u >= BURST_GAP_US
#error page program max does not fit in the BTH idle gap
#endif
#if (BOOTLOG_ERASE_BURST_BUDGET * BTH_BURST_PERIOD_US) < BOOTLOG_SECTOR_ERASE_MAX_US
#error not enough RAM-resident bursts to cover max sector erase
#endif

/* Old 10 s boot-log format (header + up to 96 records). Only the layout is
 * still used here: bootlog_rec_t is the first half of every ring record. */
#define BOOTLOG_WINDOW_MS   10000u
#define BOOTLOG_MAX_REC     96u
#define BOOTLOG_MAGIC       0x314C5442u  /* 'BTL1' */
#define BOOTLOG_VERSION     1u
#define BOOTLOG_HDR_SIZE    256u
#define BOOTLOG_N_BANKS     2u

/* Meta sector: one 256-byte slot per record, 16 slots, appended in order.
 * When fewer than 3 slots are free at boot, it is compacted (erased and the
 * records worth keeping written back). Holds the boot counter, the state of
 * this/last boot's log ("attempt") and the auto-zero records. */
#define BOOTLOG_META_MAGIC    0x314D5442u  /* 'BTM1' */
#define BOOTLOG_META_VERSION  1u
#define BOOTLOG_META_PAGE     256u
#define BOOTLOG_META_SLOTS    (4096u / BOOTLOG_META_PAGE) /* 16 */

/* Attempt record reasons. */
#define BOOTLOG_REASON_BOOT          1u  /* this boot started, capture pending */
#define BOOTLOG_REASON_WRITTEN       2u
#define BOOTLOG_REASON_SKIP_HOST     3u  /* USB host mounted / CDC connected */
#define BOOTLOG_REASON_SKIP_EMPTY    4u  /* no samples in the 10 s window */
#define BOOTLOG_REASON_SKIP_OVERLAP  5u  /* program overlaps reserved flash */
#define BOOTLOG_REASON_FAIL_ERASE    6u
#define BOOTLOG_REASON_FAIL_PROGRAM  7u

/* 16-byte per-burst record (first half of a ring record). In this build:
 * adc_counts holds the slot-0 X13 sent, pad holds the boot id, and latch
 * packs current state [3:0] | engine state [5:4] | slewed [6]. */
typedef struct __attribute__((packed)) {
    uint32_t t_ms;          /* milliseconds since boot, at burst start */
    uint16_t adc_mv;        /* Hall sensor mV (divider-compensated) */
    uint16_t adc_counts;    /* 12-bit ADC average at the Pico pin (0..4095) */
    int16_t  amps_cA;       /* measured current, 0.01 A, discharge-positive */
    uint16_t i_raw;         /* sent current raw (b2/b8) */
    uint8_t  latch;         /* 0=HOLD, 1=LIVE, 2=DB, 3=DIS, 4=CHG, 0xFF=none */
    uint8_t  b14_slot0;     /* slot-0 b14 actually sent this burst */
    uint16_t pad;           /* keep records 16-byte and 4-aligned */
} bootlog_rec_t;

/* Header of the old boot-log bank. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t seq;
    uint32_t t_end_ms;
    uint32_t erase_us;      /* time spent in the interleaved sector erase */
    uint32_t prog_us;       /* total page-program time */
    uint32_t reserved;
} bootlog_hdr_t;

/* One meta-sector record (attempt record or ZERO record). */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t reason;
    uint32_t boot_count;
    uint32_t t_ms;
    uint16_t rec_count;
    uint8_t  vbus;          /* GP24 (VBUS) level, diagnostic only; host decides */
    uint8_t  host;          /* tud_mounted || tud_cdc_connected at this record */
    int32_t  flash_rc;      /* flash_safe_execute return, 0 = PICO_OK */
    uint32_t erase_us;
    uint32_t prog_us;
    uint32_t log_seq;
    uint32_t reserved[3];
} bootlog_meta_t;

/* ZERO record: each car boot's auto-zero result is saved as its own meta
 * record, so it survives the ring log wrapping. It is a record for the log
 * dump only; the firmware does NOT load it at the next boot (every boot
 * auto-zeroes again). reason = BOOTLOG_REASON_ZERO is outside the 1..7
 * range bootlog_meta_valid() accepts, so the attempt pickers skip it.
 *   boot_count = boot that took the zero, t_ms = when it was taken,
 *   rec_count  = samples averaged, reserved[0] = zero_mv | mean_mv << 16,
 *   reserved[1] = 1 auto (mean in range) / 0 fallback, reserved[2] = 'ZERO'. */
#define BOOTLOG_REASON_ZERO   8u
#define BOOTLOG_ZERO_TAG      0x4F52455Au   /* 'ZERO' */

/* 1 if m is a ZERO record. */
static inline int bootlog_zero_valid(const bootlog_meta_t *m) {
    return m && m->magic == BOOTLOG_META_MAGIC && m->version == BOOTLOG_META_VERSION
        && m->reason == BOOTLOG_REASON_ZERO && m->reserved[2] == BOOTLOG_ZERO_TAG
        && m->boot_count > 0;
}
/* Build a ZERO record (see the field list above). */
static inline void bootlog_zero_fill(bootlog_meta_t *m, uint32_t boot_count, uint32_t t_ms,
                                     uint16_t zero_mv, uint16_t mean_mv, uint16_t n, int accepted) {
    unsigned char *p = (unsigned char *)m; unsigned i;
    for (i = 0; i < sizeof *m; i++) p[i] = 0;
    m->magic = BOOTLOG_META_MAGIC;
    m->version = BOOTLOG_META_VERSION;
    m->reason = BOOTLOG_REASON_ZERO;
    m->boot_count = boot_count;
    m->t_ms = t_ms;
    m->rec_count = n;
    m->reserved[0] = (uint32_t)zero_mv | ((uint32_t)mean_mv << 16);
    m->reserved[1] = accepted ? 1u : 0u;
    m->reserved[2] = BOOTLOG_ZERO_TAG;
}
/* Zero used (mV) and window mean (mV) from a ZERO record. */
static inline uint16_t bootlog_zero_mv(const bootlog_meta_t *m)   { return (uint16_t)(m->reserved[0] & 0xFFFFu); }
static inline uint16_t bootlog_zero_mean(const bootlog_meta_t *m) { return (uint16_t)(m->reserved[0] >> 16); }

/* Compaction plan: keep the latest attempt record, the last completed one,
 * and the newest ZERO_META_KEEP (6) ZERO records, in their original order.
 * keep[i] = 1 for slots to re-program; returns how many. */
static inline unsigned bootlog_meta_keep_plan(const bootlog_meta_t *slots, unsigned n, unsigned char *keep) {
    int latest = -1, completed = -1; unsigned i, k = 0, z = 0;
    for (i = 0; i < n; i++) {
        keep[i] = 0;
        if (slots[i].magic == BOOTLOG_META_MAGIC && slots[i].version == BOOTLOG_META_VERSION
            && slots[i].reason >= 1u && slots[i].reason <= 7u && slots[i].boot_count > 0) {
            latest = (int)i;
            if (slots[i].reason != 1u) completed = (int)i;
        }
    }
    for (i = n; i-- > 0; ) if (bootlog_zero_valid(&slots[i]) && z < ZERO_META_KEEP) { keep[i] = 1; z++; }
    if (latest >= 0) keep[latest] = 1;
    if (completed >= 0) keep[completed] = 1;
    for (i = 0; i < n; i++) k += keep[i];
    return k;
}

_Static_assert(sizeof(bootlog_rec_t) == 16, "bootlog_rec_t packing");
_Static_assert(sizeof(bootlog_hdr_t) == 28, "bootlog_hdr_t packing");
_Static_assert(sizeof(bootlog_meta_t) == 48, "bootlog_meta_t packing");
_Static_assert(BOOTLOG_HDR_SIZE >= sizeof(bootlog_hdr_t), "header page too small");
_Static_assert(sizeof(bootlog_meta_t) <= BOOTLOG_META_PAGE, "meta does not fit a page");

#define BOOTLOG_REC_BYTES      (BOOTLOG_MAX_REC * sizeof(bootlog_rec_t))
#define BOOTLOG_PAYLOAD_BYTES  (BOOTLOG_HDR_SIZE + BOOTLOG_REC_BYTES)
#define BOOTLOG_PAGES          ((BOOTLOG_PAYLOAD_BYTES + 255u) / 256u)
_Static_assert(BOOTLOG_PAYLOAD_BYTES <= 4096, "log must fit in one 4 KB sector");

/* CSV columns of the 16-byte part of a ring record, as printed by the dump. */
#define BOOTLOG_CSV_HEADER \
    "t_ms,adc_mv,x13,amps,i_raw,i_raw_hex,latch,b14_slot0,b14_slot0_hex,eng,slew"

/* Old boot-log bank header check. */
static inline int bootlog_hdr_valid(const bootlog_hdr_t *h) {
    return h && h->magic == BOOTLOG_MAGIC && h->version == BOOTLOG_VERSION
        && h->count > 0 && h->count <= BOOTLOG_MAX_REC;
}

/* Old boot log: pick the valid bank with the highest sequence number. */
static inline const bootlog_hdr_t *bootlog_pick(const bootlog_hdr_t *a,
                                                const bootlog_hdr_t *b) {
    int va = bootlog_hdr_valid(a);
    int vb = bootlog_hdr_valid(b);
    if (va && vb) return (a->seq >= b->seq) ? a : b;
    if (va) return a;
    if (vb) return b;
    return NULL;
}

/* Name of a current state (BTH5_IST_* in current_sensor.h), for the CSV. */
static inline const char *bootlog_latch_str(uint8_t latch) {
    if (latch == 0) return "HOLD";
    if (latch == 1) return "LIVE";
    if (latch == 2) return "DB";    /* inside the deadband, sent 0x1FF */
    if (latch == 3) return "DIS";   /* live discharge */
    if (latch == 4) return "CHG";   /* live charge (RUN only) */
    if (latch == 5) return "ZERO";  /* auto-zero window, sent 0x1FF */
    if (latch == 6) return "CHGH";  /* charge measured before RUN, held at 0x1FF */
    return "NA";
}

/* 1 if m is a valid attempt record (reason 1..7). */
static inline int bootlog_meta_valid(const bootlog_meta_t *m) {
    return m && m->magic == BOOTLOG_META_MAGIC && m->version == BOOTLOG_META_VERSION
        && m->reason >= BOOTLOG_REASON_BOOT && m->reason <= BOOTLOG_REASON_FAIL_PROGRAM
        && m->boot_count > 0;
}

/* Latest valid slot in an append-only array (highest index). */
static inline const bootlog_meta_t *bootlog_pick_latest_meta(const bootlog_meta_t *slots,
                                                            unsigned n) {
    const bootlog_meta_t *last = 0;
    unsigned i;
    if (!slots) return 0;
    for (i = 0; i < n; i++) {
        if (bootlog_meta_valid(&slots[i])) last = &slots[i];
    }
    return last;
}

/* Most recent record that is not a pending boot marker. */
static inline const bootlog_meta_t *bootlog_pick_last_completed(const bootlog_meta_t *slots,
                                                               unsigned n) {
    const bootlog_meta_t *last = 0;
    unsigned i;
    if (!slots) return 0;
    for (i = 0; i < n; i++) {
        if (bootlog_meta_valid(&slots[i]) && slots[i].reason != BOOTLOG_REASON_BOOT)
            last = &slots[i];
    }
    return last;
}

/* Text for an attempt reason, for the dump header. */
static inline const char *bootlog_reason_str(uint16_t reason) {
    switch (reason) {
    case BOOTLOG_REASON_BOOT:         return "this boot started, capture not finished";
    case BOOTLOG_REASON_WRITTEN:      return "written";
    case BOOTLOG_REASON_SKIP_HOST:    return "skipped, USB host mounted or CDC connected";
    case BOOTLOG_REASON_SKIP_EMPTY:   return "skipped, no samples";
    case BOOTLOG_REASON_SKIP_OVERLAP: return "skipped, program overlaps log flash";
    case BOOTLOG_REASON_FAIL_ERASE:   return "failed, sector erase";
    case BOOTLOG_REASON_FAIL_PROGRAM: return "failed, page program";
    default:                          return "unknown";
    }
}

/* Name of an engine-detector state (BTH5_ENG_*). */
static inline const char *bootlog_eng_str(uint8_t e) {
    static const char *n[4] = { "PRE", "CRANK", "SETTLE", "RUN" };
    return n[e & 3];
}
/* Format the 16-byte part of a record as a CSV row (BOOTLOG_CSV_HEADER).
 * Returns snprintf's result. */
static inline int bootlog_csv_row(char *dst, size_t dst_sz, const bootlog_rec_t *r) {
    if (!dst || !r || dst_sz < 8) return -1;
    /* latch = current state [3:0] | engine state [5:4] | slewed [6] */
    return snprintf(dst, dst_sz, "%u,%u,%u,%.2f,%u,0x%03X,%s,%u,0x%02X,%s,%u",
                    (unsigned)r->t_ms, (unsigned)r->adc_mv, (unsigned)r->adc_counts,
                    (double)r->amps_cA / 100.0, (unsigned)r->i_raw,
                    (unsigned)r->i_raw, bootlog_latch_str((uint8_t)(r->latch & 0x0F)),
                    (unsigned)r->b14_slot0, (unsigned)r->b14_slot0,
                    bootlog_eng_str((uint8_t)(r->latch >> 4)), (unsigned)((r->latch >> 6) & 1));
}

#ifdef PICO_ON_DEVICE
/* Pico-only API, implemented in flash_log.c. */
#include "bth_frames.h"
/* Core 0 at start-up: boot counter, meta record, find the ring write point. */
void bootlog_init(void);
/* Print the USB help line (build tag and dump keys). */
void bootlog_hint(void);
/* Core 0 after each burst: queue this burst's log record. */
void bootlog_after_burst(uint8_t b14_slot0);
/* Core 0 at the start of each burst: capture the block/temperature log
 * fields from the snapshot being sent. */
void bootlog_vlog_capture(const bth5_pub_t *pub, uint32_t now_ms);
/* Core 0 in the gap after each burst: USB commands, at most one flash
 * operation. During a sector erase it sends further bursts itself and
 * updates *gap_start_us and *idx. */
void bootlog_idle(uint64_t *gap_start_us, uint8_t *idx, const bth5_pub_t *pub);
/* 1 if a USB host has enumerated us (5 V alone does not count). */
int  bootlog_host_connected(void);
#endif

#ifdef __cplusplus
}
#endif
#endif
