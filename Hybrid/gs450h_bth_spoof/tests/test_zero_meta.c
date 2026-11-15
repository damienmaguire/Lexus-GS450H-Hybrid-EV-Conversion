/* test_zero_meta.c -- auto-zero (ZERO) meta records in a simulated 16-slot meta sector.
 * Car boots write BOOT + ZERO; laptop boots write BOOT + SKIP_HOST (no zero, host connected).
 * Compaction (bootlog_meta_keep_plan) runs at boot when < 3 slots are free, as in bootlog_init().
 * Checks: boot_count continuity, old pickers never return a ZERO record, the newest car zero
 * records always survive, and the previous car boot's zero is always readable at dump time. */
#include <stdio.h>
#include <string.h>
#include "flash_log.h"
#define N BOOTLOG_META_SLOTS
static bootlog_meta_t sec[N];
static int fails;
#define CHECK(c, m) do { if (!(c)) { printf("FAIL %s (boot %u)\n", m, (unsigned)boot); fails++; } } while (0)
static int nfree(void) { int k = 0; for (unsigned i = 0; i < N; i++) if (sec[i].magic == 0xFFFFFFFFu) k++; return k; }
static void put(const bootlog_meta_t *m) { for (unsigned i = 0; i < N; i++) if (sec[i].magic == 0xFFFFFFFFu) { sec[i] = *m; return; } }
static void compact(void) {
    bootlog_meta_t copy[N]; unsigned char keep[N]; unsigned o = 0;
    memcpy(copy, sec, sizeof sec); bootlog_meta_keep_plan(copy, N, keep);
    memset(sec, 0xFF, sizeof sec);
    for (unsigned i = 0; i < N; i++) if (keep[i]) sec[o++] = copy[i];
}
static void attempt(uint32_t boot, uint16_t reason) {
    bootlog_meta_t m; memset(&m, 0, sizeof m);
    m.magic = BOOTLOG_META_MAGIC; m.version = BOOTLOG_META_VERSION; m.reason = reason; m.boot_count = boot; put(&m);
}
int main(void) {
    uint32_t boot = 0, last_car = 0; unsigned compactions = 0;
    memset(sec, 0xFF, sizeof sec);
    for (int i = 0; i < 200; i++) {
        const bootlog_meta_t *lat = bootlog_pick_latest_meta(sec, N);
        uint32_t expect = boot + 1;
        boot = lat ? lat->boot_count + 1 : 1;
        CHECK(boot == expect, "boot_count continuity");
        if (nfree() < 3) { compact(); compactions++; }
        CHECK(nfree() >= 3, "free slots after compaction");
        attempt(boot, BOOTLOG_REASON_BOOT);
        int car = (i % 3) != 2;                 /* 2 car boots, 1 laptop boot */
        if (car) {
            bootlog_meta_t z; bootlog_zero_fill(&z, boot, 583, (uint16_t)(2560 + boot % 7), (uint16_t)(2560 + boot % 7), 5, 1);
            put(&z); last_car = boot;
        } else {
            attempt(boot, BOOTLOG_REASON_SKIP_HOST);
            /* laptop dump: previous car boot's zero must be present */
            int seen = 0;
            for (unsigned k = 0; k < N; k++) if (bootlog_zero_valid(&sec[k]) && sec[k].boot_count == last_car) {
                seen = 1; CHECK(bootlog_zero_mv(&sec[k]) == 2560 + last_car % 7 && sec[k].rec_count == 5 && sec[k].t_ms == 583, "zero fields"); }
            CHECK(seen, "previous car boot zero readable at dump");
        }
        lat = bootlog_pick_latest_meta(sec, N);
        const bootlog_meta_t *cmp = bootlog_pick_last_completed(sec, N);
        CHECK(lat && lat->reason != BOOTLOG_REASON_ZERO, "latest picker skips ZERO");
        CHECK(!cmp || cmp->reason != BOOTLOG_REASON_ZERO, "completed picker skips ZERO");
    }
    unsigned nz = 0; for (unsigned k = 0; k < N; k++) nz += bootlog_zero_valid(&sec[k]);
    printf("zero_meta: 200 boots, %u compactions, %u zero records held (keep %u), fails=%d\n", compactions, nz, (unsigned)ZERO_META_KEEP, fails);
    printf(fails ? "FAIL\n" : "PASS\n");
    return fails != 0;
}
