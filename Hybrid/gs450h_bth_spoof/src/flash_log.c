/* flash_log.c -- the flash log, Pico side: boot counter and meta records,
 * the rolling ring log of every burst, saving each boot's auto-zero, and
 * the USB dump commands.
 *
 * All flash writes happen on core 0 in the 8.18 ms gap after a burst, at
 * most one operation per gap:
 *   - a 256-byte page program (max 3 ms) only if enough of the gap is left;
 *   - a 4 KB sector erase (up to 400 ms) runs from RAM while this file keeps
 *     sending pre-built bursts on time (bootlog_erase_cb).
 * Nothing is written in the first 3 s of a boot or once a USB host is
 * connected, so a laptop session never overwrites the car's log.
 *
 * USB serial keys: d = previous boot (the car run), t = its last 100 rows,
 * a = whole ring. Column details: DUMP.md in the project notes.
 *
 * Function and variable names keep their "bootlog_" / "ring_" prefixes from
 * development so they match the log format notes.
 */
#include "config.h"
#include "flash_log.h"

#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/flash.h"
#include "pico/bootrom.h"
#include "pico/time.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/structs/qmi.h"
#include "hardware/timer.h"
#include "tusb.h"

#include "bth_frames.h"
#include "oem_template.h"

/* Same UART, LED and VBUS pins as main.c. */
#ifndef UART_ID
#define UART_ID uart0
#endif
#ifndef LED_PIN
#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN PICO_DEFAULT_LED_PIN
#else
#define LED_PIN 25
#endif
#endif
#ifndef PICO_VBUS_PIN
#define PICO_VBUS_PIN 24
#endif

/* Meta sector = third sector from the top of flash (layout in flash_log.h). */
#define BOOTLOG_META_OFF   (PICO_FLASH_SIZE_BYTES - 3u * FLASH_SECTOR_SIZE)

extern char __flash_binary_end;   /* end of the program image (linker symbol) */

/* ---- state ---- */
static uint16_t g_n;             /* old boot-log record count, always 0 here */
static uint8_t  g_usb_hinted;    /* help line printed since USB connected */
static uint8_t  g_overlap;       /* 1 = program image reaches the log area: logging off */
static uint32_t g_seq_next;      /* old boot-log sequence, always 0 here */
static uint32_t g_erase_us;      /* old boot-log timings, always 0 here */
static uint32_t g_prog_us;
static uint32_t g_boot_count;    /* this boot's number (last meta boot_count + 1) */
static uint8_t  g_meta_pending; /* 1 = need to program a last-attempt meta page */
static uint8_t  g_meta_done;
static bootlog_meta_t g_attempt; /* staged last-attempt record */
extern volatile int g_flash_core_ready;

/* Values from main.c, logged per burst. */
extern uint16_t g_adc_mv;
extern uint16_t g_adc_counts;
extern float    g_amps;
extern uint16_t g_i_raw;
extern uint16_t g_x13_sent;
/* Build tag printed in the USB help line and the dump header. Kept as the
 * development name of the car-tested build so dumps can be matched to it. */
#define BTH_BUILD_TAG "i18qnb_aux_db05_tb5i (i18qnb_aux_db05_tb5 + intake air slot-3 A5 = spread-limited module min (= TB2), beta law, 5-40 C, same slew/fallback to 605)"
extern bth5_eng_t g_eng;
extern int g_slewed;
extern uint16_t g_zero_mv, g_zero_avg_mv;
extern uint8_t g_zero_done;
extern uint32_t g_zero_t_ms;
extern int g_istate;

/* ---- timing helpers that run from RAM ----
 * During a sector erase the flash cannot be read, so everything used then
 * must be in RAM (__not_in_flash_func). The SDK time functions live in
 * flash, hence these copies. */
/* 64-bit microsecond timer, read directly from the timer registers. */
static uint64_t __not_in_flash_func(bootlog_now_us)(void) {
    uint32_t hi = timer_hw->timerawh;
    uint32_t lo;
    do {
        lo = timer_hw->timerawl;
        uint32_t next_hi = timer_hw->timerawh;
        if (hi == next_hi) break;
        hi = next_hi;
    } while (1);
    return ((uint64_t)hi << 32) | lo;
}

/* Busy-wait until the timer reaches t_us. */
static void __not_in_flash_func(bootlog_wait_us)(uint64_t t_us) {
    while (bootlog_now_us() < t_us)
        tight_loop_contents();
}

/* Send one 16-byte frame at 1020 us per byte (RAM copy of main.c
 * send_burst_bytes()). */
static void __not_in_flash_func(bootlog_send_frame)(const uint8_t *f) {
    uint64_t t_next = bootlog_now_us();
    for (int b = 0; b < 16; b++) {
        uart_putc_raw(UART_ID, (char)f[b]);
        t_next += BYTE_PERIOD_US;
        bootlog_wait_us(t_next);
    }
    uart_tx_wait_blocking(UART_ID);
}

/* ---- low-level flash, XIP already off, all in RAM ---- */
static void __not_in_flash_func(bootlog_qmi_cmd)(const uint8_t *tx, uint8_t *rx, size_t n) {
    hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
    hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS0N_BITS);
    size_t tx_l = n, rx_l = n;
    while (tx_l || rx_l) {
        uint32_t flags = qmi_hw->direct_csr;
        if (!(flags & QMI_DIRECT_CSR_TXFULL_BITS) && tx_l) {
            qmi_hw->direct_tx = *tx++;
            --tx_l;
        }
        if (!(flags & QMI_DIRECT_CSR_RXEMPTY_BITS) && rx_l) {
            *rx++ = (uint8_t)qmi_hw->direct_rx;
            --rx_l;
        }
    }
    hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS0N_BITS);
    hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
}

/* Flash WRITE ENABLE (0x06). */
static void __not_in_flash_func(bootlog_wren)(void) {
    uint8_t tx[1] = { 0x06 }, rx[1];
    bootlog_qmi_cmd(tx, rx, 1);
}

/* Flash READ STATUS (0x05); bit 0 = busy. */
static uint8_t __not_in_flash_func(bootlog_rdsr)(void) {
    uint8_t tx[2] = { 0x05, 0x00 }, rx[2] = { 0, 0 };
    bootlog_qmi_cmd(tx, rx, 2);
    return rx[1];
}

/* Start a 4 KB SECTOR ERASE (0x20) and return without waiting. */
static void __not_in_flash_func(bootlog_se_nowait)(uint32_t offs) {
    uint8_t tx[4] = { 0x20, (uint8_t)(offs >> 16), (uint8_t)(offs >> 8), (uint8_t)offs };
    uint8_t rx[4];
    bootlog_wren();
    bootlog_qmi_cmd(tx, rx, 4);
}

/* Arguments for bootlog_erase_cb(). */
typedef struct {
    uint64_t *gap_start_us;
    uint8_t  *idx;
    const bth5_pub_t *pub;
    uint32_t flash_offs;
    uint32_t *erase_us;
} bootlog_erase_t;

/* Erase one ring sector while keeping BTH running.
 * Called through flash_safe_execute (core 1 parked, interrupts off).
 * First, with flash still readable, it builds BOOTLOG_ERASE_BURST_BUDGET (8)
 * bursts from the current snapshot. Burst 0 carries the X13 the caller
 * set; later bursts carry X13 = the current, since the burst before them
 * had all 12 current fields equal to it. Then it starts the erase and,
 * until the chip is no longer busy, sends one of those bursts each time the
 * normal 8.18 ms gap runs out, updating *idx and *gap_start_us. */
static void __not_in_flash_func(bootlog_erase_cb)(void *param) {
    bootlog_erase_t *e = (bootlog_erase_t *)param;
    uint8_t frames[BOOTLOG_ERASE_BURST_BUDGET][6][16];
    uint8_t idx0 = *e->idx;
    unsigned n_built = BOOTLOG_ERASE_BURST_BUDGET;
    unsigned played = 0;
    uint32_t t0, t1;
    unsigned i, fr;

    /* XIP still on: pre-build enough bursts to cover max erase. */
    /* Burst 0 carries the X13 the caller set (mean of the burst just sent);
     * later bursts follow bursts whose 12 fields are all pub->i_raw. */
    {
        bth5_pub_t pb = *e->pub;
        for (i = 0; i < n_built; i++) {
            if (i == 1) pb.i_raw13 = pb.i_raw;
            for (fr = 0; fr < 6; fr++) {
                uint8_t id = (uint8_t)((idx0 + i * 6 + fr) % 30);
                bth5_buildFrame(frames[i][fr], id, &pb);
            }
        }
    }

    rom_connect_internal_flash_fn connect =
        (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn exit_xip =
        (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_flush_cache_fn flush =
        (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    if (!connect || !exit_xip || !flush) return;

    connect();
    exit_xip();
    t0 = (uint32_t)bootlog_now_us();
    bootlog_se_nowait(e->flash_offs);

    /* Keep the 106.1 ms cadence while the chip erases. */
    while (bootlog_rdsr() & 0x01) {
        uint64_t deadline = *e->gap_start_us + BURST_GAP_US;
        uint64_t now = bootlog_now_us();
        if (now >= deadline) {
            if (played < n_built) {
                for (fr = 0; fr < 6; fr++)
                    bootlog_send_frame(frames[played][fr]);
                *e->idx = (uint8_t)((idx0 + (played + 1) * 6) % 30);
                played++;
                gpio_xor_mask(1u << LED_PIN);
                *e->gap_start_us = bootlog_now_us();
            } else {
                /* Should not happen (8 * 106 ms > 400 ms max). Keep polling. */
                tight_loop_contents();
            }
        } else {
            tight_loop_contents();
        }
    }
    t1 = (uint32_t)bootlog_now_us();
    if (e->erase_us) *e->erase_us = t1 - t0;

    flush();
    flash_start_xip();
}

/* Meta slot i, read through XIP. */
static const bootlog_meta_t *meta_slot(unsigned i) {
    return (const bootlog_meta_t *)(XIP_BASE + BOOTLOG_META_OFF + i * BOOTLOG_META_PAGE);
}

/* First erased meta slot, or -1 if the sector is full. */
static int meta_next_free(void) {
    unsigned i;
    for (i = 0; i < BOOTLOG_META_SLOTS; i++) {
        const bootlog_meta_t *m = meta_slot(i);
        if (m->magic == 0xFFFFFFFFu) return (int)i;
    }
    return -1;
}

/* Newest valid attempt record, or NULL. */
static const bootlog_meta_t *meta_latest(void) {
    const bootlog_meta_t *last = 0;
    unsigned i;
    for (i = 0; i < BOOTLOG_META_SLOTS; i++) {
        if (bootlog_meta_valid(meta_slot(i))) last = meta_slot(i);
    }
    return last;
}

/* flash_safe_execute callback: program g_attempt into meta slot `param`. */
static void bootlog_meta_page_cb(void *param) {
    unsigned slot = (unsigned)(uintptr_t)param;
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    memcpy(page, &g_attempt, sizeof g_attempt);
    flash_range_program(BOOTLOG_META_OFF + slot * BOOTLOG_META_PAGE, page, FLASH_PAGE_SIZE);
}

/* flash_safe_execute callback: erase the meta sector. */
static void bootlog_meta_erase_cb(void *param) {
    (void)param;
    flash_range_erase(BOOTLOG_META_OFF, FLASH_SECTOR_SIZE);
}

/* Write g_attempt to the next free meta slot now. */
static int bootlog_program_meta_now(void) {
    int slot = meta_next_free();
    int rc;
    if (slot < 0) return PICO_ERROR_NO_DATA;
    rc = flash_safe_execute(bootlog_meta_page_cb, (void *)(uintptr_t)slot, 200);
    return rc;
}

/* Arguments for bootlog_meta_rec_cb(). */
typedef struct { unsigned slot; const bootlog_meta_t *rec; } meta_prog_t;
/* flash_safe_execute callback: program one meta record into a slot. */
static void bootlog_meta_rec_cb(void *param) {
    const meta_prog_t *mp = (const meta_prog_t *)param;
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    memcpy(page, mp->rec, sizeof *mp->rec);
    flash_range_program(BOOTLOG_META_OFF + mp->slot * BOOTLOG_META_PAGE, page, FLASH_PAGE_SIZE);
}

/* Make sure at least `need` meta slots are free. If not, erase the sector
 * and write back only what bootlog_meta_keep_plan() keeps (latest attempt,
 * last completed attempt, newest 6 ZERO records). Only called at boot,
 * before BTH starts, because the erase stalls everything. */
static int bootlog_meta_ensure_slots(int need) {
    int free_n = 0, i, rc;
    for (i = 0; i < (int)BOOTLOG_META_SLOTS; i++) {
        if (meta_slot((unsigned)i)->magic == 0xFFFFFFFFu) free_n++;
    }
    if (free_n >= need) return PICO_OK;
    {   /* keep latest + last completed attempt + newest ZERO_META_KEEP zero records */
        bootlog_meta_t copy[BOOTLOG_META_SLOTS];
        unsigned char keep[BOOTLOG_META_SLOTS];
        unsigned j, out = 0;
        for (j = 0; j < BOOTLOG_META_SLOTS; j++) copy[j] = *meta_slot(j);
        (void)bootlog_meta_keep_plan(copy, BOOTLOG_META_SLOTS, keep);
        rc = flash_safe_execute(bootlog_meta_erase_cb, 0, 200);
        if (rc != PICO_OK) return rc;
        for (j = 0; j < BOOTLOG_META_SLOTS; j++) {
            meta_prog_t mp;
            if (!keep[j]) continue;
            mp.slot = out++; mp.rec = &copy[j];
            rc = flash_safe_execute(bootlog_meta_rec_cb, &mp, 200);
            if (rc != PICO_OK) return rc;
        }
        return PICO_OK;
    }
}

/* Fill g_attempt for this boot with the given reason. */
static void bootlog_fill_attempt(uint16_t reason, int flash_rc) {
    memset(&g_attempt, 0, sizeof g_attempt);
    g_attempt.magic = BOOTLOG_META_MAGIC;
    g_attempt.version = BOOTLOG_META_VERSION;
    g_attempt.reason = reason;
    g_attempt.boot_count = g_boot_count;
    g_attempt.t_ms = to_ms_since_boot(get_absolute_time());
    g_attempt.rec_count = g_n;
    g_attempt.vbus = (uint8_t)(gpio_get(PICO_VBUS_PIN) ? 1 : 0);
    g_attempt.host = (uint8_t)(bootlog_host_connected() ? 1 : 0);
    g_attempt.flash_rc = flash_rc;
    g_attempt.erase_us = g_erase_us;
    g_attempt.prog_us = g_prog_us;
    g_attempt.log_seq = (reason == BOOTLOG_REASON_WRITTEN) ? g_seq_next : 0;
}

/* Fill g_attempt and mark it to be written in a later gap. */
static void bootlog_stage_attempt(uint16_t reason, int flash_rc) {
    bootlog_fill_attempt(reason, flash_rc);
    g_meta_pending = 1;
    g_meta_done = 0;
}

/* 1 if a USB host has enumerated the Pico. */
int bootlog_host_connected(void) {
    /* A USB *host* has enumerated us. 5 V on VBUS alone (car supply) must not count. */
    return tud_mounted() || tud_cdc_connected();
}

/* Print one attempt record in the dump header. */
static void bootlog_print_attempt(const char *label, const bootlog_meta_t *m) {
    if (!m) {
        printf("# %s: none\n", label);
        return;
    }
    printf("# %s: %s  (boot=%u t_ms=%u recs=%u seq=%u erase_us=%u prog_us=%u vbus=%u host=%u rc=%d)\n",
           label, bootlog_reason_str(m->reason),
           (unsigned)m->boot_count, (unsigned)m->t_ms, (unsigned)m->rec_count,
           (unsigned)m->log_seq, (unsigned)m->erase_us, (unsigned)m->prog_us,
           (unsigned)m->vbus, (unsigned)m->host, (int)m->flash_rc);
}

/* Write a staged attempt record if enough of the gap is left. */
static int bootlog_try_write_meta(uint64_t remain_us) {
    int rc;
    if (!g_meta_pending || g_meta_done) return PICO_OK;
    if (remain_us <= (BOOTLOG_PAGE_PROG_MAX_US + 1500u)) return PICO_ERROR_PRECONDITION_NOT_MET;
    rc = bootlog_program_meta_now();
    if (rc == PICO_OK) {
        g_meta_pending = 0;
        g_meta_done = 1;
    }
    return rc;
}

/* ===================== rolling ring log ===================== */
#include "ring_log.h"
#include "bmw_pack.h"

/* The 32-byte-record ring sits 16 sectors below the meta sector. Those 16
 * sectors belong to the older 16-byte-record ring, so either format can be
 * flashed back without one overwriting the other. */
#define RING_FLASH_OFF (BOOTLOG_META_OFF - (16u + RING_SECTORS) * FLASH_SECTOR_SIZE)
_Static_assert(RING_SPS == 128 && RING_SPP == 8, "ring geometry");

/* ---- ring log state ---- */
static ring_fifo_t g_fifo;
static uint32_t g_wr;               /* next ring slot to write */
static int      g_erase_sec = -1;   /* sector to erase before more writes */
static uint8_t  g_ring_state;       /* 0 holdoff, 1 writing, 2 host (stopped), 3 flash fail */
static uint8_t  g_zero_marked;
static uint16_t g_boot_id;
static uint32_t g_ring_written, g_ring_erases, g_ring_last_erase_us;
static uint32_t g_ring_prog_off;
static uint8_t  g_ring_page[256];

static uint32_t g_erase_deferred;
static uint8_t  g_erase_mark;      /* an erase ran in the gap before the next logged burst */
static float    g_amps_prev_gap;
/* Ask ring_log.h whether the pending erase should wait. */
static int ring_erase_defer(void) {
    int eng_busy = 0;
    eng_busy = (g_eng.st == BTH5_ENG_CRANK || g_eng.st == BTH5_ENG_SETTLE);
    return ring_erase_should_defer(g_amps, g_amps_prev_gap, eng_busy, g_fifo.n);
}

/* Start of the ring, read through XIP. */
static const ring_rec_t *ring_flash(void) {
    return (const ring_rec_t *)(XIP_BASE + RING_FLASH_OFF);
}

/* flash_safe_execute callback: program g_ring_page at g_ring_prog_off. */
static void ring_program_cb(void *param) {
    (void)param;
    flash_range_program(g_ring_prog_off, g_ring_page, FLASH_PAGE_SIZE);
}

/* Text for g_ring_state. */
static const char *ring_state_str(void) {
    switch (g_ring_state) {
    case 0: return "holdoff (no flash writes before 3 s)";
    case 1: return "writing";
    case 2: return "stopped, USB host connected (car log protected)";
    default: return "stopped, flash error";
    }
}

/* USB help line. */
void bootlog_hint(void) {
    printf("BTH i16dring: rolling log. d = previous boot (car run), t = its last 100 rows, a = whole ring.\n");
    printf("  build: %s\n", BTH_BUILD_TAG);
    printf("  Nothing is written while a USB host is connected, or in the first 3 s of a boot.\n");
}

/* Print an auto-zero marker record from the ring. */
static void ring_print_zero(const bootlog_rec_t *r) {
    unsigned z = r->adc_mv, m = r->adc_counts;
    printf("# boot=%u t_ms=%u zero_mv=%u source=%s window_mean_mv=%u\n", (unsigned)r->pad,
           (unsigned)r->t_ms, z,
           (m >= (unsigned)AUTO_ZERO_MIN_MV && m <= (unsigned)AUTO_ZERO_MAX_MV) ? "auto" : "fallback", m);
}

/* ZERO meta record for this boot. */
static uint8_t g_zmeta_state;   /* 0 waiting, 1 staged, 2 written, 3 skipped (host), 4 no slot / fail */
static bootlog_meta_t g_zmeta;
/* Text for g_zmeta_state. */
static const char *zero_meta_state_str(uint8_t st) {
    switch (st) {
    case 0: return "pending (zero not done or < 3 s)";
    case 1: return "staged";
    case 2: return "written";
    case 3: return "skipped (USB host connected)";
    default: return "not written (no free slot / flash error)";
    }
}
/* Dump header: deadband settings and every ZERO record in the meta sector. */
static void zero_meta_print(void) {
    unsigned k, n = 0;
    for (k = 0; k < BOOTLOG_META_SLOTS; k++) if (bootlog_zero_valid(meta_slot(k))) n++;
    printf("# current deadband: RUN |I| < %.1f A -> 0x1FF; pre-RUN discharge live from %.1f A; pre-RUN charge always held 0x1FF\n",
           (double)(RUN_DB_DA * 0.1f), (double)(CURRENT_DIS_DB_DA * 0.1f));
    printf("# zero meta (meta sector, not overwritten by ring wrap): %u record(s), oldest first\n", n);
    for (k = 0; k < BOOTLOG_META_SLOTS; k++) {
        const bootlog_meta_t *m = meta_slot(k);
        if (!bootlog_zero_valid(m)) continue;
        printf("# zero_meta boot=%u t_ms=%u zero_mv=%u window_mean_mv=%u n=%u source=%s\n",
               (unsigned)ring_boot_id(m->boot_count), (unsigned)m->t_ms, (unsigned)bootlog_zero_mv(m),
               (unsigned)bootlog_zero_mean(m), (unsigned)m->rec_count, m->reserved[1] ? "auto" : "fallback");
    }
    printf("# zero_meta this boot=%u: zero_mv=%u window_mean_mv=%u record %s\n", (unsigned)g_boot_id,
           (unsigned)g_zero_mv, (unsigned)g_zero_avg_mv, zero_meta_state_str(g_zmeta_state));
}

/* Dump the ring over USB. mode: 'd' = the newest boot that is not this
 * one (the car run when dumping from a laptop), 't' = its last 100 rows,
 * 'a' = everything. Prints a header with the settings, then CSV rows. */
static void ring_dump(char mode) {
    bootlog_meta_t slots[BOOTLOG_META_SLOTS];
    const bootlog_meta_t *latest, *completed;
    const ring_rec_t *fl = ring_flash();
    uint32_t wr = g_wr;
    uint16_t target;
    unsigned k, rows = 0, skip = 0, seen = 0;
    int run_boot = -1; unsigned run_n = 0; uint32_t run_t0 = 0, run_t1 = 0;
    char line[224];

    for (k = 0; k < BOOTLOG_META_SLOTS; k++) slots[k] = *meta_slot(k);
    latest = bootlog_pick_latest_meta(slots, BOOTLOG_META_SLOTS);
    completed = bootlog_pick_last_completed(slots, BOOTLOG_META_SLOTS);
    printf("# build=%s\n", BTH_BUILD_TAG);
    printf("# i18vlog rows: 32 B, ring %u sectors; ages in ms at burst build (65535 = no module)\n", (unsigned)RING_SECTORS);
    printf("# cell guard: mask 0x%04X, accept %u-%u mV, lone jump > %.0f mV rejected, hold %u ms then module avg; cell_rej = cumulative rejects, cell_flag = status-bit word seen in this burst\n",
           (unsigned)GUARD_RAW_MASK, (unsigned)GUARD_MV_MIN, (unsigned)GUARD_MV_MAX, (double)GUARD_JUMP_MV, (unsigned)GUARD_HOLD_MS);
    printf("# TB1-4 from module temps (byte-40 C): TB1=avg TB2=max(min,avg-%.0f) TB3=avg TB4=min(max,avg+%.0f), clamp %.0f-%.0f C (raw %u-%u), slew %.2f C/s, stale %u ms -> slew to template 666/673/668/650; law ln(raw/(%.0f-raw)) = %.6f + %.4f/T_K; mod_tmin_c/mod_tmax_c columns replace blk_age_ms\n",
           (double)TB_SPREAD_MAX_C, (double)TB_SPREAD_MAX_C, (double)TEMP_SEND_MIN_C, (double)TEMP_SEND_MAX_C,
           (unsigned)bth5_encodeTemp(TEMP_SEND_MAX_C), (unsigned)bth5_encodeTemp(TEMP_SEND_MIN_C),
           (double)TB_SLEW_C_PER_S, (unsigned)TB_STALE_MS, (double)TEMP_ADC_N, (double)TEMP_BETA_LNK, (double)TEMP_BETA_K);
    printf("# intake air (slot 3 A5) = TB2 target (max(module min, avg-%.0f)), same law/clamp/slew; template 605 on fallback\n", (double)TB_SPREAD_MAX_C);
    printf("# i18q: x13 column = slot-0 X13 sent (floor mean of previous burst I2/I8); b14 column = slot-0 b14 as sent, low nibble = floor(sum(b0..b14)/257); erase deferred while |I| > %.0f A or CRANK/SETTLE: deferred %u gaps\n",
           (double)ERASE_DEFER_A, (unsigned)g_erase_deferred);
    printf("# i16dring boot_count=%u this_boot_id=%u ring=%u slots @0x%06X wr=%u state=%s written=%u erases=%u dropped=%u fifo=%u\n",
           (unsigned)g_boot_count, (unsigned)g_boot_id, (unsigned)RING_SLOTS, (unsigned)RING_FLASH_OFF,
           (unsigned)wr, ring_state_str(), (unsigned)g_ring_written, (unsigned)g_ring_erases,
           (unsigned)g_fifo.dropped, (unsigned)g_fifo.n);
    if (latest && latest->reason == BOOTLOG_REASON_BOOT) bootlog_print_attempt("this boot", latest);
    bootlog_print_attempt("last attempt", completed ? completed : latest);
    zero_meta_print();

    /* pass 1: boot summary, oldest -> newest */
    for (k = 0; k <= RING_SLOTS; k++) {
        const bootlog_rec_t *r = (k < RING_SLOTS) ? RR_B(&fl[(wr + k) % RING_SLOTS]) : 0;
        int valid = r && ring_slot_valid(&fl[(wr + k) % RING_SLOTS]);
        if ((!r || (valid && (int)r->pad != run_boot)) && run_boot >= 0) {
            printf("# ring boot=%d rows=%u t_ms=%u..%u\n", run_boot, run_n, (unsigned)run_t0, (unsigned)run_t1);
            run_boot = -1;
        }
        if (!valid) continue;
        if (run_boot < 0) { run_boot = r->pad; run_n = 0; run_t0 = r->t_ms; }
        if (r->latch != RING_MARK_ZERO) run_n++;
        run_t1 = r->t_ms;
    }
    target = ring_pick_target(fl, wr, g_boot_id);
    if (mode == 't') {
        for (k = 0; k < RING_SLOTS; k++) {
            const bootlog_rec_t *r = RR_B(&fl[(wr + k) % RING_SLOTS]);
            if (ring_slot_valid(&fl[(wr + k) % RING_SLOTS]) && r->pad == target && r->latch != RING_MARK_ZERO) seen++;
        }
        skip = seen > RING_TAIL_N ? seen - RING_TAIL_N : 0;
    }
    if (mode == 'a') printf("# dump: all boots in ring\n");
    else printf("# dump: boot=%u%s\n", (unsigned)target, mode == 't' ? " (last 100 rows)" : "");
    printf("%s%s\n", BOOTLOG_CSV_HEADER, RING_VLOG_CSV_EXT);
    run_boot = -1; seen = 0;
    for (k = 0; k < RING_SLOTS; k++) {
        const bootlog_rec_t *r = RR_B(&fl[(wr + k) % RING_SLOTS]);
        if (!ring_slot_valid(&fl[(wr + k) % RING_SLOTS])) continue;
        if (mode != 'a' && r->pad != target) continue;
        if (mode == 'a' && (int)r->pad != run_boot) { printf("# boot=%u\n", (unsigned)r->pad); run_boot = r->pad; }
        if (r->latch == RING_MARK_ZERO) { if (mode != 't') ring_print_zero(r); continue; }
        if (seen++ < skip) continue;
        if (bootlog_csv_row(line, sizeof line, r) > 0 &&
            ring_vlog_csv_ext(line, sizeof line, &fl[(wr + k) % RING_SLOTS]) > 0) { printf("%s\n", line); rows++; }
    }
    printf("# end rows=%u\n", rows);
}

/* Print the help line once per USB connection and handle d / t / a. */
static void bootlog_poll_usb(void) {
    int c;
    if (stdio_usb_connected() && !g_usb_hinted) { bootlog_hint(); g_usb_hinted = 1; }
    if (!stdio_usb_connected()) return;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == 'd' || c == 'D') ring_dump('d');
        else if (c == 't' || c == 'T') ring_dump('t');
        else if (c == 'a' || c == 'A') ring_dump('a');
    }
}

/* Start-up (core 0, before the first burst). Waits up to 200 ms for core 1
 * to be ready for flash operations, sets the boot counter, compacts the
 * meta sector if fewer than 3 slots are free, writes the "boot started"
 * attempt record and finds the ring write position. Logging is disabled if
 * the program image overlaps the log area. */
void bootlog_init(void) {
    uintptr_t end = (uintptr_t)&__flash_binary_end;
    uintptr_t reserved = (uintptr_t)XIP_BASE + RING_FLASH_OFF;
    const bootlog_meta_t *last;
    uint32_t t0;
    memset(&g_fifo, 0, sizeof g_fifo);
    g_usb_hinted = 0;
    g_meta_pending = 0;
    g_meta_done = 0;
    g_boot_count = 1;
    g_ring_state = 0;
    g_overlap = (end > reserved);
    gpio_init(PICO_VBUS_PIN);
    gpio_set_dir(PICO_VBUS_PIN, GPIO_IN);
    gpio_disable_pulls(PICO_VBUS_PIN);

    t0 = to_ms_since_boot(get_absolute_time());
    while (!g_flash_core_ready && (to_ms_since_boot(get_absolute_time()) - t0) < 200u)
        tight_loop_contents();

    last = meta_latest();
    if (last) g_boot_count = last->boot_count + 1;
    g_boot_id = ring_boot_id(g_boot_count);
    if (!g_overlap && g_flash_core_ready) {
        (void)bootlog_meta_ensure_slots(3);   /* boot + attempt (+ zero) */
        bootlog_fill_attempt(BOOTLOG_REASON_BOOT, 0);
        if (bootlog_program_meta_now() != PICO_OK) g_meta_pending = 1;
    }
    /* Read-only scan (XIP, ~64 KB): find the write pointer. Any erase is deferred to the gaps. */
    g_erase_sec = g_overlap ? -1 : ring_init_plan(ring_flash(), &g_wr);
    if (g_overlap || !g_flash_core_ready) g_ring_state = 3;
    bootlog_hint();
    if (g_overlap) printf("BTH i16dring: ERROR program overlaps ring sectors; logging disabled\n");
}

/* Block/temperature fields captured when the current burst was built. */
static ring_rec_t g_vlog;   /* ext fields captured at burst build (core 0) */

/* ms from t to now, saturated at 0xFFFE; 0xFFFF if there is no module. */
static uint16_t vlog_age(uint32_t now, uint32_t t, uint8_t n) {
    uint32_t a;
    if (n == 0) return 0xFFFFu;
    a = (int32_t)(now - t) < 0 ? 0u : now - t;
    return a > 0xFFFEu ? 0xFFFEu : (uint16_t)a;
}

/* Capture the 16 extra bytes of the ring record from the snapshot about
 * to be sent (see ring_rec_t in ring_log.h for the field meanings). */
void bootlog_vlog_capture(const bth5_pub_t *pub, uint32_t now) {
    uint32_t sum = 0;
    unsigned i;
    for (i = 0; i < 20; i++) sum += pub->raw_block[i];
    g_vlog.pack_raw = (uint16_t)(sum > 0xFFFFu ? 0xFFFFu : sum);
    g_vlog.blk0w = (uint16_t)((pub->raw_block[0] & 0x3FFu) | ((uint16_t)(pub->n_vmod > 7 ? 7 : pub->n_vmod) << 10)
                              | (pub->live ? 0x8000u : 0u));
    g_vlog.blk19_raw = pub->raw_block[19];
    g_vlog.cmin_mv = pub->cmin_mv;
    g_vlog.cmax_mv = pub->cmax_mv;
    {   /* cell guard: age_new = cumulative rejects; blk0w bit 13 = a cell flagged since the last burst */
        static uint32_t last_flag;
        g_vlog.age_new_ms = (uint16_t)(pub->cell_rej > 0xFFFFu ? 0xFFFFu : pub->cell_rej);
        if (pub->cell_flag != last_flag) g_vlog.blk0w |= 0x2000u;
        last_flag = pub->cell_flag;
    }
    g_vlog.age_old_ms = vlog_age(now, pub->vold_ms, pub->n_vmod);
    {   /* module temps go in the blk_age_ms columns */
        int lo = pub->mod_tmin, hi = pub->mod_tmax;
        if (lo == -128 || hi == -128) g_vlog.blk_age_ms = 0xFFFFu;
        else {
            lo += 40; hi += 40;
            if (lo < 0) lo = 0;
            if (lo > 254) lo = 254;
            if (hi < 0) hi = 0;
            if (hi > 254) hi = 254;
            g_vlog.blk_age_ms = (uint16_t)(lo | (hi << 8));
        }
    }
}

/* Queue this burst's record (and, once, the auto-zero marker). */
void bootlog_after_burst(uint8_t b14_slot0) {
    ring_rec_t rr;
    int32_t ca;
    if (g_overlap) return;
    if (g_zero_done && !g_zero_marked) {
        memset(&rr, 0, sizeof rr);
        rr.b.t_ms = to_ms_since_boot(get_absolute_time());
        rr.b.adc_mv = g_zero_mv;
        rr.b.adc_counts = g_zero_avg_mv;
        rr.b.latch = RING_MARK_ZERO;
        rr.b.pad = g_boot_id;
        ring_fifo_push(&g_fifo, &rr);
        g_zero_marked = 1;
    }
    rr = g_vlog;
    rr.b.t_ms = to_ms_since_boot(get_absolute_time());
    rr.b.adc_mv = g_adc_mv;
    rr.b.adc_counts = g_x13_sent;          /* the X13 sent goes in the adc_counts column */
    if (g_erase_mark) { rr.blk0w |= 0x4000u; g_erase_mark = 0; }
    ca = (int32_t)lroundf(g_amps * 100.0f);
    if (ca > 32767) ca = 32767;
    if (ca < -32768) ca = -32768;
    rr.b.amps_cA = (int16_t)ca;
    rr.b.i_raw = g_i_raw;
    rr.b.latch = (uint8_t)((g_istate & 0x0F) | ((g_eng.st & 3) << 4) | (g_slewed ? 0x40 : 0));
    rr.b.b14_slot0 = b14_slot0;
    rr.b.pad = g_boot_id;
    ring_fifo_push(&g_fifo, &rr);
}

/* Microseconds left in the current gap. */
static uint64_t ring_remain_us(const uint64_t *gap_start_us) {
    uint64_t now = to_us_since_boot(get_absolute_time());
    return (*gap_start_us + BURST_GAP_US > now) ? (*gap_start_us + BURST_GAP_US - now) : 0;
}

/* Save this boot's auto-zero as a ZERO meta record. Same rules as the ring:
 * only after the 3 s holdoff, never once a USB host is connected, one page
 * program in a gap with room, never in the same gap as a pending attempt
 * record. (For the log only; it is not read back at the next boot.) */
static void zero_meta_service(const uint64_t *gap_start_us) {
    uint32_t t_ms = to_ms_since_boot(get_absolute_time());
    if (g_overlap || !g_flash_core_ready || g_zmeta_state >= 2) return;
    if (g_zmeta_state == 0) {
        if (!g_zero_done || t_ms < RING_HOLDOFF_MS) return;
        if (bootlog_host_connected()) { g_zmeta_state = 3; return; }
        bootlog_zero_fill(&g_zmeta, g_boot_count, g_zero_t_ms, g_zero_mv, g_zero_avg_mv, (uint16_t)AUTO_ZERO_N,
                          g_zero_avg_mv >= (uint16_t)AUTO_ZERO_MIN_MV && g_zero_avg_mv <= (uint16_t)AUTO_ZERO_MAX_MV);
        g_zmeta_state = 1;
    }
    if (g_meta_pending) return;
    if (bootlog_host_connected()) { g_zmeta_state = 3; return; }
    if (ring_remain_us(gap_start_us) <= (BOOTLOG_PAGE_PROG_MAX_US + 1500u)) return;
    {
        int slot = meta_next_free();
        meta_prog_t mp;
        if (slot < 0) { g_zmeta_state = 4; return; }
        mp.slot = (unsigned)slot; mp.rec = &g_zmeta;
        g_zmeta_state = (flash_safe_execute(bootlog_meta_rec_cb, &mp, 200) == PICO_OK) ? 2 : 4;
    }
}

/* Ring writer, one step per gap:
 *   - no writes in the first 3 s; stop for good once a USB host appears;
 *   - a pending sector erase runs now unless ring_erase_defer() says wait;
 *   - otherwise, with >= 4 records queued and time left in the gap, program
 *     one page. */
static void ring_service(uint64_t *gap_start_us, uint8_t *idx, const bth5_pub_t *pub) {
    uint32_t t_ms = to_ms_since_boot(get_absolute_time());
    if (g_ring_state == 0) {
        if (t_ms < RING_HOLDOFF_MS) return;
        if (bootlog_host_connected()) { g_ring_state = 2; bootlog_stage_attempt(BOOTLOG_REASON_SKIP_HOST, 0); return; }
        g_ring_state = 1;
    }
    if (g_ring_state != 1) return;
    if (bootlog_host_connected()) {
        g_ring_state = 2;
        bootlog_stage_attempt(BOOTLOG_REASON_SKIP_HOST, 0);
        if (stdio_usb_connected()) printf("i16dring: USB host connected, ring writes stopped. Type d to dump.\n");
        return;
    }
    if (g_erase_sec >= 0 && ring_erase_defer()) {
        g_erase_deferred++;                /* not now: keep programming the current sector */
    } else
    if (g_erase_sec >= 0) {
        bootlog_erase_t ctx;
        uint32_t eus = 0;
        int rc;
        ctx.gap_start_us = gap_start_us;
        ctx.idx = idx;
        ctx.pub = pub;
        ctx.flash_offs = RING_FLASH_OFF + (uint32_t)g_erase_sec * FLASH_SECTOR_SIZE;
        ctx.erase_us = &eus;
        rc = flash_safe_execute(bootlog_erase_cb, &ctx, 200);
        if (rc != PICO_OK) { g_ring_state = 3; bootlog_stage_attempt(BOOTLOG_REASON_FAIL_ERASE, rc); return; }
        g_ring_erases++;
        g_ring_last_erase_us = eus;
        g_erase_sec = -1;
        g_erase_mark = 1;
        return;
    }
    if (g_fifo.n < RING_BATCH) return;
    if (ring_remain_us(gap_start_us) <= (BOOTLOG_PAGE_PROG_MAX_US + 1500u)) return;
    {
        ring_fill_t f = ring_fill_page(ring_flash(), &g_wr, &g_fifo, g_ring_page);
        int rc;
        if (f.n > 0) {
            g_ring_prog_off = RING_FLASH_OFF + f.page * FLASH_PAGE_SIZE;
            rc = flash_safe_execute(ring_program_cb, 0, 200);
            if (rc != PICO_OK) { g_ring_state = 3; bootlog_stage_attempt(BOOTLOG_REASON_FAIL_PROGRAM, rc); return; }
            g_ring_written += (uint32_t)f.n;
        }
        if (f.need_erase >= 0) g_erase_sec = f.need_erase;
    }
}

/* Everything the log does in one gap (see flash_log.h). */
void bootlog_idle(uint64_t *gap_start_us, uint8_t *idx, const bth5_pub_t *pub) {
    bootlog_poll_usb();
    if (!g_overlap) ring_service(gap_start_us, idx, pub);
    g_amps_prev_gap = g_amps;
    if (g_meta_pending) (void)bootlog_try_write_meta(ring_remain_us(gap_start_us));
    zero_meta_service(gap_start_us);
}

