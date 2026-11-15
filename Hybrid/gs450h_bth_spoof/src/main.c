/* main.c -- Lexus GS450h battery-board (BTH) emulator on a Raspberry Pi Pico 2.
 *
 * The HV ECU expects a serial stream from the battery board ("BTH" link).
 * This firmware sends that stream, built from five BMW Gen1 PHEV modules
 * (read over CAN) and a Hall current sensor.
 *
 * Two cores:
 *   core 1  core1_can(): every 10 ms polls the BMW modules over the
 *           MCP2518FD, decodes their replies (bmw_pack.c), works out the
 *           20 block voltages and the temperatures, and publishes a
 *           snapshot (bth5_pub_t) under a spin lock.
 *   core 0  main(): sends a burst of 6 BTH frames every 106.1 ms. Before
 *           each burst it copies the latest snapshot and adds the current;
 *           after each burst it samples the current sensor and lets the
 *           flash log (flash_log.c) use the 8.18 ms gap.
 *
 * BTH link: UART0 TX on GP0, 19300 Bd, 8 data bits, odd parity, 1 stop
 * bit, 1020 us per byte (the OEM board's pacing), 16-byte frames, 6 frames
 * per burst, counter 0x20..0x3D. Frame contents: bth_frames.h.
 *
 * Hardware (pins below):
 *   GP0         BTH TX -> 74HCT04 at 5 V, wired like the OEM TC7W04 (IC401):
 *               complementary outputs through 2.7 k onto BTH+ / BTH-.
 *   GP26/ADC0   Hall sensor via a 10 k / 20 k divider (sensor 2.5 V = 0 A,
 *               10 mV/A). Auto-zeroed over the first 5 bursts.
 *   GP16-19     SPI0 to the MCP2518FD (MISO 16, CS 17, SCK 18, MOSI 19), 8 MHz
 *   GP15        MCP2518FD INT
 *   LED         toggles every burst
 *   USB         status line and log dump only.
 *
 * WARNING: do not have USB connected at key-on; that faults the car.
 * HV SAFETY: this firmware sends made-up battery data to the ECU, which
 * can close the main contactors on it. Use only with a known-safe pack,
 * fused HV and a way to cut 12 V / IG at once. Disconnect the original
 * battery sense board from BTH+ / BTH- first.
 */

#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/spi.h"

#include "bth_link.h"
#include "bth_frames.h"
#include "pack_to_bth.h"
#include "bmw_poll.h"
#include "mcp2518fd.h"
#include "pico/flash.h"
#include "flash_log.h"

/* Set by core 1 once it can be paused for flash writes (flash_log.c waits
 * for it before touching flash). */
volatile int g_flash_core_ready;

/* ---- MCP2518FD CAN controller (core 1) ---- */
#define PIN_SPI_MISO         16
#define PIN_SPI_CS           17
#define PIN_SPI_SCK          18
#define PIN_SPI_MOSI         19
#define PIN_INT              15
#ifndef PIN_STBY
#define PIN_STBY             (-1)
#endif
#define SPI_PORT             spi0
#define SPI_HZ               8000000   /* SPI clock */
#define MCP_OSC_HZ           20000000  /* crystal on the MCP2518FD board */
#define MCP_BITRATE          500000    /* BMW module CAN bus */
#define POLL_PERIOD_MS       10        /* core-1 loop and module poll period */
#define CAN_REINIT_MS        2000      /* retry the controller init this often while in error */

/* ---- BTH UART and current sensor (core 0) ---- */
#define UART_ID              uart0
#define UART_TX_PIN          0        // GP0, physical pin 1
#define UART_BAUD            19300    // OEM 19,302 Bd; the RP2350 at 150 MHz gives 19300.05
#define ADC_GPIO             26       // GP26 / ADC0, physical pin 31
#define ADC_INPUT            0
#ifndef LED_PIN
#ifdef PICO_DEFAULT_LED_PIN
#define LED_PIN              PICO_DEFAULT_LED_PIN
#else
#define LED_PIN              25
#endif
#endif

/* ---- current measurement, shared with flash_log.c ---- */
uint16_t g_i_raw = 0x1FF;      /* current raw for b2/b8 (0x1FF = 0 A) */
uint16_t g_i_raw13 = 0x1FF;    /* floor-rounded current raw (see sample_current) */
uint16_t g_adc_mv;             /* last sensor reading, mV at the sensor */
uint16_t g_adc_counts;         /* last ADC mean, 12-bit counts at the pin */
float g_amps;                  /* last current, A, discharge positive */
uint16_t g_zero_mv = (uint16_t)CURRENT_ZERO_MV; /* zero in use (mV) */
uint16_t g_zero_avg_mv;                         /* window mean (mV), 0 until done */
uint8_t  g_zero_done;                           /* 1 once the auto-zero is taken */
uint32_t g_zero_t_ms;                           /* ms since boot when the zero was taken */
static float    g_zero_acc;                     /* auto-zero window sum */
static int      g_zero_n;                       /* auto-zero window count */
bth5_eng_t g_eng = { BTH5_ENG_PRE, 0, 0x1FF };  /* engine-start detector */
int g_slewed;                                   /* always 0 in this build (no charge slew) */
int g_istate = BTH5_IST_DB;                     /* current state for the log */

/* UART0 for BTH: 19300 Bd 8O1, TX only (GP0), FIFO off so every byte
 * starts the moment it is written. */
static void uart_bth_init(void) {
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // GP1 (UART0 RX) is left unused.
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_ODD);
    uart_set_fifo_enabled(UART_ID, false);
}

/* ADC0 on GP26 for the Hall sensor. */
static void adc_current_init(void) {
    adc_init();
    adc_gpio_init(ADC_GPIO);
    // RP2350-E9: disable pad pulls on ADC inputs (adc_gpio_init already
    // does this; repeat it so the workaround is explicit).
    gpio_disable_pulls(ADC_GPIO);
    adc_select_input(ADC_INPUT);
    (void)adc_read();  // discard one settling sample
}

/* Average ADC_SAMPLES (256) reads; return the sensor output in mV (pin mV
 * x 1.5 for the divider). Also stores g_adc_counts / g_adc_mv for the log. */
static float read_sensor_mv(void) {
    uint32_t acc = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        acc += adc_read();
    }
    float adc_counts = (float)acc / (float)ADC_SAMPLES;
    float adc_mv = adc_counts * (ADC_VREF_MV / 4096.0f);
    float sensor_mv = adc_mv * DIVIDER_RATIO;
    g_adc_counts = (uint16_t)(adc_counts + 0.5f);
    {
        int mv = (int)(sensor_mv + 0.5f);
        if (mv < 0) mv = 0;
        if (mv > 65535) mv = 65535;
        g_adc_mv = (uint16_t)mv;
    }
    return sensor_mv;
}

static bth5_isense_t g_isense;     /* sensor state, fault fallback to 0 A */
static int g_isense_init;
/* Once per burst, in the gap after it: read the sensor and work out what
 * current to send next.
 *   - first AUTO_ZERO_N (5) calls: auto-zero window, 0x1FF is sent;
 *   - then: amps -> raw (b2/b8 rounded, plus a floor-rounded copy),
 *     engine-start detector step, deadband / charge-hold policy. */
static void sample_current(void) {
    float mv = read_sensor_mv();
    if (!g_isense_init) { bth5_isense_init(&g_isense); g_isense_init = 1; }
    if (!g_zero_done) {
        /* Auto-zero window: one sample per burst for the first AUTO_ZERO_N
         * bursts. Exactly 0x1FF is sent meanwhile. */
        g_zero_acc += mv;
        if (++g_zero_n >= AUTO_ZERO_N) {
            float mean = g_zero_acc / (float)g_zero_n;
            g_isense.zero_mv = bth5_autozero_pick(mean, NULL);
            g_zero_mv = (uint16_t)(g_isense.zero_mv + 0.5f);
            g_zero_avg_mv = (uint16_t)(mean < 0.0f ? 0 : mean > 65535.0f ? 65535 : mean + 0.5f);
            g_zero_done = 1;
            g_zero_t_ms = to_ms_since_boot(get_absolute_time());
        }
        g_amps = 0.0f;
        g_i_raw = 0x1FF; g_i_raw13 = 0x1FF;
        g_istate = BTH5_IST_ZERO;
        return;
    }
    g_amps = bth5_isense_step(&g_isense, (mv / DIVIDER_RATIO) * (4096.0f / ADC_VREF_MV));
    g_i_raw = bth5_encodeCurrent(g_amps);
    g_i_raw13 = bth5_encodeCurrentFloor(g_amps);
    bth5_eng_step(&g_eng, g_amps);
    g_istate = bth5_applyCurrentPolicy(&g_i_raw, &g_i_raw13, g_amps, &g_eng, &g_slewed);
}

/* ---- core 1: BMW modules -> published snapshot ---- */
static spin_lock_t *g_lock;         /* guards g_pub between the cores */
static bth5_pub_t g_pub;            /* latest snapshot; live = 0 until 5 modules are good */
static pack_state_t g_pack;         /* module model (bmw_pack.c), core 1 only */
static volatile bool g_mcp_ok;      /* CAN controller running */
static volatile mcp_status_t g_mcp_init_rc;
static volatile uint8_t g_mods;     /* modules present, for the status line */
static volatile float g_pack_v;
static volatile bool g_live_now;
static mcp_bittiming_t g_bt;

/* Milliseconds since boot. */
static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

/* Slot-0 X13 (core 0). The OEM board sends, in slot 0 b13, the floor of
 * the mean of the 12 current fields (b2/b8 of the 6 frames) of the
 * PREVIOUS burst (728/728 OEM frames). main() sums them as it sends. */
uint16_t g_x13_sent = 0x1FF;          /* X13 in the burst just sent (logged) */
static uint32_t g_x13_sum;
static uint8_t  g_x13_n;
/* X13 for the next burst; 0x1FF until a full burst has been summed. */
static uint16_t gx13_from_history(void) {
    return (g_x13_n == 12) ? (uint16_t)(g_x13_sum / 12u) : (uint16_t)0x1FF;
}

/* Read every waiting CAN frame into the pack model. Extended and remote
 * frames are skipped. Returns frames read, or < 0 on SPI error. */
static int drain_rx(void) {
    int n = 0;
    for (;;) {
        mcp_frame_t fr;
        bool ov = false;
        int rc = mcp2518fd_recv(&fr, &ov);
        if (rc < 0) return rc;
        if (rc == 0) break;
        if (fr.ide || fr.rtr) continue;
        pack_decode_can(&g_pack, fr.id, fr.data, fr.dlc, now_ms());
        n++;
    }
    return n;
}

/* Send the next module poll (0x080..0x085 in turn, see bmw_poll.h). */
static void send_poll(uint8_t *nextmes, uint8_t *mescycle) {
    mcp_frame_t fr;
    uint16_t id;
    if (*nextmes >= 6) { *nextmes = 0; *mescycle = (uint8_t)((*mescycle + 1) & 0x0F); }
    memset(&fr, 0, sizeof fr);
    bmw_build_poll(fr.data, &id, *nextmes, *mescycle);
    fr.id = id;
    fr.dlc = 8;
    (void)mcp2518fd_send(&fr);
    (*nextmes)++;
}

/* Core 1 main loop, every 10 ms:
 *   - (re)start the MCP2518FD if needed, read received frames, check for
 *     bus-off / wrong mode;
 *   - update the pack model; if all 5 modules are good, take the 20 block
 *     voltages (otherwise the last good ones are kept);
 *   - aux V (fixed 12.6 V) and the TB1-4 / intake filter (temps_aux.h);
 *   - publish the snapshot under the spin lock;
 *   - send one module poll, and drain RX again if INT is asserted. */
static void core1_can(void) {
    /* Must be first: flash writes need to pause this core. If this ran
     * after mcp2518fd_init and that call hung, the log could never write. */
    flash_safe_execute_core_init();
    g_flash_core_ready = 1;
    mcp2518fd_cfg_t cfg = {
        .spi = SPI_PORT, .pin_miso = PIN_SPI_MISO, .pin_mosi = PIN_SPI_MOSI,
        .pin_sck = PIN_SPI_SCK, .pin_cs = PIN_SPI_CS, .pin_int = PIN_INT,
        .pin_stby = PIN_STBY, .spi_hz = SPI_HZ, .osc_hz = MCP_OSC_HZ,
        .bitrate = MCP_BITRATE,
    };
    uint8_t nextmes = 0, mescycle = 0;
    uint32_t last_reinit = 0;
    bth5_pub_t pub;
    uint16_t vlog_prev_blk[20];
    memset(vlog_prev_blk, 0, sizeof vlog_prev_blk);
    memset(&pub, 0, sizeof pub);
    pub.i_raw = 0x1FF; pub.i_raw13 = 0x1FF;
    g_mcp_init_rc = mcp2518fd_init(&cfg, &g_bt);
    g_mcp_ok = (g_mcp_init_rc == MCP_OK);
    for (;;) {
        absolute_time_t t0 = get_absolute_time();
        uint32_t t_ms = now_ms(), irq;
        mcp_status_info_t st;
        bool failed = true, live;
        if (!g_mcp_ok && t_ms - last_reinit >= CAN_REINIT_MS) {
            last_reinit = t_ms;
            g_mcp_init_rc = mcp2518fd_init(&cfg, &g_bt);
            g_mcp_ok = (g_mcp_init_rc == MCP_OK);
        }
        if (g_mcp_ok) {
            if (drain_rx() < 0) g_mcp_ok = false;
            failed = (mcp2518fd_read_status(&st) != MCP_OK) || st.bus_off || st.opmod != 6;
            if (failed) { g_mcp_ok = false; last_reinit = t_ms; }
        }
        pack_set_current(&g_pack, 0.0f, false);
        pack_set_can_error(&g_pack, failed || !g_mcp_ok);
        pack_update(&g_pack, t_ms, 0.010f);
        live = bth5_blocks_from_pack(&g_pack, &pub) != 0;  /* else: hold */
        pub.cell_rej = g_pack.cell_rej;
        pub.cell_flag = g_pack.cell_flag;
        bth5_vlog_stats(&g_pack, &pub);
        if (memcmp(vlog_prev_blk, pub.raw_block, sizeof vlog_prev_blk) != 0) {
            memcpy(vlog_prev_blk, pub.raw_block, sizeof vlog_prev_blk);
            pub.blk_chg_ms = t_ms;
        }
        pub.aux_v_raw = bth5_encodeAuxV(AUX_V_FIXED);
        {   static bth5_tb_t tb;
            int nt = live ? g_pack.n_temp_used : 0;
            uint16_t tbo[5] = {0, 0, 0, 0, 0};
            bth5_tb_step(&tb, tbo, nt, g_pack.tavg, g_pack.tmin, g_pack.tmax, t_ms);
            memcpy(pub.tb_raw, tbo, sizeof pub.tb_raw);
            pub.intake_raw = tbo[4];
            pub.tb_src = tb.src;
            pub.mod_tmin = nt ? (int8_t)lroundf(g_pack.tmin < -127 ? -127 : g_pack.tmin > 127 ? 127 : g_pack.tmin) : -128;
            pub.mod_tmax = nt ? (int8_t)lroundf(g_pack.tmax < -127 ? -127 : g_pack.tmax > 127 ? 127 : g_pack.tmax) : -128;
        }
        g_live_now = live;
        g_mods = g_pack.n_present;
        g_pack_v = g_pack.pack_v;
        irq = spin_lock_blocking(g_lock);
        g_pub = pub;
        spin_unlock(g_lock, irq);
        if (g_mcp_ok) send_poll(&nextmes, &mescycle);
        if (g_mcp_ok && mcp2518fd_int_asserted()) { if (drain_rx() < 0) g_mcp_ok = false; }
        busy_wait_until(delayed_by_ms(t0, POLL_PERIOD_MS));
    }
}

/* Send one 16-byte frame, one byte every BYTE_PERIOD_US (1020 us). */
static void send_burst_bytes(const uint8_t *f) {
    // Pace from a single timeline so the byte spacing stays 1020 us
    // even if a putc blocks briefly. FIFO is off, so uart_putc_raw
    // writes the holding register and the start bit begins immediately.
    absolute_time_t t_next = get_absolute_time();
    for (int b = 0; b < 16; b++) {
        uart_putc_raw(UART_ID, f[b]);
        t_next = delayed_by_us(t_next, BYTE_PERIOD_US);
        busy_wait_until(t_next);
    }
    uart_tx_wait_blocking(UART_ID);
}

/* Core 0: set up, start core 1, then send bursts for ever.
 * Each loop: copy the snapshot, add current and X13, build and send 6
 * frames, log the burst, sample the current, print the status line (USB
 * only, once per 30 frames), give the gap to the flash log, then wait out
 * the rest of the 8.18 ms gap. */
int main(void) {
    stdio_init_all();  // USB CDC only; UART stdio is disabled in CMake
    uart_bth_init();
    adc_current_init();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    pack_init(&g_pack);
    memset(&g_pub, 0, sizeof g_pub);
    g_pub.i_raw = 0x1FF; g_pub.i_raw13 = 0x1FF;                  /* live=0: template blocks until modules are good */
    g_lock = spin_lock_instance((uint)spin_lock_claim_unused(true));
    multicore_launch_core1(core1_can);     /* CAN on core 1 */

    bootlog_init();
    sleep_ms(50);
    sample_current();

    uint8_t idx = 0;
    for (;;) {
        bth5_pub_t pub;
        {
            uint32_t irq = spin_lock_blocking(g_lock);
            pub = g_pub;
            spin_unlock(g_lock, irq);
        }
        pub.i_raw = g_i_raw;               /* live current (0 A on sensor fault) */
        pub.i_raw13 = gx13_from_history(); /* slot-0 X13 = previous burst mean */
        g_x13_sent = pub.i_raw13;
        g_x13_sum = 0; g_x13_n = 0;
        uint8_t b14_slot0 = 0;
        bootlog_vlog_capture(&pub, to_ms_since_boot(get_absolute_time()));
        for (int fr = 0; fr < 6; fr++) {
            uint8_t f[16];
            bth5_buildFrame(f, idx, &pub);  /* template + live fields + checksum */
            if (fr == 0) b14_slot0 = f[14];
            g_x13_sum += (uint32_t)bth_get10(f, 2, 6, 6) + bth_get10(f, 8, 12, 6);
            g_x13_n += 2;
            send_burst_bytes(f);
            idx = (uint8_t)((idx + 1) % 30);
        }
        bootlog_after_burst(b14_slot0);

        gpio_xor_mask(1u << LED_PIN);

        absolute_time_t gap_start = get_absolute_time();
        sample_current();

        if (idx == 0 && stdio_usb_connected()) {
            printf("v4m%c data=%s mods=%u Vpack=%.1f blk0=%.3f  I=%+.2f A (discharge+)  i15=%s  sent raw 0x%03X",
                   ' ', pub.live ? (g_live_now ? "LIVE" : "HOLD") : "FORUM", (unsigned)g_mods,
                   (double)g_pack_v, (double)bth_decodeBlock(pub.raw_block[0]),
                   (double)g_amps, g_istate == BTH5_IST_DIS ? "DIS" : g_istate == BTH5_IST_CHG ? "CHG" :
                   g_istate == BTH5_IST_ZERO ? "ZERO" :
                   g_istate == BTH5_IST_CHGH ? "CHGH" :
                   "DB",
                   (unsigned)g_i_raw);
            printf("  modT=%d..%d C TB=%u/%u/%u/%u IA=%u(%s)", (int)pub.mod_tmin, (int)pub.mod_tmax,
                   (unsigned)pub.tb_raw[0], (unsigned)pub.tb_raw[1], (unsigned)pub.tb_raw[2], (unsigned)pub.tb_raw[3],
                   (unsigned)pub.intake_raw,
                   pub.tb_src == BTH5_TB_LIVE ? "live" : pub.tb_src == BTH5_TB_STALE ? "stale" : "tpl");
            printf("  zero=%u mV (window mean %u)", (unsigned)g_zero_mv, (unsigned)g_zero_avg_mv);
            printf("  eng=%d", g_eng.st);
            if (!g_mcp_ok) printf("  CAN_ERR mcp=%s", mcp2518fd_strerror(g_mcp_init_rc));
            printf("\n");
        }

        /* Snapshot for any bursts the flash log sends during a sector erase:
         * new current, X13 = mean of the burst just sent. */
        pub.i_raw = g_i_raw;
        pub.i_raw13 = gx13_from_history();
        {
            uint64_t gap_us = to_us_since_boot(gap_start);
            uint8_t idx0 = idx;
            bootlog_idle(&gap_us, &idx, &pub);
            if (idx != idx0) {             /* erase bursts were played: all fields = pub.i_raw */
                g_x13_sum = 12u * pub.i_raw; g_x13_n = 12;
            }
            update_us_since_boot(&gap_start, gap_us);
        }
        busy_wait_until(delayed_by_us(gap_start, BURST_GAP_US));
    }
}
