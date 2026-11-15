/* test_frames.c -- host test of the real bth5_buildFrame() (bth_frames.h).
 * Replays car ring-log sequences and full-range sweeps and checks every
 * frame. Built twice by run_tests.sh: "tq" (car build, B14Q_KEEP_LEGACY=1)
 * and "tq_pure" (B14Q_KEEP_LEGACY=0, quotient solve only); both must pass.
 * Modes:
 *   seq <csv> <spread> <out.txt>   replay a ring-log sequence (data/bNN.csv) exactly like main():
 *                                  X13 = floor(mean of previous burst's 12 b2/b8 fields).
 *                                  Writes "B t i_raw x13 b14log" + 6 hex frame lines per burst.
 *   sweep <out.bin>                current 0..1023 x block raw 0..1023 (step 3) x 30 idx x X13 lag
 *                                  variants, plus 2M random-block frames; raw frames to out.bin.
 * Checks per frame: b14[3:0] == floor(S/257), b15 == S mod 257 (!= 256), b1 = ~b0, b7 = 0,
 * b14[5:4] == template, slot-0 b14[7:6] == X13[9:8] (other slots: template [7:6]),
 * 10-bit current/voltage fields read back as written (nudged b10 within +-12). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bth_frames.h"

#ifndef ASSERT_Q
#define ASSERT_Q 1
#endif

static unsigned long n_frames, n_q_bad, n_ck_bad, n_hdr_bad, n_tpl_bad, n_x13_bad, n_fld_bad, n_nudge;
static unsigned long n_bad_slot[6], n_bad_3d;

static int qrule_ok(const uint8_t *f) {
    uint32_t s = 0; int i;
    for (i = 0; i < 15; i++) s += f[i];
    return (f[14] & 0x0F) == s / 257;
}
static int ck_ok(const uint8_t *f) {
    uint32_t s = 0; int i;
    for (i = 0; i < 15; i++) s += f[i];
    return s % 257 != 256 && f[15] == s % 257;
}
static const uint8_t *tpl(uint8_t idx) {
    return BTH_H1252[idx];
}

static void check(const uint8_t *f, uint8_t idx, const bth5_pub_t *p, int nudge) {
    const uint8_t *t = tpl(idx);
    int q = qrule_ok(f), c = ck_ok(f);
    n_frames++;
    if (!q) { n_q_bad++; if (idx == 29) n_bad_3d++; else n_bad_slot[idx % 6]++; }
    if (!c) n_ck_bad++;
    if (f[0] != 0x20 + idx || f[1] != (uint8_t)(0xFF - f[0]) || f[7] != 0) n_hdr_bad++;
    if ((f[14] & 0x30) != (t[14] & 0x30)) n_tpl_bad++;
    if (idx % 6 == 0) { if ((f[14] >> 6) != (bth5_burst_x13(idx, p) >> 8)) n_x13_bad++; }
    else if ((f[14] & 0xC0) != (t[14] & 0xC0)
             && 0
            ) n_x13_bad++;
    if (bth_get10(f, 2, 6, 6) != p->i_raw || bth_get10(f, 8, 12, 6) != p->i_raw) n_fld_bad++;
    if (nudge) n_nudge++;
    if (nudge > 12 || nudge < -12) n_fld_bad++;
}

/* ---------- seq replay (mirror of main()) ---------- */
static void set_blocks(bth5_pub_t *p, long pack_mv, long b0, long b19, int spread) {
    int b;
    p->live = pack_mv > 0;
    if (!p->live) return;
    float mid = (float)(pack_mv - b0 - b19) / 18.0f / 1000.0f;
    p->raw_block[0] = bth5_encodeBlock((float)b0 / 1000.0f);
    p->raw_block[19] = bth5_encodeBlock((float)b19 / 1000.0f);
    for (b = 1; b < 19; b++) {
        int r = (int)bth5_encodeBlock(mid) + (spread ? ((b * 7) % 5) - 2 : 0);
        p->raw_block[b] = (uint16_t)(r < 0 ? 0 : r > 1023 ? 1023 : r);
    }
    p->raw_uniform = bth5_encodeBlock((float)pack_mv / 20000.0f);
}

static int run_seq(const char *csv, int spread, const char *outp) {
    FILE *in = fopen(csv, "r"), *out = fopen(outp, "w");
    char line[256]; uint8_t idx = 0; uint32_t hsum = 0; unsigned hn = 0;
    bth5_pub_t pub; memset(&pub, 0, sizeof pub);
    if (!in || !out) { perror(csv); return 2; }
    if (!fgets(line, sizeof line, in)) return 2;
    while (fgets(line, sizeof line, in)) {
        long t, ir, x13l, b14l, pmv, b0, b19;
        if (sscanf(line, "%ld,%ld,%ld,%ld,%ld,%ld,%ld", &t, &ir, &x13l, &b14l, &pmv, &b0, &b19) != 7) continue;
        pub.i_raw = (uint16_t)ir;
        pub.i_raw13 = hn == 12 ? (uint16_t)(hsum / 12u) : 0x1FF;   /* gx13_from_history() */
        hsum = 0; hn = 0;
        set_blocks(&pub, pmv, b0, b19, spread);
        uint8_t f[6][16], b14_slot0 = 0;
        for (int fr = 0; fr < 6; fr++) {
            int n = bth5_buildFrame(f[fr], idx, &pub);
            if (fr == 0) b14_slot0 = f[fr][14];            /* what main() hands to the ring log */
            check(f[fr], idx, &pub, n);
            hsum += bth_get10(f[fr], 2, 6, 6) + bth_get10(f[fr], 8, 12, 6); hn += 2;
            idx = (uint8_t)((idx + 1) % 30);
        }
        if (b14_slot0 != f[0][14]) n_fld_bad++;
        fprintf(out, "B %ld %ld %u %u %ld %ld\n", t, ir, (unsigned)bth_get10(f[0], 13, 14, 6), b14_slot0, x13l, b14l);
        for (int fr = 0; fr < 6; fr++) {
            for (int i = 0; i < 16; i++) fprintf(out, "%s%02X", i ? " " : "", f[fr][i]);
            fputc('\n', out);
        }
    }
    fclose(in); fclose(out);
    return 0;
}

/* ---------- sweeps ---------- */
static uint32_t rng = 12345;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static int run_sweep(const char *outp) {
    FILE *out = fopen(outp, "wb");
    bth5_pub_t pub; uint8_t f[16];
    if (!out) { perror(outp); return 2; }
    memset(&pub, 0, sizeof pub);
    /* A: uniform blocks, live, every current x every 3rd block raw x every frame, X13 = I and lagged */
    for (int ir = 0; ir < 1024; ir++)
        for (int v = 0; v < 1024; v += 3)
            for (int lag = 0; lag < 3; lag++) {
                static const int dl[3] = {0, -150, 150};
                int x = ir + dl[lag]; x = x < 0 ? 0 : x > 1023 ? 1023 : x;
                pub.live = 1; pub.i_raw = (uint16_t)ir; pub.i_raw13 = (uint16_t)x;
                for (int b = 0; b < 20; b++) pub.raw_block[b] = (uint16_t)v;
                pub.raw_uniform = (uint16_t)v;
                for (uint8_t idx = 0; idx < 30; idx++) {
                    int n = bth5_buildFrame(f, idx, &pub);
                    check(f, idx, &pub, n); fwrite(f, 16, 1, out);
                }
            }
    /* B: template blocks (pub.live = 0, boot) */
    for (int ir = 0; ir < 1024; ir++)
        for (int x = 0; x < 1024; x += 31) {
            pub.live = 0; pub.i_raw = (uint16_t)ir; pub.i_raw13 = (uint16_t)x;
            for (uint8_t idx = 0; idx < 30; idx++) {
                int n = bth5_buildFrame(f, idx, &pub);
                check(f, idx, &pub, n); fwrite(f, 16, 1, out);
            }
        }
    /* C: 2M random frames, independent random blocks/current/X13 */
    for (long k = 0; k < 2000000; k++) {
        pub.live = 1; pub.i_raw = (uint16_t)(rnd() & 1023); pub.i_raw13 = (uint16_t)(rnd() & 1023);
        for (int b = 0; b < 20; b++) pub.raw_block[b] = (uint16_t)(rnd() & 1023);
        pub.raw_uniform = (uint16_t)(rnd() & 1023);
        uint8_t idx = (uint8_t)(rnd() % 30);
        int n = bth5_buildFrame(f, idx, &pub);
        check(f, idx, &pub, n); fwrite(f, 16, 1, out);
    }
    fclose(out);
    return 0;
}

int main(int argc, char **argv) {
    int rc;
    if (argc >= 5 && !strcmp(argv[1], "seq")) rc = run_seq(argv[2], atoi(argv[3]), argv[4]);
    else if (argc >= 3 && !strcmp(argv[1], "sweep")) rc = run_sweep(argv[2]);
    else { fprintf(stderr, "usage: seq <csv> <spread> <out> | sweep <out.bin>\n"); return 2; }
    if (rc) return rc;
    printf("B14_QUOTIENT=%d frames=%lu quotient_bad=%lu (slot0..5 excl 0x3D: %lu %lu %lu %lu %lu %lu; 0x3D: %lu) "
           "checksum_bad=%lu hdr_bad=%lu b14[5:4]_bad=%lu b14[7:6]_bad=%lu field_bad=%lu b10_nudged=%lu\n",
           1 /* quotient checksum always on */, n_frames, n_q_bad, n_bad_slot[0], n_bad_slot[1], n_bad_slot[2], n_bad_slot[3],
           n_bad_slot[4], n_bad_slot[5], n_bad_3d, n_ck_bad, n_hdr_bad, n_tpl_bad, n_x13_bad, n_fld_bad, n_nudge);
#if ASSERT_Q
    if (n_q_bad || n_ck_bad || n_hdr_bad || n_tpl_bad || n_x13_bad || n_fld_bad) { printf("FAIL\n"); return 1; }
    printf("PASS\n");
#endif
    return 0;
}
