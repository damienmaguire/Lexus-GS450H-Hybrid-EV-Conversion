/* test_temps.c -- host test of the battery-temperature path (temps_aux.h).
 *  1. law: template raws 666/673/668/650 -> deg C and back; clamp to
 *     TEMP_SEND_MIN_C..TEMP_SEND_MAX_C.
 *  2. frame sweep: every TB raw 1..1022 in each of the 4 fields (+ clamp-range cross product,
 *     random currents/X13/blocks, all 30 frame indices) through the real bth5_buildFrame():
 *     checksum (test_frames.c check()), TB read-back at b5+b6[1:0], b11+b12[1:0], b13+b14[7:6],
 *     slot-5 b5+b6[1:0], and intake at slot-3 b5+b6[1:0]; every other bit identical to the
 *     same frame with TB = template (except b14[3:0]/b15 and the legacy b10 nudge).
 *  3. filter: init, slew, spread, clamp, invalid inputs, stale -> exact template raws. */
#define main i18q_main
#include "test_frames.c"
#undef main
static int fails;
#define T_CHECK(c, ...) do { if (!(c)) { printf("FAIL " __VA_ARGS__); printf("\n"); fails++; } } while (0)
static double law(int raw) { return (double)bth5_decodeTemp((uint16_t)raw); }
static double law_d(double t) { double x = exp((double)TEMP_BETA_LNK + (double)TEMP_BETA_K / (t + 273.15)); return (double)TEMP_ADC_N * x / (1 + x); }

static unsigned long nf2, n_rb_bad, n_other_bad;
static void frame_case(bth5_pub_t *p, uint8_t idx) {
    uint8_t f[16], g[16]; bth5_pub_t q = *p;
    int n = bth5_buildFrame(f, idx, p);
    check(f, idx, p, n); nf2++;
    int s = idx % 6;
    if (s == 4) {
        if (p->tb_raw[0] && bth_get10(f, 5, 6, 0) != p->tb_raw[0]) n_rb_bad++;
        if (p->tb_raw[1] && bth_get10(f, 11, 12, 0) != p->tb_raw[1]) n_rb_bad++;
        if (p->tb_raw[2] && bth_get10(f, 13, 14, 6) != p->tb_raw[2]) n_rb_bad++;
        if (p->tb_raw[2] && (f[14] >> 6) != (p->tb_raw[2] >> 8)) n_rb_bad++;
    }
    if (s == 5 && p->tb_raw[3] && bth_get10(f, 5, 6, 0) != p->tb_raw[3]) n_rb_bad++;
    if (s == 3 && p->intake_raw && bth_get10(f, 5, 6, 0) != p->intake_raw) n_rb_bad++;
    if (s == 3 && p->aux_v_raw && bth_get10(f, 13, 14, 6) != p->aux_v_raw) n_rb_bad++;
    q.intake_raw = 0;
    memset(q.tb_raw, 0, sizeof q.tb_raw);
    bth5_buildFrame(g, idx, &q);
    {   int d = (int)bth_get10(f, 10, 12, 2) - (int)bth_get10(g, 10, 12, 2);
        if (d > 12 || d < -12) n_other_bad++; }
    for (int i = 0; i < 15; i++) {
        uint8_t m = 0xFF;
        if (s == 4 && (i == 5 || i == 11 || i == 13)) m = 0;
        if (s == 4 && (i == 6 || i == 12)) m = 0xFC;
        if (s == 4 && i == 14) m = 0x30;          /* [7:6] = TB3 hi, [3:0] = q */
        if (s == 5 && i == 5) m = 0;
        if (s == 5 && i == 6) m = 0xFC;
        if (s == 3 && i == 5) m = 0;
        if (s == 3 && i == 6) m = 0xFC;
        if (i == 14 && s != 4) m = 0xF0;          /* q nibble may differ */
        if (i == 10) m = 0;                       /* legacy b10 borrow nudge: compared as a 10-bit field below */
        if (i == 12) m &= 0xF3;                   /* b10 field high bits (b12[3:2]) */
        if ((f[i] & m) != (g[i] & m)) n_other_bad++;
    }
}

int main(void) {
    static const int tpl[4] = {666, 673, 668, 650};
    static const double x431[4] = {26.29, 25.48, 26.06, 28.15};
    printf("law ln(raw/(%.0f-raw)) = %.6f + %.4f/T_K\n", (double)TEMP_ADC_N, (double)TEMP_BETA_LNK, (double)TEMP_BETA_K);
    T_CHECK(fabs(law(763) - 14.46) < 0.01, "clamp point 763 -> %.3f vs X431 14.46", law(763));
    printf("  car clamp point raw 763 -> %.3f C (X431 14.46)\n", law(763));
    for (int i = 0; i < 4; i++) {
        T_CHECK(fabs(law(tpl[i]) - x431[i]) < 0.01, "law TB%d %.3f vs %.2f", i + 1, law(tpl[i]), x431[i]);
        T_CHECK(bth5_encodeTemp((float)x431[i]) == tpl[i], "encode TB%d %u", i + 1, bth5_encodeTemp((float)x431[i]));
        T_CHECK(BTH5_TB_TPL_RAW[i] == tpl[i] && BTH_H1252[i == 3 ? 5 : 4][0] == (i == 3 ? 0x25 : 0x24), "tpl table");
        printf("  TB%d template raw %d -> %.3f C (X431 %.2f)\n", i + 1, tpl[i], law(tpl[i]), x431[i]);
    }
    {   double ia = law(605);
        T_CHECK(bth_get10(BTH_H1252[3], 5, 6, 0) == 605, "intake template raw");
        T_CHECK(bth5_encodeTemp((float)ia) == 605, "intake template round trip");
        printf("  intake template raw 605 -> %.3f C under the beta law (Techstream 33.2, 1-dp truncated => 33.20..33.29): %+.2f..%+.2f C = %.1f..%.1f counts\n",
               ia, ia - 33.29, ia - 33.20, (ia - 33.29) / (law(604) - law(605)), (ia - 33.20) / (law(604) - law(605)));
    }
    /* template frames really hold those raws */
    T_CHECK(bth_get10(BTH_H1252[4], 5, 6, 0) == 666 && bth_get10(BTH_H1252[4], 11, 12, 0) == 673 &&
            bth_get10(BTH_H1252[4], 13, 14, 6) == 668 && bth_get10(BTH_H1252[5], 5, 6, 0) == 650, "template raws in BTH_H1252");
    {   int lo = bth5_encodeTemp(TEMP_SEND_MAX_C), hi = bth5_encodeTemp(TEMP_SEND_MIN_C), prev = 2000;
        T_CHECK(bth5_encodeTemp(TEMP_SEND_MAX_C + 20.0f) == lo && bth5_encodeTemp(TEMP_SEND_MIN_C - 25.0f) == hi, "clamp");
        for (float c = -20; c <= 60; c += 0.01f) { int r = bth5_encodeTemp(c); T_CHECK(r <= prev, "monotonic at %.2f", c); prev = r; }
        printf("  clamp %.0f..%.0f C -> raw %d..%d (hi bits %d..%d)\n", (double)TEMP_SEND_MIN_C, (double)TEMP_SEND_MAX_C, lo, hi, lo >> 8, hi >> 8);
        T_CHECK(bth5_encodeTemp(TEMP_SEND_MIN_C) == hi && hi >= lo, "clamp raws");
        {   int nd = 0, nfar = 0;
            for (int ct = (int)(TEMP_SEND_MIN_C * 100); ct <= (int)(TEMP_SEND_MAX_C * 100); ct++) {
                double ex = law_d(ct * 0.01); int r = bth5_encodeTemp(ct * 0.01f);
                if (r != (int)lround(ex)) nd++;
                if (fabs(r - ex) > 0.5 + 1e-3) nfar++;
                if (fabs(law(r) - ct * 0.01) > 0.08) nfar++;   /* round trip within half a count (~0.07 C at 5 C) */
            }
            printf("  float encode vs double law over the clamp range: %d of %d differ by the last rounding, %d off by more\n",
                   nd, (int)((TEMP_SEND_MAX_C - TEMP_SEND_MIN_C) * 100) + 1, nfar);
            T_CHECK(nfar == 0, "encode accuracy"); }
        for (int r = 600; r <= 900; r += 50) printf("  raw %d -> %.3f C (%.4f C/count)\n", r, law(r), law(r) - law(r + 1));
    }
    /* 2. frame sweep */
    {
        bth5_pub_t p; memset(&p, 0, sizeof p);
        for (int fld = 0; fld < BTH5_TB_NCH; fld++)
            for (int r = 1; r <= 1022; r++)
                for (int k = 0; k < 8; k++) {
                    p.live = 1; p.i_raw = (uint16_t)(rnd() & 1023); p.i_raw13 = (uint16_t)(rnd() & 1023);
                    for (int b = 0; b < 20; b++) p.raw_block[b] = (uint16_t)(rnd() & 1023);
                    p.raw_uniform = (uint16_t)(rnd() & 1023);
                    p.aux_v_raw = bth5_encodeAuxV(AUX_V_FIXED);
                    for (int i = 0; i < 4; i++) p.tb_raw[i] = (uint16_t)(bth5_encodeTemp(TEMP_SEND_MAX_C) + rnd() % (bth5_encodeTemp(TEMP_SEND_MIN_C) - bth5_encodeTemp(TEMP_SEND_MAX_C) + 1));
                    p.intake_raw = (uint16_t)(bth5_encodeTemp(TEMP_SEND_MAX_C) + rnd() % (bth5_encodeTemp(TEMP_SEND_MIN_C) - bth5_encodeTemp(TEMP_SEND_MAX_C) + 1));
                    if (fld == 4) p.intake_raw = (uint16_t)r; else
                    p.tb_raw[fld] = (uint16_t)r;
                    for (uint8_t idx = 3; idx < 30; idx += 6) { frame_case(&p, idx); frame_case(&p, (uint8_t)(idx + 1)); frame_case(&p, (uint8_t)(idx + 2)); }
                }
        /* clamp-range cross product TB1 x TB3 (both slot 4, b14 hi bits + q), realistic blocks/current */
        int rlo = bth5_encodeTemp(TEMP_SEND_MAX_C), rhi = bth5_encodeTemp(TEMP_SEND_MIN_C);
        for (int a = rlo; a <= rhi; a++)
            for (int c = rlo; c <= rhi; c++) {
                p.live = 1; p.i_raw = (uint16_t)(400 + rnd() % 300); p.i_raw13 = p.i_raw;
                for (int b = 0; b < 20; b++) p.raw_block[b] = (uint16_t)(560 + rnd() % 80);
                p.raw_uniform = 600;
                p.tb_raw[0] = (uint16_t)a; p.tb_raw[2] = (uint16_t)c; p.tb_raw[1] = (uint16_t)(rlo + rnd() % (rhi - rlo + 1)); p.tb_raw[3] = (uint16_t)(rlo + rnd() % (rhi - rlo + 1));
                for (uint8_t idx = 4; idx < 30; idx += 6) frame_case(&p, idx);
            }
        /* full bursts at every clamp-range temperature, all 30 indices */
        for (int r = bth5_encodeTemp(TEMP_SEND_MAX_C); r <= bth5_encodeTemp(TEMP_SEND_MIN_C); r++)
            for (int k = 0; k < 2000; k++) {
                p.live = 1; p.i_raw = (uint16_t)(rnd() & 1023); p.i_raw13 = (uint16_t)(rnd() & 1023);
                for (int b = 0; b < 20; b++) p.raw_block[b] = (uint16_t)(rnd() & 1023);
                p.raw_uniform = (uint16_t)(rnd() & 1023);
                p.intake_raw = (uint16_t)r;
                frame_case(&p, (uint8_t)(3 + 6 * (k % 5)));
            }
        /* hi-bit boundary 767 <-> 768: all four fields walk 760..776 and back one count per burst */
        for (int rep = 0; rep < 200; rep++)
            for (int k = 0; k < 34; k++) {
                int r = k < 17 ? 760 + k : 793 - k;
                p.live = 1; p.i_raw = (uint16_t)(rnd() & 1023); p.i_raw13 = (uint16_t)(rnd() & 1023);
                for (int b = 0; b < 20; b++) p.raw_block[b] = (uint16_t)(560 + rnd() % 80);
                for (int i = 0; i < 4; i++) p.tb_raw[i] = (uint16_t)(r + (i & 1));
                p.intake_raw = (uint16_t)r;
                for (uint8_t idx = 0; idx < 30; idx++) frame_case(&p, idx);
            }
        for (int t = (int)(TEMP_SEND_MIN_C * 100); t <= (int)(TEMP_SEND_MAX_C * 100); t++) {
            uint16_t raw = bth5_encodeTemp(t * 0.01f);
            p.live = 1; p.i_raw = (uint16_t)(rnd() & 1023); p.i_raw13 = (uint16_t)(rnd() & 1023);
            for (int b = 0; b < 20; b++) p.raw_block[b] = (uint16_t)(rnd() & 1023);
            for (int i = 0; i < 4; i++) p.tb_raw[i] = raw;
            p.intake_raw = raw;
            for (uint8_t idx = 0; idx < 30; idx++) frame_case(&p, idx);
        }
        printf("frame sweep: frames=%lu quotient_bad=%lu checksum_bad=%lu hdr_bad=%lu b14[5:4]_bad=%lu slot0_x13_bad=%lu field_bad=%lu tb_readback_bad=%lu other_bits_changed=%lu b10_nudged=%lu\n",
               nf2, n_q_bad, n_ck_bad, n_hdr_bad, n_tpl_bad, n_x13_bad, n_fld_bad, n_rb_bad, n_other_bad, n_nudge);
        T_CHECK(!n_q_bad && !n_ck_bad && !n_hdr_bad && !n_tpl_bad && !n_x13_bad && !n_fld_bad && !n_rb_bad && !n_other_bad, "frame sweep");
    }
    /* 3. filter */
    {
        bth5_tb_t s; uint16_t o[5] = {0}; uint32_t t = 1000; memset(&s, 0, sizeof s);
        bth5_tb_step(&s, o, 0, 0, 0, 0, t);
        T_CHECK(!o[0] && !o[1] && !o[2] && !o[3] && s.src == BTH5_TB_TEMPLATE, "no data -> template");
        bth5_tb_step(&s, o, 10, 55.0f, 55.0f, 55.0f, t += 10);
        T_CHECK(!o[0] && s.src == BTH5_TB_TEMPLATE, "55 C unused-channel fallback rejected");
        bth5_tb_step(&s, o, 10, 20.0f, -40.0f, 25.0f, t += 10);
        T_CHECK(!o[0], "-40 missing rejected");
        bth5_tb_step(&s, o, 10, 20.4f, 19.0f, 22.0f, t += 10);
        T_CHECK(s.src == BTH5_TB_LIVE && o[0] == bth5_encodeTemp(20.4f) && o[1] == bth5_encodeTemp(19.0f) && o[3] == bth5_encodeTemp(22.0f) && o[2] == o[0], "first valid reading taken directly");
        /* +5 C step on avg: must take 100 s at 0.05 C/s */
        float t0 = s.t[0]; int n = 0;
        while (s.t[0] < 25.39f && n < 20000) { bth5_tb_step(&s, o, 10, 25.4f, 24.0f, 27.0f, t += 10); n++; }
        T_CHECK(fabsf(n * 0.01f - 5.0f / TB_SLEW_C_PER_S) < 0.5f, "slew time %.2f s", n * 0.01f);
        printf("filter: %.1f -> 25.4 C took %.1f s (%.2f C/s)\n", t0, n * 0.01f, 5.0 / (n * 0.01));
        /* max raw change per 106 ms burst */
        { int mx = 0; uint16_t prev = o[0];
          for (int k = 0; k < 20000; k++) { bth5_tb_step(&s, o, 10, (k / 2000) % 2 ? 15.0f : 40.0f, 15.0f, 40.0f, t += 10);
              if (k % 11 == 0) { int d = abs((int)o[0] - (int)prev); if (d > mx) mx = d; prev = o[0]; } }
          printf("filter: max TB1 change per burst = %d count(s)\n", mx); T_CHECK(mx <= 1, "per-burst change"); }
        /* spread + clamp */
        for (int k = 0; k < 100000; k++) bth5_tb_step(&s, o, 10, 30.0f, 10.0f, 50.0f, t += 10);
        T_CHECK(o[1] == bth5_encodeTemp(25.0f) && o[3] == bth5_encodeTemp(35.0f), "spread +-5");
        T_CHECK(o[4] == o[1], "intake = TB2");
        for (int k = 0; k < 100000; k++) bth5_tb_step(&s, o, 10, TEMP_SEND_MIN_C - 10.0f, TEMP_SEND_MIN_C - 12.0f, TEMP_SEND_MIN_C - 7.0f, t += 10);
        T_CHECK(o[0] == bth5_encodeTemp(TEMP_SEND_MIN_C) && o[1] == o[0] && o[3] == o[0], "clamp floor");
        /* brief dropout (< TB_STALE_MS) holds, long dropout slews to template and ends at exact template */
        uint16_t held = o[0];
        for (int k = 0; k < 400; k++) bth5_tb_step(&s, o, 0, 0, 0, 0, t += 10);
        T_CHECK(o[0] == held, "hold during 4 s gap");
        n = 0;
        while ((o[0] || o[1] || o[2] || o[3] || o[4]) && n < 100000) { bth5_tb_step(&s, o, 0, 0, 0, 0, t += 10); n++; }
        T_CHECK(!o[0] && !o[1] && !o[2] && !o[3] && !o[4] && s.src == BTH5_TB_STALE, "stale -> exact template");
        printf("filter: stale -> template (from floor) in %.1f s after the %u ms hold\n", n * 0.01f, (unsigned)TB_STALE_MS);
        bth5_tb_step(&s, o, 10, 26.0f, 25.0f, 28.0f, t += 10);
        T_CHECK(s.src == BTH5_TB_LIVE && o[0] && abs((int)o[0] - 666) <= 3, "resume from template position, no jump (%u)", o[0]);
    }
    /* expected X431 readings */
    printf("expected X431 at uniform module temps (min=avg=max):\n");
    {   static const float u[] = {0, 5, 9, 14, 15, 25, 35, 40, 45};
        for (unsigned k = 0; k < sizeof u / sizeof u[0]; k++) {
            bth5_tb_t s; uint16_t o[5] = {0}; memset(&s, 0, sizeof s);
            bth5_tb_step(&s, o, 10, u[k], u[k], u[k], 100);
            printf("  %4.1f C -> raw %u (0x%03X, hi bits %u) -> X431 %.2f%s", u[k], o[0], o[0], o[0] >> 8, law(o[0]), "");
            printf("   intake raw %u -> X431 %.2f\n", o[4], law(o[4]));
        }
    }
    printf("expected X431 (TB1=avg TB2=min TB3=avg TB4=max, steady state):\n");
    {   static const float cases[][3] = {{10,9,11},{15,14,16},{18,17,19},{20,19,22},{22.5f,21,24},{25,24,27},{30,28,33},{35,33,38},{42,40,45}};
        for (unsigned k = 0; k < sizeof cases / sizeof cases[0]; k++) {
            bth5_tb_t s; uint16_t o[5] = {0}; memset(&s, 0, sizeof s);
            bth5_tb_step(&s, o, 10, cases[k][0], cases[k][1], cases[k][2], 100);
            printf("  modules min/avg/max %4.1f/%4.1f/%4.1f C -> raw %u/%u/%u/%u -> X431 TB1..4 %.2f/%.2f/%.2f/%.2f",
                   cases[k][1], cases[k][0], cases[k][2], o[0], o[1], o[2], o[3], law(o[0]), law(o[1]), law(o[2]), law(o[3]));
            printf("  intake %u -> %.2f", o[4], law(o[4]));
            printf("\n");
        }
    }
    printf(fails ? "FAIL (%d)\n" : "PASS\n", fails);
    return fails != 0;
}
