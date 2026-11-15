/* test_current_replay.c -- replay logged amps + engine state through the
 * real bth5_encodeCurrent() + bth5_applyCurrentPolicy() (deadbands from config.h).
 * Input:  t_ms,amps,eng,i_raw_log,x13_log,b14_log,pack_mv,blk0_mv,blk19_mv,latch
 * Output: a seq csv (t_ms,i_raw,x13_log,b14_log,pack_mv,blk0_mv,blk19_mv) for test_frames,
 *         and on stderr: rows, rows where i_raw != logged i_raw, state counts. */
#include <stdio.h>
#include <string.h>
#include "bth_frames.h"
static const char *stn(int s) { return s == BTH5_IST_DB ? "DB" : s == BTH5_IST_DIS ? "DIS" : s == BTH5_IST_CHG ? "CHG" : s == BTH5_IST_CHGH ? "CHGH" : "?"; }
int main(int argc, char **argv) {
    FILE *in = fopen(argv[1], "r"), *out = fopen(argv[2], "w"), *rows = argc > 3 ? fopen(argv[3], "w") : NULL;
    char line[256]; long n = 0, diff = 0, cnt[8] = {0};
    if (!in || !out) return 2;
    if (!fgets(line, sizeof line, in)) return 2;
    fprintf(out, "t_ms,i_raw,x13_log,b14_log,pack_mv,blk0_mv,blk19_mv\n");
    while (fgets(line, sizeof line, in)) {
        long t, eng, irl, x13, b14, pmv, b0, b19; float a; char lat[8];
        if (sscanf(line, "%ld,%f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%7s", &t, &a, &eng, &irl, &x13, &b14, &pmv, &b0, &b19, lat) != 10) continue;
        bth5_eng_t e = { (int)eng, 0, 0x1FF }; int sl;
        uint16_t r = bth5_encodeCurrent(a), r13 = bth5_encodeCurrentFloor(a);
        int st = bth5_applyCurrentPolicy(&r, &r13, a, &e, &sl);
        cnt[st]++; n++;
        if ((long)r != irl) diff++;
        fprintf(out, "%ld,%u,%ld,%ld,%ld,%ld,%ld\n", t, r, x13, b14, pmv, b0, b19);
        if (rows) fprintf(rows, "%ld,%.2f,%u,%s,%ld,%s\n", t, a, r, stn(st), irl, lat);
    }
    fprintf(stderr, "RUN_DB=%.1f A DIS_DB=%.1f A CHG_DB=%.1f A rows=%ld i_raw!=logged=%ld DB=%ld DIS=%ld CHG=%ld CHGH=%ld\n",
            RUN_DB_DA * 0.1, CURRENT_DIS_DB_DA * 0.1, CURRENT_CHG_DB_DA * 0.1, n, diff,
            cnt[BTH5_IST_DB], cnt[BTH5_IST_DIS], cnt[BTH5_IST_CHG], cnt[BTH5_IST_CHGH]);
    return 0;
}
