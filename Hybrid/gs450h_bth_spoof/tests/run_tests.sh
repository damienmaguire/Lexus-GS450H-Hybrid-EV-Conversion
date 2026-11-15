#!/bin/bash
# Host tests for gs450h_bth_spoof. Plain gcc, no Pico SDK needed. The tests
# include the real firmware headers from ../src with the config.h defaults.
#  1. frames: car log replays (boots 68..86, and boot 101 re-run through the
#     current policy), each with temperature spread off/on, plus a full sweep;
#     built with and without B14Q_KEEP_LEGACY.
#  2. temps: temperature law, TB/intake field sweep, filter behaviour.
#  3. current deadband / pre-RUN policy, auto-zero meta records.
# Prints ALL PASS and exits 0 on success.
set -euo pipefail
cd "$(dirname "$0")"
O=${OUT:-/tmp/gs450h_bth_spoof_tests}; mkdir -p "$O"; fail=0
I="-I../src"
gcc -O2 -Wall -Wextra $I -o $O/tq test_frames.c -lm
gcc -O2 -Wall -Wextra $I -DB14Q_KEEP_LEGACY=0 -o $O/tq_pure test_frames.c -lm
gcc -O2 -Wall -Wextra $I -o $O/replay test_current_replay.c -lm
gcc -O2 -Wall -Wextra $I -o $O/temps test_temps.c -lm
gcc -O2 -Wall -Wextra $I -o $O/deadband test_deadband.c -lm
gcc -O2 -Wall -Wextra $I -o $O/zero_meta test_zero_meta.c
FR=${FRAMES_DIR:-/dev/null}   # set FRAMES_DIR to keep the frame dumps
fout() { [ "$FR" = /dev/null ] && echo /dev/null || echo "$FR/$1"; }
[ "$FR" = /dev/null ] || mkdir -p "$FR"
echo "== 1. frames / checksum =="
$O/replay data/b101_policy.csv $O/seq101.csv /dev/null 2>$O/replay.txt; echo "b101 policy: $(cat $O/replay.txt)"
for sp in 0 1; do
  for b in 68 70 72 76 78 79 81 83 85 86; do
    for v in tq tq_pure; do r=$($O/$v seq data/b$b.csv $sp $(fout b${b}_s${sp}_$v.txt) | tr '\n' ' '); echo "b$b s$sp $v: $r"; grep -q PASS <<<"$r" || fail=1; done
  done
  for v in tq tq_pure; do r=$($O/$v seq $O/seq101.csv $sp $(fout b101_s${sp}_$v.txt) | tr '\n' ' '); echo "b101 s$sp $v: $r"; grep -q PASS <<<"$r" || fail=1; done
done
for v in tq tq_pure; do r=$($O/$v sweep $(fout sweep_$v.bin) | tr '\n' ' '); echo "sweep $v: $r"; grep -q PASS <<<"$r" || fail=1; done
echo "== 2. temperatures =="
$O/temps || fail=1
echo "== 3. current deadband / zero meta =="
$O/deadband || fail=1
$O/zero_meta || fail=1
[ $fail = 0 ] && echo "ALL PASS" || { echo FAIL; exit 1; }
