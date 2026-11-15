#!/bin/bash
# Build gs450h_bth_spoof.uf2 with the car defaults from src/config.h.
# Needs the Pico SDK 2.x, the Arm GNU toolchain and Ninja. Override the
# paths with PICO_SDK_PATH / PICO_TOOLCHAIN_PATH / PIOASM_DIR / PICOTOOL_DIR.
set -euo pipefail
cd "$(dirname "$0")"
export PICO_SDK_PATH="${PICO_SDK_PATH:-/home/box/pico/pico-sdk}"
export PICO_TOOLCHAIN_PATH="${PICO_TOOLCHAIN_PATH:-/home/box/pico/toolchain/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi}"
PIOASM_DIR="${PIOASM_DIR:-/home/box/pico/tools/pioasm}"
PICOTOOL_DIR="${PICOTOOL_DIR:-/home/box/pico/tools/picotool}"

cmake -S . -B build -G Ninja \
  -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350 \
  -Dpioasm_DIR="${PIOASM_DIR}" -Dpicotool_DIR="${PICOTOOL_DIR}"
cmake --build build
cp -f build/gs450h_bth_spoof.uf2 gs450h_bth_spoof.uf2
sha256sum gs450h_bth_spoof.uf2
