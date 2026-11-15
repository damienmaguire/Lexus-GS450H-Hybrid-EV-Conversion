# gs450h_bth_spoof

Firmware for a Raspberry Pi Pico 2 (RP2350). It stands in for the battery
sense board of a Lexus GS450h with a BMW Gen1 PHEV pack. It reads five BMW
modules over CAN and a Hall current sensor, then sends the 16-byte "BTH"
frames the HV ECU expects. It reproduces the OEM board's timing, checksum
and field layout.

This is the build tested on the car on 15 Nov 2026 (development name
`i18qnb_aux_db05_tb5i`): READY, D and R, hard launches, zero DTCs.
All of its settings are the defaults in `src/config.h`, so a plain build
gives the same firmware.

> **WARNING — do not have USB connected to the Pico at key-on.** A USB host
> connected at key-on faults the car. Flash it and read the logs with the
> car off, then unplug USB before switching on.
>
> **HV safety.** The firmware sends made-up battery data to the ECU, which
> can close the main contactors on it. Use it only with a known-safe pack,
> fused HV, and a way to cut 12 V / IG at once. Disconnect the original
> battery sense board from BTH+ / BTH- first.

## Build

Needs the Pico SDK 2.x (tested with 2.3.1), the Arm GNU toolchain (tested
with 14.2.rel1), CMake and Ninja.

```
./build.sh
```

This writes `gs450h_bth_spoof.uf2` next to the script and prints its
sha256. The SDK and tool paths default to the ones on the build box. Change
them with `PICO_SDK_PATH`, `PICO_TOOLCHAIN_PATH`, `PIOASM_DIR` and
`PICOTOOL_DIR`. It is a normal Pico CMake project (`PICO_BOARD=pico2`,
Release, `-O3`), so `cmake -B build -G Ninja && cmake --build build` works
too.

To try a different setting without editing the source:
`cmake -B build -DBTH_DEFS="RUN_DB_DA=10;AUX_V_FIXED=13.0f"`. Leave
`BTH_DEFS` empty for the car build.

Build used to make the released `gs450h_bth_spoof.uf2`:
sha256 `edf120f1bc51676157b54a104f54b172ae7af8886939af4a05d45cac423a4bed`.

## Flash

1. With the car OFF, hold BOOTSEL on the Pico 2 and plug in USB.
2. Copy `gs450h_bth_spoof.uf2` to the `RP2350` drive that appears (or run
   `picotool load -x gs450h_bth_spoof.uf2`).
3. Unplug USB before key-on.

## Wiring

| Pico pin | Use |
|---|---|
| GP0 (pin 1) | BTH TX, UART0 19300 Bd 8O1 → 74HCT04 at 5 V, wired like the OEM TC7W04: complementary outputs through 2.7 kΩ to BTH+ / BTH- |
| GP26 / ADC0 (pin 31) | Hall current sensor (2.5 V at 0 A, 10 mV/A) through a 10 k / 20 k divider |
| GP16 / 17 / 18 / 19 | SPI0 to the MCP2518FD: MISO / CS / SCK / MOSI, 8 MHz |
| GP15 | MCP2518FD INT |
| CAN | BMW modules, 500 kbit/s, MCP2518FD with a 20 MHz crystal |
| USB | Status line and log dump only (never at key-on) |

## Reading the log (car off, USB connected)

The Pico keeps a rolling log of every burst (about the last 7 minutes) in
flash. It writes nothing while a USB host is connected or in the first 3 s
after power-up. Open the USB serial port and type:

- `d`: the previous boot (the car run), as CSV
- `t`: the last 100 rows of that boot
- `a`: everything in the ring

## What it sends (summary)

- 6 frames per burst, one burst every 106.1 ms (1020 µs per byte, 8.18 ms
  gap). The counter runs 0x20..0x3D. Frames start from a real OEM capture
  (`oem_template.h`) with live fields written in.
- Block voltages: 20 blocks of 4 BMW cells, 10-bit, 25/1023 V per count.
  The template blocks are sent until all 5 modules report. After that, the
  last good values are held if a module drops out.
- Current: raw = 511 − I/0.49 (discharge positive), 0x1FF = 0 A, with a
  0.5 A deadband. Before the engine-start detector reaches RUN, charge is
  held at 0x1FF. The sensor is auto-zeroed over the first 5 bursts, with a
  fallback of 2565 mV.
- Slot 0 X13 is the floor of the mean of the previous burst's 12 current
  fields, as the OEM board does.
- Checksum: b14[3:0] = S div 257 and b15 = S mod 257.
- Temperatures: TB1 = TB3 = module average, TB2 = min, TB4 = max (each
  kept within ±5 °C of the average), intake air = TB2. Clamped to 5–40 °C,
  slewed at no more than 0.05 °C/s, and back to the template values if the
  module temps go stale.
- 12 V aux battery: a fixed 12.6 V.

## Defaults (`src/config.h`)

| Setting | Default | Meaning |
|---|---|---|
| `ADC_SAMPLES` | 256 | ADC reads averaged per burst |
| `MV_PER_AMP` | 10.0 | Hall sensor gain |
| `CURRENT_ZERO_MV` | 2565 | fallback sensor zero (mV) |
| `CURRENT_SIGN` | +1 | sensor below zero = discharge |
| `AUTO_ZERO_N` / `_MIN_MV` / `_MAX_MV` | 5 / 2400 / 2700 | auto-zero window and accepted range |
| `SENSOR_FAULT_LO_MV` / `_HI_MV` | 400 / 4600 | outside = sensor fault, 0 A sent |
| `SENSOR_FAULT_SET_N` / `_CLR_N` | 5 / 5 | samples to set / clear the fault |
| `X431_I_ZERO_RAW` / `X431_A_PER_COUNT` | 511 / 0.49 | current encoding |
| `CURRENT_DIS_DB_DA` / `CURRENT_CHG_DB_DA` / `RUN_DB_DA` | 5 / 5 / 5 | deadbands, tenths of an amp |
| `ENG_CRANK_DA` / `ENG_CRANK_END_DA` / `ENG_SETTLE_N` | 250 / 150 / 5 | engine-start detector: 25 A in, 15 A out, 5 bursts settle |
| `AUX_V_FIXED` | 12.6 V | aux battery voltage sent |
| `TEMP_BETA_K` / `TEMP_BETA_LNK` | 3297.8588 / −10.389791 | temperature law |
| `TEMP_SEND_MIN_C` / `_MAX_C` | 5 / 40 °C | sent temperature clamp |
| `TB_SLEW_C_PER_S` | 0.05 | max temperature change per second |
| `TB_STALE_MS` | 5000 | no module temps for this long → template |
| `TB_SPREAD_MAX_C` | 5 | TB2 / TB4 limit around the average |
| `B14Q_KEEP_LEGACY` | 1 | prefer the older b14 encoding when it is also valid |
| `B14_X13_LO` | 482 | older b14 rule threshold |
| `EXPECTED_MODULES` | 5 | BMW modules |
| `BYTE_PERIOD_US` / `BURST_GAP_US` | 1020 / 8180 | BTH timing |
| `ZERO_META_KEEP` | 6 | auto-zero records kept in flash |
| `ERASE_DEFER_A` | 10 A | no flash erase above this current (or while cranking) |

## File map

| File | What it does |
|---|---|
| `src/main.c` | Start-up, the core 0 BTH sender loop, current sampling, the core 1 CAN loop |
| `src/config.h` | Every tunable value, with the car-tested defaults |
| `src/bth_frames.h` | Builds each 16-byte frame: template, live fields, two-part checksum |
| `src/bth_link.h` | Frame layout helpers: 10-bit fields, block encoding, header bytes |
| `src/oem_template.h` | 30 real frames from a healthy OEM board, used as the base |
| `src/current_sensor.h` | Hall sensor reading, current encoding, deadband policy, engine-start detector |
| `src/temps_aux.h` | TB1-4 / intake temperatures (law, filter) and the aux voltage field |
| `src/pack_to_bth.h` | BMW pack model → BTH block voltages and log values |
| `src/bmw_pack.c` / `.h` | Decodes BMW module CAN frames: cells (with glitch guard), temps, totals |
| `src/bmw_poll.h` | The module poll frame and its CRC |
| `src/mcp2518fd.c` / `.h` | Small MCP2518FD CAN driver |
| `src/flash_log.c` | Flash log on the Pico: boot records, ring writes between bursts, USB dump |
| `src/flash_log.h` | Log record layouts, meta sector, timing budget checks |
| `src/ring_log.h` | Ring log logic that can be tested on a PC |
| `tests/run_tests.sh` | Builds and runs all host tests with gcc; prints ALL PASS |
| `tests/test_frames.c` | Frame and checksum checks over car log replays and full sweeps |
| `tests/test_temps.c` | Temperature law, TB / intake field sweep, filter |
| `tests/test_current_replay.c` | Replays a logged drive through the current policy |
| `tests/test_deadband.c` | Deadband / no-charge-before-RUN sweep |
| `tests/test_zero_meta.c` | Auto-zero records in the flash meta sector |
| `tests/data/` | Ring-log sequences from car boots 68–86 and 101 |

## Tests

```
tests/run_tests.sh
```

Plain gcc, no SDK needed. Set `FRAMES_DIR=/some/dir` to keep the frame
dumps.

## Licence

GPL-3.0
