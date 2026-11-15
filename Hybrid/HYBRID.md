# Lexus GS450h battery ECU (BTH) emulator — BMW PHEV modules

<img width="4096" height="2304" alt="Display" src="https://github.com/user-attachments/assets/99dc225f-0a37-43ec-a611-712532ac805f" />


Replaces the battery smart unit ("BTH" link) of a 2006 Lexus GS450h with a
Raspberry Pi Pico 2. The Pico reads five BMW Gen1 PHEV battery modules over
CAN, reads pack current from a the Lexus OEM sensor, and sends the hybrid control ECU
the serial frames it expects from the original Toyota battery.

Protocol details are in [PROTOCOL.md](PROTOCOL.md). The debugging history is
in [HISTORY.md](HISTORY.md).

---

## Status (15 Nov 2026)

Working on the car:

- READY, drives in D and R.
- Hard launches with no trips.
- Zero DTCs on Hybrid Control (X431).
- Energy monitor working.
- Live pack current from the Hall sensor.
- Live block voltages from the BMW modules (20 blocks of 4 cells).
- Live battery temperatures TB1–TB4 and battery intake air, from the module
  temperatures. At ~9 °C ambient the X431 read TB1/TB3 11.55 °C, TB2
  11.00 °C, TB4 11.96 °C, Inhaling Air 11 °C.
- Aux (12 V) battery voltage reported as a fixed 12.6 V.

Not solved yet: the ECU's SOC estimate is low, so it runs long recovery
charges (~8 A with the engine running). See [Known issues](#known-issues).

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 (RP2350) | Runs the firmware. Core 1 handles CAN, core 0 sends BTH. |
| MCP2518FD CAN controller | SPI0, 500 kbit/s, to the BMW modules. |
| 5 × BMW Gen1 PHEV modules (16s) | 80 cells. Voltages and temperatures read over CAN. |
| Hall current sensor | Prius-style, 2.5 V at 0 A, 10 mV/A, on the HV pack lead. Auto-zeroed at power-up. |
| 74HCT04 at 5 V | BTH line driver, wired like the OEM board's TC7W04 (IC401). |

### Pico pin map

| Function | Pico 2 pin |
|---|---|
| BTH UART0 TX (to 74HCT04) | GP0 (pin 1) |
| Current sensor in (ADC0) | GP26 (pin 31), via 10 kΩ / 20 kΩ divider and 10nF filter cap |
| MCP2518FD MISO / CS / SCK / MOSI | GP16 / GP17 / GP18 / GP19 (SPI0, 8 MHz) |
| MCP2518FD INT | GP15 |
| Status LED | onboard, toggles each burst |
| USB | serial log dump only (see warning below) |

### BTH line driver (74HCT04)

The OEM board drives BTH+/BTH− from a TC7W04 CMOS inverter through 2.7 kΩ
resistors. It is not RS-485. The 74HCT04 copies that circuit:

- GP0 → 1 kΩ → pin 1 (1A). 47 kΩ from pin 1 to GND.
- Pin 2 (1Y) → pin 3 (2A).
- Pin 2 (1Y, idles low) → 2.7 kΩ → line A.
- Pin 4 (2Y, idles high) → 2.7 kΩ → line B.
- ~3.3 nF from each line to GND (as in the OEM EMI filter).
- 100 nF across VCC (pin 14) and GND (pin 7). VCC = 5 V.
- Tie unused inputs (pins 5, 9, 11, 13) to GND.
- Common ground between Pico, 74HCT04, current sensor and ECU.

**Polarity matters.** Line B (pin 4, idles high) goes to the wire the OEM's
idle-high output drove; line A to the idle-low wire. With the Pico idling,
B − A should read about +4 V. About −0.6 V means the pair is swapped.

### Current sensor

```
sensor out ── 10 kΩ ──┬── 20 kΩ ── GND
                      │
                    GP26
```

Sensor supply 5 V, common ground. 0.5–4.5 V at the sensor becomes 0.33–3.0 V
at the pin. The firmware reads 256 ADC samples per burst and auto-zeroes on
the first 5 bursts after power-up (fallback 2565 mV).

### CAN

- Polls the modules on 0x080–0x085, one ID every 10 ms (same poll as
  stm32_betty).
- Cell voltages from 0x12N–0x17N. Temperatures from 0x18N, byte − 40 °C;
  −40 (missing) and ~55 °C (unused thermistor) are ignored. This matches
  stm32_betty.

## Building

Needs the Pico SDK (tested with the Arm GNU toolchain 14.2) and Ninja.

```sh
cd pico2/bth_pico2_phev_v4m
export PICO_SDK_PATH=/path/to/pico-sdk
export PICO_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain
./build_i18qnb_aux_db05_tb5i.sh
```

The script passes the configuration as `V5_DEFS` and prints the UF2's sha256.
**A plain `cmake` without `V5_DEFS` builds old defaults that do not work on
the car.** Use the script.

### Configuration of the current build

| Area | Setting |
|---|---|
| Frames | OEM healthy-car template (`TEMPLATE_H=1`), live fields written over it |
| Checksum | Two-part checksum solved per frame (`B14_QUOTIENT=1`), no b10 nudge (`AVOID_BORROW=0`) |
| Current | Live (`LIVE_CURRENT=1`), sign +1, 10 mV/A, zero 511 counts, 0.49 A/count, also in slot-0 X13 |
| Auto-zero | First 5 bursts, fallback 2565 mV, result saved to a flash sector the ring log doesn't touch (`ZERO_META=1`) |
| Deadband | 0.5 A both sides in RUN. Before an engine start is seen, discharge goes out live above 0.5 A and charge is held at 0 A (`START_DETECT=1`) |
| Filtering | None (`CURRENT_FILTER_ALPHA=1`); noise handled by 256-sample ADC mean |
| Cell guard | On, 14-bit cell word mask (`CELL_GUARD=1`, `CELL_MASK14=1`) |
| Temperatures | TB1–4 and intake from module temps, NTC beta law, clamped 5–40 °C, slew 0.05 °C/s, template fallback after 5 s without data |
| Aux 12 V | Fixed 12.6 V (`AUX_V_FIXED`), scale 13.3 V = 673 counts |
| Logging | Flash ring log (~7 min) and boot log |

## Flashing

1. Hold BOOTSEL, plug the Pico into a computer, release.
2. Copy the `.uf2` onto the `RP2350` drive.
3. Unplug and fit in the car **with no USB cable attached**. USB connected
   at key-on faults the car.

## Reading the log

The firmware keeps a rolling log of the last ~7 minutes in flash.

1. Key off, take the Pico out, plug it into a laptop.
2. Open the serial port, e.g. `screen -L -Logfile ringdump.txt /dev/ttyACM0 115200`.
3. Type `d` (previous boot), `t` (last 100 rows) or `a` (all boots).

The header shows the build, boot count and the saved key-on current zeros.
The laptop boot never overwrites the car log. Column details:
`pico2/DUMP.md`.

## Known issues

- **Low SOC estimate / long charges.** The ECU's SOC estimate is low,
  probably from earlier builds that under-reported charge. It runs long
  ~8 A recovery charges with the engine on. Our pack is much larger than the
  OEM one, so this takes hours. Possible fix: scale the block voltages so
  the ECU sees a normal SOC.
- **Noise on the BTH wiring.** Heavy-load trips stopped once the checksum
  was fixed, but line noise was never ruled out. Keep the wiring short and
  the grounds common.
- **Aux 12 V voltage is fixed** at 12.6 V. The Pico doesn't measure it.
- **Unknown fields** are sent unchanged from the OEM template (see
  PROTOCOL.md).
- **No fail-safe on lost module data.** If CAN or a module drops out, the
  last block voltages are held. TODO: stop BTH after a timeout.
- **No watchdog.** TODO.
- **Block order:** which BTH field maps to which physical block is a
  placeholder order. TODO: confirm.
- Cold pack: temperatures below 5 °C are sent as 5 °C.
- The ring log only holds ~7 minutes.

## Next steps

- Work out the block-voltage to SOC relationship and fix the long charges.
- Measure the 12 V battery and send the real value.
- Add the module-data timeout and a watchdog.
- Identify the unknown fields (slot 1 X13, b14[5:4]).

