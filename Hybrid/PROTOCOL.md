# GS450h BTH link — protocol notes

The BTH link carries battery data from the battery smart unit (in the HV
battery) to the hybrid control ECU. It is one-way: the battery unit only
transmits.

Everything here comes from logic-analyser captures of the OEM unit (car and
bench), bench tests with a signal generator on the OEM board's current
input, and car tests with the Pico. Items marked **TODO** or *unknown* are
not confirmed.

---

## 1. Physical layer

- Two wires, BTH+ and BTH−, driven as a **complementary CMOS push-pull
  pair**. Not RS-485.
- OEM driver: Toshiba TC7W04 inverter (IC401). The MCU TX goes through 1 kΩ
  (47 kΩ pull-down) into two inverter inputs; one output is inverted again,
  so the two outputs are exact complements. Each output goes through 2.7 kΩ,
  with ~3.3 nF to ground (EMI filter).
- Measured on the car: about **±1.8 V** differential around a ~3.3 V common
  mode. Edges about 13 µs (10–90 %), set by the 2.7 kΩ and 3.3 nF.
- One line idles high, the other idles low. Getting the pair swapped stops
  the link.

## 2. Serial format

| Item | Value |
|---|---|
| Baud | 19,300 (OEM measured 19,302–19,303) |
| Format | 8 data bits, odd parity, 1 stop (8O1), idle = mark |
| Byte spacing | 1.020 ms start to start |
| Frame | 16 bytes |
| Burst | 6 frames = 96 bytes, ~96.9 ms, then ~9.2 ms idle |
| Burst period | 106.1 ms |
| Cycle | 5 bursts = 30 frames |

## 3. Frame layout

Counter `b0` runs **0x20–0x3D** and wraps. Each burst starts on 0x20, 0x26,
0x2C, 0x32 or 0x38. Slot = `(b0 − 0x20) mod 6`.

| Byte | Content |
|---|---|
| b0 | counter 0x20–0x3D |
| b1 | 0xFF − b0 |
| b2 | I2, low 8 bits (current) |
| b3 | V3, low 8 bits (block voltage) |
| b4 | V4, low 8 bits (block voltage) |
| b5 | A5, low 8 bits (aux / temperature, depends on slot) |
| b6 | high bits: [7:6] I2, [5:4] V3, [3:2] V4, [1:0] A5 |
| b7 | always 0 |
| b8 | I8, low 8 bits (current) |
| b9 | V9, low 8 bits (block voltage; 0 in slots 2 and 5) |
| b10 | V10, low 8 bits (block voltage) |
| b11 | A11, low 8 bits (aux / temperature) |
| b12 | high bits: [7:6] I8, [5:4] V9, [3:2] V10, [1:0] A11 |
| b13 | X13, low 8 bits |
| b14 | [7:6] X13 high bits, [5:4] flags (`10`), [3:0] checksum high nibble |
| b15 | checksum low byte |

All analog fields are **10-bit**: low 8 bits in their own byte, high 2 bits
in b6, b12 or b14[7:6].

Exception: in frame 0x3D only, slot 5's V9 carries a 23rd block voltage and
b12[5:4] becomes `10`.

## 4. Checksum

The checksum is a **12-bit sum** split across two bytes.

```
S0       = sum(b0..b13) + (b14 & 0xF0)
b14[3:0] = S0 >> 8
b15      = S0 & 0xFF
```

Written another way, with S = sum(b0..b14) including the nibble:

```
b14[3:0] = floor(S / 257)
b15      = S mod 257
```

Both forms are the same thing. This holds on 4,618/4,618 OEM frames. The
high nibble is 3–8 on real data and can never overflow (S0 ≤ 3,300).

Early firmware only had `b15 = S mod 257` right and kept b14[3:0] from the
template. That is wrong whenever the byte sum moves far enough, which
happens under high current. See HISTORY.md.

## 5. Values

### Block voltage

```
V_block = raw10 × 25 / 1023      (no offset)
```

22 block fields per burst (23 in the burst with 0x3D). The GS450h has 20
blocks. The firmware sends 20 blocks of 4 BMW cells each; the field-to-block
order is a placeholder (TODO: confirm), and unmapped fields get the average.

### Current

- 10-bit, **0 A = 511 (0x1FF)**. Discharge is positive and gives lower raw
  values; charge gives higher raw values.
- Present in I2 and I8 in all six slots (12 copies per burst).
- On the OEM unit all 12 are fresh samples (confirmed with a 2 Hz sine on the
  bench). The Pico sends one sample in all 12.

Firmware encoding:

```
raw = 511 − round(I / 0.49)      I in A, discharge positive
```

The nominal scale is ~0.5 A/count. The OEM board reads the sensor on a plain
0–5 V 10-bit input (raw ≈ V × 1023/5 − 1, i.e. 204.6 counts/V). With the
OEM sensor at ~10 mV/A that gives 0.489 A/count, which matches 0.49.

The X431 displays current as `(512 − raw) × 0.49 A`, so it shows +0.49 A when
the field is at true zero (511).

Pre-RUN: early tests showed the ECU rejecting raw ≥ 0x200 (charge side)
with the contactors open. Those builds also had checksum errors on exactly
those frames, so the real cause is not certain. The firmware still holds
charge at 0x1FF until an engine start is seen.

### Slot-0 X13 (third current copy)

```
X13 = floor(mean of the 12 current fields of the previous burst)
```

Confirmed on 724/724 checkable OEM slot-0 frames, including transients.

### Temperatures

TB1–TB4 and battery intake air use one NTC law (10-bit ADC, thermistor to a
pull-up, higher raw = colder):

```
ln(raw / (1023 − raw)) = −10.389791 + 3297.8588 / T_K
T(°C) = 3297.8588 / (ln(raw / (1023 − raw)) + 10.389791) − 273.15
```

Fitted to five X431 readings (14.46–28.15 °C), max error 0.008 °C. Below
~14 °C or above ~28 °C it is an extrapolation.

| raw | °C |
|---|---|
| 831 | 5.0 |
| 804 | 9.0 |
| 677 | 25.0 |
| 591 | 35.0 |
| 548 | 40.0 |

### Aux (12 V) battery voltage

Slot 3 X13. About **50.6 counts/V** (673 = 13.3 V, the firmware's
calibration). TODO: only loosely calibrated; one Techstream point
(449 = 8.6 V) gives ~52 counts/V.

## 6. Field map

| Field | Location | Notes |
|---|---|---|
| Current | I2, I8, all slots | see above |
| Current (lagged mean) | slot 0 X13 | see above |
| Block voltages | V3, V4, V9, V10 | V9 is 0 in slots 2 and 5, except frame 0x3D |
| TB1 | slot 4 A5 | |
| TB2 | slot 4 A11 | |
| TB3 | slot 4 X13 | |
| TB4 | slot 5 A5 | |
| Intake air ("Inhaling Air") | slot 3 A5 | |
| Aux 12 V voltage | slot 3 X13 | |

The five temperature fields read 1023 on the bench with the sensors open.

Constant fields, meaning unknown (sent as in the OEM template):

| Slot | A5 | A11 | X13 |
|---|---|---|---|
| 0 | 511 | 0 | (current) |
| 1 | 599 | 725 | 1023 |
| 2 | 0 or 87–89 | 110 or 170 | 0 |
| 3 | (intake) | 0 | (aux V) |
| 5 | (TB4) | 0 | 0 |

## 7. OEM current sensor input

The OEM battery unit reads the pack current sensor on a plain 0–5 V input,
2.5 V = 0 A, ~10 mV/A, 0.5–4.5 V for ±200 A. Higher voltage = charge
(higher raw). No
filtering or calibration delay: it tracks from the first frame after
power-up.

## 8. Unknowns

- **Slot 1 X13** sits at 1023 (railed). Earlier we linked this to P0517;
  that was wrong (P0517 is the 12 V battery temperature sensor wired to the
  ECU). Meaning unknown.
- **b14[5:4]** is `10` in every OEM frame except slot 1, which is `11` in
  some captures (mostly bench). Possibly a self-test or status flag. Not
  confirmed.
- **ECU bad-frame tolerance.** The ECU tolerates short runs of bad frames
  (one run of 24 consecutive bad frames survived), but the threshold and
  timing are not known. Toyota does not publish them.
- **Exact current scale** (0.49 vs 0.5 A/count) has not been checked against
  a calibrated load.
- **Block field order** against physical blocks.
- Other monitors on this link per Toyota documents (cooling fan voltage,
  insulation leakage) have not been located in the frame.
