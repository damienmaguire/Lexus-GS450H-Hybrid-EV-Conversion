# Findings log

Short summary of the main debugging steps, in order. Dates are 2026.
Detailed notes are in `analysis/`.

## Starting point (early Nov)

- Captured the OEM BTH link with a logic analyser. First assumed RS-485;
  tracing the OEM board showed a TC7W04 CMOS push-pull driver with 2.7 kΩ
  outputs instead.
- Framing worked out: 19,300 Bd 8O1, 16-byte frames, 6 per burst, 106.1 ms.
- The forum READY frame buffers were the starting point for the frame
  template. They had already got another car to READY. Later
  replaced by frames from a healthy-car capture (`bth_h1252.h`).

## Live block voltages

- Pico 2 + MCP2518FD reading 5 BMW Gen1 PHEV modules. 20 blocks of 4 cells
  sent as live block voltages. Accepted by the ECU.

## Current: zero and start-up bug (13–14 Nov)

- Static current probes: 0x1FC, 0x1FE, 0x1FF accepted with contactors open;
  0x200 rejected. OEM 0 A is 0x1FF, not 0x200 (512 is only the X431's
  display zero).
- Sensor scale corrected from 20 mV/A to Toyota's 10 mV/A.
- **Zero:** the hard-coded 2525 mV zero was really about 4 A off. The sensor
  reads ~2565 mV at rest on the car. Older builds hid this behind a
  start-up "fault" delay that was meant to be 0.5 s but was actually 5.3 s
  (50 samples at one per burst), which also forced 0 A through the crank.
- Fix: removed the start-up delay and added auto-zero over the first 5
  bursts (fallback 2565 mV). Later builds also save the zero to flash.

## Current: sign (13 Nov, rechecked 15 Nov)

- With the engine running, adding 12 V load made the reading move towards
  charge, so the first live build (CURRENT_SIGN = −1) was inverted. Set back
  to +1.
- 15 Nov: rechecked against pack voltage over a full log. Pack voltage drops
  on discharge spikes and rises on charge spikes (about −0.13 Ω slope with
  discharge-positive current). Sign is correct.

## Checksum quotient (14 Nov)

- The checksum was believed to be `b15 = sum mod 257`, with b14's low bits
  treated as a status value that changed with current.
- A 2 Hz sine into the OEM board's current input showed b14[3:0] is part of
  the checksum: `b14[3:0] = floor(S/257)`, `b15 = S mod 257`, i.e. a plain
  12-bit sum. Holds on 4,618/4,618 OEM frames. Cross-checked independently
  by the team.
- Our builds kept b14[3:0] from the template, so frames were wrong when the
  byte sum moved: 3.9 % of frames overall, 36.7 % just before trips, and
  almost all frames at high current. Errors get worse the further current is
  from zero.
- This also explains the early "ECU rejects charge" results: static 0x200
  and similar probes sent 100 % bad frames.
- Fixed in build i18q (quotient solved per frame). i18qnb dropped an old
  b10 "borrow" nudge that had no OEM basis.

## Trips under load and the noise theory (14–15 Nov)

- Heavy-load events (launches, kickdown, wheelspin) caused P3105 comms trips
  on discharge and charge.
- Two candidate causes: noise on the BTH line at high inverter power, and
  the checksum errors (which rise with current). Trip logs alone could not
  separate them. Conclusion: likely both.
- After the checksum fix (i18qnb, 15 Nov): no trips, including hard
  launches in D and R. Noise is not ruled out.

## Temperatures and aux voltage (14–15 Nov)

- Found the TB1–TB4, intake air and aux 12 V fields by matching Techstream
  and X431 readings against the template raws.
- Template sent the aux battery as 8.6 V (captured during a dip). Now fixed
  at 12.6 V.
- A linear temperature law fitted 25–28 °C but read 0.57 °C wrong at
  14.46 °C. Replaced with an NTC beta law fitted to five points.
- TB1–TB4 and intake now come from the BMW module temperatures
  (byte − 40 °C, same decode as stm32_betty).
- P0517 turned out to be the 12 V battery temperature sensor wired to the
  ECU, not a BTH field.

## 15 Nov 2026 — current state

Build `i18qnb_aux_db05_tb5i`:

- READY, drives in D and R, hard launches with no trips, zero DTCs.
- Energy monitor working.
- Live current, live block voltages, live TB1–4 and intake air. X431 at
  ~9 °C ambient: TB1/TB3 11.55 °C, TB2 11.00 °C, TB4 11.96 °C, Inhaling
  Air 11 °C.
- Aux fixed at 12.6 V.
- The ECU ran a long ~8 A recovery charge with the engine on. Its SOC
  estimate is low, most likely because earlier builds under-reported
  charge (discharge-only builds, a 2 A deadband, runs cut short by trips).
  Deadband reduced to 0.5 A.
