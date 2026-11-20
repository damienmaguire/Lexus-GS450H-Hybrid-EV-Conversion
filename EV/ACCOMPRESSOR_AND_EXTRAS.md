# GS450h HV-ECU (`89983-30200`) — A/C Compressor Link + Extras

**Tooling:** Ghidra 12.2 headless (V850:LE:32:default) + radare2 6.2.5 v850 — both run **only** on our descrambled image. No third-party RE'd source used.
**Firmware markers:** cal string `89983-30200-` @ `0x00076C`; build tag `GWV1.00.S3 050608` (2005-06-08) @ `0x01B5AC`.

Confidence: **[C]** certain (direct code/register proof) · **[I]** inferred (strong code logic) · **[H]** hypothesis (needs confirmation).

---

## PART 1 — A/C COMPRESSOR SERIAL LINK

### 1.1 Which peripheral / channel  [C]
The electric-A/C / aux link is **V850 UARTA channel 1**, register block base `0xFFFFFA20`.
This chip lays UARTA channels out on a **0x20 stride** (5 channels: UA0`0xFA00`, UA1`0xFA20`, UA2`0xFA40`, UA3`0xFA60`, UA4`0xFA80`) — i.e. NOT the 0x10 stride of the small V850ES parts. Registers are byte-wide at even offsets inside each block:

| Offset | Reg | ch1 addr | Evidence |
|---|---|---|---|
| +0x00 | CTL0 (power/TXE/RXE/dir/parity/len/stop) | `0xFFFFFA20` | init writes |
| +0x02 | CTL1/CTL2 (clock+baud divisor) | `0xFFFFFA22` | init + runtime reconfig |
| +0x06 | RX data (UAnRX) | `0xFFFFFA26` | read @ `0xAFEC2` (ld.h) |
| +0x08 | STR status (err bits [2:0]) | `0xFFFFFA28` | poll @ `0xAFDBA`; ch0 ISR masks `&7` @ `0x086022` |
| +0x0C | TX data (UAnTX) | `0xFFFFFA2C` | writes 0x55/0xAA @ `0xAFD9A`,`0xAFDAC` |
| +0x10/+0x12 | baud-gen compare (16-bit) | `0xFFFFFA30/32` | init writes |

Init: `FUN_ram_0002aa82` @ `0x02AA82` (all 5 UARTA ch + 3 CSIB ch). Re-config of ch1 at runtime: `FUN_ram_0002fbc4` @ `0x02FBC4` (also arms DMA, see 1.4).

### 1.2 Framing  [I]
From UA1CTL0 (`0xFFFFFA20`), set in init as `(old & 0x88) | 0x0A` and held at runtime:
Decoding standard UARTA CTL0 bits (confirmed vs Renesas UARTA datasheet):
- bit7 PWR, bit6 TXE, bit5 RXE — toggled dynamically around each transfer
- bit4 DIR = 0 → **MSB-first**
- bit3:2 PS = `10` (bit3 set, bit2 clr) → **odd parity**
- bit1 CL = 1 → **8 data bits**
- bit0 SL = 0 → **1 stop bit**

**Framing = 8 data bits, ODD parity, 1 stop, MSB-first.**
(For contrast, UA0 `0xFA00` CTL0 = `...0xB4` → 7-bit / 0-parity / LSB-first — a different device.)

### 1.3 Baud  [H]
Baud = fUCLK / (2·k), k = UA1CTL2. ch1 CTL2 @ `0xFFFFFA22` = **`0x48` (k=72)** at runtime (set in `FUN_ram_0002fbc4`; init seeds `0x08`). UA1CTL1 clock-select (`0xFFFFFA21`) left at reset → fUCLK = fXX.
- If fXX = 16 MHz → 16e6/(2·72) ≈ **111 kbps** (~3.5% off 115200).
- Exact bps needs the clock-gen/PLL config (fXX) — **you can read this straight off your Saleae capture of the ch1 pair** (measure bit width) and confirm. Register values above are the ground truth to compute against.
- baud-gen compare @ `0xFA30/0xFA32` = 0 for ch1 (so the k-divisor at `0xFA22` governs).

### 1.4 Transport: DMA + timer-paced polling  [I]
- `FUN_ram_000af59c` @ `0x0AF59C` is the A/C link bring-up: seeds the 8-byte TX buffer `RAM 0x4338` to 0, sets mode byte `RAM 0x52D5`, programs timer compares (`0xFFFFF100–0x10A`, `0xFFFFF444=12`, `0xFFFFF45A=0xC2`) that pace the link, then calls `FUN_ram_0002fbc4`.
- `FUN_ram_0002fbc4` sets DMA/pointer reg `0xFFFFF310 = &RAM_0x5C60` and re-arms UA1 — i.e. **RX streams by DMA into `RAM 0x5C60`.**

### 1.5 TX command frame (HV-ECU → compressor)  [C/I]
**8-byte frames.** Builder `FUN_ram_000afba4` @ `0x0AFBA4` copies `src[0..7]` into the TX staging block and sets a descriptor (len = 8 @ gp-0x5f5c, flags 0x300/0x1ff8).
- Command scheduler `FUN_ram_000af680` @ `0x0AF680`: big switch on mode byte `RAM 0x52D5`; most states call `afba4(1, &RAM_0x52D1)` to queue an 8-byte frame. Notable mode codes → actions:
  - `0x55/0x57/0x50` → queue normal frame (`FUN_ram_000b146e/133e`)
  - `0xEB/0xEC/0xED` → `FUN_ram_000afd80(0/1/2)` = **handshake: write 0x55 / 0xAA to TX, or read status** (see 1.6)
  - `0xD0..0xD3` → `FUN_ram_000afdea(0..3)` = **pack telemetry into TX buffer** (see 1.7)
  - `0xC7..0xCF` → `FUN_ram_000b0dc6/0ff2` builders; `0x58/0xCB/0xCD…` latch mode in `RAM 0x435A`
- Half-duplex handshake bytes **0x55 / 0xAA** are the link preamble/role tokens.

### 1.6 Handshake / sync — `FUN_ram_000afd80` @ `0x0AFD80`  [C]
```
param 0 → UA1TX(0xFA2C) = 0x55 ; buf = const @0x52D5
param 1 → UA1TX(0xFA2C) = 0xAA ; buf = const
param 2 → buf = UA1STR(0xFA28)            // read status/err byte
then FUN_ram_000afba4(1, &RAM_0x4338)     // transmit
```

### 1.7 Telemetry packer — `FUN_ram_000afdea` @ `0x0AFDEA`  [I]
Packs six/eight **16-bit, big-endian (hi byte first)** values read from `0xFFFFF210–0xFFFFF21C` (ADC/setpoint mailbox) into TX buffer `RAM 0x4338..0x433F`. These are the live fields the ECU sends toward the compressor (e.g. target speed / thermal setpoints — field identity not individually proven; [H]).

### 1.8 RX response frame (compressor → HV-ECU)  [C]
DMA fills **`RAM 0x5C60`**; validated by **`FUN_ram_0002fcda` @ `0x02FCDA`** (UARTA ch1 / DMA-complete ISR, vector `0x0680`):
- **Checksum:** 16-bit sum of bytes `[0x5C60 .. 0x5C60+0x61]` (98 bytes) compared to the 16-bit field at **`0x5CC2`** (frame offset `0x62`). → **100-byte frame = 98 data + 2-byte additive checksum.**
- On pass: copies 100 bytes to the working buffer (gp-0x1621), clears fail counters, dispatches via a jump table (frame-type handler).
- On fail: increments fault counters (`gp-0x1624/0x1623`), and a "no-reply" watchdog counter if the bus is otherwise idle.

### 1.9 Byte-map summary for ZombieVerter
```
HV-ECU → COMPRESSOR  (UART ch1, 8-O-1, MSB-first, ~111 kbps[H])
  preamble token : 0x55 (master) / 0xAA (alt)         [afd80]
  8-byte command : src[0..7] (payload carries BE16 setpoints from 0xF210..) [afba4/afdea]

COMPRESSOR → HV-ECU  (same link, DMA RX into 0x5C60)
  100-byte frame : [0..97] data , [98..99] = sum16 of [0..97]   [fcda]
  (ECU rejects frame if sum mismatch; emulator MUST append correct sum16)
```
**Deliverable for conversion:** to drive the OEM compressor, emulate the master: 8-O-1 UART (measure exact baud off your capture), send 0x55 preamble + 8-byte command carrying target speed, and reply-parse the 100-byte status frame with a trailing 16-bit additive checksum. To *keep the OEM ECU happy with no compressor*, feed it a valid 100-byte frame with correct sum16 so `fcda` passes and the no-reply DTC counter stays 0.

---

## PART 2 — OTHER NOTABLE FINDS

- **Inverter / MG sync-serial frame (CSIB1)** — ISR `FUN_ram_0002f3b6` @ `0x02F3B6`, vector `0x0640`, buffer `RAM 0x5C1C`.  [C]
  - **Frame markers:** header `0x24 0x24 0x24` (`$$$`) at `[0x5C1C..1E]`, trailer `0xDB 0xDB 0xDB` at `[0x5C5B..5D]`.
  - **Checksum:** sum of bytes `[3..0x3C]` vs **big-endian** 16-bit at `[0x5C59:0x5C5A]`; payload copied (0x42 bytes) on pass → `FUN_ram_0002f750`. This is the inverter/MG command+feedback link (the clocked-serial-over-CAN-PHY you've been probing). CSIB init `0x02ABC6`; CSIB0 ISR `0x02E128` (vec 0x630), CSIB2 ISR `0x02D102` (vec 0x650).

- **CAN layout** (from map, cross-checked):  [C]
  - Bus-2 (dedicated inverter/HV) **TX IDs 0x120,0x124,0x126,0x244,0x540**; Bus-1 TX 0x340,0x348,0x3CA,0x3CB,0x4D0,0x700. HW mailbox table `0x007A00`; SW ID table `0x0089AC`. CAN driver `0x022E00–0x024200`; TX ISRs `0x236C4`/`0x23664` (vec 0x610/0x620); RX ISRs vec 0x5E0/0x5F0.
  - U0100 (lost ECM) is a CAN-RX-timeout DTC — triggered by the Bus-? RX mailbox-liveness logic in the CAN driver; the exact timeout/DTC-set site was not isolated this pass (see gaps).

- **UDS / OBD diagnostic DID read dispatcher** — `FUN_ram_00093486` @ `0x093486` (entry via `0x09380E`).  [I]
  - Large sub-function switch (≈0x00–0x98) mapping DID index → live-data readers `FUN_ram_0005c2d2`, `0x5C6C8`, `0x5C7BC`, `0x5CC38`, etc., using table `DAT_ram_00093504`/`0x934C6`. OBD CAN IDs `0x7DF/0x720/0x7E2/0x7EA/0x72C/0x724` table at `0x008AC4`. This is the service-$22-style data path — useful to read OEM live values for mapping.

- **UARTA other channels:** UA0 `0xFA00` = 7-bit/0-parity/LSB-first device (separate aux); UA2–UA4 configured but not characterised here. ch0 RX ISR `FUN_ram_00085FD4` (vec 0x660) buffers up to 15 bytes with status-error gating.

- **Sensor/scaling math** around `FUN_ram_000896C6` @ `0x0896C6` (uses mul/div/interp helpers `0x91810/0x910B4/0x918BA`) — candidate home for MG/thermal scaling constants.  [H]

### IMMOBILISER / SEED-KEY — LOCATE ONLY (not decoded)
Per instructions, **no immobiliser/seed-key/secret bytes were decoded or output, and none appear in this file.** A dedicated immobiliser/auth routine was **not positively isolated** in this pass; the UDS dispatcher (0x093486) is where a security-access ($27) sub-handler would hang off — flagged for a locate-only follow-up. No VIN/odometer values were read or emitted.

---

## GAPS / RECOMMENDED FOLLOW-UP
- **A/C baud:** confirm fXX (clock-gen/PLL) or just measure bit-width on your ch1 Saleae capture; k=72 @ `0xFA22` is the divisor to compute against.
- **A/C payload field map:** prove the meaning of the BE16 fields packed by `afdea` (target rpm, discharge temp, current) by correlating `0xF210..` ADC mailbox with live traces.
- **Ready/IG state machine, HV contactor/precharge, inverter enable/torque-limit, S-93C56 EEPROM access, U0100 set-site, security-access ($27):** not individually isolated this pass — all reachable from the CAN driver (`0x022E00`), the CSIB1 inverter handler (`0x02F3B6`), and the UDS dispatcher (`0x093486`). The three CSIB channels (0x2E128/0x2F3B6/0x2D102) are the likely EEPROM/sub-CPU candidates to check next.
