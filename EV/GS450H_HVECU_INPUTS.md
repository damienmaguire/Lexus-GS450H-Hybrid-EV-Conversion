# GS450h HV-ECU (89983-30200 / 899833020000): what the OEM HV-ECU reads (RX + hardwired) and sends (TX)

**Cross-checks:** SavvyCAN logs in `canlogs/can_batch/extracted/` (Ready, creep R→D, ECM-out, ECM+HV-out). Sam's log pass `analysis/gs450h_chassis_bus_rx_for_zv.md` was read only. Existing refs reused (read-only): `descrambled/CAN_AND_SERIAL_MAP.md`, `analysis/CAN_PAYLOAD_DECODE.md`, `can_ecm_frames_decode.md`, `can_hv_rx_u0100.md`.
**Rules followed:** desk-only, no TX, no binary modified (md5 of `descrambled/*`, `re/*` and Sam's note checked before and after). No VIN, odometer or seed/key content. The VIN-like 0x553/0x554/0x555 frames are noted as present only.
**Tags:** [C] confirmed (firmware code, plus the logs where stated) · [I] inference · [H] hypothesis.
**Bus naming [C]:** firmware **bus 2 = logger Bus0** = chassis bus (the one ZV keeps). Firmware **bus 1 = logger Bus1** = ECM/body side (going away).
**Byte/bit notation:** `bN.k` = byte N, bit k (bit 7 = MSB). Multi-byte fields are big-endian (Motorola), read by the firmware helpers `0xAE92C` (s16 BE), `0xAE9EA` (u16 BE), `0xAEA1C` ((BE16 & mask) >> sh) and `0xAE95E` (the same, sign-extended to the given width) [C].
**Descriptions:** every CAN table (and the hardwired-input tables) has a *Description (plain English)* column. It only restates what the other columns and tags already say, in plain words — words like 'likely', 'possibly' or 'appears to' keep the [I]/[H] uncertainty; no new signal decodes were added.

---

## 0. HEADLINE: bus-2 (chassis / logger Bus0) RX summary — what ZV can still see
Hardware mailboxes MB18–31 (MCONF 0x0A = bus2 RX) plus software RX table entries 48–60 (IDs at 0x8A6C, descriptors at 0x8654). Unpack dispatcher **0x274DC** calls one routine per frame. The values are re-copied into app variables at **0x699E4–0x69E54** [C].
"Sender" comes from the ECU-disconnect logs (`can_attribution.md`: still present with ECM+HV out = chassis ECU) plus Toyota ID convention [I/H].

| ID | Description (plain English) | Period (log) | DLC | Cks | Sender | What the firmware consumes (byte/bit → use) | Tag |
|---| --- |---|---|---|---|---|---|
| **0x0A4** | Brake/stability status message, likely from the skid-control (ABS/VSC/ECB) brake ECU. It echoes the Ready state back, and carries a 9-bit value that the HV-ECU uses to switch on a creep / brake-hold condition when it is above 250 at low speed (≤ 25 km/h); what that value physically is (e.g. a brake-hold request or a traction limit) is still a hypothesis. | 12 ms | 7 | yes | Skid-control / ECB ECU [I] (present with ECM+HV out [C]) | **b0.7** → Ready-echo flag (app gp-3441.0); b0.6/b0.5 → gp-3440.0/.1; b0 bits 2–4 → status flags. **9-bit field b4.0+b5 (u, 0–511)** → gp-3452 → creep/brake-hold condition: active when field > **250** and 0x0B6 speed ≤ **2500** (with hysteresis 225 / 2600) at 0x79D2E, and a speed-dependent table at 0x84DF2. **10-bit field b0[1:0]+b1: unpacked to RAM 0x3FF21D0 but no reader in the image (not used)**. b4.1 → gp-3440.7. | [C] layout & consumers; [H] physical meaning |
| **0x0A6** | Braking-demand message, likely from the brake (ECB / skid-control) ECU. Its brake-amount byte is turned into a negative (braking) torque term and drives the brake-on decision; other fields feed the regen / friction-brake blending, but whether they are a regen request or a torque limit is not yet known. | 12 ms | 8 | yes | Skid-control / ECB ECU [I] | **b2 (u8) = brake amount** → `−(b2<<21)/15438` → gp-3468 (20 readers: torque arbitration/regen 0x77xxx, 0x7Axxx, 0x7Cxxx, 0x7Fxxx). Also ×128/255, clamped to 127, at 0x4D614. Feeds the **brake-on decision** at 0x77250 (threshold −318, ≈ b2 ≥ 3). **b1 (u8)** → `−clamp((b1<<20)/15438)` → gp-3494, gated by b3.5 → regen/brake-blend block (0x7A134, 0x85126–0x85146, next to brake-on). **b5[2:0]+b6 (s11)** → `(x<<20)/15438` → gp-3488/-3480 (gated by flags) → 0x6A7A8, 0x80240. **b3[2:0]+b4 (s11): unpacked, no reader**. b0: unpacked, no reader. Flags b3.3–b3.7, b5.4–b5.7 → validity/qualifier bits (b3.6 qualifies the brake-amount path). | [C] math & consumers; [I] b2 = brake demand; [H] b1/b5 = regen-request vs limit |
| **0x0B0** | Wheel-speed pair for one axle (likely the front), likely from the skid-control (ABS/VSC) ECU. The HV-ECU uses it as its vehicle-speed reference — it cross-checks it against its own speed estimate and converts it to a shaft-speed term — but only while the per-wheel 'valid' flags are clear. Probably 0.01 km/h per count. | 12 ms | 8 | yes | Skid-control ECU [I] | **b0-1 s16, b2-3 s16 = wheel speeds (one axle, signed; negative in R [C log])** → gp-3466/-3464. Used: `avg/100` checked against the internal speed variable gp-2706 (±10) at 0x78018, and `(avg·8/100)·15438/4096` → shaft-speed term at 0x6EBB6. Per-wheel flags b4.0–2, b4.4–6 and b6 (8 bits) → per-wheel status bytes gp-3438…-3435. b4.1/b4.2/b4.5/b4.6 must be 0 for the speed to be used. | [C] layout & consumers; [I] **unit 0.01 km/h**; [I] front axle |
| **0x0B2** | Second wheel-speed pair (likely the rear axle), same layout as 0x0B0, likely from the skid-control ECU. Stored by the HV-ECU but apart from a diagnostic-style block nothing uses it. | 12 ms | 8 | yes | Skid-control ECU [I] | Same layout as 0x0B0 (other axle) → gp-3462/-3460 and flag bytes. **No app reader of the speeds** beyond the copy, apart from 0x95B7A (a diagnostic-style block). | [C]; [I] rear axle, diag only |
| **0x0B6** | Vehicle-speed message, likely from the skid-control ECU, in the same counts as the wheel speeds. The HV-ECU uses it to decide when the creep / brake-hold logic may act (up to about 25 km/h) and in a speed lookup table; one bit marks the value invalid. | 24 ms | 8 | yes | Skid-control ECU [I] | **b0-1 s16 = vehicle speed** (same counts as the wheel words [C log]) → gp-3450: creep/brake-hold window ≤ 2500/2600 (25/26 km/h at 0.01 km/h) at 0x79D3A, and a ROM table at 0x84DBA. **b2.7** → invalid flag gp-3439.0. | [C]; [I] unit |
| **0x230** | Brake-switch message, likely from the skid-control / ECB brake ECU. Bit b3.2 is the stop-lamp (brake-pedal) switch — one of the two ways the HV-ECU decides 'brake is on' (the other is the 0x0A6 brake amount), and that flag feeds the regen / torque logic. | 24 ms | 7 | yes | Skid-control/ECB ECU [I] | **b3.2 = stop-lamp/brake switch [C log]** → gp-3440.4. b3.3 → gp-3440.5 (inverted, qualifier). b3.0 → gp-3440.3. b3.4 → flag. Brake-on decision at 0x77250: `brakeOn = (brake-amount < −318 AND 0x0A6 b3.6) OR (0x230 b3.2 AND qualifier)` → gp-1848.7 (18 readers incl. regen/torque block 0x8511A–0x85360). | [C] code; [C] b3.2 meaning (log) |
| **0x236** | Short status message, likely from the ECB / skid-control brake ECU. Its fields are read next to the 0x0A4 creep value and vehicle speed, so it appears to feed the creep logic, but what the fields mean is still a hypothesis. | 24 ms | 4 | yes | ECB/skid ECU [I] | b0[7:6] → gp-3432; b0[5:4] → gp-3428; **10-bit b0[1:0]+b1** → gp-3430 (forced to 32767 when b0 status says invalid); b2 s8 → gp-3426. Readers 0x84E58/0x84E64 (with the 0x0A4 9-bit field and speed) and 0x77F54. | [C] layout; [H] meaning |
| **0x320** | Slow status-flag message, likely from the skid-control ECU; it carries a second copy of the brake switch (b1.3). The HV-ECU appears to just relay these flags into an internal status block. | 48 ms | 3 | yes | Skid-control ECU [I] | b0 bits 0–6 → RAM 0x3FF2045; b1 bits 0–3 and 7 → 0x3FF2047 (**b1.3 = brake switch copy [C log, Sam]**). Readers 0x388B8–0x38968 (status-flag relay block) and 0x358EE. A side-call to 0x38DA4 runs on the update. Descriptor RX timeout field 2520. | [C] layout; [I] status relay |
| **0x2E8** | Power-state message, possibly from the power-source / body ECU (no checksum). Its low nibble is the car's power mode (in the logs: wake → IG on → Ready), which the HV-ECU stores as its power-state reference; the exact meaning is a hypothesis. | 500 ms | 3 | **no** | Power-source / body ECU [H] | b0 & 0x0F → gp-7686 → gp-2339 at 0x74404 (power-state code; Sam's log: 0x80 wake → 0xF1 IG → 0x81 Ready). | [C] layout; [H] meaning |
| **0x591** | Slow body-network message, possibly from the body ECU (no checksum). Two bits are unpacked, but nothing in the firmware reads them — received and ignored. | 300 ms | 8 | **no** (b0 header 0x29) | Body ECU [H] | b1.4 → 0x3FF2070.4, b2.5 → 0x3FF2071.5. **No reader in the image**: received, not used. | [C] |
| **0x3CF** | Engine-status message from the engine ECU (ECM), including an 'ECM initialisation done' flag; part of the HV-ECU's check that the ECM is talking (U0100-type supervision). It disappears with the ECM, so the single-bus ZV setup doesn't need it. | 131 ms | 5 | yes | **ECM** [C: disappears with ECM out] | b0 flags → 0x3FF2034 (17 readers; b0.4 = ECM init done); b1 s8 → 0x3FF21C0 (7 readers); b2 s8 → 0x3FF21C1; b3.0/.2 → 0x3FF2035. Supervision group G1 (U0100-type). | [C] — **goes away with the ECM** |
| **0x023** | Possibly the yaw-rate / G-sensor message. The HV-ECU only checks that it keeps arriving (communication supervision) and never reads its contents. | 12 ms | 7 | yes | Yaw-rate / G sensor [H] | **Not unpacked** (no access to its shadow buffer). Presence-supervised only (supervision group G12, timeout ≈ 88). | [C] not used |
| **0x260** | Possibly the electric power steering (EPS) steering-torque message (Toyota ID convention). Only checked for presence; contents not read. | 20 ms | 8 | yes | EPS (steering torque) [H, Toyota 0x260] | **Not unpacked**. Presence-supervised only (G3, ≈ 98). | [C] not used |
| 0x720 | Diagnostic / broadcast frame (also listed in the ECU's diagnostic ID table). Caught by one hardware mailbox only; no data is decoded. | ~3.5 s | 8 | – | Diagnostic/broadcast [I] (also in the diag-ID table at 0x8AC4) | Hardware MB18 only. No COM descriptor, no unpack. | [I] |

**Not received by this firmware on any bus (even though they are on Bus0):** 0x020, 0x022, 0x025 (steering angle), 0x028, 0x0B4 (vehicle speed), 0x0B8, 0x224 (brake), 0x262, 0x4Ax/0x4Cx, 0x521/0x523 [C: not in the RX table or the mailbox filters].

**Plain-English:** these chassis-bus frames are ignored by the OEM HV-ECU — e.g. 0x025 (steering angle), 0x0B4 (another vehicle-speed message) and 0x224 (a brake message) are available to other modules (and to ZV) but this firmware never reads them. The roles of the others are not decoded here.

### 0.1 What this means for the ZV single-bus class
- **ZV as the HV-ECU replacement only needs to *read* these. Usable sources on Bus0:** wheel speeds 0x0B0/0x0B2 (s16, 0.01 km/h [I]), vehicle speed 0x0B6 b0-1 (or 0x0B4 b5-6), brake switch 0x230 b3.2, brake amount 0x0A6 b2, Ready/IG echoes 0x0A4 b0.7 and 0x2E8 b0. 0x3CF disappears with the ECM and is not needed.
- **The OEM firmware's own chain [C]:** brake switch 0x230 b3.2 **or** brake amount 0x0A6 b2 (≥ ~3 counts, qualified by 0x0A6 b3.6) → *brake-on* flag gp-1848.7 → regen/torque-arbitration code. 0x0A6 b2 is also converted into a negative (braking) torque term `−b2·512·4096/15438`. Copying that is a good model for ZV regen blending [I].
- **Creep/brake-hold [I]:** the 0x0A4 9-bit field > 250 at vehicle speed ≤ 25.00 km/h sets a flag in the creep/launch logic (0x79D2E). In the logs it is 0 except for brief 1–2 counts, so its meaning is [H] (brake-hold/hill-hold pressure request or TRC limit).
- **Wheel-speed unit [I]:** the firmware divides the wheel words by **100** before comparing them with an internal speed variable, with a ±10 tolerance. That is consistent with **0.01 km/h/bit** (≈ km/h after ÷100). The same words ×8/100 × 15438/4096 give a shaft-speed term. The constant **15438/4096 = 3.769** shows up in both the speed path (×) and the 0x0A6 torque path (÷), so it is a fixed wheel↔shaft ratio [I] (most likely the final-drive/overall ratio [H]). Confirm with a GPS/cluster run at a steady speed.
- **Checksums:** every bus-2 RX ID except 0x2E8 and 0x591 has flag bit0 set (Toyota checksum enforced in 0x24B8C). DLC must be ≤ 8. **No rolling counters** are checked [C] (agrees with Sam).
- **Gear and accelerator are NOT inputs on the CAN.** The HV-ECU *computes* gear (from hardwired switches, §2) and *publishes* it in TX 0x540/0x120 (§3) [C log + fw].

---

## 1. Full RX map (both buses)

### 1.1 Hardware filters (ROM 0x7A00 MCONF / 0x7A20 MIDH; identical in the original image) [C]
| MB | MCONF | Bus | ID | Description (plain English) | Note |
|---|---|---|---| --- |---|
| 6–10 | 0x09 | fw bus1 | 0x039, 0x341, 0x343, 0x3C8, 0x484 | Exact-ID mailboxes for the five engine-side (ECM) messages listed in §1.2 bus 1 — engine status, crank/run request, etc. | exact |
| 11 | 0x19 | fw bus1 | 0x528 + mask | Wide 'catch-all' mailbox that lets in the slow 0x5xx body/meter messages (likely). | masked filter that catches the 0x5xx body frames (0x53F…0x5FC) [I] |
| 12 | 0x21 | fw bus1 | 0x580 + mask | Second catch-all mailbox for the 0x5xx body/meter messages (likely). | as above [I] |
| 18–31 | 0x0A | fw bus2 | 0x720, 0x023, 0x0A4, 0x0A6, 0x0B0, 0x0B2, 0x0B6, 0x230, 0x236, 0x260, 0x2E8, 0x320, 0x3CF, 0x591 | Exact-ID mailboxes for the chassis-bus messages in §0 (brake, wheel/vehicle speed, power state, ECM status, plus presence-only and diagnostic frames). | exact |

### 1.2 Software RX table (IDs 0x8A24/0x8A6C, descriptors 0x8654, unpack routines) [C]
Descriptor flags: bit0 = checksum checked, bit2 = byte0 header, bit4 = bus2. Type 0x60 = fast RX, 0x61 = slow/event RX.

**Firmware bus 1 (ECM/body, logger Bus1) — goes away in the single-bus class**
| idx | ID | Description (plain English) | DLC | cks | Unpack | Fields consumed → RAM (readers) | Sender | Tag |
|---|---| --- |---|---|---|---|---|---|
| 30 | 0x039 | Engine-status message from the ECM: a temperature-like value, an 'engine stopped' flag and other engine values. Goes away with the ECM. | 5 | y | 0x2559A | b0 s8 → 21BB (temp-like); b1.1–.6 → 2033 flags (b1.1 = engine stopped); b1.0+b2 s9 → 21BC; b3 → 21BE | ECM | [C] (see `CAN_PAYLOAD_DECODE.md` §3) |
| 31 | 0x341 | Engine-side status-flag message; the sender is uncertain because it stays on the bus with the ECM unplugged. Flags and a 4-bit field are stored; meaning not decoded. | 3 | y | 0x2577E | b0 bits 0, 2–7 → 204B; b1 bits 0, 5–7 → 204C; b1[4:1] → 21E0 | ECM-side, still present with ECM out [H] | [C] layout |
| 32 | 0x343 | Companion engine-side status message (sender uncertain, as 0x341). Flag bits and small fields are stored; meaning not decoded. | 7 | y | 0x2586E | b4 bits 0–4 → 204D; b4[7:5] → 21E5; b5 bits 3–5, 7 → 204E; b5[2:0] → 21E6 | as above [H] | [C] |
| 33 | 0x3C8 | Engine-control message from the ECM: crank-request and engine-running flags plus an engine-speed-like byte that the HV-ECU uses widely. It is also seen on the chassis bus, but the firmware only accepts it on bus 1. | 6 | y | 0x25642 | b0 flags → 2036 (b0.7 crank request, b0.2 run); b1 → 21FC; **b2 → 21FE (13 readers, engine-speed-like)**; b4 flags; b4[1:0] → 2201 | ECM (also mirrored on Bus0, **but the fw only accepts it on bus1**) | [C] |
| 34 | 0x484 | ECM message (no checksum) with two signed 16-bit values and one byte, stored by the HV-ECU; meaning not decoded here. | 5 | n | 0x2573C | b0-1 s16 → 21C2; b2 → 21C4; b3-4 s16 → 21C6 | ECM | [C] |
| 35 | 0x53F | Body-side message (sender is a hypothesis) carrying one 24-bit value, stored as a 32-bit number; meaning not decoded. | 6 | n | 0x25B36 | b1..b3 24-bit BE → u32 21F4 | body [H] | [C] |
| 36 | 0x56A | Body / instrument-cluster message (sender is a hypothesis); one data byte is stored; meaning not decoded. | 3 | n | 0x25B72 | b1 → 21F1 | body/meter [H] | [C] |
| 37 | 0x56D | Body-network message (sender unknown); one byte (used in four places) and three flag bits are stored; meaning not decoded. | 4 | n | 0x25A16 | b1 → 21F0 (4 readers); b2.5–.7 → 2052 | [H] | [C] |
| 38 | 0x57A | Body-network message (sender unknown); one 3-bit field is stored; meaning not decoded. | 3 | n | 0x25B92 | b1[7:5] → 21F8 | [H] | [C] |
| 39 | 0x57F | Body-network message (sender unknown); a single flag bit is stored; meaning not decoded. | 7 | n | 0x25BEE | b1.4 → 2054.4 | [H] | [C] |
| 40 | 0x583 | Body-network message (sender unknown); two flag bits are stored; meaning not decoded. | 6 | n | 0x25BB6 | b2.5/b2.4 → 0x3FF74B9.0/.1 | [H] | [C] |
| 41 | 0x58D | Body-network message that is in the receive table but never unpacked — received and ignored. | 3 | n | – | **not unpacked** | – | [C] |
| 42 | 0x5A6 | Body-network message that is in the receive table but never unpacked — received and ignored. | 2 | n | – | **not unpacked** | – | [C] |
| 43 | 0x5A7 | Body-network message that is in the receive table but never unpacked — received and ignored. | 4 | n | – | **not unpacked** | – | [C] |
| 44 | 0x5B6 | Body-network message (sender unknown); a single flag bit is stored; meaning not decoded. | 3 | n | 0x25C16 | b2.7 → 205E.7 | [H] | [C] |
| 45 | 0x5BF | Body-network message (sender unknown); four small 2-bit / 6-bit fields are stored; meaning not decoded. | 3 | n | 0x2596A | b1[7:6], b1[5:0], b2[7:6], b2[5:0] → 21E7…21EA | [H] | [C] |
| 46 | 0x5ED | Body-network message (sender unknown); a set of flag bits and one 16-bit value are stored; meaning not decoded. | 6 | n | 0x25A6C | b1 bits 0–3, 5, 6 → 2050; b2.4–.7 → 2051; b4-5 u16 → 21EE | [H] | [C] |
| 47 | 0x5FC | Body-network message (sender unknown); five flag bits are stored; meaning not decoded. | 8 | n | 0x259AE | b2 bits 0–4 → 204F | [H] | [C] |
(RAM addresses abbreviated: 21BB = 0x3FF21BB, etc.)

**Firmware bus 2 (chassis, logger Bus0) — keep.** See §0 for consumers. Unpack addresses: 0x3CF 0x27530 · 0x0B0 0x275FC · 0x0B2 0x27718 · 0x320 0x27834 · 0x230 0x27922 · 0x0A4 0x2797A · 0x0A6 0x27A58 · 0x236 0x27B58 · 0x2E8 0x27BA6 · 0x0B6 0x27BCA · 0x591 0x27C06. 0x023 and 0x260 have no unpack [C].

**Staging → app-variable map (copy layer 0x699E4–0x69E54) [C]:**
| Staging (unpack) | App variable | Transform |
|---|---|---|
| 0x0A6 b1 (gp-7723) | gp-3494 | `−sat16((b1<<20)/15438)`, gated by 0x0A6 b3.5 |
| 0x0A6 s11 b5-6 (gp-7676) | gp-3488 → gp-3480/-3476 | `(x<<20)/15438` (32-bit), selected by derived flags gp-3442.4–.7 |
| 0x0A6 b2 (gp-7722) | gp-3468 | `−(b2<<21)/15438` |
| 0x0B0 / 0x0B2 words (gp-7736…-7730) | gp-3466, -3464, -3462, -3460 | copy |
| 0x0B0/0x0B2 b4, b6 flag bits | gp-3438…-3435 (one byte per wheel, bits 1–7) | bit regroup |
| 0x0A4 9-bit (gp-7726) | gp-3452 | copy |
| 0x0B6 speed (gp-7714) | gp-3450 | copy |
| 0x236 fields (gp-7721/-7720/-7718/-7716) | gp-3432 / -3428 / -3430 / -3426 | copy; 10-bit forced to 32767 when invalid |
| 0x0A4 b0.7 | gp-3441.0 | Ready echo |
| 0x0A4 b0.6, b0.5; 0x0A6 b3.6; 0x230 b3.0, b3.2, ¬b3.3; 0x0A4 b4.1 | gp-3440 bits 0–5, 7 | flags |
| 0x0B6 b2.7 | gp-3439.0 | invalid |

---

## 2. Hardwired inputs (not CAN)
MCU: main CPU uPD76F0038 (V850, Toyota-custom). The I/O map below is by **register address** as accessed in the code. Port letters assume the usual NEC V850ES/Fx layout (Pn at 0xFFFFF400 + 2n) [I].

### 2.1 ADC (results 0xFFFFF210 + 2·ch, 10-bit left-aligned; mode/start registers 0xFFFFF200–F204) [C]
The scan routine **0x2EAE8** copies the results to RAM **0x3FF27E6 + 2·ch** (gp-6154). The trigger/start is at 0x2EB6A (writes 0xFFFFF201). **0xAFDEA** forwards the raw ADC words over a serial link (to the second CPU/peripheral) [I].
| ch | Description (plain English) | Consumers | Interpretation | Tag |
|---| --- |---|---|---|
| **0, 1** | Accelerator pedal: two independent position sensors that the ECU compares with each other for plausibility (likely VPA1/VPA2). | 0x579A8: each >>2, **\|ch0−ch1\| compared with a ROM limit, agreement counter** (dual-sensor plausibility); 0x33FDC: >>6 → gp-8018/-8016 | **Accelerator pedal main/sub (VPA1/VPA2)**. The HV-ECU is the APP owner on the GS450h (wiring diagram: VPA/VPA2/VCP1/VCP2/EP1/EP2 at the HV ECU), and TX 0x244 b6 follows the pedal in the creep log | [I] |
| 2, 6, 5 | Three analog inputs read by the input-interface block; what they measure is not known. | 0x33CE2/0x33CE6/0x33D0C → gp-7956/-7954/-7952 (input interface block) | unknown analog inputs | [H] |
| 3 | Analog input compared with a fixed threshold; purpose unknown. | 0x4A960 (compared with a ROM threshold) | [H] | [H] |
| 7 | Analog input read next to the shift logic — possibly shift-related; unconfirmed. | 0x319DC (next to the shift-logic function) | [H] (shift-related analog?) | [H] |
| **12, 13** | Two temperature sensors (thermistor-style, converted to °C via a lookup table); which temperatures is not known. | 0x308D0/0x308EE: >>6 → ROM table 0x155F0 → **−50** → gp-5408/-5406 | **two temperature sensors in °C** (−50 offset LUT, thermistor-style) | [I] |
| 14 | Possibly the supply / IG / +B voltage sense, used by the power-state logic; unconfirmed. | 0x1E6EE…0x1ED12, 0x2BB70, 0x32684: compared ≥ 36045 (≈ 563/1023) in power-state logic | supply/IG/+B voltage sense [H] | [H] |
| 15 | Likely the ECU's reference-voltage self-check: a fault is set if it leaves a ~2.2–2.8 V window. | 0x2EAA6: window 28835…36700 (450–573 counts) → fault bit; 0x4B6DE | **reference-voltage self-check** (~2.2–2.8 V) | [I] |
| 4, 8–11 | Analog channels not read directly by the application, only forwarded raw to the second CPU / peripheral; purpose unknown. | no direct gp reader (forwarded raw by 0xAFDEA) | [H] | [H] |
Caveat: register allocation inside 0x2EAE8 shows residual-descramble artefacts. The slot order ch n → RAM +2n is [I].

### 2.2 Discrete inputs — sampler + 2-sample debounce at 0x2B09E [C]
Debounced words: **A = gp-6588 (RAM 0x3FF2644)**, **B = gp-6586**, **C = gp-6584**.
| Word.bit | Pin (register.bit) | Description (plain English) | Polarity | Use / interpretation | Tag |
|---|---| --- |---|---|---|
| **A.0** | **0xFFFFF416.4** | Ignition switch (IG on) input, most likely: it enables CAN supervision, the communication layer, timers and start logic. | active-high | **IGSW (IG on)**: gates the CAN U0100 supervision (0x28718), the COM layer (0x24728, 0x252AA), timers, and start logic 0x3120E | [I] (strong) |
| A.1 | 0xFFFFF41E.1 | Unidentified digital input (active-low); purpose not yet known. | low | [H] | [H] |
| A.2 / A.3 | 0xFFFFF416.7 / .6 | Two unidentified digital inputs (active-high); purpose not yet known. | high | [H] | [H] |
| A.4 | 0xFFFFF40A.7 (P5.7) | Unidentified digital input (active-low); purpose not yet known. | low | [H] | [H] |
| A.9 / A.10 / A.11 / A.13 | 0xFFFFF700.4 / .5 / .0 / .7 | Four unidentified digital inputs (active-high); purpose not yet known. | high | [H] | [H] |
| A.12 | 0xFFFFF41E.6 | Unidentified digital input (active-high); purpose not yet known. | high | [H] | [H] |
| **A.14** | **0xFFFFF40C.0 (P6.0)** | Likely one contact of the shift-lever position switch — read by the shift logic and merged into the shift contact set. | high | read by the **shift-position logic** 0x316CE and mapped into the shift contact set gp-8178.2 (0x34010) | [I] shift input |
| A.15 | 0xFFFFF402.5 (P1.5) | Unidentified digital input (active-high); purpose not yet known. | high | [H] | [H] |
| **B.0–B.15** | **multiplexed matrix: 0xFFFFF40A (P5) bits 5–6 = 2-bit row-select outputs, P5 bits 0–4 = 5 inputs, 4 rows** (scanner 0x1E8CE, settle delay `0x1D34A(100)`) | Likely the shift-lever position switch, read through a scanned 4-row switch matrix; the ECU turns these contacts into P/R/N/D and broadcasts it in 0x540 / 0x120. Which contact is which lever position is not yet known. | true + complement contacts | Row integrity check on bit4 → fault gp-6656.0. B.0–B.3, B.12, B.13 plus A.14 → **gp-8178 bits 0–6 (7-contact set, 0x33FF0)** → shift logic **0x316C0–0x318C0** → **RAM 0x3FF2B3A bits 0–3 = P/R/N/D, transmitted in 0x540 b1[7:4] [C log]** | [C] mechanism; [I] = shift-position switch |
| C.4 | 0xFFFFF402.0 (P1.0) | Unidentified digital input (active-low); purpose not yet known. | low | [H] | [H] |
| C.5 | 0xFFFFF414.3 | Unidentified digital input (active-high); purpose not yet known. | high | [H] | [H] |
| C.6 | 0xFFFFF416.3 | Unidentified digital input (active-high); purpose not yet known. | high | [H] | [H] |
| C.7 | 0xFFFFF41C.0 | Unidentified digital input (active-high); purpose not yet known. | high | [H] | [H] |
| C.8 | 0xFFFFF41E.7 | Unidentified digital input (active-low); purpose not yet known. | low | [H] | [H] |
| C.9 | 0xFFFFF700.1 | Unidentified digital input (active-high); purpose not yet known. | high | [H] | [H] |
| direct | 0xFFFFF700.2, .7 and 0xFFFFF40C.4 | Three inputs sampled directly (not debounced) by the input-interface block; purpose not yet known. | – | sampled directly in the interface block 0x33CFC/0x33D16/0x33D36 → gp-8187.1/.2/.0 | [H] |

- **Shift lever [C/I]:** it is hard-wired to the HV-ECU, not on CAN. The P/R/N/D state is produced in this ECU from the multiplexed P5 switch matrix plus P6.0 and then published (0x540 b1: 0x80 P, 0x40 R, 0x20 N, 0x10 D; 0x120 b5[3:0]: 0 P, 1 R, 2 N, 3 D — creep log [C]). Which physical contact is which matrix bit is [H]. Toggle the lever with the ECU powered on the bench and watch P5.0–4 per row to pin it down.
- **Brake switch [I]:** the OEM brake-on decision uses **CAN** (0x230 b3.2 + 0x0A6 b2, 0x77250) [C]. A hard-wired STP input appears on the wiring diagram, but its port bit is not identified among the [H] rows above [H].
- **Accelerator [I]:** ADC ch0/ch1 (dual, cross-checked).
- **IG [I]:** 0xFFFFF416.4 (A.0). ST/START and the interlock (ILK) are among the [H] discrete rows.
- **HV interlock / precharge / SMR sense:** not positively identified in this pass [H]. The SMR outputs live in the 0xFFFFF4xx port writes (not traced here).

---

## 3. TX map (what the OEM HV-ECU sends) — firmware descriptors re-read from the image [C]
All 30 TX descriptors (0x8654 idx 0–29) and the ID table are **byte-identical in the original image and the working copy** [C]. Every TX ID **vanishes when the HV-ECU is unplugged** (ECM+HV-out log) [C], so the sender is confirmed.
Period unit is 1.024 ms. cks = Toyota checksum in the last byte. hdr = byte0 header (0x25/0x26/0x29 periodic, | 0x80 on event).

### 3.1 HEADLINE — TX on firmware bus 2 (logger Bus0, chassis) — what ZV must reproduce for the chassis ECUs [C]
| idx | ID | Description (plain English) | Period | DLC | cks | Layout (signal → source RAM) | Log semantics | Tag |
|---|---| --- |---|---|---|---|---|---|
| 24 | **0x120** | Main hybrid-status broadcast from the HV-ECU (also sent on bus 1). It carries the selected gear as a code (0 P / 1 R / 2 N / 3 D) plus values that move when the engine runs; chassis modules and the meter likely read the gear from here. | 16 | 8 | b7 | b0–b3 u8 (3FF2247–224A); b4 bits 0–7 flags (b4.4/.6/.7 = 3FF208E, b4.5 = 3FF2846.1); **b5[3:0] = gear code (0 P, 1 R, 2 N, 3 D)**; b5[7:4] nibble; b6 bits 0–3 flags, b6[7:4] | idle `00 00 00 00 10 20 24 7D`; b0/b1 move with the engine running | [C] layout; [C log] gear nibble |
| 25 | **0x124** | HV-ECU output carrying two signed 11-bit values plus flags; the first value moves during creep and appears to be torque-related (hypothesis). | 16 | 6 | b5 | b0[7:4] (3FF225B); b0.3 (3FF2097.1); **b0[2:0]+b1 s11 (3FF225C)**; b2 bits 3–7; b2[2:0]+b3 s11; b4 bits 3, 6, 7 | the s11 at b0-1 moves in creep (torque-like) [H] | [C] layout |
| 26 | **0x126** | HV-ECU output with two signed 11-bit values and a 12-bit value taken from what appear to be motor-generator-related variables (hypothesis). | 16 | 8 | b7 | b0 bits 4–7 flags; b0[2:0]+b1 s11 (3FF2262 ← 3FF324C); b2.5; b2[2:0]+b3 s11 (3FF2264 ← 3FF324E); b4[5:0]; b5.4; b5[7:5]; b5[3:0]+b6 u12 (3FF2266 ← 3FF2A56) | MG-related movers [H]. b2 bits 3/7 seen in the logs are not covered by the extracted signals | [C] layout |
| 27 | **0x244** | HV-ECU output whose byte 6 follows the accelerator pedal (likely accelerator position, Prius-family convention), plus two signed 16-bit values; other ECUs likely use it as the driver-demand signal. | 23 | 8 | b7 | b0 bits 2–5 flags; b2-3 s16 (3FF2256 ← 3FF3282); b4-5 s16 (3FF2258 ← 3FF3280); **b6 u8 (3FF225A ← 3FF3238>>7)** | b6 0x00–0x0D when the pedal is pressed in creep → **accelerator position** [I] (Prius-family 0x244 convention) | [C] layout; [I] |
| 28 | **0x2C6** | HV-ECU status message (also sent on bus 1) with idle / 'accelerator off' flags that appear to change when the pedal is pressed, plus several other values. | 31 | 8 | b7 | b0 u8 (3FF224C); b1 s8 (3FF224D); b2 bits 0–5 (3FF2090); b3[1:0] + b3 bits 2–4; b4.2 | b3 0x19 → 0x01 while the accelerator is pressed (idle/"accel off" flags b3.3/b3.4) [I] | [C] layout |
| 29 | **0x540** | Hybrid-system status broadcast from the HV-ECU (also on bus 1), including the selected gear (P/R/N/D) as a one-hot nibble — the instrument cluster and other modules read the gear from here. | 977 | 4 | – (hdr) | b0 header 0x25 / 0xA5 on change; **b1[7:4] = shift position one-hot: 0x80 P, 0x40 R, 0x20 N, 0x10 D** (3FF2B3A bits 0–3, written by the shift logic 0x31830); b3.7 | creep log: P → R → N → D → P sequence matches [C] | [C] |
0x120, 0x2C6 and 0x540 are also sent on bus 1 (separate descriptors 1, 3, 18) [C].

### 3.2 TX on firmware bus 1 (logger Bus1, ECM/body) — goes away with the second bus
| idx | ID | Description (plain English) | Period | DLC | cks/hdr | Layout summary (from `CAN_PAYLOAD_DECODE.md`, descriptors re-verified) | Dest. [I/H] |
|---|---| --- |---|---|---|---|---|
| 0 | 0x037 | HV-ECU message to the engine side, probably an engine request to the ECM (hypothesis); carries a 14-bit value derived from an internal variable plus flags. | 8 | 4 | cks | b0[7:6] S1 (3FF226C); b0[5:0]+b1 14-bit (3FF226A = (3FF330C<<5)/100); b2 flags | ECM (engine request) [H] |
| 1 | 0x120 | Bus-1 copy of the main hybrid-status / gear broadcast (see 0x120 on bus 2), likely for the ECM and meter. | 16 | 8 | cks | as bus 2 | ECM/meter |
| 2 | 0x2C1 | Small flag-only message from the HV-ECU; destination and meaning unknown. | 31 | 8 | cks | b0.1/.2 flags (3FF20A9) | [H] |
| 3 | 0x2C6 | Bus-1 copy of the HV-ECU status message (see 0x2C6 on bus 2). | 31 | 8 | cks | as bus 2 | |
| 4 | 0x340 | Multi-field HV-ECU message (flags, a byte and a 16-bit value), probably for the ECM (hypothesis). | 47 | 8 | cks | b0[2:0], b0.3, b0[7:4]; b1 u8; b2-3 u16 (3FF2252); b4 flags; b5; b6.7 | ECM [H] |
| 5 | 0x348 | Likely the HV-ECU's engine torque / speed request to the ECM (hypothesis): flags, a 9-bit and a signed 11-bit value. | 40 | 8 | cks | b0 flags (3FF208B); b1.0+b2 9-bit (3FF223E); b3[2:0]+b4 s11 (3FF2242); b5; b6 | ECM (engine torque/speed request) [H] |
| 6 | 0x3CA | HV-ECU message with flags and a few byte values; destination and meaning unknown. | 98 | 6 | cks | b0 flags; b1 u8 (3FF2245); b2; b3; b4 | [H] |
| 7 | 0x3CB | HV-ECU message with two signed byte values; destination and meaning unknown. | 98 | 7 | cks | b4 s8 (3FF2106), b5 s8 (3FF2107) | [H] |
| 8–12 | 0x499/49A/49B/49E/49F | Five data-block messages that send a block of HV-ECU RAM as-is; what the data is and who reads it is a hypothesis. | 488 | 8 | – | whole frame copied from RAM (gp+2848) — data-block frames | [H] |
| 13 | 0x4D0 | Fixed data from ROM, apparently version / identification information. | 1025 | 8 | – | ROM constants (version-like) | [I] |
| 14–17 | 0x526/527/528/529 | Four slow messages with a header byte plus fields / flags, possibly for the meter (hypothesis). 0x528's length (3 vs 4 bytes) is still an open question. | 488/977/488/977 | 5/4/**3**/8 | hdr | header + fields/flags (0x528: descriptor DLC 3, the log shows DLC 4 — still open) | meter [H] |
| 18 | 0x540 | Bus-1 copy of the hybrid-status / gear broadcast (see 0x540 on bus 2). | 977 | 4 | hdr | as bus 2 (7 signals on bus 1) | |
| 19–23 | 0x59D/5B2/5C8/5EC/602 | Five slow messages with a header byte plus flags; destination and meaning unknown. | 977/4883/977/488/29297 | 3/4/6/7/2 | hdr | header + flags | [H] |
The signal-by-signal list (Sxx, masks, RAM sources) for every TX frame is Appendix A of `CAN_PAYLOAD_DECODE.md`; the descriptors (period, DLC, flags, bus) were re-checked against the image in this pass [C].

### 3.3 Bus0 frames that vanish with the HV-ECU out but are NOT in this CPU's TX table
0x2E4, 0x346, 0x347, 0x524, 0x579, and the VIN-like **0x553/0x554/0x555 (presence noted only, content not reproduced)** are attributed to the HV-ECU box by the disconnect logs [C log], but none of them is in the main-CPU ID table (0x89AC) [C]. 0x56A/0x5B6 are *RX* IDs here (bus 1). Most likely sources: the **second CPU (uPD76F0040)** on the same board [H], or gateway-style re-transmission [H]. **This matters for ZV:** if a chassis ECU needs any of those, the main-CPU image will not explain them.

**Plain-English:** these are frames that disappear from the chassis bus when the HV-ECU box is unplugged, but this CPU's firmware never sends them — they are most likely sent by the box's second CPU (or re-transmitted). 0x553/0x554/0x555 look like VIN / identification data (content not reproduced). The roles of 0x2E4, 0x346, 0x347, 0x524 and 0x579 are not known.

---

## 4. Open items (all desk-checkable or need the car; no TX needed)
1. Physical units: wheel/vehicle speed (0.01 km/h [I]). Run at a steady GPS speed and log Bus0.
2. 0x0A4 9-bit field and 0x0A6 b1 / s11 (b5-6): regen request vs torque limit [H]. Needs a brake-while-moving log and a low-µ/TRC event. The 0x0A4 10-bit and 0x0A6 s11 (b3-4) fields are **not used** by this firmware [C].
3. Matrix contact ↔ lever position, the STP port bit, ILK/ST inputs: bench IG-on with a logic probe on P5.0–6, P6.0 and 0xFFFFF416/41E pins.
4. Meaning of the 15438/4096 = 3.769 ratio (final drive?) [H].
5. Senders of 0x2E4/0x346/0x347/0x524/0x579 (second CPU?) [H].
