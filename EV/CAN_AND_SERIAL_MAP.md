# GS450h HV-ECU (`89983-30200`) — CAN & Serial Reverse-Engineering Map
---

## 1. CAN Bus Message Tables

### A. Hardware AFCAN 32-Buffer Config Table (`0x007A00–0x007A9F`)
- **`0x007A00–0x007A1F`**: 32 × 1-byte `MCONF` config values (channel + TX/RX + mask).
- **`0x007A20–0x007A9F`**: 32 × 4-byte `(MIDL=0x0000, MIDH = CAN_ID << 2)` entries.

| MB | `MCONF` | Bus / Dir | `MIDH` | **CAN ID** |
|---:|:---:|:---:|:---:|:---:|
| 0 | `0x01` | Bus 1 TX | `0x0D00` | **`0x340`** |
| 1 | `0x01` | Bus 1 TX | `0x0D20` | **`0x348`** |
| 2 | `0x01` | Bus 1 TX | `0x0F28` | **`0x3CA`** |
| 3 | `0x01` | Bus 1 TX | `0x0F2C` | **`0x3CB`** |
| 4 | `0x01` | Bus 1 TX | `0x1340` | **`0x4D0`** |
| 5 | `0x11` | Bus 1 TX (mask) | `0x1C00` | **`0x700`** |
| 6 | `0x09` | Bus 1 RX | `0x00E4` | **`0x039`** |
| 7 | `0x09` | Bus 1 RX | `0x0D04` | **`0x341`** |
| 8 | `0x09` | Bus 1 RX | `0x0D0C` | **`0x343`** |
| 9 | `0x09` | Bus 1 RX | `0x0F20` | **`0x3C8`** |
| 10 | `0x09` | Bus 1 RX | `0x1210` | **`0x484`** |
| 11 | `0x19` | Bus 1 RX (mask) | `0x14A0` | **`0x528`** |
| 12 | `0x21` | Bus 1 RX (mask) | `0x1600` | **`0x580`** |
| 13 | `0x02` | Bus 2 TX | `0x0480` | **`0x120`** |
| 14 | `0x02` | Bus 2 TX | `0x0490` | **`0x124`** |
| 15 | `0x02` | Bus 2 TX | `0x0498` | **`0x126`** |
| 16 | `0x02` | Bus 2 TX | `0x0910` | **`0x244`** |
| 17 | `0x02` | Bus 2 TX | `0x1500` | **`0x540`** |
| 18 | `0x0A` | Bus 2 RX | `0x1C80` | **`0x720`** |
| 19 | `0x0A` | Bus 2 RX | `0x008C` | **`0x023`** |
| 20 | `0x0A` | Bus 2 RX | `0x0290` | **`0x0A4`** |
| 21 | `0x0A` | Bus 2 RX | `0x0298` | **`0x0A6`** |
| 22 | `0x0A` | Bus 2 RX | `0x02C0` | **`0x0B0`** |
| 23 | `0x0A` | Bus 2 RX | `0x02C8` | **`0x0B2`** |
| 24 | `0x0A` | Bus 2 RX | `0x02D8` | **`0x0B6`** |
| 25 | `0x0A` | Bus 2 RX | `0x08C0` | **`0x230`** |
| 26 | `0x0A` | Bus 2 RX | `0x08D8` | **`0x236`** |
| 27 | `0x0A` | Bus 2 RX | `0x0980` | **`0x260`** |
| 28 | `0x0A` | Bus 2 RX | `0x0BA0` | **`0x2E8`** |
| 29 | `0x0A` | Bus 2 RX | `0x0C80` | **`0x320`** |
| 30 | `0x0A` | Bus 2 RX | `0x0F3C` | **`0x3CF`** |
| 31 | `0x0A` | Bus 2 RX | `0x1644` | **`0x591`** |

### B. Full Software CAN ID Table (`0x0089AC–0x008A9F`, 61 entries × 4 bytes)
Grouped into 4 contiguous blocks (including masked/multiplexed IDs):
- **Group 1 (`0x0089AC–0x008A0B`, 24 IDs):** `0x037, 0x120, 0x2C1, 0x2C6, 0x340, 0x348, 0x3CA, 0x3CB, 0x499, 0x49A, 0x49B, 0x49E, 0x49F, 0x4D0, 0x526, 0x527, 0x528, 0x529, 0x540, 0x59D, 0x5B2, 0x5C8, 0x5EC, 0x602`
- **Group 2 (`0x008A0C–0x008A23`, 6 IDs):** `0x120, 0x124, 0x126, 0x244, 0x2C6, 0x540`
- **Group 3 (`0x008A24–0x008A6B`, 18 IDs):** `0x039, 0x341, 0x343, 0x3C8, 0x484, 0x53F, 0x56A, 0x56D, 0x57A, 0x57F, 0x583, 0x58D, 0x5A6, 0x5A7, 0x5B6, 0x5BF, 0x5ED, 0x5FC`
- **Group 4 (`0x008A6C–0x008A9F`, 13 IDs):** `0x023, 0x0A4, 0x0A6, 0x0B0, 0x0B2, 0x0B6, 0x230, 0x236, 0x260, 0x2E8, 0x320, 0x3CF, 0x591`
- **Diagnostic / OBD CAN IDs (`0x008AC4–0x008AD7`):** `0x7DF, 0x720, 0x7E2, 0x7EA, 0x72C, 0x724`

### C. CAN Driver Code & ISRs
- **CAN Driver Region:** `0x022E00–0x024200`
- **CAN Interrupt Vectors:**
  - `vec 0x05E0` → `0x022F5E`
  - `vec 0x05F0` → `0x022EB6`
  - `vec 0x0610` → `0x0236C4`
  - `vec 0x0620` → `0x023664`
- **Table code references:** `0x046C64`, `0x089700` (`0x7A20`), `0x02071E` (`0x89AC`), `0x026520`, `0x0943BE` (`0x89B0`), `0x09380E` (`0x8AC4`).

---

## 2. Serial Protocols — All 4 Hardware Links Decoded (`gp = 0x03FF4000`, `ep = 0x03FF1F80`, `sp = 0x03FFBFFC`)

### A. UARTA1 (`0xFFFFFA20`) — **HV-ECU ↔ Inverter (HTM / MTH 100-Byte DMA Protocol)**
- **ISR:** `vec 0x0680` → **`0x02FCDA`**
- **Hardware Init / Control:** `0x02AA82–0x02ABAC`, `0x02FBEC–0x02FC90`
- **RX Frame (`mth_data`, 100 bytes = `0x64`):**
  - DMA RX buffer: `0x03FF9C61–0x03FF9CC4` (`gp + 0x5C61`) → copied on valid frame to `0x03FF29DF` (`gp - 0x1621`).
  - **Checksum (`0x02FD0A–0x02FD40`):** 16-bit **big-endian** unsigned byte sum of `RX[0..97]` (`0x00..0x61`), stored at `RX[98..99]` (`0x03FF9CC2–0x03FF9CC3`).
  - **Validity gate (`0x02FD42`):** checks bit 0 of `RX[80]` (`0x03FF9CB1`) before copying to the active RX buffer.

### B. CSIB1 (`0xFFFFFD20`) — **Main-CPU ↔ Sub-CPU (66-Byte Safety-Critical DMA SPI)**
- **ISR:** `vec 0x0640` → **`0x02F3B6`**
- **Hardware Init / DMA Start:** `0x02ABEA–0x02AC0E`, `0x02F5BC–0x02F62C` (`CB1CTL0=0x99`, `CB1CTL2=0x07` 8-bit, DMA count = `0x42` = 66 bytes)
- **Buffers:**
  - TX staging: `0x03FF2930–0x03FF2971` (`gp - 0x16D0`), DMA TX: `0x03FF9BD8`
  - DMA RX: `0x03FF9C1C` (`gp + 0x5C1C`), copied on valid frame to `0x03FF2972` (`gp - 0x168E`)
- **66-Byte Frame Format (`0x02F3F8–0x02F594`):**
  - `bytes[0..2]`: **`0x24, 0x24, 0x24`** (3-byte header sync)
  - `bytes[3..60]`: 58-byte payload — boolean flags are Hamming-distance-4 encoded as **`0x69` (TRUE)** vs **`0xE2` (FALSE)**; tri-state byte `TX[6]` uses `0x00 / 0x5A / 0xFF` (`0x02F680–0x02F74E`).
  - `bytes[61..62]`: **16-bit big-endian byte sum** of `bytes[3..60]` (`sum & 0xFFFF`).
  - `bytes[63..65]`: **`0xDB, 0xDB, 0xDB`** (3-byte footer sync = `~0x24`).
- **TX Payload Builder:** `0x02F680` · **RX Payload Parser:** `0x02F750`

### C. CSIB0 (`0xFFFFFD00`) — **10-Byte Bit-Inverted / Bit-Reversed SPI (On-Board ASIC / Sub-Link)**
- **ISR:** `vec 0x0630` → **`0x02E128`**
- **Hardware Init / Start:** `0x02ABC6–0x02ABE6`, `0x02E2A4–0x02E2E0` (`CB0CTL0=0x98`, `CB0CTL1=0x02`, `CB0CTL2=0x07` 8-bit)
- **10-Byte Frame (`0x02E14C–0x02E1A2`):**
  - TX buffer: `0x03FF27BD–0x03FF27C6` (`gp - 0x1843`)
  - RX buffer: `0x03FF27C8` (`gp - 0x1838`) → double-buffered to `0x03FF27D4` (`gp - 0x182C`) on byte 10.
  - Each received byte is bitwise inverted (`not r10`) and bit-reversed via `0x02E7D6` (nibble LUT at `0x01558D`).

### D. UARTA0 (`0xFFFFFA00`) — **16-Byte Async Serial / LIN (A/C Compressor / Aux Candidate)**
- **ISR:** `vec 0x0660` → **`0x085FD4`**
- **Hardware Init:** `0x02AA82–0x02AAA8`, `0x02B02C–0x02B052`
- **16-Byte Frame (`0x085FE0–0x08604C`):**
  - Inter-byte timeout check (`0x05DC` = 1,500 ticks on `0xFFFFFB28`) resets frame index to 0.
  - Stores 16 bytes (`0..15`) into `0x03FF2BB4–0x03FF2BC3` (`gp - 0x144C`).
