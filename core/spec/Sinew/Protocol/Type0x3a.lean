-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee

import Sinew.Protocol.Spec

/-!
# Type 0x3a — Group-3 Burst (~3 Hz per node)

**Group 3, message class A.**

Per-node burst frames for 13 additional RF nodes not in the 0x18 high-rate stream.
The dongle polls them autonomously at ~3 Hz without any host command.

Byte layout is **identical to 0x18** for the quaternion + accel region (bytes
10–23).  Identity layout differs: bytes 24–26 carry the **stable per-physical-
node hardware ID** (durable across power cycles and pairings); bytes 30–32
are session-scoped and must NOT be used for identity.  The parser uses
bytes 24–26 as `hubId` for 0x3a frames.

## Frame Layout

| Bytes  | Content | Notes |
|--------|---------|-------|
| 0–3    | `FA FA AE AE` | Sync |
| 4      | `0x3A` | Type |
| 5      | flags | `0x00` or `0x20` — **not** a sub-sensor index; ignore for node identity |
| 6–7    | Sequence LE | node's own sequence counter |
| 8–9    | Status LE | session ID |
| 10–17  | Quaternion W/X/Y/Z | `Int16` LE — same format as 0x18 |
| 18–23  | Accel X/Y/Z | `Int16` LE — same scale as 0x18 (16384 LSB/g) |
| 24–26  | **Stable hardware ID** | 3-byte per-physical-node ID, durable across power cycles and pairings.  The parser uses this as `hubId` for 0x3a frames. |
| 27–29  | Trailing field | 0x18-class frames carry magnetometer at bytes 24–29; for 0x3a the first three of those are repurposed as hwid and the trailing three carry an unvalidated field |
| 30–32  | Session-scoped ID | changes every power cycle — DO NOT use for identity |
| 33     | RF slot address | e.g. `0x13`, `0x75`, `0x80` |
| 34–35  | Trailer | varies; `0x82` = marker byte |

Byte offsets 10–23 and 33 are identical to `Type0x18`; parsers can share that logic.

## Sample Frame

```
fafaaeae 3a 00 00735a 1d5c 4075 2c75 674b 5175 5075 ... 6021b2 75 3f 82
                      w    x    y    z    ax        ...  hw_id  slot
```

Decoded: hardware ID = `6021b2`, RF slot = 0x75 = 117 → `sinew_6021b2_117`.

## Nodes (15 total)

| RF slot (dec) | Hardware ID | OSC name | Rate |
|---------------|------------|----------|------|
| 11  | `656a4c` | `sinew_656a4c_11`  | ~3 Hz |
| 19  | `56ff5a` | `sinew_56ff5a_19`  | ~3 Hz |
| 117 | `6021b2` | `sinew_6021b2_117` | ~3 Hz |
| 128 | `6389a4` | `sinew_6389a4_128` | ~3 Hz |
| 134 | `5f5a71` | `sinew_5f5a71_134` | ~3 Hz |
| 160 | `60c472` | `sinew_60c472_160` | ~2 Hz |
| 195 | `677ec2` | `sinew_677ec2_195` | ~3 Hz |
| 204 | `5b8b2e` | `sinew_5b8b2e_204` | ~3 Hz |
| 223 | `60a3e7` | `sinew_60a3e7_223` | ~3 Hz |
| 237 | `5db857` | `sinew_5db857_237` | ~3 Hz |
| 243 | `5a829d` | `sinew_5a829d_243` | ~3 Hz |
| 243 | `5f59bb` | `sinew_5f59bb_243` | ~3 Hz |
| 247 | `5c0ce0` | `sinew_5c0ce0_247` | ~3 Hz |
| 248 | `5d91e9` | `sinew_5d91e9_248` | ~3 Hz |
| 249 | `63e5a1` | `sinew_63e5a1_249` | ~3 Hz |

RF slot 243 is shared by two nodes — distinguished by hardware ID.

## Autonomous Streaming

These frames stream **autonomously without any host command**. The macOS driver needs
no initialization sequence to receive them. The Windows app upgrades them to the
high-rate stream (~62.5 Hz TDMA) via an init sequence, but 3 Hz suffices for
presence detection.
-/

namespace Sinew.Protocol.Type0x3a

open Sinew.Protocol.Spec

/-- Byte 4 value identifying this packet type. -/
def typeId : UInt8 := PKT_BURST_G3

/-- Nominal sample rate in Hz. -/
def rateHz : Nat := 3

-- Sensor field offsets are identical to Type0x18; reuse them symbolically.
def quatWOff   : Nat := 10
def quatXOff   : Nat := 12
def quatYOff   : Nat := 14
def quatZOff   : Nat := 16
def accelXOff  : Nat := 18
def accelYOff  : Nat := 20
def accelZOff  : Nat := 22

/-- Byte offset of the 3-byte stable hardware ID (per-physical-node,
    durable across power cycles).  Bytes 30–32 on 0x3a frames look like a
    hwid but are session-scoped and must not be used for identity. -/
def hwIdOff : Nat := 24
def hwIdLen : Nat := 3

/-- Byte offset of the RF slot address. -/
def rfSlotOff : Nat := 33

/-- A known RF node entry. -/
structure NodeEntry where
  rfSlot : UInt8
  hwId   : Array UInt8   -- 3 bytes
  deriving Repr

/-- The RF nodes. -/
def nodes : Array NodeEntry := #[
  { rfSlot := 0x13, hwId := #[0x56, 0xFF, 0x5A] },
  { rfSlot := 0x75, hwId := #[0x60, 0x21, 0xB2] },
  { rfSlot := 0x80, hwId := #[0x63, 0x89, 0xA4] },
  { rfSlot := 0x86, hwId := #[0x5F, 0x5A, 0x71] },
  { rfSlot := 0xA0, hwId := #[0x60, 0xC4, 0x72] },
  { rfSlot := 0xC3, hwId := #[0x67, 0x7E, 0xC2] },
  { rfSlot := 0xCC, hwId := #[0x5B, 0x8B, 0x2E] },
  { rfSlot := 0xDF, hwId := #[0x60, 0xA3, 0xE7] },
  { rfSlot := 0xED, hwId := #[0x5D, 0xB8, 0x57] },
  { rfSlot := 0xF3, hwId := #[0x5A, 0x82, 0x9D] },
  { rfSlot := 0xF3, hwId := #[0x5F, 0x59, 0xBB] },  -- shared slot
  { rfSlot := 0xF7, hwId := #[0x5C, 0x0C, 0xE0] },
  { rfSlot := 0xF8, hwId := #[0x5D, 0x91, 0xE9] },
  { rfSlot := 0xF9, hwId := #[0x63, 0xE5, 0xA1] },
  { rfSlot := 0x0B, hwId := #[0x65, 0x6A, 0x4C] }
]

def hexByte (b : UInt8) : String :=
  let n (x : UInt8) : Char :=
    if x < 10 then Char.ofNat (x.toNat + '0'.toNat)
    else Char.ofNat (x.toNat - 10 + 'a'.toNat)
  String.ofList [n (b >>> 4), n (b &&& 0x0F)]

/-- OSC channel name derived from 3-byte hardware ID and RF slot address. -/
def oscName (h0 h1 h2 : UInt8) (slot : UInt8) : String :=
  s!"sinew_{hexByte h0}{hexByte h1}{hexByte h2}_{slot}"

end Sinew.Protocol.Type0x3a
