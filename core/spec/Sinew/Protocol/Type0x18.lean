-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee

import Sinew.Protocol.Spec

/-!
# Type 0x18 — Main IMU (~62.5 Hz, 16 ms TDMA cycle)

**Group 1, message class 8.**
Primary high-rate IMU stream carrying quaternion + body-frame accelerometer
for up to 5 tracker slots (T3, T4 from hub 342; T7, T8, T10 from hub 340).

Each slot may carry **two physical sensors** interleaved, distinguished by the
upper nibble of byte 5 (0 = primary, 1 = secondary, 2 = tertiary).

## Frame Layout

| Bytes  | Content | Notes |
|--------|---------|-------|
| 0–3    | `FA FA AE AE` | Sync |
| 4      | `0x18` | Type |
| 5      | sub-type | upper nibble = sub-sensor; lower nibble = cycling value |
| 6–7    | Sequence LE | global frame counter |
| 8–9    | Status LE | session ID |
| 10–11  | Quaternion W | `Int16` LE, unnormalized |
| 12–13  | Quaternion X | `Int16` LE |
| 14–15  | Quaternion Y | `Int16` LE |
| 16–17  | Quaternion Z | `Int16` LE |
| 18–19  | Accel X | `Int16` LE, body-frame, ±2g = ±16384 LSB |
| 20–21  | Accel Y | `Int16` LE |
| 22–23  | Accel Z | `Int16` LE |
| 24–29  | Magnetometer (3×i16le) | **standard variant only** — clean Earth-field vector, per-node-constant `\|m\|`.  In the shifted variant the hub ASCII overwrites bytes 29–31, so only mag-X (24–25) and mag-Y (26–27) survive and mag-Z (28–29) is garbage; host 9-axis fusion therefore uses standard frames only. |
| 29–31  | Hub suffix ASCII | `"342"` / `"340"` — **shifted variant** (byte 33 = 0); clobbers mag-Z |
| 30–32  | Hub suffix ASCII | `"342"` / `"340"` — **standard variant** (byte 33 ≠ 0); mag intact at 24–29 |
| 32     | Tracker index | shifted variant only |
| 33     | Tracker index | 3, 4, 7, 8, or 10 — standard variant; `0x00` in shifted |
| 34–35  | Reserved | `0x00` |

## Sample Frame

```
fafaaeae 18 15 0232 0000 6fbf 7999 f1da 4c56 cae5 7ec6 333430 07 00 00
         ^^ ^^                     w    x    y    z    ax   az hub idx
         |  byte5=0x15 → sub-sensor 1 (secondary)
         type=0x18
```

Decoded: hub = `"340"`, idx = 7 → `sinew_340_7` (secondary: `sinew_340_7_s1`).
Accel X = −13595/16384 = −0.83 g.

## Tracker Slots

| idx | Hub | OSC primary | Rate |
|-----|-----|-------------|------|
| 3  | 342 | `sinew_342_3`  | ~31 Hz |
| 4  | 342 | `sinew_342_4`  | ~8 Hz  |
| 7  | 340 | `sinew_340_7`  | ~8 Hz  |
| 8  | 340 | `sinew_340_8`  | ~8 Hz  |
| 10 | 340 | `sinew_340_10` | ~8 Hz  |

## Normalization

```
mag = √(W²+X²+Y²+Z²)  (raw Int16, typically 15000–35000)
q = (W, X, Y, Z) / mag
```

Accelerometer: 1 g = `Sinew.Protocol.Spec.accel1gLsb` LSB.
-/

namespace Sinew.Protocol.Type0x18

open Sinew.Protocol.Spec

/-- Byte 4 value identifying this packet type. -/
def typeId : UInt8 := PKT_MAIN_IMU

/-- TDMA cycle period in milliseconds: the byte-7 counter ticks once per
    cycle.  The cycle is 16.00 ms (median of 99 clean +1 counter-steps in a
    low-movement capture). -/
def cyclePeriodMs : Nat := 16

/-- TDMA cycle rate in Hz, as a rational: `1000 / 16 = 125/2 = 62.5`.
    See `cyclePeriodMs`. -/
def rateHz : Rat := (1000 : Rat) / (cyclePeriodMs : Rat)

/-- Byte offsets for the four quaternion Int16 fields. -/
def quatWOff : Nat := 10
def quatXOff : Nat := 12
def quatYOff : Nat := 14
def quatZOff : Nat := 16

/-- Byte offsets for the three accelerometer Int16 fields. -/
def accelXOff : Nat := 18
def accelYOff : Nat := 20
def accelZOff : Nat := 22

/-- Byte offset for the 3-byte ASCII hub suffix (e.g. `"342"`, `"340"`). -/
def hubSuffixOff : Nat := 30

/-- Byte offset for the tracker index (3, 4, 7, 8, or 10). -/
def trackerIdxOff : Nat := 33

/-- Raw (wire) quaternion: sign-extended Int16 values before normalization. -/
structure RawQuat where
  w : Int
  x : Int
  y : Int
  z : Int
  deriving Repr

/-- Raw accelerometer counts (Int16). Divide by `accel1gLsb` for g units. -/
structure RawAccel where
  x : Int
  y : Int
  z : Int
  deriving Repr

/-- Fully decoded 0x18 frame payload. -/
structure Frame where
  /-- Byte 5 upper nibble: 0 = primary, 1 = secondary, 2 = tertiary sensor. -/
  subSensor  : UInt8
  seqNum     : UInt16
  status     : UInt16
  quat       : RawQuat
  accel      : RawAccel
  /-- 3-byte ASCII hub suffix from bytes 30–32. -/
  hubSuffix  : String
  /-- Byte 33 tracker index (3, 4, 7, 8, or 10). -/
  trackerIdx : UInt8
  deriving Repr

/-- OSC channel name for a (hub, trackerIdx, subSensor) triple. -/
def oscName (hub : String) (idx : UInt8) (sub : UInt8) : String :=
  let base := s!"sinew_{hub}_{idx}"
  if sub == 0 then base else s!"{base}_s{sub}"

end Sinew.Protocol.Type0x18
