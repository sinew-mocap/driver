-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee

/-!
# Sinew Serial Protocol — Channels, Tracker Identity, Body Map

How the time-division-multiplexed USB stream (`Sinew.Protocol.Spec`,
*Sub-sensor Encoding*) is demultiplexed into per-tracker channels, how a tracker
is fingerprinted across pairings, and the current body-part map.

## TDMA channels

**Channel naming**: `sinew_<hub>_<idx>_s<sub>_f<bit0>` for IMU-class frames.
Including `_f<bit0>` is what demultiplexes two body parts that otherwise both
stream through the same `_s<N>` slot.

Up to 5 (hub, idx) bases × 3 subs × 2 session bits = 30 logical TDMA slots — far
more than the 15 physical trackers, so most slots are unallocated in any session.

**Dynamic slot assignment** (caveat): the dongle assigns trackers to slots by
pairing order, not statically per body part.  The same body part paired alone vs
alongside others may land on a different `(sub, bit0)` slot, so a static
`(channel → joint)` map is brittle — it survives only if the same set of trackers
is paired in the same order.  The per-tracker hwid (below) is the robust key.

### 0x18 high-rate stream (15 TDMA slots)

Some 0x18 frames use a **shifted layout** (hub at bytes 29-31, idx at byte 32,
byte 33 = 0x00) vs the standard layout (hub at bytes 30-32, idx at byte 33).  The
parser detects and normalises both; the shifted variant clobbers the 3rd mag
axis (see `Spec`).  Channels:

- `sinew_342_3_s<0..2>_f<0..1>`, `sinew_342_4_*`, `sinew_340_7_*`,
  `sinew_340_8_*`, `sinew_340_10_*` (standard layout)
- `sinew_000000_40_*`, `sinew_00x00_40_*`, `sinew_01x00_40_*`,
  `sinew_ffffff_255_*` — same physical traffic under shifted layout (byte 33 = 0).
  These are real per-tracker streams the dongle routes through dynamically, not
  broadcast/echo channels.

### 0x3a burst stream (15 nodes)

See `Type0x3a.lean` for the full node table.  **Identity note:** bytes 30-32 in
0x3a frames are session-scoped (change each power cycle); bytes 24-26 are the
stable per-physical-node hardware ID.  The parser uses bytes 24-26 as `hubId`
for 0x3a frames.

## Offline placeholder

When a paired tracker is asleep or powered off, the dongle keeps the USB
byte-cadence alive with a synthetic filler frame in the now-empty TDMA slot at
~30 Hz.  Signature (RightHand off→on):

| Bytes | Value                  | Notes |
|-------|------------------------|-------|
| 4     | `0x?D`                 | upper nibble varies (`0x1D`, `0x0D`, `0xFD`, `0x3D`) |
| 5     | usually `0x14`         | varies; not part of identity |
| 6–7   | `c8 8e`                | seq_num field, frozen |
| 8–9   | `76 a3`                | status field, frozen |
| 10–13 | `65 00 28 50`          | constant block |
| 15–16 | cycles `a6a6` / `f9f9` / `5353` | three-state TDMA sub-slot indicator |
| 18    | `0x40` (mostly)        | |
| 31–32 | `00 ff` → `00 00` near transitions | |
| 33    | `0x07`                 | |

`parseFrame` recognises this by the **8-byte run at bytes 6..13**
(`c8 8e 76 a3 65 00 28 50`), which never appears in real frames, and returns
`.offline` so callers can drop them.  Wake-up takes ~30 ms after a button press
and needs zero host writes: the tracker self-announces its hwid in 0x?A burst
frames, then the high-rate IMU stream resumes.

## Tracker fingerprinting (most stable per-tracker ID)

0x?D status frames (lower nibble D, parsed as IMU class) carry a stable
per-tracker 3-byte hardware ID in bytes 30-32, with a per-frame `tracker_idx`
(byte 33) from a small fixed set per tracker.  Each physical tracker emits
**4 distinct (hwid, idx) pairs** at ~3 Hz (4 status subtypes or 4 internal IMU
chips).  The set of 4 hwids is unique per tracker and stable across
pairing-order changes — the most reliable fingerprint.

| Body part      | (hwid, idx) pairs                                      |
|----------------|--------------------------------------------------------|
| `#0` Hips      | `(603f5e, 248)`, `(abd9b2, 32)`, `(55ea58, 32)`, `(c9e904, 253)` |
| `#7` Chest     | `(42ed05, 246)`, `(5aad55, 32)`, `(628260, 240)`, `(ad04b1, 32)` |

Heartbeat channel `sinew_000000_0_s0_f0` carries an extra per-active-tracker bit
at **byte 26** (`0x00` for Hips, `0xff` for Chest).

## Body Map

| Channel              | Source                       | Body part      |
|----------------------|------------------------------|----------------|
| `sinew_342_3_s0_f1`  | Head-only isolation          | `#8` Head      |
| `sinew_342_3_s1_f0`  | Head-only isolation          | `#8` Head      |
| `sinew_342_3_s2_f0`  | Head-only isolation          | `#8` Head      |
| `sinew_342_3_s0_f0`  | #7 Chest-only isolation      | `#7` Chest     |
| `sinew_342_3_s1_f1`  | #7 Chest-only isolation      | `#7` Chest     |
| `sinew_342_3_s2_f1`  | #7 Chest-only isolation      | `#7` Chest     |

`#7` Chest and `#0` Hips claim the SAME `(sub, f)` slots when each is paired
alone; with both paired, Hips keeps them and Chest moves to other channels.  This
is why the static map in `tui/src/main.cpp :: kChannelToJoint` fits only one
pairing configuration.  Unmapped: both UpperLegs/LowerLegs, both Feet, both
UpperArms/LowerArms, both Hands.

The authoritative `(channel → joint)` table lives consumer-side
(`tui/src/main.cpp :: kChannelToJoint`); the Lean driver emits raw channel ids
and lets the consumer apply labelling.

Each physical tracker has a number 0..14 on its case, matching Rebocap's
`current_device_info.json` node ordering:

```
 0  Hips           5  LeftFoot       10  RightUpperArm
 1  LeftUpperLeg   6  RightFoot      11  LeftLowerArm
 2  RightUpperLeg  7  Chest          12  RightLowerArm
 3  LeftLowerLeg   8  Head           13  LeftHand
 4  RightLowerLeg  9  LeftUpperArm   14  RightHand
```

## Stream isolation

The driver uses one stream per canonical channel name (name-based isolation) and
accepts all non-duplicate frames regardless of seq gap, so each TDMA slot is
tracked independently without cross-contamination.
-/

namespace Sinew.Protocol.Channels

/-- The 15 Rebocap NodeNumbers, indexed by body-part number on the case. -/
def nodeNames : Array String := #[
  "Hips", "LeftUpperLeg", "RightUpperLeg", "LeftLowerLeg", "RightLowerLeg",
  "LeftFoot", "RightFoot", "Chest", "Head", "LeftUpperArm",
  "RightUpperArm", "LeftLowerArm", "RightLowerArm", "LeftHand", "RightHand"]

/-- Magic 8-byte run at bytes 6..13 marking an offline-placeholder filler frame. -/
def offlineMagic : Array UInt8 := #[0xc8, 0x8e, 0x76, 0xa3, 0x65, 0x00, 0x28, 0x50]

end Sinew.Protocol.Channels
