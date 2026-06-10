-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee

/-!
# Sinew Serial Protocol — Host Writes (host → dongle)

Host writes use a framing separate from the read frames in
`Sinew.Protocol.Spec`.  SHA256 hashes of the firmware images are in
`firmware_hashes.txt`.

## Write Frame Layout (36 bytes)

| Bytes | Content             | Notes |
|-------|---------------------|-------|
| 0–1   | `EA EA`             | Host-side header. |
| 2     | `CmdId`             | one of the values below. |
| 3     | `unk_1`             | always `0x00`. |
| 4     | `unk_2`             | always `0xC8` — independent of `unk_1`, distinct from the `0xC8` OMNI value at byte 5. |
| 5     | TargetId            | dispatch target.  Three classes: `0x00..0x0E` = a specific Rebocap tracker by NodeNumber; `0xC8` = OMNI — the dongle itself (`wake_up`, `activate_node`, dongle-flash); `0xFF` = ALL_TRACKERS broadcast (every byte of the tracker-flash flow). |
| 6–7   | MessageNumber (LE)  | 16-bit per-write counter; high bit `0x8000` repurposed as a tracker-side-work flag.  **Set** when a tracker must wake/process/respond (`activate_node` to a NodeNumber; `push_node_firmware_payload` to ALL_TRACKERS during a tracker update).  **Cleared** when only the dongle acts (`wake_up` to OMNI; `shutdown_node`; `push_node_firmware_payload` to OMNI during a dongle update).  Mask with `0x7FFF` for the bare counter. |
| 8–11  | Reserved (4 bytes)  | All zeros for every `0x04` write.  An unused destination-offset slot. |
| 12–33 | Command-specific payload | 22 bytes.  For `0x04` this is firmware-image data, concatenated in MessageNumber order to recover the flash blob. |
| 34–35 | `AF AF`             | Host-side footer. |

## CmdId values

Firmware Tracker v_12 / Receiver v_6:

| CmdId | Name                              | Notes |
|-------|-----------------------------------|-------|
| `0x04` | `push_node_firmware_payload`     | TargetId `0xC8` for dongle-image bytes, `0xFF` for tracker-image bytes.  Bytes 12–33 concatenated by MessageNumber yield byte-identical dongle (43,934 B) and tracker (130,878 B) images in `firmware_hashes.txt`. |
| `0x07` | `shutdown_node`                  | **Powers the tracker fully off, not just dropping the dongle session.** After `0x07 → 0x09 (LeftUpperArm)`, even replaying a working `activate_node` fails until a physical button press.  Targets: a specific NodeNumber AND `0xFF` ALL_TRACKERS.  The ALL_TRACKERS form precedes an `activate_node` on every transition-on (clears stale state).  Each shutdown duplicates ~400 ms apart with the same MessageNumber (a retry).  Bit `0x8000` cleared. |
| `0x15` | `finish_node_firmware_update`    | Brackets each `0x04` burst (dongle: 31/31 pairs; tracker: 96 starts / 89 finishes). |
| `0x20` | `activate_node`                  | Sent when a tracker transitions on (target = specific NodeNumber, never OMNI / ALL_TRACKERS).  Bit `0x8000` SET.  Payload 22 B; bytes 14..17 CONSTANT `00 00 00 00`.  The dongle holds **persistent pairing state** — the payload is RF slot config in the dongle's flash, not a tracker-validated credential.  All-zeros works for all 15 already-paired nodes (~5,000 IMU frames each in 3 s).  `sinew_tui` 'p' runs a pairing machine: PROBE (send zeros, wait 3 s) → PAIRED if the joint responds, else NEEDS_ADDR (a never-paired tracker needs its BLE address in payload bytes 0–5).  Already-paired kits need no captured payload; durable per-kit identity lives in `tui/src/hwid_table.c`. |
| `0x21` | `gyroscope_calibration`          | |
| `0x25` | `finish_finish_node_firmware_update` | Once at the end of each completed flow (1× dongle, 1× tracker). |
| `0x27` | `wake_up`                        | **First write rebocap sends after process attach**, then a periodic ~30 s heartbeat (msg #1 at +8.5 s, #2 at +37.8 s).  Always TargetId `0xC8` OMNI.  Bit `0x8000` cleared.  Payload fixed: `00 00 00 00 00 00 b4 00` then 14 zeros; `b4` is a flag/timeout/version constant. |
| `0x31` | `set_anti_magnetic_strength`     | Numerically inverted (lower byte → higher strength). |
| `0x35` | `start_node_firmware_update`     | Brackets each `0x04` burst (counts match `0x15`). |
| `0x41` | `set_rgb`                        | |
| `0x91` | `set_transmit_power`             | Numerically inverted. |
| `0xa1` | `anti_magnetic_toggle`           | |

## Payloads

Each config command is sent as a **15-frame burst, one per node `0x00`–`0x0E`**
(not `0xFF`), work bit set.  Payload offsets below are **frame byte** positions.

- **`0x41` set_rgb** — colour at the *tail*: byte 31 = R, 32 = G, 33 = B (all
  other payload bytes 0).  Verified with navy `00 00 28`, red `64 00 00`,
  green `00 64 00`.
- **`0x21` gyroscope_calibration** — per-node IEEE-754 floats (a stored
  per-tracker cal, not a "start" trigger): bytes 12–23 = float[3] gyro-bias
  (deg/s), 24–27 = 0.0f, 28–31 = 1.0f (scale).
- **`0xa1` anti_magnetic_toggle** — byte 24 only: `0x00` ON / `0x0a` OFF.
  **This is the anti-magnetic switch, NOT the 6/9-axis selector** (a separate UI
  toggle).  Toggling also cycles every tracker (a `0x07` shutdown burst then a
  fresh `0x20` activate).
- **`0x31` set_anti_magnetic_strength** — carries per-node mag calibration with
  the level mixed in: bytes 12–29 = per-node magnetometer cal (floats, constant
  across a level change), byte 30 = level encoded inverted as `14 - N` for
  N = 1..12.  ⚠️ A `0x31` with zeroed 12–29 overwrites a tracker's mag cal, so a
  sender must *replay* the captured per-node cal and rewrite only byte 30.
- **`0x91` set_transmit_power** — byte 25 only: power inverted as `28 - N` for
  N = 1..18 (18 = +9.1 dBm).  A clean single-value command with no bundled cal.
- **`0x13` comm channel / reset** — link-wide RF setting (disrupts pairing; no
  per-node form).  Targets `0xFF` (×10 retransmits) + one `0xC8`.  Byte 5 is a
  checksum over the incrementing MessageNumber (varies per transmission).  Only
  two channels exist (no index formula); bytes 0–3 are a per-channel RF-param
  blob: channel 1 = `1b 1e 4c 1b`, default/reset = `00 28 50 00` (byte 4 = `0f`,
  byte 6 = `60` for both).  "Reset" is idempotent.

There is **no 6/9-axis mode command** — that toggle emits zero host writes
(applied host-side in Rebocap's fusion); 9-axis is implemented driver-side, see
`Sinew.Fusion`.

## Firmware-update flow

A complete update against either target follows:

```
0x35 start_node_firmware_update     ┐
0x04 push_node_firmware_payload  × N│ block (1 of K)
0x15 finish_node_firmware_update    ┘
... K blocks total ...
0x25 finish_finish_node_firmware_update  ← exactly once at the very end
```

`K` ≈ 31 for dongle v7 (~64 push writes per block) and ≈ 96 for
tracker v15 (~62 push writes per block) — the similar block size is a fixed
flash sector.  TargetId is constant within a flow: all OMNI for a dongle update,
all ALL_TRACKERS for a tracker update (the dongle re-broadcasts to in-range
trackers).

### Reconstructing a firmware blob

1. Filter the host_capture log to `,W,` lines whose CmdId byte (frame byte 2) is `0x04`.
2. Sort by MessageNumber (bytes 6–7 LE), masking with `0x7FFF`.
3. Concatenate frame bytes 12–33 (22 B per write) in that order.

The leading 8 bytes are the always-zero header from the first push; the rest is
a raw vendor firmware image.  SHA256s are in `firmware_hashes.txt`.
-/

namespace Sinew.Protocol.HostCommands

/-- Host-write header bytes 0–1 (`EA EA`) and footer bytes 34–35 (`AF AF`). -/
def hostHeader0 : UInt8 := 0xEA
def hostHeader1 : UInt8 := 0xEA
def hostFooter0 : UInt8 := 0xAF
def hostFooter1 : UInt8 := 0xAF

/-- TargetId dispatch classes (byte 5). -/
def TARGET_OMNI         : UInt8 := 0xC8
def TARGET_ALL_TRACKERS : UInt8 := 0xFF

/-- High bit of MessageNumber (bytes 6–7 LE): set ⇒ a tracker must act. -/
def MSG_TRACKER_WORK : UInt16 := 0x8000
def MSG_COUNTER_MASK : UInt16 := 0x7FFF

/-- CmdId values (byte 2). -/
def CMD_PUSH_FIRMWARE   : UInt8 := 0x04
def CMD_SHUTDOWN_NODE   : UInt8 := 0x07
def CMD_FINISH_FW       : UInt8 := 0x15
def CMD_ACTIVATE_NODE   : UInt8 := 0x20
def CMD_GYRO_CAL        : UInt8 := 0x21
def CMD_FINISH_FINISH   : UInt8 := 0x25
def CMD_WAKE_UP         : UInt8 := 0x27
def CMD_SET_ANTIMAG     : UInt8 := 0x31
def CMD_START_FW        : UInt8 := 0x35
def CMD_SET_RGB         : UInt8 := 0x41
def CMD_SET_TX_POWER    : UInt8 := 0x91
def CMD_ANTIMAG_TOGGLE  : UInt8 := 0xA1

/-- Payload offsets (frame byte positions; see "Payloads"). -/
def RGB_R_OFF        : Nat := 31      -- 0x41: R/G/B at bytes 31/32/33
def RGB_G_OFF        : Nat := 32
def RGB_B_OFF        : Nat := 33
def ANTIMAG_OFF      : Nat := 24      -- 0xa1: 0x00 on / 0x0a off
def MAG_LEVEL_OFF    : Nat := 30      -- 0x31: level = 14 - N (N = 1..12)
def TX_POWER_OFF     : Nat := 25      -- 0x91: power = 28 - N (N = 1..18)

/-- Inverted-level encodings used by `0x31` (mag strength) and `0x91` (TX power). -/
def magLevelByte (n : UInt8) : UInt8 := 14 - n
def txPowerByte  (n : UInt8) : UInt8 := 28 - n

end Sinew.Protocol.HostCommands
