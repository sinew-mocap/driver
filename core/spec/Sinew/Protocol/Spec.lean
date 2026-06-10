-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee

/-!
# Sinew Serial Protocol — Read Frames (device → host)

All frames are exactly **36 bytes**.  Read frames carry sync marker
`FA FA AE AE` at bytes 0–3 in the parser's view.  An equivalent framing
(firmware Tracker v_12 / Receiver v_6) describes the true frame as
`AE AE <body 32 B> FA FA`, where the 4-byte "sync" is the back-to-back footer
`FA FA` of the previous frame plus header `AE AE` of the current one.  Both
describe the same byte stream; the parser finds boundaries via the `FA FA AE AE`
marker.

Companion modules:
- `Sinew.Protocol.HostCommands` — host → dongle writes and the firmware flow.
- `Sinew.Protocol.Channels` — TDMA demux, tracker fingerprinting, body map,
  offline filler.
- `Sinew.Protocol.Type0x18` / `Type0x3a` — per-packet field offsets.

## Frame Layout (36 bytes)

| Bytes  | Content              | Notes |
|--------|----------------------|-------|
| 0–3    | `FA FA AE AE`        | Sync marker (= prev-footer + cur-header in the alternate framing). |
| 4      | Packet type          | upper nibble = node group, lower nibble = msg class (`read_code`). |
| 5      | Sub-sensor + flags   | see *Sub-sensor Encoding* below. |
| 6      | NodeNumber           | Rebocap body-part index 0..14, `0xC8` for OMNI/broadcast.  0x18 frames from the `sinew_340_8` channel (RightHand) carry `0x0E = 14 = RightHand` here. |
| 7      | Per-frame counter    | Increments by 1 each frame within a stream; wraps at 256. |
| 8–9    | Status (LE)          | session-scoped 16-bit ID.  `0x0000` for ≥99.7 % of high-rate frames — NOT shared with burst-frame status values and cannot be used for joint demux.  Joint identity comes from byte 6. |
| 10–17  | Quaternion (4×i16le) | normalised after dividing by sqrt(sum²); raw range ±32768. |
| 18–23  | Accelerometer (3×i16le) | body-frame, nominal ±2 g full-scale at ±16384 LSB (Q14).  **Uncalibrated**: at near-rest `\|a\|` ranges ~0.8–1.6 g across nodes with per-tracker bias/scale (node 9 alone swings 0.9→2.2 g), so it is NOT a reliable gravity reference and must not be `\|a\|≈1 g`-gated.  For tilt the onboard quaternion (already accel-fused for pitch/roll) is trustworthy; raw accel is ignored downstream (`/sinew/accel`). |
| 24–29  | **Magnetometer (3×i16le)** | each axis sweeps ±20–30k LSB while magnitude stays in 39550..39849 (0.7 % variation) — Earth field rotating through a stationary chip.  **VALID ONLY IN STANDARD LAYOUT** (byte 33 ≠ 0): standard frames give a per-node-constant `\|m\|` (node 2 = 39752 ± 3; per-node offset is hard-iron bias).  In the **shifted layout** (byte 33 = 0) the hub ASCII occupies bytes 29–31, so the 3rd mag axis (bytes 28–29) is clobbered.  Host fusion reads mag from standard frames only. |
| 30–32  | hub_id / hwid (3 bytes) | 0x18 standard = ASCII hub suffix (e.g. `342`); 0x18 shifted writes hub at bytes 29-31 instead; 0x?D status frames carry a per-tracker stable hardware ID here.  This is the RF-slot identifier used by the dongle's TDMA scheduler — distinct from the NodeNumber at byte 6. |
| 33     | tracker_idx          | RF slot 0..14 for true 0x3a burst; hub-internal index for 0x18; `0x00` marks shifted layout. |
| 34–35  | Per-channel constant (2 bytes) | CONSTANT per channel (`0x0000` for `sinew_342_3`, `0x000675` for shifted-layout aliases, `0x0007` for the heartbeat).  Not a CRC (no CRC-16 or sum/xor checksum matches); a frame-format version tag. |

## Node Group / Message Class Encoding (byte 4)

Upper nibble (node group):
- `0x0` host/dongle — `0x0a`, `0x0d`
- `0x1` node group 1 — `0x18`, `0x19`, `0x1a`, `0x1d` (primary trackers)
- `0x2` node group 2 — `0x2a`, `0x2d`
- `0x3` node group 3 — `0x3a`, `0x3d` (extra trackers)
- `0xF` firmware/config — `0xf9`, `0xfd`

Lower nibble (message class):
- `0x8` Main IMU — quaternion + accelerometer at ~62.5 Hz (16 ms TDMA cycle)
- `0x9` Extended IMU — quaternion + ASCII node ID at ~70 Hz
- `0xA` Burst — aggregate or per-node sensor data at ~5 Hz
- `0xD` Status/battery — session metadata at ~4 Hz

## Sub-sensor Encoding (byte 5)

Byte 5 decomposes into three orthogonal fields:

| Bits  | Field            | Notes |
|-------|------------------|-------|
| 4..7  | sub-sensor index | 0..2 — IMU chip / TDMA sub-slot |
| 1..3  | frame counter    | 3-bit, cycles 0..7 within a stream — drop detection |
| 0     | session bit      | per-tracker discriminator: two trackers can share the same `(hub, idx, sub)` slot, distinguished only by this bit |

The dongle USB output is time-division multiplexed: each 36-byte frame is one
slot, demultiplexed by `(hub:idx, sub-sensor, session-bit)` — see
`Sinew.Protocol.Channels` for the channel-naming and slot-assignment details.

## Packet Types

| Type | Rate | Description |
|------|------|-------------|
| `0x18` | ~62.5/s | **Primary IMU** / `tracking_data` — quaternion + accel, 11 channels |
| `0x19` | ~70/s  | **Extended IMU** / `tracking_data_sleep` — quaternion + ASCII node ID |
| `0x3a` | ~3/s   | **Group-3 burst** — per-node quat + accel, 15 nodes |
| `0x1a` | ~5/s   | Group-1 burst aggregate |
| `0x1d` | ~4/s   | Status / session metadata; `0xfd` heartbeat |
| `0x0a` | ~5/s   | Host/dongle RF-slot poll data |
| `0x0f` | rare   | Generic response |
| `0xf9` | ~5/s   | Firmware/config response |
| `0x0d`, `0x2a`, `0x2d`, `0x3d`, `0xfd` | rare | Offline-placeholder variants (`Channels`) and other stubs |
| `0x9d`, `0xb9`, `0xe9`, `0xed`, `0xfa`, `0x88`, `0x98` | sporadic | "node hit" / timeout signals |
-/

namespace Sinew.Protocol.Spec

/-- All Sinew frames are exactly 36 bytes. -/
def frameBytes : Nat := 36

/-- Sync marker bytes 0–3. -/
def sync0 : UInt8 := 0xFA
def sync1 : UInt8 := 0xFA
def sync2 : UInt8 := 0xAE
def sync3 : UInt8 := 0xAE

/-- Packet type constants (byte 4). -/
def PKT_MAIN_IMU     : UInt8 := 0x18
def PKT_EXTENDED_IMU : UInt8 := 0x19
def PKT_BURST_G1     : UInt8 := 0x1A
def PKT_STATUS_G1    : UInt8 := 0x1D
def PKT_HOST_BURST   : UInt8 := 0x0A
def PKT_HOST_STATUS  : UInt8 := 0x0D
def PKT_BURST_G2     : UInt8 := 0x2A
def PKT_STATUS_G2    : UInt8 := 0x2D
def PKT_BURST_G3     : UInt8 := 0x3A
def PKT_STATUS_G3    : UInt8 := 0x3D
def PKT_FIRMWARE     : UInt8 := 0xF9
def PKT_FW_STATUS    : UInt8 := 0xFD

/-- Accelerometer scale: ±2g full-scale at ±16384 LSB (Q14 format). -/
def accel1gLsb : Float := 16384.0

/-- Common header fields shared by all 36-byte frames. -/
structure FrameHeader where
  /-- Byte 4: upper nibble = node group; lower nibble = message class. -/
  pktType : UInt8
  /-- Byte 5: upper nibble = physical sub-sensor index (0 = primary, 1 = secondary). -/
  subType : UInt8
  /-- Byte 6: Rebocap NodeNumber 0..14 (body-part index), 0xC8 for OMNI.
      Byte 7: per-frame counter, increments by 1 per frame, wraps at 256.
      Stored as LE uint16 so seqNum = NodeNumber | (counter << 8).
      seqNum & 0xFF = NodeNumber — the primary joint-identity field. -/
  seqNum  : UInt16
  /-- Bytes 8–9 LE: almost always 0x0000 in high-rate frames; not used
      for demux. -/
  status  : UInt16
  deriving Repr

end Sinew.Protocol.Spec
