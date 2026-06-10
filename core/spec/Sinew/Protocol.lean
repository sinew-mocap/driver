-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee

import Sinew.Protocol.Spec
import Sinew.Protocol.HostCommands
import Sinew.Protocol.Channels
import Sinew.Protocol.Type0x18
import Sinew.Protocol.Type0x3a

/-
  Sinew.Protocol — pure parsing of 36-byte Sinew frames.

  Wire schema is defined in Sinew.Protocol.{Spec, HostCommands, Channels, Type0x*}:
    Spec         — read-frame layout, packet taxonomy, sub-sensor encoding.
    HostCommands — host → dongle writes and the firmware flow.
    Channels     — TDMA demux, tracker fingerprinting, body map, offline filler.
    Type0x18 / Type0x3a — per-packet field offsets.
  This module provides the runtime parser and normalised output types.
-/

namespace Sinew.Protocol

open Sinew.Protocol.Spec

-- ── Wire constants (from spec) ────────────────────────────────────────────

def FRAME_BYTES  : Nat   := frameBytes
def SYNC_0       : UInt8 := sync0
def SYNC_1       : UInt8 := sync1
def SYNC_2       : UInt8 := sync2
def SYNC_3       : UInt8 := sync3
def PKT_IMU      : UInt8 := PKT_MAIN_IMU
def PKT_EXTENDED : UInt8 := PKT_EXTENDED_IMU
def PKT_STATUS   : UInt8 := PKT_STATUS_G1
def PKT_BURST3   : UInt8 := PKT_BURST_G3
def ACCEL_1G     : Float := accel1gLsb

-- ── Normalised output types ───────────────────────────────────────────────

structure Quaternion where
  w : Float
  x : Float
  y : Float
  z : Float
  deriving Repr

-- Body-frame accelerometer in g units
structure Accel where
  x : Float
  y : Float
  z : Float
  deriving Repr

def Accel.magnitude (a : Accel) : Float :=
  Float.sqrt (a.x*a.x + a.y*a.y + a.z*a.z)

structure ImuFrame where
  pktType    : UInt8
  subType    : UInt8
  seqNum     : UInt16
  status     : UInt16
  trackerIdx : UInt8
  -- (hubId, trackerIdx) is the globally unique tracker identity.
  -- For 0x18: 3-byte ASCII suffix from bytes 30-32 (e.g. "342", "340").
  -- For 0x3a: 3-byte hex hardware ID from bytes 30-32 (e.g. "6021b2").
  hubId      : String
  quat       : Quaternion
  accel      : Accel        -- body-frame, in g (raw / ACCEL_1G)
  -- Raw Int16 accelerometer counts before scaling — used to diagnose
  -- wrong scale factors or byte-offset errors across tracker types.
  accelRaw   : Int × Int × Int
  deriving Repr

inductive ParseResult where
  | ok        : ImuFrame → ParseResult
  | skipped   : UInt8 → ParseResult
  | badSync   : ParseResult
  | tooShort  : ParseResult
  -- Synthetic "tracker offline" filler the dongle emits in an empty TDMA
  -- slot when the paired tracker is asleep or powered off.  Signature:
  -- bytes 6..13 are a fixed magic block c8 8e 76 a3 65 00 28 50 regardless
  -- of pkt_type or byte 5.  Real frames never exhibit this 8-byte run.
  | offline   : ParseResult
  deriving Repr

-- ── Byte helpers ──────────────────────────────────────────────────────────

private def getU8 (b : ByteArray) (i : Nat) : UInt8 :=
  if h : i < b.size then b[i] else 0

private def getI16LE (b : ByteArray) (i : Nat) : Int :=
  let lo := (getU8 b i).toNat
  let hi := (getU8 b (i + 1)).toNat
  let raw := lo + hi * 256
  if raw > 32767 then raw - 65536 else raw

private def getU16LE (b : ByteArray) (i : Nat) : UInt16 :=
  (getU8 b i).toUInt16 ||| ((getU8 b (i + 1)).toUInt16 <<< 8)

private def intToFloat (n : Int) : Float :=
  match n with
  | Int.ofNat   k => k.toFloat
  | Int.negSucc k => -(k.succ.toFloat)

private def normaliseQuat (rw rx ry rz : Int) : Quaternion :=
  let fw := intToFloat rw; let fx := intToFloat rx
  let fy := intToFloat ry; let fz := intToFloat rz
  let mag := Float.sqrt (fw*fw + fx*fx + fy*fy + fz*fz)
  if mag < 1.0 then { w := 1.0, x := 0.0, y := 0.0, z := 0.0 }
  else { w := fw/mag, x := fx/mag, y := fy/mag, z := fz/mag }

private def rawToAccel (rx ry rz : Int) : Accel :=
  { x := intToFloat rx / ACCEL_1G
    y := intToFloat ry / ACCEL_1G
    z := intToFloat rz / ACCEL_1G }

-- Extract the 3-byte ASCII hub suffix and tracker index for 0x18 frames.
-- Standard layout: hub at bytes 30-32, idx at byte 33.
-- Shifted layout:  hub at bytes 29-31, idx at byte 32, byte 33 = 0x00.
-- The shifted variant is identified by byte 33 = 0.  Both layouts carry the
-- same physical tracker; the shift is an undocumented frame variant.
private def extractHubAndIdx (b : ByteArray) : String × UInt8 :=
  let hex (x : UInt8) : String :=
    let nibble (n : UInt8) : Char :=
      if n < 10 then Char.ofNat (n.toNat + '0'.toNat)
      else Char.ofNat (n.toNat - 10 + 'a'.toNat)
    String.ofList [nibble (x >>> 4), nibble (x &&& 0x0F)]
  let byteStr (x : UInt8) : String :=
    if x >= 0x20 && x <= 0x7E then String.singleton (Char.ofNat x.toNat)
    else hex x
  let hub (off : Nat) :=
    byteStr (getU8 b off) ++ byteStr (getU8 b (off+1)) ++ byteStr (getU8 b (off+2))
  if getU8 b 33 == 0
  then (hub 29, getU8 b 32)   -- shifted layout
  else (hub 30, getU8 b 33)   -- standard layout

-- ── Main parser ────────────────────────────────────────────────────────────

private def isOfflinePlaceholder (b : ByteArray) : Bool :=
  getU8 b  6 == 0xc8 && getU8 b  7 == 0x8e &&
  getU8 b  8 == 0x76 && getU8 b  9 == 0xa3 &&
  getU8 b 10 == 0x65 && getU8 b 11 == 0x00 &&
  getU8 b 12 == 0x28 && getU8 b 13 == 0x50

def parseFrame (b : ByteArray) : ParseResult :=
  if b.size < FRAME_BYTES then .tooShort
  else if getU8 b 0 != SYNC_0 || getU8 b 1 != SYNC_1 ||
          getU8 b 2 != SYNC_2 || getU8 b 3 != SYNC_3 then .badSync
  else if isOfflinePlaceholder b then .offline
  else
    let pktType := getU8 b 4
    -- Lower nibble selects message class (8/9/D = IMU layout, A = burst);
    -- upper nibble identifies node group / channel.  Match by class so any
    -- group's frame is accepted (0x18, 0x1D, 0x29, ... → IMU).
    let cls := pktType &&& 0x0F
    if cls == 0x8 || cls == 0x9 || cls == 0xD then
      -- Layout defined in Sinew.Protocol.Type0x18
      let (hubId, trackerIdx) := extractHubAndIdx b
      let ax := getI16LE b Type0x18.accelXOff
      let ay := getI16LE b Type0x18.accelYOff
      let az := getI16LE b Type0x18.accelZOff
      .ok {
        pktType
        subType    := getU8 b 5
        seqNum     := getU16LE b 6
        status     := getU16LE b 8
        trackerIdx
        hubId
        quat       := normaliseQuat
                        (getI16LE b Type0x18.quatWOff) (getI16LE b Type0x18.quatXOff)
                        (getI16LE b Type0x18.quatYOff) (getI16LE b Type0x18.quatZOff)
        accel      := rawToAccel ax ay az
        accelRaw   := (ax, ay, az)
      }
    else if cls == 0xA then
      -- Layout defined in Sinew.Protocol.Type0x3a.
      -- Sensor field offsets identical to 0x18; bytes 30-32 = non-ASCII hardware ID.
      .ok {
        pktType
        subType    := getU8 b 5
        seqNum     := getU16LE b 6
        status     := getU16LE b 8
        trackerIdx := getU8 b Type0x3a.rfSlotOff
        -- bytes 24-26 are the stable per-physical-node hardware ID (durable across restarts).
        -- bytes 30-32 change every session and must NOT be used as identity.
        hubId      := Type0x3a.hexByte (getU8 b 24) ++ Type0x3a.hexByte (getU8 b 25) ++ Type0x3a.hexByte (getU8 b 26)
        quat       := normaliseQuat
                        (getI16LE b Type0x3a.quatWOff) (getI16LE b Type0x3a.quatXOff)
                        (getI16LE b Type0x3a.quatYOff) (getI16LE b Type0x3a.quatZOff)
        accel      := rawToAccel
                        (getI16LE b Type0x3a.accelXOff)
                        (getI16LE b Type0x3a.accelYOff)
                        (getI16LE b Type0x3a.accelZOff)
        accelRaw   := (getI16LE b Type0x3a.accelXOff,
                       getI16LE b Type0x3a.accelYOff,
                       getI16LE b Type0x3a.accelZOff)
      }
    else .skipped pktType

-- ── Sync alignment ────────────────────────────────────────────────────────

def findSync (b : ByteArray) : Option Nat :=
  let rec loop (i : Nat) : Option Nat :=
    if i + 3 >= b.size then none
    else if getU8 b i     == SYNC_0 && getU8 b (i+1) == SYNC_1 &&
            getU8 b (i+2) == SYNC_2 && getU8 b (i+3) == SYNC_3
         then some i
    else loop (i + 1)
  termination_by b.size - i
  loop 0

end Sinew.Protocol
