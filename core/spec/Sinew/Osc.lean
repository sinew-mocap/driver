-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee
/-
  Sinew.Osc — pure OSC 1.0 packet builder.

  Raw tracker output:
    /sinew/tracker   s f f f f d   (name  qw qx qy qz  time_s)
    /sinew/accel     s f f f       (name  ax ay az)

  time_s — elapsed seconds since the driver's output start, OSC type 'd'
  (float64), following the VMC /VMC/Ext/T convention.  Sent as double from
  the driver so the receiver needs no widening; float64 keeps sub-microsecond
  precision for sessions of any practical length.
-/

namespace Sinew.Osc

-- ── Float32 encoding via C FFI ───────────────────────────────────────────

-- C: uint32_t sinew_f64_to_f32_bits(double d)
@[extern "sinew_f64_to_f32_bits"]
opaque f64ToF32Bits (f : Float) : UInt32

-- ── OSC wire helpers ─────────────────────────────────────────────────────

private def padTo4 (n : Nat) : Nat := (n + 3) / 4 * 4

private def appendOscString (acc : Array UInt8) (s : String) : Array UInt8 :=
  let bytes1 := s.toUTF8.push 0
  let padded := padTo4 bytes1.size
  acc ++ bytes1.data ++ Array.replicate (padded - bytes1.size) 0

private def appendOscFloat (acc : Array UInt8) (f : Float) : Array UInt8 :=
  let bits := f64ToF32Bits f
  acc.push (bits >>> 24).toUInt8
     |>.push (bits >>> 16).toUInt8
     |>.push (bits >>> 8).toUInt8
     |>.push bits.toUInt8

-- OSC type 'd' (float64): the raw IEEE-754 double bits, big-endian.
-- Mirrors the C driver's osc_double (= sinew_driver.c).
private def appendOscDouble (acc : Array UInt8) (f : Float) : Array UInt8 :=
  let bits := f.toBits
  acc.push (bits >>> 56).toUInt8
     |>.push (bits >>> 48).toUInt8
     |>.push (bits >>> 40).toUInt8
     |>.push (bits >>> 32).toUInt8
     |>.push (bits >>> 24).toUInt8
     |>.push (bits >>> 16).toUInt8
     |>.push (bits >>> 8).toUInt8
     |>.push bits.toUInt8

-- ── Message builders ──────────────────────────────────────────────────────

-- Raw tracker quaternion + VMC-style elapsed time.
-- time_s = elapsed seconds since the output thread started, OSC type 'd'
-- (float64), following the VMC /VMC/Ext/T convention.
def buildTracker (name : String) (qw qx qy qz time_s : Float) : ByteArray :=
  let acc := appendOscString #[] "/sinew/tracker"
  let acc := appendOscString acc ",sffffd"
  let acc := appendOscString acc name
  let acc := appendOscFloat  acc qw
  let acc := appendOscFloat  acc qx
  let acc := appendOscFloat  acc qy
  let acc := appendOscFloat  acc qz
  let acc := appendOscDouble acc time_s
  ByteArray.mk acc

-- Body-frame accelerometer in g units
def buildAccel (name : String) (ax ay az : Float) : ByteArray :=
  let acc := appendOscString #[] "/sinew/accel"
  let acc := appendOscString acc ",sfff"
  let acc := appendOscString acc name
  let acc := appendOscFloat  acc ax
  let acc := appendOscFloat  acc ay
  let acc := appendOscFloat  acc az
  ByteArray.mk acc

end Sinew.Osc
