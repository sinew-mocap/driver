-- SPDX-License-Identifier: MIT
/-!
# Sinew magnetometer calibration (sim-only, no device writes)

Pass schedule, sphere fit, heading error, and the 22-byte `[mag]` payload
layout from `adapters/activates.ini`.  Payload byte 0 is frame byte 12, so
ini byte 18 is frame byte 30 (the level, `14 - N`).
-/

namespace Sinew.MagCal

/-- Even slots first, then odd slots (the user's pass order). -/
def passOrder : List Nat := (List.range 15).filter (· % 2 == 0) ++ (List.range 15).filter (· % 2 == 1)

/-- Two-face schedule: +Z then -Z, one CCW turn each. -/
inductive Face | posZ | negZ
  deriving Repr, BEq

def schedule : List Face := [.posZ, .negZ]

/-- Solve a 4x4 linear system by Gaussian elimination with partial pivoting. -/
def solve4 (a : Array (Array Float)) (b : Array Float) : Array Float := Id.run do
  let mut m := (Array.range 4).map fun i => (a[i]!).push b[i]!
  for col in [0:4] do
    let mut piv := col
    for r in [col+1:4] do
      if (m[r]![col]!).abs > (m[piv]![col]!).abs then piv := r
    let tmp := m[col]!
    m := m.set! col m[piv]!
    m := m.set! piv tmp
    for r in [0:4] do
      if r != col then
        let f := m[r]![col]! / m[col]![col]!
        m := m.set! r ((m[r]!).mapIdx fun j v => v - f * m[col]![j]!)
  return (Array.range 4).map fun i => m[i]![4]! / m[i]![i]!

/-- Sphere fit: solve [2x 2y 2z 1]·p = |m|^2 by normal equations; centre = p[0..3]. -/
def fitSphere (pts : List (Float × Float × Float)) : Float × Float × Float := Id.run do
  let mut ata : Array (Array Float) := (Array.range 4).map fun _ => (Array.range 4).map fun _ => 0.0
  let mut atb : Array Float := (Array.range 4).map fun _ => 0.0
  for (x, y, z) in pts do
    let row : Array Float := #[2*x, 2*y, 2*z, 1.0]
    let rhs := x*x + y*y + z*z
    for i in [0:4] do
      atb := atb.set! i (atb[i]! + row[i]! * rhs)
      for j in [0:4] do
        ata := ata.set! i (ata[i]!.set! j (ata[i]![j]! + row[i]! * row[j]!))
  let p := solve4 ata atb
  return (p[0]!, p[1]!, p[2]!)

/-- Heading-error metric: angle of a residual against |B|, in radians. -/
def headingErr (err magnitude : Float) : Float := Float.atan (2.0 * err / magnitude)

/-- Decode an IEEE-754 binary32 from little-endian bytes (no Float bit cast needed). -/
def f32le (b0 b1 b2 b3 : UInt8) : Float :=
  let bits : UInt32 := b0.toUInt32 ||| (b1.toUInt32 <<< 8) ||| (b2.toUInt32 <<< 16) ||| (b3.toUInt32 <<< 24)
  let sign := if (bits >>> 31) == 1 then -1.0 else 1.0
  let expo := ((bits >>> 23) &&& 0xFF).toNat
  let mant := (bits &&& 0x7FFFFF).toNat
  if expo == 0 then sign * (mant.toFloat) * (2.0 ^ (-149.0))
  else sign * ((mant + 8388608).toFloat) * (2.0 ^ (expo.toFloat - 150.0))

/-- Ini payload (22 bytes): three offset floats at 0/4/8, two zero bytes at 12-13,
    an unverified float at 14, the level byte at 18, and trailing bytes 19-21. -/
structure MagPayload where
  offset : Float × Float × Float
  raw    : ByteArray   -- all 22 bytes, preserved verbatim
  deriving Inhabited

/-- Parse a 22-byte payload; the offsets are read, the rest of the bytes are kept as-is. -/
def MagPayload.ofBytes (bs : ByteArray) : Option MagPayload :=
  if bs.size != 22 then none else
  let f (i : Nat) := f32le (bs.get! i) (bs.get! (i+1)) (bs.get! (i+2)) (bs.get! (i+3))
  some { offset := (f 0, f 4, f 8), raw := bs }

/-! Regression guards: a broken fit or pass order fails `lake build`. -/
#guard passOrder == [0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13]
#guard schedule.length == 2
#guard (f32le 0x00 0x00 0x80 0x3f == 1.0)
#guard (headingErr 5.0 400.0 - 0.024995).abs < 1e-5
#guard
  let c := fitSphere [(6.0,-3.0,2.0),(4.0,-3.0,2.0),(5.0,-2.0,2.0),(5.0,-4.0,2.0),(5.0,-3.0,3.0),(5.0,-3.0,1.0)]
  (c.1 - 5.0).abs < 1e-6 && (c.2.1 + 3.0).abs < 1e-6 && (c.2.2 - 2.0).abs < 1e-6
