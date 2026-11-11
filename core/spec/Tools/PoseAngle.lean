-- SPDX-License-Identifier: MIT
import Sinew.Rot3
/-! Per-capture mean pose and pairwise angles, computed in matrix space.
    Usage: poseangle label=path [label=path ...]
    Each path is an `ms,hex72` frame log. Keeps 0x18 frames with node byte 6 = 0x00.
    The quaternion (W,X,Y,Z at bytes 10-17, Int16 LE) is converted to a 3x3 rotation
    at the boundary and only matrices are averaged. Accel (bytes 18-23) is used
    only as a direction, to compare against the rotation's row and column. -/
open Sinew.Rot3

def hexNibble (c : Char) : Option Nat :=
  if '0' ≤ c && c ≤ '9' then some (c.toNat - '0'.toNat)
  else if 'a' ≤ c && c ≤ 'f' then some (c.toNat - 'a'.toNat + 10)
  else if 'A' ≤ c && c ≤ 'F' then some (c.toNat - 'A'.toNat + 10)
  else none

def parseHexBytes (payload : String) : Option (Array Nat) := Id.run do
  let cs := payload.toList
  if cs.length % 2 != 0 then return none
  let mut out : Array Nat := #[]
  let mut i := 0
  while i < cs.length do
    match hexNibble cs[i]!, hexNibble cs[i+1]! with
    | some hi, some lo => out := out.push (hi * 16 + lo)
    | _, _ => return none
    i := i + 2
  return some out

def s16 (lo hi : Nat) : Float :=
  let u : Int := Int.ofNat (hi * 256 + lo)
  Float.ofInt (if u >= 32768 then u - 65536 else u)

def frameOf (line : String) : Option (Array Nat) :=
  match parseHexBytes ((line.splitOn ",").getLast!.trimAscii.toString) with
  | some bs => if bs.size == 36 && bs[4]! == 0x18 && bs[6]! == 0 then some bs else none
  | none => none

structure CapAcc where
  sumR : Mat3 := Mat3.zero
  sumA : Vec3 := Vec3.zero
  sumU : Vec3 := Vec3.zero
  n : Nat := 0
  deriving Inhabited

def accumulate (acc : CapAcc) (bs : Array Nat) : CapAcc :=
  let w := s16 bs[10]! bs[11]!
  let x := s16 bs[12]! bs[13]!
  let y := s16 bs[14]! bs[15]!
  let z := s16 bs[16]! bs[17]!
  if w * w + x * x + y * y + z * z < 1000000.0 then acc
  else
    let a : Vec3 := ⟨s16 bs[18]! bs[19]! / 16384.0, s16 bs[20]! bs[21]! / 16384.0,
                     s16 bs[22]! bs[23]! / 16384.0⟩
    { acc with sumR := acc.sumR.add (Mat3.ofQuat w x y z),
               sumA := acc.sumA.add a,
               sumU := acc.sumU.add a.normalize,
               n := acc.n + 1 }

def loadCapture (path : String) : IO CapAcc := do
  let text ← IO.FS.readFile path
  let mut acc : CapAcc := {}
  for line in text.splitOn "\n" do
    match frameOf line with
    | some bs => acc := accumulate acc bs
    | none => pure ()
  return acc

def meanRot (acc : CapAcc) : Mat3 :=
  Mat3.orthonormalize (Mat3.scale (1.0 / Float.ofNat acc.n) acc.sumR)

def meanAccel (acc : CapAcc) : Vec3 := Vec3.scale (1.0 / Float.ofNat acc.n) acc.sumA

def meanDir (acc : CapAcc) : Vec3 := acc.sumU.normalize

def deg (r : Float) : Float := r * 180.0 / 3.141592653589793

def showV (v : Vec3) : String := s!"({v.x}, {v.y}, {v.z})"

def showM (m : Mat3) : String :=
  s!"[{m.r0.x} {m.r0.y} {m.r0.z} | {m.r1.x} {m.r1.y} {m.r1.z} | {m.r2.x} {m.r2.y} {m.r2.z}]"

def splitPair (s : String) : String × String :=
  match s.splitOn "=" with
  | [l, p] => (l, p)
  | _ => ("capture", s)

def main (args : List String) : IO UInt32 := do
  if args.isEmpty then
    IO.eprintln "usage: poseangle label=path [label=path ...]"
    return 2
  let mut labels : Array String := #[]
  let mut accs : Array CapAcc := #[]
  for a in args do
    let (l, p) := splitPair a
    let acc ← loadCapture p
    if acc.n == 0 then
      IO.eprintln s!"{l}: no usable 0x18 node-0 frames in {p}"
      return 1
    labels := labels.push l
    accs := accs.push acc
  for i in [0:labels.size] do
    let acc := accs[i]!
    let R := meanRot acc
    let ma := meanAccel acc
    let d := meanDir acc
    IO.println s!"{labels[i]!}: n={acc.n} mean accel g={showV ma} |a|={ma.norm}"
    IO.println s!"  R = {showM R}  det={R.det}  isRotation={isRotation R}"
    IO.println s!"  accel-dir vs R e3 (col2): {deg (vecAngle d (R.col 2))} deg"
    IO.println s!"  accel-dir vs Rᵀ e3 (row2): {deg (vecAngle d (R.row 2))} deg"
  IO.println "pairwise (geodesic is convention-free; accel angle should match the convention that tracks gravity):"
  for i in [0:labels.size] do
    for j in [i+1:labels.size] do
      let Ri := meanRot accs[i]!
      let Rj := meanRot accs[j]!
      let di := meanDir accs[i]!
      let dj := meanDir accs[j]!
      IO.println s!"{labels[i]!} vs {labels[j]!}: geodesic={deg (angleBetween Ri Rj)} deg, accel={deg (vecAngle di dj)} deg, R e3 diff={deg (vecAngle (Ri.col 2) (Rj.col 2))} deg, Rᵀ e3 diff={deg (vecAngle (Ri.row 2) (Rj.row 2))} deg"
  return 0
