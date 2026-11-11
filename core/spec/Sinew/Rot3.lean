-- SPDX-License-Identifier: MIT
/-! 3x3 rotation matrices (rows stored). The quaternion exists only at the wire
    boundary: `Mat3.ofQuat` takes the decoded W,X,Y,Z once and returns the rotation.
    Everything downstream of decode carries a matrix (memory #481). Averaging is done
    in matrix space, which is sign-invariant (q and -q give the same R). -/
namespace Sinew.Rot3

structure Vec3 where
  x : Float
  y : Float
  z : Float
  deriving Inhabited

structure Mat3 where
  r0 : Vec3
  r1 : Vec3
  r2 : Vec3
  deriving Inhabited

namespace Vec3
def zero : Vec3 := ⟨0.0, 0.0, 0.0⟩
def dot (a b : Vec3) : Float := a.x * b.x + a.y * b.y + a.z * b.z
def cross (a b : Vec3) : Vec3 :=
  ⟨a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x⟩
def add (a b : Vec3) : Vec3 := ⟨a.x + b.x, a.y + b.y, a.z + b.z⟩
def sub (a b : Vec3) : Vec3 := ⟨a.x - b.x, a.y - b.y, a.z - b.z⟩
def scale (s : Float) (a : Vec3) : Vec3 := ⟨s * a.x, s * a.y, s * a.z⟩
def norm (a : Vec3) : Float := Float.sqrt (a.dot a)
def normalize (a : Vec3) : Vec3 := Vec3.scale (1.0 / a.norm) a
end Vec3

namespace Mat3

def zero : Mat3 := ⟨Vec3.zero, Vec3.zero, Vec3.zero⟩
def identity : Mat3 := ⟨⟨1.0, 0.0, 0.0⟩, ⟨0.0, 1.0, 0.0⟩, ⟨0.0, 0.0, 1.0⟩⟩

def row (m : Mat3) (i : Nat) : Vec3 :=
  if i == 0 then m.r0 else if i == 1 then m.r1 else m.r2

def col (m : Mat3) (j : Nat) : Vec3 :=
  if j == 0 then ⟨m.r0.x, m.r1.x, m.r2.x⟩
  else if j == 1 then ⟨m.r0.y, m.r1.y, m.r2.y⟩
  else ⟨m.r0.z, m.r1.z, m.r2.z⟩

def transpose (m : Mat3) : Mat3 := ⟨m.col 0, m.col 1, m.col 2⟩

def mul (a b : Mat3) : Mat3 :=
  let bt := b.transpose
  ⟨⟨a.r0.dot bt.r0, a.r0.dot bt.r1, a.r0.dot bt.r2⟩,
   ⟨a.r1.dot bt.r0, a.r1.dot bt.r1, a.r1.dot bt.r2⟩,
   ⟨a.r2.dot bt.r0, a.r2.dot bt.r1, a.r2.dot bt.r2⟩⟩

def add (a b : Mat3) : Mat3 := ⟨a.r0.add b.r0, a.r1.add b.r1, a.r2.add b.r2⟩
def scale (s : Float) (m : Mat3) : Mat3 :=
  ⟨Vec3.scale s m.r0, Vec3.scale s m.r1, Vec3.scale s m.r2⟩

def trace (m : Mat3) : Float := m.r0.x + m.r1.y + m.r2.z
def det (m : Mat3) : Float := m.r0.dot (m.r1.cross m.r2)

/-- Rotation from the wire quaternion W,X,Y,Z (normalised here). Active convention:
    row i of R is the body-frame basis expressed as world components. -/
def ofQuat (w0 x0 y0 z0 : Float) : Mat3 :=
  let n := Float.sqrt (w0 * w0 + x0 * x0 + y0 * y0 + z0 * z0)
  let w := w0 / n
  let x := x0 / n
  let y := y0 / n
  let z := z0 / n
  ⟨⟨1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)⟩,
   ⟨2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)⟩,
   ⟨2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)⟩⟩

/-- Gram-Schmidt re-orthonormalisation: rows 0 and 1, row 2 = row0 × row1. -/
def orthonormalize (m : Mat3) : Mat3 :=
  let r0 := m.r0.normalize
  let r1 := (Vec3.sub m.r1 (Vec3.scale (m.r1.dot r0) r0)).normalize
  ⟨r0, r1, r0.cross r1⟩

end Mat3

def clamp1 (x : Float) : Float :=
  if x > 1.0 then 1.0 else if x < -1.0 then -1.0 else x

/-- Geodesic angle (radians) between two rotations: acos((tr(R1ᵀR2) - 1) / 2). -/
def angleBetween (a b : Mat3) : Float :=
  Float.acos (clamp1 ((Mat3.trace (Mat3.mul a.transpose b) - 1.0) / 2.0))

/-- Angle (radians) between two directions. -/
def vecAngle (u v : Vec3) : Float :=
  Float.acos (clamp1 (u.normalize.dot v.normalize))

def approx (a b : Float) (eps : Float := 1e-9) : Bool := Float.abs (a - b) < eps

def flat (m : Mat3) : List Float :=
  [m.r0.x, m.r0.y, m.r0.z, m.r1.x, m.r1.y, m.r1.z, m.r2.x, m.r2.y, m.r2.z]

def eqM (a b : Mat3) (eps : Float := 1e-9) : Bool :=
  (flat a).zip (flat b) |>.all (fun p => approx p.1 p.2 eps)

def isRotation (m : Mat3) : Bool :=
  eqM (Mat3.mul m.transpose m) Mat3.identity 1e-9 && approx m.det 1.0 1e-9

-- Build-time guards: a wrong convention or a broken formula fails `lake build`.
def diagXPi : Mat3 := ⟨⟨1.0, 0.0, 0.0⟩, ⟨0.0, -1.0, 0.0⟩, ⟨0.0, 0.0, -1.0⟩⟩
def rzPi2 : Mat3 :=
  Mat3.ofQuat 0.70710678118654752 0.0 0.0 0.70710678118654752

#guard eqM (Mat3.ofQuat 1.0 0.0 0.0 0.0) Mat3.identity
#guard eqM (Mat3.ofQuat 0.0 1.0 0.0 0.0) diagXPi
#guard eqM (Mat3.ofQuat (-0.5) 0.5 (-0.5) 0.5) (Mat3.ofQuat 0.5 (-0.5) 0.5 (-0.5))
#guard isRotation (Mat3.ofQuat 0.3 0.5 (-0.7) 0.2)
#guard approx (angleBetween Mat3.identity Mat3.identity) 0.0 1e-6
#guard approx (angleBetween Mat3.identity rzPi2) (3.141592653589793 / 2.0) 1e-6
-- Planted control: reading the same quaternion in the wrong order must NOT match
-- the correct rotation, so the guards above can tell the conventions apart.
#guard !eqM (Mat3.ofQuat 0.0 1.0 0.0 0.0) Mat3.identity

end Sinew.Rot3
