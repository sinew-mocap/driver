-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee
--
-- Nonic (degree-9) Hermite interpolation on [0,1], proven C⁴ continuous.
--
-- C⁴ continuity means value, velocity, acceleration, jerk AND snap are continuous
-- at a knot: 5 boundary conditions at each of 2 endpoints = 10 constraints, so the
-- interpolant has degree 9 — a *nonic*.  This is the order the rotation kinematics
-- need: angular snap (ω‴, the 4th derivative region) only stays continuous across
-- knots at C⁴; a C³ (degree-7) interpolant leaves snap jumping at every knot.
--
-- Coefficients are scaled by 24 so integer endpoint data yields integer
-- coefficients and the boundary identities close by `omega` (pure Lean core); the
-- same identities hold over ℝ.  Reused per component on the continuous 6D rotation
-- representation (two columns + cross-product recovery) so no quaternion narrowing.

namespace Sinew.Nonic

/-- An endpoint sample: value `p`, velocity `v`, acceleration `a`, jerk `j`, snap `s`. -/
structure Knot where
  p : Int
  v : Int
  a : Int
  j : Int
  s : Int
deriving Repr, DecidableEq

/-- A degree-9 polynomial by its ten coefficients: c0 + c1·t + … + c9·t⁹. -/
structure Poly where
  c0 : Int
  c1 : Int
  c2 : Int
  c3 : Int
  c4 : Int
  c5 : Int
  c6 : Int
  c7 : Int
  c8 : Int
  c9 : Int
deriving Repr, DecidableEq

/-- Horner evaluation H(t). -/
def Poly.eval (q : Poly) (t : Int) : Int :=
  q.c0 + t * (q.c1 + t * (q.c2 + t * (q.c3 + t * (q.c4 + t * (q.c5 + t * (q.c6 +
    t * (q.c7 + t * (q.c8 + t * q.c9))))))))

/-- Formal derivative H'(t): drop the constant, scale each term by its power. -/
def Poly.deriv (q : Poly) : Poly :=
  ⟨q.c1, 2 * q.c2, 3 * q.c3, 4 * q.c4, 5 * q.c5, 6 * q.c6, 7 * q.c7, 8 * q.c8, 9 * q.c9, 0⟩

/-- `24·H`, the nonic Hermite on [0,1] for left/right endpoint data `l`, `r`.
    (Scaled by 24 to stay in ℤ; the true interpolant is this divided by 24.) -/
def hermite24 (l r : Knot) : Poly :=
  { c0 := 24 * l.p
    c1 := 24 * l.v
    c2 := 12 * l.a
    c3 := 4 * l.j
    c4 := l.s
    c5 := -3024 * l.p + 3024 * r.p - 1680 * l.v - 1344 * r.v - 420 * l.a + 252 * r.a -
          60 * l.j - 24 * r.j - 5 * l.s + r.s
    c6 := 10080 * l.p - 10080 * r.p + 5376 * l.v + 4704 * r.v + 1260 * l.a - 924 * r.a +
          160 * l.j + 92 * r.j + 10 * l.s - 4 * r.s
    c7 := -12960 * l.p + 12960 * r.p - 6720 * l.v - 6240 * r.v - 1512 * l.a + 1272 * r.a -
          180 * l.j - 132 * r.j - 10 * l.s + 6 * r.s
    c8 := 7560 * l.p - 7560 * r.p + 3840 * l.v + 3720 * r.v + 840 * l.a - 780 * r.a +
          96 * l.j + 84 * r.j + 5 * l.s - 4 * r.s
    c9 := -1680 * l.p + 1680 * r.p - 840 * l.v - 840 * r.v - 180 * l.a + 180 * r.a -
          20 * l.j - 20 * r.j - l.s + r.s }

-- ── The ten C⁴ boundary conditions ───────────────────────────────────────────
-- 24·H and its first four derivatives hit the prescribed value / velocity /
-- acceleration / jerk / snap at each end (the factor 24 is the scale above).

theorem val0 (l r : Knot) : (hermite24 l r).eval 0 = 24 * l.p := by
  simp [Poly.eval, hermite24]
theorem vel0 (l r : Knot) : (hermite24 l r).deriv.eval 0 = 24 * l.v := by
  simp [Poly.eval, Poly.deriv, hermite24]
theorem acc0 (l r : Knot) : (hermite24 l r).deriv.deriv.eval 0 = 24 * l.a := by
  simp [Poly.eval, Poly.deriv, hermite24]; omega
theorem jerk0 (l r : Knot) : (hermite24 l r).deriv.deriv.deriv.eval 0 = 24 * l.j := by
  simp [Poly.eval, Poly.deriv, hermite24]; omega
theorem snap0 (l r : Knot) : (hermite24 l r).deriv.deriv.deriv.deriv.eval 0 = 24 * l.s := by
  simp [Poly.eval, Poly.deriv, hermite24]; omega
theorem val1 (l r : Knot) : (hermite24 l r).eval 1 = 24 * r.p := by
  simp [Poly.eval, hermite24]; omega
theorem vel1 (l r : Knot) : (hermite24 l r).deriv.eval 1 = 24 * r.v := by
  simp [Poly.eval, Poly.deriv, hermite24]; omega
theorem acc1 (l r : Knot) : (hermite24 l r).deriv.deriv.eval 1 = 24 * r.a := by
  simp [Poly.eval, Poly.deriv, hermite24]; omega
theorem jerk1 (l r : Knot) : (hermite24 l r).deriv.deriv.deriv.eval 1 = 24 * r.j := by
  simp [Poly.eval, Poly.deriv, hermite24]; omega
theorem snap1 (l r : Knot) : (hermite24 l r).deriv.deriv.deriv.deriv.eval 1 = 24 * r.s := by
  simp [Poly.eval, Poly.deriv, hermite24]; omega

/-- C⁴ join: at a shared knot `b`, the right end of segment (a,b) agrees with the
    left end of segment (b,c) in value and all four derivatives.  A spline of nonic
    segments through shared knots is therefore C⁴ — value, velocity, acceleration,
    jerk and snap are all continuous across every knot. -/
theorem c4_join (a b c : Knot) :
    (hermite24 a b).eval 1 = (hermite24 b c).eval 0 ∧
    (hermite24 a b).deriv.eval 1 = (hermite24 b c).deriv.eval 0 ∧
    (hermite24 a b).deriv.deriv.eval 1 = (hermite24 b c).deriv.deriv.eval 0 ∧
    (hermite24 a b).deriv.deriv.deriv.eval 1 = (hermite24 b c).deriv.deriv.deriv.eval 0 ∧
    (hermite24 a b).deriv.deriv.deriv.deriv.eval 1 =
      (hermite24 b c).deriv.deriv.deriv.deriv.eval 0 := by
  refine ⟨?_, ?_, ?_, ?_, ?_⟩
  · rw [val1, val0]
  · rw [vel1, vel0]
  · rw [acc1, acc0]
  · rw [jerk1, jerk0]
  · rw [snap1, snap0]

end Sinew.Nonic
