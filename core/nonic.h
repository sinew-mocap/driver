// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Nonic (degree-9) Hermite interpolation, C⁴ continuous — the C++ mirror of the
// Lean proof of record in spec/Sinew/Nonic.lean (ten boundary conditions +
// c4_join).  The Lean coefficients are scaled by 24 (so integer data stays
// integral and `omega` closes the proof); here we carry the true interpolant H
// in double, i.e. those coefficients divided by 24.
//
// Used to interpolate the continuous rotation kinematics so reported derivatives
// are the proven C⁴ model's, not an ad-hoc finite-difference cascade.
#pragma once

namespace nonic {

/// Endpoint sample: value p, velocity v, acceleration a, jerk j, snap s.
struct Knot {
	double p, v, a, j, s;
};

/// A degree-9 polynomial by its ten coefficients (c0 + c1·t + … + c9·t⁹).
struct Poly {
	double c[10];
};

/// H(t) for left/right endpoint data on [0,1].  c_k = (Lean's ×24 coefficient)/24,
/// so H(0)=l.p, H'(0)=l.v, H''(0)=l.a, H'''(0)=l.j, H''''(0)=l.s, and likewise at
/// t=1 for r (see spec/Sinew/Nonic.lean).
inline Poly hermite(const Knot &l, const Knot &r) {
	Poly q;
	q.c[0] = l.p;
	q.c[1] = l.v;
	q.c[2] = l.a / 2.0;
	q.c[3] = l.j / 6.0;
	q.c[4] = l.s / 24.0;
	q.c[5] = (-3024 * l.p + 3024 * r.p - 1680 * l.v - 1344 * r.v - 420 * l.a + 252 * r.a -
	          60 * l.j - 24 * r.j - 5 * l.s + r.s) /
	         24.0;
	q.c[6] = (10080 * l.p - 10080 * r.p + 5376 * l.v + 4704 * r.v + 1260 * l.a - 924 * r.a +
	          160 * l.j + 92 * r.j + 10 * l.s - 4 * r.s) /
	         24.0;
	q.c[7] = (-12960 * l.p + 12960 * r.p - 6720 * l.v - 6240 * r.v - 1512 * l.a + 1272 * r.a -
	          180 * l.j - 132 * r.j - 10 * l.s + 6 * r.s) /
	         24.0;
	q.c[8] = (7560 * l.p - 7560 * r.p + 3840 * l.v + 3720 * r.v + 840 * l.a - 780 * r.a + 96 * l.j +
	          84 * r.j + 5 * l.s - 4 * r.s) /
	         24.0;
	q.c[9] = (-1680 * l.p + 1680 * r.p - 840 * l.v - 840 * r.v - 180 * l.a + 180 * r.a - 20 * l.j -
	          20 * r.j - l.s + r.s) /
	         24.0;
	return q;
}

/// Horner evaluation H(t).
inline double eval(const Poly &q, double t) {
	double r = q.c[9];
	for (int k = 8; k >= 0; --k) {
		r = q.c[k] + t * r;
	}
	return r;
}

/// Formal derivative H'(t): drop the constant, scale each term by its power.
inline Poly deriv(const Poly &q) {
	Poly d;
	for (int k = 0; k < 9; ++k) {
		d.c[k] = (double)(k + 1) * q.c[k + 1];
	}
	d.c[9] = 0.0;
	return d;
}

}  // namespace nonic
