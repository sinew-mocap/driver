// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Online hard+soft iron magnetic calibration, one slot per joint.
//
// Algorithm: accumulate (R_clean, B_raw) pairs during normal wear; solve
//   B_calibrated = H·B_raw + b  s.t.  B_calibrated ≈ R_cleanᵀ·B_global
// via online least-squares (4-variable linear regression, 3 rows).
// B_global is bootstrapped by averaging R_clean·B_raw over the first 200 frames.
// Calibration is considered valid after MAG_CALIB_MIN_SAMPLES samples.
// Results are written to mag_calib.ini whenever the solve is refreshed.
#pragma once
#include "sinew_osc.h"   // Quat, Accel
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAG_CALIB_MIN_SAMPLES 500   // minimum samples before H/b are applied
#define MAG_CALIB_SOLVE_EVERY 100   // re-solve + rewrite INI after this many new samples

// Load calibration from INI file (call once at startup, before processing frames).
// Returns number of joints loaded (0 if file absent — not an error).
int  mag_calib_load(const char *path);

// Push a new (orientation, raw-mag, raw-accel) observation for joint ji (0-14).
// q_clean is the TIC-corrected quaternion (absolute, sensor→world).
// Braw is the raw magnetometer reading; araw is the raw accelerometer reading.
// Internally accumulates least-squares matrices and re-solves periodically.
// Writes updated INI after each solve.
void mag_calib_push(int ji, Quat q_clean, Accel Braw, Accel araw, const char *ini_path);

// Returns 1 if both Bg (earth magnetic field) and Ag (gravity direction) are known.
// Both are in global/world frame.
int mag_calib_get_refs(int ji, float Bg[3], float Ag[3]);

// Apply calibration to a raw magnetometer reading.
// Writes the calibrated vector into Bcal_out.
// Returns 1 if a valid calibration exists for ji, 0 if not yet converged
// (Bcal_out is set to Braw unchanged in that case).
int  mag_calib_apply(int ji, Accel Braw, Accel *Bcal_out);

// Joint name used as INI section key; must match SINEW_JOINT_NODE_TABLE order.
void mag_calib_set_joint_name(int ji, const char *name);

// Calibration progress 0.0–1.0:
//   0.0–0.5  bootstrap phase (Bg/Ag collection, needs diverse orientations)
//   0.5–1.0  least-squares fitting phase (approaches 1.0 at MIN_SAMPLES)
float mag_calib_get_progress(int ji);

#ifdef __cplusplus
}
#endif
