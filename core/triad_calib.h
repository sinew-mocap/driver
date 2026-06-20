// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// TRIAD algorithm: compute sensor→world quaternion from two reference vector pairs.
// Uses gravity and calibrated magnetometer to establish a physics-grounded zero-pose.
#pragma once
#include "sinew_osc.h"  // Quat

#ifdef __cplusplus
extern "C" {
#endif

// Compute sensor→world quaternion from two reference vector pairs.
// a_sensor: initial accel in sensor frame (gravity direction at powerup)
// m_sensor: initial calibrated mag in sensor frame
// Ag_world: gravity direction in world frame (from mag_calib bootstrap)
// Bg_world: earth magnetic field direction in world frame (from mag_calib bootstrap)
// q_out: resulting sensor→world quaternion at powerup
// Returns 1 on success, 0 if vectors are degenerate.
int triad_compute(const float a_sensor[3], const float m_sensor[3],
                  const float Ag_world[3], const float Bg_world[3],
                  Quat *q_out);

#ifdef __cplusplus
}
#endif
