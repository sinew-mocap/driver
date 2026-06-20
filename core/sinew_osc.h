// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// driver/core — the neutral /sinew OSC contract (folded in; the driver emits it).  Pure,
// vendor-neutral: the quaternion/accel value types + the OSC message encoders.
// Every cluster that speaks /sinew links this; nothing here knows any dongle.
// Mirrors Sinew.Osc.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Neutral value types carried by the /sinew protocol.
#ifndef SINEW_QUAT_DEFINED
#define SINEW_QUAT_DEFINED
typedef struct {
	float w, x, y, z;
} Quat;
#endif

#ifndef SINEW_ACCEL_DEFINED
#define SINEW_ACCEL_DEFINED
typedef struct {
	float x, y, z;
} Accel;
#endif

// buildTracker → /sinew/tracker ,sffffd  (name, qw,qx,qy,qz float32, time_s float64)
size_t build_tracker(uint8_t *buf, const char *name, float qw, float qx, float qy, float qz,
                     double time_s);
// buildAccel → /sinew/accel ,sfff
size_t build_accel(uint8_t *buf, const char *name, float ax, float ay, float az);
// buildMag → /sinew/mag ,sfff  (magnetometer, raw LSB; only emitted when mag_valid)
size_t build_mag(uint8_t *buf, const char *name, float mx, float my, float mz);
// buildMagcal → /sinew/magcal ,sfff  (calibrated magnetic field vector)
size_t build_magcal(uint8_t *buf, const char *name, float mx, float my, float mz);
// buildCalibStatus → /sinew/calib ,sf  (joint name, progress 0.0–1.0)
size_t build_calib_status(uint8_t *buf, const char *name, float progress);
// buildStateStatus → /sinew/state ,si  (joint name, SinewJointStateKind int)
size_t build_state_status(uint8_t *buf, const char *name, int state);
// buildMagQual → /sinew/magqual ,sf  (joint name, |B_raw| in LSB — deviates near metal)
size_t build_magqual(uint8_t *buf, const char *name, float mag_magnitude);
// buildBattery → /sinew/battery ,sf  (joint name, 0.0-1.0 normalised level)
size_t build_battery(uint8_t *buf, const char *name, float level);

#ifdef __cplusplus
}
#endif
