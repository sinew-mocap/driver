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
typedef struct {
	float w, x, y, z;
} Quat;
typedef struct {
	float x, y, z;
} Accel;

// buildTracker → /sinew/tracker ,sffffd  (name, qw,qx,qy,qz float32, time_s float64)
size_t build_tracker(uint8_t *buf, const char *name, float qw, float qx, float qy, float qz,
                     double time_s);
// buildAccel → /sinew/accel ,sfff
size_t build_accel(uint8_t *buf, const char *name, float ax, float ay, float az);

#ifdef __cplusplus
}
#endif
