// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Per-kit durable mapping: tracker hardware ID (3 bytes, lowercase hex)
// → Rebocap joint name.  Hwids are kit-specific identifiers — unique
// per Rebocap kit but stable for that kit across power cycles, OS
// reboots, and pairing-order changes.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	const char *hwid;   // 6-char lowercase hex string (e.g. "603f5e")
	const char *joint;  // Rebocap joint name (e.g. "Hips")
} SinewHwidEntry;

extern const SinewHwidEntry *SINEW_HWID_TABLE;
extern int SINEW_HWID_TABLE_SIZE;

// Load from hwid_table.ini. Returns entries loaded, 0 on failure.
int sinew_hwid_load(const char *p_path);

// Linear lookup; returns NULL if hwid not found.
const char *sinew_joint_for_hwid(const char *p_hwid);

#ifdef __cplusplus
}
#endif
