// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// rebocap frame codec (engine-free, NO I/O): decodes the 36-byte dongle frame
// into an ImuFrame.  Rebocap-specific — lives in the driver cluster.  The neutral
// /sinew OSC contract (Quat/Accel + build_tracker/build_accel) comes from the osc
// cluster.  Mirrors Sinew.Protocol.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "sinew_osc.h"  // neutral contract: Quat, Accel, build_tracker/build_accel

// ── Sinew.Protocol.Spec — wire constants ─────────────────────────────────────
#define FRAME_BYTES 36
#define SYNC_0 ((uint8_t)0xFA)
#define SYNC_1 ((uint8_t)0xFA)
#define SYNC_2 ((uint8_t)0xAE)
#define SYNC_3 ((uint8_t)0xAE)
#define ACCEL_1G 16384.0f

// pkt_type lower nibble selects the frame kind: 8,9,D = IMU layout; A = burst.
#define PKT_KIND_IMU8 ((uint8_t)0x8)
#define PKT_KIND_IMU9 ((uint8_t)0x9)
#define PKT_KIND_IMUD ((uint8_t)0xD)
#define PKT_KIND_BURST ((uint8_t)0xA)

#ifdef __cplusplus
extern "C" {
#endif

// Quat / Accel come from the osc contract (sinew_osc.h).

// ImuFrame (= Sinew.Protocol.ImuFrame)
typedef struct {
	uint8_t pkt_type;
	uint8_t sub_type;
	uint16_t seq_num;
	uint16_t status;
	uint8_t tracker_idx;
	char hub_id[16];
	Quat quat;
	Accel accel;
	Accel mag;      // magnetometer (bytes 24-29); only valid in standard layout
	int mag_valid;  // 1 = standard layout (byte 33 != 0), mag is clean; 0 = skip
	// Burst-frame trailing bytes 27-29 (0x3a "unvalidated field"):
	// likely compass quality / battery level — expose raw for user interpretation.
	uint8_t raw27;   // byte 27
	uint8_t raw28;   // byte 28 (commonly battery level 0-100 or 0-255)
	uint8_t raw29;   // byte 29
} ImuFrame;

// parse_frame result (= Sinew.Protocol.ParseResult, the relevant cases).
enum {
	SINEW_FRAME_SKIP = 0,     // bad sync or unsupported pkt type
	SINEW_FRAME_OK = 1,       // *out filled with a decoded frame
	SINEW_FRAME_OFFLINE = 2,  // dongle "tracker offline" filler; *out->hub_id/tracker_idx set
};

// Decode one 36-byte frame.  Pure: on OFFLINE it fills out->hub_id + tracker_idx
// (so the host can map the slot to a joint) but takes no action itself.
int parse_frame(const uint8_t *b, ImuFrame *out);

#ifdef __cplusplus
}
#endif
