// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Mechanical C translation of the Lean4 Sinew driver.
// Serial → parse → stream-registry → OSC UDP.
#pragma once
#include <stdint.h>
#include <stdio.h>

typedef struct {
	char serial_port[256];
	char dest_ip[64];
	uint16_t osc_port;      // primary OSC target (the consumer app) at dest_ip
	uint16_t monitor_port;  // 0 = off; else mirror every packet to 127.0.0.1:monitor_port
	int verbose;
	FILE *logfp;  // driver writes here; NULL = silent
	FILE *rawfp;  // every 36-byte frame as `ms,hex72`; NULL = off
	char mag_calib_path[256];  // INI for online hard+soft iron calibration; "" = disable
} SinewConfig;

void sinew_config_default(SinewConfig *p_cfg);

// Blocking loop — call from a dedicated thread.
void sinew_driver_run(const SinewConfig *p_cfg);

// Signal the loop to exit (async-signal-safe).
void sinew_driver_stop(void);

// ── Stage 3: per-joint state machine ─────────────────────────────────────
//
// Driver holds one state per Rebocap joint, updated whenever a frame
// resolves to a joint label via the hwid_table (Stage 1).  Reads race with
// the driver thread but each u64/enum is atomic on aligned 64-bit hosts.
// Mirrors tui/spec/Protocol/Lifecycle.lean :: State.
typedef enum {
	SINEW_JOINT_UNKNOWN = 0,  // unresolved
	SINEW_JOINT_ACTIVE,       // frames within last 2 s
	SINEW_JOINT_STALE,        // 2..5 s since last frame
	SINEW_JOINT_ASLEEP,       // >5 s since last frame
	SINEW_JOINT_REJOINING,    // a 0x?A burst with this hwid arrives after asleep,
	                          // awaiting first high-rate frame
} SinewJointStateKind;

#define SINEW_JOINT_NAME_LEN 32
#define SINEW_JOINT_COUNT 15

typedef struct {
	char joint[SINEW_JOINT_NAME_LEN];  // "Hips", "RightHand", ...
	int node_num;                      // 0..14 Rebocap canonical
	SinewJointStateKind state;
	uint64_t last_frame_ms;  // 0 = never seen
	uint64_t frame_count;
} SinewJointState;

// Snapshot all 15 joint states into the caller's array.  Returns count
// copied (always SINEW_JOINT_COUNT when max is large enough).  Safe to
// call from any thread; reads may race with the driver thread.
int sinew_get_joint_states(SinewJointState *p_out, int p_max);

// ── Host-write API ───────────────────────────────────────────────────────
//
// All functions are thread-safe (driver-internal lock).  Return 1 on
// success, 0 on failure (e.g. no active serial port, write rejected).
//
// `sinew_send_wake_up` sends `0x27 wake_up` to OMNI (0xC8).  Driver thread
// fires this once at port-open and every 30 s thereafter.  wake_up payload
// is `00 00 00 00 00 00 b4 00` + 14 zero bytes; the 0xb4 is the only
// non-zero byte.  Callers (e.g. the TUI shutdown flow) can also fire it
// manually.
//
// `sinew_send_shutdown` sends `0x07 shutdown_node` to a specific
// NodeNumber (0x00..0x0E) or `0xFF` for the ALL_TRACKERS broadcast.
// Per Spec.lean: this powers the tracker(s) off.  Recovering requires a
// physical button press.  No payload data — frame is `EA EA 07 00 C8
// <node> <msg LE> [22 zero bytes] AF AF`.
int sinew_send_wake_up(void);
int sinew_send_shutdown(uint8_t p_node);
#define SINEW_NODE_ALL_TRACKERS 0xFF

// `sinew_send_activate` sends `0x20 activate_node` to a specific
// NodeNumber 0x00..0x0E.  The 22-byte payload at frame bytes 12..33
// must be supplied by the caller; there is no known algorithm to
// compute it (automatic rejoin makes activate optional for already-paired
// trackers, but first-time pairing of a brand-new tracker may need a
// real payload — see Spec.lean).  The payload comes from capturing one
// rebocap session per kit with tools/host_capture and running
// tools/extract_activates, then loading the kit-specific table at startup.
//
// `sinew_load_activate_table` reads one entry per line in the format
//   <node-decimal>,<22-byte hex>
// e.g.   9,edc1d8e7f401e3ffa0258c6e7f5e00000000417a8b96
// Returns the number of entries loaded; 0 on file-missing or parse
// failure.  The loaded table is what `sinew_send_activate_known`
// looks up.
int sinew_send_activate(uint8_t p_node, const uint8_t p_payload22[22]);
int sinew_send_activate_known(uint8_t p_node);
int sinew_load_activate_table(const char *p_path);
int sinew_have_activate_for(uint8_t p_node);

// set_anti_magnetic_strength (0x31).  `sinew_load_mag_table` parses the [mag]
// section (same file/format as the activate table) into per-node 22-byte
// payloads that carry each tracker's magnetometer calibration.
// `sinew_send_mag_strength` replays that calibration and overwrites only the
// level byte (payload offset 18, encoded inverted as 14 - level for 1..12), so
// changing strength never clobbers calibration.  Sending is refused for a node
// with no captured payload.  See docs/host-commands.md.
int sinew_send_mag_strength(uint8_t p_node, int p_level);
int sinew_load_mag_table(const char *p_path);
int sinew_have_mag_for(uint8_t p_node);

// Generic host->dongle command for config opcodes (set_anti_magnetic_strength
// 0x31, set_transmit_power 0x91, set_rgb 0x41, ...).  target: 0x00..0x0E node,
// 0xC8 OMNI, 0xFF ALL_TRACKERS.  work_bit sets the 0x8000 "tracker must
// process" flag.  payload (<=22 B) at frame bytes 12..33.
int sinew_send_command(uint8_t p_opcode, uint8_t p_target, int p_work_bit, const uint8_t *p_payload,
                       int p_payload_len);
