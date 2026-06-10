// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// rebocap frame codec.  Maps 1-to-1 with Sinew.Protocol.{Spec, Type0x18,
// Type0x3a, Protocol}.  Engine-free: no serial, no socket, no host state.  The
// neutral /sinew OSC encoders live in the osc cluster (sinew_osc.c).
#include "sinew_protocol.h"

#include <math.h>
#include <string.h>

// ── Byte helpers (= Protocol.getU8 / getI16LE / getU16LE) ────────────────────

static inline uint8_t get_u8(const uint8_t *b, int i) {
	return b[i];
}
static inline uint16_t get_u16le(const uint8_t *b, int i) {
	return (uint16_t)b[i] | ((uint16_t)b[i + 1] << 8);
}
static inline int get_i16le(const uint8_t *b, int i) {
	uint16_t raw = (uint16_t)b[i] | ((uint16_t)b[i + 1] << 8);
	return (int)(int16_t)raw;
}

// ── normaliseQuat (= Protocol.normaliseQuat) ─────────────────────────────────

static Quat normalise_quat(int rw, int rx, int ry, int rz) {
	float fw = (float)rw, fx = (float)rx, fy = (float)ry, fz = (float)rz;
	float mag = sqrtf(fw * fw + fx * fx + fy * fy + fz * fz);
	if (mag < 1.0f) {
		return (Quat){1.f, 0.f, 0.f, 0.f};
	}
	return (Quat){fw / mag, fx / mag, fy / mag, fz / mag};
}

// ── rawToAccel ───────────────────────────────────────────────────────────────

static Accel raw_to_accel(int rx, int ry, int rz) {
	return (Accel){rx / ACCEL_1G, ry / ACCEL_1G, rz / ACCEL_1G};
}

// ── extractHubAndIdx (= Protocol.extractHubAndIdx) ───────────────────────────
// Standard layout: hub bytes 30-32, idx byte 33.
// Shifted layout:  hub bytes 29-31, idx byte 32, byte 33 = 0x00.

static char nibble(uint8_t n) {
	return n < 10 ? (char)('0' + n) : (char)('a' + n - 10);
}
static void byte_to_hex2(uint8_t b, char *out) {
	out[0] = nibble(b >> 4);
	out[1] = nibble(b & 0xF);
}

// Append one byte as ASCII char (if printable) or two hex digits.
static int byte_to_str(uint8_t b, char *out, int pos) {
	if (b >= 0x20 && b <= 0x7E) {
		out[pos++] = (char)b;
	} else {
		byte_to_hex2(b, out + pos);
		pos += 2;
	}
	return pos;
}

static void extract_hub_and_idx(const uint8_t *b, char *hub, uint8_t *idx) {
	int off, len = 0;
	if (get_u8(b, 33) == 0) {
		off = 29;
		*idx = get_u8(b, 32);
	} else {
		off = 30;
		*idx = get_u8(b, 33);
	}
	len = byte_to_str(b[off], hub, len);
	len = byte_to_str(b[off + 1], hub, len);
	len = byte_to_str(b[off + 2], hub, len);
	hub[len] = '\0';
}

// ── parseFrame (= Protocol.parseFrame) ───────────────────────────────────────
// Synthetic "tracker offline" filler emitted by the dongle when a
// previously-paired tracker is asleep or powered off.  Signature: bytes 6..13
// are a fixed magic block (seq=0xc88e, status=0xa376, then constants
// 65 00 28 50).  b[6]=0xC8 is the OMNI broadcast address — no NodeNumber.
static int is_offline_placeholder(const uint8_t *b) {
	return b[6] == 0xc8 && b[7] == 0x8e && b[8] == 0x76 && b[9] == 0xa3 && b[10] == 0x65 &&
	       b[11] == 0x00 && b[12] == 0x28 && b[13] == 0x50;
}

int parse_frame(const uint8_t *b, ImuFrame *out) {
	if (b[0] != SYNC_0 || b[1] != SYNC_1 || b[2] != SYNC_2 || b[3] != SYNC_3) {
		return SINEW_FRAME_SKIP;
	}

	if (is_offline_placeholder(b)) {
		// Fill the slot identity so the host can map it to a joint and post the
		// lifecycle event; the codec itself takes no action.
		extract_hub_and_idx(b, out->hub_id, &out->tracker_idx);
		return SINEW_FRAME_OFFLINE;
	}

	uint8_t pkt = b[4];
	uint8_t lo = pkt & 0x0F;

	if (lo == PKT_KIND_IMU8 || lo == PKT_KIND_IMU9 || lo == PKT_KIND_IMUD) {
		char hub[16];
		uint8_t idx;
		extract_hub_and_idx(b, hub, &idx);
		out->pkt_type = pkt;
		out->sub_type = b[5];
		out->seq_num = get_u16le(b, 6);
		out->status = get_u16le(b, 8);
		out->tracker_idx = idx;
		strncpy(out->hub_id, hub, sizeof(out->hub_id) - 1);
		out->hub_id[sizeof(out->hub_id) - 1] = '\0';
		out->quat =
		    normalise_quat(get_i16le(b, 10), get_i16le(b, 12), get_i16le(b, 14), get_i16le(b, 16));
		out->accel = raw_to_accel(get_i16le(b, 18), get_i16le(b, 20), get_i16le(b, 22));
		// Magnetometer (bytes 24-29, 3x i16le) is clean ONLY in the standard
		// layout; the shifted layout (byte 33 == 0) puts the hub ASCII at 29-31
		// and clobbers mag-Z.  See spec/Sinew/Protocol/Type0x18.lean.
		out->mag_valid = (get_u8(b, 33) != 0);
		out->mag =
		    (Accel){(float)get_i16le(b, 24), (float)get_i16le(b, 26), (float)get_i16le(b, 28)};
		return SINEW_FRAME_OK;
	}

	if (lo == PKT_KIND_BURST) {
		// bytes 24-26 = stable hardware ID (bytes 30-32 are session-scoped, ignored)
		char hub[16];
		byte_to_hex2(b[24], hub + 0);
		byte_to_hex2(b[25], hub + 2);
		byte_to_hex2(b[26], hub + 4);
		hub[6] = '\0';
		out->pkt_type = pkt;
		out->sub_type = b[5];
		out->seq_num = get_u16le(b, 6);
		out->status = get_u16le(b, 8);
		out->tracker_idx = b[33];  // RF slot address
		strncpy(out->hub_id, hub, sizeof(out->hub_id) - 1);
		out->hub_id[sizeof(out->hub_id) - 1] = '\0';
		out->quat =
		    normalise_quat(get_i16le(b, 10), get_i16le(b, 12), get_i16le(b, 14), get_i16le(b, 16));
		out->accel = raw_to_accel(get_i16le(b, 18), get_i16le(b, 20), get_i16le(b, 22));
		out->mag_valid = 0;  // burst frames carry hwid at 24-26, no magnetometer
		out->mag = (Accel){0.f, 0.f, 0.f};
		return SINEW_FRAME_OK;
	}

	return SINEW_FRAME_SKIP;  // unsupported type
}
