// SPDX-License-Identifier: MIT
// Property tests over the rebocap codec (C), using RapidCheck (C++ QuickCheck).
// Mirrors the Lean Plausible properties on the same spec.  `lake exe`-style: the
// process exits nonzero if any property fails (CI gate).
#include <rapidcheck.h>

#include <cmath>
#include <cstdint>
#include <cstring>

extern "C" {
#include "sinew_protocol.h"
}

// Build a 36-byte 0x18 IMU frame with the given quaternion i16 components at the
// Type0x18 offsets (sync + pkt-kind 8; standard layout via byte 33 != 0).
static void buildFrame(uint8_t b[FRAME_BYTES], int16_t qw, int16_t qx, int16_t qy, int16_t qz) {
	memset(b, 0, FRAME_BYTES);
	b[0] = 0xFA;
	b[1] = 0xFA;
	b[2] = 0xAE;
	b[3] = 0xAE;
	b[4] = 0x18;  // lower nibble 8 = IMU layout
	auto put = [&](int off, int16_t v) {
		b[off] = (uint8_t)(v & 0xFF);
		b[off + 1] = (uint8_t)((v >> 8) & 0xFF);
	};
	put(10, qw);
	put(12, qx);
	put(14, qy);
	put(16, qz);
	b[33] = 0x01;  // standard layout
}

int main() {
	bool ok = true;

	// Round-trip: any 0x18 frame parses, and the decoded quaternion is unit-norm
	// (the parser normalises; degenerate inputs fall back to identity, also unit).
	ok &= rc::check("parse_frame yields a unit quaternion",
	                [](int16_t qw, int16_t qx, int16_t qy, int16_t qz) {
		                uint8_t b[FRAME_BYTES];
		                buildFrame(b, qw, qx, qy, qz);
		                ImuFrame f;
		                RC_ASSERT(parse_frame(b, &f) == SINEW_FRAME_OK);
		                float n = std::sqrt(f.quat.w * f.quat.w + f.quat.x * f.quat.x +
		                                    f.quat.y * f.quat.y + f.quat.z * f.quat.z);
		                RC_ASSERT(std::fabs(n - 1.0f) < 1e-3f);
	                });

	// /sinew/tracker OSC framing: 4-byte aligned and correctly addressed.
	ok &= rc::check("build_tracker output is 4-aligned and addressed",
	                [](float a, float b2, float c, float d) {
		                uint8_t buf[512];
		                size_t len = build_tracker(buf, "Hips", a, b2, c, d, 1.0);
		                RC_ASSERT(len % 4 == 0);
		                RC_ASSERT(memcmp(buf, "/sinew/tracker", 14) == 0);
	                });

	// All scalar OSC builders (/sinew/{calib,state,magqual,battery}) are 4-aligned
	// and begin with the correct address string.
	ok &= rc::check("build_calib_status is 4-aligned and addressed",
	                [](float p) {
		                uint8_t buf[256];
		                size_t len = build_calib_status(buf, "Hips", p);
		                RC_ASSERT(len % 4 == 0);
		                RC_ASSERT(memcmp(buf, "/sinew/calib", 12) == 0);
	                });

	ok &= rc::check("build_magqual is 4-aligned and addressed",
	                [](float m) {
		                uint8_t buf[256];
		                size_t len = build_magqual(buf, "Spine", m);
		                RC_ASSERT(len % 4 == 0);
		                RC_ASSERT(memcmp(buf, "/sinew/magqual", 14) == 0);
	                });

	ok &= rc::check("build_battery is 4-aligned and addressed",
	                [](float lv) {
		                uint8_t buf[256];
		                size_t len = build_battery(buf, "LeftFoot", lv);
		                RC_ASSERT(len % 4 == 0);
		                RC_ASSERT(memcmp(buf, "/sinew/battery", 14) == 0);
	                });

	// Accel magnitude after a round-trip through parse_frame stays in a physically
	// plausible range: the parser scales by ACCEL_1G so magnitudes outside
	// [0, 64] g are impossible for int16 raw inputs.
	ok &= rc::check("parsed accel magnitude is bounded",
	                [](int16_t ax, int16_t ay, int16_t az) {
		                uint8_t b[FRAME_BYTES];
		                memset(b, 0, FRAME_BYTES);
		                b[0] = 0xFA; b[1] = 0xFA; b[2] = 0xAE; b[3] = 0xAE;
		                b[4] = 0x18; b[33] = 0x01;
		                b[10] = 0x00; b[11] = 0x40;  // qw = 0x4000 (unit)
		                auto put = [&](int off, int16_t v) {
			                b[off]   = (uint8_t)(v & 0xFF);
			                b[off+1] = (uint8_t)((v >> 8) & 0xFF);
		                };
		                put(18, ax); put(20, ay); put(22, az);
		                ImuFrame f;
		                RC_ASSERT(parse_frame(b, &f) == SINEW_FRAME_OK);
		                float mag = std::sqrt(f.accel.x*f.accel.x +
		                                      f.accel.y*f.accel.y +
		                                      f.accel.z*f.accel.z);
		                // INT16_MAX / ACCEL_1G ≈ 32767/512 ≈ 64 g — safe upper bound.
		                RC_ASSERT(mag < 70.0f);
	                });

	return ok ? 0 : 1;
}
