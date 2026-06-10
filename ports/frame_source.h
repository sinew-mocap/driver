// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// FrameSource — a DRIVEN (secondary) port of the driver hexagon: a source of
// raw 36-byte dongle frames.  Adapters (driver cluster): the serial port
// (host/driver/serial_source); a recorded .rawlog in CI.  Keeps the core codec
// out of any transport.  (Declared now; wired load-bearing in the ports step.)
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FrameSource {
	void *ctx;
	// Fill `frame36` (FRAME_BYTES) with the next frame.
	// Returns 1 on a frame, 0 on clean end-of-stream, <0 on error/disconnect.
	int (*next)(void *ctx, uint8_t *frame36);
	void (*close)(void *ctx);
} FrameSource;

#ifdef __cplusplus
}
#endif
