// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// CommandSink — a DRIVEN (secondary) port of the driver hexagon: a sink for
// host->dongle command frames (the 36-byte wake_up / activate / shutdown /
// config opcodes the host writes back out to the rebocap dongle).  It is the
// write twin of FrameSource on the same serial boundary, so the port turns the
// serial link into a symmetric source(FrameSource)/sink(CommandSink) pair
// instead of a read-only source.  Adapter (driver cluster): the serial write
// side (host/driver/serial).  (Declared now; wired load-bearing in the ports step.)
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CommandSink {
	void *ctx;
	// Write one already-built 36-byte command frame to the dongle.
	// Returns 1 on success, 0 on failure (no active port / write rejected).
	int (*send)(void *ctx, const uint8_t *frame36);
	void (*close)(void *ctx);
} CommandSink;

#ifdef __cplusplus
}
#endif
