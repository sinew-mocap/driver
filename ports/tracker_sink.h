// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// TrackerSink — a DRIVEN (secondary) port of the driver hexagon: a sink for
// encoded /sinew OSC packets (already built by build_tracker/build_accel).
// Adapter (driver cluster): the UDP socket (host/driver/osc_sink).  The same
// /sinew bytes are what the server hexagon consumes through its TrackerSource.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TrackerSink {
	void *ctx;
	void (*send)(void *ctx, const uint8_t *osc, size_t len);
	void (*close)(void *ctx);
} TrackerSink;

#ifdef __cplusplus
}
#endif
