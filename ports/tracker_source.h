// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// TrackerSource — a DRIVEN (secondary) port of the driver hexagon: a source of
// inbound /sinew OSC packets (the monitor mirror on UDP 39540 that the TUI reads
// back to display live tracker state).  It is the read twin of TrackerSink (the
// /sinew OSC-out): the same /sinew bytes, opposite direction — and the same
// contract the server hexagon consumes.  Adapter (driver cluster): the UDP
// receive socket (host/driver/osc_receiver).  (Declared now; wired load-bearing
// in the ports step.)
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TrackerSource {
	void *ctx;
	// Fill `osc` (capacity `cap`) with the next inbound /sinew packet, set *len.
	// Returns 1 on a packet, 0 if none available this poll, <0 on error/close.
	int (*next)(void *ctx, uint8_t *osc, size_t cap, size_t *len);
	void (*close)(void *ctx);
} TrackerSource;

#ifdef __cplusplus
}
#endif
