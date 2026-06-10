// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// rawlog_source — a FrameSource adapter that replays a recorded .rawlog (the
// file the driver writes with `--raw`: one line per frame, `ms,hex72\n`, where
// hex72 is the 36 raw bytes hex-encoded).  This is the second FrameSource
// implementation the port promises ("a recorded .rawlog in CI"): it lets the
// frame path be driven deterministically off disk, with no serial hardware.
#include "frame_source.h"

#include <stdio.h>
#include <stdlib.h>

#define RAWLOG_FRAME_BYTES 36

typedef struct {
	FILE *fp;
	int own;  // 1 = close fp on close()
} RawlogCtx;

static int hexval(int c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

// FrameSource.next: 1 = a frame, 0 = clean end-of-stream, <0 = malformed/error.
static int rawlog_next(void *ctx, uint8_t *frame36) {
	RawlogCtx *r = (RawlogCtx *)ctx;
	// Skip past the "ms," timestamp prefix to the first hex digit on the line.
	int c = fgetc(r->fp);
	while (c != EOF && c != ',') {
		c = fgetc(r->fp);
	}
	if (c == EOF) {
		return 0;  // no more lines
	}
	for (int i = 0; i < RAWLOG_FRAME_BYTES; i++) {
		int hi = hexval(fgetc(r->fp));
		int lo = hexval(fgetc(r->fp));
		if (hi < 0 || lo < 0) {
			return -1;  // truncated / malformed line
		}
		frame36[i] = (uint8_t)((hi << 4) | lo);
	}
	// Drain the rest of the line (newline / any trailing chars).
	while ((c = fgetc(r->fp)) != EOF && c != '\n') {
	}
	return 1;
}

static void rawlog_close(void *ctx) {
	RawlogCtx *r = (RawlogCtx *)ctx;
	if (r) {
		if (r->own && r->fp) {
			fclose(r->fp);
		}
		free(r);
	}
}

// Build a FrameSource that replays `path`.  Returns 0 on success (fills *out),
// <0 if the file can't be opened.  Free with out->close(out->ctx).
int rawlog_source_open(const char *path, FrameSource *out) {
	FILE *fp = fopen(path, "rb");
	if (!fp) {
		return -1;
	}
	RawlogCtx *r = (RawlogCtx *)calloc(1, sizeof(RawlogCtx));
	if (!r) {
		fclose(fp);
		return -1;
	}
	r->fp = fp;
	r->own = 1;
	out->ctx = r;
	out->next = rawlog_next;
	out->close = rawlog_close;
	return 0;
}
