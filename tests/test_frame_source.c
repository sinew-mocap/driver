// SPDX-License-Identifier: MIT
// Exercises the FrameSource port through the rawlog_source adapter: write a
// .rawlog, replay it, and check the bytes round-trip and end-of-stream reports
// cleanly.  Keeps the (otherwise hardware-only) FrameSource boundary covered in
// CI, and is the "recorded .rawlog in CI" the port header promises.
#include "rawlog_source.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
	const char *path = "test_frame_source.rawlog";

	uint8_t a[36], b[36];
	for (int i = 0; i < 36; i++) {
		a[i] = (uint8_t)i;
		b[i] = (uint8_t)(0xFF - i);
	}

	FILE *fp = fopen(path, "wb");
	assert(fp);
	fprintf(fp, "12345,");
	for (int i = 0; i < 36; i++) {
		fprintf(fp, "%02x", a[i]);  // lower-case hex
	}
	fputc('\n', fp);
	fprintf(fp, "12346,");
	for (int i = 0; i < 36; i++) {
		fprintf(fp, "%02X", b[i]);  // upper-case hex too
	}
	fputc('\n', fp);
	fclose(fp);

	FrameSource src;
	assert(rawlog_source_open(path, &src) == 0);

	uint8_t got[36];
	assert(src.next(src.ctx, got) == 1);
	assert(memcmp(got, a, 36) == 0);
	assert(src.next(src.ctx, got) == 1);
	assert(memcmp(got, b, 36) == 0);
	assert(src.next(src.ctx, got) == 0);  // clean end-of-stream
	src.close(src.ctx);

	// A missing file is a clean open failure, not a crash.
	FrameSource missing;
	assert(rawlog_source_open("does-not-exist.rawlog", &missing) < 0);

	remove(path);
	printf("test_frame_source: OK\n");
	return 0;
}
