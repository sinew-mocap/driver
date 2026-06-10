// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// rawlog_source — open a recorded .rawlog as a FrameSource (see rawlog_source.c).
#pragma once
#include "frame_source.h"

#ifdef __cplusplus
extern "C" {
#endif

// Open `path` as a FrameSource that replays its frames.  Returns 0 on success
// (fills *out), <0 if the file cannot be opened.  Free with out->close(out->ctx).
int rawlog_source_open(const char *path, FrameSource *out);

#ifdef __cplusplus
}
#endif
