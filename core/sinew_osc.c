// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// osc/core — the /sinew OSC encoders (encode only; no socket).  Mirrors Sinew.Osc.
#include "sinew_osc.h"

#include <string.h>

static size_t pad4(size_t n) {
	return (n + 3u) & ~3u;
}

static size_t osc_string(uint8_t *buf, size_t off, const char *s) {
	size_t slen = strlen(s) + 1;
	size_t padded = pad4(slen);
	memcpy(buf + off, s, slen);
	if (padded > slen) {
		memset(buf + off + slen, 0, padded - slen);
	}
	return off + padded;
}

static size_t osc_float(uint8_t *buf, size_t off, float f) {
	uint32_t bits;
	memcpy(&bits, &f, 4);
	buf[off + 0] = (uint8_t)(bits >> 24);
	buf[off + 1] = (uint8_t)(bits >> 16);
	buf[off + 2] = (uint8_t)(bits >> 8);
	buf[off + 3] = (uint8_t)(bits >> 0);
	return off + 4;
}

static size_t osc_double(uint8_t *buf, size_t off, double d) {
	uint64_t bits;
	memcpy(&bits, &d, 8);
	buf[off + 0] = (uint8_t)(bits >> 56);
	buf[off + 1] = (uint8_t)(bits >> 48);
	buf[off + 2] = (uint8_t)(bits >> 40);
	buf[off + 3] = (uint8_t)(bits >> 32);
	buf[off + 4] = (uint8_t)(bits >> 24);
	buf[off + 5] = (uint8_t)(bits >> 16);
	buf[off + 6] = (uint8_t)(bits >> 8);
	buf[off + 7] = (uint8_t)(bits >> 0);
	return off + 8;
}

size_t build_tracker(uint8_t *buf, const char *name, float qw, float qx, float qy, float qz,
                     double time_s) {
	size_t o = osc_string(buf, 0, "/sinew/tracker");
	o = osc_string(buf, o, ",sffffd");
	o = osc_string(buf, o, name);
	o = osc_float(buf, o, qw);
	o = osc_float(buf, o, qx);
	o = osc_float(buf, o, qy);
	o = osc_float(buf, o, qz);
	o = osc_double(buf, o, time_s);
	return o;
}

size_t build_accel(uint8_t *buf, const char *name, float ax, float ay, float az) {
	size_t o = osc_string(buf, 0, "/sinew/accel");
	o = osc_string(buf, o, ",sfff");
	o = osc_string(buf, o, name);
	o = osc_float(buf, o, ax);
	o = osc_float(buf, o, ay);
	o = osc_float(buf, o, az);
	return o;
}

size_t build_mag(uint8_t *buf, const char *name, float mx, float my, float mz) {
	size_t o = osc_string(buf, 0, "/sinew/mag");
	o = osc_string(buf, o, ",sfff");
	o = osc_string(buf, o, name);
	o = osc_float(buf, o, mx);
	o = osc_float(buf, o, my);
	o = osc_float(buf, o, mz);
	return o;
}

size_t build_calib_status(uint8_t *buf, const char *name, float progress) {
	size_t o = osc_string(buf, 0, "/sinew/calib");
	o = osc_string(buf, o, ",sf");
	o = osc_string(buf, o, name);
	o = osc_float(buf, o, progress);
	return o;
}

size_t build_magqual(uint8_t *buf, const char *name, float mag_magnitude) {
	size_t o = osc_string(buf, 0, "/sinew/magqual");
	o = osc_string(buf, o, ",sf");
	o = osc_string(buf, o, name);
	return osc_float(buf, o, mag_magnitude);
}

size_t build_battery(uint8_t *buf, const char *name, float level) {
	size_t o = osc_string(buf, 0, "/sinew/battery");
	o = osc_string(buf, o, ",sf");
	o = osc_string(buf, o, name);
	return osc_float(buf, o, level);
}

size_t build_state_status(uint8_t *buf, const char *name, int state) {
	size_t o = osc_string(buf, 0, "/sinew/state");
	o = osc_string(buf, o, ",si");
	o = osc_string(buf, o, name);
	uint32_t v = (uint32_t)state;
	buf[o+0]=(uint8_t)(v>>24); buf[o+1]=(uint8_t)(v>>16);
	buf[o+2]=(uint8_t)(v>>8);  buf[o+3]=(uint8_t)(v>>0);
	return o + 4;
}

size_t build_magcal(uint8_t *buf, const char *name, float mx, float my, float mz) {
	size_t o = osc_string(buf, 0, "/sinew/magcal");
	o = osc_string(buf, o, ",sfff");
	o = osc_string(buf, o, name);
	o = osc_float(buf, o, mx);
	o = osc_float(buf, o, my);
	o = osc_float(buf, o, mz);
	return o;
}
