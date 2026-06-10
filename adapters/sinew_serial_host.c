// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// host/driver — the slim rebocap driver's serial + UDP adapters around the pure
// core/protocol codec.  Reads 36-byte frames off the serial port, decodes them
// with parse_frame, and emits the /sinew OSC protocol over UDP.  No calibration,
// no fusion, no GPU: the dongle's device quaternion passes through as-is, and the
// server does the TIC calibration + body solve.  Maps to Main (SeqRegistry,
// runDriver) + the host-write commands + the lifecycle FSM (Lifecycle.lean).

#include "sinew_driver.h"
#include "hwid_table.h"
#include "sinew_protocol.h"  // the wire codec: ImuFrame, parse_frame, build_tracker/accel
#include "tracker_sink.h"    // the driven port the OSC-out adapter implements
#include "frame_source.h"    // the driven port the serial read side implements

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <setupapi.h>
typedef HANDLE serial_handle_t;
typedef SOCKET udp_sock_t;
#define SERIAL_INVALID INVALID_HANDLE_VALUE
#define UDP_INVALID INVALID_SOCKET
#define UDP_CLOSE(s) closesocket(s)
static void sleep_ms(unsigned ms) {
	Sleep(ms);
}
static uint64_t now_ms(void) {
	FILETIME ft;
	GetSystemTimeAsFileTime(&ft);
	uint64_t t = ((uint64_t)ft.dwHighDateTime << 32) | ft.dwLowDateTime;
	return (t - 116444736000000000ULL) / 10000;
}
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
typedef int serial_handle_t;
typedef int udp_sock_t;
#define SERIAL_INVALID (-1)
#define UDP_INVALID (-1)
#define UDP_CLOSE(s) close(s)
static void sleep_ms(unsigned ms) {
	usleep(ms * 1000u);
}
static uint64_t now_ms(void) {
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}
#endif

// ── Threading ────────────────────────────────────────────────────────────
// POSIX uses pthread directly.  Windows maps the pthread_* spellings used below
// onto Win32 SRWLOCK + CONDITION_VARIABLE (both static-initialisable, like the
// pthread initialisers) and CreateThread, so the driver builds with MinGW *and*
// MSVC without winpthreads.  (WLOCK uses a Win32 CRITICAL_SECTION already.)
#ifdef _WIN32
typedef SRWLOCK pthread_mutex_t;
typedef CONDITION_VARIABLE pthread_cond_t;
typedef HANDLE pthread_t;
#define PTHREAD_MUTEX_INITIALIZER SRWLOCK_INIT
#define PTHREAD_COND_INITIALIZER CONDITION_VARIABLE_INIT
#define pthread_mutex_lock(m) AcquireSRWLockExclusive(m)
#define pthread_mutex_unlock(m) ReleaseSRWLockExclusive(m)
#define pthread_cond_wait(c, m) SleepConditionVariableSRW((c), (m), INFINITE, 0)
#define pthread_cond_signal(c) WakeConditionVariable(c)
#define pthread_create(tp, attr, fn, arg) (*(tp) = CreateThread(NULL, 0, (fn), (arg), 0, NULL))
#define pthread_join(t, ret) (WaitForSingleObject((t), INFINITE), CloseHandle(t))
#define SINEW_THRD_RET DWORD WINAPI
#else
#define SINEW_THRD_RET void *
#endif


// Lifecycle event kinds (= Lifecycle.lean events) — fed to the per-joint FSM.
typedef enum { LC_HIGH_RATE, LC_BURST_HWID, LC_OFFLINE } LcEventKind;

static volatile int g_stop = 0;  // set by sinew_driver_stop()

// ── Stage 3: per-joint lifecycle state machine ───────────────────────────
//
// Architecture: two producers feed a bounded event queue; one consumer
// (the state machine thread) drains it.
//
//   Frame reader thread  ──► [ lifecycle queue ] ──► state machine thread
//   Aging thread (100ms) ──►                    ──►   (updates g_joint_state)
//
// The pure step() function mirrors Lifecycle.lean :: step exactly.
// No polling; the state machine blocks on a condvar until an event arrives.
// The aging thread uses nanosleep — no select/poll timeouts anywhere.

static const char *SINEW_JOINT_NODE_TABLE[SINEW_JOINT_COUNT] = {
    "Hips",          "LeftUpperLeg", "RightUpperLeg", "LeftLowerLeg", "RightLowerLeg",
    "LeftFoot",      "RightFoot",    "Chest",         "Head",         "LeftUpperArm",
    "RightUpperArm", "LeftLowerArm", "RightLowerArm", "LeftHand",     "RightHand"};

static int joint_index_for_name(const char *name) {
	if (!name) {
		return -1;
	}
	for (int i = 0; i < SINEW_JOINT_COUNT; i++) {
		if (strcmp(name, SINEW_JOINT_NODE_TABLE[i]) == 0) {
			return i;
		}
	}
	return -1;
}

// ── Pure step function (= Lifecycle.lean :: step) ────────────────────────

// LC_HIGH_RATE, LC_BURST_HWID, LC_OFFLINE — forward-declared above parse_frame.

// Pure event-driven step — no timing constants.
// Transitions are driven entirely by what the dongle reports:
//   high-rate frame  → tracker is active
//   burst frame      → tracker is waking (promote from asleep)
//   offline frame    → dongle reports this slot is empty
static SinewJointStateKind lc_step(SinewJointStateKind s, LcEventKind ev) {
	switch (ev) {
		case LC_HIGH_RATE:
			return SINEW_JOINT_ACTIVE;
		case LC_BURST_HWID:
			if (s == SINEW_JOINT_ASLEEP || s == SINEW_JOINT_STALE) {
				return SINEW_JOINT_REJOINING;
			}
			return s;
		case LC_OFFLINE:
			return SINEW_JOINT_ASLEEP;
	}
	return s;
}

// ── Lifecycle event queue ─────────────────────────────────────────────────

typedef struct {
	LcEventKind kind;
	int joint_idx;  // index into SINEW_JOINT_NODE_TABLE
	uint64_t now_ms;
} LcEvent;

#define LC_QUEUE_CAP 512
static LcEvent g_lc_queue[LC_QUEUE_CAP];
static int g_lc_head = 0;
static int g_lc_tail = 0;
static pthread_mutex_t g_lc_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_lc_cv = PTHREAD_COND_INITIALIZER;

static void lc_enqueue(LcEvent ev) {
	pthread_mutex_lock(&g_lc_mu);
	int next = (g_lc_head + 1) % LC_QUEUE_CAP;
	if (next != g_lc_tail) {  // drop if full (shouldn't happen at sensor rate)
		g_lc_queue[g_lc_head] = ev;
		g_lc_head = next;
		pthread_cond_signal(&g_lc_cv);
	}
	pthread_mutex_unlock(&g_lc_mu);
}

// ── Joint state storage (written by SM thread, read by TUI) ──────────────

static SinewJointState g_joint_state[SINEW_JOINT_COUNT];
static pthread_mutex_t g_js_mu = PTHREAD_MUTEX_INITIALIZER;

static void joint_state_init(void) {
	pthread_mutex_lock(&g_js_mu);
	for (int i = 0; i < SINEW_JOINT_COUNT; i++) {
		memset(&g_joint_state[i], 0, sizeof(g_joint_state[i]));
		g_joint_state[i].node_num = i;
		snprintf(g_joint_state[i].joint, SINEW_JOINT_NAME_LEN, "%s", SINEW_JOINT_NODE_TABLE[i]);
		g_joint_state[i].state = SINEW_JOINT_UNKNOWN;
	}
	pthread_mutex_unlock(&g_js_mu);
}

// ── State machine thread ──────────────────────────────────────────────────

static void lc_apply(const LcEvent *ev) {
	int i = ev->joint_idx;
	if (i < 0 || i >= SINEW_JOINT_COUNT) {
		return;
	}
	pthread_mutex_lock(&g_js_mu);
	SinewJointState *s = &g_joint_state[i];
	s->state = lc_step(s->state, ev->kind);
	s->last_frame_ms = ev->now_ms;
	s->frame_count += 1;
	pthread_mutex_unlock(&g_js_mu);
}

static SINEW_THRD_RET lc_sm_thread(void *arg) {
	(void)arg;
	for (;;) {
		pthread_mutex_lock(&g_lc_mu);
		while (g_lc_head == g_lc_tail && !g_stop) {
			pthread_cond_wait(&g_lc_cv, &g_lc_mu);
		}
		// Drain all pending events under the queue lock, then release
		while (g_lc_head != g_lc_tail) {
			LcEvent ev = g_lc_queue[g_lc_tail];
			g_lc_tail = (g_lc_tail + 1) % LC_QUEUE_CAP;
			pthread_mutex_unlock(&g_lc_mu);
			lc_apply(&ev);
			pthread_mutex_lock(&g_lc_mu);
		}
		pthread_mutex_unlock(&g_lc_mu);
		if (g_stop) {
			break;
		}
	}
	return 0;
}

// ── Frame event helper ────────────────────────────────────────────────────

static void lc_post_frame(const char *joint_name, uint64_t now, LcEventKind kind) {
	int idx = joint_index_for_name(joint_name);
	if (idx < 0) {
		return;
	}
	lc_enqueue((LcEvent){kind, idx, now});
}

int sinew_get_joint_states(SinewJointState *p_out, int p_max) {
	int n = (p_max < SINEW_JOINT_COUNT) ? p_max : SINEW_JOINT_COUNT;
	pthread_mutex_lock(&g_js_mu);
	for (int i = 0; i < n; i++) {
		p_out[i] = g_joint_state[i];
	}
	pthread_mutex_unlock(&g_js_mu);
	return n;
}

// ── Host writes ──────────────────────────────────────────────────────────
//
// One serial handle, one global counter, one mutex.  Writes from the
// driver thread (heartbeat) and from caller threads (TUI shutdown) all
// funnel through send_locked.

#ifdef _WIN32
static CRITICAL_SECTION g_write_cs;
static int g_write_cs_inited = 0;
#define WLOCK()                                     \
	do {                                            \
		if (!g_write_cs_inited) {                   \
			InitializeCriticalSection(&g_write_cs); \
			g_write_cs_inited = 1;                  \
		}                                           \
		EnterCriticalSection(&g_write_cs);          \
	} while (0)
#define WUNLOCK() LeaveCriticalSection(&g_write_cs)
#else
#include <pthread.h>
static pthread_mutex_t g_write_lock = PTHREAD_MUTEX_INITIALIZER;
#define WLOCK() pthread_mutex_lock(&g_write_lock)
#define WUNLOCK() pthread_mutex_unlock(&g_write_lock)
#endif

static volatile serial_handle_t g_active_fd = SERIAL_INVALID;
static uint16_t g_msg_counter = 0;   // protected by WLOCK
static uint64_t g_last_wake_ms = 0;  // protected by WLOCK

static int serial_write_n(serial_handle_t fd, const uint8_t *buf, size_t n) {
#ifdef _WIN32
	DWORD wrote = 0;
	if (!WriteFile(fd, buf, (DWORD)n, &wrote, NULL)) {
		return 0;
	}
	return wrote == (DWORD)n;
#else
	size_t off = 0;
	while (off < n) {
		ssize_t r = write(fd, buf + off, n - off);
		if (r <= 0) {
			return 0;
		}
		off += (size_t)r;
	}
	return 1;
#endif
}

// Increment counter mod 0x7FFF (avoid 0).  Caller must hold WLOCK.
static uint16_t next_msg_locked(void) {
	g_msg_counter++;
	if ((g_msg_counter & 0x7FFF) == 0) {
		g_msg_counter = 1;
	}
	return g_msg_counter & 0x7FFF;
}

int sinew_send_wake_up(void) {
	WLOCK();
	serial_handle_t fd = g_active_fd;
	if (fd == SERIAL_INVALID) {
		WUNLOCK();
		return 0;
	}

	uint16_t msg = next_msg_locked();
	uint8_t frame[36] = {0};
	frame[0] = 0xEA;
	frame[1] = 0xEA;
	frame[2] = 0x27;  // wake_up
	frame[3] = 0x00;
	frame[4] = 0xC8;
	frame[5] = 0xC8;  // OMNI
	frame[6] = (uint8_t)(msg & 0xFF);
	frame[7] = (uint8_t)((msg >> 8) & 0x7F);  // bit 0x8000 cleared
	frame[14] = 0xB4;                         // magic byte in every wake_up
	frame[34] = 0xAF;
	frame[35] = 0xAF;
	int ok = serial_write_n(fd, frame, sizeof(frame));
	if (ok) {
		g_last_wake_ms = now_ms();
	}
	WUNLOCK();
	return ok;
}

// Per-joint activate_node payload table.  Populated by
// sinew_load_activate_table (one row per joint, parsed from a
// kit-specific text file).  Indexed by NodeNumber 0..14.

static uint8_t g_activate_payload[15][22];
static uint8_t g_activate_present[15];

int sinew_send_activate(uint8_t p_node, const uint8_t p_payload22[22]) {
	if (p_node > 0x0E) {
		return 0;
	}
	WLOCK();
	serial_handle_t fd = g_active_fd;
	if (fd == SERIAL_INVALID) {
		WUNLOCK();
		return 0;
	}

	uint16_t msg = next_msg_locked();
	uint8_t frame[36] = {0};
	frame[0] = 0xEA;
	frame[1] = 0xEA;
	frame[2] = 0x20;  // activate_node
	frame[3] = 0x00;
	frame[4] = 0xC8;
	frame[5] = p_node;
	frame[6] = (uint8_t)(msg & 0xFF);
	frame[7] = (uint8_t)((msg >> 8) | 0x80);  // activate sets 0x8000
	memcpy(frame + 12, p_payload22, 22);
	frame[34] = 0xAF;
	frame[35] = 0xAF;
	int ok = serial_write_n(fd, frame, sizeof(frame));
	WUNLOCK();
	return ok;
}

int sinew_send_activate_known(uint8_t p_node) {
	if (p_node > 0x0E) {
		return 0;
	}
	if (!g_activate_present[p_node]) {
		return 0;
	}
	return sinew_send_activate(p_node, g_activate_payload[p_node]);
}

int sinew_have_activate_for(uint8_t p_node) {
	return (p_node <= 0x0E) && g_activate_present[p_node];
}

static int hex_nibble(char c) {
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return 10 + (c - 'a');
	}
	if (c >= 'A' && c <= 'F') {
		return 10 + (c - 'A');
	}
	return -1;
}

// Shared INI parser for per-node 22-byte payload tables.  Reads rows
// "node = 44hexchars [; comment]" inside the [p_section] section into
// p_payloads/p_present (indexed by NodeNumber 0..14).  Returns the count.
static int load_payload_table(const char *p_path, const char *p_section, uint8_t p_payloads[15][22],
                              uint8_t p_present[15]) {
	if (!p_path || !*p_path) {
		return 0;
	}
	FILE *f = fopen(p_path, "r");
	if (!f) {
		return 0;
	}
	memset(p_present, 0, 15);
	char line[256];
	int loaded = 0;
	int in_section = 0;
	while (fgets(line, sizeof(line), f)) {
		char *p = line;
		while (*p == ' ' || *p == '\t') {
			p++;
		}
		if (!*p || *p == '\n' || *p == ';' || *p == '#') {
			continue;
		}
		// INI section header
		if (*p == '[') {
			char *end = strchr(p, ']');
			if (end) {
				*end = '\0';
			}
			in_section = (strcmp(p + 1, p_section) == 0);
			continue;
		}
		if (!in_section) {
			continue;
		}
		// Parse: node = 44hexchars [; comment]
		char *eq = strchr(p, '=');
		if (!eq) {
			continue;
		}
		*eq = '\0';
		int node = -1;
		if (sscanf(p, "%d", &node) != 1) {
			continue;
		}
		if (node < 0 || node > 14) {
			continue;
		}
		char *hex = eq + 1;
		while (*hex == ' ' || *hex == '\t') {
			hex++;
		}
		uint8_t buf[22];
		int valid = 1;
		for (int i = 0; i < 22 && valid; i++) {
			int hi = hex_nibble(hex[i * 2]);
			int lo = hex_nibble(hex[i * 2 + 1]);
			if (hi < 0 || lo < 0) {
				valid = 0;
				break;
			}
			buf[i] = (uint8_t)((hi << 4) | lo);
		}
		if (!valid) {
			continue;
		}
		memcpy(p_payloads[node], buf, 22);
		p_present[node] = 1;
		loaded++;
	}
	fclose(f);
	return loaded;
}

int sinew_load_activate_table(const char *p_path) {
	return load_payload_table(p_path, "activates", g_activate_payload, g_activate_present);
}

// Per-joint set_anti_magnetic_strength (0x31) payload table.  Each 22-byte
// payload carries that node's magnetometer calibration; the strength level
// lives at payload byte 18, encoded inverted as (14 - level) for level 1..12
// (see docs/host-commands.md).  The captured calibration is replayed and only
// the level byte is overwritten, so a strength change never clobbers a
// tracker's mag cal.
static uint8_t g_mag_payload[15][22];
static uint8_t g_mag_present[15];

int sinew_send_mag_strength(uint8_t p_node, int p_level) {
	if (p_node > 0x0E) {
		return 0;
	}
	if (!g_mag_present[p_node]) {
		return 0;  // no calibration → refuse rather than zero it out
	}
	if (p_level < 1) {
		p_level = 1;
	}
	if (p_level > 12) {
		p_level = 12;
	}
	WLOCK();
	serial_handle_t fd = g_active_fd;
	if (fd == SERIAL_INVALID) {
		WUNLOCK();
		return 0;
	}

	uint16_t msg = next_msg_locked();
	uint8_t frame[36] = {0};
	frame[0] = 0xEA;
	frame[1] = 0xEA;
	frame[2] = 0x31;  // set_anti_magnetic_strength
	frame[3] = 0x00;
	frame[4] = 0xC8;
	frame[5] = p_node;
	frame[6] = (uint8_t)(msg & 0xFF);
	frame[7] = (uint8_t)((msg >> 8) | 0x80);  // work bit set
	memcpy(frame + 12, g_mag_payload[p_node], 22);
	frame[12 + 18] = (uint8_t)(14 - p_level);  // inverted level; cal bytes preserved
	frame[34] = 0xAF;
	frame[35] = 0xAF;
	int ok = serial_write_n(fd, frame, sizeof(frame));
	WUNLOCK();
	return ok;
}

int sinew_have_mag_for(uint8_t p_node) {
	return (p_node <= 0x0E) && g_mag_present[p_node];
}

int sinew_load_mag_table(const char *p_path) {
	return load_payload_table(p_path, "mag", g_mag_payload, g_mag_present);
}

int sinew_send_shutdown(uint8_t p_node) {
	WLOCK();
	serial_handle_t fd = g_active_fd;
	if (fd == SERIAL_INVALID) {
		WUNLOCK();
		return 0;
	}

	uint16_t msg = next_msg_locked();
	uint8_t frame[36] = {0};
	frame[0] = 0xEA;
	frame[1] = 0xEA;
	frame[2] = 0x07;  // shutdown_node
	frame[3] = 0x00;
	frame[4] = 0xC8;
	frame[5] = p_node;  // 0x00..0x0E or 0xFF
	frame[6] = (uint8_t)(msg & 0xFF);
	frame[7] = (uint8_t)((msg >> 8) & 0x7F);  // shutdown clears 0x8000
	frame[34] = 0xAF;
	frame[35] = 0xAF;
	int ok = serial_write_n(fd, frame, sizeof(frame));
	WUNLOCK();
	return ok;
}

// Generic host->dongle command for config opcodes (set_anti_magnetic_strength
// 0x31, set_transmit_power 0x91, set_rgb 0x41, gyroscope_calibration 0x21, ...).
// opcode = byte 2; target = byte 5 (0x00..0x0E node, 0xC8 OMNI, 0xFF
// ALL_TRACKERS); work_bit sets the 0x8000 "tracker must process" flag in
// MessageNumber; payload (<=22 B) at bytes 12..33.  See Spec.lean "Host writes".
int sinew_send_command(uint8_t opcode, uint8_t target, int work_bit, const uint8_t *payload,
                       int payload_len) {
	WLOCK();
	serial_handle_t fd = g_active_fd;
	if (fd == SERIAL_INVALID) {
		WUNLOCK();
		return 0;
	}

	uint16_t msg = next_msg_locked();
	uint8_t frame[36] = {0};
	frame[0] = 0xEA;
	frame[1] = 0xEA;
	frame[2] = opcode;
	frame[3] = 0x00;
	frame[4] = 0xC8;
	frame[5] = target;
	frame[6] = (uint8_t)(msg & 0xFF);
	frame[7] = (uint8_t)(((msg >> 8) & 0x7F) | (work_bit ? 0x80 : 0x00));
	if (payload && payload_len > 0) {
		memcpy(frame + 12, payload, (size_t)(payload_len > 22 ? 22 : payload_len));
	}
	frame[34] = 0xAF;
	frame[35] = 0xAF;
	int ok = serial_write_n(fd, frame, sizeof(frame));
	WUNLOCK();
	return ok;
}

// ── trackerName (= Main.trackerName) ─────────────────────────────────────

static void tracker_name(const ImuFrame *f, char *out, size_t sz) {
	uint8_t lo = f->pkt_type & 0x0F;
	int is_imu = (lo == PKT_KIND_IMU8 || lo == PKT_KIND_IMU9 || lo == PKT_KIND_IMUD);
	if (is_imu) {
		// byte 5 layout:
		//   bits 4..7 : sub-sensor index 0..2 (which IMU chip on the tracker)
		//   bit  0    : per-tracker session bit; two distinct trackers can
		//               share the same `(hub, idx, sub)` TDMA slot,
		//               distinguished by whether their frames carry bit0=0
		//               or bit0=1.  Including this in the channel name
		//               demultiplexes them.
		//   bits 1..3 : 3-bit frame counter cycling 0..7 within a stream;
		//               useful for drop detection, not part of identity.
		uint8_t sub = f->sub_type >> 4;
		uint8_t flag = f->sub_type & 0x01;
		snprintf(out, sz, "sinew_%s_%u_s%u_f%u", f->hub_id, f->tracker_idx, sub, flag);
	} else {
		snprintf(out, sz, "sinew_%s_%u", f->hub_id, f->tracker_idx);
	}
}


static void udp_send(udp_sock_t, const char *, uint16_t, const uint8_t *, size_t);

// ── Sinew.Udp ─────────────────────────────────────────────────────────────

static udp_sock_t udp_open(void) {
	return socket(AF_INET, SOCK_DGRAM, 0);
}

static void udp_send(udp_sock_t sock, const char *ip, uint16_t port, const uint8_t *buf,
                     size_t len) {
	struct sockaddr_in dst;
	memset(&dst, 0, sizeof(dst));
	dst.sin_family = AF_INET;
	dst.sin_port = htons(port);
	inet_pton(AF_INET, ip, &dst.sin_addr);
	sendto(sock, (const char *)buf, (int)len, 0, (struct sockaddr *)&dst, sizeof(dst));
}

// ── Sinew.Serial ─────────────────────────────────────────────────────────

#ifdef _WIN32

static int serial_read_exact(serial_handle_t h, uint8_t *buf, size_t n) {
	size_t total = 0;
	int eof_retries = 0;
	while (total < n) {
		DWORD got = 0;
		if (!ReadFile(h, buf + total, (DWORD)(n - total), &got, NULL)) {
			return (int)total;
		}
		if (got == 0) {
			if (++eof_retries > 20) {
				return (int)total;
			}
			sleep_ms(10);
			continue;
		}
		eof_retries = 0;
		total += got;
	}
	return (int)total;
}

static serial_handle_t serial_open(const char *path) {
	char full_path[300];
	if (path[0] == '\\' && path[1] == '\\') {
		snprintf(full_path, sizeof(full_path), "%s", path);
	} else {
		snprintf(full_path, sizeof(full_path), "\\\\.\\%s", path);
	}

	HANDLE h =
	    CreateFileA(full_path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (h == INVALID_HANDLE_VALUE) {
		return SERIAL_INVALID;
	}

	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(h, &dcb)) {
		CloseHandle(h);
		return SERIAL_INVALID;
	}
	dcb.BaudRate = CBR_115200;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fBinary = TRUE;
	dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fRtsControl = RTS_CONTROL_ENABLE;
	if (!SetCommState(h, &dcb)) {
		CloseHandle(h);
		return SERIAL_INVALID;
	}

	COMMTIMEOUTS to;
	memset(&to, 0, sizeof(to));
	to.ReadIntervalTimeout = 50;
	to.ReadTotalTimeoutMultiplier = 10;
	to.ReadTotalTimeoutConstant = 5000;
	to.WriteTotalTimeoutMultiplier = 10;
	to.WriteTotalTimeoutConstant = 1000;
	SetCommTimeouts(h, &to);
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	return h;
}

static void serial_close(serial_handle_t h) {
	CloseHandle(h);
}

#else

// Blocking read — returns n on success, -1 on disconnect (EIO/EOF).
// No timeouts; the aging thread drives lifecycle updates independently.
static int serial_read_exact(serial_handle_t fd, uint8_t *buf, size_t n) {
	size_t total = 0;
	while (total < n) {
		ssize_t r = read(fd, buf + total, n - total);
		if (r <= 0) {
			return -1;
		}
		total += (size_t)r;
	}
	return (int)total;
}

static serial_handle_t serial_open(const char *path) {
	int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		return -1;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);

	struct termios tty;
	if (tcgetattr(fd, &tty) != 0) {
		close(fd);
		return -1;
	}

	cfmakeraw(&tty);
	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cflag &= ~HUPCL;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 50;
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		close(fd);
		return -1;
	}
	tcflush(fd, TCIFLUSH);

	int modem_bits = TIOCM_DTR | TIOCM_RTS;
	ioctl(fd, TIOCMBIS, &modem_bits);
	return fd;
}

static void serial_close(serial_handle_t fd) {
	close(fd);
}

#endif

// ── Auto-detect: pick the first openable CDC-ACM candidate ───────────────
//
// No passive read-probe.  The dongle is silent until it receives a wake_up,
// so taste-testing a candidate for sync bytes forces a blocking wait and still
// misses a fresh dongle.  Instead the first openable candidate goes to the
// connection state machine: the main loop opens it, sends the wake_up (the
// trigger), and the frame reader + Lifecycle FSM confirm the device by the
// frames it emits — or the reconnect path moves on after a disconnect.
// Event-driven, no timing constants.  On Linux /dev/ttyACM0 is the usual
// dongle node and is tried first.

#ifdef _WIN32

// Match the dongle by USB VID:PID (REBORNRX = VID_248A&PID_8002) and read its
// COM port from the device's registry PortName.  This is exact regardless of
// which COMn the OS assigned, so it beats the COM-number fallbacks below.
#define SINEW_DONGLE_HWID "VID_248A&PID_8002"
#define SINEW_DEFAULT_WIN_PORT "COM5"

static int find_dongle_by_vidpid(char *out, size_t out_sz) {
	// GUID_DEVCLASS_PORTS — the COM & LPT ports class (avoids a devguid.h dep).
	static const GUID ports_class = {
	    0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};
	HDEVINFO devs = SetupDiGetClassDevsA(&ports_class, NULL, NULL, DIGCF_PRESENT);
	if (devs == INVALID_HANDLE_VALUE) {
		return 0;
	}
	int found = 0;
	SP_DEVINFO_DATA di;
	di.cbSize = sizeof(di);
	for (DWORD i = 0; !found && SetupDiEnumDeviceInfo(devs, i, &di); i++) {
		char hwid[512] = {0};
		if (!SetupDiGetDeviceRegistryPropertyA(devs, &di, SPDRP_HARDWAREID, NULL, (PBYTE)hwid,
		                                       sizeof(hwid) - 1, NULL)) {
			continue;
		}
		// Hardware IDs are upper-case (e.g. USB\VID_248A&PID_8002&...).
		for (char *p = hwid; *p; p++) {
			*p = (char)toupper((unsigned char)*p);
		}
		if (!strstr(hwid, SINEW_DONGLE_HWID)) {
			continue;
		}
		HKEY hk = SetupDiOpenDevRegKey(devs, &di, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
		if (hk == INVALID_HANDLE_VALUE) {
			continue;
		}
		char port[64] = {0};
		DWORD type = 0, len = sizeof(port) - 1;
		if (RegQueryValueExA(hk, "PortName", NULL, &type, (LPBYTE)port, &len) == ERROR_SUCCESS &&
		    type == REG_SZ) {
			port[len < sizeof(port) ? len : sizeof(port) - 1] = 0;
			snprintf(out, out_sz, "%.*s", (int)(out_sz - 1), port);
			found = 1;
		}
		RegCloseKey(hk);
	}
	SetupDiDestroyDeviceInfoList(devs);
	return found;
}

static int auto_detect_serial(char *out, size_t out_sz) {
	// 1) Exact: find the dongle by VID:PID and use whatever COM it landed on.
	if (find_dongle_by_vidpid(out, out_sz)) {
		return 1;
	}

	// 2) Hint: this kit's dongle usually enumerates as COM5.
	{
		serial_handle_t h = serial_open(SINEW_DEFAULT_WIN_PORT);
		if (h != SERIAL_INVALID) {
			serial_close(h);
			snprintf(out, out_sz, "%.*s", (int)(out_sz - 1), SINEW_DEFAULT_WIN_PORT);
			return 1;
		}
	}

	// 3) Fallback: first openable SERIALCOMM port.
	HKEY key;
	if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) !=
	    ERROR_SUCCESS) {
		return 0;
	}

	char ports[16][256];
	int n_ports = 0;
	DWORD index = 0;
	while (n_ports < 16) {
		char vname[256];
		BYTE data[256];
		DWORD vlen = sizeof(vname), dlen = sizeof(data), type;
		LONG r = RegEnumValueA(key, index++, vname, &vlen, NULL, &type, data, &dlen);
		if (r == ERROR_NO_MORE_ITEMS) {
			break;
		}
		if (r != ERROR_SUCCESS || type != REG_SZ) {
			continue;
		}
		// RegEnumValueA does not guarantee NUL termination for REG_SZ.
		data[dlen < sizeof(data) ? dlen : sizeof(data) - 1] = 0;
		snprintf(ports[n_ports++], sizeof(ports[0]), "%s", (const char *)data);
	}
	RegCloseKey(key);

	for (int i = 0; i < n_ports; i++) {
		serial_handle_t h = serial_open(ports[i]);
		if (h == SERIAL_INVALID) {
			continue;
		}
		serial_close(h);
		snprintf(out, out_sz, "%.*s", (int)(out_sz - 1), ports[i]);
		return 1;
	}
	return 0;
}

#else

// First openable candidate wins; /dev/ttyACM0 (the dongle's CDC-ACM node) is
// tried before generic USB-serial.  Confirmation is the frame reader's job.
static int auto_detect_serial(char *out, size_t out_sz) {
	static const char *patterns[] = {"/dev/ttyACM*",       // Linux CDC-ACM (dongle)
	                                 "/dev/cu.usbmodem*",  // macOS
	                                 "/dev/ttyUSB*",       // Linux USB-serial
	                                 NULL};
	for (int p = 0; patterns[p]; p++) {
		glob_t g;
		if (glob(patterns[p], 0, NULL, &g) != 0) {
			continue;
		}
		for (size_t i = 0; i < g.gl_pathc; i++) {
			int fd = serial_open(g.gl_pathv[i]);
			if (fd < 0) {
				continue;
			}
			serial_close(fd);
			snprintf(out, out_sz, "%s", g.gl_pathv[i]);
			globfree(&g);
			return 1;
		}
		globfree(&g);
	}
	return 0;
}

#endif

// ── Frame reader with sync recovery (= sinew_serial_read_frame in FFI) ───

static int valid_pkt_type(uint8_t pkt) {
	uint8_t lo = pkt & 0x0F;
	return lo == 0x8 || lo == 0x9 || lo == 0xA || lo == 0xD;
}

static const uint8_t SYNC[4] = {0xFA, 0xFA, 0xAE, 0xAE};

// Returns 1 on valid frame, -1 on disconnect.
static int read_frame(serial_handle_t fd, uint8_t *buf) {
	if (serial_read_exact(fd, buf, 36) < 0) {
		return -1;
	}
	if (memcmp(buf, SYNC, 4) == 0) {
		return 1;
	}

	// Sync at offset 1-32
	for (int p = 1; p <= 32; p++) {
		if (memcmp(buf + p, SYNC, 4) == 0) {
			memmove(buf, buf + p, 36 - p);
			if (serial_read_exact(fd, buf + 36 - p, p) < 0) {
				return -1;
			}
			if (memcmp(buf, SYNC, 4) == 0) {
				return 1;
			}
		}
	}

	// Byte-by-byte sync scanner
	uint8_t win[4];
	memcpy(win, buf + 32, 4);
	for (int attempts = 0; attempts < 512; attempts++) {
		if (memcmp(win, SYNC, 4) == 0) {
			uint8_t pkt;
			if (serial_read_exact(fd, &pkt, 1) < 0) {
				return -1;
			}
			if (!valid_pkt_type(pkt)) {
				win[0] = win[1];
				win[1] = win[2];
				win[2] = win[3];
				win[3] = pkt;
				continue;
			}
			buf[0] = 0xFA;
			buf[1] = 0xFA;
			buf[2] = 0xAE;
			buf[3] = 0xAE;
			buf[4] = pkt;
			if (serial_read_exact(fd, buf + 5, 31) < 0) {
				return -1;
			}
			return 1;
		}
		uint8_t b;
		if (serial_read_exact(fd, &b, 1) < 0) {
			return -1;
		}
		win[0] = win[1];
		win[1] = win[2];
		win[2] = win[3];
		win[3] = b;
	}
	return -1;
}

// ── FrameSource adapter: the live serial port ───────────────────────────────
// Wraps a connected dongle handle as a FrameSource so the run loop pulls frames
// through the driven port rather than calling the raw reader directly.
// rawlog_source is the other FrameSource implementation (replay from disk).
typedef struct {
	serial_handle_t fd;
} SerialSourceCtx;

static int serial_source_next(void *c, uint8_t *frame36) {
	SerialSourceCtx *s = (SerialSourceCtx *)c;
	return read_frame(s->fd, frame36);  // 1 = frame, <0 = disconnect (FrameSource contract)
}
static void serial_source_close(void *c) {
	(void)c;  // the run loop owns the fd lifetime (publishes/closes it under lock)
}

#define NAME_LEN 48

static uint8_t g_last_ctr[SINEW_JOINT_COUNT];
static uint64_t g_t_start = 0;

// ── OSC-out adapter: a UDP TrackerSink (the driver's driven port) ────────────
// Fans every /sinew packet to the consumer app (dest_ip:osc_port) and, if a
// monitor is configured, mirrors it to 127.0.0.1:monitor_port.  Swapping this
// for a capture sink is how the emit path is property-tested.
typedef struct {
	udp_sock_t sock;
	const char *dest_ip;
	uint16_t osc_port, monitor_port;
} UdpSinkCtx;

static void udp_sink_send(void *c, const uint8_t *osc, size_t len) {
	UdpSinkCtx *u = (UdpSinkCtx *)c;
	udp_send(u->sock, u->dest_ip, u->osc_port, osc, len);
	if (u->monitor_port) {
		udp_send(u->sock, "127.0.0.1", u->monitor_port, osc, len);
	}
}
static void udp_sink_close(void *c) {
	UdpSinkCtx *u = (UdpSinkCtx *)c;
	if (u->sock != UDP_INVALID) {
		UDP_CLOSE(u->sock);
	}
}

// Per-joint TDMA flag lock: two physical trackers can share the same
// hub/idx/sub slot, distinguished by bit 0 of sub_type.  Lock onto the
// flag of the first frame seen and reject the other.
typedef struct {
	int seen;
	int locked_flag;
} SubState;

static SubState g_sub[SINEW_JOINT_COUNT];

// ── Main.runDriver ────────────────────────────────────────────────────────

void sinew_driver_stop(void) {
	g_stop = 1;
	// Wake the lifecycle SM thread so it sees g_stop and exits (it blocks on the
	// queue condvar otherwise, hanging the join below).
	pthread_mutex_lock(&g_lc_mu);
	pthread_cond_signal(&g_lc_cv);
	pthread_mutex_unlock(&g_lc_mu);
}

void sinew_config_default(SinewConfig *cfg) {
	// Empty serial_port = automatic detection (probe each serial port for the
	// FA FA AE AE sync pattern).  Override with --port to force.
	cfg->serial_port[0] = '\0';
	strncpy(cfg->dest_ip, "127.0.0.1", sizeof(cfg->dest_ip) - 1);
	cfg->osc_port = 39539;
	cfg->monitor_port = 0;  // off unless a monitor (e.g. the TUI) requests a mirror
	cfg->verbose = 0;
	cfg->logfp = NULL;
	cfg->rawfp = NULL;
}

#define DLOG(cfg, ...)                          \
	do {                                        \
		if ((cfg)->logfp) {                     \
			fprintf((cfg)->logfp, __VA_ARGS__); \
			fflush((cfg)->logfp);               \
		}                                       \
	} while (0)

void sinew_driver_run(const SinewConfig *cfg) {
	int auto_port = (cfg->serial_port[0] == '\0');
	DLOG(cfg, "Sinew C driver\n");
	DLOG(cfg, "  Serial : %s\n", auto_port ? "(probe)" : cfg->serial_port);
	DLOG(cfg, "  Target : %s:%u\n", cfg->dest_ip, cfg->osc_port);
	if (cfg->monitor_port) {
		DLOG(cfg, "  Monitor: 127.0.0.1:%u\n", cfg->monitor_port);
	}

#ifdef _WIN32
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

	memset(g_last_ctr, 0, sizeof(g_last_ctr));
	memset(g_sub, 0, sizeof(g_sub));
	joint_state_init();

	UdpSinkCtx sinkctx = {udp_open(), cfg->dest_ip, cfg->osc_port, cfg->monitor_port};
	TrackerSink sink = {&sinkctx, udp_sink_send, udp_sink_close};
	g_t_start = now_ms();

	pthread_t sm_tid;
	pthread_create(&sm_tid, NULL, lc_sm_thread, NULL);

	uint8_t raw[FRAME_BYTES];

	while (!g_stop) {
		char detected[256];
		const char *port = cfg->serial_port;
		if (auto_port) {
			if (!auto_detect_serial(detected, sizeof(detected))) {
				sleep_ms(1000);
				continue;
			}
			port = detected;
			DLOG(cfg, "Auto-detected dongle on %s\n", port);
		}
		serial_handle_t fd = serial_open(port);
		if (fd == SERIAL_INVALID) {
			sleep_ms(500);
			continue;
		}
		DLOG(cfg, "Serial connected.  Streaming...\n");

		// Drive the read side through the FrameSource port (serial adapter).
		SerialSourceCtx srcctx = {fd};
		FrameSource src = {&srcctx, serial_source_next, serial_source_close};

		// Publish fd so the host-write API can use it.  Send the
		// initial wake_up immediately: first thing out the door is a
		// wake_up to OMNI.
		WLOCK();
		g_active_fd = fd;
		g_msg_counter = 0;  // fresh session, counter starts at 1 on first call
		WUNLOCK();
		sinew_send_wake_up();
		DLOG(cfg, "Sent initial wake_up (mimicking rebocap).\n");

		// Activate all 15 nodes — use stored payload if available, zeros otherwise.
		for (uint8_t n = 0; n < SINEW_JOINT_COUNT; n++) {
			if (g_activate_present[n]) {
				sinew_send_activate(n, g_activate_payload[n]);
				DLOG(cfg, "Sent activate node %u (stored payload).\n", n);
			} else {
				sinew_send_activate(n, (const uint8_t[22]){0});
				DLOG(cfg, "Sent activate node %u (zeros).\n", n);
			}
		}

		while (!g_stop) {
			if ((now_ms() - g_last_wake_ms) > 30000) {
				sinew_send_wake_up();
			}

			if (src.next(src.ctx, raw) < 0) {
				DLOG(cfg, "Serial disconnected — reconnecting\n");
				WLOCK();
				g_active_fd = SERIAL_INVALID;
				WUNLOCK();
				src.close(src.ctx);
				serial_close(fd);
				break;
			}

			if (cfg->rawfp) {
				fprintf(cfg->rawfp, "%llu,", (unsigned long long)now_ms());
				for (int b = 0; b < FRAME_BYTES; b++) {
					fprintf(cfg->rawfp, "%02x", raw[b]);
				}
				fputc('\n', cfg->rawfp);
				fflush(cfg->rawfp);
			}

			ImuFrame frame;
			int pr = parse_frame(raw, &frame);
			if (pr == SINEW_FRAME_OFFLINE) {
				// Dongle reports this slot empty — post the lifecycle OFFLINE
				// event for any joint mapped to the hwid, then move on.  (The
				// codec is pure; the host owns this side-effect.)
				const char *off_joint = sinew_joint_for_hwid(frame.hub_id);
				if (off_joint) {
					lc_post_frame(off_joint, now_ms(), LC_OFFLINE);
				}
				continue;
			}
			if (pr != SINEW_FRAME_OK) {
				continue;
			}

			char name[NAME_LEN];
			tracker_name(&frame, name, sizeof(name));

			// Byte 6 of every high-rate frame IS the Rebocap NodeNumber (0..14).
			// Spec.lean § Frame Layout: "NodeNumber — Rebocap body-part index
			// 0..14; sinew_340_8 frames (RightHand) carry 0x0E=14."
			// Byte 7 is the per-frame counter (increments by 1, wraps at 256).
			// seq_num = get_u16le(b,6) = NodeNumber | (counter << 8), so
			// seq_num & 0xFF gives the NodeNumber directly from any frame.
			//
			// Burst frames (lo nibble A) contribute only to the lifecycle FSM
			// (BURST_HWID event); they never enter the SeqRegistry because
			// their counter is independent of the high-rate counter and merging
			// them causes seq-counter collisions that corrupt the streak.
			int is_burst = ((frame.pkt_type & 0x0F) == PKT_KIND_BURST);
			uint8_t node_num = (uint8_t)(frame.seq_num & 0xFF);
			const char *joint_label = NULL;

			if (is_burst) {
				// Use hwid table for lifecycle only (LC_BURST_HWID event).
				const char *hwid_joint = sinew_joint_for_hwid(frame.hub_id);
				if (hwid_joint) {
					joint_label = hwid_joint;
					lc_post_frame(joint_label, now_ms(), LC_BURST_HWID);
				}
			} else if (node_num < SINEW_JOINT_COUNT) {
				joint_label = SINEW_JOINT_NODE_TABLE[node_num];
				lc_post_frame(joint_label, now_ms(), LC_HIGH_RATE);
			}

			if (joint_label && !is_burst) {
				uint8_t sub_idx = (frame.sub_type >> 4) & 0x07;
				if (sub_idx != 1) {
					continue;
				}

				uint8_t ctr = (uint8_t)(frame.seq_num >> 8);
				int ji = (int)node_num;

				if (ctr == g_last_ctr[ji]) {
					continue;
				}
				g_last_ctr[ji] = ctr;

				SubState *ss = &g_sub[ji];

				// Lock onto the flag of the first frame for this (joint, sub).
				// Two physical trackers can share the same TDMA slot and differ
				// only in the flag bit — reject the secondary one.
				uint8_t flag = frame.sub_type & 0x01;
				if (!ss->seen) {
					ss->locked_flag = (int)flag;
					ss->seen = 1;
				} else if ((int)flag != ss->locked_flag) {
					continue;
				}

				// Orientation: the trained TIC calibrator corrects the device
				// quaternion's mount/drift (R_clean = R_DGᵀ·R_device·R_BSᵀ) once a
				// window is ready.  Until then the uncalibrated device quaternion
				// passes through — there is no complementary-filter fusion.
				Quat q = frame.quat;
				Accel a = frame.accel;

				uint64_t now = now_ms();
				double t = (now - g_t_start) * 0.001;
				uint8_t osc_buf[512];

				// Emit /sinew at sensor rate through the TrackerSink port; the
				// UDP adapter fans out to the consumer app and the optional
				// monitor mirror.  app delivery never depends on a monitor.
				size_t len = build_tracker(osc_buf, joint_label, q.w, q.x, q.y, q.z, t);
				sink.send(sink.ctx, osc_buf, len);
				len = build_accel(osc_buf, joint_label, a.x, a.y, a.z);
				sink.send(sink.ctx, osc_buf, len);

				if (cfg->verbose) {
					DLOG(cfg, "[frame] %s sub=%u ctr=%u\n", joint_label, sub_idx, ctr);
				}
			}
		}
	}

	sink.close(sink.ctx);

	pthread_join(sm_tid, NULL);
#ifdef _WIN32
	WSACleanup();
#endif
}
