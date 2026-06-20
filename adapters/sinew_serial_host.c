// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// host/driver — the slim rebocap driver's serial + UDP adapters around the pure
// core/protocol codec.  Reads 36-byte frames off the serial port, decodes them
// with parse_frame, and emits the /sinew OSC protocol over UDP.  TIC calibration
// (mount/drift correction via accel+mag) is applied when weights are available;
// degrades gracefully to raw device quaternion when wtrained.bin is absent.
// Maps to Main (SeqRegistry, runDriver) + the host-write commands + the lifecycle
// FSM (Lifecycle.lean).

#include "sinew_driver.h"
#include "hwid_table.h"
#include "sinew_protocol.h"  // the wire codec: ImuFrame, parse_frame, build_tracker/accel
#include "tracker_sink.h"    // the driven port the OSC-out adapter implements
#include "frame_source.h"    // the driven port the serial read side implements
#include "command_sink.h"    // the driven port the serial write side implements
#include "tic_calib.h"       // TIC mount/drift calibrator (accel + mag + quat → R_clean)
#include "mag_calib.h"       // online hard+soft iron calibration, persisted to mag_calib.ini
#include "triad_calib.h"     // TRIAD zero-pose reference from gravity + calibrated mag

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

/* Rotate vector (vx,vy,vz) by unit quaternion q into (rx,ry,rz).
   v' = q * v * q^{-1}  via  v' = v + 2w(q×v) + 2(q×(q×v)) */
static void quat_rotate_vec(Quat q, float vx, float vy, float vz,
                             float *rx, float *ry, float *rz) {
	float tx = 2.f*(q.y*vz - q.z*vy);
	float ty = 2.f*(q.z*vx - q.x*vz);
	float tz = 2.f*(q.x*vy - q.y*vx);
	*rx = vx + q.w*tx + q.y*tz - q.z*ty;
	*ry = vy + q.w*ty + q.z*tx - q.x*tz;
	*rz = vz + q.w*tz + q.x*ty - q.y*tx;
}

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

// ── CommandSink adapter: the serial write side (host -> dongle) ──────────────
// The write twin of the serial FrameSource on the same link.  Every sinew_send_*
// builds its 36-byte frame under WLOCK with g_active_fd validated, then emits it
// through this port — a faithful wrap of serial_write_n on the active handle.
static int serial_command_send(void *ctx, const uint8_t *frame36) {
	(void)ctx;  // the active handle is the driver-owned g_active_fd (held under WLOCK)
	return serial_write_n(g_active_fd, frame36, FRAME_BYTES);
}
static void serial_command_close(void *ctx) {
	(void)ctx;  // the run loop owns the fd lifetime
}
static CommandSink g_cmd_sink = {NULL, serial_command_send, serial_command_close};

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
	int ok = g_cmd_sink.send(g_cmd_sink.ctx, frame);
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
	int ok = g_cmd_sink.send(g_cmd_sink.ctx, frame);
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
	int ok = g_cmd_sink.send(g_cmd_sink.ctx, frame);
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
	int ok = g_cmd_sink.send(g_cmd_sink.ctx, frame);
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
	int ok = g_cmd_sink.send(g_cmd_sink.ctx, frame);
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

// [joint][sub_idx(0-7)][flag(0-1)] dedup counters.
// sub_idx = (sub_type >> 4) & 0x07, flag = sub_type & 0x01.
#define SUB_MAX 8
static uint8_t g_last_ctr[SINEW_JOINT_COUNT][SUB_MAX][2];
static uint64_t g_t_start = 0;

// Per-joint per-subidx per-flag previous quaternion for phase-alignment.
static Quat g_prev_q[SINEW_JOINT_COUNT][SUB_MAX][2];

// Which (sub_idx, flag) slots have received at least one frame per joint.
static uint8_t g_slot_seen[SINEW_JOINT_COUNT][SUB_MAX][2];
// Previous quaternion for the derived channel (sign continuity).
static Quat g_prev_q_derived[SINEW_JOINT_COUNT];

// Zero-pose reference captured on the first frame from each slot (powerup calibration).
// Emitted orientation = inv(Q_ref) * Q_current  →  initial pose is always identity.
static Quat g_ref_q[SINEW_JOINT_COUNT][SUB_MAX][2];
static uint8_t g_ref_set[SINEW_JOINT_COUNT][SUB_MAX][2];

// TRIAD-based zero-pose reference: physics-grounded using gravity + calibrated mag.
// Replaces the TIC-snapshot reference once mag_calib bootstrap (200 frames) is done.
static Accel   g_a_init[SINEW_JOINT_COUNT];          // accel at first valid frame (sensor frame)
static Accel   g_m_init[SINEW_JOINT_COUNT];          // raw mag at first valid frame
static uint8_t g_init_captured[SINEW_JOINT_COUNT];   // 1 when initial accel+mag captured
static Quat    g_ref_triad[SINEW_JOINT_COUNT];       // TRIAD-computed reference quaternion
static uint8_t g_ref_triad_set[SINEW_JOINT_COUNT];  // 1 when TRIAD ref is valid

// ── 6D rotation merge helpers (Zhou et al., matches sixd.py) ─────────────────
// Quaternion → 6D: first two columns of the rotation matrix.
static void quat_to_6d(Quat q, float v[6]) {
    float w=q.w, x=q.x, y=q.y, z=q.z;
    v[0]=1.f-2.f*(y*y+z*z); v[1]=2.f*(x*y+w*z); v[2]=2.f*(x*z-w*y);
    v[3]=2.f*(x*y-w*z);     v[4]=1.f-2.f*(x*x+z*z); v[5]=2.f*(y*z+w*x);
}

// Average N 6D vectors and decode via Gram-Schmidt → quaternion.
// Matches sixd_to_R in sixd.py and sixd_to_m in tic_calib.c.
static Quat sixd_avg_to_quat(const float sums[6], int n) {
    float v[6]; for (int i=0;i<6;i++) v[i]=sums[i]/n;
    float an=sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    if (an<1e-9f) return (Quat){1,0,0,0};
    float ax=v[0]/an, ay=v[1]/an, az=v[2]/an;
    float dp=ax*v[3]+ay*v[4]+az*v[5];
    float bx=v[3]-dp*ax, by=v[4]-dp*ay, bz=v[5]-dp*az;
    float bn=sqrtf(bx*bx+by*by+bz*bz);
    if (bn<1e-9f) return (Quat){1,0,0,0};
    bx/=bn; by/=bn; bz/=bn;
    float cx=ay*bz-az*by, cy=az*bx-ax*bz, cz=ax*by-ay*bx;
    // Rotation matrix with columns [a,b,c]: tr = ax+by+cz
    float tr=ax+by+cz; Quat q;
    if (tr>0.f){float s=sqrtf(tr+1.f)*2.f;
        q=(Quat){0.25f*s,(bz-cy)/s,(cx-az)/s,(ay-bx)/s};}
    else if (ax>by&&ax>cz){float s=sqrtf(1.f+ax-by-cz)*2.f;
        q=(Quat){(bz-cy)/s,0.25f*s,(ay+bx)/s,(cx+az)/s};}
    else if (by>cz){float s=sqrtf(1.f+by-ax-cz)*2.f;
        q=(Quat){(cx-az)/s,(ay+bx)/s,0.25f*s,(cy+bz)/s};}
    else{float s=sqrtf(1.f+cz-ax-by)*2.f;
        q=(Quat){(ay-bx)/s,(cx+az)/s,(cy+bz)/s,0.25f*s};}
    float nn=sqrtf(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if (nn>1e-9f){q.w/=nn;q.x/=nn;q.y/=nn;q.z/=nn;} return q;
}

// Returns 1 if q is within ~2° of identity — used to detect the low-power sentinel.
// Sensors emit (1,0,0,0) when sleeping; that frame must not set the zero-pose
// reference, must not be emitted, and must not enter the 6D derived average.
static int quat_is_identity(Quat q) {
    // |w| > cos(1°) ≈ 0.99985  →  angle from identity < 2°
    float absw = q.w < 0.f ? -q.w : q.w;
    return absw > 0.99985f;
}

// Faithful C translation of Godot's Basis(q).get_rotation_quaternion():
//   set_quaternion → orthonormalize → det<0 flip → get_quaternion (Shepperd).
// rows[r][c] layout matches Godot's row-major Basis.
static Quat q_align_phase(Quat q) {
    // --- Basis::set_quaternion ---
    float d = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (d < 1e-18f) return (Quat){1.f,0.f,0.f,0.f};
    float s  = 2.0f / d;
    float xs = q.x*s,  ys = q.y*s,  zs = q.z*s;
    float wx = q.w*xs, wy = q.w*ys, wz = q.w*zs;
    float xx = q.x*xs, xy = q.x*ys, xz = q.x*zs;
    float yy = q.y*ys, yz = q.y*zs, zz = q.z*zs;
    float m[3][3] = {
        {1.f-(yy+zz), xy-wz,       xz+wy      },
        {xy+wz,       1.f-(xx+zz), yz-wx       },
        {xz-wy,       yz+wx,       1.f-(xx+yy) }
    };

    // --- Basis::orthonormalize() — Gram-Schmidt on columns ---
    float c0[3]={m[0][0],m[1][0],m[2][0]};
    float n0=sqrtf(c0[0]*c0[0]+c0[1]*c0[1]+c0[2]*c0[2]);
    if(n0>1e-9f){c0[0]/=n0;c0[1]/=n0;c0[2]/=n0;}
    float c1[3]={m[0][1],m[1][1],m[2][1]};
    float d01=c1[0]*c0[0]+c1[1]*c0[1]+c1[2]*c0[2];
    c1[0]-=d01*c0[0];c1[1]-=d01*c0[1];c1[2]-=d01*c0[2];
    float n1=sqrtf(c1[0]*c1[0]+c1[1]*c1[1]+c1[2]*c1[2]);
    if(n1>1e-9f){c1[0]/=n1;c1[1]/=n1;c1[2]/=n1;}
    float c2[3]={m[0][2],m[1][2],m[2][2]};
    float d02=c2[0]*c0[0]+c2[1]*c0[1]+c2[2]*c0[2];
    float d12=c2[0]*c1[0]+c2[1]*c1[1]+c2[2]*c1[2];
    c2[0]-=d02*c0[0]+d12*c1[0];c2[1]-=d02*c0[1]+d12*c1[1];c2[2]-=d02*c0[2]+d12*c1[2];
    float n2=sqrtf(c2[0]*c2[0]+c2[1]*c2[1]+c2[2]*c2[2]);
    if(n2>1e-9f){c2[0]/=n2;c2[1]/=n2;c2[2]/=n2;}
    m[0][0]=c0[0];m[1][0]=c0[1];m[2][0]=c0[2];
    m[0][1]=c1[0];m[1][1]=c1[1];m[2][1]=c1[2];
    m[0][2]=c2[0];m[1][2]=c2[1];m[2][2]=c2[2];

    // --- det < 0: scale by -1 to ensure proper rotation ---
    float det = m[0][0]*(m[1][1]*m[2][2]-m[2][1]*m[1][2])
               -m[1][0]*(m[0][1]*m[2][2]-m[2][1]*m[0][2])
               +m[2][0]*(m[0][1]*m[1][2]-m[1][1]*m[0][2]);
    if (det < 0.f) {
        for(int r=0;r<3;r++) for(int c=0;c<3;c++) m[r][c]*=-1.f;
    }

    // --- Basis::get_quaternion() — Shepperd's method ---
    // temp[0]=x, temp[1]=y, temp[2]=z, temp[3]=w  (Godot convention)
    float trace = m[0][0]+m[1][1]+m[2][2];
    float temp[4];
    if (trace > 0.f) {
        float sv = sqrtf(trace+1.f);
        temp[3] = sv*0.5f;
        sv = 0.5f/sv;
        temp[0] = (m[2][1]-m[1][2])*sv;
        temp[1] = (m[0][2]-m[2][0])*sv;
        temp[2] = (m[1][0]-m[0][1])*sv;
    } else {
        int i = (m[0][0]<m[1][1]) ? ((m[1][1]<m[2][2])?2:1) : ((m[0][0]<m[2][2])?2:0);
        int j = (i+1)%3, k = (i+2)%3;
        float sv = sqrtf(m[i][i]-m[j][j]-m[k][k]+1.f);
        temp[i]  = sv*0.5f;
        sv = 0.5f/sv;
        temp[3]  = (m[k][j]-m[j][k])*sv;
        temp[j]  = (m[j][i]+m[i][j])*sv;
        temp[k]  = (m[k][i]+m[i][k])*sv;
    }
    return (Quat){temp[3], temp[0], temp[1], temp[2]};  // (w,x,y,z)
}

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
	strncpy(cfg->mag_calib_path, "mag_calib.ini", sizeof(cfg->mag_calib_path) - 1);
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

	memset(g_last_ctr,       0, sizeof(g_last_ctr));
	memset(g_sub,            0, sizeof(g_sub));
	memset(g_slot_seen,      0, sizeof(g_slot_seen));
	memset(g_ref_set,        0, sizeof(g_ref_set));
	memset(g_init_captured,  0, sizeof(g_init_captured));
	memset(g_ref_triad_set,  0, sizeof(g_ref_triad_set));
	memset(g_a_init,         0, sizeof(g_a_init));
	memset(g_m_init,         0, sizeof(g_m_init));
	for (int i = 0; i < SINEW_JOINT_COUNT; i++) {
		g_prev_q_derived[i] = (Quat){1.f, 0.f, 0.f, 0.f};
		for (int s = 0; s < SUB_MAX; s++)
			for (int f = 0; f < 2; f++) {
				g_prev_q[i][s][f] = (Quat){1.f, 0.f, 0.f, 0.f};
				g_ref_q[i][s][f]  = (Quat){1.f, 0.f, 0.f, 0.f};
			}
	}
	joint_state_init();

	// Register joint names so mag_calib can key the INI by joint label.
	for (int i = 0; i < SINEW_JOINT_COUNT; i++)
		mag_calib_set_joint_name(i, SINEW_JOINT_NODE_TABLE[i]);
	if (cfg->mag_calib_path[0])
		mag_calib_load(cfg->mag_calib_path);

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
				uint8_t flag    = frame.sub_type & 0x01;
				uint8_t ctr     = (uint8_t)(frame.seq_num >> 8);
				int ji = (int)node_num;

				// Dedup per (joint, sub_idx, flag) — each unique combination is
				// a distinct data stream and gets its own named arrow.
				if (ctr == g_last_ctr[ji][sub_idx][flag]) {
					continue;
				}
				g_last_ctr[ji][sub_idx][flag] = ctr;

				// Stream name: "<joint>_s<subidx>f<flag>"
				// e.g. "RightFoot_s0f1", "RightFoot_s2f0"
				char flagged_label[NAME_LEN + 8];
				snprintf(flagged_label, sizeof(flagged_label), "%s_s%uf%u",
				         joint_label, (unsigned)sub_idx, (unsigned)flag);

				Quat q = frame.quat;
				Accel a = frame.accel;
				Accel m = frame.mag;

				uint64_t now = now_ms();
				double t = (now - g_t_start) * 0.001;
				uint8_t osc_buf[512];

				// Feed accel + mag + quat into the TIC calibrator window.  Once
				// 32 snapshots are collected the net runs and refreshes per-sensor
				// R_BS (mount) and R_DG (drift).  Degrades to a no-op when
				// wtrained.bin / GPU are absent.
				int sensor_idx = joint_index_for_name(joint_label);
				if (sensor_idx >= 0) {
					tic_calib_push(sensor_idx, a, m, q, now);
				}

				// Apply mount/drift correction: R_clean = R_DGᵀ·R_device·R_BSᵀ.
				// Returns 1 and writes corrected quat when a calibration is ready;
				// 0 → pass the raw device quaternion through unchanged.
				Quat q_out;
				if (sensor_idx < 0 || !tic_calib_apply(sensor_idx, q, &q_out)) {
					q_out = q;
				}

				// Re-normalise before emitting.
				{
					float n = sqrtf(q_out.w*q_out.w + q_out.x*q_out.x +
					                q_out.y*q_out.y + q_out.z*q_out.z);
					if (n > 1e-9f) {
						q_out.w /= n; q_out.x /= n; q_out.y /= n; q_out.z /= n;
					} else {
						q_out = (Quat){1.f, 0.f, 0.f, 0.f};
					}
				}

				// Ignore identity quaternion — it signals low-power / sleeping sensor.
				// Do not set reference, do not emit, do not include in derived average.
				if (quat_is_identity(q_out)) continue;

				// mag_calib_push must run BEFORE zero-pose so it sees the absolute
				// sensor→world orientation (not the relativized one).  Also accumulates
				// gravity direction (Ag) for the TRIAD bootstrap.
				if (ji >= 0 && ji < SINEW_JOINT_COUNT && cfg->mag_calib_path[0]) {
					if (frame.mag_valid)
						mag_calib_push(ji, q_out, m, a, cfg->mag_calib_path);

					// Capture initial accel+mag for TRIAD (first valid frame per joint).
					if (!g_init_captured[ji] && frame.mag_valid) {
						g_a_init[ji] = a;
						g_m_init[ji] = m;
						g_init_captured[ji] = 1;
					}

					// Try TRIAD whenever we have init data but no physics-grounded ref yet.
					if (g_init_captured[ji] && !g_ref_triad_set[ji]) {
						float Bg[3], Ag[3];
						if (mag_calib_get_refs(ji, Bg, Ag)) {
							Accel m_cal;
							mag_calib_apply(ji, g_m_init[ji], &m_cal);
							float a_arr[3] = {g_a_init[ji].x, g_a_init[ji].y, g_a_init[ji].z};
							float m_arr[3] = {m_cal.x, m_cal.y, m_cal.z};
							Quat q_triad;
							if (triad_compute(a_arr, m_arr, Ag, Bg, &q_triad)) {
								g_ref_triad[ji] = q_triad;
								g_ref_triad_set[ji] = 1;
								// Retroactively align all already-active slots to the TRIAD
								// reference.  Slots that came online before TRIAD was ready
								// captured a random first-frame as their zero-pose; updating
								// them here puts every slot in the same reference frame so the
								// 6D-averaged derived channel tracks ch1/ch2 proportionally.
								for (int _s = 0; _s < SUB_MAX; _s++)
									for (int _f = 0; _f < 2; _f++)
										if (g_ref_set[ji][_s][_f])
											g_ref_q[ji][_s][_f] = q_triad;
							}
						}
					}
				}

				// Save absolute orientation before zero-pose.
				// Accel and magcal are rotated using this so all slots on the same joint
				// produce the same world-frame direction (q_out is per-slot and differs
				// across slots with different zero-pose references, causing oscillation).
				Quat q_abs = q_out;

				// Zero-pose calibration (powerup): capture first frame as reference so
				// the emitted orientation starts at identity.  Q_emit = inv(Q_ref) * Q_out.
				// TRIAD gives a physics-grounded reference; fall back to TIC snapshot if not ready.
				if (ji >= 0 && ji < SINEW_JOINT_COUNT) {
					if (!g_ref_set[ji][sub_idx][flag]) {
						g_ref_q[ji][sub_idx][flag] = g_ref_triad_set[ji] ? g_ref_triad[ji] : q_out;
						g_ref_set[ji][sub_idx][flag] = 1;
					}
					Quat r = g_ref_q[ji][sub_idx][flag];
					// inv(r) * q_out  (unit quaternion: inv = conjugate)
					Quat ri = {r.w, -r.x, -r.y, -r.z};
					Quat qr;
					qr.w = ri.w*q_out.w - ri.x*q_out.x - ri.y*q_out.y - ri.z*q_out.z;
					qr.x = ri.w*q_out.x + ri.x*q_out.w + ri.y*q_out.z - ri.z*q_out.y;
					qr.y = ri.w*q_out.y - ri.x*q_out.z + ri.y*q_out.w + ri.z*q_out.x;
					qr.z = ri.w*q_out.z + ri.x*q_out.y - ri.y*q_out.x + ri.z*q_out.w;
					float nn = sqrtf(qr.w*qr.w+qr.x*qr.x+qr.y*qr.y+qr.z*qr.z);
					if (nn > 1e-9f) { qr.w/=nn; qr.x/=nn; qr.y/=nn; qr.z/=nn; }
					q_out = qr;
				}

				// Godot spherical_cubic_interpolate phase-aligner:
				// 1. Align flip phase: Basis(q).get_rotation_quaternion()
				// 2. Flip to shortest path: signbit(from_q.dot(to_q)) → negate
				q_out = q_align_phase(q_out);
				if (ji >= 0 && ji < SINEW_JOINT_COUNT) {
					Quat *pq = &g_prev_q[ji][sub_idx][flag];
					float dot = pq->w*q_out.w + pq->x*q_out.x
					          + pq->y*q_out.y + pq->z*q_out.z;
					if (dot < 0.f) {
						q_out.w = -q_out.w; q_out.x = -q_out.x;
						q_out.y = -q_out.y; q_out.z = -q_out.z;
					}
					*pq = q_out;
				}

				// Emit /sinew at sensor rate through the TrackerSink port.
				// Use q_abs (pre-zero-pose) so all slots converge on the same world direction.
				float aw_x, aw_y, aw_z;
				quat_rotate_vec(q_abs, a.x, a.y, a.z, &aw_x, &aw_y, &aw_z);

				size_t len = build_tracker(osc_buf, flagged_label, q_out.w, q_out.x, q_out.y, q_out.z, t);
				sink.send(sink.ctx, osc_buf, len);
				len = build_accel(osc_buf, flagged_label, aw_x, aw_y, aw_z);
				sink.send(sink.ctx, osc_buf, len);

				// Emit joint lifecycle state for Blender sphere visualisation.
				if (ji >= 0 && ji < SINEW_JOINT_COUNT) {
					pthread_mutex_lock(&g_js_mu);
					int st = (int)g_joint_state[ji].state;
					pthread_mutex_unlock(&g_js_mu);
					len = build_state_status(osc_buf, joint_label, st);
					sink.send(sink.ctx, osc_buf, len);
				}

				if (frame.mag_valid) {
					len = build_mag(osc_buf, flagged_label, m.x, m.y, m.z);
					sink.send(sink.ctx, osc_buf, len);

					// Emit calibrated mag vector in absolute world frame (q_abs) for Blender.
					if (ji >= 0 && ji < SINEW_JOINT_COUNT && cfg->mag_calib_path[0]) {
						Accel Bcal;
						mag_calib_apply(ji, m, &Bcal);
						float bw_x, bw_y, bw_z;
						quat_rotate_vec(q_abs, Bcal.x, Bcal.y, Bcal.z, &bw_x, &bw_y, &bw_z);
						char magcal_label[NAME_LEN + 8];
						snprintf(magcal_label, sizeof(magcal_label), "%s_magcal", joint_label);
						len = build_magcal(osc_buf, magcal_label, bw_x, bw_y, bw_z);
						sink.send(sink.ctx, osc_buf, len);
						// Calibration progress pie chart (0.0–1.0)
						float progress = mag_calib_get_progress(ji);
						len = build_calib_status(osc_buf, joint_label, progress);
						sink.send(sink.ctx, osc_buf, len);

						// Compass quality = raw magnetometer magnitude (LSB).
						// Deviates from per-sensor constant when near ferromagnetic objects.
						float mag_mag = sqrtf(m.x*m.x + m.y*m.y + m.z*m.z);
						len = build_magqual(osc_buf, joint_label, mag_mag);
						sink.send(sink.ctx, osc_buf, len);
					}
				}

				// Battery level from burst-frame trailing byte 28 (0x3a raw28).
				// raw28 is 0 for non-burst frames; normalise to 0.0-1.0 assuming 0-100 range.
				if (frame.raw28 > 0) {
					float batt = (float)frame.raw28 / 100.f;
					if (batt > 1.f) batt = 1.f;
					len = build_battery(osc_buf, joint_label, batt);
					sink.send(sink.ctx, osc_buf, len);
				}

				// Mark slot active and emit the 6D-averaged derived channel.
				// R_clean per slot is already in g_prev_q (tic_calib + phase-aligned).
				// Average all seen, non-sleeping slots via sixd_to_R → "<joint>_derived".
				g_slot_seen[ji][sub_idx][flag] = 1;
				{
					float v6sum[6] = {0};
					int n_slots = 0;
					for (int si = 0; si < SUB_MAX; si++) {
						for (int fi = 0; fi < 2; fi++) {
							if (!g_slot_seen[ji][si][fi]) continue;
							if (quat_is_identity(g_prev_q[ji][si][fi])) continue;
							float v6[6];
							quat_to_6d(g_prev_q[ji][si][fi], v6);
							for (int k = 0; k < 6; k++) v6sum[k] += v6[k];
							n_slots++;
						}
					}
					if (n_slots >= 2) {
						Quat qd = sixd_avg_to_quat(v6sum, n_slots);
						qd = q_align_phase(qd);
						Quat *pd = &g_prev_q_derived[ji];
						float ddot = pd->w*qd.w + pd->x*qd.x + pd->y*qd.y + pd->z*qd.z;
						if (ddot < 0.f) {
							qd.w=-qd.w; qd.x=-qd.x; qd.y=-qd.y; qd.z=-qd.z;
						}
						*pd = qd;
						char derived_label[NAME_LEN + 10];
						snprintf(derived_label, sizeof(derived_label), "%s_derived", joint_label);
						len = build_tracker(osc_buf, derived_label, qd.w, qd.x, qd.y, qd.z, t);
						sink.send(sink.ctx, osc_buf, len);
					}
				}

				if (cfg->verbose) {
					DLOG(cfg, "[frame] %s flag=%u ctr=%u\n", flagged_label, (unsigned)flag, ctr);
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
