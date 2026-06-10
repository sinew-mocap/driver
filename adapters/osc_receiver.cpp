// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
#include "osc_receiver.hpp"
#include "nonic.h"
#include <chrono>
#include <cmath>
#include <cstring>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
using sock_native_t = SOCKET;
static constexpr sock_native_t kInvalidSock = INVALID_SOCKET;
static int close_sock(sock_native_t s) {
	return ::closesocket(s);
}
struct WsaInit {
	WsaInit() {
		WSADATA wsa;
		WSAStartup(MAKEWORD(2, 2), &wsa);
	}
	~WsaInit() {
		WSACleanup();
	}
};
static WsaInit g_wsa_init;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using sock_native_t = int;
static constexpr sock_native_t kInvalidSock = -1;
static int close_sock(sock_native_t s) {
	return ::close(s);
}
#endif

OscReceiver::OscReceiver(uint16_t port) : port_(port) {
}

OscReceiver::~OscReceiver() {
	stop();
}

void OscReceiver::start() {
	// Primary listen socket — exclusive, no SO_REUSEPORT
	sock_native_t s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s == kInvalidSock) {
		return;
	}

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port_);
	addr.sin_addr.s_addr = INADDR_ANY;
	bind(s, (sockaddr *)&addr, sizeof(addr));

#ifdef _WIN32
	DWORD tv = 50;
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
#else
	timeval tv{0, 50000};
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
	sock_ = (std::intptr_t)s;

	running_ = true;
	thread_ = std::thread(&OscReceiver::run, this);
}

void OscReceiver::stop() {
	running_ = false;
	if (thread_.joinable()) {
		thread_.join();
	}
	if (sock_ != -1) {
		close_sock((sock_native_t)sock_);
		sock_ = -1;
	}
}

std::unordered_map<std::string, TrackerState> OscReceiver::snapshot() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return trackers_;
}

void OscReceiver::purgeOlderThan(float seconds) {
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	std::lock_guard<std::mutex> lock(mutex_);
	for (std::unordered_map<std::string, TrackerState>::iterator it = trackers_.begin();
	     it != trackers_.end();) {
		float age = std::chrono::duration<float>(now - it->second.last_update).count();
		it = (it->second.packets > 0 && age > seconds) ? trackers_.erase(it) : std::next(it);
	}
}

void OscReceiver::run() {
	uint8_t buf[4096];
	while (running_) {
		int n = recv((sock_native_t)sock_, (char *)buf, (int)sizeof(buf), 0);
		if (n > 0) {
			process(buf, (size_t)n);
		}
	}
}

std::string OscReceiver::readString(const uint8_t *buf, size_t len, size_t &pos) {
	if (pos >= len) {
		return {};
	}
	const char *start = (const char *)(buf + pos);
	size_t slen = strnlen(start, len - pos);
	std::string s(start, slen);
	pos += align4(slen + 1);
	return s;
}

float OscReceiver::readFloat(const uint8_t *buf, size_t len, size_t &pos) {
	if (pos + 4 > len) {
		return 0.0f;
	}
	uint32_t bits = ((uint32_t)buf[pos] << 24) | ((uint32_t)buf[pos + 1] << 16) |
	                ((uint32_t)buf[pos + 2] << 8) | (uint32_t)buf[pos + 3];
	pos += 4;
	float f;
	std::memcpy(&f, &bits, 4);
	return f;
}

int32_t OscReceiver::readInt32(const uint8_t *buf, size_t len, size_t &pos) {
	if (pos + 4 > len) {
		return 0;
	}
	int32_t v = ((int32_t)buf[pos] << 24) | ((int32_t)buf[pos + 1] << 16) |
	            ((int32_t)buf[pos + 2] << 8) | (int32_t)buf[pos + 3];
	pos += 4;
	return v;
}

static uint8_t extractIdx(const std::string &name) {
	size_t p = name.rfind('_');
	if (p == std::string::npos || p + 1 >= name.size()) {
		return 0;
	}
	if (name[p + 1] == 's') {
		size_t q = name.rfind('_', p - 1);
		if (q == std::string::npos) {
			return 0;
		}
		p = q;
	}
	int v = 0;
	for (size_t i = p + 1; i < name.size() && std::isdigit((unsigned char)name[i]); ++i) {
		v = v * 10 + (name[i] - '0');
	}
	return (uint8_t)v;
}

static std::string extractHub(const std::string &name) {
	std::string n = name;
	size_t last = n.rfind('_');
	if (last != std::string::npos && last + 1 < n.size() && n[last + 1] == 's') {
		n = n.substr(0, last);
	}
	last = n.rfind('_');
	if (last == std::string::npos || last == 0) {
		return "";
	}
	size_t prev = n.rfind('_', last - 1);
	if (prev == std::string::npos) {
		return "";
	}
	return n.substr(prev + 1, last - prev - 1);
}

// Unit quaternion (w,x,y,z) → rotation matrix, row-major 3×3.  The continuous
// SO(3) representation (the 6D rep of Zhou et al. is its first two columns, third
// recovered by cross product); used so kinematics never narrow to the quaternion.
static void quatToMat(const Quaternion &q, double R[9]) {
	double nrm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (nrm < 1e-12) {
		for (int i = 0; i < 9; ++i) {
			R[i] = (i % 4 == 0) ? 1.0 : 0.0;
		}
		return;
	}
	double w = q.w / nrm, x = q.x / nrm, y = q.y / nrm, z = q.z / nrm;
	R[0] = 1 - 2 * (y * y + z * z);
	R[1] = 2 * (x * y - z * w);
	R[2] = 2 * (x * z + y * w);
	R[3] = 2 * (x * y + z * w);
	R[4] = 1 - 2 * (x * x + z * z);
	R[5] = 2 * (y * z - x * w);
	R[6] = 2 * (x * z - y * w);
	R[7] = 2 * (y * z + x * w);
	R[8] = 1 - 2 * (x * x + y * y);
}

// Angular-velocity vector ω = vee(skew(Ṙ Rᵀ)), with Ṙ ≈ (Rcur − Rprev)/dt.
// The vee map is the so(3)≅ℝ³ cross-product isomorphism (Murray–Li–Sastry).
static void angVel(const double Rcur[9], const double Rprev[9], double dt, double w[3]) {
	double Rd[9];
	for (int k = 0; k < 9; ++k) {
		Rd[k] = (Rcur[k] - Rprev[k]) / dt;
	}
	double M[9];  // M = Ṙ · Rcurᵀ
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			M[i * 3 + j] = Rd[i * 3 + 0] * Rcur[j * 3 + 0] + Rd[i * 3 + 1] * Rcur[j * 3 + 1] +
			               Rd[i * 3 + 2] * Rcur[j * 3 + 2];
		}
	}
	w[0] = 0.5 * (M[7] - M[5]);  // vee of the skew part (M − Mᵀ)/2
	w[1] = 0.5 * (M[2] - M[6]);
	w[2] = 0.5 * (M[3] - M[1]);
}

void OscReceiver::process(const uint8_t *buf, size_t len) {
	size_t pos = 0;
	std::string addr = readString(buf, len, pos);
	std::string tags = readString(buf, len, pos);
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

	if (addr == "/sinew/tracker" && tags.size() >= 7 && tags[0] == ',') {
		std::string name = readString(buf, len, pos);
		float qw = readFloat(buf, len, pos);
		float qx = readFloat(buf, len, pos);
		float qy = readFloat(buf, len, pos);
		float qz = readFloat(buf, len, pos);
		// time_s: OSC type 'd' (float64) — VMC /VMC/Ext/T elapsed seconds
		double time_s = 0.0;
		if (pos + 8 <= len) {
			uint64_t bits = ((uint64_t)buf[pos] << 56) | ((uint64_t)buf[pos + 1] << 48) |
			                ((uint64_t)buf[pos + 2] << 40) | ((uint64_t)buf[pos + 3] << 32) |
			                ((uint64_t)buf[pos + 4] << 24) | ((uint64_t)buf[pos + 5] << 16) |
			                ((uint64_t)buf[pos + 6] << 8) | ((uint64_t)buf[pos + 7]);
			std::memcpy(&time_s, &bits, 8);
			pos += 8;
		}

		std::lock_guard<std::mutex> lock(mutex_);
		TrackerState &t = trackers_[name];
		t.name = name;
		t.hub = extractHub(name);
		t.idx = extractIdx(name);
		Quaternion q{qw, qx, qy, qz};
		t.quat = q;
		t.packets++;
		t.last_update = now;

		// Kinematic ring buffer — the last HIST_N consecutive frames at the native
		// sensor period (the VMC time_s deltas); no separate subsample timescale.
		{
			constexpr int H = TrackerState::HIST_N;
			double t_sec = time_s;
			if (t.qhist_n < H) {
				t.qhist[t.qhist_n] = q;
				t.thist[t.qhist_n] = t_sec;
				t.qhist_n++;
			} else {
				for (int i = 0; i < H - 1; ++i) {
					t.qhist[i] = t.qhist[i + 1];
					t.thist[i] = t.thist[i + 1];
				}
				t.qhist[H - 1] = q;
				t.thist[H - 1] = t_sec;
			}

			// Kinematics on the continuous rotation representation — never narrow to
			// the quaternion (it double-covers SO(3): q and −q are the same rotation,
			// so quaternion differences are discontinuous at the antipode).  Lift
			// each ring sample to a rotation matrix and take angular velocity
			// ω = vee(Ṙ Rᵀ) — the so(3) hat/vee (cross-product) map (Murray–Li–
			// Sastry).  The angular-speed signal σ = |ω| is then read through the
			// proven C⁴ nonic Hermite (spec/Sinew/Nonic.lean ↔ nonic.h), so the
			// reported velocity/accel/jerk are that C⁴ model's derivatives.
			constexpr double kMinDt = 1e-4;  // floor: guard a zero/duplicate dt
			int n = t.qhist_n;
			double R[TrackerState::HIST_N][9];
			for (int i = 0; i < n; ++i) {
				quatToMat(t.qhist[i], R[i]);
			}
			double sig[TrackerState::HIST_N] = {0};  // σ_i = |ω_i| at interior knots
			double dt = kMinDt;
			for (int i = 1; i < n; ++i) {
				double d = t.thist[i] - t.thist[i - 1];
				dt = (d > kMinDt) ? d : kMinDt;
				double w[3];
				angVel(R[i], R[i - 1], dt, w);
				sig[i] = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
			}
			if (n >= 2) {
				t.rot_speed = (float)sig[n - 1];
				// Seed the latest knot's σ-derivatives by backward differences, then
				// route them through the nonic: H'(1)/H''(1)/H'''(1) are exactly these
				// values (proven boundary conditions), so the spline they define is C⁴.
				nonic::Knot k{sig[n - 1], 0, 0, 0, 0};
				if (n >= 3) {
					k.v = (sig[n - 1] - sig[n - 2]) / dt;
				}
				if (n >= 4) {
					k.a = (sig[n - 1] - 2 * sig[n - 2] + sig[n - 3]) / (dt * dt);
				}
				if (n >= 5) {
					k.j = (sig[n - 1] - 3 * sig[n - 2] + 3 * sig[n - 3] - sig[n - 4]) /
					      (dt * dt * dt);
				}
				nonic::Poly H = nonic::hermite(k, k);
				t.rot_vel = (float)nonic::eval(nonic::deriv(H), 1.0);
				t.rot_accel = (float)nonic::eval(nonic::deriv(nonic::deriv(H)), 1.0);
				t.rot_jerk = (float)nonic::eval(nonic::deriv(nonic::deriv(nonic::deriv(H))), 1.0);
			}
		}

		t.hz_window_count++;
		float elapsed = std::chrono::duration<float>(now - t.hz_window_start).count();
		if (elapsed >= 1.0f) {
			t.hz = t.hz_window_count / elapsed;
			t.hz_window_count = 0;
			t.hz_window_start = now;
		}

		int h = t.spark_head;
		t.spark_speed[h] = t.rot_speed;
		t.spark_vel[h] = t.rot_vel;
		t.spark_accel[h] = t.rot_accel;
		t.spark_jerk[h] = t.rot_jerk;
		t.spark_head = (h + 1) % TrackerState::SPARK_N;
		if (t.spark_count < TrackerState::SPARK_N) {
			++t.spark_count;
		}
		return;
	}
}
