// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

struct Quaternion {
	float w = 1, x = 0, y = 0, z = 0;
};

struct TrackerState {
	std::string name;
	std::string hub;
	uint8_t idx = 0;
	Quaternion quat;
	uint64_t packets = 0;
	float hz = 0.f;
	std::chrono::steady_clock::time_point last_update;
	uint64_t hz_window_count = 0;
	std::chrono::steady_clock::time_point hz_window_start;

	// Rotational kinematics — ω = vee(Ṙ Rᵀ) on the rotation matrix (no quaternion
	// narrowing), its derivatives read through the proven C⁴ nonic Hermite
	// (spec/Sinew/Nonic.lean ↔ src/nonic.h).
	float rot_speed = 0;  // ω   rad/s   angular speed
	float rot_vel = 0;    // Δω  rad/s²  angular acceleration
	float rot_accel = 0;  // α   rad/s³  angular jerk
	float rot_jerk = 0;   // Δα  rad/s⁴  angular snap

	// Kinematic ring buffer — the last HIST_N consecutive frames at the native
	// sensor period (VMC time_s deltas), for ω = vee(Ṙ Rᵀ) + the nonic interpolant.
	static constexpr int HIST_N = 5;
	Quaternion qhist[HIST_N]{};
	double thist[HIST_N]{};  // seconds since driver start (double)
	int qhist_n = 0;

	// Sparkline history — last 40 samples per signal
	static constexpr int SPARK_N = 40;
	float spark_speed[SPARK_N]{};  // ω
	float spark_vel[SPARK_N]{};    // Δω
	float spark_accel[SPARK_N]{};  // α
	float spark_jerk[SPARK_N]{};   // Δα
	int spark_head = 0;
	int spark_count = 0;
};

class OscReceiver {
   public:
	// port: monitor port the driver mirrors to; bound passively for display.
	// The driver fans out at the source, so this receiver is a pure observer
	// and is never in the consumer app's data path.
	explicit OscReceiver(uint16_t port = 39540);
	~OscReceiver();

	void start();
	void stop();

	std::unordered_map<std::string, TrackerState> snapshot() const;
	void purgeOlderThan(float seconds);
	uint16_t port() const {
		return port_;
	}

   private:
	void run();
	void process(const uint8_t *buf, size_t len);

	static size_t align4(size_t n) {
		return (n + 3) & ~3u;
	}
	static std::string readString(const uint8_t *buf, size_t len, size_t &pos);
	static float readFloat(const uint8_t *buf, size_t len, size_t &pos);
	static int32_t readInt32(const uint8_t *buf, size_t len, size_t &pos);

	uint16_t port_;
	// intptr_t holds an int FD on POSIX or a SOCKET (UINT_PTR) on Windows.
	// -1 = unopened.
	std::intptr_t sock_ = -1;
	std::atomic<bool> running_{false};
	std::thread thread_;
	mutable std::mutex mutex_;
	std::unordered_map<std::string, TrackerState> trackers_;
};
