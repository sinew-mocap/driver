// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
// Sinew — merged driver + TUI.  Driver runs in a background thread.
#include "osc_receiver.hpp"
#include "okhsl.hpp"
#include "default_ini.h"

extern "C" {
#include "sinew_driver.h"
#include "hwid_table.h"
}

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <set>
#include <thread>

using namespace ftxui;

// ── OKHSL palette ─────────────────────────────────────────────────────────
// All hues in [0,1] = degrees/360.  Tuned for dark terminal backgrounds.
// Mnemonic: pal::good = sensor working, pal::warn = borderline, pal::err = fault.
namespace pal {
const Color good = okhsl::c(0.400f, 0.85f, 0.68f);      // emerald – active, healthy Hz
const Color warn = okhsl::c(0.145f, 0.88f, 0.70f);      // amber   – stale, moderate ω
const Color err = okhsl::c(0.030f, 0.82f, 0.62f);       // coral   – asleep, artifact
const Color info = okhsl::c(0.540f, 0.75f, 0.68f);      // sky     – ports
const Color accent = okhsl::c(0.120f, 0.72f, 0.78f);    // gold    – joint name labels
const Color hotkey = okhsl::c(0.510f, 0.82f, 0.72f);    // cyan    – keyboard shortcuts
const Color rejoin = okhsl::c(0.830f, 0.78f, 0.65f);    // mauve   – rejoining state
const Color dim = okhsl::c(0.600f, 0.06f, 0.40f);       // slate   – inactive / dim text
const Color muted = okhsl::c(0.600f, 0.04f, 0.58f);     // cool-gr – secondary labels
const Color text_fg = okhsl::c(0.000f, 0.00f, 0.92f);   // white   – primary foreground
const Color kine = okhsl::c(0.510f, 0.65f, 0.65f);      // teal    – kinematic header
const Color title_bg = okhsl::c(0.670f, 0.55f, 0.22f);  // navy    – title bar background
}  // namespace pal

// ── Column widths (one source of truth for header, rows, and sparklines) ─────
// Display columns, not bytes — the degree sign and ω/Δ/superscripts each render
// as one column.  jointRow / sparkRow / tableHeader all derive from these so the
// titles, values, and sparklines stay vertically aligned.
namespace col {
constexpr int label = 24;                                 // "{:<24}"  node + joint name
constexpr int mode = 10;                                  // "{:>8}  " lifecycle MODE
constexpr int ang = 9;                                    // "{:+8.1f}°" roll / pitch / yaw
constexpr int gap = 2;                                    // block separators
constexpr int speed = 6;                                  // "{:6.3f}"  ω
constexpr int vel = 7;                                    // "{:+7.2f}" Δω
constexpr int accel = 8;                                  // "{:+8.1f}" α
constexpr int jerk = 9;                                   // "{:+9.1f}" Δα
constexpr int kine_start = label + mode + ang * 3 + gap;  // first ω column = 63
}  // namespace col

// ── Helpers ───────────────────────────────────────────────────────────────

struct Euler {
	float roll, pitch, yaw;
};

// Tait-Bryan intrinsic X-Y-Z  (= extrinsic Z-Y-X, aerospace yaw-pitch-roll).
// Decomposition: R = Rz(yaw) · Ry(pitch) · Rx(roll)
// Roll  = rotation about X (right)
// Pitch = rotation about Y (forward)
// Yaw   = rotation about Z (up)
// Gimbal lock at pitch = ±90°.
static Euler toEulerIntrinsicXYZ(const Quaternion &q) {
	float sinr = 2.f * (q.w * q.x + q.y * q.z);
	float cosr = 1.f - 2.f * (q.x * q.x + q.y * q.y);
	float roll = std::atan2(sinr, cosr) * (180.f / 3.14159265f);

	float sinp = 2.f * (q.w * q.y - q.z * q.x);
	float pitch = std::fabs(sinp) >= 1.f ? std::copysign(90.f, sinp)
	                                     : std::asin(sinp) * (180.f / 3.14159265f);

	float siny = 2.f * (q.w * q.z + q.x * q.y);
	float cosy = 1.f - 2.f * (q.y * q.y + q.z * q.z);
	float yaw = std::atan2(siny, cosy) * (180.f / 3.14159265f);

	return {roll, pitch, yaw};
}

// The driver resolves joints via NodeNumber (byte 6) and emits the
// Rebocap joint name directly as the OSC channel.  jointForChannel
// just validates that the name is one of the 15 known joints.
static int nodeNumberForJoint(const std::string &joint);  // defined below

static std::string jointForChannel(const std::string &name) {
	if (nodeNumberForJoint(name) >= 0) {
		return name;
	}
	return std::string();
}

// Canonical Rebocap node number (0..14) for a Rebocap joint name.
// Returns -1 if the joint isn't one of the 15 documented body parts.
// Displayed next to the joint label so testers can call out numbers
// instead of full names ("power on 8" rather than "power on Head").
static int nodeNumberForJoint(const std::string &joint) {
	static const std::unordered_map<std::string, int> table = {
	    {"Hips", 0},           {"LeftUpperLeg", 1}, {"RightUpperLeg", 2},  {"LeftLowerLeg", 3},
	    {"RightLowerLeg", 4},  {"LeftFoot", 5},     {"RightFoot", 6},      {"Chest", 7},
	    {"Head", 8},           {"LeftUpperArm", 9}, {"RightUpperArm", 10}, {"LeftLowerArm", 11},
	    {"RightLowerArm", 12}, {"LeftHand", 13},    {"RightHand", 14},
	};
	return table.count(joint) ? table.at(joint) : -1;
}

// Lifecycle FSM state → short column label + colour.  The authoritative
// per-joint mode comes from the driver (SinewJointStateKind, mirrors
// Lifecycle.lean :: State) — not a wall-clock rate/age heuristic.
static std::pair<const char *, Color> modeLabel(SinewJointStateKind state) {
	switch (state) {
		case SINEW_JOINT_ACTIVE:
			return {"ACTIVE", pal::good};
		case SINEW_JOINT_REJOINING:
			return {"WAKING", pal::rejoin};
		case SINEW_JOINT_STALE:
			return {"stale", pal::warn};
		case SINEW_JOINT_ASLEEP:
			return {"ASLEEP", pal::dim};
		default:
			return {"---", pal::dim};
	}
}

// ── Sparkline helpers ─────────────────────────────────────────────────────

static std::string sparkline(const float *buf, int head, int count, int N, float lo, float hi) {
	static const char *blocks[] = {"▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"};
	std::string s;
	s.reserve(N * 3);
	// Always render N chars: pad left with ▁ when buffer isn't full yet
	int pad = N - count;
	for (int i = 0; i < pad; ++i) {
		s += "▁";
	}
	int start = (head - count + N) % N;
	for (int i = 0; i < count; ++i) {
		float v = buf[(start + i) % N];
		float t = (hi > lo) ? (v - lo) / (hi - lo) : 0.f;
		t = std::clamp(t, 0.f, 1.f);
		s += blocks[(int)(t * 7.f + 0.5f)];
	}
	return s;
}

static Element sparkRow(const TrackerState &t) {
	int n = t.spark_count, h = t.spark_head, N = TrackerState::SPARK_N;

	std::function<float(const float *)> absmax = [&](const float *a) {
		float m = 0.001f;
		for (int i = 0; i < n; ++i) {
			m = std::max(m, std::fabs(a[(h - n + N + i) % N]));
		}
		return m;
	};
	float mw = absmax(t.spark_speed);
	float mv = absmax(t.spark_vel);
	float ma = absmax(t.spark_accel);
	float mj = absmax(t.spark_jerk);

	// Each sparkline width = its data column's width in jointRow (col::), so the
	// sparkline sits directly under the value it describes.
	std::function<std::string(const float *, float, int)> sp = [&](const float *a, float mx,
	                                                               int w) {
		return sparkline(a, h, std::min(n, w), w, -mx, mx);
	};

	Color cw = (mw > 1.0f) ? pal::err : (mw > 0.05f) ? pal::warn : pal::good;
	Color cv = (mv > 10.f) ? pal::err : (mv > 1.0f) ? pal::warn : pal::good;
	Color ca = (ma > 100.f) ? pal::err : (ma > 10.f) ? pal::warn : pal::good;
	Color cj = (mj > 1000.f) ? pal::err : (mj > 100.f) ? pal::warn : pal::good;

	// Indent past label + MODE + roll/pitch/yaw + the block gap to the ω column,
	// then lay sparklines under each kinematic value, separated like rot_elem.
	return hbox({
	    text(std::string(col::kine_start, ' ')),
	    text(sp(t.spark_speed, mw, col::speed)) | color(cw),
	    text(" "),
	    text(sp(t.spark_vel, mv, col::vel)) | color(cv),
	    text(" "),
	    text(sp(t.spark_accel, ma, col::accel)) | color(ca),
	    text(" "),
	    text(sp(t.spark_jerk, mj, col::jerk)) | color(cj),
	});
}

// ── One row per joint ──────────────────────────────────────────────────────
//
// Single renderer driven by the driver's lifecycle FSM state.  The MODE column
// always shows js.state (ACTIVE / WAKING / stale / ASLEEP / ---).  When `live`,
// the full kinematic row is drawn from `data`; otherwise the last-known
// orientation (if any) is shown dimmed so VR poses don't teleport on reconnect.
static Element jointRow(const SinewJointState &js, const TrackerState *data, bool live) {
	std::string label = std::format("{:>2}  {}", js.node_num, js.joint);
	std::pair<const char *, Color> mode = modeLabel(js.state);
	std::string mode_field = std::format("{:>8}  ", mode.first);  // col::mode (10)

	Elements cols;
	cols.push_back(text(std::format("{:<24}", label)) | bold |
	               color(live ? pal::accent : pal::dim));
	cols.push_back(text(mode_field) | color(mode.second));

	if (!data) {  // never seen — label + mode only
		cols.push_back(filler());
		return hbox(std::move(cols));
	}

	Euler e = toEulerIntrinsicXYZ(data->quat);
	if (!live) {  // last-known orientation, dimmed, no kinematics
		cols.push_back(text(std::format("{:+8.1f}°", e.roll)) | color(pal::dim));
		cols.push_back(text(std::format("{:+8.1f}°", e.pitch)) | color(pal::dim));
		cols.push_back(text(std::format("{:+8.1f}°", e.yaw)) | color(pal::dim));
		cols.push_back(filler());
		return hbox(std::move(cols));
	}

	const TrackerState &t = *data;

	// Colour by physical magnitude (time-normalised units).
	// Red = interpolation artifact / code error only.
	// All physically plausible human motion stays green or amber.
	// Peak human joint angular velocity ≈ 15-20 rad/s (fast throw/swing).
	Color (*wColor)(float) = [](float v) {  // ω  rad/s
		float a = std::fabs(v);
		if (a < 0.1f) {
			return pal::dim;
		}
		if (a < 10.f) {
			return pal::good;
		}
		if (a < 20.f) {
			return pal::warn;
		}
		return pal::err;  // > 20 rad/s: physically impossible
	};
	Color (*dvColor)(float) = [](float v) {  // Δω rad/s²
		float a = std::fabs(v);
		if (a < 1.f) {
			return pal::dim;
		}
		if (a < 200.f) {
			return pal::good;
		}
		if (a < 500.f) {
			return pal::warn;
		}
		return pal::err;
	};
	Color (*acColor)(float) = [](float v) {  // α  rad/s³
		float a = std::fabs(v);
		if (a < 10.f) {
			return pal::dim;
		}
		if (a < 2000.f) {
			return pal::good;
		}
		if (a < 5000.f) {
			return pal::warn;
		}
		return pal::err;
	};
	Color (*jkColor)(float) = [](float v) {  // Δα rad/s⁴
		float a = std::fabs(v);
		if (a < 100.f) {
			return pal::dim;
		}
		if (a < 20000.f) {
			return pal::good;
		}
		if (a < 50000.f) {
			return pal::warn;
		}
		return pal::err;
	};
	Element rot_elem = hbox({
	    text(std::format("{:6.3f}", t.rot_speed)) | color(wColor(t.rot_speed)),
	    text(" "),
	    text(std::format("{:+7.2f}", t.rot_vel)) | color(dvColor(t.rot_vel)),
	    text(" "),
	    text(std::format("{:+8.1f}", t.rot_accel)) | color(acColor(t.rot_accel)),
	    text(" "),
	    text(std::format("{:+9.1f}", t.rot_jerk)) | color(jkColor(t.rot_jerk)),
	});

	cols.push_back(text(std::format("{:+8.1f}°", e.roll)));
	cols.push_back(text(std::format("{:+8.1f}°", e.pitch)));
	cols.push_back(text(std::format("{:+8.1f}°", e.yaw)));
	cols.push_back(text("  "));
	cols.push_back(rot_elem);
	return hbox(std::move(cols));
}

static Element tableHeader() {
	// Each segment is padded to its col:: width (display columns) so the titles
	// sit directly over the values in jointRow and the sparklines in sparkRow.
	return hbox({
	    text(" # JOINT" + std::string(col::label - 8, ' ')) | color(pal::accent) | bold,
	    text("    MODE  ") | color(pal::good) | bold,                    // 10 = col::mode
	    text("     Roll    Pitch      Yaw") | color(pal::muted) | bold,  // 27 = 3×col::ang
	    text(std::string(col::gap, ' ')),
	    text(" ω r/s Δω r/s²   α r/s³   Δα r/s⁴") | color(pal::kine) | bold,  // kinematics block
	});
}

// ── Pairing helpers (called from the pairing thread) ─────────────────────

enum class PairState { Probe, Paired, NeedsAddr, Pairing, Done, Fail };

static const char *pair_label(PairState s) {
	switch (s) {
		case PairState::Probe:
			return "PROBE";
		case PairState::Paired:
			return "PAIRED";
		case PairState::NeedsAddr:
			return "NEEDS_ADDR";
		case PairState::Pairing:
			return "PAIRING";
		case PairState::Done:
			return "DONE";
		case PairState::Fail:
			return "FAIL";
	}
	return "?";
}

// Poll sinew_get_joint_states until node is ACTIVE, or `ms` elapses.
static bool joint_is_active(int node, int ms) {
	std::chrono::steady_clock::time_point deadline =
	    std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
	SinewJointState js[SINEW_JOINT_COUNT];
	while (std::chrono::steady_clock::now() < deadline) {
		sinew_get_joint_states(js, SINEW_JOINT_COUNT);
		if (js[node].state == SINEW_JOINT_ACTIVE) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return false;
}

// ── Main ──────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
#ifdef _WIN32
	// Enforce UTF-8 console I/O so em-dashes, °, ω, Δ, sparkline blocks etc.
	// render correctly instead of mojibake under the Windows-1252 default.
	SetConsoleOutputCP(CP_UTF8);
	SetConsoleCP(CP_UTF8);
#endif

	uint16_t app_port = 39539;      // consumer app: where the driver sends (stable)
	uint16_t monitor_port = 39540;  // TUI display: driver mirrors here; bound passively

	for (int i = 1; i < argc; ++i) {
		if (std::string(argv[i]) == "--version" || std::string(argv[i]) == "-v") {
			std::puts("sinew_tui " SINEW_VERSION);
			return 0;
		}
		if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
			std::puts(
			    "Usage: sinew_tui [appPort] [monitorPort] [options]\n"
			    "\n"
			    "  appPort           OSC consumer port — where the driver sends,\n"
			    "                    independent of the TUI       (default 39539)\n"
			    "  monitorPort       OSC mirror the TUI displays   (default 39540)\n"
			    "\n"
			    "Options:\n"
			    "  --port DEV        Force serial port (default: probe ports)\n"
			    "  --ip IP           OSC destination IP    (default 127.0.0.1)\n"
			    "  --headless        Run as a service: no TUI, just stream OSC to\n"
			    "                    --ip:appPort until SIGINT/SIGTERM\n"
			    "  --verbose         Log per-frame diagnostics\n"
			    "  --raw-log PATH    Dump raw 36-byte frames as ms,hex72\\n\n"
			    "\n"
			    "Mode nudges (fired ~2.8s after connect):\n"
			    "  --rgb RRGGBB      Set tracker LED colour (confirmed)\n"
			    "  --mag N           Magnetic resistance, 1..12 (confirmed; needs [mag] cal)\n"
			    "  --antimag on|off  Anti-magnetic toggle (confirmed, 0xa1)\n"
			    "  --tx N            Signal emission power, 1..18 (confirmed, 0x91)\n"
			    "\n"
			    "Kit data files (searched in ./ ../ ../../):\n"
			    "  hwid_table.ini    hwid → joint mapping  (SINEW_HWID_FILE)\n"
			    "  activates.ini     activate_node + [mag] cal payloads (SINEW_ACTIVATES_FILE)\n");
			return 0;
		}
	}

	// Driver config: parse --port, --ip, --verbose before TUI args
	SinewConfig drv;
	sinew_config_default(&drv);
	std::string raw_log_path;
	// Named mode-nudge handles (--rgb/--mag/--tx) queue a host command fired
	// ~2.8 s after connect (once streaming).  No raw-command escape hatch:
	// every handle uses a known opcode with a doc-clamped value range.
	struct PendingCmd {
		uint8_t op, target;
		std::vector<uint8_t> payload;
	};
	std::vector<PendingCmd> pending_cmds;
	int pending_mag_level = -1;  // --mag: fired via sinew_send_mag_strength (cal-replay)
	bool headless = false;       // --headless: run as a service, no TUI (see below)
	for (int i = 1; i < argc; ++i) {
		if (std::string(argv[i]) == "--port" && i + 1 < argc) {
			strncpy(drv.serial_port, argv[++i], sizeof(drv.serial_port) - 1);
		} else if (std::string(argv[i]) == "--headless") {
			headless = true;
		} else if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
			strncpy(drv.dest_ip, argv[++i], sizeof(drv.dest_ip) - 1);
		} else if (std::string(argv[i]) == "--verbose") {
			drv.verbose = 1;
		} else if (std::string(argv[i]) == "--raw-log" && i + 1 < argc) {
			raw_log_path = argv[++i];
		} else if (std::string(argv[i]) == "--rgb" && i + 1 < argc) {
			// set_rgb 0x41 (docs/host-commands.md): RGB sits at payload bytes
			// 31/32/33 (= payload offsets 19/20/21), and one frame is sent per
			// node 0x00..0x0E, not a single 0xFF broadcast.
			long v = strtol(argv[++i], nullptr, 16);
			std::vector<uint8_t> p(22, 0);
			p[19] = (uint8_t)(v >> 16);  // R
			p[20] = (uint8_t)(v >> 8);   // G
			p[21] = (uint8_t)v;          // B
			for (uint8_t node = 0x00; node <= 0x0E; ++node) {
				pending_cmds.push_back({0x41, node, p});
			}
		} else if (std::string(argv[i]) == "--mag" && i + 1 < argc) {
			// set_anti_magnetic_strength 0x31 → 1..12.  Fired per-node via
			// sinew_send_mag_strength, which replays each tracker's captured
			// magnetometer calibration and overwrites only the level byte
			// (mag table in activates.ini [mag]; see docs/host-commands.md).
			pending_mag_level = atoi(argv[++i]);
		} else if (std::string(argv[i]) == "--tx" && i + 1 < argc) {
			// set_transmit_power 0x91: level at payload byte 13 (frame byte 25),
			// encoded inverted as (28 - N) for N = 1..18 (18 = +9.1 dBm).
			// Per-node burst, clean single-value payload (no bundled
			// calibration, unlike 0x31).
			int n = atoi(argv[++i]);
			n = n < 1 ? 1 : n > 18 ? 18 : n;
			std::vector<uint8_t> p(22, 0);
			p[13] = (uint8_t)(28 - n);
			for (uint8_t node = 0x00; node <= 0x0E; ++node) {
				pending_cmds.push_back({0x91, node, p});
			}
		} else if (std::string(argv[i]) == "--antimag" && i + 1 < argc) {
			// anti_magnetic_toggle 0xa1: state at payload byte 12 — 0x00 =
			// anti-mag ON, 0x0a = anti-mag OFF.  Sent per-node; the official
			// app then cycles the trackers (shutdown/re-activate).  The
			// anti-magnetic switch is a SEPARATE control from 6/9-axis mode
			// (whose opcode is uncaptured).
			std::string v = argv[++i];
			uint8_t state = (v == "on" || v == "1") ? 0x00 : 0x0a;
			std::vector<uint8_t> p(22, 0);
			p[12] = state;
			for (uint8_t node = 0x00; node <= 0x0E; ++node) {
				pending_cmds.push_back({0xa1, node, p});
			}
		} else if (i == 1) {
			int v = atoi(argv[i]);
			if (v > 0) {
				app_port = (uint16_t)v;
			}
		} else if (i == 2) {
			int v = atoi(argv[i]);
			if (v > 0) {
				monitor_port = (uint16_t)v;
			}
		}
	}
	drv.osc_port = app_port;
	drv.monitor_port = monitor_port;

	// Open log file before starting driver so driver output goes there, not stderr
	std::string log_path = (std::filesystem::temp_directory_path() / "sinew_tui.log").string();
	FILE *drv_log = fopen(log_path.c_str(), "w");
	SinewConfig drv_with_log = drv;
	drv_with_log.logfp = drv_log;
	FILE *raw_log = nullptr;
	if (!raw_log_path.empty()) {
		raw_log = fopen(raw_log_path.c_str(), "w");
		drv_with_log.rawfp = raw_log;
	}

	// Start the C driver in a background thread.  It sends OSC to the consumer
	// app (app_port) and mirrors to monitor_port; the receiver below only taps
	// the mirror, so it's never in the app's data path.
	std::thread driver_thread([drv_with_log]() { sinew_driver_run(&drv_with_log); });

	// Fire any --send commands ~2.8 s after start (connect + wake + activate done).
	if (!pending_cmds.empty() || pending_mag_level >= 0) {
		std::thread([pending_cmds, pending_mag_level]() {
			std::this_thread::sleep_for(std::chrono::milliseconds(2800));
			for (const PendingCmd &c : pending_cmds) {
				sinew_send_command(c.op, c.target, 1,
				                   c.payload.empty() ? nullptr : c.payload.data(),
				                   (int)c.payload.size());
				std::this_thread::sleep_for(std::chrono::milliseconds(150));
			}
			// --mag: per-node 0x31, replaying captured calibration with only the
			// level byte changed.  Nodes without a captured payload are skipped
			// (sinew_send_mag_strength refuses them) so a tracker's cal is never zeroed.
			if (pending_mag_level >= 0) {
				for (uint8_t node = 0x00; node <= 0x0E; ++node) {
					sinew_send_mag_strength(node, pending_mag_level);
					std::this_thread::sleep_for(std::chrono::milliseconds(150));
				}
			}
		}).detach();
	}

	OscReceiver receiver(monitor_port);
	receiver.start();

	ScreenInteractive screen = ScreenInteractive::Fullscreen();

	// Once a channel has been seen, keep it on screen for ten minutes even if
	// it goes silent — idle is a normal recoverable state, not a reason
	// to hide the row.  The renderer marks stale rows ASLEEP so the operator
	// knows which tracker to wake.
	constexpr float ACTIVE_WINDOW = 600.0f;

	// Log to file so it doesn't interfere with the fullscreen TUI.
	std::ofstream logFile(log_path, std::ios::out | std::ios::trunc);
	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	std::function<double()> elapsedSec = [&]() -> double {
		return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	};
	logFile << std::format("[0.0s] Sinew TUI  app={} monitor={}\n", app_port, monitor_port);
	logFile.flush();

	std::set<std::string> knownTrackers;

	std::atomic<bool> alive{true};
	std::thread refresher([&] {
		int tick = 0;
		while (alive) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			++tick;

			if (tick % 20 == 0) {
				receiver.purgeOlderThan(ACTIVE_WINDOW);
			}

			// Lifecycle + periodic summary (both use the same snapshot)
			if (tick % 4 == 0) {  // check every 200 ms — cheap enough
				std::unordered_map<std::string, TrackerState> snap = receiver.snapshot();
				double now = elapsedSec();

				std::set<std::string> current;
				for (std::unordered_map<std::string, TrackerState>::const_iterator e = snap.begin();
				     e != snap.end(); ++e) {
					if (e->second.hz > 0.f) {
						current.insert(e->first);
					}
				}

				for (const std::string &k : current) {
					if (!knownTrackers.count(k)) {
						logFile << std::format("[{:.1f}s] NEW  {}\n", now, k);
						logFile.flush();
					}
				}
				for (const std::string &k : knownTrackers) {
					if (!current.count(k)) {
						logFile << std::format("[{:.1f}s] GONE {}\n", now, k);
						logFile.flush();
					}
				}
				knownTrackers = current;

				// Periodic summary every 100 ticks (~5 s)
				if (tick % 100 == 0 && !current.empty()) {
					logFile << std::format("[{:.1f}s] --- {} active ---\n", now, current.size());
					for (const std::string &k : current) {
						TrackerState &t = snap.at(k);
						logFile << std::format("  {:<20} hz={:5.1f}  ω={:.4f}  Δω={:+.4f}\n", k,
						                       t.hz, t.rot_speed, t.rot_vel);
					}
					logFile.flush();
				}
			}

			screen.PostEvent(Event::Custom);
		}
	});

	// Shutdown / activate command state.  Same picker pattern for both:
	// press the lead key ('s' or 'a') → next single key picks a node
	// ('0'..'9' / 'a'..'e' for nodes 0..14, lead-key-uppercase for the
	// collective form — 'S' = ALL_TRACKERS shutdown, 'A' = activate every
	// joint that has a saved payload in the loaded table).  `*_status`
	// is a one-line transient message that surfaces in the footer for
	// ~3 s after a send.
	static const char *JOINT_NAMES[15] = {
	    "Hips",          "LeftUpperLeg", "RightUpperLeg", "LeftLowerLeg", "RightLowerLeg",
	    "LeftFoot",      "RightFoot",    "Chest",         "Head",         "LeftUpperArm",
	    "RightUpperArm", "LeftLowerArm", "RightLowerArm", "LeftHand",     "RightHand"};

	bool shutdown_pending = false;
	bool pair_pending = false;
	std::string action_status;
	std::chrono::steady_clock::time_point action_status_until;

	// activate_all: fire activate_node for every joint, use stored payload
	// if available else zeros (zeros work for already-paired trackers).
	std::function<void()> activate_all = [&]() {
		static constexpr uint8_t kZeros22[22]{};
		int sent = 0;
		for (uint8_t node = 0; node <= 14; ++node) {
			if (sinew_have_activate_for(node)) {
				sent += sinew_send_activate_known(node);
			} else {
				sent += sinew_send_activate(node, kZeros22);
			}
		}
		action_status = std::format("activate all → sent {}/15", sent);
		action_status_until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
		screen.PostEvent(Event::Custom);
	};

	// Pairing state machine — runs in a background thread so the TUI
	// stays live.  pair_msg is written from the thread and read from the
	// renderer; both accesses are under pair_mu.
	// pair_running is managed by the thread wrapper, not by run_pair.
	std::mutex pair_mu;
	std::string pair_msg;
	std::atomic<bool> pair_running{false};

	// run_pair: drive the SM for one node; updates pair_msg and posts
	// a redraw event on each state transition.  Returns the final state.
	std::function<PairState(int)> run_pair = [&](int node) -> PairState {
		static constexpr uint8_t zeros22[22]{};
		PairState state = PairState::Probe;

		sinew_send_wake_up();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		for (;;) {
			{
				std::lock_guard<std::mutex> lk(pair_mu);
				pair_msg = std::format("[{}] {}", pair_label(state), JOINT_NAMES[node]);
			}
			screen.PostEvent(Event::Custom);

			switch (state) {
				case PairState::Probe:
					sinew_send_activate((uint8_t)node, zeros22);
					state = joint_is_active(node, 3000) ? PairState::Paired : PairState::NeedsAddr;
					break;
				case PairState::Paired: {
					std::lock_guard<std::mutex> lk(pair_mu);
					pair_msg = std::format("[PAIRED] {}", JOINT_NAMES[node]);
				}
					screen.PostEvent(Event::Custom);
					return PairState::Paired;
				case PairState::NeedsAddr: {
					std::lock_guard<std::mutex> lk(pair_mu);
					pair_msg = std::format(
					    "[NEEDS_ADDR] {} (node {}) — tracker never paired; BLE addr required",
					    JOINT_NAMES[node], node);
				}
					screen.PostEvent(Event::Custom);
					return PairState::NeedsAddr;
				case PairState::Pairing:
				case PairState::Done:
				case PairState::Fail:
					return state;
			}
		}
	};

	// Unpack default INI files if not present in the working directory.
	for (std::pair<const char *, const char *> ini :
	     {std::make_pair("hwid_table.ini", kDefaultHwidIni),
	      std::make_pair("activates.ini", kDefaultActivatesIni)}) {
		if (!std::filesystem::exists(ini.first)) {
			std::ofstream f(ini.first);
			if (f) {
				f << ini.second;
			}
		}
	}

	// Load kit-specific data files (hwid_table.ini, activates.ini).
	{
		std::string (*find_ini)(const char *, std::initializer_list<const char *>) =
		    [](const char *env_var, std::initializer_list<const char *> names) {
			    std::string path;
			    if (const char *env = std::getenv(env_var)) {
				    path = env;
			    } else {
				    for (const char *c : names) {
					    if (std::filesystem::exists(c)) {
						    path = c;
						    break;
					    }
				    }
			    }
			    return path;
		    };

		std::string p = find_ini("SINEW_HWID_FILE",
		                         {"hwid_table.ini", "../hwid_table.ini", "../../hwid_table.ini"});
		if (!p.empty()) {
			int n = sinew_hwid_load(p.c_str());
			std::ofstream(log_path, std::ios::app)
			    << std::format("[0.0s] Loaded {} hwid entries from {}\n", n, p);
		}

		p = find_ini("SINEW_ACTIVATES_FILE",
		             {"activates.ini", "../activates.ini", "../../activates.ini"});
		if (!p.empty()) {
			int n = sinew_load_activate_table(p.c_str());
			std::ofstream(log_path, std::ios::app)
			    << std::format("[0.0s] Loaded {} activate payloads from {}\n", n, p);
			// The mag-strength calibration table lives in the same file ([mag]).
			int m = sinew_load_mag_table(p.c_str());
			std::ofstream(log_path, std::ios::app)
			    << std::format("[0.0s] Loaded {} mag-cal payloads from {}\n", m, p);
		}
	}

	// Headless service mode: the C driver thread (started above) is the whole
	// data path — it reads the dongle and emits OSC to app_port@dest_ip on its
	// own.  No FTXUI screen and no monitor receiver; activate the trackers once,
	// then block until systemd asks us to stop (SIGTERM) or Ctrl-C (SIGINT).
	if (headless) {
		// Stop the TUI-only helpers that were started above (the monitor tap and
		// the redraw ticker); the driver thread keeps streaming OSC untouched.
		alive = false;
		refresher.join();
		receiver.stop();

		std::thread([&]() {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			static constexpr uint8_t kZeros22[22]{};
			for (uint8_t node = 0; node <= 14; ++node) {
				if (sinew_have_activate_for(node)) {
					sinew_send_activate_known(node);
				} else {
					sinew_send_activate(node, kZeros22);
				}
			}
		}).detach();

		static std::atomic<bool> run{true};
		std::signal(SIGINT, [](int) { run = false; });
		std::signal(SIGTERM, [](int) { run = false; });
		while (run.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}

		sinew_driver_stop();
		driver_thread.join();
		if (drv_log) {
			fclose(drv_log);
		}
		if (raw_log) {
			fclose(raw_log);
		}
		return 0;
	}

	// Last known TrackerState per joint — persisted across frames so standby
	// rows can show the last seen orientation instead of a blank placeholder.
	std::unordered_map<std::string, TrackerState> last_seen;

	Component renderer = Renderer([&] {
		std::unordered_map<std::string, TrackerState> trackers = receiver.snapshot();
		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

		// One row per joint: show the highest-hz sub-sensor channel.
		// Active-rate (≥30 Hz) wins over burst; burst shows if no active
		// Build a joint-name → best TrackerState map from the OSC snapshot.
		// The driver emits joint names directly, so the channel key IS the joint.
		std::unordered_map<std::string, const TrackerState *> live;
		for (const std::pair<const std::string, TrackerState> &kv : trackers) {
			float age = std::chrono::duration<float>(now - kv.second.last_update).count();
			if (kv.second.hz <= 0.f || age >= ACTIVE_WINDOW) {
				continue;
			}
			if (jointForChannel(kv.first).empty()) {
				continue;
			}
			// Prefer highest-hz (active ~100 Hz beats standby ~5 Hz).
			std::unordered_map<std::string, const TrackerState *>::iterator it =
			    live.find(kv.first);
			if (it == live.end() || kv.second.hz > it->second->hz) {
				live[kv.first] = &kv.second;
				last_seen[kv.first] = kv.second;  // persist for standby display
			}
		}

		// Always show all 15 joints.  Active = has live OSC data.
		// Standby = no OSC data; show lifecycle state as a placeholder.
		SinewJointState js[SINEW_JOINT_COUNT];
		int n_js = sinew_get_joint_states(js, SINEW_JOINT_COUNT);
		int active = (int)live.size();

		Elements rows;
		rows.push_back(tableHeader());
		rows.push_back(separator());
		for (int i = 0; i < n_js; ++i) {
			const std::string joint_name(js[i].joint);
			std::unordered_map<std::string, const TrackerState *>::const_iterator it =
			    live.find(joint_name);
			const TrackerState *liveData = (it != live.end()) ? it->second : nullptr;
			std::unordered_map<std::string, TrackerState>::const_iterator ls =
			    last_seen.find(joint_name);
			const TrackerState *prev = (ls != last_seen.end()) ? &ls->second : nullptr;

			// Row presentation is driven by the FSM state: ACTIVE (with live OSC
			// data) draws the full kinematic row; any other state shows the
			// last-known orientation dimmed.  MODE is shown for every joint.
			bool active = (js[i].state == SINEW_JOINT_ACTIVE) && liveData != nullptr;
			const TrackerState *data = liveData ? liveData : prev;
			rows.push_back(jointRow(js[i], data, active));
			rows.push_back(active ? sparkRow(*data) : text(""));
		}
		Element body = vbox(std::move(rows)) | flex;
		int j_active = 0, j_stale = 0, j_asleep = 0, j_rejoin = 0, j_unknown = 0;
		std::string asleep_list, rejoin_list;
		for (int i = 0; i < n_js; ++i) {
			switch (js[i].state) {
				case SINEW_JOINT_ACTIVE:
					j_active++;
					break;
				case SINEW_JOINT_STALE:
					j_stale++;
					break;
				case SINEW_JOINT_REJOINING:
					j_rejoin++;
					if (!rejoin_list.empty()) {
						rejoin_list += " ";
					}
					rejoin_list += js[i].joint;
					break;
				case SINEW_JOINT_ASLEEP:
					j_asleep++;
					if (!asleep_list.empty()) {
						asleep_list += " ";
					}
					asleep_list += js[i].joint;
					break;
				default:
					j_unknown++;
					break;
			}
		}

		return vbox({
		    hbox({
		        text(" ◈ Sinew ") | bold | color(pal::text_fg),
		        text(std::format("app:{} mon:{}", app_port, monitor_port)) | color(pal::info),
		        text("  "),
		        text(std::format("{}/{}", active, SINEW_JOINT_COUNT)) | bold |
		            color(active == SINEW_JOINT_COUNT ? pal::good
		                  : active > 0                ? pal::warn
		                                              : pal::dim),
		        text(" joints") | color(pal::muted),
		        text("  "),
		        text(std::format("+{:.0f}s", elapsedSec())) | color(pal::dim),
		        filler(),
		        text(" q:quit ") | color(pal::muted),
		    }) | bgcolor(pal::title_bg),
		    separator(),
		    body,
		    separator(),
		    hbox({
		        text(std::format(" joints: {} active", j_active)) |
		            color(j_active > 0 ? pal::good : pal::dim),
		        text(std::format("  {} stale", j_stale)) |
		            color(j_stale > 0 ? pal::warn : pal::dim),
		        text(std::format("  {} rejoin", j_rejoin)) |
		            color(j_rejoin > 0 ? pal::rejoin : pal::dim),
		        text(std::format("  {} asleep", j_asleep)) |
		            color(j_asleep > 0 ? pal::err : pal::dim),
		        text(std::format("  {} unseen", j_unknown)) | color(pal::dim),
		        text(rejoin_list.empty() ? std::string() : "   REJOIN: " + rejoin_list) | bold |
		            color(pal::rejoin),
		        text(asleep_list.empty() ? std::string() : "   ASLEEP: " + asleep_list) | bold |
		            color(pal::err),
		        filler(),
		    }),
		    // Bottom prompt line: picker / pairing status / action status / hotkeys
		    [&]() -> Element {
			    if (shutdown_pending) {
				    return hbox({
				        text(" SHUTDOWN ▶ ") | bold | color(Color::Black) | bgcolor(Color::Red),
				        text(" pick node: ") | color(pal::warn),
				        text("0-9 / a-e") | bold | color(pal::text_fg),
				        text(" = node 0..14, ") | color(pal::dim),
				        text("S") | bold | color(pal::err),
				        text(" = ALL_TRACKERS, ") | color(pal::dim),
				        text("esc") | color(pal::text_fg),
				        text(" = cancel") | color(pal::dim),
				        filler(),
				    });
			    }
			    if (pair_pending) {
				    return hbox({
				        text(" PAIR ▶ ") | bold | color(Color::Black) | bgcolor(Color::Cyan),
				        text(" pick node: ") | color(pal::warn),
				        text("0-9 / a-e") | bold | color(pal::text_fg),
				        text(" = node 0..14, ") | color(pal::dim),
				        text("P") | bold | color(pal::hotkey),
				        text(" = ALL,  ") | color(pal::dim),
				        text("esc") | color(pal::text_fg),
				        text(" = cancel") | color(pal::dim),
				        filler(),
				    });
			    }
			    // Pairing in progress — show live state
			    std::string pm;
			    {
				    std::lock_guard<std::mutex> lk(pair_mu);
				    pm = pair_msg;
			    }
			    if (pair_running || !pm.empty()) {
				    return hbox({
				        text(" PAIR ") | bold | color(Color::Black) | bgcolor(Color::Cyan),
				        text(" " + pm + " ") | color(pal::hotkey),
				        filler(),
				    });
			    }
			    if (now < action_status_until) {
				    return hbox({
				        text(" " + action_status + " ") | bold | color(Color::Black) |
				            bgcolor(Color::Yellow),
				        filler(),
				    });
			    }
			    return hbox({
			        text(" "),
			        text("[a]") | bold | color(pal::hotkey),
			        text(" activate all  ") | color(pal::dim),
			        text("[p]") | bold | color(pal::hotkey),
			        text(" pair  ") | color(pal::dim),
			        text("[s]") | bold | color(pal::hotkey),
			        text(" shutdown  ") | color(pal::dim),
			        text("[q]") | bold | color(pal::hotkey),
			        text(" quit") | color(pal::dim),
			        filler(),
			    });
		    }(),
		});
	});

	Component component = CatchEvent(renderer, [&](Event e) {
		// Generic node picker — pulls 0..14 / 'S'/'A' (collective) / esc.
		// Returns -2 for "collective", -1 for "cancel", node# 0..14 otherwise.
		int (*pick_node)(Event, char) = [](Event e, char collective_key) -> int {
			if (e == Event::Escape) {
				return -1;
			}
			if (!e.is_character()) {
				return -1;
			}
			std::string ch = e.character();
			if (ch.empty()) {
				return -1;
			}
			char c = ch[0];
			if (c >= '0' && c <= '9') {
				return c - '0';
			}
			if (c >= 'a' && c <= 'e') {
				return 10 + (c - 'a');
			}
			if (c == collective_key) {
				return -2;
			}
			return -1;
		};

		// Shutdown picker
		if (shutdown_pending) {
			if (!e.is_character() && e != Event::Escape) {
				return false;
			}
			shutdown_pending = false;
			int p = pick_node(e, 'S');
			if (p == -1) {
				action_status = "shutdown cancelled";
			} else if (p == -2) {
				int ok = sinew_send_shutdown(SINEW_NODE_ALL_TRACKERS);
				action_status =
				    ok ? "shutdown sent → ALL_TRACKERS (0xFF)" : "shutdown FAILED → ALL_TRACKERS";
			} else {
				int ok = sinew_send_shutdown((uint8_t)p);
				action_status =
				    (ok ? std::string("shutdown sent → ") : std::string("shutdown FAILED → ")) +
				    JOINT_NAMES[p];
			}
			action_status_until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
			return true;
		}

		// Pair picker
		if (pair_pending) {
			if (!e.is_character() && e != Event::Escape) {
				return false;
			}
			pair_pending = false;
			int p = pick_node(e, 'P');
			if (p == -1) {
				return true;  // cancelled
			}
			if (pair_running.exchange(true)) {
				return true;  // already running
			}
			if (p >= 0) {
				std::thread([&, p] {
					run_pair(p);
					pair_running = false;
				}).detach();
			} else {
				// p == -2: pair all nodes sequentially
				std::thread([&] {
					for (int n = 0; n < SINEW_JOINT_COUNT; ++n) {
						PairState result = run_pair(n);
						if (result == PairState::NeedsAddr) {
							break;  // stop at first unpaired
						}
					}
					pair_running = false;
				}).detach();
			}
			return true;
		}

		if (e == Event::Character('s')) {
			shutdown_pending = true;
			return true;
		}
		if (e == Event::Character('a')) {
			activate_all();
			return true;
		}
		if (e == Event::Character('p')) {
			if (!pair_running) {
				pair_pending = true;
			}
			return true;
		}
		if (e == Event::Character('q') || e == Event::Escape) {
			screen.ExitLoopClosure()();
			return true;
		}
		return false;
	});

	// Auto-activate all joints 1 s after startup — gives the driver time to
	// open the serial port and send the initial wake_up before the flood.
	std::thread([&]() {
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		activate_all();
	}).detach();

	screen.Loop(component);

	alive = false;
	refresher.join();
	receiver.stop();
	sinew_driver_stop();
	driver_thread.join();
	return 0;
}
