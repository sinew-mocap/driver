# Host → dongle commands & mode nudges

How `sinew` sends commands to the dongle, which are wired to a CLI handle, and
how to capture undecoded ones.

> **Wire format is specified once, in the spec — not here.** The host-write frame
> layout, the CmdId table, the per-command payload byte offsets
> (`0x41` set_rgb, `0x21` gyro cal, `0xa1` anti-mag, `0x31` mag strength, `0x91`
> TX power, `0x13` comm channel), and the firmware-update flow all live in
> [`spec/Sinew/Protocol/HostCommands.lean`](../spec/Sinew/Protocol/HostCommands.lean).
> This doc covers the *driver/CLI* side and the capture procedure; it points
> to the spec for every byte.

The frame envelope is implemented once in `sinew_send_command()`
(`src/sinew_driver.c`); the typed wrappers (`sinew_send_wake_up`,
`sinew_send_activate`, `sinew_send_shutdown`, `sinew_send_mag_strength`, …) and
the CLI nudges build on it. Each config command goes out as a per-node burst
(`0x00`–`0x0E`) with the work bit set.

## Mode-nudge CLI

`sinew_tui` fires one config command ~2.8 s after connect (once streaming).
Doc-clamped, named handles only — no raw-command escape hatch. Byte offsets and
encodings are in the spec; the operational behaviour is:

| Handle | CmdId | Effect | Notes |
|---|---|---|---|
| `--rgb RRGGBB` | `0x41` | set LED colour | per-node burst |
| `--mag N` (1..12) | `0x31` | anti-magnetic *strength* | **replays** the node's captured `[mag]` cal from `activates.ini` and rewrites only the level byte; nodes with no captured payload are **skipped** (a zeroed `0x31` would destroy the tracker's mag cal) |
| `--antimag on\|off` | `0xa1` | anti-magnetic *switch* | NOT the 6-/9-axis selector |
| `--tx N` (1..18) | `0x91` | transmit power | clean single-value, no bundled cal |

There is **no `--axis` handle**: 6-/9-axis mode is applied host-side in Rebocap's
fusion and emits zero host writes (every other control goes through the hooked
`WriteFile`, this one sends nothing). There is no command to land; 9-axis is
implemented driver-side instead (next section).

`0x13` (comm channel) is structurally decoded but deliberately **not wired to a
handle** — it is a link-wide RF setting that can disrupt pairing.

## 9-axis fusion (driver-side)

No dongle command selects 6-/9-axis mode, so 9-axis is implemented in the driver.
The tracker streams its onboard 6-axis quaternion (gyro+accel: good pitch/roll,
free-running yaw → the per-limb yaw drift in the viz); the raw magnetometer is
in every frame (bytes 24–29, *standard layout only* — see
[`spec/.../Type0x18.lean`](../spec/Sinew/Protocol/Type0x18.lean)).

The fusion **method, the solver comparison, and the shipped filter are specified in the spec** —
[`spec/Sinew/Fusion.lean`](../spec/Sinew/Fusion.lean). The solvers are scored by the C driver.

The **complementary filter (TRIAD + onboard-quat delta, α≈0.10)** is ported into
`src/sinew_driver.c`, which emits the corrected quat on `/sinew/tracker`. The viz
T-pose calibration absorbs each tracker's constant hard-iron heading offset, so
the magnetometer only needs to stop yaw from *wandering*, not to be absolutely
accurate.

## Getting payloads for an undecoded command — capture the official app

The deterministic method that produces each payload in the spec: capture the
official Rebocap Windows app issuing the command, then read the bytes. The
Detours-based hook is at `tools/host_capture/`
(`git show fe04a02^:tools/host_capture/README.md`). A working tree is at
`E:\sinew-host-capture`.

1. Dongle on Windows; run the official Rebocap software with `host_capture`
   injected (it hooks `WriteFile`/`ReadFile`, logging each frame with a `W`/`R`
   direction column).
2. In the config UI perform ONE clear action (change LED colour, set magnetic
   resistance, …).
3. Filter the log to `,W,` lines and decode byte 2 (opcode), byte 5 (target),
   bytes 12–33 (payload). Record the layout in
   `spec/Sinew/Protocol/HostCommands.lean`.
4. Update the wrapper in `src/sinew_driver.c` to match.

## Scope note: there is no "rate" mode

The config UI and the opcode space hold **no frame-rate / update-rate /
sub-sensor-selection command**. The per-joint rate ceiling (~60 % of the 62.5 Hz
TDMA slot rate) is fixed by dongle firmware, not unlockable by any host command —
corroborated by the datasheets in `docs/datasheets/` (IMU 200 Hz, radio
1–2 Mbit/s GFSK, USB ~50× headroom; the limit is the firmware TDMA airtime
schedule). The only product-tier lever Rebocap documents is a separate "commercial
version."
