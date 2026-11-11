-- SPDX-License-Identifier: MIT
-- SinewDriver — the REBOCAP cluster's spec: the serial frame codec
-- (Protocol{,.Spec,.HostCommands,.Channels,.Type0x18,.Type0x3a,.Lifecycle}), the
-- serial framing (Serial), the 9-axis spline (Nonic), and the /sinew OSC contract
-- it emits (Osc, Udp — folded in, as the driver is the only encoder consumer).
-- Independent (no deps).
import Lake
open Lake DSL

-- Property-based testing: Plausible (QuickCheck for Lean4), pinned to our toolchain.
require plausible from git "https://github.com/leanprover-community/plausible" @ "v4.30.0-rc2"

package "SinewDriver" where
  version := v!"0.1.0"

lean_lib SinewDriver where
  srcDir := "."
  globs  := #[Glob.andSubmodules `Sinew.Protocol, Glob.one `Sinew.Serial,
              Glob.one `Sinew.Nonic, Glob.one `Sinew.Osc, Glob.one `Sinew.Udp,
              Glob.one `Sinew.MagCal, Glob.one `Sinew.Rot3]

-- Property tests over the codec spec.  `lake exe proptest` exits nonzero on a
-- counterexample (CI-runnable).
lean_exe proptest where
  root := `Tests

-- Thin raw-log decoder over Sinew.Protocol.parseFrame (no decode logic here).
lean_exe rawdecode where
  root := `Tools.RawDecode

-- Raw byte-stream recorder with +36 framing (Tools.RecordRaw).
lean_exe recordraw where
  root := `Tools.RecordRaw

-- One ALL_TRACKERS shutdown_node frame (Tools.Poweroff).
lean_exe poweroff where
  root := `Tools.Poweroff

-- Per-capture mean pose in 3x3 matrix space (Tools.PoseAngle).
lean_exe poseangle where
  root := `Tools.PoseAngle

-- Host-capture CSV to frames (Tools.HostCapFrames).
lean_exe hostcapframes where
  root := `Tools.HostCapFrames
