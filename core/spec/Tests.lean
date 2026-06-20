-- SPDX-License-Identifier: MIT
-- Property tests over the rebocap codec spec.
-- • Standard Plausible (#eval) properties run at build time — build fails on a counterexample.
-- • PlausibleWitnessDag checks run in main — `lake exe proptest` exits nonzero if a witness
--   search fails.
import Plausible
import PlausibleWitnessDag
import Sinew.Protocol
open Plausible Sinew.Protocol PlausibleWitnessDag

-- ── Build-time (#eval) Plausible properties ───────────────────────────────

-- Harness smoke check (native Nat generator).
#eval Testable.check (∀ n : Nat, n < n + 1)

-- Codec property: a frame shorter than FRAME_BYTES never decodes to .ok.
#eval Testable.check (∀ (l : List UInt8),
  l.length < FRAME_BYTES → ¬ (parseFrame ⟨l.toArray⟩ matches .ok _))

-- ── PlausibleWitnessDag runtime properties ────────────────────────────────

-- Build a minimal 36-byte 0x18 IMU frame with a specific quat-W i16 seed.
-- All other fields are zero except sync, pkt_type, and trackerIdx.
private def buildSeedFrame (qw : UInt16) : ByteArray :=
  ByteArray.mk #[
    0xFA, 0xFA, 0xAE, 0xAE,           -- sync bytes 0-3
    0x18,                              -- pkt_type (lower nibble 8 = IMU)
    0x00, 0x00, 0x00,                  -- sub_type, seqnum lo, hi
    0x00, 0x00,                        -- status
    (qw &&& 0xFF).toUInt8, (qw >>> 8).toUInt8,  -- quat_w at bytes 10-11
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  -- quat_xyz bytes 12-17
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  -- accel bytes 18-23
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  -- bytes 24-29
    0x31, 0x32, 0x33,                  -- hub ASCII "123" bytes 30-32
    0x01,                              -- trackerIdx != 0 → standard layout; bytes 33-35
    0x00, 0x00 ]

-- Witness search: there exists a seed in [0, 4096) such that the synthetic frame
-- parses as .ok (any non-zero quat-W normalises; the witness DAG finds it quickly).
def checkValidFrameWitness : IO Bool := do
  let (_, lvl, trace) ← resolve
    "valid-imu-frame-exists"
    (fun _lvl n => (parseFrame (buildSeedFrame n.toUInt16)) matches .ok _)
    (fun _ =>
      let found := (parseFrame (buildSeedFrame 0x4000)) matches .ok _
      { value := 0x4000, found, budgetHit := false })
  IO.println s!"[proptest] {trace.query}: {repr trace.outcome} (level {lvl})"
  return trace.outcome matches .found _

def main : IO Unit := do
  IO.println "proptest: #eval properties ran at build time; running witness-DAG checks…"
  let ok ← checkValidFrameWitness
  if ¬ ok then
    IO.println "proptest: FAIL — valid-imu-frame-exists witness not found"
    IO.Process.exit 1
  IO.println "proptest: all checks passed"
