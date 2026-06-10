-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee

/-!
# Sinew.Protocol.Lifecycle — per-tracker lifecycle FSM

Specifies the state a single tracker (Rebocap joint 0..14) goes through
on the **device-read side**: what the host sees by observing frames the
dongle emits, with no view of the host's own writes.

The C driver in `tui/src/sinew_driver.c :: SinewJointStateKind` mirrors
this FSM 1-to-1; the TUI footer renders the current state per joint.

## States

* `unknown`   — no frame present for this joint
* `active`    — high-rate IMU frame received (sub-sensor 1)
* `stale`     — retained for TUI display only; reachable via `burstHwid`
                from `active`; unused in C.
* `asleep`    — dongle sent an offline-placeholder frame for this slot.
                The placeholder (bytes 6-13 magic block, see Spec.lean)
                is the dongle's explicit signal that the tracker is off.
* `rejoining` — asleep, then a 0x?A burst frame arrives whose stable hwid
                (frame bytes 24..26) is in `hwid_table.c` for this joint,
                with no high-rate frame yet.  Transient state lasting ~30 ms
                (burst, then high-rate ~39 ms later).

## Transitions

```
            unknown ─── highRate ──────────────────────────────► active
                                                                    ▲
            active  ─── offline ──────────────────────────────► asleep
               │                                                    │
               │   burstHwid                                        │
               └──────────────────────────────────────────► rejoining
                                                                    │
                                                  highRate ─────────┘
            asleep  ─── burstHwid ─────────────────────────► rejoining
```

Notes:
- No timing constants. All transitions are driven by dongle-emitted frames:
  `highRate` → active, `offline` → asleep, `burstHwid` → rejoining.
- There is no aging thread or dt_ms threshold.  The dongle's
  offline-placeholder frames are the authoritative signal for tracker loss.
- `stale` is preserved in the State enum for TUI compatibility but is not
  produced by the C step function.
-/

namespace Sinew.Protocol.Lifecycle

inductive State where
  | unknown
  | active
  | stale
  | asleep
  | rejoining
  deriving Repr, DecidableEq, Inhabited

/-- Frame classification used by the lifecycle step function. -/
inductive Kind where
  | highRate   -- pkt_type lo nibble 8/9/D, sub-sensor 1 locked
  | burstHwid  -- pkt_type lo nibble A, hwid matches this joint
  | offline    -- dongle offline-placeholder frame for this slot
  deriving Repr, DecidableEq

/-- Pure event-driven step — no timing constants.
    Transitions are driven entirely by what the dongle reports. -/
def step (s : State) (ev : Kind) : State :=
  match ev with
  | .highRate  => .active
  | .burstHwid => match s with
                  | .asleep | .stale => .rejoining
                  | _                => s
  | .offline   => .asleep

-- ── Sanity proofs ────────────────────────────────────────────────────────

/-- A high-rate frame always lifts the tracker to `active`,
    regardless of prior state. -/
theorem step_highRate_to_active (s : State) :
    step s .highRate = .active := by
  cases s <;> rfl

/-- A burst frame carrying this tracker's hwid only promotes asleep/stale to rejoining;
    other prior states are unchanged. -/
theorem step_burstHwid_promotes :
    step .asleep .burstHwid = .rejoining ∧
    step .stale  .burstHwid = .rejoining ∧
    step .active .burstHwid = .active := by
  refine ⟨?_, ?_, ?_⟩ <;> rfl

/-- An offline frame always moves the tracker to asleep,
    regardless of prior state. -/
theorem step_offline_to_asleep (s : State) :
    step s .offline = .asleep := by
  cases s <;> rfl

end Sinew.Protocol.Lifecycle
