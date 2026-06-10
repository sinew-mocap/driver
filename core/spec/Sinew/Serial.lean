-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee
/-
  Sinew.Serial — Lean4 FFI bindings for the serial port C helpers.
-/

namespace Sinew.Serial

structure Handle where
  fd : Int32
  deriving Repr

@[extern "sinew_serial_open"]
opaque serialOpenRaw (path : @&String) : IO Int32

-- Read one valid 36-byte frame with limited sync recovery.
-- Fast path: if the first 4 bytes are FA FA AE AE the frame is returned immediately.
-- Recovery path: if sync is lost, scans up to 512 bytes byte-by-byte for the next
-- FA FA AE AE boundary.  If no sync is found within that window the function returns
-- an empty ByteArray, signalling disconnect; the outer loop reconnects.
-- 512 bytes ≈ 14 frames — short enough that a genuinely desynced stream reconnects
-- quickly rather than blocking the reader for tens of seconds.
@[extern "sinew_serial_read_frame"]
opaque serialReadFrameRaw (fd : Int32) : IO ByteArray

-- Write bytes to the serial port. BORROWED ByteArray.
-- Returns true on success.
@[extern "sinew_serial_write"]
opaque serialWriteRaw (fd : Int32) (buf : @&ByteArray) : IO Bool

@[extern "sinew_serial_close"]
opaque serialCloseRaw (fd : Int32) : IO Unit

-- ── Safe wrappers (no exceptions) ────────────────────────────────────────

/-- Open the serial port.  Returns `none` on failure (logs errno). -/
def openHandle (path : String) : IO (Option Handle) := do
  let fd ← serialOpenRaw path
  if fd < 0 then
    IO.eprintln s!"Cannot open {path} (errno {-fd})"
    return none
  return some { fd }

/-- Read one self-aligned 36-byte frame.  Returns empty ByteArray on disconnect. -/
def readFrame (h : Handle) : IO ByteArray :=
  serialReadFrameRaw h.fd

def writeBytes (h : Handle) (buf : ByteArray) : IO Bool :=
  serialWriteRaw h.fd buf

def closeHandle (h : Handle) : IO Unit :=
  serialCloseRaw h.fd

end Sinew.Serial
