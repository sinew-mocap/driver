-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee
/-
  Sinew.Udp — Lean4 FFI bindings for UDP socket helpers.
-/

namespace Sinew.Udp

structure Socket where
  fd : Int32
  deriving Repr

@[extern "sinew_udp_open"]
opaque udpOpenRaw : IO Int32

@[extern "sinew_udp_send"]
opaque udpSendRaw (fd : Int32) (ip : @&String) (port : UInt16)
                  (data : @&ByteArray) (len : UInt32) : IO Int32

@[extern "sinew_udp_close"]
opaque udpCloseRaw (fd : Int32) : IO Unit

-- ── Safe wrappers ─────────────────────────────────────────────────────────

def mkSocket : IO Socket := do
  let fd ← udpOpenRaw
  if fd < 0 then
    throw (IO.userError s!"Failed to open UDP socket: errno {-fd}")
  return { fd }

def send (s : Socket) (ip : String) (port : UInt16) (pkt : ByteArray) : IO Unit := do
  let r ← udpSendRaw s.fd ip port pkt (UInt32.ofNat pkt.size)
  if r < 0 then
    throw (IO.userError s!"UDP send error: errno {-r}")

def closeSocket (s : Socket) : IO Unit :=
  udpCloseRaw s.fd

end Sinew.Udp
