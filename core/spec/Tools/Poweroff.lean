-- SPDX-License-Identifier: MIT
/-! Sends one shutdown_node (CmdId 0x07) to ALL_TRACKERS (TargetId 0xFF).
    Byte layout follows adapters/sinew_serial_host.c sinew_send_shutdown and the
    Write Frame Layout in Sinew/Protocol/HostCommands.lean. Nothing else is sent. -/

def encodeShutdown (msg : Nat) : ByteArray := Id.run do
  let mut a := ByteArray.mk (Array.replicate 36 0)
  a := a.set! 0 0xEA
  a := a.set! 1 0xEA
  a := a.set! 2 0x07
  a := a.set! 4 0xC8
  a := a.set! 5 0xFF
  a := a.set! 6 (UInt8.ofNat (msg % 256))
  a := a.set! 7 (UInt8.ofNat ((msg / 256) % 128))
  a := a.set! 34 0xAF
  a := a.set! 35 0xAF
  return a

#guard (encodeShutdown 1).size == 36
#guard (encodeShutdown 1)[0]! == 0xEA
#guard (encodeShutdown 1)[2]! == 0x07
#guard (encodeShutdown 1)[4]! == 0xC8
#guard (encodeShutdown 1)[5]! == 0xFF
#guard (encodeShutdown 1)[6]! == 0x01
#guard (encodeShutdown 1)[7]! == 0x00
#guard (encodeShutdown 1)[34]! == 0xAF

def main (args : List String) : IO UInt32 := do
  match args with
  | [dev, "--send"] =>
    let _ ← IO.Process.run { cmd := "stty", args := #["-f", dev, "115200", "raw", "-echo"] }
    let h ← IO.FS.Handle.mk dev .write
    h.write (encodeShutdown 1)
    h.flush
    IO.println "sent shutdown_node 0x07 target=0xFF msg=1 (36 bytes)"
    return 0
  | _ =>
    IO.eprintln "usage: poweroff <dev> --send"
    return 2
