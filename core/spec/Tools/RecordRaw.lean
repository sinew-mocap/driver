-- SPDX-License-Identifier: MIT
import Sinew.Protocol
/-! Raw recorder in Lean 4. Reads the dongle port passively (never writes to the
    dongle), keeps every received byte in raw_stream.bin, and frames the stream
    with an explicit +36 continuity rule into frames.log (ms,hex72). The framing
    is pure and is checked by #guard at build time. -/
open Sinew.Protocol

def syncAt (b : ByteArray) (i : Nat) : Bool :=
  i + 4 ≤ b.size && b[i]! == SYNC_0 && b[i+1]! == SYNC_1 &&
    b[i+2]! == SYNC_2 && b[i+3]! == SYNC_3

def findFrom (b : ByteArray) (i : Nat) : Option Nat := Id.run do
  let mut j := i
  while j + 4 ≤ b.size do
    if syncAt b j then return some j
    j := j + 1
  return none

/-- Consume whole frames. Returns the frames, the number of resyncs (bytes skipped
    to reach a sync), and the unconsumed tail to prepend to the next chunk. -/
def consume (buf : ByteArray) : Array ByteArray × Nat × ByteArray := Id.run do
  let mut i := 0
  let mut out : Array ByteArray := #[]
  let mut rs := 0
  while true do
    if syncAt buf i then
      if i + FRAME_BYTES ≤ buf.size then
        out := out.push (buf.extract i (i + FRAME_BYTES))
        i := i + FRAME_BYTES
      else
        break
    else if i + 4 > buf.size then
      break
    else
      match findFrom buf (i + 1) with
      | some j =>
        rs := rs + 1
        i := j
      | none =>
        rs := rs + 1
        i := buf.size - 3
        break
  return (out, rs, buf.extract i buf.size)

def mkFrame (fill : UInt8) : ByteArray :=
  ByteArray.mk (#[SYNC_0, SYNC_1, SYNC_2, SYNC_3] ++ (List.replicate 32 fill).toArray)

def cat (xs : List ByteArray) : ByteArray := xs.foldl (fun a b => a ++ b) ByteArray.empty

-- Build-time guards: a broken framer fails `lake build`.
#guard (consume (cat [mkFrame 0, mkFrame 0])).1.size == 2
#guard (consume (cat [mkFrame 0, mkFrame 0])).2.1 == 0
#guard (consume (cat [ByteArray.mk #[1, 2, 3, 4, 5], mkFrame 0])).1.size == 1
#guard (consume (cat [ByteArray.mk #[1, 2, 3, 4, 5], mkFrame 0])).2.1 == 1
#guard (consume (mkFrame 7 |>.extract 0 20)).1.size == 0
#guard (consume (cat [(mkFrame 7).extract 0 20, (mkFrame 7).extract 20 36])).1.size == 1

def hex2 (n : UInt8) : String :=
  let s := String.ofList (Nat.toDigits 16 n.toNat)
  if s.length < 2 then "0" ++ s else s

def toHex (bs : ByteArray) : String :=
  (List.range bs.size).foldl (fun acc i => acc ++ hex2 bs[i]!) ""

def main (args : List String) : IO UInt32 := do
  match args with
  | [dev, secsStr, dir] =>
    let secs := secsStr.toNat!
    IO.FS.createDirAll dir
    let _ ← IO.Process.run { cmd := "stty", args := #["-f", dev, "115200", "raw", "-echo"] }
    let h ← IO.FS.Handle.mk dev .read
    let rawH ← IO.FS.Handle.mk (dir ++ "/raw_stream.bin") .write
    let fo ← IO.FS.Handle.mk (dir ++ "/frames.log") .write
    let t0 ← IO.monoMsNow
    let deadline := t0 + secs * 1000
    let mut carry := ByteArray.empty
    let mut ok := 0
    let mut rs := 0
    let mut total := 0
    let mut lastFlush := t0
    repeat
      let now ← IO.monoMsNow
      if now ≥ deadline then break
      let chunk ← h.read 4096
      if chunk.size == 0 then break
      rawH.write chunk
      total := total + chunk.size
      let (frames, r, tail) := consume (carry ++ chunk)
      carry := tail
      rs := rs + r
      let stamp ← IO.monoMsNow
      for f in frames do
        fo.putStrLn s!"{stamp - t0},{toHex f}"
        ok := ok + 1
      if stamp - lastFlush ≥ 1000 then
        rawH.flush
        fo.flush
        lastFlush := stamp
    rawH.flush
    fo.flush
    let fin ← IO.monoMsNow
    IO.FS.writeFile (dir ++ "/summary.txt")
      s!"frames {ok}\nresyncs {rs}\nbytes {total}\nelapsed_ms {fin - t0}\n"
    IO.println s!"frames {ok} resyncs {rs} bytes {total}"
    return 0
  | _ =>
    IO.eprintln "usage: recordraw <dev> <seconds> <outdir>"
    return 2
