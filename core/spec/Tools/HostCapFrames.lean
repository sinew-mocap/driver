-- SPDX-License-Identifier: MIT
import Sinew.Protocol
/-! Host-capture CSV to frames. Input rows are `ms,DIR,hex`.
    R rows are USB read chunks: their hex is concatenated in file order and framed on the
    fafaaeae sync with the same +36 rule as RecordRaw. Each frame takes the ms of the chunk
    that holds its first byte.
    W rows are host writes (EA EA ... AF AF): one complete 36-byte frame per row, shape-checked.
    Output TSV: dir, frame_ord, abs_off, chunk_ms, chunk_line_or_src_line, hex72. -/
open Sinew.Protocol

def hexNibble (c : Char) : Option Nat :=
  if '0' ≤ c && c ≤ '9' then some (c.toNat - '0'.toNat)
  else if 'a' ≤ c && c ≤ 'f' then some (c.toNat - 'a'.toNat + 10)
  else if 'A' ≤ c && c ≤ 'F' then some (c.toNat - 'A'.toNat + 10)
  else none

def parseHex (s : String) : Option ByteArray := Id.run do
  let cs := s.toList
  if cs.length % 2 != 0 then return none
  let mut out : ByteArray := ByteArray.empty
  let mut i := 0
  while i < cs.length do
    match hexNibble cs[i]!, hexNibble cs[i+1]! with
    | some hi, some lo => out := out.push (UInt8.ofNat (hi * 16 + lo))
    | _, _ => return none
    i := i + 2
  return some out

def syncAt (b : ByteArray) (i : Nat) : Bool :=
  i + 4 ≤ b.size && b[i]! == SYNC_0 && b[i+1]! == SYNC_1 &&
    b[i+2]! == SYNC_2 && b[i+3]! == SYNC_3

def findFrom (b : ByteArray) (i : Nat) : Option Nat := Id.run do
  let mut j := i
  while j + 4 ≤ b.size do
    if syncAt b j then return some j
    j := j + 1
  return none

/-- Frame starts in `buf`, resync count, and the index where the unconsumed tail begins. -/
def consumeStarts (buf : ByteArray) : Array Nat × Nat × Nat := Id.run do
  let mut i := 0
  let mut starts : Array Nat := #[]
  let mut rs := 0
  while true do
    if syncAt buf i then
      if i + FRAME_BYTES ≤ buf.size then
        starts := starts.push i
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
  return (starts, rs, i)

/-- A host write is one complete 36-byte frame: EA EA ... AF AF. -/
def isHostWrite (bs : ByteArray) : Bool :=
  bs.size == FRAME_BYTES && bs[0]! == 0xEA && bs[1]! == 0xEA && bs[34]! == 0xAF && bs[35]! == 0xAF

def mkFrame (fill : UInt8) : ByteArray :=
  ByteArray.mk (#[SYNC_0, SYNC_1, SYNC_2, SYNC_3] ++ (List.replicate 32 fill).toArray)

def cat (xs : List ByteArray) : ByteArray := xs.foldl (fun a b => a ++ b) ByteArray.empty

def hostWriteFrame : ByteArray :=
  ByteArray.mk (#[0xEA, 0xEA] ++ (List.replicate 32 0).toArray ++ #[0xAF, 0xAF])

/-- Sync with byte 3 corrupted: no frame may be emitted from it. -/
def badSyncFrame : ByteArray :=
  ByteArray.mk (#[SYNC_0, SYNC_1, SYNC_2, 0x00] ++ (List.replicate 32 0).toArray)

-- Build-time guards: a broken framer or shape check fails `lake build`.
#guard (consumeStarts (cat [mkFrame 0, mkFrame 0])).1.size == 2
#guard (consumeStarts (cat [mkFrame 0, mkFrame 0])).2.1 == 0
#guard (consumeStarts (cat [ByteArray.mk #[1, 2, 3, 4, 5], mkFrame 0])).1.size == 1
#guard (consumeStarts (cat [ByteArray.mk #[1, 2, 3, 4, 5], mkFrame 0])).2.1 == 1

/-- Cross-chunk: a frame split 20/16 across two reads yields nothing after the
    first read, then one frame once the carried tail is joined to the second read. -/
def crossChunkOk : Bool :=
  let a := (mkFrame 7).extract 0 20
  let b := (mkFrame 7).extract 20 36
  let (s1, _, t1) := consumeStarts a
  let (s2, _, _) := consumeStarts (a.extract t1 a.size ++ b)
  s1.size == 0 && s2.size == 1
#guard crossChunkOk

#guard isHostWrite hostWriteFrame
#guard !isHostWrite (hostWriteFrame.set! 35 0x00)
-- Planted control: the corrupted sync must produce 0 starts.
#guard (consumeStarts (cat [badSyncFrame])).1.size == 0

def hex2 (n : UInt8) : String :=
  let s := String.ofList (Nat.toDigits 16 n.toNat)
  if s.length < 2 then "0" ++ s else s

def toHex (bs : ByteArray) : String :=
  (List.range bs.size).foldl (fun acc i => acc ++ hex2 bs[i]!) ""

def main (args : List String) : IO UInt32 := do
  match args with
  | [src, dst] => do
    let h ← IO.FS.Handle.mk src .read
    let out ← IO.FS.Handle.mk dst .write
    out.putStrLn "dir\tframe_ord\tabs_off\tchunk_ms\tsrc_line\thex72"
    let mut carry := ByteArray.empty
    let mut base := 0
    let mut total := 0
    let mut lineNo := 0
    let mut frames := 0
    let mut resyncs := 0
    let mut chunks : Array (Nat × Nat × Nat) := #[]
    let mut ptr := 0
    let mut wRows := 0
    let mut wOk := 0
    let mut wBad := 0
    let mut rBad := 0
    repeat
      let line ← h.getLine
      if line.isEmpty then break
      lineNo := lineNo + 1
      let t := line.trimAscii.toString
      if t.isEmpty || t.startsWith "#" then continue
      match t.splitOn "," with
      | [msS, dir, hexS] =>
        let ms := msS.toNat?.getD 0
        if dir == "R" then
          match parseHex hexS with
          | none => rBad := rBad + 1
          | some bytes =>
            chunks := chunks.push (total, ms, lineNo)
            total := total + bytes.size
            carry := carry ++ bytes
            let (starts, rs, tailIdx) := consumeStarts carry
            resyncs := resyncs + rs
            for st in starts do
              let abs := base + st
              while ptr + 1 < chunks.size && chunks[ptr+1]!.1 ≤ abs do
                ptr := ptr + 1
              let (_, cms, cl) := chunks[ptr]!
              let f := carry.extract st (st + FRAME_BYTES)
              out.putStrLn s!"R\t{frames}\t{abs}\t{cms}\t{cl}\t{toHex f}"
              frames := frames + 1
            carry := carry.extract tailIdx carry.size
            base := base + tailIdx
        else if dir == "W" then
          wRows := wRows + 1
          match parseHex hexS with
          | some bs =>
            if isHostWrite bs then
              out.putStrLn s!"W\t{wOk}\t-1\t{ms}\t{lineNo}\t{toHex bs}"
              wOk := wOk + 1
            else
              wBad := wBad + 1
          | none => wBad := wBad + 1
        else
          pure ()
      | _ => pure ()
    out.flush
    let skipped := total - frames * FRAME_BYTES - carry.size
    IO.println s!"R: bytes={total} frames={frames} resyncs={resyncs} skipped_bytes={skipped} tail_bytes={carry.size} unparsable_rows={rBad}"
    IO.println s!"W: rows={wRows} ok={wOk} bad_shape={wBad}"
    return 0
  | _ => do
    IO.eprintln "usage: hostcapframes <csv> <out.tsv>"
    return 2
