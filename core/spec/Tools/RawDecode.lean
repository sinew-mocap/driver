-- SPDX-License-Identifier: MIT
import Sinew.Protocol
/-! Thin driver over `Sinew.Protocol.parseFrame`: reads `ms,hex72` raw-log lines
    (upper or lower case hex, `ms,` prefix optional) and prints one result per line.
    No decoding logic lives here; every branch is the spec's `parseFrame`. -/
open Sinew.Protocol

def hexNibble (c : Char) : Option Nat :=
  if '0' ≤ c && c ≤ '9' then some (c.toNat - '0'.toNat)
  else if 'a' ≤ c && c ≤ 'f' then some (c.toNat - 'a'.toNat + 10)
  else if 'A' ≤ c && c ≤ 'F' then some (c.toNat - 'A'.toNat + 10)
  else none

/-- Hex payload after the last comma; none if any character is not hex or the length is odd. -/
def parseHexLine (line : String) : Option ByteArray := Id.run do
  let payload := (line.splitOn ",").getLast!.trimAscii.toString
  let cs := payload.toList
  if cs.length % 2 != 0 then return none
  let mut out : ByteArray := ByteArray.empty
  let mut i := 0
  while i < cs.length do
    match hexNibble cs[i]!, hexNibble cs[i+1]! with
    | some hi, some lo => out := out.push (UInt8.ofNat (hi * 16 + lo))
    | _, _ => return none
    i := i + 2
  return some out

def hex2 (n : UInt8) : String :=
  let s := String.mk (Nat.toDigits 16 n.toNat)
  if s.length < 2 then "0" ++ s else s

def showResult : ParseResult → String
  | .ok f =>
      s!"ok hub={f.hubId} idx={f.trackerIdx} pkt=0x{hex2 f.pktType} seq={f.seqNum} status={f.status} accelRaw={f.accelRaw}"
  | .skipped k => s!"skipped pkt=0x{hex2 k}"
  | .badSync => "badSync"
  | .tooShort => "tooShort"
  | .offline => "offline"

def main (args : List String) : IO UInt32 := do
  match args with
  | [path] =>
    let text ← IO.FS.readFile path
    for line in text.splitOn "\n" do
      if line.trimAscii.toString.isEmpty then continue
      match parseHexLine line with
      | some bs => IO.println (showResult (parseFrame bs))
      | none => IO.println "unparsable"
    return 0
  | _ =>
    IO.eprintln "usage: rawdecode <ms,hex72 file>"
    return 2
