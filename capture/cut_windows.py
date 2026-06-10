"""Cut the valid on-body windows out of a sinew raw log.

A window starts when the trackers are donned + calibrated (T/S/B poses) and ENDS
when the trackers come off the body — which shows up as a long dead-still stretch
(trackers set on a table/floor stop moving entirely), as opposed to the short
held stills of a calibration pose.  So:

  off-body  = a run of low-motion bins lasting >= OFF_SEC (trackers not on a body)
  window    = an active span between off-body runs (>= MIN_WIN long)

Two passes over the log: pass 1 classifies time bins, pass 2 writes each window's
frames to <base>.winK_<start>s_<dur>s.log.

    python driver/capture/cut_windows.py <raw.log>
"""
import math
import os
import sys

SYNC = "fafaaeae"
BIN_MS = 250
STILL = 0.05    # rad/tracker per bin → "not moving"
OFF_SEC = 20.0  # sustained stillness this long ⇒ off the body
MIN_WIN = 5.0   # discard windows shorter than this


def i16(b, i):
    v = b[i] | (b[i + 1] << 8)
    return v - 65536 if v >= 32768 else v


def frame_node_quat(hx):
    if len(hx) < 72 or hx[:8] != SYNC:
        return None
    b = bytes.fromhex(hx[:72])
    if (b[4] & 0xF) not in (8, 9, 13) or b[6] >= 15 or b[33] == 0:
        return None
    q = [i16(b, 10), i16(b, 12), i16(b, 14), i16(b, 16)]
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return b[6], [c / n for c in q]


def parse(line):
    c = line.find(",")
    if c < 0:
        return None
    try:
        ms = int(line[:c])
    except ValueError:
        return None
    d = frame_node_quat(line[c + 1:].rstrip("\n").split(",")[-1])
    return (ms, d) if d else None


def main():
    path = sys.argv[1]
    base = os.path.splitext(path)[0]

    # ── pass 1: per-bin motion ───────────────────────────────────────────────
    prevq = {}
    binmot = {}   # bin -> total per-tracker rotation
    bintrk = {}   # bin -> set of nodes
    with open(path, errors="ignore") as f:
        for line in f:
            r = parse(line)
            if not r:
                continue
            ms, (node, q) = r
            bk = ms // BIN_MS
            bintrk.setdefault(bk, set()).add(node)
            if node in prevq:
                dot = abs(sum(q[k] * prevq[node][k] for k in range(4)))
                binmot[bk] = binmot.get(bk, 0.0) + 2.0 * math.acos(max(-1.0, min(1.0, dot)))
            prevq[node] = q
    bins = sorted(bintrk)
    if not bins:
        print(f"{path}: no frames")
        return

    # still per bin (low motion per active tracker)
    still = {bk: (binmot.get(bk, 0.0) / max(1, len(bintrk[bk]))) < STILL for bk in bins}
    off_run = max(1, int(OFF_SEC * 1000 / BIN_MS))

    # mark bins inside a long still run as off-body
    offbody = {bk: False for bk in bins}
    i = 0
    while i < len(bins):
        if still[bins[i]]:
            j = i
            while j < len(bins) and still[bins[j]]:
                j += 1
            if (j - i) >= off_run:
                for k in range(i, j):
                    offbody[bins[k]] = True
            i = j
        else:
            i += 1

    # windows = maximal runs of on-body bins
    wins = []
    i = 0
    while i < len(bins):
        if not offbody[bins[i]]:
            j = i
            while j < len(bins) and not offbody[bins[j]]:
                j += 1
            t0, t1 = bins[i] * BIN_MS, bins[j - 1] * BIN_MS + BIN_MS
            if (t1 - t0) / 1000.0 >= MIN_WIN:
                wins.append((t0, t1))
            i = j
        else:
            i += 1

    if not wins:
        print(f"{path}: no on-body windows")
        return
    t_base = bins[0] * BIN_MS
    print(f"{path}: {len(wins)} window(s):")
    for k, (t0, t1) in enumerate(wins):
        print(f"  win{k}: {(t0 - t_base) / 1000:.0f}s .. {(t1 - t_base) / 1000:.0f}s  "
              f"({(t1 - t0) / 1000:.0f}s)")

    # ── pass 2: write each window's frames ───────────────────────────────────
    outs = [open(f"{base}.win{k}_{int((t0 - t_base) / 1000)}s_{int((t1 - t0) / 1000)}s.log", "w")
            for k, (t0, t1) in enumerate(wins)]
    with open(path, errors="ignore") as f:
        for line in f:
            c = line.find(",")
            if c < 0:
                continue
            try:
                ms = int(line[:c])
            except ValueError:
                continue
            for (t0, t1), o in zip(wins, outs):
                if t0 <= ms < t1:
                    o.write(line if line.endswith("\n") else line + "\n")
                    break
    for o in outs:
        o.close()
    print(f"wrote {len(wins)} window clip(s) next to {base}")


if __name__ == "__main__":
    main()
