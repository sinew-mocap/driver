"""Find calibration windows in a sinew raw log.

Calibration = the wearer holding a short sequence of *distinct, still, full-body
poses* (T-pose, S-pose, B-pose).  We bin frames by time, measure per-bin body
motion (sum of per-tracker frame-to-frame rotation) and how many trackers are
active, then flag "held poses" = runs that are still, full-body, and not on the
floor.  A run of >= 2 distinct held poses is a calibration sequence.

On-floor sections are excluded as held poses by the distinctness + an upright
check (worn trackers point in diverse directions; trackers flat on the floor all
read gravity straight down their shared axis, so their orientations cluster).

    python driver/capture/detect_calib.py <raw.log>            # <ms>,<72hex> driver format
"""
import math
import sys

SYNC = "fafaaeae"
BIN_MS = 250  # 0.25 s bins


def i16(b, i):
    v = b[i] | (b[i + 1] << 8)
    return v - 65536 if v >= 32768 else v


def decode(hx):
    """Return (node, unit-quat (w,x,y,z), accel-dir) or None for a 72-hex frame."""
    if len(hx) < 72 or hx[:8] != SYNC:
        return None
    b = bytes.fromhex(hx[:72])
    if (b[4] & 0xF) not in (8, 9, 13) or b[6] >= 15 or b[33] == 0:
        return None
    q = [i16(b, 10), i16(b, 12), i16(b, 14), i16(b, 16)]
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    q = [c / n for c in q]
    a = [float(i16(b, 18)), float(i16(b, 20)), float(i16(b, 22))]
    an = math.sqrt(sum(c * c for c in a)) or 1.0
    return b[6], q, [c / an for c in a]


def main():
    path = sys.argv[1]
    prevq = {}       # node -> last unit quat
    bins = {}        # bin -> {"mot":float,"nodes":set,"q":{node:[sumq]} }
    with open(path, errors="ignore") as f:
        for line in f:
            line = line.rstrip("\n")
            c = line.find(",")
            if c < 0:
                continue
            try:
                ms = int(line[:c])
            except ValueError:
                continue
            d = decode(line[c + 1:].split(",")[-1])
            if not d:
                continue
            node, q, _a = d
            bk = ms // BIN_MS
            e = bins.setdefault(bk, {"mot": 0.0, "nodes": set(), "q": {}})
            e["nodes"].add(node)
            sq = e["q"].setdefault(node, [0.0, 0.0, 0.0, 0.0, 0])
            for k in range(4):
                sq[k] += q[k]
            sq[4] += 1
            if node in prevq:
                dot = abs(sum(q[k] * prevq[node][k] for k in range(4)))
                e["mot"] += 2.0 * math.acos(max(-1.0, min(1.0, dot)))
            prevq[node] = q

    order = sorted(bins)
    if not order:
        print(f"{path}: no decodable frames")
        return
    # Held pose = a run of still, full-body bins.  Signature = mean quat per node.
    holds = []
    run = []
    for bk in order:
        e = bins[bk]
        motPerTrack = e["mot"] / max(1, len(e["nodes"]))
        still = motPerTrack < 0.05 and len(e["nodes"]) >= 10  # rad, >=10 trackers
        if still:
            run.append(bk)
        else:
            if len(run) >= 2:  # >= ~0.5 s held
                holds.append(run)
            run = []
    if len(run) >= 2:
        holds.append(run)

    def sig(run):
        acc = {}
        for bk in run:
            for node, sq in bins[bk]["q"].items():
                a = acc.setdefault(node, [0.0, 0.0, 0.0, 0.0, 0])
                for k in range(4):
                    a[k] += sq[k]
                a[4] += sq[4]
        out = {}
        for node, a in acc.items():
            if a[4]:
                n = math.sqrt(sum(a[k] * a[k] for k in range(4))) or 1.0
                out[node] = [a[k] / n for k in range(4)]
        return out

    sigs = [sig(r) for r in holds]

    def dist(s1, s2):  # mean per-node quaternion angle between two poses
        common = set(s1) & set(s2)
        if not common:
            return 0.0
        tot = 0.0
        for node in common:
            dot = abs(sum(s1[node][k] * s2[node][k] for k in range(4)))
            tot += 2.0 * math.acos(max(-1.0, min(1.0, dot)))
        return tot / len(common)

    distinct = 1 if holds else 0
    for i in range(1, len(sigs)):
        if max((dist(sigs[i], sigs[j]) for j in range(i)), default=0.0) > 0.35:  # ~20°
            distinct += 1
    durs = [f"{len(r) * BIN_MS / 1000:.1f}s@{min(r) * BIN_MS / 1000:.0f}s" for r in holds]
    verdict = "CALIB" if distinct >= 2 else "no-calib"
    span = (order[-1] - order[0]) * BIN_MS / 1000
    print(f"{path}: span {span:.0f}s, held-poses {len(holds)} {durs[:8]}, "
          f"distinct {distinct} -> {verdict}")


if __name__ == "__main__":
    main()
