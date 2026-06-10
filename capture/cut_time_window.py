"""Cut a time window out of a sinew host-capture CSV into the driver --raw-log format.

The host capture (`<ms>,R,<hex>`) holds many 0xFAFAAEAE frames per line; this splits
the byte stream on the sync the same way hostcap_to_rawlog does, keeps frames whose
time (relative to the first frame) falls in [start_s, end_s), and rebases their
timestamps so the window starts at 0.

    python driver/capture/cut_time_window.py <host_capture.csv> <start_s> <end_s> <out.log>
"""
import sys

SYNC = b"\xfa\xfa\xae\xae"
FRAME = 36  # bytes


def main() -> None:
    src, start_s, end_s, dst = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    carry = bytearray()
    t0 = None
    nframes = 0
    with open(src) as f, open(dst, "w") as o:
        for line in f:
            line = line.rstrip("\n")
            if not line or line[0] == "#":
                continue
            parts = line.split(",", 2)
            if len(parts) < 3 or parts[1] != "R":
                continue
            try:
                ms = int(parts[0])
                data = carry + bytes.fromhex(parts[2])
            except ValueError:
                continue
            if t0 is None:
                t0 = ms
            rel = (ms - t0) / 1000.0
            i = data.find(SYNC)
            while i >= 0:
                k = data.find(SYNC, i + 4)
                if k < 0:
                    break
                if k - i >= FRAME and start_s <= rel < end_s:
                    o.write(f"{int((ms - t0) - start_s * 1000)},{data[i:i + FRAME].hex()}\n")
                    nframes += 1
                i = k
            carry = bytearray(data[i:]) if i >= 0 else bytearray(data[-3:])
    print(f"wrote {nframes} frames ({start_s:.0f}-{end_s:.0f}s) -> {dst}")


if __name__ == "__main__":
    main()
