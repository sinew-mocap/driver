"""Convert a sinew-host-capture CSV into the driver --raw-log format.

The host capture (sinew-host-capture/rebocap_capture_*.csv) logs the raw USB
dongle stream as `<ms>,R,<hex>`, where each line's hex holds many 0xFAFAAEAE
frames concatenated (and frames can span lines).  spec/Backtest.lean wants the
driver's `--raw-log` shape — `<ms>,<72hex>`, one 36-byte frame per line — so it
can decode and score them.  We split the byte stream on the FAFAAEAE sync,
keeping each sync-to-sync run that is at least one full 36-byte frame and
emitting its first 36 bytes (Backtest filters non-IMU / non-standard frames).

    python driver/capture/hostcap_to_rawlog.py <host_capture.csv> <out.log>
"""
import sys

SYNC = b"\xfa\xfa\xae\xae"
FRAME = 36  # bytes


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    carry = bytearray()
    nframes = 0
    with open(src) as f, open(dst, "w") as o:
        for line in f:
            line = line.rstrip("\n")
            if not line or line[0] == "#":
                continue
            parts = line.split(",", 2)
            if len(parts) < 3 or parts[1] != "R":  # dongle→host frames only
                continue
            ms = parts[0]
            try:
                data = carry + bytes.fromhex(parts[2])
            except ValueError:
                continue
            i = data.find(SYNC)
            while i >= 0:
                k = data.find(SYNC, i + 4)  # next sync bounds this frame
                if k < 0:
                    break  # frame incomplete; wait for the next line
                if k - i >= FRAME:
                    o.write(f"{ms},{data[i:i + FRAME].hex()}\n")
                    nframes += 1
                i = k
            carry = bytearray(data[i:]) if i >= 0 else bytearray(data[-3:])
    print(f"wrote {nframes} frames -> {dst}")


if __name__ == "__main__":
    main()
