# SPDX-License-Identifier: MIT
# Sinew mocap — OSC listener for Blender.
# Run once in the Blender Python console to start receiving tracker data.
# The listener thread stores the latest quaternion per stream name in
# bpy.types._sinew_buf so the timer (02_arrows.py) can read it at display rate.
#
# Streams arriving: "<Joint>_s<subidx>f<flag>"  (raw channels)
#                   "<Joint>_derived"            (6D-merged, emitted by driver)

import bpy, socket, struct, threading

def _read_osc_string(data, off):
    end = data.index(b'\x00', off)
    s = data[off:end].decode('ascii', errors='replace')
    return s, (end + 4) & ~3

def _listener():
    sock = bpy.types._sinew_raw_sock
    buf  = bpy.types._sinew_buf
    lock = bpy.types._sinew_buf_lock
    while not bpy.types._sinew_kill:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception:
            break
        try:
            addr, off = _read_osc_string(data, 0)
            if addr == '/sinew/tracker':
                _, off  = _read_osc_string(data, off)
                name, off = _read_osc_string(data, off)
                qw, qx, qy, qz = struct.unpack_from('>ffff', data, off)
                with lock:
                    buf[name] = (qw, qx, qy, qz)
            elif addr == '/sinew/magcal':
                _, off  = _read_osc_string(data, off)
                name, off = _read_osc_string(data, off)
                x, y, z = struct.unpack_from('>fff', data, off)
                with lock:
                    buf[name] = (x, y, z)   # key is already "{joint}_magcal"
            elif addr == '/sinew/accel':
                _, off  = _read_osc_string(data, off)
                name, off = _read_osc_string(data, off)
                x, y, z = struct.unpack_from('>fff', data, off)
                with lock:
                    buf['accel:' + name] = (x, y, z)   # prefix avoids collision with tracker
            elif addr == '/sinew/calib':
                _, off  = _read_osc_string(data, off)
                name, off = _read_osc_string(data, off)
                progress, = struct.unpack_from('>f', data, off)
                with lock:
                    buf['calib:' + name] = progress
            elif addr == '/sinew/state':
                _, off  = _read_osc_string(data, off)
                name, off = _read_osc_string(data, off)
                state, = struct.unpack_from('>i', data, off)
                with lock:
                    buf['state:' + name] = state
            elif addr == '/sinew/magqual':
                _, off  = _read_osc_string(data, off)
                name, off = _read_osc_string(data, off)
                mag_mag, = struct.unpack_from('>f', data, off)
                with lock:
                    buf['magqual:' + name] = mag_mag
            elif addr == '/sinew/battery':
                _, off  = _read_osc_string(data, off)
                name, off = _read_osc_string(data, off)
                level, = struct.unpack_from('>f', data, off)
                with lock:
                    buf['battery:' + name] = level
            else:
                pass
        except Exception:
            pass

if not hasattr(bpy.types, '_sinew_raw_sock') or bpy.types._sinew_raw_sock is None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', 39539))
    sock.settimeout(0.1)
    bpy.types._sinew_raw_sock  = sock
    bpy.types._sinew_kill      = False
    bpy.types._sinew_buf       = {}
    bpy.types._sinew_buf_lock  = threading.Lock()
    t = threading.Thread(target=_listener, daemon=True)
    t.start()
    print("Sinew OSC listener started on :39539")
else:
    print(f"Listener already running — {len(bpy.types._sinew_buf)} streams in buffer")
