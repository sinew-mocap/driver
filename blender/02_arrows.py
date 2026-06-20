# SPDX-License-Identifier: MIT
# Sinew mocap — arrow visualisation + calibration pie charts for Blender.
# Run after 01_osc_listener.py.  Creates 75 arrow objects + 15 pie charts in the scene
# (15 body trackers × 5 channels: ch1=blue, ch2=yellow, derived=green, magcal=cyan, accel=orange)
# and starts a 20 Hz timer that drives them from the live OSC buffer.
#
# Layout: trackers spread along X (2.5 m spacing), channels along Y.
# Labels show the 0-based node number matching SINEW_JOINT_NODE_TABLE in the C driver.
#
# Derived channel: computed in C driver as 6D-average (Zhou et al.) of all active
# slots for that joint.  Emitted as "<Joint>_derived" over OSC.
#
# Identity quaternions (|w|>0.99985) signal low-power / sleeping sensor channels.
# When a channel is sleeping the arrow holds its last known orientation (no update).
#
# Pie charts (y=3.0 row): calibration progress 0.0–1.0.
# Magqual disk (y=-3.5 row): |B_raw| magnitude — green=normal, red=disturbed (near metal).
# Battery sector (y=4.5 row): 0-36 sectors, 0=empty (dead), 36=full. green=high, red=low.

import sys, math, bpy, mathutils

# ── Phase aligner (Godot Basis round-trip, matches q_align_phase in driver) ──
def q_align_phase(q):
    d = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z
    if d < 1e-10: return q.copy()
    s = 2.0/d
    xs,ys,zs = q.x*s,q.y*s,q.z*s
    wx,wy,wz = q.w*xs,q.w*ys,q.w*zs
    xx,xy,xz = q.x*xs,q.x*ys,q.x*zs
    yy,yz,zz = q.y*ys,q.y*zs,q.z*zs
    r=[[1-(yy+zz),xy-wz,xz+wy],[xy+wz,1-(xx+zz),yz-wx],[xz-wy,yz+wx,1-(xx+yy)]]
    def d3(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
    def n3(a):
        l=math.sqrt(d3(a,a)); return [a[i]/l for i in range(3)] if l>1e-10 else list(a)
    x=[r[0][0],r[1][0],r[2][0]]; y=[r[0][1],r[1][1],r[2][1]]; z=[r[0][2],r[1][2],r[2][2]]
    x=n3(x); dxy=d3(x,y); y=n3([y[i]-x[i]*dxy for i in range(3)])
    dxz=d3(x,z); dyz=d3(y,z); z=n3([z[i]-x[i]*dxz-y[i]*dyz for i in range(3)])
    r[0][0],r[1][0],r[2][0]=x; r[0][1],r[1][1],r[2][1]=y; r[0][2],r[1][2],r[2][2]=z
    det=(r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
        -r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
        +r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]))
    if det<0: r=[[-r[i][j] for j in range(3)] for i in range(3)]
    tr=r[0][0]+r[1][1]+r[2][2]
    if tr>0:
        sv=math.sqrt(tr+1); w=sv*0.5; sv=0.5/sv
        px=(r[2][1]-r[1][2])*sv; py=(r[0][2]-r[2][0])*sv; pz=(r[1][0]-r[0][1])*sv
    else:
        i=(2 if r[1][1]<r[2][2] else 1) if r[0][0]<r[1][1] else (2 if r[0][0]<r[2][2] else 0)
        j=(i+1)%3; k=(i+2)%3
        sv=math.sqrt(r[i][i]-r[j][j]-r[k][k]+1); qi=sv*0.5; sv=0.5/sv
        qw=(r[k][j]-r[j][k])*sv; qj=(r[j][i]+r[i][j])*sv; qk=(r[k][i]+r[i][k])*sv
        vals=[0.0,0.0,0.0]; vals[i]=qi; vals[j]=qj; vals[k]=qk; px,py,pz,w=vals[0],vals[1],vals[2],qw
    out=mathutils.Quaternion((w,px,py,pz)); out.normalize(); return out

bpy.types._q_align_phase = q_align_phase

# ── Arrow geometry ────────────────────────────────────────────────────────────
ARROW_LEN=0.9; SHAFT_R=0.03; HEAD_R=0.09; HEAD_FRAC=0.25

def _make_sphere(name, r=0.35):
    """Icosphere-like sphere via parametric UV mesh."""
    NL, NL2 = 8, 16   # latitude bands, longitude segments
    verts, faces = [], []
    for i in range(NL + 1):
        lat = math.pi * i / NL - math.pi / 2
        for j in range(NL2):
            lon = 2 * math.pi * j / NL2
            verts.append((r * math.cos(lat) * math.cos(lon),
                          r * math.cos(lat) * math.sin(lon),
                          r * math.sin(lat)))
    for i in range(NL):
        for j in range(NL2):
            a = i * NL2 + j
            b = i * NL2 + (j + 1) % NL2
            c = (i + 1) * NL2 + (j + 1) % NL2
            d = (i + 1) * NL2 + j
            faces.append((a, b, c, d))
    mesh = bpy.data.meshes.new(name + '_m')
    mesh.from_pydata(verts, [], faces)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj

def _make_arrow(name):
    shaft_end=ARROW_LEN*(1-HEAD_FRAC); N=8
    ang=[2*math.pi*i/N for i in range(N)]; verts=[]; faces=[]
    def push(*v): verts.append(v); return len(verts)-1
    s0=[push(SHAFT_R*math.cos(a),SHAFT_R*math.sin(a),0) for a in ang]
    s1=[push(SHAFT_R*math.cos(a),SHAFT_R*math.sin(a),shaft_end) for a in ang]
    h0=[push(HEAD_R*math.cos(a), HEAD_R*math.sin(a), shaft_end) for a in ang]
    tip=push(0,0,ARROW_LEN)
    for i in range(N):
        j=(i+1)%N
        faces+=[(s0[i],s0[j],s1[j],s1[i]),(s1[i],s1[j],h0[j],h0[i]),(h0[i],h0[j],tip)]
    mesh=bpy.data.meshes.new(name+'_m'); mesh.from_pydata(verts,[],faces)
    obj=bpy.data.objects.new(name,mesh); bpy.context.scene.collection.objects.link(obj); return obj

def _get_mat(name,color):
    if name in bpy.data.materials: return bpy.data.materials[name]
    mat=bpy.data.materials.new(name); mat.use_nodes=True
    nd=mat.node_tree.nodes.get('Principled BSDF')
    if nd: nd.inputs['Base Color'].default_value=(*color,1.0); nd.inputs['Roughness'].default_value=0.4
    return mat

# Tracker state sphere materials  (SinewJointStateKind order: 0-4)
STATE_MATS = [
    _get_mat('sinew_state_unknown',   (0.4, 0.4, 0.4)),   # 0 UNKNOWN  — grey
    _get_mat('sinew_state_active',    (0.1, 0.9, 0.2)),   # 1 ACTIVE   — green
    _get_mat('sinew_state_stale',     (0.9, 0.8, 0.1)),   # 2 STALE    — yellow
    _get_mat('sinew_state_asleep',    (0.1, 0.1, 0.6)),   # 3 ASLEEP   — dark blue
    _get_mat('sinew_state_rejoining', (0.9, 0.4, 0.1)),   # 4 REJOINING— orange
]

mat_ch1 = _get_mat('sinew_ch1',(0.2,0.6,1.0))   # blue   = channel 1
mat_ch2 = _get_mat('sinew_ch2',(0.8,0.8,0.1))   # yellow = channel 2
mat_drv = _get_mat('sinew_drv',(0.1,0.9,0.3))   # green  = derived (6D merge)
mat_mag = _get_mat('sinew_mag',(0.0,0.9,0.9))   # cyan   = calibrated magnetometer
mat_acc = _get_mat('sinew_acc',(1.0,0.4,0.1))   # orange = accelerometer unit vector

# Pie chart materials: red=bootstrapping, amber=LS fitting, green=calibrated
mat_pie_lo  = _get_mat('sinew_pie_lo',  (0.9, 0.1, 0.1))   # red
mat_pie_mid = _get_mat('sinew_pie_mid', (0.9, 0.5, 0.0))   # amber
mat_pie_hi  = _get_mat('sinew_pie_hi',  (0.1, 0.9, 0.1))   # green

# ── Layout ────────────────────────────────────────────────────────────────────
JOINT_ORDER = [
    'Hips','LeftUpperLeg','RightUpperLeg','LeftLowerLeg','RightLowerLeg',
    'LeftFoot','RightFoot','Chest','Head','LeftUpperArm','RightUpperArm',
    'LeftLowerArm','RightLowerArm','LeftHand','RightHand'
]
JOINT_X=2.5; CH_Y=0.75
# 5 channels per tracker: ch1, ch2, derived (quaternion), magcal, accel (unit vector)
CHANNELS = [
    ('s_ch1',   mat_ch1),   # blue   — raw channel 1 quaternion
    ('s_ch2',   mat_ch2),   # yellow — raw channel 2 quaternion
    ('derived', mat_drv),   # green  — 6D-merged derived quaternion
    ('magcal',  mat_mag),   # cyan   — calibrated magnetic field direction
    ('accel',   mat_acc),   # orange — accelerometer unit vector
]
CH_Y_OFFSETS = [-1.5, -0.75, 0.0, 0.9, 1.8]   # ch1, ch2, derived, magcal, accel

# Wipe old sinew objects
bpy.ops.object.select_all(action='DESELECT')
for obj in list(bpy.data.objects):
    if obj.name.startswith(('rot_','lbl_','pie_','sph_','mqdisk_','bat_')):
        obj.select_set(True)
bpy.ops.object.delete()

arrow_map   = {}   # stream_key → (rot_obj, base_xyz)
pie_map     = {}   # joint → calibration pie mesh obj
sphere_map  = {}   # joint → state sphere mesh obj
mqual_map   = {}   # joint → magqual disk mesh obj
battery_map = {}   # joint → battery sector mesh obj

for ji, joint in enumerate(JOINT_ORDER):
    tracker_num = ji   # 0-based, matches SINEW_JOINT_NODE_TABLE in C driver
    x = ji * JOINT_X
    for ci, ((tag, mat), yo) in enumerate(zip(CHANNELS, CH_Y_OFFSETS)):
        y = yo
        base = (x, y, 0.0)
        key  = f'{joint}_{tag}'
        obj  = _make_arrow(f'rot_{key}')
        obj.location = base
        obj.data.materials.append(mat)
        crv = bpy.data.curves.new(f'lbl_{key}_c', type='FONT')
        crv.body = f"#{tracker_num} {joint}\n{tag}"
        crv.size = 0.11
        lo = bpy.data.objects.new(f'lbl_{key}', crv)
        lo.location = (x-0.35, y+0.5, 0.0)
        bpy.context.scene.collection.objects.link(lo)
        arrow_map[key] = (obj, base)

    # Pie chart — one per tracker, above the accel row
    pie_mesh = bpy.data.meshes.new(f'pie_{joint}_m')
    pie_obj  = bpy.data.objects.new(f'pie_{joint}', pie_mesh)
    pie_obj.location = (x, 3.0, 0.0)
    pie_obj.data.materials.append(mat_pie_lo)
    bpy.context.scene.collection.objects.link(pie_obj)
    pie_map[joint] = pie_obj

    # State sphere — below the ch1 row; colour = SinewJointStateKind
    sph = _make_sphere(f'sph_{joint}')
    sph.location = (x, -2.5, 0.0)
    sph.data.materials.append(STATE_MATS[0])   # start unknown/grey
    sphere_map[joint] = sph

    # Magqual disk (flat circle mesh) — below state sphere; green=normal, red=disturbed
    mq_mesh = bpy.data.meshes.new(f'mqdisk_{joint}_m')
    mq_obj  = bpy.data.objects.new(f'mqdisk_{joint}', mq_mesh)
    mq_obj.location = (x, -3.5, 0.0)
    mq_obj.data.materials.append(_get_mat('sinew_mq_ok', (0.1, 0.9, 0.2)))
    bpy.context.scene.collection.objects.link(mq_obj)
    mqual_map[joint] = mq_obj

    # Battery sector (pie chart style) — above calibration pie
    bat_mesh = bpy.data.meshes.new(f'bat_{joint}_m')
    bat_obj  = bpy.data.objects.new(f'bat_{joint}', bat_mesh)
    bat_obj.location = (x, 4.5, 0.0)
    bat_obj.data.materials.append(_get_mat('sinew_bat_hi', (0.1, 0.9, 0.2)))
    bpy.context.scene.collection.objects.link(bat_obj)
    battery_map[joint] = bat_obj

bpy.types._sinew_arrow_map2  = arrow_map
bpy.types._sinew_pie_map     = pie_map
bpy.types._sinew_sphere_map  = sphere_map
bpy.types._sinew_mqual_map   = mqual_map
bpy.types._sinew_battery_map = battery_map
print(f"Created {len(arrow_map)} arrows + {len(pie_map)} calib pies + {len(sphere_map)} state spheres + {len(mqual_map)} magqual disks + {len(battery_map)} battery sectors")

# ── Timer ─────────────────────────────────────────────────────────────────────
buf         = bpy.types._sinew_buf
buf_lock    = bpy.types._sinew_buf_lock
sphere_map  = bpy.types._sinew_sphere_map
mqual_map   = bpy.types._sinew_mqual_map
battery_map = bpy.types._sinew_battery_map

joint_raw_order = {j: [] for j in JOINT_ORDER}
prev_q          = {k: mathutils.Quaternion((1,0,0,0)) for k in arrow_map}
pie_sectors     = {j: -1 for j in JOINT_ORDER}   # last-drawn sector count

PIE_R = 0.45
PIE_N = 36   # sectors in a full circle

MAG_REF = 20000.0   # expected |B_raw| in LSB — per-node-constant in clean environment
MAG_TOL = 0.30      # ±30% tolerance before colour changes

mat_mq_ok   = _get_mat('sinew_mq_ok',  (0.1, 0.9, 0.2))   # green = clean field
mat_mq_warn = _get_mat('sinew_mq_warn',(0.9, 0.8, 0.1))   # yellow = slight disturbance
mat_mq_bad  = _get_mat('sinew_mq_bad', (0.9, 0.1, 0.1))   # red = strong interference

mat_bat_hi  = _get_mat('sinew_bat_hi',  (0.1, 0.9, 0.2))  # green  ≥60 %
mat_bat_mid = _get_mat('sinew_bat_mid', (0.9, 0.8, 0.1))  # yellow 20–59 %
mat_bat_lo  = _get_mat('sinew_bat_lo',  (0.9, 0.1, 0.1))  # red    <20 %

DISK_R = 0.40   # magqual disk radius
mqual_ref  = {}   # joint → running reference magnitude (updated slowly)
bat_sectors = {j: -1 for j in JOINT_ORDER}

def _rebuild_disk(joint, mag_mag):
    """Solid filled disk (36 triangles), colour by deviation from reference."""
    global mqual_ref
    ref = mqual_ref.get(joint, 0.0)
    if ref == 0.0:
        mqual_ref[joint] = mag_mag; ref = mag_mag
    else:
        mqual_ref[joint] = ref * 0.995 + mag_mag * 0.005   # slow EMA update
    obj  = mqual_map[joint]
    mesh = obj.data
    if len(mesh.vertices) == 0:
        verts = [(0.0, 0.0, 0.0)]
        for i in range(36):
            a = 2 * math.pi * i / 36
            verts.append((DISK_R * math.cos(a), DISK_R * math.sin(a), 0.0))
        faces = [(0, i+1, (i+1)%36+1) for i in range(36)]
        mesh.from_pydata(verts, [], faces)
        mesh.update()
    dev = abs(mag_mag - ref) / (ref + 1.0)
    mat = mat_mq_bad if dev > MAG_TOL else (mat_mq_warn if dev > MAG_TOL*0.5 else mat_mq_ok)
    if not obj.data.materials: obj.data.materials.append(mat)
    elif obj.data.materials[0] is not mat: obj.data.materials[0] = mat

BAT_R = 0.45; BAT_N = 36

def _rebuild_battery(joint, level):
    k = int(max(0.0, min(1.0, level)) * BAT_N)
    if bat_sectors[joint] == k: return
    bat_sectors[joint] = k
    obj  = battery_map[joint]
    mesh = obj.data
    mesh.clear_geometry()
    if k == 0: return
    verts = [(0.0, 0.0, 0.0)]
    for i in range(k + 1):
        a = math.pi/2 - 2*math.pi * i / BAT_N
        verts.append((BAT_R * math.cos(a), BAT_R * math.sin(a), 0.0))
    faces = [(0, i+1, i+2) for i in range(k)]
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    mat = mat_bat_hi if level >= 0.6 else (mat_bat_mid if level >= 0.2 else mat_bat_lo)
    if not obj.data.materials: obj.data.materials.append(mat)
    elif obj.data.materials[0] is not mat: obj.data.materials[0] = mat

def _rebuild_pie(joint, progress):
    k = int(progress * PIE_N)
    if pie_sectors[joint] == k:
        return
    pie_sectors[joint] = k
    obj  = pie_map[joint]
    mesh = obj.data
    mesh.clear_geometry()
    if k == 0:
        return
    verts = [(0.0, 0.0, 0.0)]
    for i in range(k + 1):
        a = math.pi/2 - 2*math.pi * i / PIE_N   # start from top, clockwise
        verts.append((PIE_R * math.cos(a), PIE_R * math.sin(a), 0.0))
    faces = [(0, i+1, i+2) for i in range(k)]
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    mat = mat_pie_hi if progress >= 1.0 else (mat_pie_mid if progress >= 0.5 else mat_pie_lo)
    if not obj.data.materials:
        obj.data.materials.append(mat)
    else:
        obj.data.materials[0] = mat

def _sign_align(ref, q):
    qc = q.copy()
    if ref.dot(qc) < 0: qc.negate()
    return qc

def _is_identity(q, tol=0.99985):
    # True = this channel is in low-power mode; caller holds last orientation instead of updating
    return abs(q.w) > tol

def _vec_to_quat(v):
    """Quaternion rotating +Z to the direction of vector v."""
    n = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if n < 1e-9: return mathutils.Quaternion((1,0,0,0))
    vn = [v[0]/n, v[1]/n, v[2]/n]
    cx, cy = -vn[1], vn[0]
    cn = math.sqrt(cx*cx + cy*cy)
    if cn < 1e-9:
        if vn[2] > 0: return mathutils.Quaternion((1,0,0,0))
        return mathutils.Quaternion((0,1,0,0))
    angle = math.acos(max(-1.0, min(1.0, vn[2])))
    ax, ay = cx/cn, cy/cn
    s = math.sin(angle*0.5)
    return mathutils.Quaternion((math.cos(angle*0.5), ax*s, ay*s, 0.0))

def sinew_timer():
    if not getattr(bpy.types, '_sinew_timer_running', False):
        return None
    with buf_lock:
        snap = dict(buf)

    for joint in JOINT_ORDER:
        # Maintain stable first-seen order of raw quaternion streams for this joint
        raw_streams = sorted(k for k in snap
                             if k.startswith(joint+'_s') and not k.endswith('_derived'))
        seen = joint_raw_order[joint]
        for rs in raw_streams:
            if rs not in seen:
                seen.append(rs)

        # ch1 and ch2 — quaternion channels (index 1 and 2 in arrival order)
        for ci, tag in [(1,'s_ch1'),(2,'s_ch2')]:
            key = f'{joint}_{tag}'
            if key not in arrow_map: continue
            obj, base = arrow_map[key]; obj.location = base
            src = seen[ci] if ci < len(seen) else None
            if not src or src not in snap: continue
            q = mathutils.Quaternion(snap[src]); q.normalize()
            if _is_identity(q): continue
            q = q_align_phase(q); q = _sign_align(prev_q[key], q)
            prev_q[key] = q.copy()
            obj.rotation_mode = 'QUATERNION'; obj.rotation_quaternion = q

        # Derived — 6D-merged quaternion from C driver
        key = f'{joint}_derived'
        if key in arrow_map:
            obj, base = arrow_map[key]; obj.location = base
            raw = snap.get(f'{joint}_derived')
            if raw and len(raw) == 4:
                q = mathutils.Quaternion(raw); q.normalize()
                if not _is_identity(q):
                    q = q_align_phase(q); q = _sign_align(prev_q[key], q)
                    prev_q[key] = q.copy()
                    obj.rotation_mode = 'QUATERNION'; obj.rotation_quaternion = q

        # Magcal — calibrated magnetic field already in world frame (rotated by C driver).
        key = f'{joint}_magcal'
        if key in arrow_map:
            obj, base = arrow_map[key]; obj.location = base
            raw = snap.get(f'{joint}_magcal')
            if raw and len(raw) == 3:
                obj.rotation_mode = 'QUATERNION'
                obj.rotation_quaternion = _vec_to_quat(raw)

        # Accel — already in world frame (gravity direction, rotated by C driver).
        key = f'{joint}_accel'
        if key in arrow_map:
            obj, base = arrow_map[key]; obj.location = base
            raw = None
            for src in seen:
                v = snap.get('accel:' + src)
                if v and len(v) == 3:
                    raw = v; break
            if raw:
                obj.rotation_mode = 'QUATERNION'
                obj.rotation_quaternion = _vec_to_quat(raw)

        # Calibration pie
        progress = snap.get(f'calib:{joint}', 0.0)
        _rebuild_pie(joint, progress)

        # Magqual disk — |B_raw| magnitude; green=clean, red=near metal
        mag_mag = snap.get(f'magqual:{joint}')
        if mag_mag is not None and mag_mag > 0:
            _rebuild_disk(joint, mag_mag)

        # Battery sector
        batt = snap.get(f'battery:{joint}')
        if batt is not None:
            _rebuild_battery(joint, batt)

        # State sphere — colour from /sinew/state
        state = snap.get(f'state:{joint}', 0)
        sph = sphere_map[joint]
        mat = STATE_MATS[state] if 0 <= state < len(STATE_MATS) else STATE_MATS[0]
        if not sph.data.materials:
            sph.data.materials.append(mat)
        elif sph.data.materials[0] is not mat:
            sph.data.materials[0] = mat

    return 0.05   # 20 Hz

bpy.types._sinew_timer_running = True
bpy.app.timers.register(sinew_timer, first_interval=0.3)
print("Timer running at 20 Hz")
