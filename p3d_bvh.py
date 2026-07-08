"""
BVH (Biovision Hierarchy) reader — the inverse of p3d_export.export_bvh.

Turns a `.bvh` (Blender / Maya / MotionBuilder / our own exporter) into the same in-memory
shape the rest of the viewer uses, so a BVH can be:
  * viewed          -> `bvh_to_model(bvh)`      → a core.Model (skeleton + one clip)
  * written to .p3d -> `bvh_to_channels(bvh)`   → {bone: {rot,loc}} for p3d_write.build_clip

It respects each joint's declared CHANNELS order (BVH files vary: `Zrotation Yrotation Xrotation`,
`Zrotation Xrotation Yrotation`, …), composing the per-axis rotations left-to-right, then converts
the resulting matrix to a quaternion (column-vector convention, matching core._qmat_col). A BVH
exported by this tool round-trips to the original pose.
"""
import math

import p3d_core as core

_D2R = math.pi / 180.0


# ------------------------------------------------------------------ parse
def read_bvh(path):
    """Parse a BVH file. Returns dict:
        joints     : [{name, parent, offset:(x,y,z), channels:[str,...]}]  (ROOT/JOINT only)
        frames     : [[float, ...], ...]   (motion rows, one per frame)
        frame_time : float (seconds)
    """
    joints = []
    stack = []            # indices into joints; -1 sentinel for an End Site
    frame_time = 1.0 / 30.0
    frames = []
    with open(path, "r", encoding="latin-1") as fh:
        lines = fh.read().splitlines()

    i = 0
    n = len(lines)
    # --- HIERARCHY ---
    while i < n:
        t = lines[i].split()
        i += 1
        if not t:
            continue
        k = t[0]
        if k in ("ROOT", "JOINT"):
            idx = len(joints)
            joints.append({"name": t[1], "parent": (stack[-1] if stack and stack[-1] >= 0 else -1),
                           "offset": (0.0, 0.0, 0.0), "channels": []})
            stack.append(idx)
        elif k == "End":                       # End Site — a leaf tip, not a joint
            stack.append(-1)
        elif k == "OFFSET" and stack and stack[-1] >= 0:
            joints[stack[-1]]["offset"] = (float(t[1]), float(t[2]), float(t[3]))
        elif k == "CHANNELS" and stack and stack[-1] >= 0:
            joints[stack[-1]]["channels"] = t[2:]
        elif k == "}":
            if stack:
                stack.pop()
        elif k == "MOTION":
            break
    # --- MOTION ---
    while i < n:
        t = lines[i].split()
        i += 1
        if not t:
            continue
        if t[0] == "Frames:":
            continue
        if t[0] == "Frame" and len(t) >= 3 and t[1] == "Time:":
            frame_time = float(t[2])
            break
    for line in lines[i:]:
        t = line.split()
        if t:
            frames.append([float(v) for v in t])
    return {"joints": joints, "frames": frames, "frame_time": frame_time}


# ------------------------------------------------------------------ math
def _axis_mat(axis, deg):
    a = deg * _D2R
    c, s = math.cos(a), math.sin(a)
    if axis == "X":
        return [1, 0, 0, 0, c, -s, 0, s, c]
    if axis == "Y":
        return [c, 0, s, 0, 1, 0, -s, 0, c]
    return [c, -s, 0, s, c, 0, 0, 0, 1]        # Z


def _mm(a, b):
    return [sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3)) for r in range(3) for c in range(3)]


def _mat_to_quat(M):
    """Column-vector rotation matrix (flat9) -> (x,y,z,w), matching core._qmat_col."""
    tr = M[0] + M[4] + M[8]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (M[7] - M[5]) * s
        y = (M[2] - M[6]) * s
        z = (M[3] - M[1]) * s
    elif M[0] > M[4] and M[0] > M[8]:
        s = 2.0 * math.sqrt(1.0 + M[0] - M[4] - M[8])
        w = (M[7] - M[5]) / s
        x = 0.25 * s
        y = (M[1] + M[3]) / s
        z = (M[2] + M[6]) / s
    elif M[4] > M[8]:
        s = 2.0 * math.sqrt(1.0 + M[4] - M[0] - M[8])
        w = (M[2] - M[6]) / s
        x = (M[1] + M[3]) / s
        y = 0.25 * s
        z = (M[5] + M[7]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + M[8] - M[0] - M[4])
        w = (M[3] - M[1]) / s
        x = (M[2] + M[6]) / s
        y = (M[5] + M[7]) / s
        z = 0.25 * s
    nrm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return (x / nrm, y / nrm, z / nrm, w / nrm)


# ------------------------------------------------------------------ manual rotation
# The game + this viewer are Y-up; other tools (Blender = Z-up) differ. There's no reliable auto
# convert (Blender also re-derives bone rolls), so the user dials in a whole-animation rotation
# manually (e.g. X = -90 to stand up a Z-up source).
def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _parent_map(joints):
    """name -> parent-name, from a list of core.Joint or BVH joint dicts."""
    names, pars = [], []
    for j in joints:
        if isinstance(j, dict):
            names.append(j["name"]); pars.append(j["parent"])
        else:
            names.append(j.name); pars.append(j.parent)
    pm = {}
    for i, nm in enumerate(names):
        p = pars[i]
        pm[nm] = names[p] if (i != 0 and isinstance(p, int) and 0 <= p < i) else None
    return pm


def _axis_quat(axis, deg):
    r = math.radians(deg) * 0.5
    s, c = math.sin(r), math.cos(r)
    return {"x": (s, 0, 0, c), "y": (0, s, 0, c), "z": (0, 0, s, c)}[axis]


def _euler_quat(ax, ay, az):
    """(x,y,z,w) for R = Rx(ax) @ Ry(ay) @ Rz(az)  (degrees)."""
    return _qmul(_axis_quat("x", ax), _qmul(_axis_quat("y", ay), _axis_quat("z", az)))


def apply_rotation(channels, joints, ax, ay, az):
    """Re-express the whole animation in a coordinate system rotated by (ax, ay, az) degrees about
    X/Y/Z — a CHANGE OF BASIS, not a spin.

    Tools like Blender author BVH in a Z-up world and, on import/export, rotate the ENTIRE
    coordinate system (offsets get rotated; each bone's local rotation gets CONJUGATED). A naive
    'rotate the root' only fixes the global facing while leaving every bone twisted about its own
    axis (looks fine as a stick figure, but the skinned mesh is twisted in-game). The correct undo
    is the inverse change of basis applied to EVERY bone: q -> Q q Q^-1, loc -> Q·loc, where Q is
    the rotation. For a Z-up Blender source, X = -90 recovers the game frame exactly.

    (`joints` is unused now — kept for call-compatibility.)"""
    if not (ax or ay or az):
        return
    q = _euler_quat(ax, ay, az)
    qi = (-q[0], -q[1], -q[2], q[3])                   # inverse of a unit quaternion
    R = core._qmat_col((q[0], q[1], q[2], q[3]))       # 3x3 flat, for rotating loc vectors
    for b in list(channels):
        s = channels[b]
        if "rot" in s:
            fr, qs = s["rot"]
            s["rot"] = (fr, [_qmul(q, _qmul(qq, qi)) for qq in qs])   # conjugation
        if "loc" in s:
            fr, vs = s["loc"]
            s["loc"] = (fr, [(R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
                              R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
                              R[6] * v[0] + R[7] * v[1] + R[8] * v[2]) for v in vs])


# ------------------------------------------------------------------ conversions
def _iter_channels(bvh):
    """Yield (joint_index, {channel_name: column_index}) using the row layout of MOTION."""
    col = 0
    layout = []
    for ji, j in enumerate(bvh["joints"]):
        cmap = {}
        for ch in j["channels"]:
            cmap[ch] = col
            col += 1
        layout.append((ji, cmap))
    return layout, col


def bvh_to_channels(bvh, rot=(0.0, 0.0, 0.0)):
    """-> (channels{bone:{'rot':(frames,quats), 'loc':(frames,xyz)}}, num_frames, fps).

    rot = the joint's parent-local rotation quaternion (x,y,z,w) per frame.
    loc = OFFSET + position-channels (absolute parent-local translation) — only for joints that
    actually carry position channels (root, translated bones).
    rot=(ax,ay,az) -- optional manual rotation (degrees about world X/Y/Z) applied to the whole
    animation (e.g. (-90,0,0) to stand up a Z-up source)."""
    joints = bvh["joints"]
    layout, ncol = _iter_channels(bvh)
    rows = [r for r in bvh["frames"] if len(r) >= ncol]
    nframes = len(rows)
    fps = round(1.0 / bvh["frame_time"]) if bvh["frame_time"] > 0 else 30
    frames = list(range(nframes))

    channels = {}
    for ji, cmap in layout:
        name = joints[ji]["name"]
        ox, oy, oz = joints[ji]["offset"]
        rot_chans = [c for c in joints[ji]["channels"] if c.endswith("rotation")]
        pos_chans = [c for c in joints[ji]["channels"] if c.endswith("position")]
        quats = []
        locs = []
        for row in rows:
            R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            for ch in rot_chans:                       # compose in declared order
                R = _mm(R, _axis_mat(ch[0], row[cmap[ch]]))
            quats.append(_mat_to_quat(R))
            if pos_chans:
                px = row[cmap["Xposition"]] if "Xposition" in cmap else 0.0
                py = row[cmap["Yposition"]] if "Yposition" in cmap else 0.0
                pz = row[cmap["Zposition"]] if "Zposition" in cmap else 0.0
                locs.append((ox + px, oy + py, oz + pz))
        slots = {}
        if rot_chans:
            slots["rot"] = (frames, quats)
        if pos_chans:
            slots["loc"] = (frames, locs)
        if slots:
            channels[name] = slots
    if rot and any(rot):
        apply_rotation(channels, bvh["joints"], rot[0], rot[1], rot[2])
    return channels, nframes, fps


def bvh_to_model(bvh, source="(BVH)", clip_name="bvh_clip", rot=(0.0, 0.0, 0.0)):
    """Build a core.Model (skeleton with identity-rest bones at the BVH offsets + one clip)."""
    joints = []
    for j in bvh["joints"]:
        ox, oy, oz = j["offset"]
        # core.Joint.local is row-major; pure translation (identity rotation), translation in row 3
        local = [1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 ox, oy, oz, 1.0]
        jt = core.Joint(j["name"], j["parent"] if j["parent"] >= 0 else 0, local)
        joints.append(jt)
    core._fill_bind(joints)
    channels, nframes, fps = bvh_to_channels(bvh, rot=rot)
    clip = core.Clip(clip_name, channels)
    return core.Model(source, joints, source, [clip]), fps
