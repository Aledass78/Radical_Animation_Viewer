"""
Exporters for decoded Prototype 2 animations.

  * export_bvh   — Biovision Hierarchy (.bvh). Blender imports this natively
                   (File > Import > Motion Capture (.bvh)); also read by Maya, MotionBuilder,
                   Unity, etc. This is the "Blender-compatible" export.
  * export_json  — a lossless dump of the decoded channels (skeleton + rotation/translation/
                   scale curves) for custom pipelines.

All three viewers/importers share the verified decode in p3d_core; the exporters only turn a
posed skeleton into a file, so they inherit that correctness. The BVH writer is validated by a
round-trip (parse the BVH back, re-run FK, compare joint world positions — see validate_bvh).

BVH notes
---------
BVH bones have NO rest rotation — the rest orientation is encoded purely in each joint's OFFSET
(its parent-local rest translation). Our animation stores each bone's FULL parent-local rotation
(it replaces the rest rotation), so with OFFSET = rest translation the per-frame rotation channel
is exactly that quaternion converted to Euler. Bones with a translation (TRAN) channel also get
position channels carrying (animated - rest) translation. Scale (SCL) has no BVH equivalent and is
dropped (it only appears on non-deforming helper bones). Angles are exported in ZYX order
(`Zrotation Yrotation Xrotation`); coordinates are the game's native Y-up (Blender's importer
"Y Up" option, on by default, brings it in upright).
"""
import json
import math


# ------------------------------------------------------------------ Euler
def _mat_to_euler_zyx(M):
    """Column-vector rotation matrix (flat 9, M[r*3+c]) -> (z, y, x) radians for R = Rz·Ry·Rx."""
    sy = max(-1.0, min(1.0, -M[6]))          # M[2][0] = -sin(y)
    y = math.asin(sy)
    cy = math.cos(y)
    if abs(cy) > 1e-6:
        x = math.atan2(M[7], M[8])           # M[2][1], M[2][2]
        z = math.atan2(M[3], M[0])           # M[1][0], M[0][0]
    else:                                    # gimbal lock (y = ±90°)
        x = math.atan2(-M[5], M[4])          # M[1][2], M[1][1]
        z = 0.0
    return z, y, x


_R2D = 180.0 / math.pi


# ------------------------------------------------------------------ BVH
def _children_map(model):
    kids = [[] for _ in model.joints]
    roots = []
    for i, j in enumerate(model.joints):
        if i != 0 and 0 <= j.parent < len(model.joints) and j.parent != i:
            kids[j.parent].append(i)
        else:
            roots.append(i)
    return kids, roots


# Rx(+90): game Y-up -> Blender Z-up, as a CHANGE OF BASIS applied to every bone (this is exactly
# what Blender does to a BVH — offsets/translations rotate by Rx(+90), local rotations conjugate by
# it — so our export matches Blender's convention and round-trips through the importer's X=-90 fix).
_R90 = (1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)     # Rx(+90) row-major flat9
_R90i = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)    # Rx(-90) = its inverse/transpose


def _m3v(M, v):
    return (M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
            M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
            M[6] * v[0] + M[7] * v[1] + M[8] * v[2])


def _m3m(A, B):
    return [sum(A[r * 3 + k] * B[k * 3 + c] for k in range(3)) for r in range(3) for c in range(3)]


def export_bvh(model, clip_idx, path, fps=30, rest_bones=frozenset(), zup=False):
    """Write clip `clip_idx` of `model` to a BVH file at `path`.

    rest_bones -- joint indices to write at their REST pose (ignore their animation). Use this for
    non-deforming attachment locators (`*_Grapple`, `*_Con`): the game parks them at an off-body
    anchor (a held/thrown object's spot), which drags a lone bone far from the character — often
    below the floor — in Blender. Resting them keeps the bone present (glued to its parent, e.g. the
    wrist) so you can still animate holding on it, without the stray spike.
    zup -- write in Blender's **Z-up** convention (default). BVH carries no up-axis, and Blender
    reads it Z-up; the game is Y-up, so a raw export imports **upside-down** in Blender. Rotating the
    root by Rx(+90) makes it import upright/grounded (matching how an .fbx comes in). Set False to
    keep the game's native Y-up."""
    joints = model.joints
    clip = model.clips[clip_idx]
    kids, roots = _children_map(model)
    root = roots[0] if roots else 0
    # a joint gets position channels if it's the root or carries a translation channel (rested
    # bones keep theirs too, but the value is constant = their offset, i.e. a zero position channel)
    has_pos = [(i == root) or ('loc' in model.channels_for(clip_idx, i)) for i in range(len(joints))]

    order = []                                # DFS pre-order (== MOTION value order)
    lines = ["HIERARCHY"]

    def _offset(i):
        if i == root:
            # The root carries its full world height in the POSITION channel, with a zero OFFSET.
            # Otherwise the ~1-unit root height is baked into the bone offset, and importers that
            # place the root by its position channel alone (ignoring the offset) drop the whole
            # character ~1 unit — legs end up below the floor in Blender.
            return (0.0, 0.0, 0.0)
        o = model.rest_offset(i)
        return _m3v(_R90, o) if zup else o

    def emit(i, depth, is_root):
        order.append(i)
        pad = "\t" * depth
        kw = "ROOT" if is_root else "JOINT"
        ox, oy, oz = _offset(i)
        lines.append(f"{pad}{kw} {joints[i].name}")
        lines.append(f"{pad}{{")
        lines.append(f"{pad}\tOFFSET {ox:.6f} {oy:.6f} {oz:.6f}")
        if has_pos[i]:
            lines.append(f"{pad}\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation")
        else:
            lines.append(f"{pad}\tCHANNELS 3 Zrotation Yrotation Xrotation")
        for c in kids[i]:
            emit(c, depth + 1, False)
        if not kids[i]:                       # leaf -> End Site (small tip so the bone is visible)
            tip = 0.03 * model.span
            tx, ty, tz = _m3v(_R90, (0.0, tip, 0.0)) if zup else (0.0, tip, 0.0)
            lines.append(f"{pad}\tEnd Site")
            lines.append(f"{pad}\t{{")
            lines.append(f"{pad}\t\tOFFSET {tx:.6f} {ty:.6f} {tz:.6f}")
            lines.append(f"{pad}\t}}")
        lines.append(f"{pad}}}")

    emit(root, 0, True)

    nframes = clip.max_frame + 1
    lines.append("MOTION")
    lines.append(f"Frames: {nframes}")
    lines.append(f"Frame Time: {1.0 / max(1, fps):.7f}")

    for f in range(nframes):
        vals = []
        for i in order:
            if i in rest_bones:
                R, t = model.rest_rot_trans(i)               # park at rest (ignore its animation)
            else:
                R, t = model.local_rot_trans(clip_idx, i, float(f))
            if zup:                                          # Y-up -> Z-up change of basis (every bone)
                R = _m3m(_m3m(_R90, R), _R90i)               # conjugate the local rotation
                t = _m3v(_R90, t)
            if has_pos[i]:
                ox, oy, oz = _offset(i)
                vals += [t[0] - ox, t[1] - oy, t[2] - oz]      # position = animated - rest offset
            z, y, x = _mat_to_euler_zyx(R)
            vals += [z * _R2D, y * _R2D, x * _R2D]
        lines.append(" ".join(f"{v:.6f}" for v in vals))

    with open(path, "w", encoding="ascii") as fh:
        fh.write("\n".join(lines) + "\n")
    return nframes


# ------------------------------------------------------------------ JSON
def export_json(model, clip_idx, path, fps=30, rounding=6):
    """Write a lossless dump of the decoded skeleton + clip channels to `path`."""
    def r(x):
        return round(x, rounding)

    clip = model.clips[clip_idx]
    sk = {
        "name": model.name,
        "joints": [
            {"name": j.name, "parent": j.parent,
             "offset": [r(v) for v in model.rest_offset(i)],
             "bind": [r(v) for v in j.bind]}
            for i, j in enumerate(model.joints)
        ],
    }
    channels = {}
    for bone, slots in clip.channels.items():
        out = {}
        for slot, (frames, vals) in slots.items():
            out[slot] = {"frames": list(frames),
                         "values": [[r(c) for c in v] for v in vals]}
        channels[bone] = out
    data = {
        "source": model.source,
        "skeleton": sk,
        "clip": {"name": clip.name, "frameCount": clip.max_frame + 1, "fps": fps,
                 "channels": channels},
        "note": "rot values = (x,y,z,w) quaternions; loc/scl = (x,y,z). "
                "Local FK: world = parent @ (T(loc) . R(quat) . S(scl)); loc/rot replace rest.",
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(data, fh, separators=(",", ":"))
    return len(channels)


# ------------------------------------------------------------------ validation
def validate_bvh(model, clip_idx, fps=30):
    """Round-trip check: write BVH to a string, re-parse it, run BVH FK, and compare joint
    world positions to the model's own rot+trans FK (no scale). Returns max positional error."""
    import io
    import tempfile
    import os as _os

    fd, tmp = tempfile.mkstemp(suffix=".bvh")
    _os.close(fd)
    try:
        export_bvh(model, clip_idx, tmp, fps=fps)
        names, offsets, chans, parents, order = _parse_bvh_hierarchy(tmp)
        frames = _parse_bvh_motion(tmp, order, chans)
    finally:
        _os.remove(tmp)

    # reference: model rot+trans FK (no scale), by joint name
    idx = {j.name: i for i, j in enumerate(model.joints)}
    worst = 0.0
    test_frames = range(0, model.clips[clip_idx].max_frame + 1,
                        max(1, (model.clips[clip_idx].max_frame + 1) // 6))
    for f in test_frames:
        # BVH world positions
        bw = _bvh_world(names, offsets, chans, parents, order, frames[f])
        # reference world positions (rot+trans only)
        ref = _model_world_noscale(model, clip_idx, float(f))
        for nm, p in bw.items():
            i = idx.get(nm)
            if i is None:
                continue
            worst = max(worst, math.dist(p, ref[i]))
    return worst


def _model_world_noscale(model, clip_idx, frame):
    n = len(model.joints)
    W = [None] * n
    pos = [None] * n
    for i in range(n):
        j = model.joints[i]
        R, t = model.local_rot_trans(clip_idx, i, frame)
        L = [R[0], R[1], R[2], t[0], R[3], R[4], R[5], t[1], R[6], R[7], R[8], t[2], 0, 0, 0, 1]
        from p3d_core import mul4
        W[i] = L if (i == 0 or not (0 <= j.parent < i)) else mul4(W[j.parent], L)
        pos[i] = (W[i][3], W[i][7], W[i][11])
    return pos


# --- minimal BVH re-parser (validation only) ---
def _parse_bvh_hierarchy(path):
    names, offsets, chans, parents, order = [], [], [], [], []
    stack = []
    with open(path) as fh:
        it = iter(fh)
        for line in it:
            t = line.split()
            if not t:
                continue
            if t[0] in ("ROOT", "JOINT"):
                i = len(names)
                names.append(t[1])
                parents.append(stack[-1] if stack else -1)
                offsets.append((0.0, 0.0, 0.0))
                chans.append([])
                order.append(i)
                stack.append(i)
            elif t[0] == "OFFSET" and stack and stack[-1] >= 0:
                offsets[stack[-1]] = (float(t[1]), float(t[2]), float(t[3]))
            elif t[0] == "CHANNELS" and stack and stack[-1] >= 0:
                chans[stack[-1]] = t[2:]
            elif t[0] == "End":
                stack.append(-999)                 # End Site: skip its OFFSET, no channels
            elif t[0] == "}":
                stack.pop()
            elif t[0] == "MOTION":
                break
    return names, offsets, chans, parents, order


def _parse_bvh_motion(path, order, chans):
    rows = []
    with open(path) as fh:
        started = False
        for line in fh:
            if line.startswith("Frame Time"):
                started = True
                continue
            if started:
                t = line.split()
                if t:
                    rows.append([float(v) for v in t])
    return rows


def _euler_zyx_mat(z, y, x):
    cz, sz = math.cos(z), math.sin(z)
    cy, sy = math.cos(y), math.sin(y)
    cx, sx = math.cos(x), math.sin(x)
    Rz = [cz, -sz, 0, sz, cz, 0, 0, 0, 1]
    Ry = [cy, 0, sy, 0, 1, 0, -sy, 0, cy]
    Rx = [1, 0, 0, 0, cx, -sx, 0, sx, cx]

    def mm(a, b):
        return [sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3)) for r in range(3) for c in range(3)]
    return mm(mm(Rz, Ry), Rx)


def _bvh_world(names, offsets, chans, parents, order, row):
    from p3d_core import mul4
    W = {}
    cur = 0
    D2R = math.pi / 180.0
    for i in order:
        vals = {}
        for ch in chans[i]:
            vals[ch] = row[cur]
            cur += 1
        px = vals.get("Xposition", 0.0); py = vals.get("Yposition", 0.0); pz = vals.get("Zposition", 0.0)
        R = _euler_zyx_mat(vals.get("Zrotation", 0.0) * D2R, vals.get("Yrotation", 0.0) * D2R,
                           vals.get("Xrotation", 0.0) * D2R)
        ox, oy, oz = offsets[i]
        tx, ty, tz = ox + px, oy + py, oz + pz
        L = [R[0], R[1], R[2], tx, R[3], R[4], R[5], ty, R[6], R[7], R[8], tz, 0, 0, 0, 1]
        p = parents[i]
        M = L if p < 0 else mul4(W[p], L)
        W[i] = M
    return {names[i]: (W[i][3], W[i][7], W[i][11]) for i in order}
