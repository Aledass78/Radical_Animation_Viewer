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
                # Blender's BVH importer uses the position channel AS the joint's local translation
                # (it computes `position - rest_head_local`, which cancels the OFFSET), so the channel
                # must carry the FULL local translation. Subtracting the rest offset here sank every
                # animated joint by that offset (the whole body ~Character_Root's offset, plus face /
                # collar / whip / grapple bones by their own). OFFSET still defines the edit-pose rest.
                vals += [t[0], t[1], t[2]]
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
             "bind": [r(v) for v in j.bind],
             # full parent-local REST matrix (16 floats, row-major, DirectX convention) — carries
             # the rest ROTATION that BVH can't, so the Blender addon rebuilds the correct rest pose.
             "local": [r(v) for v in j.local]}
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


# ------------------------------------------------------------------ glTF (.glb)
def _mat3_to_quat(R):
    """Column-vector 3x3 (flat9, row-major R[r*3+c]) -> quaternion (x,y,z,w)."""
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = R
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return (x / n, y / n, z / n, w / n)


def export_gltf(model, clip_idx, path, fps=30):
    """Write clip `clip_idx` to a binary glTF (`.glb`) with the full skeleton (correct rest pose,
    rotation included), a skin, a minimal skinned mesh (so importers build an armature), and the
    animation as rotation/translation/scale samplers. Unlike BVH this keeps the rest rotations and
    scale; unlike our JSON it's a standard format Blender/Unity/three.js import directly.
    Returns the frame count.
    """
    import struct as _st
    joints = model.joints
    clip = model.clips[clip_idx]
    n = len(joints)
    nframes = clip.max_frame + 1

    # ---- world rest matrices (column-vector, row-major flat16) for inverse-bind ----
    from p3d_core import mul4
    world = [None] * n
    for i in range(n):
        L = model._local_col[i]
        p = joints[i].parent
        world[i] = L[:] if (i == 0 or not (0 <= p < i)) else mul4(world[p], L)

    def _inv_rigid(M):                                  # inverse of [R|t] (rigid) -> [R^T | -R^T t]
        R = (M[0], M[1], M[2], M[4], M[5], M[6], M[8], M[9], M[10])
        t = (M[3], M[7], M[11])
        Rt = (R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8])
        it = (-(Rt[0] * t[0] + Rt[1] * t[1] + Rt[2] * t[2]),
              -(Rt[3] * t[0] + Rt[4] * t[1] + Rt[5] * t[2]),
              -(Rt[6] * t[0] + Rt[7] * t[1] + Rt[8] * t[2]))
        return [Rt[0], Rt[1], Rt[2], it[0], Rt[3], Rt[4], Rt[5], it[1], Rt[6], Rt[7], Rt[8], it[2], 0, 0, 0, 1]

    def _colmajor(M):                                   # row-major flat16 -> column-major (glTF)
        return [M[r * 4 + c] for c in range(4) for r in range(4)]

    # ---- binary buffer + accessor builder ----
    bin_data = bytearray()
    bufferViews = []
    accessors = []

    def _accessor(raw, comp_type, atype, count, minmax=None, target=None):
        while len(bin_data) % 4:                         # 4-byte align
            bin_data.append(0)
        off = len(bin_data)
        bin_data.extend(raw)
        bv = {"buffer": 0, "byteOffset": off, "byteLength": len(raw)}
        if target is not None:
            bv["target"] = target
        bufferViews.append(bv)
        acc = {"bufferView": len(bufferViews) - 1, "componentType": comp_type,
               "count": count, "type": atype}
        if minmax:
            acc["min"], acc["max"] = minmax
        accessors.append(acc)
        return len(accessors) - 1

    F = 5126   # FLOAT
    UB = 5121  # UNSIGNED_BYTE
    US = 5123  # UNSIGNED_SHORT

    # inverse bind matrices (MAT4, column-major)
    ibm = bytearray()
    for i in range(n):
        for v in _colmajor(_inv_rigid(world[i])):
            ibm += _st.pack("<f", v)
    ibm_acc = _accessor(ibm, F, "MAT4", n)

    # ---- nodes (bones) with rest TRS ----
    kids = [[] for _ in range(n)]
    roots = []
    for i, j in enumerate(joints):
        if i != 0 and 0 <= j.parent < n and j.parent != i:
            kids[j.parent].append(i)
        else:
            roots.append(i)
    nodes = []
    for i, j in enumerate(joints):
        R, t = model.rest_rot_trans(i)
        q = _mat3_to_quat(R)
        node = {"name": j.name, "translation": [t[0], t[1], t[2]],
                "rotation": [q[0], q[1], q[2], q[3]], "scale": [1.0, 1.0, 1.0]}
        if kids[i]:
            node["children"] = list(kids[i])
        nodes.append(node)

    # ---- minimal skinned mesh (1 triangle at the root, weighted to joint 0) so an armature builds ----
    pos = _st.pack("<9f", 0, 0, 0, 0.001, 0, 0, 0, 0.001, 0)
    pos_acc = _accessor(pos, F, "VEC3", 3, minmax=([0, 0, 0], [0.001, 0.001, 0]), target=34962)
    jnt = _st.pack("<12B", *([0, 0, 0, 0] * 3))
    jnt_acc = _accessor(jnt, UB, "VEC4", 3, target=34962)
    wgt = _st.pack("<12f", *([1, 0, 0, 0] * 3))
    wgt_acc = _accessor(wgt, F, "VEC4", 3, target=34962)
    idx = _st.pack("<3H", 0, 1, 2)
    idx_acc = _accessor(idx, US, "SCALAR", 3, target=34963)
    mesh_node = len(nodes)
    nodes.append({"name": "p3d_skin_mesh", "mesh": 0, "skin": 0})

    # ---- animation samplers/channels ----
    def _slot_channel(i, slot):
        s = clip.channels.get(joints[i].name, {}).get(slot)
        if not s:
            return None
        frames, vals = s
        times = bytearray()
        for f in frames:
            times += _st.pack("<f", f / float(fps or 30))
        tmin, tmax = frames[0] / float(fps or 30), frames[-1] / float(fps or 30)
        in_acc = _accessor(times, F, "SCALAR", len(frames), minmax=([tmin], [tmax]))
        out = bytearray()
        if slot == 'rot':
            for q in vals:
                out += _st.pack("<4f", q[0], q[1], q[2], q[3])
            out_acc = _accessor(out, F, "VEC4", len(frames))
            path_name = "rotation"
        else:
            for v in vals:
                out += _st.pack("<3f", v[0], v[1], v[2])
            out_acc = _accessor(out, F, "VEC3", len(frames))
            path_name = "translation" if slot == 'loc' else "scale"
        return in_acc, out_acc, path_name

    samplers = []
    channels = []
    for i in range(n):
        for slot in ('rot', 'loc', 'scl'):
            r = _slot_channel(i, slot)
            if not r:
                continue
            in_acc, out_acc, path_name = r
            samplers.append({"input": in_acc, "output": out_acc, "interpolation": "LINEAR"})
            channels.append({"sampler": len(samplers) - 1, "target": {"node": i, "path": path_name}})

    gltf = {
        "asset": {"version": "2.0", "generator": "pure3d_anim_viewer"},
        "scene": 0,
        "scenes": [{"nodes": roots + [mesh_node]}],
        "nodes": nodes,
        "meshes": [{"primitives": [{"attributes": {"POSITION": pos_acc, "JOINTS_0": jnt_acc,
                                                   "WEIGHTS_0": wgt_acc}, "indices": idx_acc}]}],
        "skins": [{"inverseBindMatrices": ibm_acc, "joints": list(range(n)), "skeleton": roots[0]}],
        "accessors": accessors,
        "bufferViews": bufferViews,
        "buffers": [{"byteLength": len(bin_data)}],
        "animations": [{"name": clip.name, "samplers": samplers, "channels": channels}],
    }

    # ---- assemble .glb (12-byte header + JSON chunk + BIN chunk) ----
    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    while len(json_bytes) % 4:
        json_bytes += b" "
    while len(bin_data) % 4:
        bin_data.append(0)
    total = 12 + 8 + len(json_bytes) + 8 + len(bin_data)
    with open(path, "wb") as fh:
        fh.write(_st.pack("<III", 0x46546C67, 2, total))           # 'glTF', version 2
        fh.write(_st.pack("<II", len(json_bytes), 0x4E4F534A))      # JSON chunk
        fh.write(json_bytes)
        fh.write(_st.pack("<II", len(bin_data), 0x004E4942))        # BIN chunk
        fh.write(bytes(bin_data))
    return nframes


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
