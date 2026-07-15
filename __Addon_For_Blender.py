"""
Pure3D (Prototype 2) importer for Blender — standalone add-on.

Install: Blender ▸ Edit ▸ Preferences ▸ Add-ons ▸ Install… ▸ pick this file ▸ enable it.
Then:    File ▸ Import ▸ Pure3D Animation (.p3d)
         File ▸ Import ▸ Pure3D JSON (.json)   (exported by pure3d_anim_viewer)

Unlike a BVH, both paths rebuild the skeleton from its **real rest matrices** (rotation included),
so the rest/bind pose matches the game — and they keyframe rotation + translation + **scale** onto
the pose bones. Self-contained: pure Python (struct/zlib/math/json + bpy/mathutils), no external
DLLs, so it runs anywhere Blender does. The .p3d decode is the verified Prototype2/ImportAnim.py
engine; the JSON path consumes pure3d_anim_viewer's own `Export ▸ JSON` (which now carries each
bone's full rest matrix).
"""

bl_info = {
    "name": "Pure3D (Prototype 2) Animation Import",
    "author": "Pure3D tools",
    "version": (1, 0, 0),
    "blender": (2, 93, 0),
    "location": "File > Import > Pure3D Animation (.p3d) / Pure3D JSON (.json)",
    "description": "Import Prototype 2 .p3d and pure3d_anim_viewer .json skeletal animation, "
                   "with the correct rest pose (rotation + translation + scale).",
    "category": "Import-Export",
}

import struct
import zlib
import math
import json

import bpy
import mathutils
from bpy.props import StringProperty, BoolProperty, FloatProperty
from bpy_extras.io_utils import ImportHelper, ExportHelper


# ===========================================================================
#  Minimal pure-Python Pure3D parser
# ===========================================================================
MAGIC_LE = 0xFF443350
MAGIC_BE = 0x503344FF


class Chunk:
    __slots__ = ("chunk_id", "data", "children", "big_endian")

    def __init__(self, chunk_id, data, big_endian):
        self.chunk_id = chunk_id
        self.data = data
        self.children = []
        self.big_endian = big_endian

    def _u(self, fmt, off):
        return struct.unpack_from((">" if self.big_endian else "<") + fmt, self.data, off)[0]

    def read_u16(self, off): return self._u("H", off)
    def read_u32(self, off): return self._u("I", off)
    def read_f32(self, off): return self._u("f", off)

    def read_4cc(self, off):
        raw = self.data[off:off + 4]
        if self.big_endian:
            raw = bytes(reversed(raw))
        return raw.decode("ascii", errors="replace")

    def read_p3d_string(self, off):
        length = self.data[off]
        raw = self.data[off + 1: off + 1 + length]
        s = raw.split(b"\x00")[0].decode("latin-1", errors="replace")
        return s, off + 1 + length

    def find_child(self, chunk_id):
        for c in self.children:
            if c.chunk_id == chunk_id:
                return c
        return None

    def find_children(self, chunk_id):
        return [c for c in self.children if c.chunk_id == chunk_id]


def _parse_chunk(data, off, be):
    rd = ">III" if be else "<III"
    chunk_id, data_size, total_size = struct.unpack_from(rd, data, off)
    own = data[off + 12: off + data_size]
    ch = Chunk(chunk_id, own, be)
    cursor = off + data_size
    end = off + total_size
    while cursor + 12 <= end and cursor + 12 <= len(data):
        _, _, ctot = struct.unpack_from(rd, data, cursor)
        if ctot < 12:
            break
        ch.children.append(_parse_chunk(data, cursor, be))
        cursor += ctot
    return ch


def parse_p3d_bytes(raw):
    if len(raw) < 12:
        return None, None
    magic = struct.unpack_from("<I", raw, 0)[0]
    if magic == MAGIC_LE:
        be = False
    elif magic == MAGIC_BE:
        be = True
    else:
        return None, None
    return _parse_chunk(raw, 0, be), be


def iter_all_chunks(root):
    yield root
    for c in root.children:
        yield from iter_all_chunks(c)


# ===========================================================================
#  Skeleton
# ===========================================================================
class Joint:
    __slots__ = ("name", "parent", "local")

    def __init__(self, name, parent, local):
        self.name = name
        self.parent = parent
        self.local = local        # 16 floats, row-major, local-to-parent


def parse_skeleton(chunk):
    name, off = chunk.read_p3d_string(0)
    joints = []
    for j in chunk.find_children(0x00023001):
        nm, o = j.read_p3d_string(0)
        parent = j.read_u32(o)
        local = [j.read_f32(o + 4 + 4 * i) for i in range(16)]
        joints.append(Joint(nm, parent, local))
    return name, joints


# ===========================================================================
#  Value-channel decoder (verified against Radical's Nixson source)
# ===========================================================================
STATIC_INDEX = ((1, 2), (0, 2), (0, 1))

_CH = {
    0x00121105: ('rot', 12, 8, 16, 3, 'q4'),
    0x00121112: ('rot', 12, 8, 6, 3, 'q6'),
    0x00121114: ('rot', 12, 8, 3, 3, 'q3'),
    0x00121119: ('vec', 12, 8, 6, 3, 'h3'),
    0x00121104: ('vec', 12, 8, 12, 3, 'f3'),
    0x00121118: ('vec', 26, 22, 4, 2, 'h2'),
    0x00121103: ('vec', 26, 22, 8, 2, 'f2'),
    0x00121102: ('vec', 26, 22, 4, 1, 'f1'),
}


def _get_zlib_buffer(anim_hdr):
    for c in anim_hdr.children:
        if c.chunk_id == 0x02F00000 and len(c.data) >= 16:
            comp = c.read_u32(12)
            try:
                return zlib.decompress(c.data[16:16 + comp])
            except zlib.error:
                return None
    return None


def _region_bases(anim_hdr, buf_len=0):
    c6 = anim_hdr.find_child(0x00121006)
    o8 = c6.find_child(0x00121008) if c6 else None
    ex = o8.find_child(0x00121010) if o8 else None
    if ex is not None and len(ex.data) >= 12:
        n = ex.read_u32(8)
        if 0 < n < 64 and len(ex.data) >= 12 + 4 * n:
            bases = [0]
            for i in range(n):
                bases.append(bases[-1] + ex.read_u32(12 + 4 * i))
            if buf_len == 0 or bases[-1] == buf_len:
                return bases
    return [0]


def _read_frames(buf, base, count, be):
    return [struct.unpack_from(">H" if be else "<H", buf, base + 2 * i)[0] for i in range(count)]


def _decode_values(buf, vbase, count, dec, be):
    o = ">" if be else "<"
    fmt = {'q4': 'ffff', 'q6': 'hhh', 'q3': 'bbb', 'h3': 'eee', 'h2': 'ee',
           'f3': 'fff', 'f2': 'ff', 'f1': 'f'}[dec]
    step = {'q4': 16, 'q6': 6, 'q3': 3, 'h3': 6, 'h2': 4, 'f3': 12, 'f2': 8, 'f1': 4}[dec]
    return [struct.unpack_from(o + fmt, buf, vbase + step * i) for i in range(count)]


def _quat(vals, dec):
    if dec == 'q4':
        x, y, z, w = vals
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        return (x / n, y / n, z / n, w / n)
    if dec == 'q6':
        x, y, z = vals[0] / 32767.0, vals[1] / 32767.0, vals[2] / 32767.0
    else:
        x, y, z = vals[0] * 0.007874016, vals[1] * 0.007874016, vals[2] * 0.007874016
    w2 = 1.0 - (x * x + y * y + z * z)
    return (x, y, z, math.sqrt(w2) if w2 > 0 else 0.0)


def _vec3(vals, dof, mapping, base):
    if dof == 3:
        return (float(vals[0]), float(vals[1]), float(vals[2]))
    v = [base[0], base[1], base[2]]
    if dof == 1:
        v[mapping] = float(vals[0])
    else:
        ax = STATIC_INDEX[mapping]
        v[ax[0]] = float(vals[0])
        v[ax[1]] = float(vals[1])
    return tuple(v)


def _slot_of(role_kind, ttype):
    if role_kind == 'rot':
        return 'rot'
    t = ttype.rstrip('\x00')
    return {'TRAN': 'loc', 'SCL': 'scl'}.get(t)


def decode_clip_channels(anim_hdr, want=('rot', 'loc', 'scl')):
    size = anim_hdr.find_child(0x00121002)
    if size is None:
        return {}
    be = anim_hdr.big_endian
    buf = _get_zlib_buffer(anim_hdr)
    bases = _region_bases(anim_hdr, len(buf) if buf is not None else 0)
    out = {}
    for kv in size.find_children(0x00121001):
        try:
            bone, _ = kv.read_p3d_string(4)
        except Exception:
            continue
        slots = {}
        for v in kv.children:
            meta = _CH.get(v.chunk_id)
            if meta is None:
                continue
            role_kind, hlen, nf_off, vbytes, dof, dec = meta
            ttype = v.read_4cc(4)
            slot = _slot_of(role_kind, ttype)
            if slot is None or slot not in want:
                continue
            mapping, base = 0, (0.0, 0.0, 0.0)
            if hlen == 26:
                mapping = v.read_u16(8)
                if not (0 <= mapping <= 2):
                    mapping = 0
                base = (v.read_f32(10), v.read_f32(14), v.read_f32(18))
            try:
                n_inline = v.read_u32(nf_off)
                if n_inline > 0:
                    frames = _read_frames(v.data, hlen, n_inline, be)
                    raw = _decode_values(v.data, hlen + 2 * n_inline, n_inline, dec, be)
                else:
                    ref = next((c for c in v.children if c.chunk_id == 0x00121121), None)
                    if ref is None or buf is None:
                        continue
                    count = ref.read_u32(4)
                    off = ref.read_u32(8)
                    rtag = ref.read_u16(12) if len(ref.data) >= 14 else 0
                    if not (0 <= rtag < len(bases)):
                        continue
                    fbase = bases[rtag] + off
                    vbase = fbase + 2 * count + (2 if count % 2 else 0)
                    if vbase + vbytes * count > len(buf):
                        continue
                    frames = _read_frames(buf, fbase, count, be)
                    raw = _decode_values(buf, vbase, count, dec, be)
            except (struct.error, IndexError):
                continue
            if not frames:
                continue
            if slot == 'rot':
                vals = [_quat(r, dec) for r in raw]
            else:
                vals = [_vec3(r, dof, mapping, base) for r in raw]
            slots[slot] = (frames, vals)
        if slots:
            out[bone] = slots
    return out


def clip_name(anim_hdr):
    try:
        nm, _ = anim_hdr.read_p3d_string(4)
        return nm or "(anim)"
    except Exception:
        return "(anim)"


# ===========================================================================
#  Blender armature from real rest matrices
# ===========================================================================
def _blender_rest_local(joint):
    m = joint.local                                   # row-major; m[r*4+c]
    pos = mathutils.Vector((m[12], m[13], m[14]))
    rot3 = mathutils.Matrix(((m[0], m[4], m[8]),      # transpose DirectX row-major -> column-major
                             (m[1], m[5], m[9]),
                             (m[2], m[6], m[10])))
    rl = rot3.to_4x4()
    rl[0][3], rl[1][3], rl[2][3] = pos.x, pos.y, pos.z
    return rl


def _bvh_tails_and_connect(joints, edit_bones, loc_bones, do_connect=False):
    """Re-shape an already-built armature so it matches a Blender BVH import: each bone's TAIL goes to
    the BVH tail (single child head / average of children / a 0.03*span tip for leaves, with attachment
    stubs collapsed onto the primary child), giving the same bone SIZES as the .bvh. Heads are unchanged,
    so the rest keeps the real in-game positions. When do_connect is on, single-child chains are CONNECTED
    (head == parent tail, and not a translating/loc bone) exactly as Blender's BVH importer does. The
    animation for a connected rig is applied by _apply_clip_bvh (a per-frame aim bake), NOT the plain
    rest-relative delta — a connected bone can't carry a location, so it must be driven by aiming its
    parent at it. Translating bones stay unconnected (BVH does the same)."""
    n = len(joints)
    kids = [[] for _ in range(n)]
    for i, j in enumerate(joints):
        if i != 0 and 0 <= j.parent < n and j.parent != i:
            kids[j.parent].append(i)
    sz = [1] * n
    for i in range(n - 1, -1, -1):
        for c in kids[i]:
            sz[i] += sz[c]
    heads = [edit_bones[i].head.copy() for i in range(n)]
    mn = [min(h[k] for h in heads) for k in range(3)]
    mx = [max(h[k] for h in heads) for k in range(3)]
    span = max(mx[k] - mn[k] for k in range(3)) or 1.0
    tiplen = 0.03 * span
    for i in range(n):
        b = edit_bones[i]
        if not kids[i]:                                       # leaf -> continue the PARENT's direction
            p = joints[i].parent                              # (game +Y is ~90 deg off & mirrored on the R)
            d = (heads[i] - heads[p]) if (0 <= p < n and p != i) else b.matrix.to_3x3().col[1]
            if d.length < 1e-6:
                d = b.matrix.to_3x3().col[1]
            d = d.normalized()
            if d.length < 1e-6:
                d = mathutils.Vector((0.0, 1.0, 0.0))
            b.tail = heads[i] + tiplen * d
        else:
            prim = max(kids[i], key=lambda c: sz[c])
            acc = mathutils.Vector((0.0, 0.0, 0.0))
            for c in kids[i]:
                acc += heads[prim] if (c != prim and sz[c] <= 2 and sz[prim] >= 3 * sz[c]) else heads[c]
            b.tail = acc / len(kids[i])
        if (b.tail - b.head).length < 1e-5:                   # never a zero-length bone
            b.tail = b.head + mathutils.Vector((0.0, tiplen, 0.0))
    if not do_connect:
        return
    for i in range(n):                                        # connect rigid single-child chains (== BVH):
        b = edit_bones[i]                                     # head coincides with the parent's tail and the
        if b.parent is not None and joints[i].name not in loc_bones \
                and (b.head - b.parent.tail).length < 1e-4:   # bone doesn't translate. _apply_clip_bvh then
            b.use_connect = True                              # drives it by aiming the parent at this child.


def build_armature(name, joints, extra_le=b"", bvh_like=False, loc_bones=frozenset()):
    arm = bpy.data.armatures.new("Armature_" + name)
    arm.display_type = 'STICK'
    arm_obj = bpy.data.objects.new(name, arm)
    bpy.context.scene.collection.objects.link(arm_obj)

    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    try:
        edit_bones = arm.edit_bones
        for idx, joint in enumerate(joints):
            rest_local = _blender_rest_local(joint)
            position = rest_local.to_translation()
            bone = edit_bones.new(joint.name)
            if idx == 0 or not (0 <= joint.parent < idx):
                bone.head = position
                bone.matrix = rest_local
            else:
                parent = edit_bones[joint.parent]
                bone.parent = parent
                par3 = parent.matrix.to_3x3()
                bone.head = parent.head + par3 @ position
                R = parent.matrix @ rest_local
                R[0][3], R[1][3], R[2][3] = bone.head
                bone.matrix = R
            bone_dir = bone.matrix.to_3x3().col[1].normalized()
            if bone_dir.length < 1e-6:
                bone_dir = mathutils.Vector((0.0, 1.0, 0.0))
            bone.tail = bone.head + 0.02 * bone_dir
        if bvh_like:                                     # reshape to BVH-style connected/sized bones
            _bvh_tails_and_connect(joints, arm.edit_bones, loc_bones, do_connect=True)
    finally:
        bpy.ops.object.mode_set(mode='OBJECT')

    arm_obj["bonenames"] = [j.name for j in joints]
    if extra_le:                                     # 0x23002/0x23003 region masks — preserve for re-export
        arm_obj["_p3d_skel_extra"] = extra_le.hex()
    return arm_obj


# ===========================================================================
#  Apply decoded clips as Blender Actions
# ===========================================================================
def _rest_local_rel(pbone):
    if pbone.parent is not None:
        return pbone.parent.bone.matrix_local.inverted() @ pbone.bone.matrix_local
    return pbone.bone.matrix_local.copy()


def _apply_clip(arm_obj, name, chanmap, linear=True):
    action = bpy.data.actions.new(name=name)
    action.use_fake_user = True

    if arm_obj.animation_data is None:
        arm_obj.animation_data_create()
    prev_action = arm_obj.animation_data.action
    arm_obj.animation_data.action = action

    max_frame = 1
    pose_bones = arm_obj.pose.bones
    for bone_name, slots in chanmap.items():
        pbone = pose_bones.get(bone_name)
        if pbone is None:
            continue
        rest_rel = _rest_local_rel(pbone)
        rest_q = rest_rel.to_quaternion()
        rqi = rest_q.conjugated()
        rest_l = rest_rel.to_translation()

        rot = slots.get('rot')
        if rot:
            pbone.rotation_mode = 'QUATERNION'
            for f, q in zip(*rot):                     # q = (x,y,z,w)
                pbone.rotation_quaternion = rqi @ mathutils.Quaternion((q[3], q[0], q[1], q[2]))
                bf = f + 1
                pbone.keyframe_insert("rotation_quaternion", frame=bf, group=bone_name)
                max_frame = max(max_frame, bf)

        loc = slots.get('loc')
        if loc:
            for f, t in zip(*loc):
                pbone.location = rqi @ (mathutils.Vector(t) - rest_l)
                bf = f + 1
                pbone.keyframe_insert("location", frame=bf, group=bone_name)
                max_frame = max(max_frame, bf)

        scl = slots.get('scl')
        if scl:
            for f, s in zip(*scl):
                pbone.scale = mathutils.Vector(s)
                bf = f + 1
                pbone.keyframe_insert("scale", frame=bf, group=bone_name)
                max_frame = max(max_frame, bf)

    if linear:
        for fc in action.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = 'LINEAR'

    arm_obj.animation_data.action = prev_action
    return action, max_frame


def _stash_action(arm_obj, action, start=1):
    ad = arm_obj.animation_data
    track = ad.nla_tracks.new()
    track.name = action.name
    strip = track.strips.new(action.name, int(start), action)
    strip.name = action.name
    track.mute = True
    return track


# ---------------------------------------------------------------------------
#  BVH-like connected import: per-frame aim bake
#  Reproduces the .bvh skeleton (same bone sizes + connections) but adds scale and keeps the real
#  in-game rest. Connected bones can't carry a location, so a rest-relative delta can't drive them;
#  instead we FK the game pose each keyframe, aim every bone at its child (a swing that preserves the
#  game twist), and bake pose-bone matrices. Verified 0.0000000 joint-position error incl. scaled clips.
# ---------------------------------------------------------------------------
def _sample_quat_bvh(frames, quats, f):
    if f <= frames[0]:
        q = quats[0]
    elif f >= frames[-1]:
        q = quats[-1]
    else:
        i = 1
        while i < len(frames) and frames[i] < f:
            i += 1
        t = (f - frames[i - 1]) / ((frames[i] - frames[i - 1]) or 1)
        a = mathutils.Quaternion((quats[i - 1][3], quats[i - 1][0], quats[i - 1][1], quats[i - 1][2]))
        b = mathutils.Quaternion((quats[i][3], quats[i][0], quats[i][1], quats[i][2]))
        return a.slerp(b, t)
    return mathutils.Quaternion((q[3], q[0], q[1], q[2]))


def _sample_vec_bvh(frames, vals, f):
    if f <= frames[0]:
        return mathutils.Vector(vals[0])
    if f >= frames[-1]:
        return mathutils.Vector(vals[-1])
    i = 1
    while i < len(frames) and frames[i] < f:
        i += 1
    t = (f - frames[i - 1]) / ((frames[i] - frames[i - 1]) or 1)
    return mathutils.Vector(vals[i - 1]).lerp(mathutils.Vector(vals[i]), t)


def _game_local_matrix(joint, slots, f):
    """Game LOCAL (parent-relative) transform of a bone at frame f as a Matrix: rot (slerp), loc/scl
    (lerp) sampled from its channels, falling back to the rest matrix for any missing slot."""
    rl = _blender_rest_local(joint)
    q = _sample_quat_bvh(slots['rot'][0], slots['rot'][1], f) if (slots and 'rot' in slots) else rl.to_quaternion()
    t = _sample_vec_bvh(slots['loc'][0], slots['loc'][1], f) if (slots and 'loc' in slots) else rl.to_translation()
    s = _sample_vec_bvh(slots['scl'][0], slots['scl'][1], f) if (slots and 'scl' in slots) else mathutils.Vector((1.0, 1.0, 1.0))
    return mathutils.Matrix.LocRotScale(t, q, s)


def _decimate_keys(frames, values, tol, kind):
    """Greedily keep the minimal subset of keyframe indices so interpolating between the kept keys
    stays within `tol` of every original value. kind='quat' -> slerp + angle error (radians);
    'vec' -> lerp + distance. Returns sorted indices into `frames`. (Douglas-Peucker-ish forward fit.)"""
    n = len(frames)
    if n <= 2 or tol <= 0.0:
        return list(range(n))
    fit = (lambda a, b, t: a.slerp(b, t)) if kind == 'quat' else (lambda a, b, t: a.lerp(b, t))
    err = ((lambda ap, tr: ap.rotation_difference(tr).angle) if kind == 'quat'
           else (lambda ap, tr: (ap - tr).length))
    keep = [0]
    anchor = 0
    j = 2
    while j < n:
        fa = frames[anchor]
        span = (frames[j] - fa) or 1
        ok = True
        for k in range(anchor + 1, j):
            if err(fit(values[anchor], values[j], (frames[k] - fa) / span), values[k]) > tol:
                ok = False
                break
        if ok:
            j += 1
        else:
            keep.append(j - 1)
            anchor = j - 1
            j = anchor + 2
    if keep[-1] != n - 1:
        keep.append(n - 1)
    return keep


def _apply_clip_bvh(arm_obj, name, chanmap, joints, linear=True, decimate=False, decimate_tol=0.001):
    """Bake a clip onto a BVH-like (connected) armature. If `decimate`, prune keyframes that
    interpolation reproduces within `decimate_tol` metres of the exact bake, measured as world-position
    impact (a bone's rotation/scale error is scaled by its reach to the farthest descendant; a location
    error moves its whole subtree 1:1). Returns (action, max_frame)."""
    n = len(joints)
    par = [j.parent for j in joints]
    names = [j.name for j in joints]
    kids = [[] for _ in range(n)]
    for i in range(n):
        if i and 0 <= par[i] < n and par[i] != i:
            kids[par[i]].append(i)
    sz = [1] * n
    for i in range(n - 1, -1, -1):
        for c in kids[i]:
            sz[i] += sz[c]
    primary = [max(kids[i], key=lambda c: sz[c]) if kids[i] else -1 for i in range(n)]

    frames = set()
    for slots in chanmap.values():
        for fr, _v in slots.values():
            frames.update(fr)
    frames = sorted(frames) or [0]
    nf = len(frames)

    action = bpy.data.actions.new(name=name)
    action.use_fake_user = True
    if arm_obj.animation_data is None:
        arm_obj.animation_data_create()
    prev_action = arm_obj.animation_data.action
    arm_obj.animation_data.action = action

    pose_bones = arm_obj.pose.bones
    pbs = [pose_bones.get(nm) for nm in names]
    restrel = [(_rest_local_rel(pb) if pb else None) for pb in pbs]
    restrel_inv = [(rr.inverted() if rr is not None else None) for rr in restrel]
    slots_by_i = [chanmap.get(nm) for nm in names]
    upY = mathutils.Vector((0.0, 1.0, 0.0))
    zero = mathutils.Vector((0.0, 0.0, 0.0))

    # ---- phase 1: compute the full per-bone / per-frame pose-bone basis (decomposed loc/rot/scl) ----
    ROT = [[None] * nf for _ in range(n)]
    LOC = [[None] * nf for _ in range(n)]
    SCL = [[None] * nf for _ in range(n)]
    for fi, f in enumerate(frames):
        ff = float(f)
        G = [None] * n                                        # game world matrices (FK)
        for i in range(n):
            L = _game_local_matrix(joints[i], slots_by_i[i], ff)
            G[i] = L if (i == 0 or not (0 <= par[i] < i)) else G[par[i]] @ L
        W = [None] * n                                        # aimed target world (Y -> tail, keep twist)
        for i in range(n):
            loc, rot, scl = G[i].decompose()
            gY = rot.to_matrix().col[1]
            # aim at the SAME target the rest tail uses (so a multi-child bone e.g. Pelvis points
            # between its children, not at one of them): average of children with attachment stubs
            # collapsed onto the primary; the single child; or, for a leaf, the parent continuation.
            if kids[i]:
                prim = primary[i]
                acc = mathutils.Vector((0.0, 0.0, 0.0))
                for c in kids[i]:
                    acc += (G[prim].to_translation() if (c != prim and sz[c] <= 2 and sz[prim] >= 3 * sz[c])
                            else G[c].to_translation())
                aim = acc / len(kids[i]) - loc
            else:
                p = par[i]
                aim = loc - (G[p].to_translation() if (0 <= p < n and p != i) else (loc - upY))
            if aim.length < 1e-9:
                aim = gY
            W[i] = mathutils.Matrix.LocRotScale(loc, gY.rotation_difference(aim) @ rot, scl)
        P = [None] * n                                        # reconstructed pose (== Blender eval)
        for i in range(n):
            pb = pbs[i]
            if pb is None:
                P[i] = P[par[i]] if (0 <= par[i] < i and P[par[i]] is not None) else W[i]
                continue
            if i == 0 or not (0 <= par[i] < i):
                basis = restrel_inv[i] @ W[i]
            else:
                basis = restrel_inv[i] @ (P[par[i]].inverted() @ W[i])
            # Blender stores loc/rot/scale keys (drops any shear); re-compose so P mirrors what Blender
            # will actually evaluate. Connected bones ignore basis location -> zero it.
            bl, br, bsc = basis.decompose()
            if pb.bone.use_connect:
                bl = zero
            LOC[i][fi], ROT[i][fi], SCL[i][fi] = bl.copy(), br, bsc
            comp = mathutils.Matrix.LocRotScale(bl, br, bsc)
            P[i] = (restrel[i] @ comp) if (i == 0 or not (0 <= par[i] < i)) else P[par[i]] @ restrel[i] @ comp

    # ---- reach: max rest distance from each bone to any descendant (scales rotation/scale error) ----
    heads = [(pbs[i].bone.head_local.copy() if pbs[i] else None) for i in range(n)]

    def _reach(i):
        if heads[i] is None:
            return 0.0
        mx = 0.0
        stack = list(kids[i])
        while stack:
            c = stack.pop()
            if heads[c] is not None:
                mx = max(mx, (heads[c] - heads[i]).length)
            stack.extend(kids[c])
        return mx

    # ---- phase 2 + 3: (decimate then) insert keyframes, one channel at a time ----
    all_idx = list(range(nf))
    max_frame = 1
    for i in range(n):
        pb = pbs[i]
        if pb is None:
            continue
        pb.rotation_mode = 'QUATERNION'
        if decimate:
            r = max(_reach(i), 1e-3)
            rot_keep = _decimate_keys(frames, ROT[i], decimate_tol / r, 'quat')
            scl_keep = _decimate_keys(frames, SCL[i], decimate_tol / r, 'vec')
            loc_keep = _decimate_keys(frames, LOC[i], decimate_tol, 'vec')
        else:
            rot_keep = scl_keep = loc_keep = all_idx
        for fi in rot_keep:
            pb.rotation_quaternion = ROT[i][fi]
            bf = frames[fi] + 1
            pb.keyframe_insert("rotation_quaternion", frame=bf, group=names[i])
            max_frame = max(max_frame, bf)
        for fi in scl_keep:
            pb.scale = SCL[i][fi]
            pb.keyframe_insert("scale", frame=frames[fi] + 1, group=names[i])
        if not pb.bone.use_connect:
            for fi in loc_keep:
                pb.location = LOC[i][fi]
                pb.keyframe_insert("location", frame=frames[fi] + 1, group=names[i])

    if linear:
        for fc in action.fcurves:
            for kp in fc.keyframe_points:
                kp.interpolation = 'LINEAR'
    arm_obj.animation_data.action = prev_action
    return action, max_frame


def _install_clips(operator, arm_obj, clips, fps, set_first_active=True, bvh_joints=None,
                   decimate=False, decimate_tol=0.001):
    """clips = list of (name, chanmap). Builds one Action per clip; returns count."""
    if arm_obj.animation_data is None:
        arm_obj.animation_data_create()
    made = 0
    first_action = None
    total_max = 1
    used = set()
    for base, chanmap in clips:
        if not chanmap:
            continue
        nm = base
        i = 1
        while nm in used:
            i += 1
            nm = "%s.%03d" % (base, i)
        used.add(nm)
        if bvh_joints is not None:
            action, mx = _apply_clip_bvh(arm_obj, nm, chanmap, bvh_joints,
                                         decimate=decimate, decimate_tol=decimate_tol)
        else:
            action, mx = _apply_clip(arm_obj, nm, chanmap)
        _stash_action(arm_obj, action)
        total_max = max(total_max, mx)
        if first_action is None:
            first_action = action
        made += 1

    if set_first_active and first_action is not None:
        arm_obj.animation_data.action = first_action

    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = max(scene.frame_end, total_max)
    scene.frame_current = 1
    try:
        scene.render.fps = int(fps)
    except Exception:
        pass
    return made


# ===========================================================================
#  Import entry points
# ===========================================================================
def import_p3d(operator, filepath, fps, import_translation, import_scale, bvh_like=False,
               decimate=False, decimate_tol=0.001):
    want = ['rot']
    if import_translation:
        want.append('loc')
    if import_scale:
        want.append('scl')
    want = tuple(want)
    try:
        with open(filepath, "rb") as f:
            raw = f.read()
    except OSError:
        operator.report({'ERROR'}, "Could not open file.")
        return {'CANCELLED'}
    root, _be = parse_p3d_bytes(raw)
    if root is None:
        operator.report({'ERROR'}, "Not a valid Pure3D (.p3d) file.")
        return {'CANCELLED'}

    # A .p3d can hold SEVERAL skeletons (e.g. a character + a prop/tricorder) and hundreds of clips.
    # Build one armature per skeleton, then route each clip to the skeleton whose bones it drives
    # (best bone-name coverage) — so multi-skeleton files import both and assign animations right.
    skels = []                                         # (name, joints, extra_le_bytes)
    for c in iter_all_chunks(root):
        if c.chunk_id != 0x00023000:
            continue
        nm, js = parse_skeleton(c)
        if not js:
            continue
        # preserve the skeleton's bone-group masks (0x23002) + leg mirror map (0x23003) verbatim (LE)
        extra = b"".join(_region_chunk_le(k) for k in c.children
                         if k.chunk_id in (0x00023002, 0x00023003))
        skels.append((nm, js, extra))
    if not skels:
        operator.report({'ERROR'}, "No skeleton (0x00023000) in file.")
        return {'CANCELLED'}
    # Route clips to skeletons FIRST (by bone-name coverage) — the BVH-like build needs to know which
    # bones translate (loc) so it leaves them unconnected (a connected bone can't move in Blender).
    skel_names = [set(j.name for j in js) for _nm, js, _ex in skels]
    anims = [c for c in iter_all_chunks(root) if c.chunk_id == 0x00121000
             and c.find_child(0x00121002) is not None]
    buckets = [[] for _ in skels]
    unrouted = 0
    for a in anims:
        chanmap = decode_clip_channels(a, want=want)
        if not chanmap:
            continue
        clip_bones = set(chanmap)
        scored = [(len(clip_bones & skel_names[i]), -i) for i in range(len(skels))]
        cov, negi = max(scored)
        if cov == 0:                                   # e.g. camera-only clips -> no skeleton
            unrouted += 1
            continue
        buckets[-negi].append((clip_name(a), chanmap))

    armatures = []                                     # (arm_obj, set(bone_names), joints)
    for (nm, joints, extra), bucket, names in zip(skels, buckets, skel_names):
        loc_bones = frozenset(b for _cn, cm in bucket for b, sl in cm.items() if 'loc' in sl) \
            if bvh_like else frozenset()
        armatures.append((build_armature(nm, joints, extra, bvh_like=bvh_like, loc_bones=loc_bones),
                          names, joints))

    total = 0
    for (arm, _names, joints), bucket in zip(armatures, buckets):
        total += _install_clips(operator, arm, bucket, fps, bvh_joints=joints if bvh_like else None,
                                decimate=decimate and bvh_like, decimate_tol=decimate_tol)
    msg = "Imported %d skeleton(s) + %d clip(s) from .p3d." % (len(armatures), total)
    if unrouted:
        msg += " (%d clip(s) matched no skeleton, skipped.)" % unrouted
    operator.report({'INFO'}, msg)
    return {'FINISHED'}


def import_json(operator, filepath, fps, import_translation, import_scale, bvh_like=False,
                decimate=False, decimate_tol=0.001):
    try:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        operator.report({'ERROR'}, "Could not read JSON: %s" % e)
        return {'CANCELLED'}
    sk = data.get("skeleton", {})
    jl = sk.get("joints", [])
    if not jl:
        operator.report({'ERROR'}, "JSON has no skeleton joints.")
        return {'CANCELLED'}
    joints = []
    for j in jl:
        local = j.get("local")
        if local is None or len(local) < 16:
            operator.report({'ERROR'}, "JSON joints lack full rest matrices ('local'); "
                                       "re-export from pure3d_anim_viewer.")
            return {'CANCELLED'}
        joints.append(Joint(j["name"], int(j.get("parent", -1)), [float(x) for x in local]))
    clip = data.get("clip", {})
    chan = clip.get("channels", {})
    want = {'rot'}
    if import_translation:
        want.add('loc')
    if import_scale:
        want.add('scl')
    chanmap = {}
    for bone, slots in chan.items():
        sd = {}
        for slot, cur in slots.items():
            if slot not in want:
                continue
            frames = [int(x) for x in cur.get("frames", [])]
            vals = [tuple(float(c) for c in v) for v in cur.get("values", [])]
            if frames and vals:
                sd[slot] = (frames, vals)
        if sd:
            chanmap[bone] = sd
    loc_bones = frozenset(b for b, sl in chanmap.items() if 'loc' in sl) if bvh_like else frozenset()
    arm_obj = build_armature(sk.get("name", "Pure3D"), joints, bvh_like=bvh_like, loc_bones=loc_bones)
    made = _install_clips(operator, arm_obj, [(clip.get("name", "clip"), chanmap)],
                          clip.get("fps", fps), bvh_joints=joints if bvh_like else None,
                          decimate=decimate and bvh_like, decimate_tol=decimate_tol)
    operator.report({'INFO'}, "Imported skeleton '%s' + %d clip from JSON."
                    % (arm_obj.name, made))
    return {'FINISHED'}


# ===========================================================================
#  Pure3D WRITER (export) — clips are byte-exact (verified); skeleton is readable by these tools
# ===========================================================================
def _u32w(v, be):
    return struct.pack(">I" if be else "<I", v & 0xFFFFFFFF)


def _f32w(v, be):
    return struct.pack(">f" if be else "<f", v)


def _serialize(cid, payload, children, be):
    body = bytes(payload) + b"".join(children)
    return struct.pack(">III" if be else "<III", cid, 12 + len(payload), 12 + len(body)) + body


def _aligned_str(s):
    b = s.encode("latin-1")
    pad = (4 - (len(b) % 4)) % 4
    return bytes([len(b) + pad]) + b + b"\x00" * pad


def _quat_to_q6(q):                                    # (x,y,z,w) -> raw s16 triple (w>=0)
    x, y, z, w = q
    if w < 0:
        x, y, z = -x, -y, -z
    return (int(round(x * 32767)), int(round(y * 32767)), int(round(z * 32767)))


def _quat_to_q3(q):                                    # (x,y,z,w) -> raw s8 triple (w>=0)
    x, y, z, w = q
    if w < 0:
        x, y, z = -x, -y, -z
    clamp = lambda v: max(-127, min(127, int(round(v * 127))))
    return (clamp(x), clamp(y), clamp(z))


# Rotation encodings by chunk id: (bytes/frame, is-quantized). Quat4 = full float (lossless);
# Quat6 = 16-bit (game default); Quat3 = 8-bit (smallest). All decode to a unit quaternion (w>=0).
_ROT_Q4 = 0x00121105
_ROT_Q6 = 0x00121112
_ROT_Q3 = 0x00121114
_ROT_BYTES = {_ROT_Q4: 16, _ROT_Q6: 6, _ROT_Q3: 3}


def _rot_roundtrip(q, cid):
    """Encode a quaternion at `cid`'s precision and decode it back (w reconstructed, w>=0 convention)."""
    x, y, z, w = q
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    if cid == _ROT_Q4:
        return (x, y, z, w)
    s = 32767.0 if cid == _ROT_Q6 else 127.0
    lim = 32767 if cid == _ROT_Q6 else 127
    xi = max(-lim, min(lim, int(round(x * s)))) / s
    yi = max(-lim, min(lim, int(round(y * s)))) / s
    zi = max(-lim, min(lim, int(round(z * s)))) / s
    wr = math.sqrt(max(0.0, 1.0 - xi * xi - yi * yi - zi * zi))
    return (xi, yi, zi, wr)


def _rot_angle_err(values, cid):
    """Max rotation-angle error (radians) introduced by encoding `values` at `cid`'s precision."""
    worst = 0.0
    for q in values:
        d = _rot_roundtrip(q, cid)
        dot = abs(q[0] * d[0] + q[1] * d[1] + q[2] * d[2] + q[3] * d[3])
        worst = max(worst, 2.0 * math.acos(max(0.0, min(1.0, dot))))
    return worst


_PREC_BY_NAME = {'q3': _ROT_Q3, 'q6': _ROT_Q6, 'q4': _ROT_Q4}


def _parse_precision_manual(text):
    """'Pelvis=q4, Index_L=q3' -> {bone: chunk_id}. Unrecognised entries are ignored."""
    out = {}
    for part in (text or "").replace(";", ",").split(","):
        name, sep, p = part.partition("=")
        cid = _PREC_BY_NAME.get(p.strip().lower())
        if sep and cid:
            out[name.strip()] = cid
    return out


def _pick_rot_cids(order_names, children, heads, channels, mode, tol, manual):
    """Decide the rotation encoding per bone. AUTO picks the SMALLEST of Quat3/Quat6/Quat4 whose
    quantization moves the bone's farthest descendant by <= `tol` (a rotation error of a radians
    displaces a joint `reach` units away by ~reach*a). So still/leaf bones get Quat3, the root/forks
    that swing a whole limb get Quat4, most bones stay Quat6."""
    def reach(nm):                                     # max rest distance to any descendant head
        h = heads[nm]; mx = 0.0; stack = list(children.get(nm, ()))
        while stack:
            c = stack.pop(); mx = max(mx, (heads[c] - h).length); stack.extend(children.get(c, ()))
        return mx
    out = {}
    for nm in order_names:
        slots = channels.get(nm)
        if not slots or 'rot' not in slots:
            continue
        if mode == 'NONE':
            out[nm] = _ROT_Q6
        elif mode == 'FULL':
            out[nm] = _ROT_Q4
        elif mode == 'MANUAL':
            out[nm] = manual.get(nm, _ROT_Q6)
        else:                                          # AUTO
            r = max(reach(nm), 1e-4)
            out[nm] = _ROT_Q4
            for cid in (_ROT_Q3, _ROT_Q6):
                if r * _rot_angle_err(slots['rot'][1], cid) <= tol:
                    out[nm] = cid
                    break
    return out


def _encode_channel(cid, ttype, frames, values, be):
    tt = ttype.encode("ascii")[:4].ljust(4, b"\x00")
    if be:
        tt = bytes(reversed(tt))
    ho = ">H" if be else "<H"
    header = _u32w(0, be) + tt + _u32w(len(frames), be)
    fb = b"".join(struct.pack(ho, f & 0xFFFF) for f in frames)
    o = ">" if be else "<"
    if cid == _ROT_Q6:                                 # Quaternion6 (16-bit x,y,z ; w reconstructed)
        vb = b"".join(struct.pack(o + "hhh", *_quat_to_q6(v)) for v in values)
    elif cid == _ROT_Q3:                               # Quaternion3 (8-bit x,y,z ; w reconstructed)
        vb = b"".join(struct.pack(o + "bbb", *_quat_to_q3(v)) for v in values)
    elif cid == _ROT_Q4:                               # Quaternion (full float x,y,z,w)
        vb = b"".join(struct.pack(o + "ffff", v[0], v[1], v[2], v[3]) for v in values)
    else:                                              # 0x121104 Vector3DOF f32 (TRAN/SCL)
        vb = b"".join(struct.pack(o + "fff", v[0], v[1], v[2]) for v in values)
    return _serialize(cid, header + fb + vb, [], be)


def _build_header(type_keys, nbones, be):
    ho = ">H" if be else "<H"
    entries = []
    for cid in sorted(type_keys):
        kc = type_keys[cid]
        payload = (_u32w(0, be) + _u32w(cid, be) + _u32w(len(kc), be)
                   + b"".join(struct.pack(ho, k & 0xFFFF) for k in kc))
        entries.append(_serialize(0x00121007, payload, [], be))
    return _serialize(0x00121006, _u32w(0, be) + _u32w(nbones, be), entries, be)


def build_clip(name, order, channels, num_frames, fps, be, rot_cids=None):
    """0x121000 subtree from {bone:{'rot':(fr,quats),'loc':(fr,xyz),'scl':(fr,xyz)}}. `rot_cids` maps a
    bone name to the rotation chunk id to use (Quat3/Quat6/Quat4); default Quat6 (the game's format)."""
    rot_cids = rot_cids or {}
    bone_chunks = []
    type_keys = {}
    for bone in order:
        slots = channels.get(bone)
        if not slots:
            continue
        chan = []
        if 'rot' in slots:
            fr, q = slots['rot']
            rcid = rot_cids.get(bone, _ROT_Q6)
            chan.append(_encode_channel(rcid, "ROT\0", fr, q, be))
            type_keys.setdefault(rcid, []).append(len(fr))
        if 'loc' in slots:
            fr, v = slots['loc']
            chan.append(_encode_channel(0x00121104, "TRAN", fr, v, be))
            type_keys.setdefault(0x00121104, []).append(len(fr))
        if 'scl' in slots:
            fr, v = slots['scl']
            chan.append(_encode_channel(0x00121104, "SCL\0", fr, v, be))
            type_keys.setdefault(0x00121104, []).append(len(fr))
        if not chan:
            continue
        payload = _u32w(0, be) + _aligned_str(bone) + _u32w(0, be) + _u32w(len(chan), be)
        bone_chunks.append(_serialize(0x00121001, payload, chan, be))
    nbones = len(bone_chunks)
    header = _build_header(type_keys, nbones, be)
    bonelist = _serialize(0x00121002, _u32w(0, be) + _u32w(nbones, be), bone_chunks, be)
    animtime = _serialize(0x00121402, _u32w(0, be) + _f32w(num_frames / float(fps or 30), be), [], be)
    tt = b"PTRN"
    if be:
        tt = bytes(reversed(tt))
    ap = (_u32w(0, be) + _aligned_str(name) + tt
          + _f32w(float(num_frames), be) + _f32w(float(fps), be) + _u32w(0, be))
    return _serialize(0x00121000, ap, [header, bonelist, animtime], be)


def _region_chunk_le(chunk):
    """Re-serialise a leaf skeleton region chunk (0x23002 bone-group mask / 0x23003 leg mirror) as
    little-endian bytes for the (always-LE) export. Only LE sources are preserved: the payload is a
    name + joint bitmask, and PS3/BE files pad the name differently, so a byte-swap can't be done
    reliably without per-field parsing — safer to skip BE than to emit a wrong mask. Returns b'' for
    BE sources."""
    if chunk.big_endian:
        return b""
    return _serialize(chunk.chunk_id, chunk.data, [], False)   # verbatim (LE -> LE)


def write_skeleton(name, joints, be, extra=b""):
    """joints = [(name, parent_index, local16_row_major)]. `extra` = already-serialised leaf sub-chunks
    to preserve verbatim under the skeleton (e.g. 0x23002 bone-group masks / 0x23003 leg mirror map,
    which the game uses for partial-body blending / mirroring / leg IK)."""
    jchunks = []
    for jn, par, local in joints:
        payload = _aligned_str(jn) + _u32w(par, be) + b"".join(_f32w(x, be) for x in local)
        jchunks.append(_serialize(0x00023001, payload, [], be))
    sk = _aligned_str(name) + _u32w(1, be) + _u32w(len(joints), be) + _u32w(0, be) + _u32w(0, be)
    return _serialize(0x00023000, sk, jchunks + ([extra] if extra else []), be)


def write_p3d(top_chunks, be=False):
    return _serialize(0xFF443350, b"", top_chunks, be)


# ---- extract skeleton + animation from a Blender armature ----
def _bone_order(arm_obj):
    """Bones parent-first, with each bone's parent index in the list (root -> its own index)."""
    order = []
    index = {}

    def visit(bone):
        i = len(order)
        index[bone.name] = i
        order.append(bone)
        for c in bone.children:
            visit(c)
    for b in arm_obj.data.bones:
        if b.parent is None:
            visit(b)
    return order, index


def _game_local_from_rest(rest_rel):
    """Blender parent-local rest matrix (column-vector) -> game row-major 16 (inverse of import)."""
    Rb = rest_rel.to_3x3()
    t = rest_rel.to_translation()
    return [Rb[0][0], Rb[1][0], Rb[2][0], 0.0,
            Rb[0][1], Rb[1][1], Rb[2][1], 0.0,
            Rb[0][2], Rb[1][2], Rb[2][2], 0.0,
            t.x, t.y, t.z, 1.0]


def _export_armature(arm_obj, be, import_translation, import_scale,
                     prec_mode='AUTO', prec_tol=0.001, prec_manual=""):
    """-> (skeleton_bytes, [clip_bytes...]) for one armature and its NLA/active actions."""
    order, index = _bone_order(arm_obj)
    # rest heads + child map, for the auto rotation-precision reach calc
    heads = {b.name: b.head_local.copy() for b in order}
    children = {}
    for b in order:
        if b.parent is not None:
            children.setdefault(b.parent.name, []).append(b.name)
    manual_map = _parse_precision_manual(prec_manual)
    # skeleton
    pbones = arm_obj.pose.bones
    joints = []
    for b in order:
        pb = pbones.get(b.name)
        rest_rel = _rest_local_rel(pb)
        par = index[b.parent.name] if b.parent is not None else 0
        joints.append((b.name, par, _game_local_from_rest(rest_rel)))
    # Re-attach preserved region masks (0x23002/0x23003) ONLY when the joint ORDER is unchanged —
    # their bitmasks are index-based, so a reordered/edited skeleton would make them point at the
    # wrong bones. `bonenames` was stashed at import in the original order.
    extra = b""
    stash = arm_obj.get("_p3d_skel_extra", "")
    if stash and list(arm_obj.get("bonenames", [])) == [b.name for b in order]:
        try:
            extra = bytes.fromhex(stash)
        except ValueError:
            extra = b""
    sk_bytes = write_skeleton(arm_obj.name, joints, be, extra)
    bone_names = [b.name for b in order]

    # collect actions: everything stashed on NLA + the active one
    actions = []
    ad = arm_obj.animation_data
    seen = set()
    if ad:
        for tr in ad.nla_tracks:
            for strip in tr.strips:
                if strip.action and strip.action.name not in seen:
                    seen.add(strip.action.name)
                    actions.append(strip.action)
        if ad.action and ad.action.name not in seen:
            seen.add(ad.action.name)
            actions.append(ad.action)

    scene = bpy.context.scene
    saved_action = ad.action if ad else None
    saved_frame = scene.frame_current
    clips = []
    for action in actions:
        rng = action.frame_range
        f0, f1 = int(round(rng[0])), int(round(rng[1]))
        # which bone+slot are keyed, and at exactly which frames (so a decimated/sparse action exports
        # sparse — not re-baked to every frame, which used to bloat the file and ignore decimation)
        keyed = {}                                     # bone -> set('rot','loc','scl')
        kf = {}                                        # (bone, slot) -> set(frame ints)
        for fc in action.fcurves:
            dp = fc.data_path
            if not dp.startswith('pose.bones["'):
                continue
            bn = dp[len('pose.bones["'):dp.index('"]')]
            if dp.endswith("rotation_quaternion"):
                slot = 'rot'
            elif dp.endswith("location") and import_translation:
                slot = 'loc'
            elif dp.endswith("scale") and import_scale:
                slot = 'scl'
            else:
                continue
            keyed.setdefault(bn, set()).add(slot)
            times = kf.setdefault((bn, slot), set())
            for kp in fc.keyframe_points:
                times.add(int(round(kp.co[0])))
        if not keyed:
            continue
        if ad:
            ad.action = action
        channels = {b: {s: ([], []) for s in slots} for b, slots in keyed.items()}
        visit = sorted({f for times in kf.values() for f in times if f0 <= f <= f1})
        for f in visit:
            scene.frame_set(f)
            gf = f - f0                                # game frame index starts at 0
            for bn, slots in keyed.items():
                pb = pbones.get(bn)
                if pb is None:
                    continue
                rest_rel = _rest_local_rel(pb)
                rest_q = rest_rel.to_quaternion()
                rest_l = rest_rel.to_translation()
                # decompose the pose transform so this works whatever the bone's rotation_mode is.
                # These are the exact inverse of the import's rest_q^-1 conversions.
                p_loc, p_rot, p_scl = pb.matrix_basis.decompose()
                if 'rot' in slots and f in kf[(bn, 'rot')]:
                    rq = rest_q @ p_rot
                    channels[bn]['rot'][0].append(gf)
                    channels[bn]['rot'][1].append((rq.x, rq.y, rq.z, rq.w))
                if 'loc' in slots and f in kf[(bn, 'loc')]:
                    rl = rest_q @ p_loc + rest_l
                    channels[bn]['loc'][0].append(gf)
                    channels[bn]['loc'][1].append((rl.x, rl.y, rl.z))
                if 'scl' in slots and f in kf[(bn, 'scl')]:
                    channels[bn]['scl'][0].append(gf)
                    channels[bn]['scl'][1].append((p_scl.x, p_scl.y, p_scl.z))
        nframes = f1 - f0 + 1
        rot_cids = _pick_rot_cids(bone_names, children, heads, channels,
                                  prec_mode, prec_tol, manual_map)
        clips.append(build_clip(action.name, bone_names, channels, nframes,
                                scene.render.fps, be, rot_cids=rot_cids))
    if ad:
        ad.action = saved_action
    scene.frame_set(saved_frame)
    return sk_bytes, clips


def export_p3d(operator, filepath, import_translation, import_scale,
               prec_mode='AUTO', prec_tol=0.001, prec_manual=""):
    arms = [o for o in bpy.context.scene.objects if o.type == 'ARMATURE']
    sel = [o for o in arms if o.select_get()]
    if sel:
        arms = sel
    if not arms:
        operator.report({'ERROR'}, "No armature to export.")
        return {'CANCELLED'}
    top = []
    nclips = 0
    for arm in arms:
        sk, clips = _export_armature(arm, False, import_translation, import_scale,
                                     prec_mode, prec_tol, prec_manual)
        top.append(sk)
        top.extend(clips)
        nclips += len(clips)
    data = write_p3d(top, be=False)
    try:
        with open(filepath, "wb") as f:
            f.write(data)
    except OSError as e:
        operator.report({'ERROR'}, "Write failed: %s" % e)
        return {'CANCELLED'}
    operator.report({'INFO'}, "Wrote %d skeleton(s) + %d clip(s) to .p3d." % (len(arms), nclips))
    return {'FINISHED'}


# ===========================================================================
#  Operators + menu
# ===========================================================================
class _CommonProps:
    fps: FloatProperty(name="FPS", default=30.0, min=1.0, max=240.0,
                       description="Scene frame rate for the imported clip(s)")
    import_translation: BoolProperty(name="Import Translation", default=True,
                                     description="Keyframe TRAN channels onto bone location")
    import_scale: BoolProperty(name="Import Scale", default=True,
                               description="Keyframe SCL channels onto bone scale")
    bvh_like: BoolProperty(name="BVH-like bones", default=False,
                           description="Build the armature like a BVH import: bones aim at their "
                                       "children (connected octahedral bones) with BVH bone sizes, "
                                       "instead of the game's rest orientation with tiny stub tails")
    decimate: BoolProperty(name="Decimate keyframes", default=False,
                           description="BVH-like only: connecting bones bakes a key on every frame; "
                                       "this prunes keys that interpolation can reproduce within the "
                                       "tolerance below, shrinking the animation. Off = exact bake")
    decimate_tol: FloatProperty(name="Decimate Tolerance (m)", default=0.001, min=0.0, max=1.0,
                                precision=5,
                                description="Max distance any joint may move from dropping a keyframe "
                                            "(e.g. 0.001 = 1 mm)")

    def draw(self, context):
        col = self.layout.column()
        col.prop(self, "fps")
        col.prop(self, "import_translation")
        col.prop(self, "import_scale")
        col.prop(self, "bvh_like")
        if self.bvh_like:                              # decimation only applies to the BVH-like bake
            box = col.box()
            box.prop(self, "decimate")
            if self.decimate:
                box.prop(self, "decimate_tol")


class IMPORT_OT_pure3d_p3d(bpy.types.Operator, ImportHelper, _CommonProps):
    bl_idname = "import_scene.pure3d_p3d"
    bl_label = "Import Pure3D Animation (.p3d)"
    bl_options = {'REGISTER', 'UNDO'}
    filename_ext = ".p3d"
    filter_glob: StringProperty(default="*.p3d", options={'HIDDEN'})

    def execute(self, context):
        return import_p3d(self, self.filepath, self.fps, self.import_translation, self.import_scale,
                          self.bvh_like, self.decimate, self.decimate_tol)


class IMPORT_OT_pure3d_json(bpy.types.Operator, ImportHelper, _CommonProps):
    bl_idname = "import_scene.pure3d_json"
    bl_label = "Import Pure3D JSON (.json)"
    bl_options = {'REGISTER', 'UNDO'}
    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        return import_json(self, self.filepath, self.fps, self.import_translation, self.import_scale,
                           self.bvh_like, self.decimate, self.decimate_tol)


class EXPORT_OT_pure3d_p3d(bpy.types.Operator, ExportHelper):
    bl_idname = "export_scene.pure3d_p3d"
    bl_label = "Export Pure3D Animation (.p3d)"
    bl_options = {'REGISTER'}
    filename_ext = ".p3d"
    filter_glob: StringProperty(default="*.p3d", options={'HIDDEN'})

    import_translation: BoolProperty(name="Export Translation", default=True,
                                     description="Write TRAN channels from bone location")
    import_scale: BoolProperty(name="Export Scale", default=True,
                               description="Write SCL channels from bone scale")
    precision_mode: bpy.props.EnumProperty(
        name="Rotation Precision",
        description="How to encode each bone's rotation. Quat6 (16-bit) is the game default; Quat4 is "
                    "full float (lossless, biggest); Quat3 is 8-bit (smallest). Matters most for "
                    "BVH-like exports, whose aim rotations lose precision in Quat6",
        items=[
            ('AUTO', "Auto (per bone)", "Smallest of Quat3/Quat6/Quat4 per bone that stays within the "
                                        "tolerance below (position-aware). Recommended"),
            ('FULL', "Force full precision", "All bones use Quat4 (full float) — lossless, largest"),
            ('MANUAL', "Manual per-bone", "Use the override list below; unlisted bones stay Quat6"),
            ('NONE', "No changes (all Quat6)", "Game-identical: every bone uses Quat6 (16-bit)"),
        ],
        default='AUTO')
    precision_tol: FloatProperty(
        name="Auto Tolerance (m)", default=0.001, min=0.0, max=1.0, precision=5,
        description="AUTO mode: max distance a joint may move from rounding (e.g. 0.001 = 1 mm)")
    precision_manual: StringProperty(
        name="Per-bone override", default="",
        description="MANUAL mode: comma list like 'Pelvis=q4, Index_L=q3'. q3/q6/q4 = 8-bit/16-bit/full")

    def draw(self, context):
        col = self.layout.column()
        col.prop(self, "import_translation")
        col.prop(self, "import_scale")
        col.separator()
        col.prop(self, "precision_mode")
        if self.precision_mode == 'AUTO':
            col.prop(self, "precision_tol")
        elif self.precision_mode == 'MANUAL':
            col.prop(self, "precision_manual")

    def execute(self, context):
        return export_p3d(self, self.filepath, self.import_translation, self.import_scale,
                          self.precision_mode, self.precision_tol, self.precision_manual)


def _menu_p3d(self, context):
    self.layout.operator(IMPORT_OT_pure3d_p3d.bl_idname, text="Pure3D Animation (.p3d)")


def _menu_json(self, context):
    self.layout.operator(IMPORT_OT_pure3d_json.bl_idname, text="Pure3D JSON (.json)")


def _menu_export_p3d(self, context):
    self.layout.operator(EXPORT_OT_pure3d_p3d.bl_idname, text="Pure3D Animation (.p3d)")


_CLASSES = (IMPORT_OT_pure3d_p3d, IMPORT_OT_pure3d_json, EXPORT_OT_pure3d_p3d)


def register():
    for c in _CLASSES:
        bpy.utils.register_class(c)
    bpy.types.TOPBAR_MT_file_import.append(_menu_p3d)
    bpy.types.TOPBAR_MT_file_import.append(_menu_json)
    bpy.types.TOPBAR_MT_file_export.append(_menu_export_p3d)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(_menu_export_p3d)
    bpy.types.TOPBAR_MT_file_import.remove(_menu_json)
    bpy.types.TOPBAR_MT_file_import.remove(_menu_p3d)
    for c in reversed(_CLASSES):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
