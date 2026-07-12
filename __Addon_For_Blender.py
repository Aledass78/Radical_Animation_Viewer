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


def build_armature(name, joints, extra_le=b""):
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


def _install_clips(operator, arm_obj, clips, fps, set_first_active=True):
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
def import_p3d(operator, filepath, fps, import_translation, import_scale):
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
    armatures = []                                     # (arm_obj, set(bone_names))
    for nm, joints, extra in skels:
        armatures.append((build_armature(nm, joints, extra), set(j.name for j in joints)))

    anims = [c for c in iter_all_chunks(root) if c.chunk_id == 0x00121000
             and c.find_child(0x00121002) is not None]
    buckets = [[] for _ in armatures]
    unrouted = 0
    for a in anims:
        chanmap = decode_clip_channels(a, want=want)
        if not chanmap:
            continue
        clip_bones = set(chanmap)
        # pick the skeleton that covers the most of this clip's bones (ties -> first skeleton)
        scored = [(len(clip_bones & names), -i) for i, (_arm, names) in enumerate(armatures)]
        cov, negi = max(scored)
        if cov == 0:                                   # e.g. camera-only clips -> no skeleton
            unrouted += 1
            continue
        buckets[-negi].append((clip_name(a), chanmap))

    total = 0
    for (arm, _names), bucket in zip(armatures, buckets):
        total += _install_clips(operator, arm, bucket, fps)
    msg = "Imported %d skeleton(s) + %d clip(s) from .p3d." % (len(armatures), total)
    if unrouted:
        msg += " (%d clip(s) matched no skeleton, skipped.)" % unrouted
    operator.report({'INFO'}, msg)
    return {'FINISHED'}


def import_json(operator, filepath, fps, import_translation, import_scale):
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
    arm_obj = build_armature(sk.get("name", "Pure3D"), joints)

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
    made = _install_clips(operator, arm_obj, [(clip.get("name", "clip"), chanmap)],
                          clip.get("fps", fps))
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


def _encode_channel(cid, ttype, frames, values, be):
    tt = ttype.encode("ascii")[:4].ljust(4, b"\x00")
    if be:
        tt = bytes(reversed(tt))
    ho = ">H" if be else "<H"
    header = _u32w(0, be) + tt + _u32w(len(frames), be)
    fb = b"".join(struct.pack(ho, f & 0xFFFF) for f in frames)
    o = ">" if be else "<"
    if cid == 0x00121112:                              # Quaternion6
        vb = b"".join(struct.pack(o + "hhh", *_quat_to_q6(v)) for v in values)
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


def build_clip(name, order, channels, num_frames, fps, be):
    """0x121000 subtree from {bone:{'rot':(fr,quats),'loc':(fr,xyz),'scl':(fr,xyz)}}."""
    bone_chunks = []
    type_keys = {}
    for bone in order:
        slots = channels.get(bone)
        if not slots:
            continue
        chan = []
        if 'rot' in slots:
            fr, q = slots['rot']
            chan.append(_encode_channel(0x00121112, "ROT\0", fr, q, be))
            type_keys.setdefault(0x00121112, []).append(len(fr))
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


def _export_armature(arm_obj, be, import_translation, import_scale):
    """-> (skeleton_bytes, [clip_bytes...]) for one armature and its NLA/active actions."""
    order, index = _bone_order(arm_obj)
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
        # which bone+slot are actually keyed?
        keyed = {}                                     # bone -> set('rot','loc','scl')
        for fc in action.fcurves:
            dp = fc.data_path
            if not dp.startswith('pose.bones["'):
                continue
            bn = dp[len('pose.bones["'):dp.index('"]')]
            if dp.endswith("rotation_quaternion"):
                keyed.setdefault(bn, set()).add('rot')
            elif dp.endswith("location") and import_translation:
                keyed.setdefault(bn, set()).add('loc')
            elif dp.endswith("scale") and import_scale:
                keyed.setdefault(bn, set()).add('scl')
        if not keyed:
            continue
        if ad:
            ad.action = action
        channels = {b: {s: ([], []) for s in slots} for b, slots in keyed.items()}
        for f in range(f0, f1 + 1):
            scene.frame_set(f)
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
                gf = f - f0                            # game frame index starts at 0
                if 'rot' in slots:
                    rq = rest_q @ p_rot
                    channels[bn]['rot'][0].append(gf)
                    channels[bn]['rot'][1].append((rq.x, rq.y, rq.z, rq.w))
                if 'loc' in slots:
                    rl = rest_q @ p_loc + rest_l
                    channels[bn]['loc'][0].append(gf)
                    channels[bn]['loc'][1].append((rl.x, rl.y, rl.z))
                if 'scl' in slots:
                    channels[bn]['scl'][0].append(gf)
                    channels[bn]['scl'][1].append((p_scl.x, p_scl.y, p_scl.z))
        nframes = f1 - f0 + 1
        clips.append(build_clip(action.name, bone_names, channels, nframes,
                                scene.render.fps, be))
    if ad:
        ad.action = saved_action
    scene.frame_set(saved_frame)
    return sk_bytes, clips


def export_p3d(operator, filepath, import_translation, import_scale):
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
        sk, clips = _export_armature(arm, False, import_translation, import_scale)
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


class IMPORT_OT_pure3d_p3d(bpy.types.Operator, ImportHelper, _CommonProps):
    bl_idname = "import_scene.pure3d_p3d"
    bl_label = "Import Pure3D Animation (.p3d)"
    bl_options = {'REGISTER', 'UNDO'}
    filename_ext = ".p3d"
    filter_glob: StringProperty(default="*.p3d", options={'HIDDEN'})

    def execute(self, context):
        return import_p3d(self, self.filepath, self.fps, self.import_translation, self.import_scale)


class IMPORT_OT_pure3d_json(bpy.types.Operator, ImportHelper, _CommonProps):
    bl_idname = "import_scene.pure3d_json"
    bl_label = "Import Pure3D JSON (.json)"
    bl_options = {'REGISTER', 'UNDO'}
    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        return import_json(self, self.filepath, self.fps, self.import_translation, self.import_scale)


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

    def execute(self, context):
        return export_p3d(self, self.filepath, self.import_translation, self.import_scale)


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
