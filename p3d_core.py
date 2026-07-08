"""
Pure3D (.p3d) parser + skeletal-animation decoder + forward kinematics.

Self-contained (Python stdlib only: struct, zlib, math). This is the engine behind the
desktop viewer (pure3d_anim_viewer.py) and is a faithful port of the verified analysis
code in ../_analysis and the web viewer (anim_viewer.html):

  * parses both PC (little-endian) and PS3 (big-endian) files,
  * reads the skeleton (0x00023000) into a bind pose,
  * decodes every rotation / translation / scale channel of every clip (0x00121000),
  * poses the skeleton with column-vector forward kinematics + quaternion SLERP.

See ../docs/ANIMATION_FORMAT.md for the format writeup.
"""
import struct
import zlib
import math

MAGIC_LE = 0xFF443350
MAGIC_BE = 0x503344FF

# chunk ids
SKEL = 0x00023000
JOINT = 0x00023001
ANIM = 0x00121000
BONELIST = 0x00121002
BONE = 0x00121001
REF = 0x00121121
ZLIBBUF = 0x02F00000

STATIC_INDEX = ((1, 2), (0, 2), (0, 1))     # Constants.STATIC_INDEX (axis placement)

# chunk id -> (role_kind, header_len, numframes_off, value_bytes, dof, decoder)
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


# ===========================================================================
#  Chunk tree
# ===========================================================================
class Chunk:
    __slots__ = ("chunk_id", "data", "children", "be")

    def __init__(self, chunk_id, data, be):
        self.chunk_id = chunk_id
        self.data = data
        self.children = []
        self.be = be

    def _u(self, fmt, off):
        return struct.unpack_from((">" if self.be else "<") + fmt, self.data, off)[0]

    def u16(self, off): return self._u("H", off)
    def u32(self, off): return self._u("I", off)
    def f32(self, off): return self._u("f", off)

    def fourcc(self, off):
        raw = self.data[off:off + 4]
        if self.be:
            raw = bytes(reversed(raw))
        return raw.decode("ascii", errors="replace")

    def p3d_string(self, off):
        n = self.data[off]
        raw = self.data[off + 1: off + 1 + n]
        return raw.split(b"\x00")[0].decode("latin-1", errors="replace"), off + 1 + n

    def find(self, cid):
        for c in self.children:
            if c.chunk_id == cid:
                return c
        return None

    def find_all(self, cid):
        return [c for c in self.children if c.chunk_id == cid]


def _parse(data, off, be):
    fmt = ">III" if be else "<III"
    cid, dsize, tsize = struct.unpack_from(fmt, data, off)
    ch = Chunk(cid, data[off + 12: off + dsize], be)
    cur, end = off + dsize, off + tsize
    while cur + 12 <= end and cur + 12 <= len(data):
        _, _, ctot = struct.unpack_from(fmt, data, cur)
        if ctot < 12:
            break
        ch.children.append(_parse(data, cur, be))
        cur += ctot
    return ch


def parse_bytes(raw):
    """Parse .p3d bytes -> (root Chunk, big_endian) or (None, None)."""
    if len(raw) < 12:
        return None, None
    magic = struct.unpack_from("<I", raw, 0)[0]
    if magic == MAGIC_LE:
        be = False
    elif magic == MAGIC_BE:
        be = True
    else:
        return None, None
    return _parse(raw, 0, be), be


def walk(root):
    yield root
    for c in root.children:
        yield from walk(c)


# ===========================================================================
#  Skeleton
# ===========================================================================
class Joint:
    __slots__ = ("name", "parent", "local", "bind")

    def __init__(self, name, parent, local):
        self.name = name
        self.parent = parent
        self.local = local          # 16 floats row-major, local-to-parent
        self.bind = (0.0, 0.0, 0.0)


def parse_skeleton(chunk):
    name, off = chunk.p3d_string(0)
    joints = []
    for j in chunk.find_all(JOINT):
        nm, o = j.p3d_string(0)
        parent = j.u32(o)
        local = [j.f32(o + 4 + 4 * i) for i in range(16)]
        joints.append(Joint(nm, parent, local))
    _fill_bind(joints)
    return name, joints


# ---- column-vector 4x4 helpers (flat 16, element (r,c)=m[r*4+c]) ----
def _col(local):
    """row-major stored local -> column-vector 4x4 (transpose)."""
    m = local
    return [m[0], m[4], m[8], m[12],
            m[1], m[5], m[9], m[13],
            m[2], m[6], m[10], m[14],
            m[3], m[7], m[11], m[15]]


def mul4(a, b):
    r = [0.0] * 16
    for i in range(4):
        ai = i * 4
        for j in range(4):
            r[ai + j] = (a[ai] * b[j] + a[ai + 1] * b[4 + j]
                         + a[ai + 2] * b[8 + j] + a[ai + 3] * b[12 + j])
    return r


def _fill_bind(joints):
    W = [None] * len(joints)
    for i, j in enumerate(joints):
        L = _col(j.local)
        if i == 0 or not (0 <= j.parent < i):
            W[i] = L
        else:
            W[i] = mul4(W[j.parent], L)
        j.bind = (W[i][3], W[i][7], W[i][11])


# ===========================================================================
#  Value-channel decode (verified; port of _analysis/anim_channels.py)
# ===========================================================================
def _zlib_buffer(anim):
    for c in anim.children:
        if c.chunk_id == ZLIBBUF and len(c.data) >= 16:
            try:
                return zlib.decompress(c.data[16:16 + c.u32(12)])
            except zlib.error:
                return None
    return None


def _region_bases(anim, buf_len=0):
    c6 = anim.find(0x00121006)
    o8 = c6.find(0x00121008) if c6 else None
    ex = o8.find(0x00121010) if o8 else None
    if ex is not None and len(ex.data) >= 12:
        n = ex.u32(8)
        if 0 < n < 64 and len(ex.data) >= 12 + 4 * n:
            bases = [0]
            for i in range(n):
                bases.append(bases[-1] + ex.u32(12 + 4 * i))
            if buf_len == 0 or bases[-1] == buf_len:
                return bases
    return [0]


def _frames(buf, base, count, be):
    return [struct.unpack_from(">H" if be else "<H", buf, base + 2 * i)[0] for i in range(count)]


def _values(buf, vbase, count, dec, be):
    o = ">" if be else "<"
    fmt = {'q4': 'ffff', 'q6': 'hhh', 'q3': 'bbb', 'h3': 'eee', 'h2': 'ee', 'f3': 'fff', 'f2': 'ff', 'f1': 'f'}[dec]
    step = {'q4': 16, 'q6': 6, 'q3': 3, 'h3': 6, 'h2': 4, 'f3': 12, 'f2': 8, 'f1': 4}[dec]
    return [struct.unpack_from(o + fmt, buf, vbase + step * i) for i in range(count)]


def _quat(vals, dec):
    if dec == 'q4':                              # uncompressed (x,y,z,w)
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


def _slot(role_kind, ttype):
    if role_kind == 'rot':
        return 'rot'
    t = ttype.rstrip('\x00')
    return {'TRAN': 'loc', 'SCL': 'scl'}.get(t)


def decode_clip_channels(anim, want=('rot', 'loc', 'scl')):
    """0x121000 -> {bone: {slot:(frames, values)}} (rot=(x,y,z,w), loc/scl=(x,y,z))."""
    size = anim.find(BONELIST)
    if size is None:
        return {}
    be = anim.be
    buf = _zlib_buffer(anim)
    bases = _region_bases(anim, len(buf) if buf is not None else 0)
    out = {}
    for kv in size.find_all(BONE):
        try:
            bone, _ = kv.p3d_string(4)
        except Exception:
            continue
        slots = {}
        for v in kv.children:
            meta = _CH.get(v.chunk_id)
            if meta is None:
                continue
            role_kind, hlen, nf_off, vbytes, dof, dec = meta
            slot = _slot(role_kind, v.fourcc(4))
            if slot is None or slot not in want:
                continue
            mapping, base = 0, (0.0, 0.0, 0.0)
            if hlen == 26:
                mapping = v.u16(8)
                if not (0 <= mapping <= 2):
                    mapping = 0
                base = (v.f32(10), v.f32(14), v.f32(18))
            try:
                n_inline = v.u32(nf_off)
                if n_inline > 0:
                    fr = _frames(v.data, hlen, n_inline, be)
                    raw = _values(v.data, hlen + 2 * n_inline, n_inline, dec, be)
                else:
                    ref = next((c for c in v.children if c.chunk_id == REF), None)
                    if ref is None or buf is None:
                        continue
                    count = ref.u32(4)
                    off = ref.u32(8)
                    rtag = ref.u16(12) if len(ref.data) >= 14 else 0
                    if not (0 <= rtag < len(bases)):
                        continue
                    fbase = bases[rtag] + off
                    vbase = fbase + 2 * count + (2 if count % 2 else 0)
                    if vbase + vbytes * count > len(buf):
                        continue
                    fr = _frames(buf, fbase, count, be)
                    raw = _values(buf, vbase, count, dec, be)
            except (struct.error, IndexError):
                continue
            if not fr:
                continue
            vals = [_quat(r, dec) for r in raw] if slot == 'rot' else [_vec3(r, dof, mapping, base) for r in raw]
            slots[slot] = (fr, vals)
        if slots:
            out[bone] = slots
    return out


def clip_name(anim):
    try:
        nm, _ = anim.p3d_string(4)
        return nm or "(anim)"
    except Exception:
        return "(anim)"


# ===========================================================================
#  High-level model + forward kinematics
# ===========================================================================
def _qmat_col(q):
    """[x,y,z,w] -> column-vector 3x3 (flat 9, row-major)."""
    x, y, z, w = q
    return [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]


def _slerp(a, b, t):
    d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
    if d < 0:
        b = (-b[0], -b[1], -b[2], -b[3]); d = -d
    if d > 0.9995:
        r = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t,
             a[2] + (b[2] - a[2]) * t, a[3] + (b[3] - a[3]) * t)
        n = math.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]) or 1.0
        return (r[0] / n, r[1] / n, r[2] / n, r[3] / n)
    th = math.acos(max(-1.0, min(1.0, d)))
    s = math.sin(th)
    w1, w2 = math.sin((1 - t) * th) / s, math.sin(t * th) / s
    return (a[0] * w1 + b[0] * w2, a[1] * w1 + b[1] * w2,
            a[2] * w1 + b[2] * w2, a[3] * w1 + b[3] * w2)


def _sample_quat(frames, quats, f):
    if f <= frames[0]:
        return quats[0]
    if f >= frames[-1]:
        return quats[-1]
    for i in range(1, len(frames)):
        if f <= frames[i]:
            t = (f - frames[i - 1]) / ((frames[i] - frames[i - 1]) or 1)
            return _slerp(quats[i - 1], quats[i], t)
    return quats[-1]


def _sample_vec(frames, vals, f):
    if f <= frames[0]:
        return vals[0]
    if f >= frames[-1]:
        return vals[-1]
    for i in range(1, len(frames)):
        if f <= frames[i]:
            t = (f - frames[i - 1]) / ((frames[i] - frames[i - 1]) or 1)
            a, b = vals[i - 1], vals[i]
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)
    return vals[-1]


class Clip:
    __slots__ = ("name", "max_frame", "channels")

    def __init__(self, name, channels):
        self.name = name
        self.channels = channels    # {bone: {slot:(frames, vals)}}
        mf = 0
        for slots in channels.values():
            for fr, _ in slots.values():
                if fr:
                    mf = max(mf, fr[-1])
        self.max_frame = mf or 1


class Model:
    """A skeleton + its clips, with FK posing. `skeleton` may be None until set."""

    def __init__(self, source, skeleton_joints, skeleton_name, clips):
        self.source = source
        self.name = skeleton_name
        self.joints = skeleton_joints or []
        self.clips = clips
        self._local_col = [_col(j.local) for j in self.joints] if self.joints else []
        self._index = {j.name: i for i, j in enumerate(self.joints)}
        self._compute_bounds()

    # ---- bounds for the camera (from the bind pose) ----
    def _compute_bounds(self):
        if not self.joints:
            self.center = (0.0, 1.0, 0.0); self.span = 2.0; self.ymin = 0.0
            return
        xs = [j.bind[0] for j in self.joints]
        ys = [j.bind[1] for j in self.joints]
        zs = [j.bind[2] for j in self.joints]
        self.center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2)
        self.span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.5)
        self.ymin = min(ys)

    def bone_coverage(self):
        """Fraction of clip channel-bones that exist in the skeleton (0..1)."""
        tot = hit = 0
        for cl in self.clips:
            for b in cl.channels:
                tot += 1
                if b in self._index:
                    hit += 1
        return hit / tot if tot else 0.0

    # ---- forward kinematics ----
    def pose_world(self, clip_idx, frame):
        """Return a list of world-space joint positions (x,y,z) for the given frame."""
        joints = self.joints
        n = len(joints)
        W = [None] * n
        pos = [None] * n
        ch = self.clips[clip_idx].channels if 0 <= clip_idx < len(self.clips) else {}
        for i in range(n):
            j = joints[i]
            slots = ch.get(j.name)
            L = self._local_col[i]
            if slots:
                L = self._anim_local(i, slots, frame)
            if i == 0 or not (0 <= j.parent < i):
                W[i] = L[:]
            else:
                W[i] = mul4(W[j.parent], L)
            pos[i] = (W[i][3], W[i][7], W[i][11])
        return pos

    def _anim_local(self, i, slots, frame):
        L = self._local_col[i]
        # rotation
        if 'rot' in slots:
            fr, q = slots['rot']
            R = _qmat_col(_sample_quat(fr, q, frame))
        else:
            R = [L[0], L[1], L[2], L[4], L[5], L[6], L[8], L[9], L[10]]
        # translation (absolute parent-local; replaces rest translation)
        if 'loc' in slots:
            fr, v = slots['loc']
            tx, ty, tz = _sample_vec(fr, v, frame)
        else:
            tx, ty, tz = L[3], L[7], L[11]
        # scale
        if 'scl' in slots:
            fr, v = slots['scl']
            sx, sy, sz = _sample_vec(fr, v, frame)
        else:
            sx = sy = sz = 1.0
        # column-vector 4x4: 3x3 = R @ diag(s) (scale column k), translation in col 3
        return [R[0] * sx, R[1] * sy, R[2] * sz, tx,
                R[3] * sx, R[4] * sy, R[5] * sz, ty,
                R[6] * sx, R[7] * sy, R[8] * sz, tz,
                0.0, 0.0, 0.0, 1.0]

    def edges(self):
        """Parent->child bone pairs (indices)."""
        out = []
        for i, j in enumerate(self.joints):
            if i != 0 and 0 <= j.parent < len(self.joints):
                out.append((j.parent, i))
        return out

    def channels_for(self, clip_idx, joint_idx):
        """Slots present for a joint in a clip, e.g. {'rot','loc'}."""
        if not (0 <= clip_idx < len(self.clips)) or not (0 <= joint_idx < len(self.joints)):
            return set()
        return set(self.clips[clip_idx].channels.get(self.joints[joint_idx].name, {}).keys())

    # ---- exporter accessors (per-joint LOCAL transform, no scale) ----
    def rest_offset(self, i):
        """Parent-local rest translation of joint i (the BVH OFFSET)."""
        L = self._local_col[i]
        return (L[3], L[7], L[11])

    def local_rot_trans(self, clip_idx, i, frame):
        """Animated LOCAL rotation matrix (flat9, column-vector) + translation for joint i,
        sampled at `frame`. Falls back to the rest rotation/translation where no channel exists.
        (Scale is intentionally excluded — for BVH/skeletal export.)"""
        L = self._local_col[i]
        slots = self.clips[clip_idx].channels.get(self.joints[i].name, {}) \
            if 0 <= clip_idx < len(self.clips) else {}
        if 'rot' in slots:
            fr, q = slots['rot']
            R = _qmat_col(_sample_quat(fr, q, frame))
        else:
            R = [L[0], L[1], L[2], L[4], L[5], L[6], L[8], L[9], L[10]]
        if 'loc' in slots:
            fr, v = slots['loc']
            t = _sample_vec(fr, v, frame)
        else:
            t = (L[3], L[7], L[11])
        return R, t


def load_p3d(path):
    """Load a .p3d. Returns (skeleton_name_or_None, joints_or_None, clips[list[Clip]], big_endian).
    joints is None when the file is an animation-only package (no 0x00023000)."""
    with open(path, "rb") as f:
        raw = f.read()
    root, be = parse_bytes(raw)
    if root is None:
        raise ValueError("Not a valid Pure3D (.p3d) file.")
    # clips first, so we know which bones the animations actually drive
    clips = []
    clip_bones = set()
    for a in walk(root):
        if a.chunk_id == ANIM and a.find(BONELIST) is not None:
            chans = decode_clip_channels(a)
            if chans:
                clips.append(Clip(clip_name(a), chans))
                clip_bones.update(chans.keys())
    clips.sort(key=lambda c: c.name)
    # a file can hold several skeletons (e.g. a character + a prop like a tricorder). Pick the
    # one whose joints best cover the clip bones, then by joint count — not just the first found.
    sk_name, joints = None, None
    skels = [parse_skeleton(c) for c in walk(root) if c.chunk_id == SKEL]
    if skels:
        def _score(item):
            _nm, js = item
            names = set(j.name for j in js)
            return (len(clip_bones & names), len(js))
        sk_name, joints = max(skels, key=_score)
    return sk_name, joints, clips, be
