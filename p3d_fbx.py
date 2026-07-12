"""
FBX (ASCII) export + FBX (ASCII/binary) read for Prototype 2 skeletal animation.

Why FBX: it stores per-bone **length** (so Blender draws proper octahedral bones, not glTF's sphere
blobs) AND the **rest pose** (rotations, unlike BVH) AND per-frame TRS **with scale**. So a `.p3d`
clip exported here imports into Blender with the bone look you want and the game-accurate rest/pose.

  * export_fbx(model, clip_idx, path, fps)  -> writes ASCII FBX 7.4 (Blender imports ASCII fine).
  * read_fbx(path) -> {joints, clips, fps}  -> parses ASCII *or* binary FBX (Blender exports binary)
                                               back into our channel form for injection into a .p3d.

Conventions
-----------
Rotations are Euler **XYZ** (FBX RotationOrder 0 / eEulerXYZ): R = Rz·Ry·Rx applied to a column
vector — i.e. rotate about X first. Our quat<->euler pair below is an exact inverse of each other,
so an export->read round-trip through this module is lossless; Blender interprets eEulerXYZ the same
way. Coordinates are the game's Y-up (FBX UpAxis=Y); Blender's FBX importer converts to its Z-up.
"""
import struct
import math
import zlib

_R2D = 180.0 / math.pi
_D2R = math.pi / 180.0
_KTIME = 46186158000            # FBX KTime units per second (default time mode)


# ------------------------------------------------------------------ euler <-> matrix/quat (XYZ)
def _quat_to_mat(q):
    x, y, z, w = q
    return [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]


def _mat_to_euler_xyz(M):
    """Column-vector 3x3 (flat9, row-major) with R = Rz·Ry·Rx  ->  (x, y, z) radians (eEulerXYZ).
    For that composition M[2][0] = -sin(y), M[2][1]=cy·sx, M[2][2]=cy·cx, M[1][0]=sz·cy, M[0][0]=cz·cy."""
    sy = max(-1.0, min(1.0, -M[6]))          # -M[2][0]
    y = math.asin(sy)
    if abs(M[6]) < 0.9999:                    # cos(y) not ~0
        x = math.atan2(M[7], M[8])            # cy·sx , cy·cx
        z = math.atan2(M[3], M[0])            # sz·cy , cz·cy
    else:                                     # gimbal lock: fold z into x
        z = 0.0
        x = math.atan2(-M[5], M[4])
    return x, y, z


def _euler_xyz_to_mat(x, y, z):
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    Rx = [1, 0, 0, 0, cx, -sx, 0, sx, cx]
    Ry = [cy, 0, sy, 0, 1, 0, -sy, 0, cy]
    Rz = [cz, -sz, 0, sz, cz, 0, 0, 0, 1]

    def mm(a, b):
        return [sum(a[r * 3 + k] * b[k * 3 + c] for k in range(3)) for r in range(3) for c in range(3)]
    return mm(mm(Rz, Ry), Rx)


def _quat_to_euler_xyz_deg(q):
    x, y, z = _mat_to_euler_xyz(_quat_to_mat(q))
    return (x * _R2D, y * _R2D, z * _R2D)


def _mat_to_quat(M):
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = M
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


def _euler_xyz_deg_to_quat(ex, ey, ez):
    return _mat_to_quat(_euler_xyz_to_mat(ex * _D2R, ey * _D2R, ez * _D2R))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


# ------------------------------------------------------------------ matrix helpers (compat / Mixamo)
def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(v):
    l = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
    return (v[0] / l, v[1] / l, v[2] / l)


def _lookat_y(fwd, up):
    """3x3 (column-vector flat9) whose +Y axis = fwd, roll from `up`."""
    fy = _norm(fwd)
    fx = _cross(up, fy)
    if fx[0] * fx[0] + fx[1] * fx[1] + fx[2] * fx[2] < 1e-8:
        fx = _cross((1.0, 0.0, 0.0), fy)
        if fx[0] * fx[0] + fx[1] * fx[1] + fx[2] * fx[2] < 1e-8:
            fx = _cross((0.0, 0.0, 1.0), fy)
    fx = _norm(fx)
    fz = _cross(fx, fy)
    return [fx[0], fy[0], fz[0], fx[1], fy[1], fz[1], fx[2], fy[2], fz[2]]


def _m4mul(A, B):
    return [sum(A[r * 4 + k] * B[k * 4 + c] for k in range(4)) for r in range(4) for c in range(4)]


def _m4inv_rigid(M):
    R = (M[0], M[1], M[2], M[4], M[5], M[6], M[8], M[9], M[10])
    t = (M[3], M[7], M[11])
    Rt = (R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8])
    it = (-(Rt[0] * t[0] + Rt[1] * t[1] + Rt[2] * t[2]),
          -(Rt[3] * t[0] + Rt[4] * t[1] + Rt[5] * t[2]),
          -(Rt[6] * t[0] + Rt[7] * t[1] + Rt[8] * t[2]))
    return [Rt[0], Rt[1], Rt[2], it[0], Rt[3], Rt[4], Rt[5], it[1], Rt[6], Rt[7], Rt[8], it[2], 0, 0, 0, 1]


def _qrotv(q, v):
    """Rotate 3-vector v by quaternion q (x,y,z,w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _shortest_arc(u, v):
    """Unit quaternion of the shortest rotation carrying unit vector u onto unit vector v."""
    d = max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1] + u[2] * v[2]))
    if d >= 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    if d <= -1.0 + 1e-9:                          # antiparallel: 180 deg about any perpendicular axis
        ax = _cross((1.0, 0.0, 0.0), u)
        if ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2] < 1e-8:
            ax = _cross((0.0, 0.0, 1.0), u)
        ax = _norm(ax)
        return (ax[0], ax[1], ax[2], 0.0)
    c = _cross(u, v)
    w = 1.0 + d
    n = math.sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2] + w * w) or 1.0
    return (c[0] / n, c[1] / n, c[2] / n, w / n)


def _aim_up_axis(gq, head, child_head):
    """Pick, from the rest pose, which of the game orientation's local axes (0/1/2 = X/Y/Z) to use as
    the roll reference for `_aim_world` — the one most perpendicular to the aim. Returns -1 if the bone
    has no usable aim (coincident/absent child), meaning 'keep the game orientation'."""
    if child_head is None:
        return -1
    a = (child_head[0] - head[0], child_head[1] - head[1], child_head[2] - head[2])
    if a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < 1e-10:
        return -1
    a = _norm(a)
    axes = (_qrotv(gq, (1.0, 0.0, 0.0)), _qrotv(gq, (0.0, 1.0, 0.0)), _qrotv(gq, (0.0, 0.0, 1.0)))
    return min(range(3), key=lambda k: abs(axes[k][0] * a[0] + axes[k][1] * a[1] + axes[k][2] * a[2]))


def _aim_world(gq, head, child_head, up_axis):
    """World-orientation quat whose +Y points from `head` toward `child_head`, with the roll taken from
    a FIXED game axis (`up_axis`, chosen once per bone at rest by `_aim_up_axis`) so twist is preserved
    AND continuous across frames. `up_axis` < 0 means keep the game orientation `gq` unchanged.

    Fixing the roll axis per bone is what keeps it smooth: swinging gq's +Y onto the aim is degenerate
    when they are ~antiparallel (the near-vertical root connectors), and picking the most-perpendicular
    axis PER FRAME swaps between two near-equal candidates and pops ~90 deg. Position depends only on
    the aim direction, so this leaves joint positions exact."""
    if up_axis < 0 or child_head is None:
        return gq
    a = (child_head[0] - head[0], child_head[1] - head[1], child_head[2] - head[2])
    if a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < 1e-10:
        return gq
    a = _norm(a)
    up = _qrotv(gq, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))[up_axis])
    R3 = _lookat_y(a, up)
    return _mat_to_quat([R3[0], R3[1], R3[2], R3[3], R3[4], R3[5], R3[6], R3[7], R3[8]])


def _continuous_euler(seq):
    """Unroll a list of (x,y,z)-degree Euler triples so consecutive frames never jump ~360 degrees
    (prevents wild fcurve interpolation on large rotations)."""
    out = []
    prev = None
    for e in seq:
        if prev is None:
            out.append(tuple(e)); prev = tuple(e); continue
        ne = []
        for a in range(3):
            val = e[a]
            while val - prev[a] > 180.0:
                val -= 360.0
            while val - prev[a] < -180.0:
                val += 360.0
            ne.append(val)
        out.append(tuple(ne)); prev = tuple(ne)
    return out


# ==================================================================== EXPORT (ASCII FBX)
# ---- binary FBX serialization (Blender only reads BINARY fbx, not ASCII) ----
def _ser_prop(p):
    tag, val = p
    if tag == 'S':
        b = val.encode("latin-1"); return b'S' + struct.pack("<I", len(b)) + b
    if tag == 'D':
        return b'D' + struct.pack("<d", val)
    if tag == 'I':
        return b'I' + struct.pack("<i", val)
    if tag == 'L':
        return b'L' + struct.pack("<q", val)
    if tag == 'C':
        return b'C' + struct.pack("<B", 1 if val else 0)
    code, fmt = {'farr': ('f', 'f'), 'darr': ('d', 'd'),
                 'larr': ('l', 'q'), 'iarr': ('i', 'i')}[tag]
    raw = struct.pack("<%d%s" % (len(val), fmt), *val)
    return code.encode() + struct.pack("<III", len(val), 0, len(raw)) + raw   # encoding=0 (uncompressed)


def _ser_record(node, offset):
    name = node.name.encode("latin-1")
    props = b"".join(_ser_prop(p) for p in node.props)
    child_start = offset + 13 + len(name) + len(props)
    cur = child_start
    body = b""
    if node.children:
        for c in node.children:
            cb, cur = _ser_record(c, cur)
            body += cb
        body += b"\x00" * 13                     # nested-list null terminator
        cur += 13
    rec = struct.pack("<III", cur, len(node.props), len(props)) + bytes([len(name)]) + name + props + body
    return rec, cur


def _ser_fbx(top_nodes):
    out = bytearray(b"Kaydara FBX Binary  \x00" + bytes([0x1A, 0x00]) + struct.pack("<I", 7400))
    cur = len(out)
    for node in top_nodes:
        rec, cur = _ser_record(node, cur)
        out += rec
    out += b"\x00" * 13                          # top-level null terminator
    pad = (16 - (len(out) % 16)) % 16 or 16
    out += b"\x00" * pad
    out += struct.pack("<I", 7400) + b"\x00" * 120
    out += bytes([0xF8, 0x5A, 0x8C, 0x6A, 0xDE, 0xF5, 0xD9, 0x7E,
                  0xEC, 0xE9, 0x0C, 0xE3, 0x75, 0x8F, 0x29, 0x0B])
    return bytes(out)


def export_fbx(model, clip_idx, path, fps=30, compat=False):
    """Write clip `clip_idx` to a **binary** FBX (7400) with a skeleton (rest pose + per-bone length
    so Blender draws octahedral bones) and rotation/translation/scale animation. Returns frames.

    compat -- "as compatible as possible" (Mixamo-style): retarget onto a look-at skeleton so bones
    import connected/oriented like a Mixamo rig WITHOUT needing Blender's Automatic Bone Orientation.
    Each rigid limb bone's +Y aims at its first child; the bind holds that aim orientation in
    `PreRotation` with a +Y-dominant bind `Lcl Translation`. Per frame we recompute the aim world
    orientation (look-at down the child direction, roll from a fixed game axis so twist is kept and
    stays smooth; non-rigid root connectors keep their game orientation) and re-express it
    parent-relative minus PreRotation as `Lcl Rotation`; only the root (and any bone whose offset
    actually moves) gets a `Lcl Translation` curve, so rigid bones stay connected. Connected bones then
    reproduce EXACT game joint positions (verified <1e-4 across clips); only per-bone twist is
    approximated, which is inherent to any connected-bone rig. Default off = game-faithful frames."""
    joints = model.joints
    clip = model.clips[clip_idx]
    n = len(joints)
    nframes = clip.max_frame + 1
    _next = [1000000]

    def nid():
        _next[0] += 1
        return _next[0]

    model_id = [nid() for _ in range(n)]
    attr_id = [nid() for _ in range(n)]

    rest_eul, rest_pos, length = [], [], []
    for i in range(n):
        R, t = model.rest_rot_trans(i)
        rest_eul.append(_quat_to_euler_xyz_deg(_mat_to_quat(R)))
        rest_pos.append(t)
    kids = [[] for _ in range(n)]
    for i, j in enumerate(joints):
        if i != 0 and 0 <= j.parent < n and j.parent != i:
            kids[j.parent].append(i)
    for i in range(n):
        if kids[i]:
            d = math.sqrt(sum(v * v for v in rest_pos[kids[i][0]]))
            length.append(d if d > 1e-4 else 0.05)
        else:
            length.append(0.05)

    chan = clip.channels
    prerot = [(0.0, 0.0, 0.0)] * n
    if compat:
        # ---- retarget to a look-at ("Mixamo") skeleton -------------------------------------------
        # Each RIGID limb bone's +Y aims at its first child; the bind holds that aim orientation in
        # PreRotation with a +Y-dominant Lcl Translation so Blender imports the bones connected/oriented.
        # Per frame we recompute the aim world orientation (look-at along the child direction with the
        # roll from a fixed game axis, so twist is kept and stays smooth) and re-express it
        # parent-relative minus PreRotation as Lcl Rotation. Non-rigid root/pelvis connectors keep their
        # game orientation instead (see up_axis below). Connected bones then reproduce EXACT joint
        # positions; only per-bone twist is approximated, inherent to any connected-bone (Mixamo) rig.
        def _fk_world(mat_of):
            """FK a per-bone LOCAL flat16 provider (scale included) into (world_quat[], world_head[]).
            World rotation is taken scale-free by normalizing the world matrix columns."""
            Wm = [None] * n; wq = [None] * n; hd = [None] * n
            for i in range(n):
                Lm = mat_of(i)
                p = joints[i].parent
                Wm[i] = Lm[:] if (i == 0 or not (0 <= p < i)) else _m4mul(Wm[p], Lm)
                cx = _norm((Wm[i][0], Wm[i][4], Wm[i][8]))       # column 0
                cy = _norm((Wm[i][1], Wm[i][5], Wm[i][9]))       # column 1
                cz = _norm((Wm[i][2], Wm[i][6], Wm[i][10]))      # column 2
                wq[i] = _mat_to_quat([cx[0], cy[0], cz[0], cx[1], cy[1], cz[1], cx[2], cy[2], cz[2]])
                hd[i] = (Wm[i][3], Wm[i][7], Wm[i][11])
            return wq, hd

        # FK rest + every frame up front
        gqr, hdr = _fk_world(lambda i: model.rest_matrix(i))
        frames = list(range(nframes))
        gq_f, head_f = [], []
        for f in frames:
            gq, hd = _fk_world(lambda i, ff=f: model.local_matrix(clip_idx, i, ff))
            gq_f.append(gq); head_f.append(hd)

        # Give each bone the SAME rest orientation Blender builds from our .bvh, so the compat rig LOOKS
        # like the BVH (bones running down the limbs) — but with FBX's scale + upright rest. Blender aims
        # a BVH bone's tail at its single child, or at the AVERAGE of its children (multi-child), with
        # attachment stubs (Shield / *_Grapple / *_Con / Collar ...) collapsed onto the primary child,
        # exactly as our BVH export does. The animation then just rotates that rest frame:
        #     aim_f[i] = G_i(f) * (G_i(rest)^-1 * N_bvh_i)
        # which is stable (no per-frame look-at) and leaves joint positions exact (roll never moves a joint).
        sz = [1] * n
        for i in range(n - 1, -1, -1):
            for c in kids[i]:
                sz[i] += sz[c]

        def _bvh_tail(i, heads):                          # avg of children heads, stubs -> primary child
            ch = kids[i]
            if not ch:
                return None
            prim = max(ch, key=lambda c: sz[c])
            ax = ay = az = 0.0
            for c in ch:
                h = heads[prim] if (c != prim and sz[c] <= 2 and sz[prim] >= 3 * sz[c]) else heads[c]
                ax += h[0]; ay += h[1]; az += h[2]
            k = len(ch)
            return (ax / k - heads[i][0], ay / k - heads[i][1], az / k - heads[i][2])

        def _rigid(i):
            # a bone can be BVH-aimed only if EVERY child stays FIXED relative to it (constant offset
            # VECTOR, not just distance — a body-sway moves the child in a circle at ~constant distance).
            # Aiming at a child that moves makes Blender +Y-CONNECT it, and a connected bone in Blender
            # ignores its Lcl Translation animation — which for Character_Root (parent Balance_Root) is
            # the whole body's fall/translation. Such connectors keep their game orientation so the child
            # stays unconnected and its loc curve still plays.
            if not kids[i]:
                return False
            for c in kids[i]:
                xs = []; ys = []; zs = []
                for fi in range(len(frames)):
                    o = _qrotv(_qconj(gq_f[fi][i]),
                               (head_f[fi][c][0] - head_f[fi][i][0], head_f[fi][c][1] - head_f[fi][i][1],
                                head_f[fi][c][2] - head_f[fi][i][2]))
                    xs.append(o[0]); ys.append(o[1]); zs.append(o[2])
                if (max(xs) - min(xs) > 5e-3 or max(ys) - min(ys) > 5e-3 or max(zs) - min(zs) > 5e-3):
                    return False
            return True

        Nb = [None] * n; C = [None] * n                   # N_bvh (rest orientation) + game->bvh reorient
        for i in range(n):
            d = _bvh_tail(i, hdr) if _rigid(i) else None
            if d is None or (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) < 1e-10:
                Nb[i] = gqr[i]                             # leaf / non-rigid connector -> keep game orientation
            else:
                a = _norm(d)
                axes = (_qrotv(gqr[i], (1., 0., 0.)), _qrotv(gqr[i], (0., 1., 0.)), _qrotv(gqr[i], (0., 0., 1.)))
                up = min(axes, key=lambda v: abs(v[0] * a[0] + v[1] * a[1] + v[2] * a[2]))
                R3 = _lookat_y(a, up)
                Nb[i] = _mat_to_quat([R3[0], R3[1], R3[2], R3[3], R3[4], R3[5], R3[6], R3[7], R3[8]])
            C[i] = _qmul(_qconj(gqr[i]), Nb[i])
        aim_f = [[_qmul(gq_f[fi][i], C[i]) for i in range(n)] for fi in range(len(frames))]
        Pq = [None] * n; prerot = [None] * n
        for i in range(n):
            p = joints[i].parent
            if i == 0 or not (0 <= p < i):
                Pq[i] = Nb[i]
                rest_pos[i] = hdr[i]                       # root: bind position = world head
            else:
                Pq[i] = _qmul(_qconj(Nb[p]), Nb[i])        # parent-relative bind orientation
                d = (hdr[i][0] - hdr[p][0], hdr[i][1] - hdr[p][1], hdr[i][2] - hdr[p][2])
                rest_pos[i] = _qrotv(_qconj(Nb[p]), d)     # child head in parent aim frame (+Y-dominant)
            prerot[i] = _quat_to_euler_xyz_deg(Pq[i])
            rest_eul[i] = (0.0, 0.0, 0.0)                  # orientation lives in PreRotation
        # Bone length = distance to the BVH tail (the average of children it now aims at), and leaf tips
        # = 0.03*span, exactly like our BVH -> matching bone SIZES too. Sizing to the first child instead
        # made multi-child bones (e.g. symmetric L/R face bones) far too long (first child is off to one
        # side while the average sits near the head).
        span = getattr(model, "span", 1.0) or 1.0
        for i in range(n):
            d = _bvh_tail(i, hdr)
            if d is None:
                length[i] = 0.03 * span                    # leaf -> BVH End Site tip
            else:
                L = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
                length[i] = L if L > 1e-4 else 0.03 * span

        # animation: per-frame aim orientation -> parent-relative -> minus PreRotation (aim_f ready above)
        chan = {}
        for i in range(n):
            p = joints[i].parent
            is_root = (i == 0 or not (0 <= p < i))
            qpi = _qconj(Pq[i])
            locq = []
            locv = []
            for fi in range(len(frames)):
                if is_root:
                    rel = aim_f[fi][i]
                    locv.append(head_f[fi][i])            # root: world head
                else:
                    rel = _qmul(_qconj(aim_f[fi][p]), aim_f[fi][i])
                    d = (head_f[fi][i][0] - head_f[fi][p][0], head_f[fi][i][1] - head_f[fi][p][1],
                         head_f[fi][i][2] - head_f[fi][p][2])
                    locv.append(_qrotv(_qconj(aim_f[fi][p]), d))   # child head in parent aim frame
                locq.append(_qmul(qpi, rel))
            out = {'rot': (frames, locq)}
            # Only emit a translation curve when the offset actually moves (attachment anchors, IK-ish
            # bones). Rigid body bones keep the constant bind offset so Blender imports them connected.
            if is_root or any(abs(locv[fi][k] - rest_pos[i][k]) > 1e-4
                              for fi in range(len(frames)) for k in range(3)):
                out['loc'] = (frames, locv)
            s = clip.channels.get(joints[i].name, {})
            if 'scl' in s:
                out['scl'] = s['scl']
            chan[joints[i].name] = out

    def P(name, typ, sub, flags, *vals):
        return _Node("P", [('S', name), ('S', typ), ('S', sub), ('S', flags)] + list(vals), [])

    def D(v):
        return ('D', float(v))

    def NC(name, cls):                               # binary FBX object name: Name<0x00 0x01>Class
        return ('S', name + "\x00\x01" + cls)

    stack_id, layer_id = nid(), nid()
    curvenodes, curves = [], []
    for i in range(n):
        slots = chan.get(joints[i].name, {})
        for kind, slot in (('T', 'loc'), ('R', 'rot'), ('S', 'scl')):
            s = slots.get(slot)
            if not s:
                continue
            frames, vals = s
            cn = nid()
            curvenodes.append((cn, i, kind))
            eul = _continuous_euler([_quat_to_euler_xyz_deg(q) for q in vals]) if kind == 'R' else None
            for ax_i, ax in enumerate("XYZ"):
                av = [e[ax_i] for e in eul] if kind == 'R' else [v[ax_i] for v in vals]
                curves.append((nid(), cn, ax, list(frames), av))

    top = []
    top.append(_Node("FBXHeaderExtension", [], [
        _Node("FBXHeaderVersion", [('I', 1003)], []),
        _Node("FBXVersion", [('I', 7400)], []),
        _Node("Creator", [('S', "pure3d_anim_viewer")], []),
    ]))
    # FBX key times are absolute (KTime), so the frame rate MUST be declared or the importer guesses
    # (~25 fps) and the action ends on the wrong frame — looking shorter/longer than a frame-based BVH.
    stop_kt = int(round((nframes - 1) / float(fps) * _KTIME))
    _TIMEMODE = {120: 1, 100: 2, 60: 3, 50: 4, 48: 5, 30: 6, 24: 11, 25: 10, 96: 15, 72: 16}
    timemode = _TIMEMODE.get(int(round(fps)), 14)          # 14 = eCustom -> CustomFrameRate
    top.append(_Node("GlobalSettings", [], [
        _Node("Version", [('I', 1000)], []),
        _Node("Properties70", [], [
            P("UpAxis", "int", "Integer", "", ('I', 1)),
            P("UpAxisSign", "int", "Integer", "", ('I', 1)),
            P("FrontAxis", "int", "Integer", "", ('I', 2)),
            P("FrontAxisSign", "int", "Integer", "", ('I', 1)),
            P("CoordAxis", "int", "Integer", "", ('I', 0)),
            P("CoordAxisSign", "int", "Integer", "", ('I', 1)),
            P("UnitScaleFactor", "double", "Number", "", D(1)),
            P("TimeMode", "enum", "", "", ('I', timemode)),
            P("CustomFrameRate", "double", "Number", "", D(float(fps))),
            P("TimeSpanStart", "KTime", "Time", "", ('L', 0)),
            P("TimeSpanStop", "KTime", "Time", "", ('L', stop_kt)),
        ]),
    ]))
    top.append(_Node("Definitions", [], [
        _Node("Version", [('I', 100)], []),
        _Node("Count", [('I', n * 2 + 2 + len(curvenodes) + len(curves))], []),
        _Node("ObjectType", [('S', "Model")], [_Node("Count", [('I', n)], [])]),
        _Node("ObjectType", [('S', "NodeAttribute")], [_Node("Count", [('I', n)], [])]),
        _Node("ObjectType", [('S', "AnimationStack")], [_Node("Count", [('I', 1)], [])]),
        _Node("ObjectType", [('S', "AnimationLayer")], [_Node("Count", [('I', 1)], [])]),
        _Node("ObjectType", [('S', "AnimationCurveNode")], [_Node("Count", [('I', len(curvenodes))], [])]),
        _Node("ObjectType", [('S', "AnimationCurve")], [_Node("Count", [('I', len(curves))], [])]),
    ]))

    objc = []
    for i in range(n):
        ex, ey, ez = rest_eul[i]
        tx, ty, tz = rest_pos[i]
        p70 = [P("RotationActive", "bool", "", "", ('I', 1)),
               P("RotationOrder", "enum", "", "", ('I', 0))]
        px, py, pz = prerot[i]
        if px or py or pz:
            p70.append(P("PreRotation", "Vector3D", "Vector", "", D(px), D(py), D(pz)))
        p70 += [P("Lcl Translation", "Lcl Translation", "", "A", D(tx), D(ty), D(tz)),
                P("Lcl Rotation", "Lcl Rotation", "", "A", D(ex), D(ey), D(ez)),
                P("Lcl Scaling", "Lcl Scaling", "", "A", D(1), D(1), D(1))]
        objc.append(_Node("Model", [('L', model_id[i]), NC(joints[i].name, "Model"), ('S', "LimbNode")], [
            _Node("Version", [('I', 232)], []),
            _Node("Properties70", [], p70),
            _Node("Shading", [('C', True)], []),
            _Node("Culling", [('S', "CullingOff")], []),
        ]))
        objc.append(_Node("NodeAttribute", [('L', attr_id[i]), NC("", "NodeAttribute"), ('S', "LimbNode")], [
            _Node("Properties70", [], [P("Size", "double", "Number", "", D(length[i]))]),
            _Node("TypeFlags", [('S', "Skeleton")], []),
        ]))
    objc.append(_Node("AnimationStack", [('L', stack_id), NC(clip.name, "AnimStack"), ('S', "")], [
        _Node("Properties70", [], [
            P("LocalStart", "KTime", "Time", "", ('L', 0)),
            P("LocalStop", "KTime", "Time", "", ('L', stop_kt)),
            P("ReferenceStart", "KTime", "Time", "", ('L', 0)),
            P("ReferenceStop", "KTime", "Time", "", ('L', stop_kt)),
        ]),
    ]))
    objc.append(_Node("AnimationLayer", [('L', layer_id), NC("Layer", "AnimLayer"), ('S', "")], []))
    for cn, mi, kind in curvenodes:
        dflt = (1.0, 1.0, 1.0) if kind == 'S' else (0.0, 0.0, 0.0)
        objc.append(_Node("AnimationCurveNode", [('L', cn), NC(kind, "AnimCurveNode"), ('S', "")], [
            _Node("Properties70", [], [P("d|X", "Number", "", "A", D(dflt[0])),
                                       P("d|Y", "Number", "", "A", D(dflt[1])),
                                       P("d|Z", "Number", "", "A", D(dflt[2]))]),
        ]))
    for cid, cn, ax, frames, vals in curves:
        times = [int(round(f / float(fps) * _KTIME)) for f in frames]
        objc.append(_Node("AnimationCurve", [('L', cid), NC("", "AnimCurve"), ('S', "")], [
            _Node("Default", [('D', 0.0)], []),
            _Node("KeyVer", [('I', 4008)], []),
            _Node("KeyTime", [('larr', times)], []),
            _Node("KeyValueFloat", [('farr', [float(v) for v in vals])], []),
            _Node("KeyAttrFlags", [('iarr', [8192])], []),
            _Node("KeyAttrDataFloat", [('farr', [0.0, 0.0, 0.0, 0.0])], []),
            _Node("KeyAttrRefCount", [('iarr', [len(vals)])], []),
        ]))
    top.append(_Node("Objects", [], objc))

    conns = []
    for i in range(n):
        p = joints[i].parent
        parent = 0 if (i == 0 or not (0 <= p < n) or p == i) else model_id[p]
        conns.append(_Node("C", [('S', "OO"), ('L', model_id[i]), ('L', parent)], []))
        conns.append(_Node("C", [('S', "OO"), ('L', attr_id[i]), ('L', model_id[i])], []))
    conns.append(_Node("C", [('S', "OO"), ('L', layer_id), ('L', stack_id)], []))
    for cn, mi, kind in curvenodes:
        prop = {'T': "Lcl Translation", 'R': "Lcl Rotation", 'S': "Lcl Scaling"}[kind]
        conns.append(_Node("C", [('S', "OO"), ('L', cn), ('L', layer_id)], []))
        conns.append(_Node("C", [('S', "OP"), ('L', cn), ('L', model_id[mi]), ('S', prop)], []))
    for cid, cn, ax, frames, vals in curves:
        conns.append(_Node("C", [('S', "OP"), ('L', cid), ('L', cn), ('S', "d|" + ax)], []))
    top.append(_Node("Connections", [], conns))

    with open(path, "wb") as fh:
        fh.write(_ser_fbx(top))
    return nframes


# ==================================================================== READ (ASCII or binary)
class _Node:
    __slots__ = ("name", "props", "children")

    def __init__(self, name, props, children):
        self.name = name
        self.props = props
        self.children = children

    def find(self, name):
        for c in self.children:
            if c.name == name:
                return c
        return None

    def find_all(self, name):
        return [c for c in self.children if c.name == name]


# ---- binary FBX ----
def _parse_binary(data):
    ver = struct.unpack_from("<I", data, 23)[0]
    is64 = ver >= 7500
    hdr = 25 if is64 else 13

    def read_prop(pos):
        t = chr(data[pos]); pos += 1
        if t == 'Y':
            v = struct.unpack_from("<h", data, pos)[0]; pos += 2
        elif t == 'C':
            v = bool(data[pos]); pos += 1
        elif t == 'I':
            v = struct.unpack_from("<i", data, pos)[0]; pos += 4
        elif t == 'F':
            v = struct.unpack_from("<f", data, pos)[0]; pos += 4
        elif t == 'D':
            v = struct.unpack_from("<d", data, pos)[0]; pos += 8
        elif t == 'L':
            v = struct.unpack_from("<q", data, pos)[0]; pos += 8
        elif t in 'SR':
            ln = struct.unpack_from("<I", data, pos)[0]; pos += 4
            raw = data[pos:pos + ln]; pos += ln
            v = raw.decode("latin-1") if t == 'S' else raw
        elif t in 'fdlib':
            arrlen, enc, clen = struct.unpack_from("<III", data, pos); pos += 12
            raw = data[pos:pos + clen]; pos += clen
            if enc == 1:
                raw = zlib.decompress(raw)
            fmt = {'f': 'f', 'd': 'd', 'l': 'q', 'i': 'i', 'b': 'b'}[t]
            v = list(struct.unpack("<" + fmt * arrlen, raw))
        else:
            raise ValueError("unknown FBX prop type %r" % t)
        return pos, v

    def read_record(pos):
        if is64:
            end, nprop, _plen = struct.unpack_from("<QQQ", data, pos); pos += 24
        else:
            end, nprop, _plen = struct.unpack_from("<III", data, pos); pos += 12
        namelen = data[pos]; pos += 1
        if end == 0 and nprop == 0 and namelen == 0:
            return None, pos                          # null terminator
        name = data[pos:pos + namelen].decode("latin-1"); pos += namelen
        props = []
        for _ in range(nprop):
            pos, val = read_prop(pos)
            props.append(val)
        children = []
        while pos < end - hdr:
            child, pos = read_record(pos)
            if child is None:
                break
            children.append(child)
        return _Node(name, props, children), int(end)

    nodes = []
    pos = 27
    while pos < len(data) - hdr:
        node, pos = read_record(pos)
        if node is None:
            break
        nodes.append(node)
    return _Node("", [], nodes)


# ---- ASCII FBX ----
def _parse_ascii(text):
    i = 0
    N = len(text)

    def skip_ws():
        nonlocal i
        while i < N:
            c = text[i]
            if c == ';':
                while i < N and text[i] != '\n':
                    i += 1
            elif c in " \t\r\n,":
                i += 1
            else:
                break

    def read_token():
        nonlocal i
        skip_ws()
        if i >= N:
            return None
        c = text[i]
        if c == '"':
            i += 1
            start = i
            while i < N and text[i] != '"':
                i += 1
            s = text[start:i]
            i += 1
            return ('str', s)
        if c in '{}':
            i += 1
            return ('brace', c)
        if c == '*':                                  # array marker *N { a: ... }
            i += 1
            start = i
            while i < N and text[i].isdigit():
                i += 1
            return ('array', int(text[start:i] or 0))
        start = i
        while i < N and text[i] not in " \t\r\n,{};":
            i += 1
        tok = text[start:i]
        if tok.endswith(':'):
            return ('key', tok[:-1])
        try:
            return ('num', int(tok))
        except ValueError:
            try:
                return ('num', float(tok))
            except ValueError:
                return ('word', tok)

    def parse_nodes(depth):
        nonlocal i
        nodes = []
        while True:
            save = i
            tok = read_token()
            if tok is None:
                break
            if tok[0] == 'brace' and tok[1] == '}':
                break
            if tok[0] != 'key':
                # stray token (e.g. 'a:' array content handled inline) — skip
                continue
            name = tok[1]
            props = []
            children = []
            while True:
                s2 = i
                t = read_token()
                if t is None:
                    break
                if t[0] == 'brace' and t[1] == '{':
                    children = parse_nodes(depth + 1)
                    break
                if t[0] == 'key' or (t[0] == 'brace' and t[1] == '}'):
                    i = s2                            # next node / close brace
                    break
                if t[0] == 'array':
                    props.append(('__arraylen__', t[1]))
                elif t[0] in ('num', 'str', 'word'):
                    props.append(t[1])
            nodes.append(_Node(name, props, children))
        return nodes

    return _Node("", [], parse_nodes(0))


def _flatten_props(node):
    """Numbers of a KeyTime/KeyValueFloat node — binary stores them as a single LIST property;
    ASCII stores them in an 'a:' child. Handle both."""
    a = node.find("a")
    props = a.props if a is not None else node.props
    out = []
    for p in props:
        if isinstance(p, list):
            out.extend(p)
        elif isinstance(p, (int, float)):
            out.append(p)
    return out


def read_fbx(path):
    """Parse an FBX (ASCII or binary) -> {'joints':[(name,parent,rest_quat,rest_pos)],
    'clips':[{'name','channels','frames'}], 'fps'}. channels = {bone:{'rot'|'loc'|'scl':(fr,vals)}}."""
    with open(path, "rb") as f:
        raw = f.read()
    if raw[:20] == b"Kaydara FBX Binary  ":
        root = _parse_binary(raw)
    else:
        root = _parse_ascii(raw.decode("latin-1", errors="replace"))

    objs = root.find("Objects")
    conns = root.find("Connections")
    if objs is None or conns is None:
        raise ValueError("FBX has no Objects/Connections")

    # id -> node info
    models = {}          # id -> {'name','props':{prop:vals}}
    curvenodes = {}      # id -> just a marker
    curves = {}          # id -> (times, values)
    for c in objs.children:
        if not c.props:
            continue
        oid = c.props[0] if isinstance(c.props[0], int) else None
        if oid is None:
            continue
        if c.name == "Model":
            raw_nm = str(c.props[1])
            nm = raw_nm.split("\x00\x01")[0] if "\x00\x01" in raw_nm else raw_nm.split("::")[-1]
            p70 = c.find("Properties70")
            lt = (0.0, 0.0, 0.0)
            lr = (0.0, 0.0, 0.0)
            pre = (0.0, 0.0, 0.0)
            post = (0.0, 0.0, 0.0)
            if p70 is not None:
                for P in p70.find_all("P"):
                    if not P.props:
                        continue
                    key = P.props[0]
                    nums = [x for x in P.props if isinstance(x, (int, float))]
                    if key == "Lcl Translation" and len(nums) >= 3:
                        lt = tuple(nums[-3:])
                    elif key == "Lcl Rotation" and len(nums) >= 3:
                        lr = tuple(nums[-3:])
                    elif key == "PreRotation" and len(nums) >= 3:
                        pre = tuple(nums[-3:])
                    elif key == "PostRotation" and len(nums) >= 3:
                        post = tuple(nums[-3:])
            models[oid] = {"name": nm, "lt": lt, "lr": lr, "pre": pre, "post": post}
        elif c.name == "AnimationCurveNode":
            curvenodes[oid] = True
        elif c.name == "AnimationCurve":
            kt = c.find("KeyTime")
            kv = c.find("KeyValueFloat")
            times = _flatten_props(kt) if kt is not None else []
            vals = _flatten_props(kv) if kv is not None else []
            curves[oid] = (times, vals)

    # connections
    cn_to_model = {}     # curvenode id -> (model id, prop 'Lcl Rotation'/...)
    cn_to_curves = {}    # curvenode id -> {'X':curveid,'Y':..,'Z':..}
    model_parent = {}    # model id -> parent model id (or None)
    for C in conns.find_all("C"):
        p = C.props
        if len(p) < 3:
            continue
        typ = p[0]
        src, dst = p[1], p[2]
        if typ == "OO":
            if src in curvenodes:                     # curvenode -> layer (ignore)
                pass
            elif src in models and dst in models:
                model_parent[src] = dst
        elif typ == "OP" and len(p) >= 4:
            prop = p[3]
            if src in curvenodes and dst in models:
                cn_to_model[src] = (dst, prop)
            elif src in curves and dst in curvenodes:
                axis = prop.split("|")[-1]            # 'd|X' -> 'X'
                cn_to_curves.setdefault(dst, {})[axis] = src

    # order models parent-first
    ids = list(models)
    order = []
    seen = set()

    def emit(mid):
        if mid in seen:
            return
        par = model_parent.get(mid)
        if par is not None and par in models and par not in seen:
            emit(par)
        seen.add(mid)
        order.append(mid)
    for mid in ids:
        emit(mid)
    index = {mid: k for k, mid in enumerate(order)}

    joints = []
    for mid in order:
        m = models[mid]
        par = model_parent.get(mid)
        pidx = index[par] if (par in index) else -1
        # full local rotation = PreRotation * Lcl Rotation * PostRotation^-1 (FBX rotation chain)
        q = _qmul(_euler_xyz_deg_to_quat(*m["pre"]),
                  _qmul(_euler_xyz_deg_to_quat(*m["lr"]),
                        _qconj(_euler_xyz_deg_to_quat(*m["post"]))))
        joints.append((m["name"], pidx, q, m["lt"]))

    # derive fps from the tightest key spacing across all curves (Blender bakes ~per frame)
    step = None
    for times, _v in curves.values():
        for a, b in zip(times, times[1:]):
            d = b - a
            if d > 0:
                step = d if step is None else min(step, d)
    fps = round(_KTIME / step) if step else 30

    def times_to_frames(times):
        return [int(round(t / float(step))) if step else i for i, t in enumerate(times)]

    def val_at(cid, frame_time):
        times, vals = curves[cid]
        if not times:
            return None
        if frame_time <= times[0]:
            return vals[0]
        if frame_time >= times[-1]:
            return vals[-1]
        for k in range(1, len(times)):
            if frame_time <= times[k]:
                f = (frame_time - times[k - 1]) / ((times[k] - times[k - 1]) or 1)
                return vals[k - 1] + (vals[k] - vals[k - 1]) * f
        return vals[-1]

    channels = {}
    for cn, (mid, prop) in cn_to_model.items():
        if mid not in models:
            continue
        slot = {'Lcl Rotation': 'rot', 'Lcl Translation': 'loc', 'Lcl Scaling': 'scl'}.get(prop)
        if slot is None:
            continue
        axes = cn_to_curves.get(cn, {})
        axis_curves = [axes.get(a) for a in "XYZ"]
        # union of all key times for this node's axes
        alltimes = sorted(set(t for cid in axis_curves if cid in curves for t in curves[cid][0]))
        if not alltimes:
            continue
        frames = times_to_frames(alltimes)
        dflt = 1.0 if slot == 'scl' else 0.0
        vecs = []
        for tt in alltimes:
            comp = []
            for cid in axis_curves:
                comp.append(val_at(cid, tt) if cid in curves else dflt)
            vecs.append(comp)
        if slot == 'rot':
            mm = models[mid]
            qpre = _euler_xyz_deg_to_quat(*mm["pre"])
            qposti = _qconj(_euler_xyz_deg_to_quat(*mm["post"]))
            vals = [_qmul(qpre, _qmul(_euler_xyz_deg_to_quat(v[0], v[1], v[2]), qposti)) for v in vecs]
        else:
            vals = [(v[0], v[1], v[2]) for v in vecs]
        channels.setdefault(models[mid]["name"], {})[slot] = (frames, vals)

    return {"joints": joints, "channels": channels, "fps": fps}
