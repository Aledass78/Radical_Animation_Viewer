"""
Pure3D (.p3d) animation WRITER — the inverse of the decoder.

Writes skeletal animation clips back into the Pure3D container so they can be added to a
character `.p3d`. Everything is written **inline** (each value channel carries its own
`[frames][values]`, `NumberOfFrames > 0`), which is a form the game's own loader reads and
which avoids the ZLIB keyframe buffer + region table entirely.

Verified foundations (see the analysis notes):
  * the container round-trips **byte-identical** (headers, dataSize/totalSize, child order);
  * AlignedU8 strings use `round_up(byteCount, 4)` padding (no forced null on aligned lengths);
  * inline value bytes re-encode **byte-identical** (compression is an exact inverse), so a
    fully-inline clip re-encodes byte-for-byte and a buffered clip re-encodes to inline that
    the decoder reads back identically.

Public API:
  * `reencode_inline(anim_chunk)`         -> bytes of an all-inline 0x121000 subtree
  * `build_clip(name, skeleton_joints, channels, num_frames, fps, cyclic)` -> bytes (from scratch)
  * `inject_clips(src_p3d_path, [clip_bytes...], out_path)` -> add clips to a copy of a .p3d
  * `serialize(chunk_id, payload, children, be)` -> one chunk's bytes
"""
import struct
import math

import p3d_core as core

# animation chunk ids
ANIM = 0x00121000
ANIMHEADER = 0x00121006
CHANPARAM = 0x00121007       # per value-type: [0][chunkID][count][count x u16 keycount]
INDEXMAP = 0x00121008
TRANSREF = 0x00121009
REGIONTBL = 0x00121010
AUX_HPCL = 0x00121101
LIMBPROPS = 0x00121400
LIMBREF = 0x00121401
ANIMTIME = 0x00121402
ZLIBBUF = 0x02F00000
ZLIBTRAIL = 0x02F00001
BONELIST = 0x00121002
BONE = 0x00121001
REF = 0x00121121

_CH = core._CH


def _build_anim_header(type_keys, num_bones, be, extra=()):
    """Build 0x121006 AnimationHeader the way SHIPPED INLINE clips do:
        [u32 0][u32 numBones]  +  one 0x121007 per value-type (sorted ascending by chunkID):
        0x121007 = [u32 0][u32 chunkID][u32 channelCount][channelCount x u16 keyCount]
    `type_keys` = {chunkID: [per-channel keyCount, ... in bone order]}.
    `extra` = already-serialised child chunks to append (e.g. a preserved 0x121400 LimbProps)."""
    ho = ">H" if be else "<H"
    entries = []
    for cid in sorted(type_keys):
        kc = type_keys[cid]
        payload = _u32(0, be) + _u32(cid, be) + _u32(len(kc), be) \
            + b"".join(struct.pack(ho, k & 0xFFFF) for k in kc)
        entries.append(serialize(CHANPARAM, payload, [], be))
    return serialize(ANIMHEADER, _u32(0, be) + _u32(num_bones, be), entries + list(extra), be)


# ------------------------------------------------------------------ primitives
def serialize(chunk_id, payload, children, be):
    """One Pure3D chunk: [id][dataSize=12+payload][totalSize=12+payload+children][payload][children]."""
    fmt = ">III" if be else "<III"
    body = bytes(payload) + b"".join(children)
    return struct.pack(fmt, chunk_id, 12 + len(payload), 12 + len(body)) + body


def aligned_str(s):
    """AlignedU8 string: [u8 field][field bytes], field = round_up(len, 4) (game convention)."""
    b = s.encode("latin-1")
    pad = (4 - (len(b) % 4)) % 4
    field = len(b) + pad
    if field > 255:
        raise ValueError("string too long: %r" % s)
    return bytes([field]) + b + b"\x00" * pad


def _u32(v, be):
    return struct.pack(">I" if be else "<I", v & 0xFFFFFFFF)


def _f32(v, be):
    return struct.pack(">f" if be else "<f", v)


def _pack_frames(frames, be):
    o = ">H" if be else "<H"
    return b"".join(struct.pack(o, f & 0xFFFF) for f in frames)


def _encode_raw(dec, v, be):
    """Encode ONE value's raw components (as decoded by core._values) back to bytes — exact inverse."""
    o = ">" if be else "<"
    if dec == "q6":
        return struct.pack(o + "hhh", int(v[0]), int(v[1]), int(v[2]))
    if dec == "q3":
        return struct.pack(o + "bbb", int(v[0]), int(v[1]), int(v[2]))
    if dec == "q4":
        return struct.pack(o + "ffff", *v)
    if dec == "h3":
        return struct.pack(o + "eee", *v)
    if dec == "h2":
        return struct.pack(o + "ee", *v)
    if dec == "f3":
        return struct.pack(o + "fff", *v)
    if dec == "f2":
        return struct.pack(o + "ff", *v)
    if dec == "f1":
        return struct.pack(o + "f", v[0])
    raise ValueError(dec)


# ------------------------------------------------------------------ re-encode a value channel inline
def _encode_channel_inline(v, be, buf, bases):
    """Serialize a value-channel chunk `v` as INLINE bytes (no 0x121121 child, NumberOfFrames>0)."""
    meta = _CH[v.chunk_id]
    role, hlen, nf_off, vbytes, dof, dec = meta
    n_inline = v.u32(nf_off)
    if n_inline > 0:                                   # already inline -> re-emit (byte-identical)
        frames = core._frames(v.data, hlen, n_inline, be)
        raw = core._values(v.data, hlen + 2 * n_inline, n_inline, dec, be)
        header = bytes(v.data[:hlen])
        count = n_inline
    else:                                              # buffered -> pull keys out of the ZLIB buffer
        ref = next((c for c in v.children if c.chunk_id == REF), None)
        if ref is None or buf is None:
            # nothing to write; emit an empty (0-frame) inline channel
            header = bytearray(v.data[:hlen])
            struct.pack_into(">I" if be else "<I", header, nf_off, 0)
            return serialize(v.chunk_id, bytes(header), [], be)
        count = ref.u32(4)
        off = ref.u32(8)
        rtag = ref.u16(12) if len(ref.data) >= 14 else 0
        fbase = bases[rtag] + off if 0 <= rtag < len(bases) else off
        vbase = fbase + 2 * count + (2 if count % 2 else 0)
        frames = core._frames(buf, fbase, count, be)
        raw = core._values(buf, vbase, count, dec, be)
        header = bytearray(v.data[:hlen])              # keep Version/Type/Mapping/BaseValues
        struct.pack_into(">I" if be else "<I", header, nf_off, count)   # NumberOfFrames = count
        header = bytes(header)
    body = header + _pack_frames(frames, be) + b"".join(_encode_raw(dec, r, be) for r in raw)
    return serialize(v.chunk_id, body, [], be)         # inline channel has NO children


def _strip_buffer_meta(c, be):
    """Recursively copy a chunk, dropping the region-size table 0x121010 (invalid once inline)."""
    kids = [_strip_buffer_meta(ch, be) for ch in c.children if ch.chunk_id != REGIONTBL]
    return serialize(c.chunk_id, c.data, kids, be)


def _copy(c, be):
    return serialize(c.chunk_id, c.data, [_copy(ch, be) for ch in c.children], be)


def reencode_inline(anim):
    """Re-serialize a parsed Animation (0x121000) chunk as an all-inline subtree (bytes),
    matching the metadata layout shipped inline clips use (0x121006 -> 0x121007, no buffer)."""
    be = anim.be
    buf = core._zlib_buffer(anim)
    bases = core._region_bases(anim, len(buf) if buf is not None else 0)

    bonelist_chunk = anim.find(BONELIST)
    bone_chunks = []
    type_keys = {}
    for kv in bonelist_chunk.find_all(BONE):
        chan = []
        for v in kv.children:
            if v.chunk_id in _CH:
                chan.append(_encode_channel_inline(v, be, buf, bases))
                nf_off = _CH[v.chunk_id][2]
                n = v.u32(nf_off)
                if n == 0:                              # buffered -> keycount from the reference
                    ref = next((c for c in v.children if c.chunk_id == REF), None)
                    n = ref.u32(4) if ref is not None else 0
                type_keys.setdefault(v.chunk_id, []).append(n)
            elif v.chunk_id == AUX_HPCL:               # keep the 'HPCL' aux chunk verbatim
                chan.append(_copy(v, be))
                type_keys.setdefault(AUX_HPCL, []).append(v.u32(8))
            elif v.chunk_id != REF:                     # keep any other aux; drop stale refs
                chan.append(_copy(v, be))
        bone_chunks.append(serialize(BONE, kv.data, chan, be))

    nbones = len(bone_chunks)
    # preserve the original 0x121400 LimbProps (IK) under the rebuilt header, if present
    orig_header = anim.find(ANIMHEADER)
    limbprops = [_copy(c, be) for c in (orig_header.children if orig_header else [])
                 if c.chunk_id == LIMBPROPS]
    header = _build_anim_header(type_keys, nbones, be, extra=limbprops)
    bonelist = serialize(BONELIST, bonelist_chunk.data, bone_chunks, be)
    # keep the IK limb references and the time chunk; drop buffer, IndexMap, region table
    limbs = [_copy(c, be) for c in anim.children if c.chunk_id == LIMBREF]
    animtime = [_copy(c, be) for c in anim.children if c.chunk_id == ANIMTIME]
    return serialize(ANIM, anim.data, [header, bonelist] + limbs + animtime, be)


# ------------------------------------------------------------------ build a clip from scratch
def _encode_value_channel(chunk_id, ttype, frames, values, be, mapping=0, base=(0.0, 0.0, 0.0)):
    """Build one INLINE value channel from (frames, raw component tuples)."""
    role, hlen, nf_off, vbytes, dof, dec = _CH[chunk_id]
    tt = ttype.encode("ascii")[:4].ljust(4, b"\x00")
    if be:
        tt = bytes(reversed(tt))
    if hlen == 12:
        header = _u32(0, be) + tt + _u32(len(frames), be)
    else:                                              # 26-byte header: + Mapping + BaseValues
        header = (_u32(0, be) + tt + struct.pack(">H" if be else "<H", mapping)
                  + _f32(base[0], be) + _f32(base[1], be) + _f32(base[2], be) + _u32(len(frames), be))
    body = header + _pack_frames(frames, be) + b"".join(_encode_raw(dec, v, be) for v in values)
    return serialize(chunk_id, body, [], be)


def _quat_to_q6(q):
    """(x,y,z,w) unit quat -> raw s16 triple for a Quaternion6 channel (w>=0 canonical)."""
    x, y, z, w = q
    if w < 0:
        x, y, z = -x, -y, -z                           # q and -q are the same rotation
    return (int(round(x * 32767)), int(round(y * 32767)), int(round(z * 32767)))


def build_clip(name, joints, channels, num_frames, fps=30, cyclic=0, be=False):
    """Build a full 0x121000 Animation subtree (bytes) from decoded channels.

    joints    -- list with .name (bone order for the AnimationBone list; only animated bones are written)
    channels  -- {bone_name: {'rot': (frames, quats[xyzw]), 'loc': (frames, xyz), 'scl': (frames, xyz)}}
                 rot -> Quaternion6 (0x121112); loc/scl -> Vector3DOF f32 (0x121104).
    """
    order = [(j if isinstance(j, str) else j.name) for j in joints] if joints else list(channels)
    animated = [b for b in order if b in channels] + [b for b in channels if b not in set(order)]

    bone_chunks = []
    type_keys = {}                                      # chunkID -> [per-channel keyCount, ...]
    for bone in animated:
        slots = channels[bone]
        chan = []
        if "rot" in slots:
            frames, quats = slots["rot"]
            vals = [_quat_to_q6(q) for q in quats]
            chan.append(_encode_value_channel(0x00121112, "ROT\0", frames, vals, be))
            type_keys.setdefault(0x00121112, []).append(len(frames))
        if "loc" in slots:
            frames, xyz = slots["loc"]
            chan.append(_encode_value_channel(0x00121104, "TRAN", frames, xyz, be))
            type_keys.setdefault(0x00121104, []).append(len(frames))
        if "scl" in slots:
            frames, xyz = slots["scl"]
            chan.append(_encode_value_channel(0x00121104, "SCL\0", frames, xyz, be))
            type_keys.setdefault(0x00121104, []).append(len(frames))
        if not chan:
            continue
        payload = _u32(0, be) + aligned_str(bone) + _u32(0, be) + _u32(len(chan), be)
        bone_chunks.append(serialize(BONE, payload, chan, be))

    nbones = len(bone_chunks)
    header = _build_anim_header(type_keys, nbones, be)  # 0x121006 -> 0x121007 (shipped-inline layout)
    bonelist = serialize(BONELIST, _u32(0, be) + _u32(nbones, be), bone_chunks, be)
    animtime = serialize(ANIMTIME, _u32(0, be) + _f32(num_frames / float(fps or 30), be), [], be)

    tt = b"PTRN"
    if be:
        tt = bytes(reversed(tt))
    anim_payload = (_u32(0, be) + aligned_str(name) + tt
                    + _f32(float(num_frames), be) + _f32(float(fps), be) + _u32(cyclic, be))
    return serialize(ANIM, anim_payload, [header, bonelist, animtime], be)


# ------------------------------------------------------------------ import from JSON
def clip_from_json(json_path, be=False):
    """Read a clip exported by p3d_export.export_json and build a 0x121000 subtree (bytes).

    Round-trips our own JSON; also accepts hand-authored JSON with the same shape:
      {"clip": {"name","frameCount","fps","channels": {bone: {rot|loc|scl: {frames, values}}}},
       "skeleton": {"joints": [{"name", ...}]}}   (skeleton optional — used only for bone order)
    """
    import json
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    clip = data["clip"]
    names = [j["name"] for j in data.get("skeleton", {}).get("joints", [])]
    channels = {}
    for bone, slots in clip["channels"].items():
        out = {}
        for slot, cur in slots.items():
            frames = [int(x) for x in cur["frames"]]
            vals = [tuple(v) for v in cur["values"]]
            out[slot] = (frames, vals)
        channels[bone] = out
    return build_clip(clip.get("name", "IMPORTED"), names, channels,
                      int(clip.get("frameCount", 1)), fps=clip.get("fps", 30), be=be)


# ------------------------------------------------------------------ replace / rebuild
def _rebuild_root(root, be, transform):
    """Re-serialise the file root, running `transform(child)->bytes|None` over each child."""
    parts = []
    for c in root.children:
        b = transform(c)
        if b is not None:
            parts.append(b)
    fmt = ">III" if be else "<III"
    body = bytes(root.data) + b"".join(parts)
    return struct.pack(fmt, root.chunk_id, 12 + len(root.data), 12 + len(body)) + body


def replace_clip(src_path, target_name, new_clip_bytes, out_path):
    """Replace the first Animation clip named `target_name` with `new_clip_bytes`. Returns bool."""
    with open(src_path, "rb") as f:
        raw = f.read()
    root, be = core.parse_bytes(raw)
    if root is None:
        raise ValueError("not a valid .p3d")
    done = [False]

    def tr(c):
        if not done[0] and c.chunk_id == ANIM and c.find(BONELIST) is not None:
            try:
                nm = c.p3d_string(4)[0]
            except Exception:
                nm = None
            if nm == target_name:
                done[0] = True
                return bytes(new_clip_bytes)
        return _copy(c, be)

    out = _rebuild_root(root, be, tr)
    with open(out_path, "wb") as f:
        f.write(out)
    return done[0]


def clip_names(src_path):
    """List Animation clip names in a .p3d (in file order)."""
    with open(src_path, "rb") as f:
        root, be = core.parse_bytes(f.read())
    out = []
    if root:
        for c in root.children:
            if c.chunk_id == ANIM and c.find(BONELIST) is not None:
                try:
                    out.append(c.p3d_string(4)[0])
                except Exception:
                    pass
    return out


# ------------------------------------------------------------------ inject into a .p3d
def inject_clips(src_path, clip_bytes_list, out_path):
    """Append Animation clips (already-serialized 0x121000 bytes) to a copy of `src_path`.

    Clips are added as new children at the end of the root chunk; the root's totalSize is
    bumped accordingly. Returns the number of clips written.
    """
    with open(src_path, "rb") as f:
        raw = f.read()
    root, be = core.parse_bytes(raw)
    if root is None:
        raise ValueError("not a valid .p3d")
    extra = b"".join(clip_bytes_list)
    # root header: [id @0][dataSize @4][totalSize @8]; total spans the whole file
    fmt = ">I" if be else "<I"
    total = struct.unpack_from(fmt, raw, 8)[0]
    new_total = total + len(extra)
    out = bytearray(raw)
    struct.pack_into(fmt, out, 8, new_total)
    # place the new clips at the end of the root's region (== end of file for these assets)
    out = bytes(out[:total]) + extra + bytes(out[total:])
    with open(out_path, "wb") as f:
        f.write(out)
    return len(clip_bytes_list)
