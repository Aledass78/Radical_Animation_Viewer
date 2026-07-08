# Pure3D Animation Viewer (desktop)

A standalone Python GUI that opens Prototype 2 **Pure3D (`.p3d`)** files and plays their
skeletal animations — the same decode as the in-repo web viewer (`anim_viewer.html`), but as
a native desktop app with **zero third-party dependencies**.

![clips + 3D skeleton + transport](.)

## What it does

- Parses `.p3d` files, **PC (little-endian)** and **PS3 (big-endian)**, auto-detected.
- Reads the skeleton (`0x00023000`) into a bind pose.
- Decodes **every** animation channel of every clip (`0x00121000`): rotation
  (`Quaternion6/3Compressed`), translation and scale (`Vector1/2/3DOF[Compressed]`).
- Poses a 3D stick figure with column-vector **forward kinematics** and quaternion **SLERP**,
  applying rotation + translation + scale exactly as the game composes them.

See [`../docs/ANIMATION_FORMAT.md`](../docs/ANIMATION_FORMAT.md) for the format.

## Requirements

- **Python 3.8+** with **tkinter** (Tk).
  - Windows / macOS: bundled with the standard python.org installer — nothing to install.
  - Linux: `sudo apt install python3-tk` (or your distro's equivalent).
- No other packages (`struct`, `zlib`, `math` are stdlib).

## Run

```bash
cd Pure3D_animation_viewer
python pure3d_anim_viewer.py                 # opens; auto-loads alex.p3d if the repo is nearby
python pure3d_anim_viewer.py path/to/file.p3d
```

## Controls

| Action | Control |
|---|---|
| Orbit camera | drag in the 3D view |
| Zoom | mouse wheel |
| Select a bone (highlights its chain) | click a joint, or pick it in **Bones** |
| Play / pause | **▶ Play** button or **Space** |
| Step one frame | **←** / **→** |
| Choose a clip | click it in **Clips** (type in the box to filter) |
| See full clip names | drag the sash to widen the **Clips** panel, or use its horizontal scrollbar |
| Open a file | **📂 Open .p3d** or **File ▸ Open** (Ctrl+O) |
| Export the current clip | **💾 Export BVH** button, or **File ▸ Export** |
| Hide helper bones | checkbox, top-right (on by default) |

The **Clips** / **Bones** panels are resizable — drag the sash between them, or the one between the
lists and the 3D view.

The **Bones** list tags each bone with the channels it animates in the current clip, e.g.
`Pelvis [ROT+LOC]`, `Spine_1 [ROT]`, and marks helper bones with `·helper`.

## Exporting

**File ▸ Export** (or the **💾 Export BVH** button) writes the currently selected clip to:

- **BVH** (`.bvh`) — **Blender-compatible** (and Maya / MotionBuilder / Unity / …). Import in
  Blender with **File ▸ Import ▸ Motion Capture (.bvh)** — no addon needed. Rotation + root/bone
  translation are baked to Euler; scale (rare, helper-bones only) is dropped. Coordinates are the
  game's native Y-up, so leave Blender's **Y Up** import option on. The writer is verified by a
  round-trip (re-parse + FK) to ~3×10⁻⁶ against the viewer's own pose.
- **JSON** (`.json`) — a lossless dump of the decoded skeleton + channels (rotation quaternions,
  translation, scale) for custom pipelines.
- **ALL clips → BVH folder** — batch-writes one `.bvh` per clip.

> For the highest-fidelity Blender path (quaternions, no Euler baking, plus scale), use the
> bundled Blender addon instead: `P3DAddon` → *File ▸ Import ▸ Pure 3D Animation (.p3d)*, which
> imports clips straight onto an armature as Actions. BVH is the portable, addon-free option.

## Helper bones

Some bones are **non-deforming drivers/attachment points**, not body geometry:
`R_Wrist_Grapple`, `Root_Grapple`, `Shoulder_Con_L/R`. Their translation channel parks them as
grapple / constraint targets, so they get "flung" tens of units from the body in grapple and shield
clips (and sit at arbitrary parked offsets otherwise). This is **intended** — the game doesn't
render them as part of the mesh — but it clutters the stick figure, so they are **hidden by
default**. Untick **Hide helper bones** to show them.

Independently, the viewer never draws a joint whose posed position is non-finite or absurdly far
(`|coord| > 1e4`): a few clips (e.g. `alex_grap_beatdown_rcv`) store a garbage sentinel value on a
helper bone's final keyframe — the decode is correct, the data is junk — and this guard keeps one
bad keyframe from drawing a line to ~3×10¹¹.

## Animation-only packages

Some `.p3d` files (e.g. `art/packages/animations/smartnodesBase/smartnodesBase.p3d`) contain
**only clips, no skeleton** — they're authored to play on a character rig from another file.
Load a character first (e.g. `alex.p3d`), **then** open the package: the viewer reuses the
on-screen skeleton and reports how many of the clip's bones matched it. If you open a package
with no skeleton loaded, it tells you to load a character first.

## Files

| File | Purpose |
|---|---|
| `pure3d_anim_viewer.py` | the tkinter GUI (rendering, camera, transport, file handling) |
| `p3d_core.py` | self-contained engine: parser + skeleton + channel decoder + FK |

`p3d_core.py` is a faithful port of the verified analysis code in `../_analysis`
(`p3d.py`, `skeleton.py`, `anim_channels.py`) and can be reused headless:

```python
import p3d_core as core
name, joints, clips, be = core.load_p3d("alex.p3d")
model = core.Model("alex.p3d", joints, name, clips)
positions = model.pose_world(clip_idx=0, frame=12.0)   # list of (x, y, z)
```

## Caveat

A single decoded clip won't be pixel-identical to gameplay: the engine merges animation
layers, applies leg IK, and retargets skeletons at runtime — none of which live in the `.p3d`.
The clip itself is decoded and posed exactly as authored.
