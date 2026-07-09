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
| Open a file | **📂 Open** or **File ▸ Open** (Ctrl+O) |
| Save changes | **💾 Save** (Ctrl+S) / **Save As** (Ctrl+Shift+S) |
| Undo / redo an edit | **↶ Undo** (Ctrl+Z) / **↷ Redo** (Ctrl+Y) |
| Export the current clip | **📤 Export BVH** button, or **File ▸ Export** |
| Import a BVH | **📥 Import BVH** button, or **File ▸ Import / Write** |
| Delete the selected clip | **🗑 Delete clip** button, or **File ▸ Import / Write** |
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

## Importing BVH

**📥 Import BVH** (button, or **File ▸ Import / Write**) reads a `.bvh` (from Blender, Maya, our
own **Export BVH**, …) and offers three actions:

- **View it** — loads the BVH as a skeleton + clip and plays it in the viewer, no `.p3d` needed
  (shows the raw BVH, before the game-faithful policy).
- **Add as a NEW clip** to the loaded `.p3d` — you name it.
- **REPLACE an existing clip** (pick it from the list); the new clip keeps the replaced clip's
  **name** so it occupies the same animation slot.

Add/Replace apply to the **in-memory document** (see below) — undoable, and written only when you
**Save**.

**Game-faithful import policy.** On Add/Replace the tool reshapes the clip to match how the shipped
game authors the root region (verified across `alex`/`alex_boss`/`evolved`), so an imported clip
behaves in-game like a real one:

- **Body bones** (Pelvis, Spine, limbs, head, hands, grapple anchors) — kept **exactly as authored**,
  so a flip/roll on `Pelvis`+`Spine` survives intact.
- **`Motion_Root`** — keeps translation and **facing**, but its rotation is constrained to a **pure
  vertical yaw** (how the game turns a character; height-preserving, so the feet stay grounded). A
  stray pitch/roll on the root — e.g. from rotating the whole rig in Blender — is removed so the
  character can't tip over / lift off the floor.
- **`Balance_Root`** — dropped (no shipped clip ever animates it).

This replaced the old "structure like a template clip" filter. Validated by round-tripping shipped
clips through export→import: a 180° **turn** keeps its grounded yaw and a parkour **back-flip** keeps
its full jump arc, both frame-for-frame identical to the originals (feet + facing).

The reader (`p3d_bvh.py`) respects each joint's declared `CHANNELS` order and converts Euler →
quaternion; a BVH exported by this tool round-trips to the original pose (~3×10⁻⁶). Add/Replace are
disabled until a character `.p3d` is loaded (that's the skeleton the clip is written against).

**Coordinate fix (Blender / Z-up sources).** The game (and this viewer) are **Y-up**; Blender is
**Z-up**. When Blender imports/exports a BVH it rotates the **entire coordinate system** Y-up→Z-up —
offsets get rotated *and* every bone's rotation gets **conjugated** (a change of basis). The dialog's
**Coordinate fix — rotate axes X / Y / Z** (degrees) undoes this: enter **X = −90** for a Z-up
Blender source, **0** for anything that came straight from the game. It's applied as a proper change
of basis to every bone, so it fixes both the orientation **and** the per-bone twist.

> **BVH through Blender — now handled.** Earlier this produced twisted arms/head (and sank the root
> below ground): Blender's Z-up conversion left every bone rolled 90° about its own axis — invisible
> in a stick-figure view but it twists the skinned mesh in-game. Import with **X = −90** and it's
> recovered **exactly** (verified 0.00° per-bone vs the original, twist included). Note Blender may
> also **resample** the clip (e.g. 410 frames @30 → 250 @24); set your Blender scene to 30 fps and the
> right frame range to keep the timing.

## Editing model — everything is in memory

Opening a `.p3d` reads it **entirely into memory** as an editable document (`p3d_write.Document`).
From then on the tool has **no dependency on the file on disk** — you can move or delete the original
and keep working; nothing is written until you **Save**.

- **Add / Replace / Import / Delete / inline-convert** all mutate the in-memory document. The **Clips**
  list refreshes instantly and the affected clip is auto-selected — no reopening.
- **↶ Undo / ↷ Redo** (Ctrl+Z / Ctrl+Y) step through every edit; the title dot (`●`) marks unsaved
  changes.
- **💾 Save** (Ctrl+S) writes the whole document back to the current file (recreating it if it was
  deleted). **Save As** (Ctrl+Shift+S) writes a copy and switches to it. Closing with unsaved changes
  prompts you.

**File ▸ Import / Write** (all now in-memory edits, via `p3d_write.py`):

- **Import BVH…** — see above (view / add / replace).
- **JSON clip → inject** — reads a clip JSON (as exported above), builds a valid animation subtree,
  and adds it to the document. Round-trip **export JSON → edit → import** is verified.
- **Delete selected clip** — removes it from the document (undoable).
- **Re-save (clips inline)** — converts every clip to inline channels (no ZLIB keyframe buffer).

Everything is written **inline** (each channel carries its own `[frames][values]`) — the game's own
channel format. The container round-trips **byte-identical** (an unedited Save reproduces the input
exactly), and edited clips re-decode identically.

> **Untested in-game.** The output is structurally valid and decoder-verified, but hasn't been
> loaded in the actual game here. Injected clips likely need bone names that match the target
> skeleton and a registered clip/animation-table entry to be usable in-game — treat that as
> experimental. See `../docs/ANIMATION_FORMAT.md` §14.

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
