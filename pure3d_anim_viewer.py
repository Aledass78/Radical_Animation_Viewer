"""
Pure3D Animation Viewer  —  desktop (tkinter) build.

A standalone Python GUI that opens Prototype 2 Pure3D (.p3d) files and plays their skeletal
animations exactly the way the web viewer (anim_viewer.html) does: it decodes the skeleton
and every rotation / translation / scale channel, then poses a 3D stick figure with
forward kinematics + quaternion SLERP.

Requirements: Python 3.8+ with tkinter (bundled with the standard Windows/macOS installers;
on Linux install `python3-tk`). No other dependencies. All decoding lives in p3d_core.py.

Usage:
    python pure3d_anim_viewer.py [file.p3d]

Controls:
    * drag in the 3D view to orbit, mouse wheel to zoom
    * click a joint (or pick one in the Bones list) to highlight its chain
    * Space = play/pause, Left/Right arrows = step one frame
    * File > Open (or the "Open .p3d" button) to load a file

Animation-only packages (e.g. art/packages/animations/smartnodesBase/smartnodesBase.p3d)
carry clips but NO skeleton — load a character (e.g. alex.p3d) first, then open the package
and its clips play on that skeleton.
"""
import os
import sys
import math
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import p3d_core as core
import p3d_export as pexport


# --- colours (dark theme) ---
BG = "#12161b"
GRID = "#232b35"
BONE = "#8a94a3"
JOINT = "#66717f"
ACCENT = "#4aa3df"
SEL = "#f0a53a"
TEXT = "#c7ccc4"
AX = ("#4aa3df", "#e0703a", "#33b07a")   # X, Y, Z

# Non-deforming helper / attachment / driver bones (e.g. R_Wrist_Grapple, Root_Grapple,
# Shoulder_Con_L/R). They aren't body geometry — their TRAN channel parks them as grapple /
# constraint targets, so they get "flung" far from the body. Hidden by default so the
# humanoid skeleton reads cleanly. Matched by substring on the joint name.
HELPER_TOKENS = ("Grapple", "Con_", "Attach", "Weapon", "Prop", "Marker",
                 "Dummy", "Helper", "Null", "Target")


def is_helper_bone(name):
    return any(tok in name for tok in HELPER_TOKENS)


class Viewer(tk.Tk):
    def __init__(self, initial=None):
        super().__init__()
        self.title("Pure3D Animation Viewer")
        self.geometry("1200x760")
        self.minsize(880, 560)
        self.configure(bg=BG)

        self.model = None
        self.clip_idx = 0
        self.frame = 0.0
        self.sel = 0
        self.playing = False
        self.fps = 30
        self._after = None
        self._helper = set()          # joint indices classified as helper bones
        # camera
        self.yaw = 0.5
        self.pitch = -0.12
        self.dist = 3.2
        self._drag = None
        self._proj = []          # cached projected joint points for picking

        self._build_menu()
        self._build_ui()
        self._bind_keys()

        if initial and os.path.isfile(initial):
            self.load(initial)
        else:
            default = self._find_default()
            if default:
                self.load(default)

    # ------------------------------------------------------------------ UI
    def _build_menu(self):
        m = tk.Menu(self)
        fm = tk.Menu(m, tearoff=0)
        fm.add_command(label="Open .p3d…", command=self.open_dialog, accelerator="Ctrl+O")
        fm.add_separator()
        ex = tk.Menu(fm, tearoff=0)
        ex.add_command(label="Current clip → BVH (Blender / mocap)…", command=self.export_bvh)
        ex.add_command(label="Current clip → JSON (decoded curves)…", command=self.export_json)
        ex.add_command(label="ALL clips → BVH folder…", command=self.export_all_bvh)
        fm.add_cascade(label="Export", menu=ex)
        fm.add_separator()
        fm.add_command(label="Exit", command=self.destroy)
        m.add_cascade(label="File", menu=fm)
        hm = tk.Menu(m, tearoff=0)
        hm.add_command(label="About", command=self._about)
        m.add_cascade(label="Help", menu=hm)
        self.config(menu=m)

    def _build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=BG, foreground=TEXT, fieldbackground="#1b2028")
        style.configure("TButton", padding=4)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("TFrame", background=BG)

        # top toolbar
        top = ttk.Frame(self, padding=(8, 6))
        top.pack(side="top", fill="x")
        ttk.Button(top, text="📂 Open .p3d", command=self.open_dialog).pack(side="left")
        ttk.Button(top, text="💾 Export BVH", command=self.export_bvh).pack(side="left", padx=(6, 0))
        self.src_lbl = ttk.Label(top, text="no file loaded")
        self.src_lbl.pack(side="left", padx=12)
        self.hide_helpers = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Hide helper bones", variable=self.hide_helpers,
                        command=self._toggle_helpers).pack(side="right")

        # main split: [ left lists ] | [ 3d view ]  — draggable sash (resizable)
        body = ttk.PanedWindow(self, orient="horizontal")
        body.pack(side="top", fill="both", expand=True)

        # left column: Clips over Bones, also a draggable vertical sash
        left = ttk.PanedWindow(body, orient="vertical")
        body.add(left, weight=0)

        # --- Clips pane ---
        cpane = ttk.Frame(left, padding=(8, 4))
        left.add(cpane, weight=3)
        ttk.Label(cpane, text="Clips").pack(anchor="w")
        self.filter_var = tk.StringVar()
        fe = ttk.Entry(cpane, textvariable=self.filter_var)
        fe.pack(fill="x")
        fe.bind("<KeyRelease>", lambda e: self._refill_clips())
        cf = ttk.Frame(cpane)
        cf.pack(fill="both", expand=True, pady=(3, 0))
        self.clip_list = tk.Listbox(cf, width=42, height=18, activestyle="none",
                                    bg="#1b2028", fg=TEXT, selectbackground=ACCENT,
                                    selectforeground="#0b0e12", highlightthickness=0, exportselection=False)
        cvsb = ttk.Scrollbar(cf, orient="vertical", command=self.clip_list.yview)
        chsb = ttk.Scrollbar(cf, orient="horizontal", command=self.clip_list.xview)
        self.clip_list.config(yscrollcommand=cvsb.set, xscrollcommand=chsb.set)
        self.clip_list.grid(row=0, column=0, sticky="nsew")
        cvsb.grid(row=0, column=1, sticky="ns")
        chsb.grid(row=1, column=0, sticky="ew")
        cf.rowconfigure(0, weight=1)
        cf.columnconfigure(0, weight=1)
        self.clip_list.bind("<<ListboxSelect>>", self._on_clip)

        # --- Bones pane ---
        bpane = ttk.Frame(left, padding=(8, 4))
        left.add(bpane, weight=2)
        ttk.Label(bpane, text="Bones").pack(anchor="w")
        bf = ttk.Frame(bpane)
        bf.pack(fill="both", expand=True, pady=(3, 0))
        self.bone_list = tk.Listbox(bf, width=42, height=12, activestyle="none",
                                    bg="#1b2028", fg=TEXT, selectbackground=SEL,
                                    selectforeground="#0b0e12", highlightthickness=0, exportselection=False)
        bvsb = ttk.Scrollbar(bf, orient="vertical", command=self.bone_list.yview)
        bhsb = ttk.Scrollbar(bf, orient="horizontal", command=self.bone_list.xview)
        self.bone_list.config(yscrollcommand=bvsb.set, xscrollcommand=bhsb.set)
        self.bone_list.grid(row=0, column=0, sticky="nsew")
        bvsb.grid(row=0, column=1, sticky="ns")
        bhsb.grid(row=1, column=0, sticky="ew")
        bf.rowconfigure(0, weight=1)
        bf.columnconfigure(0, weight=1)
        self.bone_list.bind("<<ListboxSelect>>", self._on_bone)

        # 3D canvas (takes the extra space when the window is resized)
        self.canvas = tk.Canvas(body, bg=BG, highlightthickness=0, width=760)
        body.add(self.canvas, weight=1)
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._motion)
        self.canvas.bind("<ButtonRelease-1>", self._release)
        self.canvas.bind("<MouseWheel>", self._wheel)          # Windows/macOS
        self.canvas.bind("<Button-4>", lambda e: self._zoom(0.92))  # Linux up
        self.canvas.bind("<Button-5>", lambda e: self._zoom(1.08))  # Linux down
        self.canvas.bind("<Configure>", lambda e: self.draw())

        # transport bar
        tb = ttk.Frame(self, padding=(8, 6))
        tb.pack(side="bottom", fill="x")
        self.play_btn = ttk.Button(tb, text="▶ Play", width=8, command=self.toggle_play)
        self.play_btn.pack(side="left")
        self.frame_scale = tk.Scale(tb, from_=0, to=1, orient="horizontal", showvalue=False,
                                    bg=BG, fg=TEXT, troughcolor="#1b2028", highlightthickness=0,
                                    command=self._on_scrub)
        self.frame_scale.pack(side="left", fill="x", expand=True, padx=8)
        self.frame_lbl = ttk.Label(tb, text="0 / 0", width=12)
        self.frame_lbl.pack(side="left")
        ttk.Label(tb, text="FPS").pack(side="left", padx=(8, 2))
        self.fps_var = tk.IntVar(value=30)
        tk.Spinbox(tb, from_=1, to=120, width=4, textvariable=self.fps_var,
                   bg="#1b2028", fg=TEXT, highlightthickness=0,
                   command=self._on_fps).pack(side="left")

        self.status = ttk.Label(self, text="Ready.", padding=(8, 2), anchor="w")
        self.status.pack(side="bottom", fill="x")

    def _bind_keys(self):
        self.bind("<Control-o>", lambda e: self.open_dialog())
        self.bind("<space>", lambda e: self.toggle_play())
        self.bind("<Left>", lambda e: self.step(-1))
        self.bind("<Right>", lambda e: self.step(1))

    # -------------------------------------------------------------- loading
    def _find_default(self):
        here = os.path.dirname(os.path.abspath(__file__))
        for up in ("..", "."):
            cand = os.path.normpath(os.path.join(here, up, "Pc_Version", "art", "alex", "alex.p3d"))
            if os.path.isfile(cand):
                return cand
        return None

    def open_dialog(self):
        path = filedialog.askopenfilename(title="Open Pure3D file",
                                          filetypes=[("Pure3D", "*.p3d"), ("All files", "*.*")])
        if path:
            self.load(path)

    # ---------------------------------------------------------- export
    def _safe_name(self, s):
        return "".join(c if (c.isalnum() or c in "._-") else "_" for c in s)

    def _current_clip_name(self):
        return self.model.clips[self.clip_idx].name if self.model else ""

    def export_bvh(self):
        if not self.model:
            messagebox.showinfo("Export", "Open a .p3d file first.")
            return
        clip = self.model.clips[self.clip_idx]
        path = filedialog.asksaveasfilename(
            title="Export clip to BVH (Blender: File ▸ Import ▸ Motion Capture)",
            defaultextension=".bvh", initialfile=self._safe_name(clip.name) + ".bvh",
            filetypes=[("Biovision Hierarchy", "*.bvh")])
        if not path:
            return
        try:
            n = pexport.export_bvh(self.model, self.clip_idx, path, fps=self.fps)
        except Exception as e:
            messagebox.showerror("Export failed", str(e))
            return
        self.status.config(text="Exported %d frames → %s" % (n, os.path.basename(path)))

    def export_json(self):
        if not self.model:
            messagebox.showinfo("Export", "Open a .p3d file first.")
            return
        clip = self.model.clips[self.clip_idx]
        path = filedialog.asksaveasfilename(
            title="Export decoded clip to JSON",
            defaultextension=".json", initialfile=self._safe_name(clip.name) + ".json",
            filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            nb = pexport.export_json(self.model, self.clip_idx, path, fps=self.fps)
        except Exception as e:
            messagebox.showerror("Export failed", str(e))
            return
        self.status.config(text="Exported %d channel-bones → %s" % (nb, os.path.basename(path)))

    def export_all_bvh(self):
        if not self.model:
            messagebox.showinfo("Export", "Open a .p3d file first.")
            return
        folder = filedialog.askdirectory(title="Choose a folder for one .bvh per clip")
        if not folder:
            return
        n = 0
        for ci in range(len(self.model.clips)):
            fn = os.path.join(folder, self._safe_name(self.model.clips[ci].name) + ".bvh")
            try:
                pexport.export_bvh(self.model, ci, fn, fps=self.fps)
                n += 1
            except Exception:
                pass
            if ci % 25 == 0:
                self.status.config(text="Exporting BVH… %d/%d" % (ci + 1, len(self.model.clips)))
                self.update_idletasks()
        messagebox.showinfo("Export", "Wrote %d BVH file(s) to\n%s" % (n, folder))
        self.status.config(text="Exported %d BVH file(s)." % n)

    def load(self, path):
        self.status.config(text="Loading " + os.path.basename(path) + " …")
        self.update_idletasks()
        try:
            name, joints, clips, be = core.load_p3d(path)
        except Exception as e:
            messagebox.showerror("Open failed", "Could not parse %s:\n%s" % (os.path.basename(path), e))
            self.status.config(text="Load failed.")
            return
        if not clips:
            messagebox.showwarning("No animations", "No animation clips found in %s." % os.path.basename(path))
            self.status.config(text="No animations.")
            return

        reused = False
        if joints is None:
            # animation-only package: reuse the skeleton already loaded
            if self.model is None or not self.model.joints:
                messagebox.showinfo(
                    "No skeleton",
                    "“%s” contains only animations (no skeleton).\n\n"
                    "Load a character .p3d that has a skeleton first (e.g. alex.p3d), then "
                    "open this animation package to play its clips on that skeleton."
                    % os.path.basename(path))
                self.status.config(text="No skeleton — load a character first.")
                return
            joints = self.model.joints
            name = self.model.name
            reused = True

        self.model = core.Model(os.path.basename(path), joints, name, clips)
        self._helper = {i for i, j in enumerate(self.model.joints) if is_helper_bone(j.name)}
        self.clip_idx = 0
        self.frame = 0.0
        self.playing = False
        self.play_btn.config(text="▶ Play")
        self.sel = self._default_sel()
        self.endian = "PS3 (big-endian)" if be else "PC (little-endian)"

        cov = self.model.bone_coverage()
        self._refill_clips()
        self._fill_bones()
        self._select_clip_row(0)
        self._sync_transport()
        self.draw()

        srctxt = "%s · %s · %d clips · %d joints" % (
            self.model.source, self.endian, len(clips), len(joints))
        if reused:
            srctxt += "  (reusing %s, %d%% bones matched)" % (name, round(cov * 100))
        self.src_lbl.config(text=srctxt)
        self.status.config(text="Loaded %d clip(s)." % len(clips))
        if reused and cov < 0.5:
            messagebox.showwarning(
                "Bone mismatch",
                "Only %d%% of the clip bones exist in the current skeleton (%s).\n\n"
                "This package targets a different character rig — load that character "
                "first for a correct pose." % (round(cov * 100), name))

    def _default_sel(self):
        for want in ("Knee_L", "Elbow_L", "Head"):
            i = self.model._index.get(want)
            if i is not None:
                return i
        return 0

    # -------------------------------------------------------------- lists
    def _refill_clips(self):
        if not self.model:
            return
        flt = self.filter_var.get().lower().strip()
        self.clip_list.delete(0, "end")
        self._clip_map = []      # listbox row -> clip index
        for i, cl in enumerate(self.model.clips):
            if flt and flt not in cl.name.lower():
                continue
            self.clip_list.insert("end", cl.name)
            self._clip_map.append(i)
        # keep current selection visible if still listed
        if self.clip_idx in self._clip_map:
            row = self._clip_map.index(self.clip_idx)
            self.clip_list.selection_clear(0, "end")
            self.clip_list.selection_set(row)
            self.clip_list.see(row)

    def _select_clip_row(self, clip_idx):
        self._refill_clips()

    def _fill_bones(self):
        self.bone_list.delete(0, "end")
        for i, j in enumerate(self.model.joints):
            slots = self.model.clips[self.clip_idx].channels.get(j.name)
            tag = ""
            if slots:
                tag = " [" + "".join(s[0] for s in ("rot", "loc", "scl") if s in slots).upper() + "]"
            if i in self._helper:
                tag += " ·helper"
            self.bone_list.insert("end", j.name + tag)
            if i in self._helper:
                self.bone_list.itemconfig(i, foreground="#7b8794")
        if 0 <= self.sel < self.bone_list.size():
            self.bone_list.selection_clear(0, "end")
            self.bone_list.selection_set(self.sel)
            self.bone_list.see(self.sel)

    def _toggle_helpers(self):
        # if the current selection is now hidden, keep it (chain lines just won't draw)
        if self.model:
            self._fill_bones()
        self.draw()

    def _on_clip(self, _e):
        sel = self.clip_list.curselection()
        if not sel:
            return
        self.clip_idx = self._clip_map[sel[0]]
        self.frame = 0.0
        self._fill_bones()
        self._sync_transport()
        self.draw()

    def _on_bone(self, _e):
        sel = self.bone_list.curselection()
        if sel:
            self.sel = sel[0]
            self.draw()

    # -------------------------------------------------------------- transport
    def _sync_transport(self):
        mx = self.model.clips[self.clip_idx].max_frame if self.model else 1
        self.frame_scale.config(to=mx)
        self.frame_scale.set(int(self.frame))
        self.frame_lbl.config(text="%d / %d" % (int(self.frame), mx))

    def _on_scrub(self, val):
        if self.playing:
            return
        self.frame = float(val)
        self.frame_lbl.config(text="%d / %d" % (int(self.frame), self.model.clips[self.clip_idx].max_frame if self.model else 0))
        self.draw()

    def _on_fps(self):
        try:
            self.fps = max(1, int(self.fps_var.get()))
        except (tk.TclError, ValueError):
            self.fps = 30

    def toggle_play(self):
        if not self.model:
            return
        self.playing = not self.playing
        self.play_btn.config(text="⏸ Pause" if self.playing else "▶ Play")
        if self.playing:
            self._tick()
        elif self._after:
            self.after_cancel(self._after)
            self._after = None

    def _tick(self):
        if not self.playing or not self.model:
            return
        mx = self.model.clips[self.clip_idx].max_frame
        self.frame += 1.0
        if self.frame > mx:
            self.frame = 0.0
        self.frame_scale.set(int(self.frame))
        self.frame_lbl.config(text="%d / %d" % (int(self.frame), mx))
        self.draw()
        self._after = self.after(int(1000 / max(1, self.fps)), self._tick)

    def step(self, d):
        if not self.model:
            return
        mx = self.model.clips[self.clip_idx].max_frame
        self.frame = (int(self.frame) + d) % (mx + 1)
        self.frame_scale.set(int(self.frame))
        self.frame_lbl.config(text="%d / %d" % (int(self.frame), mx))
        self.draw()

    # -------------------------------------------------------------- camera
    def _press(self, e):
        self._drag = (e.x, e.y)
        # click-to-pick nearest joint
        if self._proj:
            best, bd = -1, 1e9
            for i, pp in enumerate(self._proj):
                if pp is None:
                    continue
                d = (pp[0] - e.x) ** 2 + (pp[1] - e.y) ** 2
                if d < bd:
                    bd, best = d, i
            if best >= 0 and bd < 240:
                self.sel = best
                if best < self.bone_list.size():
                    self.bone_list.selection_clear(0, "end")
                    self.bone_list.selection_set(best)
                    self.bone_list.see(best)
                self.draw()

    def _motion(self, e):
        if not self._drag:
            return
        dx, dy = e.x - self._drag[0], e.y - self._drag[1]
        self._drag = (e.x, e.y)
        self.yaw += dx * 0.01
        self.pitch = max(-1.4, min(1.4, self.pitch + dy * 0.01))
        self.draw()

    def _release(self, _e):
        self._drag = None

    def _wheel(self, e):
        self._zoom(0.92 if e.delta > 0 else 1.08)

    def _zoom(self, f):
        self.dist = max(1.6, min(7.0, self.dist * f))
        self.draw()

    def _project(self, p, W, H):
        cx, cy, cz = self.model.center
        x, y, z = p[0] - cx, p[1] - cy, p[2] - cz
        cyaw, syaw = math.cos(self.yaw), math.sin(self.yaw)
        x1, z1 = x * cyaw + z * syaw, -x * syaw + z * cyaw
        cpit, spit = math.cos(self.pitch), math.sin(self.pitch)
        y1, z2 = y * cpit - z1 * spit, y * spit + z1 * cpit
        persp = self.dist / (self.dist - z2) if (self.dist - z2) else 1.0
        sc = min(W, H) / self.model.span * 0.42
        return (W / 2 + x1 * persp * sc, H / 2 - y1 * persp * sc, z2)

    def _chain(self, i):
        c = set()
        g = 0
        while 0 < i < len(self.model.joints) and g < 300:
            g += 1
            c.add(i)
            p = self.model.joints[i].parent
            if p == i or p < 0:
                break
            c.add(p)
            i = p
        c.add(0)
        return c

    # -------------------------------------------------------------- draw
    def draw(self):
        cv = self.canvas
        cv.delete("all")
        if not self.model or not self.model.joints:
            cv.create_text(cv.winfo_width() / 2, cv.winfo_height() / 2, fill=JOINT,
                           text="Open a .p3d file (File ▸ Open)", font=("Segoe UI", 13))
            return
        W = max(1, cv.winfo_width())
        H = max(1, cv.winfo_height())

        # ground grid at feet
        gy = self.model.ymin
        cx, _cyv, cz = self.model.center
        for gx in [i * 0.2 for i in range(-3, 4)]:
            a = self._project((cx + gx, gy, cz - 0.6), W, H)
            b = self._project((cx + gx, gy, cz + 0.6), W, H)
            cv.create_line(a[0], a[1], b[0], b[1], fill=GRID)
        for gz in [i * 0.2 for i in range(-3, 4)]:
            a = self._project((cx - 0.6, gy, cz + gz), W, H)
            b = self._project((cx + 0.6, gy, cz + gz), W, H)
            cv.create_line(a[0], a[1], b[0], b[1], fill=GRID)

        pos = self.model.pose_world(self.clip_idx, self.frame)
        # Sanity guard: a few clips store a garbage/sentinel keyframe on non-deforming
        # helper bones (e.g. R_Wrist_Grapple can decode to ~3e11 on a trailing junk key).
        # Such joints would draw a line to near-infinity; hide them instead of exploding.
        # Also optionally hide the helper/attachment bones themselves (checkbox).
        LIM = 1.0e4
        hide = self.hide_helpers.get()
        valid = [all(math.isfinite(v) and abs(v) < LIM for v in p) and not (hide and i in self._helper)
                 for i, p in enumerate(pos)]
        self._proj = [self._project(p, W, H) if valid[i] else None for i, p in enumerate(pos)]
        chain = self._chain(self.sel)

        # bones
        for (a, b) in self.model.edges():
            if self._proj[a] is None or self._proj[b] is None:
                continue
            pa, pb = self._proj[a], self._proj[b]
            hot = a in chain and b in chain
            cv.create_line(pa[0], pa[1], pb[0], pb[1],
                           fill=ACCENT if hot else BONE, width=2 if hot else 1)
        # joints
        for i, pp in enumerate(self._proj):
            if pp is None:
                continue
            x, y, _z = pp
            if i == self.sel:
                r, col = 5, SEL
            elif i in chain:
                r, col = 3, ACCENT
            else:
                r, col = 2, JOINT
            cv.create_oval(x - r, y - r, x + r, y + r, fill=col, outline="")

        self._draw_gizmo(W, H)
        self._draw_hud(W, H)

    def _draw_gizmo(self, W, H):
        cv = self.canvas
        ox, oy, L = W - 56, H - 56, 26
        for v, lab, col in (((0.5, 0, 0), "X", AX[0]), ((0, 0.5, 0), "Y", AX[1]), ((0, 0, 0.5), "Z", AX[2])):
            cyaw, syaw = math.cos(self.yaw), math.sin(self.yaw)
            x1, z1 = v[0] * cyaw + v[2] * syaw, -v[0] * syaw + v[2] * cyaw
            cpit, spit = math.cos(self.pitch), math.sin(self.pitch)
            y1 = v[1] * cpit - z1 * spit
            ex, ey = ox + x1 * L, oy - y1 * L
            cv.create_line(ox, oy, ex, ey, fill=col, width=2)
            cv.create_text(ox + x1 * (L + 8), oy - y1 * (L + 8), text=lab, fill=col, font=("Segoe UI", 8, "bold"))

    def _draw_hud(self, W, H):
        if not (0 <= self.sel < len(self.model.joints)):
            return
        j = self.model.joints[self.sel]
        slots = self.model.channels_for(self.clip_idx, self.sel)
        tag = "+".join(sorted(slots)).upper() if slots else "rest"
        self.canvas.create_text(10, 14, anchor="w", fill=TEXT, font=("Consolas", 10),
                                 text="%s   [%s]" % (j.name, tag))

    def _about(self):
        messagebox.showinfo(
            "About",
            "Pure3D Animation Viewer (desktop)\n\n"
            "Opens Prototype 2 .p3d files and plays their skeletal animations — the same "
            "decode as the web anim_viewer. Rotation + translation + scale channels, "
            "quaternion SLERP, column-vector FK.\n\n"
            "Drag to orbit · wheel to zoom · Space play/pause · ←/→ step.")


def main():
    initial = sys.argv[1] if len(sys.argv) > 1 else None
    Viewer(initial).mainloop()


if __name__ == "__main__":
    main()
