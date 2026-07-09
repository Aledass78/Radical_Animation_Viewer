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
import p3d_write as pwrite
import p3d_bvh as pbvh


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


class _BvhImportDialog(tk.Toplevel):
    """Modal: choose what to do with an imported BVH — view it, add it as a new clip, or
    replace an existing clip in the loaded .p3d."""

    def __init__(self, parent, has_target, clip_names, default_name, target_file):
        super().__init__(parent)
        self.title("Import BVH")
        self.configure(bg=BG)
        self.resizable(False, False)
        self.transient(parent)
        self.result = None
        self.mode = tk.StringVar(value="view")
        self.name_var = tk.StringVar(value=default_name)
        self.target_var = tk.StringVar(value=(clip_names[0] if clip_names else ""))
        self.rx_var = tk.StringVar(value="0")
        self.ry_var = tk.StringVar(value="0")
        self.rz_var = tk.StringVar(value="0")

        frm = ttk.Frame(self, padding=12)
        frm.pack(fill="both", expand=True)
        ttk.Label(frm, text="What should we do with this BVH?").pack(anchor="w", pady=(0, 8))

        ttk.Radiobutton(frm, text="View it in the viewer", variable=self.mode, value="view",
                        command=self._sync).pack(anchor="w")

        addf = ttk.Frame(frm)
        addf.pack(fill="x", anchor="w", pady=(6, 0))
        self.rb_add = ttk.Radiobutton(addf, text="Add as NEW clip to", variable=self.mode,
                                      value="add", command=self._sync)
        self.rb_add.pack(side="left")
        ttk.Label(addf, text=(target_file or "—"), foreground=ACCENT).pack(side="left", padx=(4, 0))
        namef = ttk.Frame(frm)
        namef.pack(fill="x", anchor="w", padx=(24, 0))
        ttk.Label(namef, text="name:").pack(side="left")
        self.name_entry = ttk.Entry(namef, textvariable=self.name_var, width=24)
        self.name_entry.pack(side="left", padx=(4, 0))

        repf = ttk.Frame(frm)
        repf.pack(fill="x", anchor="w", pady=(6, 0))
        self.rb_rep = ttk.Radiobutton(repf, text="REPLACE existing clip", variable=self.mode,
                                      value="replace", command=self._sync)
        self.rb_rep.pack(side="left")
        repline = ttk.Frame(frm)
        repline.pack(fill="x", anchor="w", padx=(24, 0))
        ttk.Label(repline, text="clip:").pack(side="left")
        self.combo = ttk.Combobox(repline, textvariable=self.target_var, values=clip_names,
                                  width=30, state="readonly")
        self.combo.pack(side="left", padx=(4, 0))

        if not has_target:
            self.rb_add.config(state="disabled")
            self.rb_rep.config(state="disabled")
            ttk.Label(frm, text="(open a character .p3d first to add/replace clips)",
                      foreground=TEXT).pack(anchor="w", pady=(8, 0))

        ttk.Separator(frm, orient="horizontal").pack(fill="x", pady=(10, 6))
        rotf = ttk.Frame(frm)
        rotf.pack(fill="x", anchor="w")
        ttk.Label(rotf, text="Coordinate fix — rotate axes (deg):").pack(side="left")
        for lab, var in (("X", self.rx_var), ("Y", self.ry_var), ("Z", self.rz_var)):
            ttk.Label(rotf, text=lab).pack(side="left", padx=(8, 1))
            ttk.Entry(rotf, textvariable=var, width=5).pack(side="left")
        ttk.Label(frm, text="0 = as-is (game clips need none). A Z-up (Blender) source needs X = -90 "
                            "(fixes orientation AND the per-bone twist).",
                  foreground=TEXT).pack(anchor="w", pady=(3, 0))

        btns = ttk.Frame(frm)
        btns.pack(fill="x", pady=(12, 0))
        ttk.Button(btns, text="OK", command=self._ok).pack(side="right")
        ttk.Button(btns, text="Cancel", command=self.destroy).pack(side="right", padx=(0, 6))

        self._sync()
        self.grab_set()
        self.update_idletasks()

    def _sync(self):
        m = self.mode.get()
        self.name_entry.config(state="normal" if m == "add" else "disabled")
        self.combo.config(state="readonly" if m == "replace" else "disabled")

    def _ok(self):
        m = self.mode.get()
        if m == "add" and not self.name_var.get().strip():
            messagebox.showwarning("Import BVH", "Enter a clip name.", parent=self)
            return
        if m == "replace" and not self.target_var.get():
            messagebox.showwarning("Import BVH", "Choose a clip to replace.", parent=self)
            return
        def f(v):
            try:
                return float(v.get())
            except ValueError:
                return 0.0
        self.result = {"action": m, "name": self.name_var.get().strip(),
                       "target": self.target_var.get(),
                       "rot": (f(self.rx_var), f(self.ry_var), f(self.rz_var))}
        self.destroy()


class Viewer(tk.Tk):
    def __init__(self, initial=None):
        super().__init__()
        self.title("Pure3D Animation Viewer")
        self.geometry("1200x760")
        self.minsize(880, 560)
        self.configure(bg=BG)

        self.model = None
        self.doc = None               # in-memory editable .p3d (p3d_write.Document)
        self.file_path = None         # where Save writes (may not exist on disk yet)
        self.dirty = False            # unsaved changes?
        self.undo_stack = []          # list of Document child-snapshots
        self.redo_stack = []
        self._saved_children = []     # doc.children as of the last save/load (for the dirty flag)
        self._last_joints = None      # last real skeleton (for animation-only packages)
        self._last_name = None
        self.be = False               # endianness of the loaded .p3d
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
        fm.add_command(label="Save", command=self.save, accelerator="Ctrl+S")
        fm.add_command(label="Save As…", command=self.save_as, accelerator="Ctrl+Shift+S")
        fm.add_separator()
        fm.add_command(label="Undo", command=self.undo, accelerator="Ctrl+Z")
        fm.add_command(label="Redo", command=self.redo, accelerator="Ctrl+Y")
        fm.add_separator()
        ex = tk.Menu(fm, tearoff=0)
        ex.add_command(label="Current clip → BVH (Blender / mocap)…", command=self.export_bvh)
        ex.add_command(label="Current clip → JSON (decoded curves)…", command=self.export_json)
        ex.add_command(label="ALL clips → BVH folder…", command=self.export_all_bvh)
        fm.add_cascade(label="Export", menu=ex)
        im = tk.Menu(fm, tearoff=0)
        im.add_command(label="Import BVH… (view / add / replace)", command=self.import_bvh)
        im.add_command(label="JSON clip → inject into loaded .p3d…", command=self.import_json_clip)
        im.add_command(label="Delete selected clip…", command=self.delete_selected_clip)
        im.add_command(label="Re-save loaded .p3d (clips inline)…", command=self.resave_inline)
        fm.add_cascade(label="Import / Write", menu=im)
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
        ttk.Button(top, text="📂 Open", command=self.open_dialog).pack(side="left")
        ttk.Button(top, text="💾 Save", command=self.save).pack(side="left", padx=(6, 0))
        ttk.Button(top, text="Save As", command=self.save_as).pack(side="left", padx=(6, 0))
        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=6)
        ttk.Button(top, text="↶ Undo", command=self.undo).pack(side="left")
        ttk.Button(top, text="↷ Redo", command=self.redo).pack(side="left", padx=(6, 0))
        ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=6)
        ttk.Button(top, text="📤 Export BVH", command=self.export_bvh).pack(side="left")
        ttk.Button(top, text="📥 Import BVH", command=self.import_bvh).pack(side="left", padx=(6, 0))
        ttk.Button(top, text="🗑 Delete clip", command=self.delete_selected_clip).pack(side="left", padx=(6, 0))
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
        self.bind("<Control-s>", lambda e: self.save())
        self.bind("<Control-S>", lambda e: self.save_as())          # Ctrl+Shift+S
        self.bind("<Control-z>", lambda e: self.undo())
        self.bind("<Control-y>", lambda e: self.redo())
        self.bind("<Control-Z>", lambda e: self.redo())             # Ctrl+Shift+Z
        self.bind("<space>", lambda e: self.toggle_play())
        self.bind("<Left>", lambda e: self.step(-1))
        self.bind("<Right>", lambda e: self.step(1))
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        if self.dirty:
            ans = messagebox.askyesnocancel("Unsaved changes",
                                            "You have unsaved changes. Save before closing?")
            if ans is None:            # Cancel
                return
            if ans:                    # Yes -> save; abort close if the save was cancelled
                self.save()
                if self.dirty:
                    return
        self.destroy()

    # -------------------------------------------------------------- loading
    def _find_default(self):
        here = os.path.dirname(os.path.abspath(__file__))
        for up in ("..", "."):
            cand = os.path.normpath(os.path.join(here, up, "Pc_Version", "art", "alex", "alex.p3d"))
            if os.path.isfile(cand):
                return cand
        return None

    def open_dialog(self):
        if self.dirty and not messagebox.askyesno(
                "Unsaved changes", "Opening another file discards unsaved changes. Continue?"):
            return
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

    # ---------------------------------------------------------- import / write
    def import_json_clip(self):
        if not self.doc:
            messagebox.showinfo("Import", "Open a character .p3d first — the clip is added to it.")
            return
        jpath = filedialog.askopenfilename(title="Choose a JSON clip (exported by this viewer)",
                                           filetypes=[("JSON", "*.json"), ("All files", "*.*")])
        if not jpath:
            return
        try:
            clip = pwrite.clip_from_json(jpath, be=self.be)
        except Exception as e:
            messagebox.showerror("Import failed", str(e))
            return
        name = self.doc._name_of(clip) or "(clip)"
        if self._commit(lambda: self.doc.add_clip(clip), select_name=name):
            self.status.config(text="Added clip '%s' (unsaved — use Save)." % name)

    def delete_selected_clip(self):
        """Delete the currently selected clip from the in-memory document (undoable; not saved yet)."""
        if not self.doc or not self.model or not self.model.clips:
            messagebox.showinfo("Delete clip", "Open a .p3d and select a clip first.")
            return
        name = self.model.clips[self.clip_idx].name
        if not messagebox.askyesno("Delete clip", "Delete clip '%s'?\n\n(You can Undo, and nothing is "
                                   "written until you Save.)" % name):
            return
        if self._commit(lambda: self.doc.delete_clip(name)):
            self.status.config(text="Deleted '%s' (unsaved — use Save; Undo to restore)." % name)
        else:
            messagebox.showwarning("Delete clip", "Clip '%s' not found; nothing deleted." % name)

    def resave_inline(self):
        """Convert every animation clip in the document to inline channels (in memory)."""
        if not self.doc:
            messagebox.showinfo("Write", "Open a .p3d first.")
            return
        if self._commit(lambda: self.doc.reencode_all_inline()):
            self.status.config(text="Converted all clips to inline channels (unsaved — use Save).")

    # ---------------------------------------------------------- import BVH
    def import_bvh(self):
        path = filedialog.askopenfilename(title="Import BVH animation",
                                          filetypes=[("Biovision Hierarchy", "*.bvh"), ("All files", "*.*")])
        if not path:
            return
        try:
            bvh = pbvh.read_bvh(path)
        except Exception as e:
            messagebox.showerror("Import BVH failed", "Could not read %s:\n%s" % (os.path.basename(path), e))
            return
        if not bvh["joints"] or not bvh["frames"]:
            messagebox.showwarning("Import BVH", "No hierarchy/motion found in the BVH.")
            return

        has_target = bool(self.doc)
        clips = [c.name for c in self.model.clips] if self.model else []
        default_name = self._safe_name(os.path.splitext(os.path.basename(path))[0])
        dlg = _BvhImportDialog(self, has_target, clips, default_name,
                               self._src_name() if has_target else None)
        self.wait_window(dlg)
        if not dlg.result:
            return
        action = dlg.result["action"]
        rot = dlg.result.get("rot", (0.0, 0.0, 0.0))

        if action == "view":
            try:
                model, fps = pbvh.bvh_to_model(bvh, source=os.path.basename(path),
                                               clip_name=default_name, rot=rot)
            except Exception as e:
                messagebox.showerror("Import BVH failed", str(e))
                return
            self.fps = fps
            self.fps_var.set(fps)
            self._show_bvh_model(model, os.path.basename(path), fps)
            return

        # action in ('add', 'replace') -> write into the loaded .p3d
        # a replacement keeps the TARGET clip's name so it occupies the same animation slot
        clip_name = dlg.result["target"] if action == "replace" else dlg.result.get("name", default_name)
        try:
            channels, nframes, fps = pbvh.bvh_to_channels(bvh, rot=(0.0, 0.0, 0.0))
            # (1) Coordinate fix FIRST — re-express the whole animation in the game frame (a Z-up
            #     Blender source needs X = -90). Must precede the yaw constraint so 'vertical' is
            #     the game's up-axis.
            if any(rot):
                pbvh.apply_rotation(channels, self.model.joints, rot[0], rot[1], rot[2])
            # (2) Game-faithful policy — keep body rotation exactly as authored (flips on
            #     Pelvis/Spine survive), keep Motion_Root FACING but constrain it to a grounded
            #     vertical yaw, and drop the reserved Balance_Root. This is what makes an imported
            #     clip behave in-game like a shipped one, independent of any template.
            channels = pbvh.game_faithful_filter(channels)
            clip = pwrite.build_clip(clip_name, self.model.joints, channels, nframes,
                                     fps=fps, be=self.be)
        except Exception as e:
            messagebox.showerror("Import BVH failed", str(e))
            return
        # apply to the in-memory document (undoable; written only on Save)
        if action == "add":
            ok = self._commit(lambda: self.doc.add_clip(clip), select_name=clip_name)
            note = "Added new clip '%s' (game-faithful: body motion kept, root constrained to a grounded yaw)." % clip_name
        else:
            ok = self._commit(lambda: self.doc.replace_clip(clip_name, clip), select_name=clip_name)
            note = "Replaced clip '%s' (game-faithful root handling)." % clip_name
        if ok:
            self.status.config(text=note + " Unsaved — use Save / Save As.")

    def _show_bvh_model(self, model, label, fps):
        """Install a Model built from a BVH for a transient preview (the loaded document, if any,
        is untouched — Save still writes the .p3d, not this preview)."""
        self.model = model
        self._helper = {i for i, j in enumerate(model.joints) if is_helper_bone(j.name)}
        self.clip_idx = 0
        self.frame = 0.0
        self.playing = False
        self.play_btn.config(text="▶ Play")
        self.sel = self._default_sel()
        self._refill_clips()
        self._fill_bones()
        self._select_clip_row(0)
        self._sync_transport()
        self.draw()
        self.src_lbl.config(text="%s · BVH · %d clips · %d joints · %d fps"
                            % (label, len(model.clips), len(model.joints), fps))
        self.status.config(text="Loaded BVH: " + label)

    def load(self, path):
        """Open a .p3d fully into memory (an editable Document). No further disk dependency."""
        self.status.config(text="Loading " + os.path.basename(path) + " …")
        self.update_idletasks()
        try:
            with open(path, "rb") as f:
                raw = f.read()
            doc = pwrite.Document(raw)
        except Exception as e:
            messagebox.showerror("Open failed", "Could not parse %s:\n%s" % (os.path.basename(path), e))
            self.status.config(text="Load failed.")
            return
        self.doc = doc
        self.file_path = path
        self.dirty = False
        self.undo_stack = []
        self.redo_stack = []
        self._saved_children = list(doc.children)
        if self._rebuild_from_doc(source_label=os.path.basename(path)):
            self.status.config(text="Loaded %d clip(s) into memory." % len(self.model.clips))

    def _rebuild_from_doc(self, source_label=None, select_name=None):
        """(Re)build the on-screen model from the in-memory document and refresh the UI.
        Called on load and after every edit / undo / redo. Returns True on success."""
        self.dirty = bool(self.doc) and (self.doc.children != self._saved_children)
        try:
            name, joints, clips, be = core.load_bytes(self.doc.to_bytes())
        except Exception as e:
            messagebox.showerror("Error", "Could not parse the in-memory document:\n%s" % e)
            return False
        if not clips:
            messagebox.showwarning("No animations", "No animation clips in the file.")
            return False
        reused = False
        cov = 1.0
        if joints is None:                       # animation-only package -> reuse a real skeleton
            if not self._last_joints:
                messagebox.showinfo("No skeleton",
                                    "This file has only animations (no skeleton). Open a character "
                                    ".p3d first (e.g. alex.p3d), then open this package.")
                return False
            joints, name, reused = self._last_joints, self._last_name, True
        else:
            self._last_joints, self._last_name = joints, name

        prev = self.model.clips[self.clip_idx].name \
            if (self.model and self.model.clips and 0 <= self.clip_idx < len(self.model.clips)) else None
        self.be = be
        self.model = core.Model(source_label or self._src_name(), joints, name, clips)
        if reused:
            cov = self.model.bone_coverage()
        self._reused = reused
        self._cov = cov
        self._helper = {i for i, j in enumerate(self.model.joints) if is_helper_bone(j.name)}
        # pick a clip to show: the requested one, else keep the same clip if it still exists, else 0
        want = select_name or prev
        self.clip_idx = next((i for i, c in enumerate(clips) if c.name == want), 0)
        self.frame = 0.0
        self.playing = False
        self.play_btn.config(text="▶ Play")
        self.sel = self._default_sel()
        self.endian = "PS3 (big-endian)" if be else "PC (little-endian)"
        self._refill_clips()
        self._fill_bones()
        self._sync_transport()
        self.draw()
        self._update_src_label()
        if reused and cov < 0.5:
            messagebox.showwarning("Bone mismatch",
                                   "Only %d%% of the clip bones exist in the current skeleton (%s)."
                                   % (round(cov * 100), name))
        return True

    def _src_name(self):
        return os.path.basename(self.file_path) if self.file_path else "(in memory)"

    def _update_src_label(self):
        if not self.model:
            return
        dot = "●  " if self.dirty else ""
        extra = "  (reusing %s, %d%% bones)" % (self._last_name, round(getattr(self, "_cov", 1) * 100)) \
            if getattr(self, "_reused", False) else ""
        self.src_lbl.config(text="%s%s · %s · %d clips · %d joints%s"
                            % (dot, self._src_name(), self.endian,
                               len(self.model.clips), len(self.model.joints), extra))

    # ---------------------------------------------------------- edit / undo / redo / save
    def _commit(self, fn, select_name=None):
        """Run fn() (which mutates self.doc and returns True on success), recording it for undo
        and rebuilding the view. Nothing is written to disk — that's Save's job."""
        if not self.doc:
            messagebox.showinfo("Edit", "Open a .p3d first.")
            return False
        snap = self.doc.snapshot()
        try:
            ok = fn()
        except Exception as e:
            self.doc.restore(snap)
            messagebox.showerror("Edit failed", str(e))
            return False
        if not ok:
            return False
        self.undo_stack.append(snap)
        self.redo_stack.clear()
        self._rebuild_from_doc(select_name=select_name)
        return True

    def undo(self):
        if not self.doc or not self.undo_stack:
            self.status.config(text="Nothing to undo.")
            return
        self.redo_stack.append(self.doc.snapshot())
        self.doc.restore(self.undo_stack.pop())
        self._rebuild_from_doc()
        self.status.config(text="Undid change.")

    def redo(self):
        if not self.doc or not self.redo_stack:
            self.status.config(text="Nothing to redo.")
            return
        self.undo_stack.append(self.doc.snapshot())
        self.doc.restore(self.redo_stack.pop())
        self._rebuild_from_doc()
        self.status.config(text="Redid change.")

    def save(self):
        if not self.doc:
            messagebox.showinfo("Save", "Open a .p3d first.")
            return
        if not self.file_path:
            return self.save_as()
        try:
            with open(self.file_path, "wb") as f:
                f.write(self.doc.to_bytes())
        except Exception as e:
            messagebox.showerror("Save failed", str(e))
            return
        self._saved_children = list(self.doc.children)
        self.dirty = False
        self._update_src_label()
        self.status.config(text="Saved → " + os.path.basename(self.file_path))

    def save_as(self):
        if not self.doc:
            messagebox.showinfo("Save As", "Open a .p3d first.")
            return
        path = filedialog.asksaveasfilename(title="Save .p3d as", defaultextension=".p3d",
                                            initialfile=self._src_name(),
                                            filetypes=[("Pure3D", "*.p3d")])
        if not path:
            return
        try:
            with open(path, "wb") as f:
                f.write(self.doc.to_bytes())
        except Exception as e:
            messagebox.showerror("Save failed", str(e))
            return
        self.file_path = path
        self._saved_children = list(self.doc.children)
        self.dirty = False
        self._update_src_label()
        self.status.config(text="Saved → " + os.path.basename(path))

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
