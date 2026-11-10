"""dashboard.py - Low-Latency MuJoCo 3D Simulator Bridge & MoS Telemetry Console."""
import math
import os
import shlex
import time
import threading
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node

from . import cad as cadmod
from . import hand as HD
from . import nptel_priority as NP
from .hand_plant import Episode, run_headless
from .arm_ik import ARM, ik_wrist, tcp_home
from .materials import GRIPPER, OBJECTS, PADS, G, height
from .world import TABLE_TOP_Z, build_mjcf

def fmt_modulus(val): return "probing..." if val is None or math.isnan(val) or val <= 0 else (f"{val/1e9:.2f} GPa" if val >= 1e9 else f"{val/1e6:.2f} MPa")
def fmt_fos(f):
    """Safety factor as it really is (no cap).  inf -> no load / no limit."""
    if f is None or (isinstance(f, float) and math.isnan(f)): return "-"
    if math.isinf(f): return "no limit"
    return f"{f:.2f}" if f < 100 else (f"{f:.0f}" if f < 1000 else f"{f:.2e}")
def fmt_stress(val): return "inf" if val is None or math.isinf(val) else (f"{val/1e6:.3f} MPa" if abs(val) >= 1e6 else f"{val/1e3:.2f} kPa")

class MuJoCoSimBridgeNode(Node):
    def __init__(self):
        super().__init__('grip_sim_dashboard_node')
        self.declare_parameter('object', 'apple')
        self.declare_parameter('pad', 'small')
        self.declare_parameter('seed', 0)
        self.declare_parameter('cad', '')
        self.declare_parameter('cad_material', 'aluminium')
        self.declare_parameter('cad_unit', 'mm')
        self.declare_parameter('cad_args', '')
        self.declare_parameter('fingers', 5)

        self.init_obj = self.get_parameter('object').value
        self.init_cad_spec = self.init_cad_rep = self.init_cad_error = None
        self.cad_dirty = False
        cad_file = str(self.get_parameter('cad').value or '').strip()
        if cad_file:
            try:
                argv = [cad_file, '--material', str(self.get_parameter('cad_material').value or 'aluminium'),
                        '--unit', str(self.get_parameter('cad_unit').value or 'mm')] + shlex.split(str(self.get_parameter('cad_args').value or ''))
                spec = cadmod.spec_from_args(argv)
                ckey, crep = self.load_cad(spec)
                self.init_obj, self.init_cad_spec, self.init_cad_rep = ckey, spec, crep
                self.get_logger().info('CAD part loaded:\n' + cadmod.format_report(crep))
            except BaseException as e:
                self.init_cad_error = str(e) or repr(e)
                self.get_logger().error('CAD load failed: %s' % self.init_cad_error)

        self.init_pad = self.get_parameter('pad').value
        self.seed = int(self.get_parameter('seed').value)
        HD.HAND['n_fingers'] = max(3, min(5, int(self.get_parameter('fingers').value)))

        self.running = self.paused = self.stop_req = False
        self.z0 = TABLE_TOP_Z
        self._mj_lock = threading.Lock()
        self.mj_model = self.mj_data = self.viewer = self.n_spawned = self.active_obj_key = None
        self.known_objects = set()
        self.qadr_arm = ()
        self.qadr_fingers = []
        self.mocap_intact, self.mocap_crush = {}, {}
        self.ensure_scene(HD.HAND['n_fingers'], self.init_obj)

    def load_cad(self, spec):
        """Analyse a CAD file with the given spec, register it as a specimen. Returns (key, report). Raises CadError."""
        from .physics_check import best_pose
        key, obj, rep = best_pose(spec, int(HD.HAND['n_fingers']))
        obj['mesh_ok'] = self._mesh_compiles(obj.get('mesh'))
        if not obj['mesh_ok']:                                    # should not happen (face limit is enforced in cad.py) - degrade gracefully: coarser mesh, then its convex hull
            try:
                from . import meshprep as MP
                V, F = cadmod.load_mesh(obj['mesh'])
                V, F = MP._compact(*MP.weld(V, F)) if False else MP.weld(V, F)
                for tgt in (30000, 8000, 2000):
                    V2, F2 = MP.decimate(V, F, tgt); F2, _ = MP.orient_consistent(F2)
                    cadmod._write_stl(obj['mesh'], V2, F2)
                    if self._mesh_compiles(obj['mesh']): obj['mesh_ok'] = True; rep['warnings'].append('Display mesh reduced to %d triangles so MuJoCo could load it.' % len(F2)); break
            except Exception: pass
        if not obj['mesh_ok']:
            rep['warnings'].append('MuJoCo could not load this mesh; a plain cylinder is shown instead (physics is unaffected).')
        cadmod.register(key, obj)
        self.cad_dirty = True              # forces the MuJoCo scene to be rebuilt with the new / changed mesh
        return key, rep

    @staticmethod
    def _mesh_compiles(path):
        try:
            mujoco.MjModel.from_xml_string('<mujoco><asset><mesh name="m" file="%s"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>' % path)
            return True
        except Exception:
            return False

    def ensure_scene(self, n=None, obj_key=None):
        n = int(n or HD.HAND['n_fingers'])
        with self._mj_lock:
            if self.mj_model is None or self.n_spawned != n or self.cad_dirty or (obj_key is not None and obj_key not in self.known_objects):
                HD.HAND['n_fingers'] = n
                self.cad_dirty = False
                self.mj_model = mujoco.MjModel.from_xml_string(build_mjcf(n_fingers=n, z0=self.z0))
                self.mj_data = mujoco.MjData(self.mj_model)
                self.n_spawned, self.known_objects = n, set(OBJECTS.keys())
                self.qadr_arm = tuple(self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, jn)] for jn in ('shoulder', 'elbow', 'wrist'))
                self.qadr_yaw = self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, 'yaw')]
                self.qadr_spread = [self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, f'f{i}_spread')] for i in range(n)]
                self.qadr_fingers = [tuple(self.mj_model.jnt_qposadr[mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, f'f{i}_{jt}')] for jt in ('mcp', 'pip', 'dip')) for i in range(n)]
                for k in OBJECTS:
                    bid_i, bid_c = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, f'obj_{k}'), mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, f'crush_{k}')
                    if bid_i >= 0: self.mocap_intact[k] = self.mj_model.body_mocapid[bid_i]
                    if bid_c >= 0: self.mocap_crush[k] = self.mj_model.body_mocapid[bid_c]
                if self.viewer is not None:
                    try: self.viewer.close()
                    except Exception: pass
                try:
                    self.viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data, show_left_ui=False, show_right_ui=False)
                    self.viewer.cam.lookat[:], self.viewer.cam.distance, self.viewer.cam.elevation, self.viewer.cam.azimuth = [-0.08, 0.0, 0.18], 1.15, -18.0, 132.0
                except Exception: self.viewer = None
            if obj_key is not None:
                for k, mid in self.mocap_intact.items(): self.mj_data.mocap_pos[mid] = [0.0, 0.0, -5.0]
                for k, mid in self.mocap_crush.items(): self.mj_data.mocap_pos[mid] = [0.0, 0.0, -5.0]
                self.active_obj_key = obj_key
                if obj_key in OBJECTS and obj_key in self.mocap_intact:
                    self.mj_data.mocap_pos[self.mocap_intact[obj_key]] = [0.0, 0.0, height(OBJECTS[obj_key]) / 2.0 + self.z0]
            self._set_arm_and_fingers_locked(*tcp_home(), [0.0] * n, 0.0, [0.0] * n)
            mujoco.mj_forward(self.mj_model, self.mj_data)
            if self.viewer and self.viewer.is_running(): self.viewer.sync()

    def _set_arm_and_fingers_locked(self, tcp_x, tcp_z, s_list, yaw=0.0, sigmas=None):
        th1, th2, th3 = ik_wrist(tcp_x, tcp_z + ARM['tcp_drop'])
        self.mj_data.qpos[self.qadr_arm[0]], self.mj_data.qpos[self.qadr_arm[1]], self.mj_data.qpos[self.qadr_arm[2]] = th1, th2, th3
        self.mj_data.qpos[self.qadr_yaw] = yaw
        for i, sg in enumerate(sigmas or []): self.mj_data.qpos[self.qadr_spread[i]] = sg
        for i, s in enumerate(s_list):
            q1, q2, q3 = HD.joint_angles(s)
            self.mj_data.qpos[self.qadr_fingers[i][0]], self.mj_data.qpos[self.qadr_fingers[i][1]], self.mj_data.qpos[self.qadr_fingers[i][2]] = q1, q2, q3

    def sync_frame(self, tcp_x, tcp_z, s_list, o, obj_x, obj_rise_z, crushed, yaw=0.0, sigmas=None, ep=None):
        with self._mj_lock:
            if self.mj_model is None: return
            if not all(math.isfinite(v) for v in [tcp_x, tcp_z, yaw, *s_list]): return
            self._set_arm_and_fingers_locked(tcp_x, tcp_z, s_list, yaw, sigmas)
            k = self.active_obj_key
            if k in self.mocap_intact and k in self.mocap_crush:
                z_world, mid_i, mid_c = height(o) / 2.0 + obj_rise_z + self.z0, self.mocap_intact[k], self.mocap_crush[k]
                self.mj_data.mocap_pos[mid_i] = [0.0, 0.0, -5.0] if crushed else [obj_x, 0.0, z_world]
                self.mj_data.mocap_pos[mid_c] = [obj_x, 0.0, z_world] if crushed else [0.0, 0.0, -5.0]
            mujoco.mj_forward(self.mj_model, self.mj_data)
            self._draw_contacts(ep, obj_x, obj_rise_z, crushed)
            if self.viewer and self.viewer.is_running(): self.viewer.sync()

    def _draw_contacts(self, ep, obj_x, rise_z, crushed):
        """Red dot on the part where each loaded fingertip touches it + a force arrow along the contact normal (viewer overlay only)."""
        v = self.viewer
        if v is None or not hasattr(v, 'user_scn'): return
        scn = v.user_scn
        scn.ngeom = 0
        if ep is None or crushed or not ep.plan.get('ok'): return
        try:
            m, d, pl, R = self.mj_model, self.mj_data, ep.plant, HD.HAND['tip_r']
            L3, pb = HD.HAND['L'][2], HD.HAND['pad_back']
            org = np.array([obj_x, 0.0, self.z0 + rise_z])
            for i in range(ep.n):
                if not pl.active[i] or pl.N[i] <= 0.01: continue
                bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'grip_f{i}_p3')
                c = d.xpos[bid] + d.xmat[bid].reshape(3, 3) @ np.array([L3 - pb, 0.0, 0.0])
                P = (c - org)[None]
                sd = float(ep.shape.sdf(P)[0])
                nrm = ep.shape.normal(P)[0]
                p_surf = c - nrm * sd                                   # nearest point on the part surface
                if scn.ngeom + 2 > scn.maxgeom: break
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.0035, 0, 0]), p_surf, np.eye(3).ravel(), np.array([1.0, 0.1, 0.1, 1.0], dtype=np.float32))
                scn.geoms[scn.ngeom].emission = 0.9
                scn.ngeom += 1
                # arrow sits OUTSIDE the fingertip dome (dome reaches ~2*tip_r from the surface) and points at the contact point
                base = 2.0 * R + 0.004
                ln = min(0.004 + 0.0015 * float(pl.N[i]), 0.025)
                mujoco.mjv_connector(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, 0.0022, p_surf + nrm * (base + ln), p_surf + nrm * base)
                scn.geoms[scn.ngeom].rgba[:] = (1.0, 0.85, 0.1, 1.0)
                scn.geoms[scn.ngeom].emission = 0.9
                scn.ngeom += 1
        except Exception:
            scn.ngeom = 0

    def close_viewer(self):
        with self._mj_lock:
            if self.viewer is not None:
                try: self.viewer.close()
                except Exception: pass
                self.viewer = None

class SimDashboardGUI:
    BG_MAIN, BG_CARD, BG_HEADER, BORDER = "#0b0f17", "#111827", "#161f30", "#1f2937"
    TEXT_MAIN, TEXT_MUTED = "#f3f4f6", "#9ca3af"
    ACCENT_BLUE, ACCENT_GREEN, ACCENT_AMBER, ACCENT_RED, ACCENT_PURPLE = "#38bdf8", "#22c55e", "#f59e0b", "#ef4444", "#a855f7"
    FSM_STAGES = ["CALIB", "WAIT_ARM", "CLOSE", "SQUEEZE", "SETTLE", "LIFT", "HOLD"]

    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("grip_gz // MuJoCo Dexterous Adaptive Force & MoS Telemetry Console")
        self.root.geometry("1440x960")
        self.root.configure(bg=self.BG_MAIN)
        self._latest_ep = self._latest_strat = None
        self._dirty_ui = False
        self._episode_setup_needed = True
        self._last_plot_t = 0.0
        self._last_err = None

        style = ttk.Style(self.root)
        style.theme_use("clam")
        for k in ("TCombobox", "Stick.Horizontal.TProgressbar", "Torque.Horizontal.TProgressbar"):
            style.configure(k, fieldbackground="#1e293b", background="#334155" if k == "TCombobox" else (self.ACCENT_BLUE if "Stick" in k else self.ACCENT_PURPLE), foreground="#f8fafc", arrowcolor="#38bdf8", bordercolor="#334155" if k == "TCombobox" else "#1e293b", lightcolor="#1e293b" if k == "TCombobox" else (self.ACCENT_BLUE if "Stick" in k else self.ACCENT_PURPLE), darkcolor="#1e293b" if k == "TCombobox" else (self.ACCENT_BLUE if "Stick" in k else self.ACCENT_PURPLE))
        style.configure("TNotebook", background=self.BG_CARD, borderwidth=0)
        style.configure("TNotebook.Tab", background="#1e293b", foreground=self.TEXT_MUTED, padding=(12, 4), font=("Consolas", 9, "bold"))
        style.map("TNotebook.Tab", background=[("selected", "#0f172a")], foreground=[("selected", self.ACCENT_BLUE)])
        style.configure("Dark.Treeview", background="#0f172a", fieldbackground="#0f172a", foreground="#e2e8f0", rowheight=22, borderwidth=0, font=("Consolas", 9))
        style.configure("Dark.Treeview.Heading", background="#1e293b", foreground=self.ACCENT_BLUE, font=("Consolas", 9, "bold"), relief="flat")
        style.map("Dark.Treeview", background=[("selected", "#334155")])
        style.map("TCombobox", fieldbackground=[("readonly", "#1e293b")], selectbackground=[("readonly", "#1e293b")], selectforeground=[("readonly", "#38bdf8")])

        top = tk.Frame(self.root, bg=self.BG_HEADER, pady=10, padx=14, highlightthickness=1, highlightbackground=self.BORDER)
        top.pack(fill=tk.X)
        brand = tk.Frame(top, bg=self.BG_HEADER)
        brand.pack(side=tk.LEFT, padx=(0, 14))
        tk.Label(brand, text="GRIP_MUJOCO", bg=self.BG_HEADER, fg=self.ACCENT_BLUE, font=("Consolas", 13, "bold")).pack(side=tk.LEFT)
        tk.Label(brand, text=" // MoS TACTILE LAB", bg=self.BG_HEADER, fg=self.TEXT_MUTED, font=("Consolas", 10, "bold")).pack(side=tk.LEFT)

        def add_combo(label, var, values, width):
            tk.Label(top, text=label, bg=self.BG_HEADER, fg=self.TEXT_MUTED, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=(6, 3))
            cb = ttk.Combobox(top, textvariable=var, values=values, width=width, state="readonly", font=("Consolas", 9))
            cb.pack(side=tk.LEFT, padx=(0, 6))
            return cb

        self.obj_var, self.strat_var, self.pad_var, self.fing_var, self.speed_var = tk.StringVar(value=self.node.init_obj), tk.StringVar(value="Adaptive (blind)"), tk.StringVar(value="small (R=25mm)" if self.node.init_pad == "small" else "large (R=100mm)"), tk.StringVar(value=str(HD.HAND['n_fingers'])), tk.StringVar(value="1x (Real-time)")
        self.obj_cb = add_combo("OBJECT", self.obj_var, list(OBJECTS.keys()), 16)
        add_combo("MODE", self.strat_var, ["Adaptive (blind)", "Fixed 25 N"], 15)
        add_combo("PAD", self.pad_var, ["small (R=25mm)", "large (R=100mm)"], 14)
        add_combo("FINGERS", self.fing_var, ["3", "4", "5"], 3)
        add_combo("SPEED", self.speed_var, ["0.5x", "1x (Real-time)", "2x", "4x"], 13)
        self.speed_mult = 1.0
        self.speed_var.trace_add("write", self._speed_changed)

        btn_frame = tk.Frame(top, bg=self.BG_HEADER)
        btn_frame.pack(side=tk.RIGHT)
        self.btns = {}
        for t, c, cmd in [("▶ RUN EPISODE", "#16a34a", self.start_sim), ("⏸ PAUSE", "#d97706", self.toggle_pause), ("⏹ STOP", "#dc2626", self.stop_sim), ("▦ BENCHMARK ALL", "#2563eb", self.start_benchmark)]:
            b = tk.Button(btn_frame, text=t, bg=c, fg="white", command=cmd, font=("Consolas", 9, "bold"), bd=0, padx=9, pady=5, cursor="hand2", activebackground="#334155")
            b.pack(side=tk.LEFT, padx=3)
            self.btns[t] = b

        bar = tk.Frame(self.root, bg="#0f172a", pady=6, padx=14, highlightthickness=1, highlightbackground=self.BORDER)
        bar.pack(fill=tk.X)
        tk.Label(bar, text="CONTROLLER PIPELINE:", bg="#0f172a", fg=self.TEXT_MUTED, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, padx=(0, 10))
        self.stage_pills = {st: tk.Label(bar, text=f" {st} ", bg="#1e293b", fg="#64748b", font=("Consolas", 9, "bold"), padx=8, pady=2) for st in self.FSM_STAGES}
        for pill in self.stage_pills.values(): pill.pack(side=tk.LEFT, padx=3)
        self.arm_badge = tk.Label(bar, text="ARM: HOME | t = 0.00s", bg="#0f172a", fg=self.ACCENT_BLUE, font=("Consolas", 9, "bold"))
        self.arm_badge.pack(side=tk.RIGHT)

        self._build_cad_bar()

        main = tk.Frame(self.root, bg=self.BG_MAIN)
        main.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)
        left = tk.Frame(main, bg=self.BG_MAIN, width=560)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))
        left.pack_propagate(False)

        kpi_row = tk.Frame(left, bg=self.BG_MAIN)
        kpi_row.pack(fill=tk.X, pady=(0, 8))
        self.kpi_vars, self.kpi_boxes = {}, {}
        for i, (key, title, init_v, color) in enumerate([("fz", "NORMAL FORCE f_z", "0.00 N", self.ACCENT_BLUE), ("fos", "SAFETY FACTOR", "-", self.ACCENT_GREEN), ("vm", "VON MISES σ_vM", "0.0 kPa", "#f472b6"), ("lift", "OBJECT LIFT z", "0.0 mm", self.ACCENT_AMBER)]):
            card = tk.Frame(kpi_row, bg=self.BG_CARD, padx=8, pady=6, highlightthickness=1, highlightbackground=self.BORDER)
            card.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 4, 0))
            kpi_row.columnconfigure(i, weight=1)
            tk.Label(card, text=title, bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold")).pack(anchor="w")
            v = tk.StringVar(value=init_v)
            lbl = tk.Label(card, textvariable=v, bg=self.BG_CARD, fg=color, font=("Consolas", 13, "bold"))
            lbl.pack(anchor="w", pady=(2, 0))
            self.kpi_vars[key], self.kpi_boxes[key] = v, lbl

        gauge_card = tk.Frame(left, bg=self.BG_CARD, padx=10, pady=7, highlightthickness=1, highlightbackground=self.BORDER)
        gauge_card.pack(fill=tk.X, pady=(0, 8))
        g1 = tk.Frame(gauge_card, bg=self.BG_CARD)
        g1.pack(fill=tk.X, pady=2)
        self.stick_lbl = tk.Label(g1, text="Mindlin Stick Zone (c/a): 100%", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Consolas", 8, "bold"), width=32, anchor="w")
        self.stick_lbl.pack(side=tk.LEFT)
        self.stick_bar = ttk.Progressbar(g1, orient="horizontal", mode="determinate", maximum=100, value=100, style="Stick.Horizontal.TProgressbar")
        self.stick_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(8, 0))
        g2 = tk.Frame(gauge_card, bg=self.BG_CARD)
        g2.pack(fill=tk.X, pady=2)
        self.torque_lbl = tk.Label(g2, text="MCP Peak Torque Load:   0% (0.00 Nm)", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Consolas", 8, "bold"), width=32, anchor="w")
        self.torque_lbl.pack(side=tk.LEFT)
        self.torque_bar = ttk.Progressbar(g2, orient="horizontal", mode="determinate", maximum=100, value=0, style="Torque.Horizontal.TProgressbar")
        self.torque_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(8, 0))

        self.panel_frame = tk.Frame(left, bg=self.BG_CARD, highlightthickness=2, highlightbackground=self.BORDER)
        self.panel_frame.pack(fill=tk.BOTH, expand=True)
        self.result_lbl = tk.Label(self.panel_frame, text="STATUS: STANDBY", bg="#1e293b", fg=self.ACCENT_BLUE, font=("Consolas", 11, "bold"), pady=8)
        self.result_lbl.pack(fill=tk.X, side=tk.BOTTOM)
        self.nb = ttk.Notebook(self.panel_frame)
        self.nb.pack(fill=tk.BOTH, expand=True)
        tab_tel = tk.Frame(self.nb, bg=self.BG_CARD)
        tab_fing = tk.Frame(self.nb, bg=self.BG_CARD)
        tab_res = tk.Frame(self.nb, bg=self.BG_CARD)
        self.nb.add(tab_tel, text=" TELEMETRY ")
        self.nb.add(tab_fing, text=" FINGERS ")
        self.nb.add(tab_res, text=" RESULTS ")
        tab_cad = tk.Frame(self.nb, bg=self.BG_CARD)
        self.nb.add(tab_cad, text=" CAD REPORT ")
        self.cad_text = tk.Text(tab_cad, bg="#0f172a", fg="#e2e8f0", font=("Consolas", 9), wrap="word", bd=0, padx=10, pady=8, height=10)
        self.cad_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.cad_text.insert("1.0", "No CAD part loaded.\n\nUse  BROWSE STL/OBJ...  in the CAD INPUT bar, set the unit and material\n(edit any property if you know better), then press  ANALYSE & GRIP.")
        self.cad_text.config(state="disabled")
        tab_mech = tk.Frame(self.nb, bg=self.BG_CARD)
        self.nb.add(tab_mech, text=" ANALYSIS ")
        mbar = tk.Frame(tab_mech, bg=self.BG_CARD); mbar.pack(fill=tk.X, padx=6, pady=(6, 0))
        tk.Button(mbar, text="OPEN ANALYSIS GRAPHS", command=self.open_np_window, bg="#1e293b", fg=self.ACCENT_GREEN,
                  font=("Consolas", 8, "bold"), relief="flat", padx=8).pack(side=tk.LEFT)
        tk.Label(mbar, text="stress state - element equilibrium - wrist shaft - beams - finger deflection - failure limits",
                 bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Consolas", 7)).pack(side=tk.LEFT, padx=8)
        self.mech_text = tk.Text(tab_mech, bg="#0f172a", fg="#e2e8f0", font=("Consolas", 8), wrap="none", bd=0, padx=8, pady=6, height=10)
        msb = ttk.Scrollbar(tab_mech, orient="vertical", command=self.mech_text.yview)
        self.mech_text.configure(yscrollcommand=msb.set)
        msb.pack(side=tk.RIGHT, fill=tk.Y, pady=6)
        self.mech_text.pack(fill=tk.BOTH, expand=True, padx=(6, 0), pady=6)
        self.mech_text.insert("1.0", "The analysis report appears here once the pads touch the object.")
        self.mech_text.config(state="disabled")
        self._mech_t = 0.0
        self.telemetry_lbl = tk.Label(tab_tel, text="MuJoCo 3D Engine Ready & Tactile Controller Standby...", bg=self.BG_CARD, fg=self.TEXT_MAIN, font=("Consolas", 9), justify=tk.LEFT, anchor="nw", padx=12, pady=10)
        self.telemetry_lbl.pack(fill=tk.BOTH, expand=True)

        fcols = (("finger", "Finger", 58), ("contact", "Contact", 62), ("gap", "Gap mm", 58), ("N", "N (N)", 62), ("mcp", "MCP Nm", 66), ("pip", "PIP Nm", 66), ("dip", "DIP Nm", 66), ("s", "Curl s", 56))
        self.fing_tree = ttk.Treeview(tab_fing, columns=[c[0] for c in fcols], show="headings", style="Dark.Treeview", height=6)
        for cid, ctext, w in fcols:
            self.fing_tree.heading(cid, text=ctext)
            self.fing_tree.column(cid, width=w, anchor="center", stretch=True)
        self.fing_tree.tag_configure("touch", foreground=self.ACCENT_GREEN)
        self.fing_tree.tag_configure("free", foreground="#64748b")
        self.fing_tree.tag_configure("sum", foreground=self.ACCENT_AMBER)
        self.fing_tree.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        rcols = (("n", "#", 28), ("obj", "Object", 92), ("mode", "Mode", 62), ("fg", "Fn", 26), ("res", "Result", 92), ("peak", "Peak N", 56), ("fos", "FoS", 46), ("lift", "Lift mm", 56), ("m", "m_est g", 56))
        self.res_tree = ttk.Treeview(tab_res, columns=[c[0] for c in rcols], show="headings", style="Dark.Treeview")
        for cid, ctext, w in rcols:
            self.res_tree.heading(cid, text=ctext)
            self.res_tree.column(cid, width=w, anchor="center" if cid != "obj" else "w", stretch=True)
        self.res_tree.tag_configure("ok", foreground=self.ACCENT_GREEN)
        self.res_tree.tag_configure("bad", foreground=self.ACCENT_RED)
        self.res_tree.tag_configure("warn", foreground=self.ACCENT_AMBER)
        rsb = ttk.Scrollbar(tab_res, orient="vertical", command=self.res_tree.yview)
        self.res_tree.configure(yscrollcommand=rsb.set)
        rsb.pack(side=tk.RIGHT, fill=tk.Y, pady=6)
        self.res_tree.pack(fill=tk.BOTH, expand=True, padx=(6, 0), pady=6)
        self._res_count = 0
        self._bench_running = False

        plot_frame = tk.Frame(main, bg=self.BG_CARD, highlightthickness=1, highlightbackground=self.BORDER)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(7.6, 6.8), dpi=100, facecolor=self.BG_CARD)
        self.ax_force = self.fig.add_subplot(211)
        self.ax_stress = self.fig.add_subplot(212)
        self.fig.subplots_adjust(hspace=0.34, left=0.10, right=0.97, top=0.93, bottom=0.08)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        
        self.root.after(66, self._ui_poll_loop)
        if self.node.init_cad_spec:
            self._cad_fill_from_spec(self.node.init_cad_spec)
            self._cad_show_report(self.node.init_cad_rep)
        if self.node.init_cad_error:
            err = self.node.init_cad_error
            self.cad_status.config(text="CAD load failed: " + err, fg=self.ACCENT_RED)
            self.root.after(900, lambda: messagebox.showerror("CAD load failed", err))
        self.root.after(600, self.start_sim)

    # ------------------------------------------------------------------ CAD input
    CAD_FIELDS = (  # key, label, width, scale to SI, hint
        ("mass", "MASS g", 7, 1e-3, "blank = density x volume"),
        ("density", "RHO kg/m3", 7, 1.0, ""),
        ("mu", "MU", 5, 1.0, "friction coefficient pad/part"),
        ("E", "E GPa", 7, 1e9, "Young's modulus"),
        ("nu", "NU", 5, 1.0, "Poisson ratio"),
        ("strength", "SIGMA_FAIL MPa", 8, 1e6, "yield / fracture / crush strength"),
        ("F_allow", "F_MAX N", 6, 1.0, "blank = none (shell default 10 N)"),
        ("R_contact", "R mm", 6, 1e-3, "blank = measured from the mesh"),
    )

    def _build_cad_bar(self):
        BG = "#0d1424"
        cad = tk.Frame(self.root, bg=BG, pady=6, padx=14, highlightthickness=1, highlightbackground=self.BORDER)
        cad.pack(fill=tk.X)
        r1 = tk.Frame(cad, bg=BG)
        r1.pack(fill=tk.X)
        r2 = tk.Frame(cad, bg=BG)
        r2.pack(fill=tk.X, pady=(5, 0))

        def lab(parent, text): tk.Label(parent, text=text, bg=BG, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold")).pack(side=tk.LEFT, padx=(8, 3))
        def entry(parent, var, width): 
            e = tk.Entry(parent, textvariable=var, width=width, bg="#1e293b", fg="#f8fafc", insertbackground="#f8fafc", relief="flat", font=("Consolas", 9), highlightthickness=1, highlightbackground="#334155", highlightcolor=self.ACCENT_BLUE)
            e.pack(side=tk.LEFT)
            return e
        def combo(parent, var, values, width):
            cb = ttk.Combobox(parent, textvariable=var, values=values, width=width, state="readonly", font=("Consolas", 9))
            cb.pack(side=tk.LEFT)
            return cb

        tk.Label(r1, text="CAD INPUT:", bg=BG, fg=self.ACCENT_PURPLE, font=("Consolas", 9, "bold")).pack(side=tk.LEFT)
        tk.Button(r1, text="📂 BROWSE STL / OBJ...", bg="#7c3aed", fg="white", command=self.cad_browse, font=("Consolas", 9, "bold"), bd=0, padx=9, pady=4, cursor="hand2", activebackground="#334155").pack(side=tk.LEFT, padx=(8, 4))
        self.cad_path_var = tk.StringVar(value="")
        entry(r1, self.cad_path_var, 52)
        self.cad_unit_var, self.cad_up_var, self.cad_rot_var = tk.StringVar(value="mm"), tk.StringVar(value="auto"), tk.StringVar(value="")
        lab(r1, "UNIT"); combo(r1, self.cad_unit_var, list(cadmod.UNITS.keys()), 4)
        lab(r1, "UP AXIS"); combo(r1, self.cad_up_var, ["auto", "z", "y", "-y", "x", "-x", "-z"], 5)
        lab(r1, "ROT Z deg (blank=auto)"); entry(r1, self.cad_rot_var, 5)
        self.cad_status = tk.Label(r1, text="", bg=BG, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold"), anchor="w")
        self.cad_status.pack(side=tk.LEFT, padx=12)

        self.cad_mat_var = tk.StringVar(value="aluminium")
        self.cad_kind_var = tk.StringVar(value="ductile")
        self.cad_vars = {k: tk.StringVar(value="") for k, *_ in self.CAD_FIELDS}
        lab(r2, "MATERIAL")
        mcb = combo(r2, self.cad_mat_var, list(cadmod.MATERIALS.keys()), 13)
        mcb.bind("<<ComboboxSelected>>", self._cad_material_changed)
        for key, label, width, _scale, _hint in self.CAD_FIELDS:
            lab(r2, label); entry(r2, self.cad_vars[key], width)
        lab(r2, "FAILURE MODE"); combo(r2, self.cad_kind_var, list(cadmod.KIND_INFO.keys()), 9)
        tk.Button(r2, text="▶ ANALYSE & GRIP", bg="#16a34a", fg="white", command=self.cad_apply, font=("Consolas", 9, "bold"), bd=0, padx=9, pady=4, cursor="hand2", activebackground="#334155").pack(side=tk.LEFT, padx=(14, 0))
        self._cad_busy = False
        self._cad_material_changed()

    def cad_browse(self):
        path = filedialog.askopenfilename(parent=self.root, title="Select a CAD mesh (STL / OBJ)",
                                          filetypes=[("CAD mesh", "*.stl *.STL *.obj *.OBJ *.ply *.off *.3mf *.glb"), ("All files", "*.*")])
        if path:
            self.cad_path_var.set(path)
            self.cad_status.config(text="file selected - pick unit & material, then ANALYSE & GRIP", fg=self.TEXT_MUTED)

    def _cad_material_changed(self, *_):
        m = cadmod.MATERIALS.get(self.cad_mat_var.get())
        if not m: return
        v = self.cad_vars
        v["mass"].set("")
        v["density"].set("%g" % m["rho"]); v["mu"].set("%g" % m["mu"]); v["E"].set("%g" % (m["E"] / 1e9))
        v["nu"].set("%g" % m["nu"]); v["strength"].set("%g" % (m["strength"] / 1e6))
        v["F_allow"].set("10" if m["kind"] == "shell" else ""); v["R_contact"].set("")
        self.cad_kind_var.set(m["kind"])

    def _cad_spec(self):
        path = self.cad_path_var.get().strip().strip('"').strip("'")
        if not path: raise ValueError("Choose a CAD file first (BROWSE STL / OBJ...).")
        spec = dict(file=os.path.expanduser(path), unit=self.cad_unit_var.get(), up=self.cad_up_var.get(), material=self.cad_mat_var.get(), kind=self.cad_kind_var.get())
        try:
            if self.cad_rot_var.get().strip(): spec["rotz"] = float(self.cad_rot_var.get())
        except ValueError: raise ValueError("ROT Z must be a number (degrees).")
        for key, label, _w, scale, _h in self.CAD_FIELDS:
            txt = self.cad_vars[key].get().strip()
            if txt:
                try: spec[key] = float(txt) * scale
                except ValueError: raise ValueError("%s must be a number (got %r)." % (label, txt))
        return spec

    def _cad_fill_from_spec(self, spec):
        self.cad_path_var.set(spec.get("file", ""))
        self.cad_unit_var.set(spec.get("unit", "mm")); self.cad_up_var.set(spec.get("up", "auto")); self.cad_rot_var.set("" if spec.get("rotz") in (None, "") else "%g" % float(spec["rotz"]))
        self.cad_mat_var.set(spec.get("material", "aluminium")); self._cad_material_changed()
        if spec.get("kind"): self.cad_kind_var.set(spec["kind"])
        for key, _l, _w, scale, _h in self.CAD_FIELDS:
            if spec.get(key) not in (None, ""): self.cad_vars[key].set("%g" % (float(spec[key]) / scale))

    def _cad_show_report(self, rep):
        self.cad_text.config(state="normal")
        self.cad_text.delete("1.0", tk.END)
        self.cad_text.insert("1.0", cadmod.format_report(rep) + "\n\nMaterial / failure mode used: %s / %s\nE = %.3g GPa, nu = %.2f, mu = %.2f, strength = %.3g MPa" % (
            rep["material"], rep["kind"], rep["E"] / 1e9, rep["nu"], rep["mu"], rep["strength"] / 1e6))
        self.cad_text.config(state="disabled")

    def cad_apply(self):
        if self._cad_busy: return
        try: spec = self._cad_spec()
        except ValueError as e:
            messagebox.showerror("CAD input", str(e)); return
        self._cad_busy = True
        self._cad_n = max(3, min(5, int(self.fing_var.get())))
        self.cad_status.config(text="analysing mesh and planning the grasp (can take up to a minute)...", fg=self.ACCENT_AMBER)
        threading.Thread(target=self._cad_worker, args=(spec,), daemon=True).start()

    def _cad_worker(self, spec):
        try:
            key, rep = self.node.load_cad(spec)
            try:
                from .physics_check import plan_summary
                rep['plan_text'] = plan_summary(key, getattr(self, '_cad_n', 5))      # plan + MuJoCo proof here, so the report says why a part is refused
            except Exception as exc:
                rep['plan_text'] = 'Grasp planning failed: %r' % (exc,)
            self.root.after(0, self._cad_done, key, rep, None)
        except BaseException as e:
            self.root.after(0, self._cad_done, None, None, str(e) or repr(e))

    def _cad_done(self, key, rep, err):
        self._cad_busy = False
        if err:
            self.cad_status.config(text="CAD error: " + err, fg=self.ACCENT_RED)
            messagebox.showerror("CAD input", err)
            return
        self.obj_cb["values"] = list(OBJECTS.keys())
        self.obj_var.set(key)
        self._cad_show_report(rep)
        self.nb.select(3)
        ok_pad = [("GRIPPABLE" if self._grippable(key, p) and "NO SAFE GRIP" not in rep.get("plan_text", "") else "NO SAFE GRIP") for p in ("small", "large")]
        self.cad_status.config(text="loaded %s | small pad: %s | large pad: %s%s" % (key, ok_pad[0], ok_pad[1], " | " + rep["warnings"][0] if rep["warnings"] else ""), fg=self.ACCENT_GREEN if "GRIPPABLE" in ok_pad else self.ACCENT_AMBER)
        self.start_sim()

    @staticmethod
    def _grippable(key, pad):
        from .materials import truth
        t = truth(key, pad)
        return t["N_slip"] < t["N_max"]

    def _speed_changed(self, *_):
        try: self.speed_mult = max(0.1, float(self.speed_var.get().split("x")[0]))
        except Exception: self.speed_mult = 1.0

    def _init_persistent_plot_artists(self):
        self.ax_force.clear()
        self.ax_force.set_facecolor("#090d16")
        self.line_fz, = self.ax_force.plot([], [], color="#38bdf8", lw=1.8, label="f_z Normal (N)")
        self.line_fy, = self.ax_force.plot([], [], color="#22c55e", lw=1.6, label="f_y Shear (N)")
        self.line_fd, = self.ax_force.plot([], [], color="#f43f5e", ls="--", lw=1.5, label="f_d Target (N)")
        self.line_nslip = self.ax_force.axhline(0.0, color="#e2e8f0", ls=":", lw=1.2, label="N_slip")
        self.line_nmax = self.ax_force.axhline(40.0, color="#c084fc", ls=":", lw=1.5, label="N_max")
        self.ax_force.set_ylabel("Force / Finger (N)", color="#94a3b8", fontsize=8.5)
        self.ax_force.tick_params(colors="#94a3b8", labelsize=8)
        for sp in self.ax_force.spines.values(): sp.set_color("#1e293b")
        self.ax_force.legend(loc="upper left", facecolor="#111827", edgecolor="#1f2937", labelcolor="#f8fafc", fontsize=7.5, ncol=3)
        self.ax_force.grid(True, color="#1e293b", linestyle="--", alpha=0.6)

        self.ax_stress.clear()
        self.ax_stress.set_facecolor("#090d16")
        self.line_p0, = self.ax_stress.plot([], [], color="#38bdf8", lw=1.3, ls="--", label="Peak Hertz Traction p0 (kPa)")
        self.line_vm, = self.ax_stress.plot([], [], color="#f43f5e", lw=1.9, label="Von Mises Stress σ_vM (kPa)")
        self.line_tr, = self.ax_stress.plot([], [], color="#f59e0b", lw=1.3, label="Tresca Shear σ_Tr (kPa)")
        self.line_rk, = self.ax_stress.plot([], [], color="#c084fc", lw=1.3, label="Rankine Tensile σ_Rk (kPa)")
        self.line_sy = self.ax_stress.axhline(0.0, color="#ef4444", ls=":", lw=1.5, visible=False, label="Yield Limit σ_Y")
        self.ax_stress.set_xlabel("Simulation Time (s)", color="#94a3b8", fontsize=8.5)
        self.ax_stress.set_ylabel("Stress (kPa)", color="#94a3b8", fontsize=8.5)
        self.ax_stress.tick_params(colors="#94a3b8", labelsize=8)
        for sp in self.ax_stress.spines.values(): sp.set_color("#1e293b")
        self.ax_stress.legend(loc="upper left", facecolor="#111827", edgecolor="#1f2937", labelcolor="#f8fafc", fontsize=7.5, ncol=2)
        self.ax_stress.grid(True, color="#1e293b", linestyle="--", alpha=0.6)
        self._episode_setup_needed = True

    def _ui_poll_loop(self):
        if self._dirty_ui and self._latest_ep is not None:
            self._dirty_ui = False
            try: self.update_panel_and_plot(self._latest_ep, self._latest_strat)
            except Exception as e:
                if repr(e) != self._last_err:
                    import traceback; traceback.print_exc()
                    self._last_err = repr(e)
        self.root.after(66, self._ui_poll_loop)

    def _mech_compute(self, ep):
        ms, o = ep.ms, ep.o
        if ms.get('N', 0.0) <= 1e-6: return
        self._np_ctx = dict(key=ep.obj_key, o=o, ms=ms, N=ms['N'], pad=ep.pad_key, n_f=int(ep.plant.n_act),
                            s_curl=float(ep.plant.s_of(ep.plant.xf[0])))

    def update_mech(self, ep):
        now = time.perf_counter()
        if not ep.done and now - self._mech_t < 0.6: return
        self._mech_t = now
        self._mech_compute(ep)
        ctx = getattr(self, '_np_ctx', None)
        if ctx is None: return
        txt = f"{ctx['o']['label']}  |  pad force N = {ctx['N']:.2f} N  (peak-loaded finger)\n\n"
        try: txt += NP.info_text(ctx, self._np_par())
        except Exception as e: txt += f"[analysis report unavailable: {e!r}]"
        top = self.mech_text.yview()[0]
        self.mech_text.config(state="normal"); self.mech_text.delete("1.0", tk.END); self.mech_text.insert("1.0", txt)
        self.mech_text.config(state="disabled"); self.mech_text.yview_moveto(top)
        if getattr(self, '_np_win', None) is not None and ep.done: self._np_draw()

    # ---------------- analysis graphs window ----------------
    def _np_par(self):
        v = getattr(self, '_np_vars', None)
        if not v: return dict(NP.DEFAULTS)
        return dict(theta=float(v['theta'].get()), phi=float(v['phi'].get()), comp=v['comp'].get(), fsec=v['fsec'].get(),
                    theory=v['theory'].get(), fos=float(v['fos'].get()))

    def open_np_window(self):
        if getattr(self, '_np_win', None) is not None:
            try: self._np_win.lift(); self._np_draw(); return
            except tk.TclError: self._np_win = None
        w = tk.Toplevel(self.root); w.title("Stress, structure and failure analysis at the current grasp"); w.configure(bg=self.BG_CARD)
        w.geometry("1380x860")
        fig = Figure(figsize=(14.0, 7.6), dpi=100, facecolor=self.BG_CARD)
        D = NP.DEFAULTS
        self._np_vars = dict(theta=tk.DoubleVar(value=D['theta']), phi=tk.DoubleVar(value=D['phi']), comp=tk.StringVar(value=D['comp']),
                             fsec=tk.StringVar(value=D['fsec']), theory=tk.StringVar(value=D['theory']), fos=tk.DoubleVar(value=D['fos']))
        self._np_page = 0
        bar = tk.Frame(w, bg=self.BG_CARD); bar.pack(fill=tk.X, side=tk.BOTTOM)
        for i, (nm, _) in enumerate(NP.PAGES):
            tk.Button(bar, text=nm, command=lambda i=i: self._np_set_page(i), bg="#1e293b", fg=self.ACCENT_BLUE, relief="flat").pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(bar, text="REFRESH", command=self._np_draw, bg="#1e293b", fg=self.ACCENT_AMBER, relief="flat").pack(side=tk.LEFT)
        ctl = tk.Frame(w, bg=self.BG_CARD); ctl.pack(fill=tk.X, side=tk.TOP, padx=6, pady=4)
        def lab(t): tk.Label(ctl, text=t, bg=self.BG_CARD, fg=self.TEXT_MUTED, font=("Consolas", 8, "bold")).pack(side=tk.LEFT, padx=(8, 2))
        def scale(var, lo, hi, step):
            s = tk.Scale(ctl, variable=var, from_=lo, to=hi, resolution=step, orient=tk.HORIZONTAL, length=110, bg=self.BG_CARD, fg=self.TEXT_MAIN,
                         highlightthickness=0, troughcolor="#1e293b", font=("Consolas", 7), command=lambda _=None: self._np_draw()); s.pack(side=tk.LEFT)
        def combo(var, vals, width):
            c = ttk.Combobox(ctl, textvariable=var, values=list(vals), width=width, state="readonly"); c.pack(side=tk.LEFT)
            c.bind("<<ComboboxSelected>>", lambda _e: self._np_draw())
        lab("PLANE theta"); scale(self._np_vars['theta'], 0, 90, 1)
        lab("phi"); scale(self._np_vars['phi'], 0, 360, 5)
        lab("BEAM"); combo(self._np_vars['comp'], NP.COMPONENTS, 9)
        lab("FINGER SECTION"); combo(self._np_vars['fsec'], NP.FINGER_SECTIONS, 24)
        lab("FAILURE THEORY"); combo(self._np_vars['theory'], NP.THEORIES, 12)
        lab("FoS"); scale(self._np_vars['fos'], 1.0, 4.0, 0.1)
        cv = FigureCanvasTkAgg(fig, master=w); cv.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._np_win, self._np_fig, self._np_cv = w, fig, cv
        w.protocol("WM_DELETE_WINDOW", lambda: (w.destroy(), setattr(self, '_np_win', None)))
        self._np_draw()

    def _np_set_page(self, i):
        self._np_page = i
        self._np_draw()

    def _np_draw(self):
        ctx = getattr(self, '_np_ctx', None)
        if getattr(self, '_np_win', None) is None or ctx is None: return
        try:
            NP.draw_page(self._np_fig, self._np_page, ctx, self._np_par(), dict(bg=self.BG_CARD, fg="#e2e8f0", grid="#334155"))
            self._np_fig.suptitle(f"{ctx['o']['label']}   N = {ctx['N']:.2f} N   [{NP.PAGES[self._np_page][0]}]", color="#f8fafc", fontsize=10)
            self._np_cv.draw_idle()
        except Exception:
            import traceback; traceback.print_exc()

    def toggle_pause(self): self.node.paused = not self.node.paused
    def stop_sim(self): self.node.stop_req = True

    def start_sim(self):
        params = dict(obj_key=self.obj_var.get(), strat_label=self.strat_var.get(),
                      pad='small' if 'small' in self.pad_var.get() else 'large',
                      n=max(3, min(5, int(self.fing_var.get()))))
        self.node.stop_req = True                    # asks a running episode to finish; the new thread waits for it
        self._init_persistent_plot_artists()
        threading.Thread(target=self.run_episode, args=(params,), daemon=True).start()

    def _verdict_tag(self, v):
        return "ok" if v == "OK" else ("warn" if v in ("NOT LIFTED", "ABORT") else "bad")

    def add_result_row(self, obj, mode, n, verdict, peak, fos, lift_mm, m_est_g):
        self._res_count += 1
        self.res_tree.insert("", tk.END, values=(self._res_count, obj, mode, n, verdict, f"{peak:.2f}", fmt_fos(fos), f"{lift_mm:.1f}", f"{m_est_g:.0f}"), tags=(self._verdict_tag(verdict),))
        kids = self.res_tree.get_children()
        if kids: self.res_tree.see(kids[-1])

    def log_episode(self, ep):
        if getattr(ep, "_logged", False) or not ep.plan["ok"]: return
        ep._logged = True
        m_est = (ep.out["m_est"] * 1000.0) if ep.out else 0.0
        self.add_result_row(ep.obj_key if hasattr(ep, "obj_key") else self.obj_var.get(), ep.strat_key if hasattr(ep, "strat_key") else self.strat_var.get().split()[0].lower(), ep.plant.n, ep.verdict(), ep.peak_N, ep.min_fos, ep.plant.zo * 1000.0, m_est)

    def update_finger_table(self, pl, ep=None):
        n = pl.n
        kids = self.fing_tree.get_children()
        if len(kids) != n + 1:
            for k in kids: self.fing_tree.delete(k)
            for i in range(n): self.fing_tree.insert("", tk.END, iid=f"f{i}", values=())
            self.fing_tree.insert("", tk.END, iid="fsum", values=(), tags=("sum",))
        for i in range(n):
            tq = pl.torques(i)
            touching = pl.N[i] > 0.0
            self.fing_tree.item(f"f{i}", values=(f"F{i+1}", "YES" if touching else "-", (f"{ep.dome_gap[i]*1000:+.2f}" if ep is not None else ""), f"{pl.N[i]:.2f}", f"{tq[0]:.2f}", f"{tq[1]:.2f}", f"{tq[2]:.2f}", f"{pl.s_of(pl.xf[i]):.2f}"), tags=("touch" if touching else "free",))
        self.fing_tree.item("fsum", values=("TOTAL", f"{pl.n_contact()}/{n}", "", f"{sum(pl.N):.2f}", "", "", "", ""), tags=("sum",))

    def start_benchmark(self):
        if self._bench_running: return
        self._bench_running = True
        self.btns["▦ BENCHMARK ALL"].config(state=tk.DISABLED, text="▦ RUNNING...")
        self.nb.select(2)
        threading.Thread(target=self._benchmark_worker, daemon=True).start()

    def _benchmark_worker(self):
        try:
            if self.node.running:
                self.node.stop_req = True
                t0 = time.time()
                while self.node.running and time.time() - t0 < 4.0: time.sleep(0.05)
            n = max(3, min(5, int(self.fing_var.get())))
            pad = "small" if "small" in self.pad_var.get() else "large"
            for k in list(OBJECTS.keys()):
                for strat in ("adaptive", "fixed"):
                    try:
                        ep = run_headless(k, pad, strat, self.node.seed, n)
                        if not ep.plan["ok"]:
                            row = (k, strat, n, "CANNOT GRASP", 0.0, float("nan"), 0.0, 0.0)
                        else:
                            row = (k, strat, n, ep.verdict(), ep.peak_N, ep.min_fos, ep.plant.zo * 1000.0, (ep.out["m_est"] * 1000.0) if ep.out else 0.0)
                    except Exception as e:
                        row = (k, strat, n, "ERROR", 0.0, float("nan"), 0.0, 0.0)
                    self.root.after(0, lambda r=row: self.add_result_row(*r))
        finally:
            self._bench_running = False
            self.root.after(0, lambda: self.btns["▦ BENCHMARK ALL"].config(state=tk.NORMAL, text="▦ BENCHMARK ALL"))

    def run_episode(self, params):
        t0 = time.time()
        while self.node.running and time.time() - t0 < 4.0: time.sleep(0.02)
        self.node.running, self.node.stop_req, self.node.paused = True, False, False
        try: self._run_episode(params)
        except Exception:
            import traceback; traceback.print_exc()
        finally: self.node.running = False

    def _run_episode(self, params):
        obj_key, strat_label, n = params['obj_key'], params['strat_label'], params['n']
        o = OBJECTS[obj_key]
        try:
            from .physics_check import verified_plan
            verified_plan(obj_key, n)            # MuJoCo rigid-contact proof; replaces the plan by the first alternative that really lifts
        except Exception as exc:
            print('physics check skipped:', exc)
        ep = Episode(obj_key, params['pad'], 'adaptive' if 'Adaptive' in strat_label else 'fixed', self.node.seed, n)
        if not ep.plan['ok']:
            self.node.ensure_scene(n, obj_key)                      # show the part and the hand at home, never a stale / previous scene
            self.node.sync_frame(ep.tcp0[0], ep.tcp0[1], [0.0] * n, o, 0.0, 0.0, False, 0.0, [0.0] * n)
            self._latest_ep, self._latest_strat, self._dirty_ui = ep, strat_label, True
            return

        self.node.ensure_scene(n, obj_key)
        self.node.sync_frame(ep.tcp_cmd[0], ep.tcp_cmd[1], ep.plant.s_list(), o, 0.0, 0.0, False, ep.yaw_cmd, ep.plan['sigmas'])

        SIM_DT, FRAME_DT = 0.01, 1.0 / 60.0
        acc, last = 0.0, time.perf_counter()
        while ep.t < 90.0 and not self.node.stop_req:
            if self.node.paused:
                while self.node.paused and not self.node.stop_req: time.sleep(0.03)
                last = time.perf_counter()
                continue
            t_frame = time.perf_counter()
            mult = self.speed_mult                                  # 1.0 = simulated time runs exactly at wall-clock speed
            acc = min(acc + (t_frame - last) * mult, 0.25 * max(mult, 1.0))
            last = t_frame
            nt = int(acc / SIM_DT)
            acc -= nt * SIM_DT
            for _ in range(nt):
                ep.tick()
                if ep.done: break
            if nt:
                self.node.sync_frame(ep.tcp_cmd[0], ep.tcp_cmd[1], ep.plant.s_list(), o, 0.0, ep.plant.zo, ep.crushed, ep.yaw_cmd, ep.plan['sigmas'], ep)
                self._latest_ep, self._latest_strat, self._dirty_ui = ep, strat_label, True
            if ep.done: break
            sleep_s = FRAME_DT - (time.perf_counter() - t_frame)
            if sleep_s > 0.001: time.sleep(sleep_s)

    def update_panel_and_plot(self, ep, strat_label):
        if not ep.plan['ok']:
            self.panel_frame.config(highlightbackground=self.ACCENT_AMBER)
            self.result_lbl.config(text="CANNOT GRASP: " + '; '.join(sorted(set(ep.plan['msgs']))[:2]), fg=self.ACCENT_AMBER)
            return
        if ep.out is None: return

        o, out, ms, tr, pl = ep.o, ep.out, ep.ms, ep.tr, ep.plant
        active_key = "HOLD" if out['state'] in ("HOLD", "DONE") else out['state']
        for st, pill in self.stage_pills.items():
            pill.config(bg=self.ACCENT_BLUE if st == active_key else ("#3f1d24" if out['state'] == "ABORT" else "#1e293b"), fg="#090d16" if st == active_key else ("#f87171" if out['state'] == "ABORT" else "#64748b"))
        self.arm_badge.config(text=f"ARM: {ep.arm_stage:<7} | t = {ep.t:5.2f}s")
        
        self.kpi_vars["fz"].set(f"{out['fz']:.2f} N")
        self.kpi_vars["fos"].set(fmt_fos(ms['fos']))
        self.kpi_vars["vm"].set(fmt_stress(ms['sig_vm']))
        self.kpi_vars["lift"].set(f"{pl.zo*1000:.1f} mm")
        self.kpi_boxes["fos"].config(fg=self.ACCENT_GREEN if ms['fos'] >= 1.5 else (self.ACCENT_AMBER if ms['fos'] >= 1.0 else self.ACCENT_RED))
        
        self.stick_bar["value"] = max(0.0, min(100.0, ms['c_over_a'] * 100.0))
        self.stick_lbl.config(text=f"Mindlin Stick Zone (c/a): {self.stick_bar['value']:4.0f}%")
        self.torque_bar["value"] = min(100.0, (pl.torques(0)[0] / max(HD.HAND['tau_max'][0], 1e-6)) * 100.0)
        self.torque_lbl.config(text=f"MCP Peak Torque Load: {self.torque_bar['value']:4.0f}% ({pl.torques(0)[0]:.2f} Nm)")

        self.telemetry_lbl.config(text=(
            f"SPECIMEN : {o['label']}  |  {ep.pad_key} pad  |  {o['mode']}\n"
            f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
            f"▸ 1. BLIND TACTILE CONTROLLER\n"
            f"  Force f_z / f_y / f_d : {out['fz']:5.2f} / {out['fy']:5.2f} / {out['fd']:5.2f} N  (f0={out['f0'] or 0.0:.2f}N)\n"
            f"  Slip Gate s_c / alpha : {out['s_c']} / {out['alpha']:4.2f}   [ramp={out['ramp']}  probe={out['probe']}]\n"
            f"  Probed E* / Force Cap : {fmt_modulus(out['Es_est'])}  ->  Cap {out['N_cap']:.1f} N\n"
            f"  Inferred E_obj / R*    : {'> ' if out['E_obj_lb'] else ''}{fmt_modulus(out['E_obj_est'])}{' (pad-limited: stiff part)' if out['E_obj_lb'] else ''} / {out['Re_est']*1000:.1f} mm\n"
            f"  Inferred Mass / mu_lb : {out['m_est']*1000:4.0f} g (true {o['m']*1000:.0f} g)  |  mu >= {out['mu_lb']:.2f}\n\n"
            f"▸ 2. HERTZIAN CONTACT & MINDLIN TRACTIONS\n"
            f"  Composite E*, R*      : {fmt_modulus(ms['Es'])},  R* = {ms['Re']*1000:.1f} mm\n"
            f"  Indentation d, a, e_c : d = {ms['delta']*1000:5.3f} mm, a = {ms['a']*1000:5.2f} mm, e_c = {ms['eps_c']*100:4.2f}%\n"
            f"  Normal p0 / p_mean    : {fmt_stress(ms['p0'])} / {fmt_stress(ms['p_mean'])}\n"
            f"  Shear tau_m / edge    : {fmt_stress(ms['tau_mean'])} / {fmt_stress(ms['tau_edge'])}\n\n"
            f"▸ 3. SUB-SURFACE CAUCHY STRESS (z = 0.48a)\n"
            f"  Principal (sr, sz)    : {fmt_stress(ms['sig1'])},  {fmt_stress(ms['sig3'])}\n"
            f"  Von Mises sig_vM      : {fmt_stress(ms['sig_vm'])}  (Allow: {fmt_stress(ms['sig_y_allow'])})\n"
            f"  Tresca / Rankine      : {fmt_stress(ms['sig_tresca'])} / {fmt_stress(ms['sig_rankine'])}\n"
            f"  Arm vs Object Lift    : {ep.rise*1000:5.1f} mm / {pl.zo*1000:5.1f} mm  (slip: {-(ep.rise - pl.zo)*1000.0:4.1f} mm)\n\n"
            f"▸ 4. DEXTEROUS HAND MECHANICS ({pl.n}-Finger Synergy)\n"
            f"  Synergy Curl s / Tilt : {pl.s_of(pl.mean_xf()):4.2f} / {math.degrees(pl.psi_of(pl.mean_xf())):4.0f} deg   (Contacts: {pl.n_contact()}/{pl.n})\n"
            f"  Torques MCP/PIP/DIP   : {pl.torques(0)[0]:4.2f} / {pl.torques(0)[1]:4.2f} / {pl.torques(0)[2]:4.2f} Nm  (defl {pl.deflect*1000:4.2f} mm)\n"
            f"  Finger / Arm TCP speed: {pl.close_speed()*1000:4.1f} mm/s / {ep.tcp_speed*1000:5.1f} mm/s  (lift paused for slip: {'YES' if ep.lift_held else 'no'})\n"
            f"  Gravity W vs n*mu*N   : {o['m']*G:5.2f} N  vs  {sum(pl.N)*o['mu']:6.2f} N  (Margin x{(sum(pl.N)*o['mu']/(o['m']*G) if o['m']*G>0 else 0):.1f})"
        ))

        self.update_finger_table(pl, ep)
        self.update_mech(ep)
        if ep.done: self.log_episode(ep)

        if ep.crushed:
            self.panel_frame.config(highlightbackground=self.ACCENT_RED)
            self.result_lbl.config(text=f"✖ CRUSHED / YIELDED  (min FoS={fmt_fos(ep.min_fos)}, Peak {ep.peak_N:.1f}N)", bg="#450a0a", fg="#fca5a5")
        elif out['done']:
            if out['abort']:
                self.panel_frame.config(highlightbackground=self.ACCENT_AMBER)
                self.result_lbl.config(text=f"⚠ ABORTED: {out['hint']} (Safety Cap Reached)", bg="#451a03", fg="#fcd34d")
            elif pl.zo / GRIPPER['lift_height'] >= 0.5:
                self.panel_frame.config(highlightbackground=self.ACCENT_GREEN)
                self.result_lbl.config(text=f"✔ GRASP & LIFT SUCCESS  (Peak {ep.peak_N:.2f}N, min FoS={fmt_fos(ep.min_fos)})", bg="#052e16", fg="#86efac")
            elif ep.lifted_max / GRIPPER['lift_height'] >= 0.2:
                self.panel_frame.config(highlightbackground=self.ACCENT_RED)
                self.result_lbl.config(text="✖ OBJECT SLIPPED OFF DURING LIFT", bg="#450a0a", fg="#fca5a5")
            else:
                self.panel_frame.config(highlightbackground=self.ACCENT_RED)
                self.result_lbl.config(text="✖ FAILED TO LIFT OBJECT", bg="#450a0a", fg="#fca5a5")
        elif pl.zo > 0.005:
            self.panel_frame.config(highlightbackground=self.ACCENT_GREEN)
            self.result_lbl.config(text=f"▲ LIFTING SPECIMEN ({pl.zo*1000:.1f} mm, FoS={self.kpi_vars['fos'].get()})", bg="#052e16", fg="#86efac")
        else:
            self.panel_frame.config(highlightbackground=self.BORDER)
            self.result_lbl.config(text=f"● EXECUTING: {out['state']} ({ep.arm_stage})...", bg="#1e293b", fg=self.ACCENT_BLUE)

        now = time.perf_counter()
        if not ep.done and now - self._last_plot_t < 0.4: return
        self._last_plot_t = now
        m = min(len(ep.t_hist), len(ep.fz_hist), len(ep.fy_hist), len(ep.fd_hist), len(ep.p0_hist), len(ep.vm_hist), len(ep.tr_hist), len(ep.rk_hist))
        if m == 0: return
        step = max(1, m // 500)
        T, FZ, FY, FD = ep.t_hist[:m:step], ep.fz_hist[:m:step], ep.fy_hist[:m:step], ep.fd_hist[:m:step]
        P0, VM, TR, RK = ep.p0_hist[:m:step], ep.vm_hist[:m:step], ep.tr_hist[:m:step], ep.rk_hist[:m:step]
        if self._episode_setup_needed:
            self._episode_setup_needed = False
            self.line_nslip.set_ydata([tr['N_slip'], tr['N_slip']])
            if tr['N_max'] < 65.0:
                self.line_nmax.set_ydata([tr['N_max'], tr['N_max']])
                self.line_nmax.set_visible(True)
            else: self.line_nmax.set_visible(False)
            if ms['sig_y_allow'] is not None and ms['sig_y_allow'] < 1e6:
                self.line_sy.set_ydata([ms['sig_y_allow'] / 1e3, ms['sig_y_allow'] / 1e3])
                self.line_sy.set_visible(True)
            else: self.line_sy.set_visible(False)

        self.line_fz.set_data(T, FZ); self.line_fy.set_data(T, FY); self.line_fd.set_data(T, FD)
        self.ax_force.set_xlim(0.0, max(5.0, T[-1] + 0.5))
        self.ax_force.set_ylim(0.0, max(max(FZ, default=1.0), max(FD, default=1.0), tr['N_slip'] * 1.3, 2.0) * 1.15)
        self.ax_force.set_title(f"TACTILE FORCE & SLIP CONTROL  //  {pl.n}-Finger Hand on {o['label']}", color="#f8fafc", fontsize=9.5, fontweight="bold")
        
        self.line_p0.set_data(T, P0); self.line_vm.set_data(T, VM); self.line_tr.set_data(T, TR); self.line_rk.set_data(T, RK)
        self.ax_stress.set_xlim(0.0, max(5.0, T[-1] + 0.5))
        self.ax_stress.set_ylim(0.0, max(max(P0, default=1.0), max(VM, default=1.0), max(TR, default=1.0), 1.0) * 1.15)
        self.ax_stress.set_title(f"SUB-SURFACE CAUCHY STRESSES  //  Strain e_c = {ms['eps_c']*100:.2f}%  |  Stick Zone c/a = {ms['c_over_a']*100:.0f}%", color="#f8fafc", fontsize=9.5, fontweight="bold")
        self.canvas.draw_idle()

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoSimBridgeNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    gui = SimDashboardGUI(node)
    try: gui.root.mainloop()
    finally:
        node.close_viewer()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
