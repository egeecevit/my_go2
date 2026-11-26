#!/usr/bin/env python3
#
# Copyright (C) 2005-2026 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Every experiment of thesis Sec. 6.6 (Ozden 2026, AMB as a whole-body
controller task) on the simulated Go2, run headless, scored and plotted.

  /home/ege/legged/iros/.venv/bin/python figs/amb_wbc/run_experiments.py [--only NAME ...] [--out DIR]

  ablation     Sec. 6.6.4, Table 6.4, Fig. 6.7-6.8 (+ joint-space pair, thesis gains)
  transfer     Sec. 6.6.2 and 6.6.7, Fig. 6.3, 6.21, 6.22, Table 6.6
  x3           Sec. 6.6.3, Fig. 6.4-6.5
  orientation  Sec. 6.6.5.1, Fig. 6.11-6.12
  force        Sec. 6.6.5.2, Fig. 6.14-6.15
  push         Sec. 6.6.5.3, Table 6.5, Fig. 6.16-6.18 (WBC and joint-space sweeps)
  workspace    Sec. 6.6.6, Fig. 6.19-6.20 (on the WBC sweep's runs)

Each run is one bin/sim.sh call driven by the Supervisor key script "0 s",
"2.5 b" (stand, then MdlDiagonalStance, which hands over to MdlAMB by itself);
everything experiment-specific is a -c override. Runs are serialised by a lock
(the simulator binds a fixed port). Logs: runs/amb_wbc/<experiment>/<run>.mat
and .log; results.json and figs/ in reports/amb_wbc/. --out DIR puts all three
under DIR. Once results.json holds every experiment, REPORT.md and REPORT.pdf
are rebuilt next to it (build_report.py, which also runs alone, without
simulating). Torso state, CoM and L come from simulator truth (qpos, qvel through
the MuJoCo model), reduced and principal-frame quantities from MdlAMB_state and
MdlAMB_wbc (layouts at _log and _logWbc in include/quadruped/MdlAMB.hh).
"""

import argparse
import fcntl
import json
import os
import re
import subprocess
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from scipy.io import loadmat  # noqa: E402

import build_report  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BIN = os.path.join(REPO, "bin")
LOCK = os.path.join(REPO, "runs", ".simlock")   # one lock whatever --out says: one port

# Sec. 6.6.1 fall criteria, fixed here and never tuned per trial.
FALL_Z = 0.16                  # torso height [m]; the ablation's only criterion
FALL_ANG = np.radians(45.0)    # |roll| or |pitch|: push sweep, and every other trial
FOOT_R = 0.022                 # go2.xml foot sphere: contacts sit one radius below the centres
T_B = 2.5                      # 'b' key [s]: MdlStand (2 s) has settled
ENTRY_MAX = 6.0                # bound on 'b' -> hand-off (4.6 s), for the ablation's fixed exit
DELAY = 2.0                    # disturbances 2 s after the hand-off: the thesis' 5.5 s warm-up
                               # minus its ~3.5 s balance start (Fig. 6.14, 6.18, 6.19)
OBS_ABL = 12.0                 # ablation observation [s]: >= 10 s gate (thesis 8.447 s)
OBS_LONG = 60.0                # long balance [s]: the slow x2 mode needs tens of seconds to show
POST_FALL = 1.0                # ablation metrics end 1 s after a fall, torso on the ground
                               # (the thesis observed 1.15 s past its fall)
PULSE, OBS_PUSH, PUSH_MAX = 0.1, 8.0, 40   # pulse [s], window from its start [s], sweep cap [N]
OBS_FORCE = 12.0               # persistent force: 2 s ramp from 2 s, then 8 s at full force
LOG_PERIOD = 2                 # 500 Hz: resolves the 0.1 s pulse and the L-dot spikes
# 125 of the 128 doubles a LogClient takes; push runs trade the GRF for the
# MIT-law torque, which the Sec. 6.6.6 saturation question needs.
VARS = ["MdlAMB_state", "MdlAMB_wbc", "MdlSimDriver_qpos", "MdlSimDriver_qvel", "MdlSimDriver_grf"]
VARS_TAU = VARS[:4] + ["MdlAMB_tau"]
TAU_LIM = np.tile([23.7, 23.7, 45.43], 4)   # Go2 motors, motor order
X3_SCHEDULE = [[0.4, -340.0], [2.0, -220.0], [10.0, -250.0], [15.0, -270.0], [20.0, -360.0]]  # Fig. 6.4
# Table 6.2. The code's u0 = -(k0 L + ...)/g takes the thesis' negative gains as magnitudes.
THESIS_GAINS = ["amb.gains = [3430.0, 1470.0, 210.0]", "amb.template_m = [10.5, 2.26, 2.26]"]
THESIS_ORI = ["amb.kp_body_ori = 100.0", "amb.kd_body_ori = 10.0"]   # Table 6.2; amb.toml has (400, 40)
PATHS = {"wbc": [], "joint": ["amb.wbc = false"]}
EXPERIMENTS = ["ablation", "transfer", "x3", "orientation", "force", "push", "workspace"]
LEGS = ["FL", "FR", "RL", "RR"]
SWING = (("FR", 14), ("RL", 21))   # MdlAMB_wbc block: xi_n, rho, xi_t, their refs, feedforward status
DES, SOL = {"FL": 4, "FR": 40, "RL": 43, "RR": 7}, {"FL": 10, "FR": 46, "RL": 49, "RR": 13}  # Fz slots
COL = {"FL": "#00b894", "FR": "#257de0", "RL": "#f47230", "RR": "#6b6b6b",
       "on": "#257de0", "off": "#f47230", "ink": "#191919"}
CMAP = LinearSegmentedColormap.from_list("blues", plt.get_cmap("Blues")(np.linspace(0.3, 1.0, 64)))
THESIS = {
    "table_6_4": {"disabled": {"fall": True, "time_to_fall": 1.299, "min_z": 0.1595, "peak_roll": 29.66,
                               "peak_pitch": 14.41, "peak_w": 5.404, "peak_L": 3.830, "peak_cx": 0.1769,
                               "observed": 2.447},
                  "enabled": {"fall": False, "min_z": 0.3026, "peak_roll": 1.887, "peak_pitch": 0.978,
                              "peak_w": 0.151, "peak_L": 0.0352, "peak_cx": 0.00245, "observed": 8.447}},
    "table_6_5": {"columns": ["impulse", "max_cx", "max_L", "z_min", "success"],
                  9: [0.9, 0.00957, 0.219, 0.3066, True], 10: [1.0, 0.00920, 0.244, 0.3066, True],
                  11: [1.1, 0.00922, 0.273, 0.3017, True], 12: [1.2, 0.20182, 4.754, 0.1312, False],
                  13: [1.3, 0.36269, 6.924, 0.0874, False]},
    "table_6_6": {"columns": ["mean_desired", "mean_solved", "rmse"],
                  "transfer": {"FR": [18.41, 19.61, 1.40], "FL": [55.24, 58.42, 3.38],
                               "RR": [55.24, 49.58, 5.89], "RL": [18.41, 16.22, 2.57]},
                  "final": {"FR": [36.83, 39.36, 2.54], "FL": [36.83, 37.83, 1.00],
                            "RR": [36.83, 34.07, 2.76], "RL": [36.83, 32.37, 4.46]},
                  "total_rmse": {"transfer": 3.48, "final": 3.68}},
    "principal_rmse_mm": {"normal": 7.59, "radial": 18.95}, "final_body_rmse_mm": 7.13,
    "force_steady_deg": {"x1": {2: -0.95, 4: -1.7, 6: -2.4, 8: -3.2}, "x2": {2: -10, 4: -21, 6: -33, 8: -42}},
    "orientation_peaks_ref_meas_deg": {"pitch": [5.1, 4.6], "yaw": [2.9, 2.3]},
}

plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5, "legend.fontsize": 7, "lines.linewidth": 1.1,
                     "axes.grid": True, "grid.color": "#e4e4e4", "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "savefig.dpi": 200})

_M = mujoco.MjModel.from_xml_path(os.path.join(REPO, "models", "unitree_go2", "scene.xml"))
_D = mujoco.MjData(_M)
_FOOT = [_M.geom(n).id for n in LEGS]
_HIP = [_M.body(n + "_hip").id for n in LEGS]
ABD_RANGE = _M.jnt_range[_M.joint("FR_hip_joint").id]   # +-1.0472, every abduction
MASS = float(_M.body_subtreemass[1])


def simulate(path, cfg, exit_time, keys=(), logvars=VARS):
    """One headless run -> path.mat, path.log; returns its wall time [s]."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    for ext in (".mat", ".log"):
        if os.path.exists(path + ext):
            os.remove(path + ext)
    over = ["simulation.headless = true", "simulation.realtime = false",
            f"supervisor.exit_time = {exit_time:.4f}",
            f"supervisor.script = {json.dumps(['0 s', f'{T_B} b'] + list(keys))}",
            "supervisor.log.enable = true", f"supervisor.log.start = {T_B}",
            f"supervisor.log.period = {LOG_PERIOD}", 'supervisor.log.file_format = "matlab"',
            f'supervisor.log.file_name = "{path}.mat"', f"supervisor.log.vars = {json.dumps(logvars)}"] + cfg
    args = ["./sim.sh"] + [x for c in over for x in ("-c", c)]
    os.makedirs(os.path.dirname(LOCK), exist_ok=True)
    t0 = time.time()
    with open(LOCK, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if subprocess.run(["pgrep", "-x", "simulation"], capture_output=True).returncode == 0:
            raise RuntimeError("another simulation process is running (it binds the same port)")
        with open(path + ".log", "w") as log:
            log.write("overrides: " + json.dumps(over) + "\n")
            log.flush()
            rc = subprocess.run(args, cwd=BIN, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                timeout=3 * exit_time + 60).returncode
    if rc != 0 or not os.path.exists(path + ".mat"):
        raise RuntimeError(f"{path}: exit {rc}, no log (see {path}.log)")
    print(f"  [{time.time() - t0:5.1f} s] {os.path.basename(os.path.dirname(path))}/{os.path.basename(path)}")
    return time.time() - t0


def qmul(a, b):
    w1, x1, y1, z1 = a.T
    w2, x2, y2, z2 = b.T
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=1)


def zyx(q):
    w, x, y, z = q.T
    return np.stack([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                     np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                     np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))], axis=1)


class Run:
    """A loaded run. t is relative to the hand-off to MdlAMB (first sample
    with a non-zero MdlAMB_state), th is its sim time. Truth quantities are
    about the current FL-RR support line through the ground contact points."""

    def __init__(self, path):
        raw = loadmat(path + ".mat", squeeze_me=False, struct_as_record=True)
        rec = raw["data"].ravel()
        D = {n: np.concatenate([np.atleast_2d(v) for v in rec[n]], axis=1).T.astype(float) for n in rec.dtype.names}
        tabs = D["time"][:, 0] - D["time"][0, 0] + float(np.squeeze(raw["params"]["time_offset"][0, 0])) * 1e-6
        self.S, self.W = D["MdlAMB_state"], D["MdlAMB_wbc"]
        self.qpos, self.qvel = D["MdlSimDriver_qpos"], D["MdlSimDriver_qvel"]
        self.grf, self.tau = D.get("MdlSimDriver_grf"), D.get("MdlAMB_tau")
        active = np.nonzero(np.any(self.S != 0.0, axis=1))[0]
        if not len(active):
            raise RuntimeError(f"{path}: MdlAMB never ran (no hand-off in the log)")
        self.i0 = active[0]
        self.th = float(tabs[self.i0])
        self.t = tabs - self.th
        n = len(self.t)
        com, cp, Lc, vc = np.empty((n, 3)), np.empty((n, 4, 3)), np.empty((n, 3)), np.empty((n, 3))
        self.base_com = np.empty((n, 3))   # where xfrc_applied puts the push
        for i in range(n):
            _D.qpos[:], _D.qvel[:] = self.qpos[i], self.qvel[i]
            for f in (mujoco.mj_kinematics, mujoco.mj_comPos, mujoco.mj_comVel, mujoco.mj_subtreeVel):
                f(_M, _D)
            com[i], cp[i] = _D.subtree_com[1], _D.geom_xpos[_FOOT] - [0.0, 0.0, FOOT_R]
            self.base_com[i] = _D.xipos[1]
            Lc[i], vc[i] = _D.subtree_angmom[1], _D.subtree_linvel[1]
        s = cp[:, 3] - cp[:, 0]
        s /= np.linalg.norm(s, axis=1, keepdims=True)
        mid = 0.5 * (cp[:, 0] + cp[:, 3])
        th = np.stack([s[:, 1], -s[:, 0], np.zeros(n)], axis=1)   # t_h = s x t_v (5.43), levelled
        th /= np.linalg.norm(th, axis=1, keepdims=True)
        self.com, self.cp, self.mid, self.s_line = com, cp, mid, s
        self.L = np.sum(s * (Lc + MASS * np.cross(com - mid, vc)), axis=1)   # equals MdlAMB_state[9]
        self.Ld = np.gradient(self.L, self.t)
        self.cx = np.sum(th * (com - mid), axis=1)          # thesis c_x, signed along t_h
        self.vperp = np.sum(th * self.qvel[:, 0:3], axis=1)
        self.rpy, self.z = zyx(self.qpos[:, 3:7]), self.qpos[:, 2]
        self.wnorm = np.linalg.norm(self.qvel[:, 3:6], axis=1)
        text = open(path + ".log", errors="replace").read()
        m = re.search(r"MdlAMB: max\|theta1\| after the lift ([\d.]+) deg, torque saturated on ([\d.]+)%", text)
        # Another module's warning can land inside the line (seen after a failed sit).
        k = re.search(r"computeTick mean ([\d.]+) us, max ([\d.]*)(?:\x1b\[[\d;]*m\[WW\][^\n]*\n)?([\d.]*) us", text)
        self.closing = {"max_theta1_deg": float(m.group(1)) if m else None,
                        "torque_saturated_pct": float(m.group(2)) if m else None,
                        "tick_mean_us": float(k.group(1)) if k else None,
                        "tick_max_us": float(k.group(2) + k.group(3)) if k else None}
        e = re.search(r"\[WW\] MdlAMB: (.*?) at t=([\d.]+)(.*)", text)
        self.error = e and {"t": round(float(e.group(2)) - self.th, 4),
                            "message": (e.group(1) + e.group(3)).replace("\x1b[0m", "").strip()}
        # theta2 in (-2pi, 0] and theta3 in (0, 2pi] (MdlAMB geometry()) jump by
        # 2 pi when a swing leg passes straight up: a branch cut, not motion.
        # Plots use the unwrapped angles; the first jump is recorded.
        self.live = live = slice(self.i0, None if not self.error else np.searchsorted(self.t, self.error["t"]))
        self.theta = self.S[:, 1:3].copy()
        self.theta[live] = np.unwrap(self.theta[live], axis=0)
        jump = np.zeros(n, bool)
        jump[live][1:] = np.any(np.abs(np.diff(self.S[live, 1:3], axis=0)) > np.pi, axis=1)
        self.wrap_t = first_t(self, jump)
        # Swing-abduction margin to the +-60 deg stops, from the truth (both paths).
        self.abd_margin = np.degrees(ABD_RANGE[1] - np.abs(self.qpos[:, [10, 13]]).max(axis=1))
        self.msg_t = {}
        for v, key in re.findall(r"MdlAMB: t=([\d.]+) (placement started|balance stop|SETTLED)", text):
            self.msg_t.setdefault(key, float(v) - self.th)

    def win(self, a, b):
        return (self.t >= a) & (self.t <= b)

    def fall(self, t_from, angles=True):
        bad = self.z < FALL_Z
        if angles:
            bad |= np.any(np.abs(self.rpy[:, :2]) > FALL_ANG, axis=1)
        return first_t(self, bad & (self.t >= t_from))

    def ori_ref(self):
        """ZYX angles of the controller's reference R_d = Exp(e) R, e the
        logged rotation vector of R_d R^T (MdlAMB_wbc[31:34])."""
        e = self.W[:, 31:34]
        a = np.linalg.norm(e, axis=1)
        k = np.where(a > 1e-12, np.sin(a / 2) / np.maximum(a, 1e-12), 0.5)
        return zyx(qmul(np.c_[np.cos(a / 2), k[:, None] * e], self.qpos[:, 3:7]))

    def summary(self):
        return {"handoff_s": round(self.th, 4), "closing": self.closing, "amb_error": self.error,
                "theta_wrap_t": self.wrap_t}

    def margin(self, t_end):
        """Abduction margin at the end of a balanced window and its trend over
        the last 2 s: a 'no fall' over a window is not a settled state while
        the swing legs still walk toward their stops."""
        w = self.win(t_end - 2.0, t_end)
        return {"abduction_margin_end_deg": round(float(self.abd_margin[w][-1]), 2),
                "abduction_margin_rate_deg_per_s": round(float(np.polyfit(self.t[w], self.abd_margin[w], 1)[0]), 3)}


def first_t(R, mask):
    idx = np.nonzero(mask)[0]
    return round(float(R.t[idx[0]]), 4) if len(idx) else None


def r6(x):
    return None if x is None else round(float(x), 6)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else None


def figure(nr, nc, h_cm, **kw):
    return plt.subplots(nr, nc, figsize=(16 / 2.54, h_cm / 2.54), squeeze=False, **kw)


def topdown(ax, R, t0, t1, color=None, label=None, mark=None, line=True):
    """Top-down CoM [mm] from the hand-off support midpoint and the hand-off
    support line (thesis Fig. 6.4, 6.8, 6.12, 6.15-6.17); the view is the
    CoM path's."""
    w = R.win(t0, t1)
    o = R.mid[R.i0]
    x, y = 1e3 * (R.com[w, 0] - o[0]), 1e3 * (R.com[w, 1] - o[1])
    if color:
        ax.plot(x, y, color=color, label=label)
    else:
        sc = ax.scatter(x, y, c=R.t[w], cmap=CMAP, s=1.5, vmin=t0, vmax=t1, rasterized=True)
        plt.colorbar(sc, ax=ax, label="t [s]", pad=0.01)
    if line:
        a, b = 1e3 * (R.cp[R.i0, 0, :2] - o[:2]), 1e3 * (R.cp[R.i0, 3, :2] - o[:2])
        ax.axline(a, b, color=COL["ink"], ls="--", lw=0.8, label="support line")
    h = max(np.ptp(x), np.ptp(y), 1.0) * 0.6
    ax.set(xlim=(x.min() + x.max()) / 2 + np.r_[-h, h], ylim=(y.min() + y.max()) / 2 + np.r_[-h, h],
           xlabel="$c_x$ world [mm]", ylabel="$c_y$ world [mm]")
    if mark is not None:
        i = np.searchsorted(R.t, mark)
        ax.plot(1e3 * (R.com[i, 0] - o[0]), 1e3 * (R.com[i, 1] - o[1]), "o", mfc="none", mec=COL["ink"], ms=6)


def ref_plot(ax, t, meas, ref, color, label=""):
    ax.plot(t, meas, color=color, label=f"{label} measured".strip())
    ax.plot(t, ref, color=color, ls="--", label=f"{label} reference".strip())


def gate(name, ok, numbers):
    return {"gate": name, "result": "pass" if ok else "fail", "numbers": numbers}


class Experiments:
    def __init__(self, logs, rep):
        self.logs, self.rep, self.wall, self.sweeps = logs, rep, {}, {}

    def save(self, f, name):
        os.makedirs(os.path.join(self.rep, "figs"), exist_ok=True)
        f.tight_layout()
        f.savefig(os.path.join(self.rep, "figs", name))
        plt.close(f)
        return "figs/" + name

    def run(self, exp, name, cfg, exit_time, keys=(), logvars=VARS):
        path = os.path.join(self.logs, exp, name)
        self.wall[f"{exp}/{name}"] = round(simulate(path, cfg, exit_time, keys, logvars), 1)
        R = Run(path)
        if abs(R.th - self.th) > 0.003:
            print(f"  WARNING {name}: hand-off at {R.th:.3f} s, baseline {self.th:.3f} s: disturbance timing off")
        return R

    def baseline(self, rerun):
        """Hand-off time and support-normal direction from the canonical AMB-on
        run, which is the ablation's first run."""
        path = os.path.join(self.logs, "ablation", "wbc_on")
        if rerun or not os.path.exists(path + ".mat"):
            self.wall["ablation/wbc_on"] = round(simulate(path, [], T_B + ENTRY_MAX + OBS_ABL), 1)
        R = self.base = Run(path)
        self.th = R.th
        if self.th - T_B > ENTRY_MAX:
            raise RuntimeError(f"hand-off {self.th - T_B:.2f} s after 'b' exceeds ENTRY_MAX")
        # Support line from the feet that carry the weight when the disturbances start.
        i = np.searchsorted(R.t, DELAY)
        s = R.cp[i, 3, :2] - R.cp[i, 0, :2]
        s /= np.linalg.norm(s)
        # Thesis direction: +t_h. A horizontal force F d at height h adds F h
        # (d . t_h) to L-dot about s (FL -> RR, the code's and the thesis' L);
        # Fig. 6.18 has L and L-dot positive in the pulse, Fig. 6.15 the CoM
        # leaning to -t_h against the ramped force, Fig. 6.14 x1 settling
        # negative. One sign for every trial, as in the thesis.
        self.n_push = np.array([s[1], -s[0]])
        yaw = R.rpy[i, 2]
        self.direction = {"support_line_s_xy": s.round(4).tolist(), "push_direction_xy": self.n_push.round(4).tolist(),
                          # MdlDiagonalStance puts FL and RR on the centreline, so the line
                          # runs almost along body x and the swing feet's tangential
                          # channel is nearly pure hip abduction.
                          "support_line_angle_to_body_x_deg": r6(np.degrees(np.arccos(
                              abs(s @ [np.cos(yaw), np.sin(yaw)])))),
                          "note": "+t_h, t_h = s x t_v levelled, s from the FL to the RR contact at hand-off + 2 s "
                                  "of the baseline run; sign of thesis Fig. 6.18 (L, L-dot > 0 in the pulse)",
                          "grf_z_N": dict(zip(LEGS, R.grf[i, 2::3].round(2).tolist())),
                          "contact_height_mm": dict(zip(LEGS, (1e3 * R.cp[i, :, 2]).round(1).tolist()))}
        print(f"hand-off at {self.th:.3f} s sim time; push direction (world xy) {self.n_push.round(4)}")

    def push_cfg(self, F, duration, ramp=0.0, sign=1):
        f = sign * F * self.n_push
        return [f"simulation.push_force = [{f[0]:.6f}, {f[1]:.6f}, 0.0]", f"simulation.push_time = {self.th + DELAY:.4f}",
                f"simulation.push_duration = {duration}", f"simulation.push_ramp = {ramp}"]

    # -- a. ablation, Table 6.4, Fig. 6.7-6.8 --------------------------------
    def ablation(self):
        cfgs = {"wbc_off": ["amb.enabled = false"], "joint_on": PATHS["joint"],
                "joint_off": PATHS["joint"] + ["amb.enabled = false"], "wbc_thesis_gains": THESIS_GAINS}
        runs = {"wbc_on": self.base}
        runs.update({n: self.run("ablation", n, c, T_B + ENTRY_MAX + OBS_ABL) for n, c in cfgs.items()})
        res = {"cfg": cfgs, "runs": {}, "thesis_table_6_4": THESIS["table_6_4"],
               "units": "s, m, deg, rad/s, kg m^2/s, m; x2 (unwrapped) while MdlAMB runs",
               "amb_off_realization": "amb.enabled = false keeps the WBC, its hierarchy and the lift; it removes the "
                                      "AMB feedforward a_f and the tangential pass-through (6.23): the swing-foot task "
                                      "takes the tangential channel through the same lift to the lift target's angle "
                                      "about e_r0 and PD-holds it there (MdlAMB.cc _wbcTick), as the joint-space "
                                      "ablation PD-holds the lift posture. The thesis says only that the run 'removes "
                                      "the tangential balance acceleration'."}
        for name, R in runs.items():
            tf = R.fall(0.0, angles=False)
            t_end = min(OBS_ABL, tf + POST_FALL) if tf is not None else OBS_ABL
            w = R.win(0.0, t_end)
            x2, live = np.degrees(R.theta.sum(axis=1)), R.win(0.0, R.error["t"] if R.error else OBS_ABL)
            last = live & (R.t >= R.t[live][-1] - 2.0)
            res["runs"][name] = dict(R.summary(), **{
                "fall": tf is not None, "time_to_fall": tf, "min_z": r6(R.z[w].min()),
                "peak_roll": r6(np.degrees(np.abs(R.rpy[w, 0]).max())),
                "peak_pitch": r6(np.degrees(np.abs(R.rpy[w, 1]).max())), "peak_w": r6(R.wnorm[w].max()),
                "peak_L": r6(np.abs(R.L[w]).max()), "peak_cx": r6(np.abs(R.cx[w]).max()), "observed": r6(t_end),
                "max_abs_x2_deg": r6(np.abs(x2[live]).max()),
                "x2_drift_last_2s_deg_per_s": r6(np.polyfit(R.t[last], x2[last], 1)[0]),
                **(R.margin(OBS_ABL) if tf is None else {})})
        # Is the balance a settled state? Default against Table 6.2's body-orientation gains.
        res["long"] = {}
        for name, c in (("wbc_on", []), ("wbc_thesis_ori_gains", THESIS_ORI)):
            R = self.run("ablation", name + "_long", c, T_B + ENTRY_MAX + OBS_LONG)
            tf = R.fall(0.0)
            live = R.win(0.0, R.error["t"] if R.error else OBS_LONG)
            x2 = np.degrees(R.theta[live].sum(axis=1))
            at = {f"x2_deg_at_{t}s": r6(x2[min(np.searchsorted(R.t[live], t), len(x2) - 1)]) for t in (5, 20, 35)}
            res["long"][name] = dict(R.summary(), cfg=c, observed=OBS_LONG, fall=tf is not None, time_to_fall=tf,
                                     max_theta1_deg=r6(np.degrees(np.abs(R.S[live, 0]).max())), **at,
                                     **(R.margin(OBS_LONG) if tf is None else {}))
        on, off, tg = (res["runs"][n] for n in ("wbc_on", "wbc_off", "wbc_thesis_gains"))
        res["thesis_gains_balance"] = not tg["fall"] and tg["amb_error"] is None
        res["gate"] = gate(
            "ablation: AMB on no fall >= 10 s, AMB off falls",
            not on["fall"] and on["amb_error"] is None and on["observed"] >= 10.0 and off["fall"],
            f"on: no fall over {on['observed']:.0f} s, min z {on['min_z']:.3f} m, peak |L| {on['peak_L']:.4f}, "
            f"peak |c_x| {1e3 * on['peak_cx']:.2f} mm, abduction margin at the end "
            f"{on.get('abduction_margin_end_deg')} deg ({on.get('abduction_margin_rate_deg_per_s')} deg/s); "
            f"off: falls {off['time_to_fall']} s after activation "
            f"(thesis 1.299), MdlAMB error at {off['amb_error'] and off['amb_error']['t']} s; joint path on/off falls "
            f"{res['runs']['joint_on']['fall']}/{res['runs']['joint_off']['fall']}; thesis gains "
            f"{'balance' if res['thesis_gains_balance'] else 'do not balance'}")
        f, ax = figure(1, 2, 6.5)
        for a, names, title in ((ax[0, 0], ("wbc_on", "wbc_off"), "whole body (canonical pair)"),
                                (ax[0, 1], ("joint_on", "joint_off", "wbc_thesis_gains"), "variants")):
            for name, c, ls in zip(names, (COL["on"], COL["off"], COL["FL"]), ("-", "--", "-.")):
                w = runs[name].win(-2.0, OBS_ABL)
                a.plot(runs[name].t[w], runs[name].z[w], color=c, ls=ls, label=name.replace("_", " "))
            a.axhline(FALL_Z, color=COL["ink"], ls=":", lw=0.8, label="0.16 m fall threshold")
            a.axvline(0.0, color="#999999", lw=0.8)
            a.set(title=title, xlabel="t from diagonal-balance activation [s]", ylabel="torso height [m]", ylim=(0, 0.36))
            a.legend(loc="lower left")
        res["figs"] = [self.save(f, "fig6_07_ablation_height.png")]
        f, ax = figure(1, 2, 7)
        for a, names in zip(ax[0], (("wbc_on", "wbc_off"), ("wbc_off", "wbc_on"))):
            for name in names:      # the last one drawn sets the view
                tf = res["runs"][name]["time_to_fall"]
                topdown(a, runs[name], 0.0, min(OBS_ABL, (tf or OBS_ABL) + POST_FALL), color=COL[name[4:]],
                        label=f"AMB {name[4:]}" + (" (to 1 s after the fall)" if tf else ""), line=name == names[1])
            a.set_title("AMB off, whole fall" if names[1] == "wbc_off" else f"AMB on, {OBS_ABL:.0f} s (zoom)")
            a.legend(loc="best")
        res["figs"].append(self.save(f, "fig6_08_ablation_com.png"))
        return res

    # -- b. principal frame + load transfer, Fig. 6.3, 6.21, 6.22, Table 6.6 --
    def transfer(self):
        t_stop = 5.0
        R = self.run("transfer", "wbc", [], self.th + t_stop + 13.0, keys=[f"{self.th + t_stop:.4f} d"])
        W, ph, act = R.W, R.W[:, 0], R.t >= 0.0
        t_pl, t_tr, t_fi = (first_t(R, act & (ph == p)) for p in (2.0, 3.0, 4.0))
        t_set = R.msg_t.get("SETTLED")
        res = dict(R.summary(), phases={"placement": t_pl, "transfer": t_tr, "final": t_fi, "settled": r6(t_set)})
        if t_tr is None or t_set is None:
            res["gate"] = gate("load transfer", False, f"sequence incomplete: {res['phases']}, error {R.error}")
            res["gate_principal"] = gate("principal-frame tracking", False, "sequence incomplete")
            return res
        table = {}
        for key, (a, b) in {"transfer": (t_tr, t_fi), "final": (t_fi, t_set)}.items():
            w = R.win(a, b - 1e-6)
            rows = {}
            for k, leg in enumerate(LEGS):
                d, s, g = W[w, DES[leg]], W[w, SOL[leg]], R.grf[w, 3 * k + 2]
                rows[leg] = {"mean_desired": r6(d.mean()), "mean_solved": r6(s.mean()), "rmse_des_sol": r6(rms(d - s)),
                             "mean_sim": r6(g.mean()), "rmse_des_sim": r6(rms(d - g)), "rmse_sol_sim": r6(rms(s - g))}
            rows["pooled_rmse_des_sol"] = r6(np.sqrt(np.mean([rows[leg]["rmse_des_sol"] ** 2 for leg in LEGS])))
            rows["rmse_of_summed_force_des_sol"] = r6(rms(sum(W[w, DES[g]] - W[w, SOL[g]] for g in LEGS)))
            rows["duration_s"] = r6(b - a)
            table[key] = rows
        res.update(table_6_6=table, thesis_table_6_6=THESIS["table_6_6"],
                   table_6_6_note="forces [N]; final = the 3 s four-foot hold up to SETTLED; the thesis' 'total' "
                                  "RMSE is undefined there, pooled and summed-force RMSE given here")
        prin, errs = {}, {"normal": [], "radial": []}
        for leg, b in SWING:       # two-foot part: lift end to each foot's promotion
            w = act & (ph >= 1.0) & (ph <= 2.0) & (W[:, b + 6] != 3.0)
            e = {"normal": W[w, b] - W[w, b + 3], "radial": W[w, b + 1] - W[w, b + 4]}
            prin[leg] = {f"{c}_{m}_mm": r6(1e3 * f(e[c])) for c in e
                         for m, f in (("rmse", rms), ("max", lambda x: np.abs(x).max()))}
            prin[leg]["tangential_rmse_mm"] = r6(1e3 * rms(W[w, b + 2] - W[w, b + 5]))   # 0 by (6.23)
            # The thesis' "two-leg portion" also holds the transient right after
            # balance start, so the same with the 0.4 s lift included.
            wl = act & (ph <= 2.0) & (W[:, b + 6] != 3.0)
            el = {"normal": W[wl, b] - W[wl, b + 3], "radial": W[wl, b + 1] - W[wl, b + 4]}
            prin[leg]["with_lift"] = {f"{c}_{m}_mm": r6(1e3 * f(el[c])) for c in el
                                      for m, f in (("rmse", rms), ("max", lambda x: np.abs(x).max()))}
            for c in e:
                errs[c].append(e[c])
        prin["both_rmse_mm"] = {c: r6(1e3 * rms(np.concatenate(errs[c]))) for c in errs}
        prin["thesis_rmse_mm"] = THESIS["principal_rmse_mm"]
        prin["window"] = "lift end to each foot's promotion; with_lift: from activation"
        res["principal_two_foot"] = prin
        wf = R.win(t_fi, t_set - 1e-6)
        res["final_body_rmse_mm"] = r6(1e3 * rms(np.linalg.norm(W[wf, 28:31], axis=1)))
        # The same in the truth: base position from the support midpoint, against
        # its value at the hand-off, which is what the controller latches as _p_ref.
        rel = R.qpos[:, 0:3] - R.mid
        res["final_body_rmse_truth_mm"] = r6(1e3 * rms(np.linalg.norm(rel[wf] - rel[R.i0], axis=1)))
        res["final_body_note"] = ("final_body_rmse_mm is the controller's own error (MdlAMB_wbc 28-30, base from "
                                  "the stance kinematics); _truth_mm is the simulator's; the thesis' 7.13 mm was "
                                  "measured under its simulated-state override")
        fz = R.grf[R.win(t_set - 0.5, t_set - 1e-6)][:, 2::3].mean(0)
        fell = R.fall(0.0)
        res.update(final_sim_fz_N=dict(zip(LEGS, fz.round(2).tolist())), weight_N=r6(MASS * 9.81),
                   fall_before_settled=fell is not None and fell <= t_set)
        worst = max(table[k][g]["rmse_des_sol"] for k in table for g in LEGS)
        worst_sim = max(table[k][g]["rmse_des_sim"] for k in table for g in LEGS)
        # Four-foot support: every simulated foot force above 5 N over the last
        # 0.5 s of the hold (an unloaded contact reads ~0; test_amb_wbc_sim's bound).
        res["gate"] = gate("load transfer: four-foot support, no fall, desired-vs-solved RMSE < 10 N",
                           (fz > 5.0).all() and not res["fall_before_settled"] and worst < 10.0,
                           f"SETTLED {t_set:.2f} s after activation, final sim Fz {fz.round(1).tolist()} N (sum "
                           f"{fz.sum():.1f}, weight {MASS * 9.81:.1f}); worst per-foot RMSE desired-solved {worst:.2f} "
                           f"N (thesis <= 5.89), desired-simulated {worst_sim:.2f} N; final body RMSE "
                           f"{res['final_body_rmse_mm']:.2f} mm controller frame, "
                           f"{res['final_body_rmse_truth_mm']:.2f} mm truth (thesis 7.13)")
        mx = max(prin[g][c + "_max_mm"] for g, _ in SWING for c in errs)
        # Bounded: below a quarter of the ~0.2 m held radius throughout.
        res["gate_principal"] = gate(
            "principal-frame tracking: normal and radial errors bounded", mx < 50.0,
            f"two-foot part, max |error| {mx:.1f} mm (< 50); RMSE normal FR/RL {prin['FR']['normal_rmse_mm']:.2f}/"
            f"{prin['RL']['normal_rmse_mm']:.2f} mm, radial {prin['FR']['radial_rmse_mm']:.2f}/"
            f"{prin['RL']['radial_rmse_mm']:.2f} mm (thesis 7.59 / 18.95)")
        marks = [(0.0, "balance start"), (t_pl, "placement"), (t_tr, "balance stop"), (t_fi, "final hold")]
        w = R.win(0.0, t_set)
        f, ax = figure(4, 1, 17, sharex=True)
        for leg, b in SWING:
            unused = np.where(W[w, b + 6] == 3.0, np.nan, 1.0)   # references unused once promoted
            rd = W[w, b + 4]
            for a, j in ((ax[0, 0], 1), (ax[2, 0], 0)):
                ref_plot(a, R.t[w], W[w, b + j], W[w, b + 3 + j] * unused, COL[leg], leg)
            ref_plot(ax[1, 0], R.t[w], np.degrees(W[w, b + 2] / rd), np.degrees(W[w, b + 5] / rd) * unused,
                     COL[leg], leg)
        # x3d finishes its placement ramp after the balance stop, unused by then (no swing task).
        ref_plot(ax[3, 0], R.t[w], np.degrees(R.S[w, 1] - R.S[w, 2]),
                 np.degrees(R.S[w, 23]) * np.where(R.t[w] >= t_tr, np.nan, 1.0), COL["ink"], "$x_3$")
        for a, lab in zip(ax[:, 0], ("radial $\\rho$ [m]", "tangential $\\xi_t/\\rho^d$ [deg]",
                                     "normal $\\xi_n$ [m]", "$x_3$ [deg]")):
            a.set_ylabel(lab)
            for tm, txt in marks:
                a.axvline(tm, color="#999999", lw=0.7, ls=":")
            a.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))
        for tm, txt in marks:
            ax[0, 0].text(tm, ax[0, 0].get_ylim()[1], txt, fontsize=6.5, va="bottom")
        ax[3, 0].set_xlabel("t from diagonal-balance activation [s]")
        res["figs"] = [self.save(f, "fig6_03_principal_frame.png")]
        f, ax = figure(2, 1, 11, sharex=True)
        for a, pair, title in ((ax[0, 0], ("FL", "RR"), "original diagonal stance feet"),
                               (ax[1, 0], ("FR", "RL"), "swing feet: touchdown, transfer, final support")):
            for leg in pair:
                a.plot(R.t[w], W[w, DES[leg]], color=COL[leg], ls="--", label=f"{leg} desired")
                a.plot(R.t[w], W[w, SOL[leg]], color=COL[leg], label=f"{leg} solved")
                a.plot(R.t[w], R.grf[w, 3 * LEGS.index(leg) + 2], color=COL[leg], ls=":", lw=1.4, label=f"{leg} simulated")
            a.axvspan(t_tr, t_fi, color="#f2f2f2", zorder=0)
            a.axvspan(t_fi, t_set, color="#e6eef8", zorder=0)
            a.set(title=title, ylabel="normal force $F_z$ [N]")
            a.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))
        ax[1, 0].set_xlabel("t from diagonal-balance activation [s] (grey: 3 s transfer, blue: final hold)")
        res["figs"].append(self.save(f, "fig6_21_normal_forces.png"))
        f, ax = figure(4, 1, 15, sharex=True)
        for j, a in enumerate(ax[:3, 0]):
            ref_plot(a, R.t[w], R.qpos[w, j], R.qpos[w, j] + W[w, 28 + j], COL["ink"])
            a.set_ylabel(f"body {'xyz'[j]} [m]")
        ref_plot(ax[3, 0], R.t[w], np.degrees(R.rpy[w, 0]), np.degrees(R.ori_ref()[w, 0]), COL["ink"])
        ax[3, 0].set(ylabel="roll [deg]", xlabel="t from diagonal-balance activation [s]")
        for a in ax[:, 0]:
            a.axvspan(t_tr, t_set, color="#e6eef8", zorder=0)
        ax[0, 0].legend(loc="best", title="blue: four-leg balance; reference = truth + controller's error",
                        title_fontsize=6.5)
        res["figs"].append(self.save(f, "fig6_22_body.png"))
        return res

    # -- c. x3 schedule, Fig. 6.4-6.5 -------------------------------------------
    def x3(self):
        T = 26.0
        R = self.run("x3", "schedule", [f"amb.x3_schedule = {json.dumps(X3_SCHEDULE)}"], self.th + T + 0.2)
        x3, x3d = np.degrees(R.S[:, 1] - R.S[:, 2]), np.degrees(R.S[:, 23])
        steps = []
        for k, (ts, ref) in enumerate(X3_SCHEDULE):
            te = X3_SCHEDULE[k + 1][0] if k + 1 < len(X3_SCHEDULE) else T
            w = R.win(te - 1.0, te - 1e-6)    # last 1 s of the step
            steps.append({"from_s": ts, "to_s": te, "x3d_deg": ref, "steady_error_deg": r6(np.mean(x3[w] - x3d[w]))})
        fell, Rd, w = R.fall(0.0), R.ori_ref(), R.win(0.0, T)
        roll_dev = np.abs(R.rpy[w, 0] - Rd[w, 0])
        res = dict(R.summary(), steps=steps, fall=fell, max_cx_mm=r6(1e3 * np.abs(R.cx[w]).max()),
                   reachable_note="Fig. 6.4 applied in full: -360...-220 deg lies in the measured reachable "
                                  "[-360, -200] deg (kinematic [-372, -205] with the x2 coupling)",
                   torso_max_dev={"height_mm": r6(1e3 * np.abs(R.W[w, 30]).max()),
                                  **{a + "_deg": r6(np.degrees(np.abs(R.rpy[w, j] - Rd[w, j]).max()))
                                     for j, a in enumerate(("roll", "pitch", "yaw"))},
                                  "roll_max_at_s": r6(R.t[w][np.argmax(roll_dev)])},
                   **(R.margin(T) if fell is None else {}))
        worst = max(abs(s["steady_error_deg"]) for s in steps)
        res["gate"] = gate("x3: steady error < 5 deg on every step, no fall",
                           worst < 5.0 and fell is None and R.error is None,
                           f"steady errors {[round(s['steady_error_deg'], 2) for s in steps]} deg (last 1 s of each "
                           f"step), no fall over {T:.0f} s: {fell is None}, max |c_x| {res['max_cx_mm']:.2f} mm, "
                           f"abduction margin at the end {res.get('abduction_margin_end_deg')} deg "
                           f"({res.get('abduction_margin_rate_deg_per_s')} deg/s)")
        f, ax = figure(1, 2, 6.5)
        topdown(ax[0, 0], R, 0.0, T)
        ax[0, 0].set_title("top-down CoM")
        ref_plot(ax[0, 1], R.t[w], x3[w], x3d[w], COL["ink"], "$x_3$")
        ax[0, 1].set(xlabel="t from activation [s]", ylabel="$x_3$ [deg]", title="appendage separation")
        ax[0, 1].legend()
        res["figs"] = [self.save(f, "fig6_04_x3_tracking.png")]
        f, ax = figure(2, 2, 9, sharex=True)
        pairs = [(R.z, R.z + R.W[:, 30], "torso height [m]", 1.0)] + \
                [(R.rpy[:, j], Rd[:, j], f"{a} [deg]", 180 / np.pi) for j, a in enumerate(("roll", "pitch", "yaw"))]
        for a, (m, r, lab, sc) in zip(ax.ravel(), pairs):
            ref_plot(a, R.t[w], sc * m[w], sc * r[w], COL["ink"])
            a.set_ylabel(lab)
        ax[0, 0].legend(title="reference: WBC task", title_fontsize=6.5)
        for a in ax[1]:
            a.set_xlabel("t from activation [s]")
        res["figs"].append(self.save(f, "fig6_05_x3_torso.png"))
        return res

    # -- d. orientation, Fig. 6.11-6.12 -------------------------------------------
    def orientation(self):
        T, om = 18.0, 2 * np.pi * 0.1
        res, runs = {}, {}
        for name, j, key, amp in (("pitch5", 1, "pitch_amp_deg", 5.0), ("yaw3", 2, "yaw_amp_deg", 3.0)):
            R = runs[name] = self.run("orientation", name, [f"amb.{key} = {amp}"], self.th + T + 0.2)
            R.ref = R.ori_ref()
            w, wl = R.win(4.0, T), R.win(0.0, T)     # after the 2 s delay and 2 s envelope
            X = np.c_[np.sin(om * R.t[w]), np.cos(om * R.t[w]), np.ones(w.sum())]
            cm, cr = (np.linalg.lstsq(X, np.degrees(v[w, j]), rcond=None)[0] for v in (R.rpy, R.ref))
            # a sin + b cos = A sin(om t + phi), phi = arg(a + ib); positive lag: measured behind
            res[name] = dict(R.summary(), amplitude_ref_deg=r6(np.hypot(*cr[:2])),
                             amplitude_ratio=r6(np.hypot(*cm[:2]) / np.hypot(*cr[:2])),
                             lag_s=r6(np.angle(complex(*cr[:2]) / complex(*cm[:2])) / om),
                             tracking_rms_deg=r6(np.degrees(rms(R.rpy[w, j] - R.ref[w, j]))),
                             max_L=r6(np.abs(R.L[wl]).max()), max_L_sinusoid=r6(np.abs(R.L[w]).max()),
                             max_Ld=r6(np.abs(R.Ld[wl]).max()), max_cx_mm=r6(1e3 * np.abs(R.cx[wl]).max()),
                             fall=R.fall(0.0), **R.margin(T))
        # Tracked: amplitude within -20/+25 %, lag under 0.5 s (18 deg at 0.1 Hz); the
        # thesis' own ratios are ~0.8-0.9. L bounded: under 0.15 kg m^2/s, the range
        # of thesis Fig. 6.11, which holds its own transient (0.12).
        ok = all(0.8 <= r["amplitude_ratio"] <= 1.25 and abs(r["lag_s"]) <= 0.5 and r["max_L"] < 0.15
                 and r["fall"] is None and r["amb_error"] is None for r in res.values())
        res["thesis_peaks_ref_meas_deg"] = THESIS["orientation_peaks_ref_meas_deg"]
        res["gate"] = gate("orientation: both sinusoids tracked, L bounded, no fall", ok, "; ".join(
            f"{n}: ratio {res[n]['amplitude_ratio']:.3f}, lag {res[n]['lag_s']:.3f} s, max|L| {res[n]['max_L']:.4f}, "
            f"no fall {res[n]['fall'] is None}" for n in runs))
        f, ax = figure(3, 2, 13, sharex=True)
        for c, (name, j, lab) in enumerate((("pitch5", 1, "pitch"), ("yaw3", 2, "yaw"))):
            R = runs[name]
            w = R.win(0.0, T)
            ref_plot(ax[0, c], R.t[w], np.degrees(R.rpy[w, j]), np.degrees(R.ref[w, j]), COL["ink"])
            ax[0, c].set(title=f"{lab} trial", ylabel=f"{lab} [deg]")
            ax[1, c].plot(R.t[w], R.L[w], color=COL["on"])
            ax[2, c].plot(R.t[w], R.Ld[w], color=COL["on"])
            ax[1, c].set_ylabel("$L$ [kg m$^2$/s]")
            ax[2, c].set(ylabel="$\\dot L$ [N m]", xlabel="t from activation [s]")
            ax[0, c].legend(loc="lower left")
        res["figs"] = [self.save(f, "fig6_11_orientation.png")]
        f, ax = figure(1, 2, 6.5)
        for a, name in zip(ax[0], runs):
            topdown(a, runs[name], 0.0, T)
            a.set_title(f"{name[:-1]} trial")
        res["figs"].append(self.save(f, "fig6_12_orientation_com.png"))
        return res

    # -- e. persistent force, Fig. 6.14-6.15 ------------------------------------
    def force(self):
        runs = {}

        def bounded(F):
            if F not in runs:
                runs[F] = self.run("force", f"F{F}", self.push_cfg(F, OBS_FORCE - DELAY, ramp=2.0),
                                   self.th + OBS_FORCE + 0.1)
            R = runs[F]
            return R.fall(0.0) is None and (R.error is None or R.error["t"] > OBS_FORCE)

        ok = {F: bounded(F) for F in (2, 4, 6, 8)}
        fails = [F for F in ok if not ok[F]]
        if fails:          # 1 N refinement below the first failing force
            lo = max([F for F in ok if ok[F] and F < min(fails)], default=0)
            ok.update({F: bounded(F) for F in range(lo + 1, min(fails))})
            fails = [F for F in ok if not ok[F]]
        res = {"direction": self.direction, "runs": {}, "thesis": THESIS["force_steady_deg"],
               "bounded_definition": f"no fall and no MdlAMB error until {OBS_FORCE:.0f} s after activation "
                                     f"(8 s at full force)"}
        for F in sorted(runs):
            R = runs[F]
            x = np.c_[R.S[:, 3], R.theta.sum(axis=1), R.theta[:, 0] - R.theta[:, 1]]
            w, tail, t2 = R.win(0.0, OBS_FORCE), R.win(OBS_FORCE - 1, OBS_FORCE), R.win(OBS_FORCE - 2, OBS_FORCE)
            live = w & (R.t <= (R.error["t"] if R.error else OBS_FORCE))
            # Static balance about the line: M g c_x = -F h, h the height of the
            # base body's CoM, where xfrc_applied acts.
            res["runs"][F] = dict(R.summary(), bounded=ok[F], fall=R.fall(0.0),
                                  steady_x_deg=None if R.error else np.degrees(x[tail].mean(0)).round(3).tolist(),
                                  x2_drift_deg_per_s=None if R.error else r6(np.degrees(np.polyfit(R.t[t2], x[t2, 1], 1)[0])),
                                  steady_cx_mm=None if R.error else r6(1e3 * R.cx[tail].mean()),
                                  required_cx_mm=r6(-1e3 * F * (R.base_com[R.i0, 2] - R.mid[R.i0, 2]) / (MASS * 9.81)),
                                  min_joint_limit_distance_rad=r6(R.W[live, 34].min()),
                                  max_cx_mm=r6(1e3 * np.abs(R.cx[w]).max()),
                                  **(R.margin(OBS_FORCE) if ok[F] else {}))
        lo = max([F for F in ok if ok[F] and (not fails or F < min(fails))], default=None)
        res["boundary_N"] = [lo, min(fails)] if fails else None
        res["gate"] = gate("persistent force: bounded for 2/4/6/8 N, or the boundary reported",
                           not fails or lo is not None,
                           ("all four bounded" if not fails else
                            f"NOT all bounded (thesis: all four): bounded {sorted(F for F in ok if ok[F])} N, failing "
                            f"{sorted(fails)} N, boundary {lo}-{min(fails)} N") + "; min joint-limit distance [rad] " +
                           ", ".join(f"{F} N {res['runs'][F]['min_joint_limit_distance_rad']:.3f}" for F in sorted(runs)) +
                           "; bounded runs' abduction margin at the end [deg] (rate [deg/s]) " +
                           ", ".join(f"{F} N {res['runs'][F]['abduction_margin_end_deg']} "
                                     f"({res['runs'][F]['abduction_margin_rate_deg_per_s']})" for F in sorted(runs) if ok[F]))
        Fs = sorted(runs)
        f, ax = figure(3, 1, 13, sharex=True)
        for k, F in enumerate(Fs):
            R = runs[F]
            w = R.win(0.0, R.error["t"] if R.error else OBS_FORCE)
            for a, v in zip(ax[:, 0], (R.S[:, 3], R.theta.sum(axis=1), R.theta[:, 0] - R.theta[:, 1])):
                a.plot(R.t[w], np.degrees(v[w]), color=CMAP(k / max(1, len(Fs) - 1)),
                       label=f"{F} N" + ("" if ok[F] else " (fails)"))
        for a, lab in zip(ax[:, 0], ("$x_1$ [deg]", "$x_2$ [deg]", "$x_3$ [deg]")):
            a.set_ylabel(lab)
            a.axvspan(DELAY, DELAY + 2.0, color="#f2f2f2", zorder=0)
        ax[0, 0].legend(loc="center left", bbox_to_anchor=(1.0, 0.5), title="grey: ramp")
        ax[2, 0].set_xlabel("t from activation [s] (failing runs end at the MdlAMB error; "
                            "$\\theta_2$, $\\theta_3$ unwrapped)")
        res["figs"] = [self.save(f, "fig6_14_force_states.png")]
        f, ax = figure(2, (len(Fs) + 1) // 2, 11)
        for a, F in zip(ax.ravel(), Fs):
            R = runs[F]
            topdown(a, R, 0.0, min(OBS_FORCE, R.error["t"] + 0.2) if R.error else OBS_FORCE, mark=DELAY)
            a.set_title(f"{F} N" + ("" if ok[F] else " (fails)"))
        for a in ax.ravel()[len(Fs):]:
            a.axis("off")
        res["figs"].append(self.save(f, "fig6_15_force_com.png"))
        return res

    # -- f. pushes, Table 6.5, Fig. 6.16-6.18 ------------------------------------
    def push_run(self, path, F, sign=1):
        R = self.run("push", f"{path}_F{F}" if sign > 0 else f"{path}_minus_th_F{F}",
                     PATHS[path] + self.push_cfg(F, PULSE, sign=sign), self.th + DELAY + OBS_PUSH + 0.1,
                     logvars=VARS_TAU)
        w = R.win(DELAY + PULSE, DELAY + OBS_PUSH)     # Table 6.5: post-pulse quantities
        tf = R.fall(DELAY + PULSE)
        tf = tf if tf is not None and tf <= DELAY + OBS_PUSH else None
        a = R.win(DELAY, min(DELAY + OBS_PUSH, R.error["t"] if R.error else np.inf))
        R.row = dict(R.summary(), F=F, impulse=r6(F * PULSE), max_cx=r6(np.abs(R.cx[w]).max()),
                     max_L=r6(np.abs(R.L[w]).max()), z_min=r6(R.z[w].min()), success=tf is None, fall_t=tf,
                     # What realizes the recovery, before the MdlAMB error (Sec. 6.6.6).
                     peak_abduction_deg=r6(np.degrees(np.abs(R.qpos[a][:, [10, 13]]).max())),
                     feedforward_norm_limited_samples=None if path == "joint" else
                     int(np.sum(a & ((R.W[:, 20] == 1) | (R.W[:, 27] == 1)))),
                     torque_saturated_t=first_t(R, a & np.any(np.abs(R.tau) > TAU_LIM, axis=1)),
                     **(R.margin(DELAY + OBS_PUSH) if tf is None else {}))
        return R

    def sweep(self, path, sign=1):
        """1 N steps from 1 N to two past the first failure (cap PUSH_MAX)."""
        if (path, sign) not in self.sweeps:
            runs, first_fail, F = {}, None, 1
            while F <= PUSH_MAX and (first_fail is None or F <= first_fail + 2):
                runs[F] = self.push_run(path, F, sign)
                if first_fail is None and not runs[F].row["success"]:
                    first_fail = F
                F += 1
            last_ok = max([f for f in runs if runs[f].row["success"] and (first_fail is None or f < first_fail)],
                          default=None)
            self.sweeps[path, sign] = (runs, last_ok, first_fail)
        return self.sweeps[path, sign]

    def push(self):
        res = {"direction": self.direction, "figs": [], "thesis_table_6_5": THESIS["table_6_5"],
               "window": f"pulse {PULSE} s at {DELAY} s after activation; Table 6.5 metrics over "
                         f"[{DELAY + PULSE}, {DELAY + OBS_PUSH}] s (post-pulse, as the thesis), success judged after "
                         "the pulse; peak abduction, norm-limited samples and torque saturation from the pulse start "
                         "to the MdlAMB error"}
        for path in PATHS:
            runs, last_ok, first_fail = self.sweep(path)
            res[path] = {"table_6_5": [runs[F].row for F in sorted(runs)], "boundary_N": [last_ok, first_fail],
                         "recovering_above_first_failure": [F for F in runs if runs[F].row["success"] and first_fail
                                                            and F > first_fail]}
        # Side result, same procedure with the opposite sign: the boundary depends
        # on the direction. The canonical sign stays +t_h (thesis Fig. 6.18).
        res["minus_t_h"] = {"note": "side result: pulses along -t_h; the canonical sign is +t_h"}
        for path in PATHS:
            runs, last_ok, first_fail = self.sweep(path, -1)
            res["minus_t_h"][path] = {"boundary_N": [last_ok, first_fail],
                                      "table_6_5": [runs[F].row for F in sorted(runs)]}
        bw, bj = res["wbc"]["boundary_N"], res["joint"]["boundary_N"]
        bracketed = all(None not in b and b[1] - b[0] == 1 for b in (bw, bj))
        res["gate"] = gate("pushes: boundary bracketed, WBC >= joint space, failure plotted vs workspace bounds",
                           bracketed and bw[0] >= bj[0],
                           f"WBC recovers up to {bw[0]} N, fails from {bw[1]} N (thesis 11-12 N); joint space recovers up "
                           f"to {bj[0]} N, fails from {bj[1]} N; workspace plot in the workspace experiment; side "
                           f"result along -t_h: WBC {res['minus_t_h']['wbc']['boundary_N']} N, joint space "
                           f"{res['minus_t_h']['joint']['boundary_N']} N")
        runs, last_ok, first_fail = self.sweep("wbc")
        ok = [F for F in sorted(runs) if runs[F].row["success"] and (first_fail is None or F < first_fail)][-3:]
        bad = [F for F in sorted(runs) if not runs[F].row["success"]][:3]
        for Fs, name, kind in ((ok, "fig6_16_push_com_recovering.png", "recovers"),
                               (bad, "fig6_17_push_com_failing.png", "falls")):
            if Fs:
                f, ax = figure(1, len(Fs), 6.5)
                for a, F in zip(ax[0], Fs):
                    topdown(a, runs[F], DELAY - 0.5, DELAY + OBS_PUSH, mark=DELAY)
                    a.set_title(f"{F} N ({kind})")
                res["figs"].append(self.save(f, name))
        f, ax = figure(3, 1, 13, sharex=True)
        for k, F in enumerate(ok):
            R = runs[F]
            w = R.win(DELAY - 0.5, DELAY + 3.0)
            for a, v in zip(ax[:, 0], (R.vperp, R.L, R.Ld)):
                a.plot(R.t[w], v[w], color=CMAP(k / max(1, len(ok) - 1)), label=f"{F} N")
        for a, lab in zip(ax[:, 0], ("$v_\\perp$ along $t_h$ [m/s]", "$L$ [kg m$^2$/s]", "$\\dot L$ [N m]")):
            a.set_ylabel(lab)
            a.axvspan(DELAY, DELAY + PULSE, color="#e4e4e4", zorder=0)
        ax[0, 0].legend(title="recovering, WBC")
        ax[2, 0].set_xlabel("t from activation [s] (grey: pulse)")
        res["figs"].append(self.save(f, "fig6_18_push_dynamics.png"))
        f, ax = figure(1, 2, 6.5)
        for path in PATHS:
            rows, c = res[path]["table_6_5"], COL["on" if path == "wbc" else "off"]
            for a, key, lab in ((ax[0, 0], "max_cx", "max $|c_x|$ [m]"), (ax[0, 1], "max_L", "max $|L|$ [kg m$^2$/s]")):
                a.plot([r["F"] for r in rows], [r[key] for r in rows], color=c, lw=0.8)
                for succ, mk, txt in ((True, "o", "recovers"), (False, "x", "falls")):
                    sel = [r for r in rows if r["success"] == succ]
                    a.plot([r["F"] for r in sel], [r[key] for r in sel], mk, ms=4.5, color=c, label=f"{path}: {txt}")
                a.set(yscale="log", xlabel="pulse force [N], 0.1 s", ylabel=lab)
        ax[0, 0].legend(loc="upper left")
        res["figs"].append(self.save(f, "table6_05_push_sweep.png"))
        return res

    # -- g. workspace limit, Fig. 6.19-6.20 --------------------------------------
    def workspace(self):
        runs, last_ok, first_fail = self.sweep("wbc")
        if last_ok is None or first_fail is None:
            return {"gate": gate("workspace: failed push plotted against the tangential bounds", False,
                                 "push boundary not bracketed")}
        cases = ((max(1, last_ok - 2), "recovering"), (first_fail + 2, "failing"))
        res = {"choice": "last recovering - 2 N and first failing + 2 N, as the thesis' 9 and 14 N around 11-12 N",
               "xi_t_lim_derivation": "MuJoCo FK of the logged posture with only that swing leg's abduction moved to "
                                      "either end of its range (+-1.0472 rad; thigh, knee, base held): xi_t,lim = "
                                      "xi_t + rho_d (alpha(q_lim) - alpha(q)), alpha the foot-centre angle about the "
                                      "support line seen from the abduction anchor (the controller's hip point), "
                                      "rho_d the held radius. Dashed: at the sample before the pulse (thesis style); "
                                      "dotted: at every sample"}
        f19, ax19 = figure(2, 2, 10)
        f20, ax20 = figure(4, 2, 17)
        for c, (F, kind) in enumerate(cases):
            R = runs[F] = runs.get(F) or self.push_run("wbc", F)
            t_err = R.error["t"] if R.error else np.inf
            lim = {g: v[0].round(6).tolist() for g, v in xi_t_limits(R, [np.searchsorted(R.t, DELAY) - 1]).items()}
            w = R.win(DELAY - 0.5, min(DELAY + 3.0, t_err))
            lim_t = xi_t_limits(R, np.nonzero(w)[0])
            pre, post = R.win(DELAY - 1.0, DELAY), R.win(DELAY, min(DELAY + OBS_PUSH, t_err))
            info = {"F": F, "xi_t_lim_m": lim, "events_s_after_activation": events(R, lim, t_err)}
            for k, (leg, b) in enumerate(SWING):
                a = ax19[c, k]
                xt = xi_t_unwrapped(R, b)
                ref_plot(a, R.t[w], xt[w, 0], xt[w, 1], COL[leg], "$\\xi_t$")
                for j in (0, 1):
                    a.axhline(lim[leg][j], color=COL["ink"], ls="--", lw=0.8)
                    a.plot(R.t[w], lim_t[leg][:, j], color="#999999", ls=":", lw=0.9)
                a.set(title=f"{F} N {leg} ({kind})", ylabel="tangential $\\xi_t$ [m]")
                for r, (j, e, lab) in enumerate(((0, "normal", "normal $\\xi_n$ [m]"), (1, "radial", "radial $\\rho$ [m]"))):
                    a2 = ax20[2 * c + r, k]
                    ref_plot(a2, R.t[w], R.W[w, b + j], R.W[w, b + 3 + j], COL[leg])
                    a2.set(title=f"{F} N {leg} ({kind})", ylabel=lab)
                    info[f"{leg}_max_{e}_error_mm"] = {p: r6(1e3 * np.abs(R.W[m, b + j] - R.W[m, b + 3 + j]).max())
                                                       for p, m in (("pre", pre), ("post", post))}
                info[f"{leg}_xi_t_range_after_pulse_m"] = [r6(xt[post, 0].min()), r6(xt[post, 0].max())]   # unwrapped
            res[kind] = dict(R.row, **info)
        for a in list(ax19.ravel()) + list(ax20.ravel()):
            a.axvspan(DELAY, DELAY + PULSE, color="#e4e4e4", zorder=0)
        for a in (ax19[1, 0], ax19[1, 1], ax20[3, 0], ax20[3, 1]):
            a.set_xlabel("t from activation [s]")
        ax19[0, 0].legend(loc="lower left", title="dashed $\\pm\\xi_{t,lim}$ before the pulse, dotted instantaneous;\n"
                          "$\\xi_t$ unwrapped across its $\\pm\\pi\\rho^d$ cut", title_fontsize=6)
        ax20[0, 0].legend(loc="best")
        res["figs"] = [self.save(f19, "fig6_19_workspace_tangential.png"),
                       self.save(f20, "fig6_20_workspace_normal_radial.png")]
        # Diagnostics the thesis did not have (Sec. 6.7, limitation 4), failing run.
        R = runs[cases[1][0]]
        t_err = R.error["t"] if R.error else np.inf
        w = R.win(DELAY - 0.5, DELAY + 4.0)
        wa = w & (R.t <= t_err)
        f, ax = figure(4, 1, 15, sharex=True)
        for leg, q in (("FR", 10), ("RL", 13)):
            ax[0, 0].plot(R.t[w], np.degrees(R.qpos[w, q]), color=COL[leg], label=f"{leg} abduction")
        for v in np.degrees(ABD_RANGE):
            ax[0, 0].axhline(v, color=COL["ink"], ls="--", lw=0.8)
        ax[1, 0].plot(R.t[wa], R.W[wa, 34], color=COL["ink"])
        for leg, b in SWING:
            ax[2, 0].step(R.t[wa], R.W[wa, b + 6], color=COL[leg], where="post", label=f"{leg} feedforward")
        ax[2, 0].step(R.t[wa], R.W[wa, 1], color=COL["ink"], where="post", ls=":", label="QP status")
        ratio = np.max(np.abs(R.tau[wa]) / TAU_LIM, axis=1)
        ax[3, 0].plot(R.t[wa], ratio, color=COL["ink"], label=f"peak {ratio.max():.1f}")
        ax[3, 0].axhline(1.0, color=COL["ink"], ls="--", lw=0.8)
        for a, lab in zip(ax[:, 0], ("abduction [deg]", "joint-limit\ndistance [rad]", "status",
                                     "max $|\\tau|/\\tau_{lim}$")):
            a.set_ylabel(lab)
            a.axvspan(DELAY, DELAY + PULSE, color="#e4e4e4", zorder=0)
            if R.error:
                a.axvline(t_err, color=COL["off"], lw=0.9)
        ax[2, 0].set(yticks=[0, 1, 2])
        ax[3, 0].set(xlabel="t from activation [s]", ylim=(0, 2.5))
        ax[0, 0].legend(loc="best")
        ax[2, 0].legend(loc="best", title="0 applied, 1 norm-limited, 2 off", title_fontsize=6.5)
        ax[3, 0].legend(loc="upper left", title="MIT-law torque, clipped view", title_fontsize=6.5)
        ax[0, 0].set_title(f"{cases[1][0]} N push: realizability diagnostics (orange line: MdlAMB error)")
        res["figs"].append(self.save(f, "fig6_19b_workspace_diagnostics.png"))
        ev = res["failing"]["events_s_after_activation"]
        # Facts behind the Sec. 6.6.6 verdict, from the whole WBC sweep.
        rows = [runs[F].row for F in sorted(runs)]
        res["sweep_facts"] = {"abduction_stop_deg": r6(np.degrees(ABD_RANGE[1])),
                              "recovering_peak_abduction_deg": {r["F"]: r["peak_abduction_deg"] for r in rows
                                                                if r["success"]},
                              "first_force_norm_limited_N": min([r["F"] for r in rows
                                                                 if r["feedforward_norm_limited_samples"]], default=None)}
        # The pair shows the mechanism only if one recovers and the other fails.
        res["gate"] = gate("workspace: failed push plotted against the tangential bounds",
                           res["recovering"]["success"] and not res["failing"]["success"],
                           f"{cases[0][0]} N and {cases[1][0]} N plotted; failing run, s after activation: feedforward "
                           f"norm-limited {ev['feedforward_norm_limited']}, joint limit {ev['joint_limit_within_0.01rad']},"
                           f" xi_t past bound FR/RL {ev['FR_xi_t_beyond_bound']}/{ev['RL_xi_t_beyond_bound']}, torque "
                           f"saturated {ev['torque_saturated']}, QP status != 0 {ev['qp_status_nonzero']}, theta "
                           f"branch-cut jump {ev['theta_wrap']}, fall {ev['fall']}, MdlAMB error {ev['amb_error']}; "
                           f"recovering pushes' peak |abduction| {res['sweep_facts']['recovering_peak_abduction_deg']} "
                           f"deg (stop 60), feedforward first norm-limited at "
                           f"{res['sweep_facts']['first_force_norm_limited_N']} N")
        return res


def xi_t_limits(R, idx):
    """(lower, upper) xi_t,lim of FR and RL at the samples idx."""
    out = {}
    for leg, b in SWING:
        li = LEGS.index(leg)
        qi = 7 + 3 * li
        vals, xt = np.empty((len(idx), 2)), xi_t_unwrapped(R, b)[:, 0]
        for n, i in enumerate(idx):
            d = []
            for q0 in (R.qpos[i, qi], *ABD_RANGE):
                _D.qpos[:] = R.qpos[i]
                _D.qpos[qi] = q0
                mujoco.mj_kinematics(_M, _D)
                d.append(_D.geom_xpos[_FOOT[li]] - _D.xpos[_HIP[li]])
            s = R.s_line[i]
            er = d[0] - d[0].dot(s) * s
            er /= np.linalg.norm(er)
            et = np.cross(s, er)     # (6.16) orientation: e_t = e_n x e_r
            vals[n] = xt[i] + R.W[i, b + 4] * np.array([np.arctan2(x.dot(et), x.dot(er)) for x in d[1:]])
        out[leg] = np.sort(vals, axis=1)
    return out


def xi_t_unwrapped(R, b):
    """Measured and desired xi_t = rho_d * angle (6.16), the angle's +-pi cut
    unwrapped from the hand-off to the MdlAMB error."""
    out = R.W[:, [b + 2, b + 5]].copy()
    rd = R.W[R.live, b + 4][:, None]
    out[R.live] = np.unwrap(out[R.live] / rd, axis=0) * rd
    return out


def events(R, lim, t_err):
    """First time after the pulse start of each realizability event (before
    the MdlAMB error, after which its log is frozen)."""
    a = (R.t >= DELAY) & (R.t < t_err)
    ev = {"feedforward_norm_limited": first_t(R, a & ((R.W[:, 20] == 1) | (R.W[:, 27] == 1))),
          "joint_limit_within_0.01rad": first_t(R, a & (R.W[:, 34] < 0.01)),
          "torque_saturated": first_t(R, a & np.any(np.abs(R.tau) > TAU_LIM, axis=1)),
          "qp_status_nonzero": first_t(R, a & (R.W[:, 1] != 0))}
    for leg, b in SWING:
        xt = xi_t_unwrapped(R, b)[:, 0]
        ev[f"{leg}_xi_t_beyond_bound"] = first_t(R, a & ((xt < lim[leg][0]) | (xt > lim[leg][1])))
        # Its atan2 cut: the foot passed straight up from the hip (a 2 pi rho_d jump).
        ev[f"{leg}_xi_t_wrap"] = first_t(R, a & (np.abs(np.diff(R.W[:, b + 2], prepend=0.0)) > np.pi * R.W[:, b + 4]))
    ev.update(theta_wrap=R.wrap_t, fall=R.fall(DELAY + PULSE), amb_error=R.error)
    return ev


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", nargs="+", choices=EXPERIMENTS, help="experiments to run (default: all)")
    ap.add_argument("--out", help="root for logs, results and figures (default: runs/amb_wbc, reports/amb_wbc)")
    a = ap.parse_args()
    out = a.out and os.path.abspath(a.out)
    logs = os.path.join(out, "runs") if out else os.path.join(REPO, "runs", "amb_wbc")
    rep = out or os.path.join(REPO, "reports", "amb_wbc")
    os.makedirs(rep, exist_ok=True)
    only, t0 = a.only or EXPERIMENTS, time.time()
    E = Experiments(logs, rep)
    E.baseline(rerun="ablation" in only)
    path = os.path.join(rep, "results.json")
    res = json.load(open(path)) if os.path.exists(path) else {}
    for name in EXPERIMENTS:
        if name in only:
            print(f"== {name}")
            res[name] = getattr(E, name)()
    res["meta"] = {"handoff_s": E.th, "push_direction": E.direction, "experiments_run": only,
                   "fall_criteria": {"ablation": f"torso height < {FALL_Z} m",
                                     "others": f"torso height < {FALL_Z} m or |roll|/|pitch| > 45 deg"},
                   "wall_time_s": round(time.time() - t0, 1), "run_wall_times_s": E.wall,
                   "log_period_ms": LOG_PERIOD, "script": ["0 s", f"{T_B} b"],
                   "closing_note": "computeTick timing varies between identical runs; the joint-space path prints "
                                   "no computeTick line (tick_mean_us null there)"}
    res["gates"] = [res[n][k] for n in EXPERIMENTS for k in ("gate_principal", "gate") if k in res.get(n, {})]
    with open(path, "w") as f:
        json.dump(res, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print(f"\n{'gate':<80} result\n" + "-" * 88)
    for g in res["gates"]:
        print(f"{g['gate']:<80} {g['result']}\n    {g['numbers']}")
    print(f"\nwall time {res['meta']['wall_time_s']} s; results in {path}")
    # A --only run into a fresh --out has no full results.json to report on.
    if all(n in res for n in EXPERIMENTS):
        build_report.build(rep)


if __name__ == "__main__":
    main()
