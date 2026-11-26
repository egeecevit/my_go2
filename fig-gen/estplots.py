#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Figure library for the state estimator, working on any Supervisor .mat log.

Self-contained: its own .mat loader and quaternion helpers; the only project
knowledge is the layout of the logged variables (see COLS). Every figure plots
errors against simulator truth, one quantity per axis, with fixed y-ranges so
runs and configurations can be compared side by side:

  fig_story        one log  : velocity error per gait cycle, zoom, attitude, bias, consistency
  fig_phase        one log  : errors and foot motion folded on the gait phase
  fig_convergence  one log  : planted tilt error against the third-order loop model
  fig_before_after two logs : the same quantities, baseline grey against final colour
  fig_dumbbell     metrics  : scalar metrics across speeds, baseline -> final
  fig_explain_*    one log  : the mechanisms (rolling foot, compliant touchdown, heading)

Colours are a validated categorical set: x blue, y orange, z aqua; grey is the
de-emphasised comparison series; ink is truth or a model.
"""

import os
import warnings

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402
from scipy.io import loadmat  # noqa: E402

DEG = 180.0 / np.pi
FOOT_RADIUS = 0.022   # go2.xml foot sphere radius = posvelestimator.foot_ground_height

# --------------------------------------------------------------------------- style
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
AXIS = (BLUE, ORANGE, AQUA)          # x, y, z: always in this order
GREY, INK, MUTED, GRID, SURFACE = "#898781", "#0b0b0b", "#52514e", "#e1e0d9", "#fcfcfb"
LW = 1.2
BAND_ALPHA = 0.20
STANCE_ALPHA = 0.12
FIG_W = 7.0


def setup_style():
    plt.rcParams.update({
        "font.size": 8, "axes.titlesize": 8.5, "axes.titleweight": "semibold", "axes.titlelocation": "left",
        "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
        "legend.frameon": False, "lines.linewidth": LW, "axes.grid": True, "grid.color": GRID,
        "grid.linewidth": 0.6, "grid.linestyle": "-", "axes.edgecolor": MUTED, "axes.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "xtick.color": MUTED, "ytick.color": MUTED,
        "axes.labelcolor": MUTED, "text.color": INK, "axes.titlecolor": INK,
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "xtick.direction": "out", "ytick.direction": "out", "pdf.fonttype": 42, "ps.fonttype": 42,
        "savefig.dpi": 150, "figure.constrained_layout.use": True,
    })


def save(fig, stem):
    """PNG at 150 dpi plus a vector PDF; returns the paths."""
    os.makedirs(os.path.dirname(os.path.abspath(stem)), exist_ok=True)
    paths = [stem + ".png", stem + ".pdf"]
    for p in paths:
        fig.savefig(p)
    plt.close(fig)
    return paths


def headline(ax, text, loc="upper left"):
    """One number per panel, in ink, no box."""
    x, ha = (0.02, "left") if "left" in loc else (0.98, "right")
    y, va = (0.96, "top") if "upper" in loc else (0.5, "center") if "center" in loc else (0.04, "bottom")
    ax.text(x, y, text, transform=ax.transAxes, fontsize=7, color=INK, ha=ha, va=va, in_layout=False)


def zero_line(ax):
    ax.axhline(0.0, color=MUTED, linewidth=0.6, zorder=1)


def clip_note(y, lo, hi, unit, nd=0):
    """Clip to the fixed range and say what was cut, so a fixed axis never hides a number."""
    yc = np.clip(y, lo, hi)
    mn, mx = np.nanmin(y), np.nanmax(y)
    notes = []
    if mn < lo:
        notes.append(f"min {mn:.{nd}f} {unit} off scale")
    if mx > hi:
        notes.append(f"max {mx:.{nd}f} {unit} off scale")
    return yc, ("; ".join(notes) if notes else None)


def block_mean(t, x, n):
    """Mean over blocks of n samples (never strided decimation: that aliases the gait ripple)."""
    m = (len(t) // n) * n
    if m == 0:
        return t, x
    tb = t[:m].reshape(-1, n).mean(1)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        xb = np.nanmean(x[:m].reshape(-1, n, *x.shape[1:]), axis=1)
    return tb, xb


# --------------------------------------------------------------------------- math
def quat_rotate(q, v, inverse=False):
    """Rotate (N,3) vectors by (N,4) w-first quaternions (body->world); inverse for world->body."""
    v = np.broadcast_to(np.asarray(v, dtype=float), (q.shape[0], 3))
    w, u = q[:, 0:1], q[:, 1:4]
    if inverse:
        u = -u
    uxv = np.cross(u, v)
    return v + 2.0 * (w * uxv + np.cross(u, uxv))


def heading_of(q):
    """Heading of body +x about the world vertical, from (N,4) quaternions [rad]."""
    hx = quat_rotate(q, np.array([1.0, 0.0, 0.0]))
    return np.arctan2(hx[:, 1], hx[:, 0])


def tilt_heading_error(q_true, q_est):
    """What gravity sees and what it does not: angle between the true and estimated
    up vectors, and the heading difference about the vertical [rad]."""
    up_t = quat_rotate(q_true, np.array([0.0, 0.0, 1.0]), inverse=True)
    up_e = quat_rotate(q_est, np.array([0.0, 0.0, 1.0]), inverse=True)
    tilt = np.arctan2(np.linalg.norm(np.cross(up_t, up_e), axis=1), np.sum(up_t * up_e, axis=1))
    heading = np.arctan2(np.sin(heading_of(q_est) - heading_of(q_true)), np.cos(heading_of(q_est) - heading_of(q_true)))
    return tilt, heading


# --------------------------------------------------------------------------- log access
COLS = {
    "qpos_pos": ("MdlSimDriver_qpos", 0, 3), "qpos_quat": ("MdlSimDriver_qpos", 3, 7), "qpos_joint": ("MdlSimDriver_qpos", 7, 19),
    "qvel_lin": ("MdlSimDriver_qvel", 0, 3), "qvel_ang": ("MdlSimDriver_qvel", 3, 6), "qvel_joint": ("MdlSimDriver_qvel", 6, 18),
    "os_quat": ("MdlOrientationEstimator_state", 0, 4), "os_rpy": ("MdlOrientationEstimator_state", 4, 7),
    "os_omega": ("MdlOrientationEstimator_state", 7, 10), "os_bias": ("MdlOrientationEstimator_state", 13, 16),
    "oe_tilt": ("MdlOrientationEstimator_error", 3, 4), "oe_heading": ("MdlOrientationEstimator_error", 4, 5),
    "oe_omega": ("MdlOrientationEstimator_error", 5, 8), "oe_bias": ("MdlOrientationEstimator_error", 8, 11),
    "of_injbias": ("MdlOrientationEstimator_filter", 7, 10),
    "ps_pos": ("MdlPosVelEstimator_state", 0, 3), "ps_vel": ("MdlPosVelEstimator_state", 3, 6), "ps_vbody": ("MdlPosVelEstimator_state", 6, 9),
    "pe_pos": ("MdlPosVelEstimator_error", 0, 3), "pe_vel": ("MdlPosVelEstimator_error", 3, 6), "pe_path": ("MdlPosVelEstimator_error", 7, 8),
    "cov_pos": ("MdlPosVelEstimator_cov", 0, 3), "cov_vel": ("MdlPosVelEstimator_cov", 3, 6),
    "con": ("MdlPosVelEstimator_contacts", 0, 4), "ft": ("MdlPosVelEstimator_foottruth", 0, 12),
}


def load_mat(path):
    """Supervisor matlab log -> (absolute sim time [s], {var: (N, width)})."""
    raw = loadmat(path, squeeze_me=False, struct_as_record=True)
    rec = None
    for k, v in raw.items():
        if not k.startswith("__") and getattr(v, "dtype", None) is not None and v.dtype.names and "time" in v.dtype.names:
            rec = v.ravel()
            break
    if rec is None:
        raise ValueError(f"{path}: no struct with a time field; not a Supervisor log")
    data = {n: np.concatenate([np.atleast_2d(x) for x in rec[n]], axis=1).T.astype(float) for n in rec.dtype.names}
    t = data.pop("time")[:, 0]
    offset = 0.0
    try:
        offset = float(np.asarray(raw["params"]["time_offset"][0, 0]).ravel()[0]) * 1e-6
    except Exception:
        pass
    return t - t[0] + offset, data


class Log:
    """One log with lazily derived estimator quantities; missing variables give None."""

    def __init__(self, path, label=None):
        self.path = path
        self.label = label or os.path.splitext(os.path.basename(path))[0]
        self.t, self.data = load_mat(path)
        self.dt = float(np.median(np.diff(self.t)))
        self._cache = {}

    def col(self, key):
        var, a, b = COLS[key]
        arr = self.data.get(var)
        return None if arr is None else arr[:, a:b]

    def has(self, key):
        return COLS[key][0] in self.data

    # -- derived -----------------------------------------------------------
    def q_true(self):
        return self.col("qpos_quat")

    def active(self):
        """PosVel log buffers freeze while the module is inactive (sitting): drop repeats."""
        if "active" not in self._cache:
            src = self.col("pe_pos") if self.has("pe_pos") else self.col("ps_pos")
            a = np.ones(len(self.t), bool)
            if src is not None:
                a[1:] = np.any(np.diff(src, axis=0) != 0.0, axis=1)
            self._cache["active"] = a
        return self._cache["active"]

    def vel_err_body(self):
        """Body velocity error, each velocity in its own body frame [m/s]."""
        if self.has("pe_vel"):
            return self.col("pe_vel")
        if self.has("ps_vbody") and self.has("qvel_lin"):
            return self.col("ps_vbody") - quat_rotate(self.q_true(), self.col("qvel_lin"), inverse=True)
        return None

    def vel_err_world(self):
        """World-frame velocity error [m/s]: what a naive comparison shows (heading drift leaks in)."""
        if self.has("ps_vel") and self.has("qvel_lin"):
            return self.col("ps_vel") - self.col("qvel_lin")
        return None

    def pos_err_world(self):
        if self.has("pe_pos"):
            return self.col("pe_pos")
        if self.has("ps_pos") and self.has("qpos_pos"):
            return self.col("ps_pos") - self.col("qpos_pos")
        return None

    def tilt_heading(self):
        """Tilt error and heading error [rad]; in-module (skew-free) when logged."""
        if "th" not in self._cache:
            if self.has("oe_tilt"):
                self._cache["th"] = (self.col("oe_tilt")[:, 0], self.col("oe_heading")[:, 0])
            elif self.has("os_quat") and self.has("qpos_quat"):
                self._cache["th"] = tilt_heading_error(self.q_true(), self.col("os_quat"))
            else:
                self._cache["th"] = (None, None)
        return self._cache["th"]

    def bias_est(self):
        return self.col("os_bias")

    def bias_inj(self):
        if self.has("oe_bias"):
            return self.col("os_bias") - self.col("oe_bias")
        return self.col("of_injbias") if self.has("of_injbias") else None

    def bias_err(self):
        if self.has("oe_bias"):
            return self.col("oe_bias")
        inj = self.bias_inj()
        return None if inj is None else self.col("os_bias") - inj

    def omega_err(self):
        if self.has("oe_omega"):
            return self.col("oe_omega")
        if self.has("os_omega") and self.has("qvel_ang"):
            return self.col("os_omega") - self.col("qvel_ang")
        return None

    def expected_heading_slope(self, mask):
        """Heading drift the unlearned gyro bias predicts [rad/s]: bias error projected on the vertical."""
        be = self.bias_err()
        if be is None:
            return None
        up = quat_rotate(self.q_true(), np.array([0.0, 0.0, 1.0]), inverse=True)
        return float(-np.nanmean(np.sum(be[mask] * up[mask], axis=1)))

    def heading_leak_xy(self, mask):
        """Integral of (R(psi) - I) v_true: the position error an unobservable heading error must cause."""
        _, psi = self.tilt_heading()
        vt = self.col("qvel_lin")
        if psi is None or vt is None:
            return None
        dvx = (np.cos(psi) - 1.0) * vt[:, 0] - np.sin(psi) * vt[:, 1]
        dvy = np.sin(psi) * vt[:, 0] + (np.cos(psi) - 1.0) * vt[:, 1]
        return np.column_stack([np.cumsum(dvx * mask * self.dt), np.cumsum(dvy * mask * self.dt)])

    def foot_velocity_body(self, leg):
        """True foot-centre velocity of a leg in the body frame [m/s] (truth only, skew-free)."""
        ft = self.col("ft")
        if ft is None:
            return None
        vf = np.gradient(ft[:, 3 * leg:3 * leg + 3], self.dt, axis=0)
        return quat_rotate(self.q_true(), vf, inverse=True)

    def rolling_prediction_body(self, leg, r=FOOT_RADIUS):
        """r (omega_calf x z) in the body frame: how fast the sphere centre rolls with the calf.
        MuJoCo joint columns are already the kinematics' internal sign convention."""
        qj, qdj, wb = self.col("qpos_joint"), self.col("qvel_joint"), self.col("qvel_ang")
        if qj is None or qdj is None or wb is None:
            return None
        q1 = qj[:, 3 * leg]
        pitch_rate = qdj[:, 3 * leg + 1] + qdj[:, 3 * leg + 2]
        omega_rel = np.column_stack([qdj[:, 3 * leg], pitch_rate * np.cos(q1), pitch_rate * np.sin(q1)])
        omega_calf_w = quat_rotate(self.q_true(), wb + omega_rel)
        v_roll_w = r * np.cross(omega_calf_w, np.array([0.0, 0.0, 1.0]))
        return quat_rotate(self.q_true(), v_roll_w, inverse=True)


# --------------------------------------------------------------------------- gait
def mask_runs(mask):
    """[start, end) index pairs of the True runs of a boolean mask."""
    m = np.concatenate([[False], mask.astype(bool), [False]])
    d = np.diff(m.astype(int))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def gait_info(t, con, lo, hi):
    """Scheduled gait structure from the trust signal, or None if the log has no gait in [lo, hi].

    A stance is a run of trust > 0: both ramp configurations start and end exactly at the
    scheduled touchdown and liftoff, so the > 0 edges are the schedule. The leg FL (index 0)
    is the reference; FL and RR move together, FR and RL half a period later."""
    if con is None:
        return None
    runs = [(a, b) for a, b in mask_runs(con[:, 0] > 0) if lo <= t[a] < hi and b < len(t)]
    if len(runs) < 10:
        return None
    dur = np.array([t[b - 1] - t[a] for a, b in runs]) + (t[1] - t[0])
    med = np.median(dur)
    runs = [(a, b) for (a, b), d in zip(runs, dur) if abs(d - med) < 0.1 * med]
    rise = np.array([a for a, _ in runs])
    if len(rise) < 10:
        return None
    period = float(np.median(np.diff(t[rise])))
    if not 0.1 < period < 2.0:
        return None
    dt = float(np.median(np.diff(t)))
    n_cyc = int(round(period / dt))
    n_st = int(round(med / dt))
    # Ramp fractions, measured: first and last sample at full trust within the median stance.
    up, down = [], []
    for a, b in runs[:50]:
        tr = con[a:b, 0]
        full = np.flatnonzero(tr >= 0.999)
        if len(full):
            up.append(full[0] / n_st)
            down.append((len(tr) - 1 - full[-1]) / n_st)
    return dict(period=period, stance=float(med), n_cyc=n_cyc, n_st=n_st, rise=rise,
                ramp_up=float(np.median(up)) if up else float("nan"),
                ramp_down=float(np.median(down)) if down else float("nan"))


def cycle_windows(rise, n_cyc, n):
    seg = rise[:, None] + np.arange(n_cyc)[None, :]
    return seg[(seg[:, -1] < n)]


def fixed_windows(t, lo, hi, block_s):
    """Non-overlapping blocks for logs without a gait (standing, sitting)."""
    dt = float(np.median(np.diff(t)))
    n = int(round(block_s / dt))
    i0 = np.flatnonzero((t >= lo) & (t <= hi - block_s))
    if len(i0) == 0:
        return np.zeros((0, n), int)
    starts = np.arange(i0[0], i0[-1] + 1, n)
    return cycle_windows(starts, n, len(t))


def cycle_stats(t, x, seg):
    """Per-window mean and within-window sd of x (NaN-aware); windows < 80 % finite are dropped."""
    if len(seg) == 0:
        return np.array([]), np.array([]), np.array([])
    blk = x[seg]
    ok = np.mean(np.isfinite(blk), axis=1) >= 0.8
    tm = t[seg].mean(1)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return tm[ok], np.nanmean(blk[ok], axis=1), np.nanstd(blk[ok], axis=1)


def fold(x, seg):
    """Mean and sd across windows at each in-window sample: x folded on the cycle phase."""
    blk = x[seg]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmean(blk, axis=0), np.nanstd(blk, axis=0)


def trust_ramp(phase, w_up, w_down):
    """MdlPosVelEstimator::trustFromPhase for a stance phase in [0, 1]."""
    tr = np.ones_like(phase)
    if w_up > 0:
        tr = np.where(phase < w_up, phase / w_up, tr)
    if w_down > 0:
        tr = np.where(phase > 1.0 - w_down, (1.0 - phase) / w_down, tr)
    return np.clip(tr, 0.0, 1.0)


def default_window(L):
    """Steady segment of a log: the gait minus its ramp-up when there is one, else the active span."""
    act = L.active()
    ia = np.flatnonzero(act)
    if len(ia) == 0:
        return float(L.t[0]), float(L.t[-1])
    lo, hi = L.t[ia[0]], L.t[ia[-1]]
    g = gait_info(L.t, L.col("con"), lo, hi)
    if g is not None:
        # MdlTrot ramps the command up over 2 s after the first stance and down over 1 s
        # before the sit; neither belongs to the steady gait.
        return float(L.t[g["rise"][0]] + 3.0), float(L.t[g["rise"][-1]] - 1.0)
    return float(lo + 1.0), float(hi - 0.5)


def shade_stance(ax, t, con, lo, hi, label="FL+RR stance"):
    first = True
    for a, b in mask_runs(con[:, 0] > 0):
        if t[b - 1] < lo or t[a] > hi:
            continue
        ax.axvspan(t[a], t[b - 1], color=GREY, alpha=STANCE_ALPHA, linewidth=0, label=label if first else None)
        first = False


# --------------------------------------------------------------------------- figures
VEL_LIM = 10.0      # mm/s, per-cycle mean panels
ZOOM_LIM = 15.0     # mm/s, raw zoom
TILT_LIM = 2.0      # deg
BIAS_LIM = 8.0      # mrad/s


def _velocity_rows(L, lo, hi):
    """Per-cycle (or per-block) statistics of the body-velocity error, with the windows used."""
    w = (L.t >= lo) & (L.t <= hi)
    ve = L.vel_err_body()
    if ve is None:
        return None
    ve = np.where((w & L.active())[:, None], ve, np.nan)
    g = gait_info(L.t, L.col("con"), lo, hi)
    if g is not None:
        seg = cycle_windows(g["rise"], g["n_cyc"], len(L.t))
        kind = f"per gait cycle ({g['period']*1e3:.0f} ms)"
    else:
        seg = fixed_windows(L.t, lo, hi, 0.5)
        kind = "per 0.5 s block"
    return dict(ve=ve, seg=seg, gait=g, kind=kind, w=w)


def fig_story(L, lo, hi, stem, title=None):
    """One log, the whole story: is the velocity error zero-mean and bounded, are roll/pitch
    drift-free, is the gyro bias learned, is the covariance honest."""
    t, deg = L.t, DEG
    V = _velocity_rows(L, lo, hi)
    act = L.active()
    w = (t >= lo) & (t <= hi)
    # Fixed spacing rather than constrained layout: the 5x3 grid of short panels with
    # left-aligned titles makes the automatic layout give up, and a fixed grid also keeps
    # every run's figure pixel-identical in layout.
    fig = plt.figure(figsize=(FIG_W, 10.6))
    fig.set_layout_engine("none")
    gs = GridSpec(5, 3, figure=fig, left=0.085, right=0.985, bottom=0.05, top=0.955, hspace=0.62, wspace=0.38)
    fig.suptitle((title or L.label) + f"   window {lo:.1f}-{hi:.1f} s   row 1: mean ± sd {V['kind'] if V else ''}",
                 fontsize=8.5, color=MUTED, x=0.01, y=0.992, ha="left")

    # Row 1: velocity error per cycle, one axis per component.
    axv = [fig.add_subplot(gs[0, i]) for i in range(3)]
    for i, (ax, lab) in enumerate(zip(axv, "xyz")):
        if V is not None and len(V["seg"]):
            tm, mean, sd = cycle_stats(t, V["ve"][:, i], V["seg"])
            m_mm, note = clip_note(mean * 1e3, -VEL_LIM, VEL_LIM, "mm/s")
            ax.fill_between(tm, np.clip((mean - sd) * 1e3, -VEL_LIM, VEL_LIM), np.clip((mean + sd) * 1e3, -VEL_LIM, VEL_LIM),
                            color=AXIS[i], alpha=BAND_ALPHA, linewidth=0, label="± sd within cycle")
            ax.plot(tm, m_mm, color=AXIS[i], label=f"cycle mean")
            e = V["ve"][w & act, i]
            headline(ax, f"mean {np.nanmean(e)*1e3:+.1f}  rms {np.sqrt(np.nanmean(e**2))*1e3:.1f} mm/s" + (f"\n{note}" if note else ""))
            if i == 0:
                ax.legend(loc="lower left", ncol=2)
        ax.set_ylim(-VEL_LIM, VEL_LIM)
        zero_line(ax)
        ax.set_title(f"v{lab} error (body)")
        ax.set_ylabel("mm/s" if i == 0 else "")
        ax.set_xlim(lo, hi)
    for ax in axv:
        ax.tick_params(labelbottom=False)

    # Row 2: raw zoom over two gait cycles mid-window, stance shaded.
    axz = fig.add_subplot(gs[1, :])
    if V is not None:
        g = V["gait"]
        if g is not None:
            mid = g["rise"][len(g["rise"]) // 2]
            z0 = t[mid]
        else:
            z0 = 0.5 * (lo + hi)
        z1 = z0 + 0.6
        zm = (t >= z0) & (t <= z1)
        if g is not None:
            shade_stance(axz, t, L.col("con"), z0, z1)
        for i, lab in enumerate("xyz"):
            y, note = clip_note(V["ve"][zm, i] * 1e3, -ZOOM_LIM, ZOOM_LIM, "mm/s")
            axz.plot(t[zm], y, color=AXIS[i], linewidth=0.9, label=f"v{lab}")
        axz.set_xlim(z0, z1)
        axz.legend(loc="upper right", ncol=4)
        headline(axz, "raw samples; shaded = FL+RR stance, FR+RL between" if g is not None else "raw samples", loc="lower left")
    axz.set_ylim(-ZOOM_LIM, ZOOM_LIM)
    zero_line(axz)
    axz.set_title("Zoom: velocity error through two gait cycles" if (V and V["gait"]) else "Zoom: 0.6 s of raw velocity error")
    axz.set_ylabel("mm/s")
    axz.set_xlabel("sim time [s]")

    # Row 3: tilt, heading, xy position error (raw vs heading-aligned).
    tilt, head = L.tilt_heading()
    ax = fig.add_subplot(gs[2, 0])
    if tilt is not None:
        tb, yb = block_mean(t[w], tilt[w] * deg, max(1, int(round(0.05 / L.dt))))
        yc, note = clip_note(yb, 0.0, TILT_LIM, "deg", 2)
        ax.plot(tb, yc, color=BLUE)
        headline(ax, f"rms {np.sqrt(np.mean((tilt[w]*deg)**2)):.2f}  max {np.max(tilt[w]*deg):.2f} deg" + (f"\n{note}" if note else ""))
    ax.set_ylim(0, TILT_LIM)
    ax.set_xlim(lo, hi)
    ax.set_title("Tilt error (gravity-observable)")
    ax.set_ylabel("deg")
    ax = fig.add_subplot(gs[2, 1])
    if head is not None:
        hd = np.unwrap(head[w]) * deg
        tb, yb = block_mean(t[w], hd, max(1, int(round(0.05 / L.dt))))
        ax.plot(tb, yb, color=BLUE, label="heading error")
        slope = L.expected_heading_slope(w)
        if slope is not None:
            ax.plot([lo, hi], [hd[0], hd[0] + slope * deg * (hi - lo)], color=INK, linewidth=0.8, linestyle="--", label="unlearned z-bias × t")
            meas = np.polyfit(t[w], hd, 1)[0]
            headline(ax, f"{meas*60:.2f} deg/min measured\n{slope*deg*60:.2f} expected from bias")
        ax.legend(loc="lower right")
        lim = max(2.0, np.ceil(np.max(np.abs(hd)) + 0.5))
        ax.set_ylim(-lim if hd.min() < -0.5 else -0.5, lim)
    ax.set_xlim(lo, hi)
    ax.set_title("Heading error (unobservable)")
    ax.set_ylabel("deg")
    ax = fig.add_subplot(gs[2, 2])
    pe = L.pos_err_world()
    if pe is not None:
        mask = w & act
        raw = np.where(mask, np.linalg.norm(pe[:, :2], axis=1), np.nan)
        leak = L.heading_leak_xy(mask)
        tb, rb = block_mean(t, raw, max(1, int(round(0.1 / L.dt))))
        ax.plot(tb, rb, color=GREY, label="raw (world frame)")
        if leak is not None:
            al = np.where(mask, np.linalg.norm(pe[:, :2] - leak, axis=1), np.nan)
            tb, ab = block_mean(t, al, max(1, int(round(0.1 / L.dt))))
            ax.plot(tb, ab, color=BLUE, label="heading-aligned")
        path = L.col("pe_path")
        last = np.flatnonzero(mask)[-1] if mask.any() else -1
        if path is not None and last >= 0 and path[last, 0] > 0.25:
            txt = f"drift {100*raw[last]/path[last,0]:.1f} %"
            if leak is not None:
                txt += f" → aligned {100*al[last]/path[last,0]:.1f} %"
            headline(ax, txt.replace(" → ", "\n") + f" of {path[last,0]:.0f} m")
        ax.legend(loc="center left")
        ax.set_ylim(0, max(0.2, np.ceil(np.nanmax(rb) * 4) / 4))
    ax.set_xlim(lo, hi)
    ax.set_title("xy position error")
    ax.set_ylabel("m")
    ax.legend(loc="center left")

    # Row 4: gyro bias estimate vs injected, one axis per component.
    best, binj = L.bias_est(), L.bias_inj()
    for i, lab in enumerate("xyz"):
        ax = fig.add_subplot(gs[3, i])
        if best is not None:
            run = act | (t >= lo)   # the bias story includes the transitions: whole active run
            sel = t >= t[np.flatnonzero(act)[0]] if act.any() else np.ones_like(t, bool)
            tb, yb = block_mean(t[sel], best[sel, i] * 1e3, max(1, int(round(0.05 / L.dt))))
            yc, note = clip_note(yb, -BIAS_LIM, BIAS_LIM, "mrad/s", 1)
            ax.plot(tb, yc, color=AXIS[i], label="estimate")
            if binj is not None:
                tb2, yb2 = block_mean(t[sel], binj[sel, i] * 1e3, max(1, int(round(0.05 / L.dt))))
                ax.plot(tb2, np.clip(yb2, -BIAS_LIM, BIAS_LIM), color=INK, linewidth=0.8, linestyle="--", label="injected")
                be = (best[:, i] - binj[:, i]) * 1e3
                headline(ax, f"err {np.mean(be[w]):+.2f}, peak {np.max(np.abs(be[sel])):.2f}" + (" mrad/s" if i == 0 else "") + (f"\n{note}" if note else ""))
            ax.axvspan(lo, hi, color=GREY, alpha=0.08, linewidth=0)
            if i == 0:
                ax.legend(loc="lower right", ncol=1)
        ax.set_ylim(-BIAS_LIM, BIAS_LIM)
        zero_line(ax)
        ax.set_title(f"Gyro bias {lab}" + (", whole run" if i == 0 else ""))
        ax.set_ylabel("mrad/s" if i == 0 else "")

    # Row 5: z error with 3 sigma, normalised-error histogram, angular-velocity error per cycle.
    ax = fig.add_subplot(gs[4, 0])
    if pe is not None:
        z = np.where(w & act, pe[:, 2] * 1e3, np.nan)
        tb, zb = block_mean(t, z, max(1, int(round(0.05 / L.dt))))
        cov = L.col("cov_pos")
        if cov is not None:
            s3 = 3.0 * np.sqrt(np.maximum(cov[:, 2], 0)) * 1e3
            tb3, s3b = block_mean(t, np.where(w, s3, np.nan), max(1, int(round(0.05 / L.dt))))
            if np.nanmax(s3b) <= 5.0:
                ax.fill_between(tb3, -s3b, s3b, color=GREY, alpha=0.25, linewidth=0, label="± 3σ")
            else:
                ax.plot([], [], color=GREY, alpha=0.5, label=f"± 3σ = {np.nanmean(s3b):.0f} mm, off scale")
        ax.plot(tb, np.clip(zb, -5, 5), color=AQUA, label="z error")
        headline(ax, f"mean {np.nanmean(z):+.1f}  rms {np.sqrt(np.nanmean(z**2)):.1f} mm")
        ax.legend(loc="lower left")
    ax.set_ylim(-5, 5)
    zero_line(ax)
    ax.set_xlim(lo, hi)
    ax.set_title("Height error")
    ax.set_ylabel("mm")
    ax.set_xlabel("sim time [s]")
    ax = fig.add_subplot(gs[4, 1])
    cov = L.col("cov_vel")
    ve = L.vel_err_body()
    if cov is not None and ve is not None:
        mask = w & act
        ew = quat_rotate(L.q_true()[mask], ve[mask])          # covariance is world-frame
        sig = np.sqrt(np.maximum(cov[mask], 1e-12))
        bins = np.linspace(-4, 4, 81)
        for i, lab in enumerate("xyz"):
            ax.hist(ew[:, i] / sig[:, i], bins=bins, histtype="step", density=True, color=AXIS[i], linewidth=1.0, label=f"v{lab}")
        xx = np.linspace(-4, 4, 200)
        ax.plot(xx, np.exp(-xx**2 / 2) / np.sqrt(2 * np.pi), color=INK, linewidth=0.8, linestyle="--", label="N(0,1)")
        sd = np.std(ew / sig, axis=0)
        headline(ax, f"sd(e/σ) {sd[0]:.2f} / {sd[1]:.2f} / {sd[2]:.2f}\nσ {np.mean(sig[:,0])*1e3:.0f} mm/s, err rms {np.sqrt(np.mean(ew[:,0]**2))*1e3:.1f}", loc="lower right")
        ax.legend(loc="upper right")
        ax.set_xlim(-4, 4)
    else:
        headline(ax, "covariance not logged\n(MdlPosVelEstimator_cov)")
    ax.set_title("Consistency: e / σ")
    ax.set_xlabel("e / σ")
    ax.set_ylabel("density")
    ax = fig.add_subplot(gs[4, 2])
    oe = L.omega_err()
    if oe is not None and V is not None and len(V["seg"]):
        for i, lab in enumerate("xyz"):
            tm, mean, sd = cycle_stats(t, np.where(w[:, None], oe, np.nan)[:, i], V["seg"])
            ax.plot(tm, np.clip(mean * 1e3, -BIAS_LIM, BIAS_LIM), color=AXIS[i], label=f"ω{lab}")
        headline(ax, "mean " + " / ".join(f"{np.nanmean(oe[w, i])*1e3:+.1f}" for i in range(3)) + "\n(z: unlearned bias)")
        ax.legend(loc="lower right", ncol=3)
    ax.set_ylim(-BIAS_LIM, BIAS_LIM)
    zero_line(ax)
    ax.set_xlim(lo, hi)
    ax.set_title("ω error (cycle mean)")
    ax.set_ylabel("mrad/s")
    ax.set_xlabel("sim time [s]")
    return save(fig, stem)


def fig_phase(L, lo, hi, stem, title=None):
    """Errors and foot motion folded on the gait cycle (FL touchdown = phase 0)."""
    g = gait_info(L.t, L.col("con"), lo, hi)
    if g is None:
        return []
    t = L.t
    w = (t >= lo) & (t <= hi)
    seg = cycle_windows(g["rise"][(t[g["rise"]] >= lo) & (t[g["rise"]] + g["period"] <= hi)], g["n_cyc"], len(t))
    ph = (np.arange(g["n_cyc"]) + 0.5) / g["n_cyc"]
    fig, axes = plt.subplots(6, 1, figsize=(FIG_W, 9.0), sharex=True)
    fig.suptitle((title or L.label) + f"   {len(seg)} cycles folded, FL touchdown at phase 0", fontsize=8.5, color=MUTED, x=0.01, ha="left")
    ve = L.vel_err_body()
    for i, lab in enumerate("xyz"):
        ax = axes[i]
        if ve is not None:
            m, s = fold(np.where(w[:, None], ve, np.nan)[:, i] * 1e3, seg)
            mc, note = clip_note(m, -ZOOM_LIM, ZOOM_LIM, "mm/s")
            ax.fill_between(ph, np.clip(m - s, -ZOOM_LIM, ZOOM_LIM), np.clip(m + s, -ZOOM_LIM, ZOOM_LIM), color=AXIS[i], alpha=BAND_ALPHA, linewidth=0, label="± sd across cycles")
            ax.plot(ph, mc, color=AXIS[i], label="mean")
            headline(ax, f"mean over cycle {np.nanmean(m):+.1f} mm/s" + (f"; {note}" if note else ""))
            if i == 0:
                ax.legend(loc="lower right", ncol=2)
        ax.set_ylim(-ZOOM_LIM, ZOOM_LIM)
        zero_line(ax)
        ax.set_ylabel("mm/s")
        ax.set_title(f"v{lab} error vs gait phase")
    ft = L.col("ft")
    if ft is not None:
        vf = L.foot_velocity_body(0)
        m, s = fold(vf[:, 2] * 1e3, seg)
        ax = axes[3]
        ax.fill_between(ph, np.clip(m - s, -150, 150), np.clip(m + s, -150, 150), color=BLUE, alpha=BAND_ALPHA, linewidth=0)
        ax.plot(ph, np.clip(m, -150, 150), color=BLUE)
        ax.set_ylim(-150, 150)
        zero_line(ax)
        ax.set_ylabel("mm/s")
        ax.set_title("FL foot-centre vertical velocity (truth): sinks at touchdown, rebounds while trusted")
        z = ft[:, 2] * 1e3
        m, s = fold(z, seg)
        ax = axes[4]
        ax.plot(ph, np.clip(m, 14, 24), color=BLUE, label="FL foot-centre height")
        ax.axhline(FOOT_RADIUS * 1e3, color=INK, linewidth=0.8, linestyle="--", label="datum 22 mm")
        ax.set_ylim(14, 24)
        ax.set_ylabel("mm")
        ax.legend(loc="lower right", ncol=2)
        ax.set_title("FL foot-centre height (truth); swing off scale")
    ax = axes[5]
    con = L.col("con")
    for i, (leg, col) in enumerate(zip((0, 1), (BLUE, ORANGE))):
        m, _ = fold(con[:, leg], seg)
        ax.plot(ph, m, color=col, label=("FL (and RR)", "FR (and RL)")[i])
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("trust")
    ax.set_xlabel("gait phase (FL touchdown = 0, period %.0f ms)" % (g["period"] * 1e3))
    ax.legend(loc="center right", ncol=2)
    headline(ax, f"ramp up {g['ramp_up']:.2f}, down {g['ramp_down']:.2f} of stance")
    ax.set_title("Contact trust the filter applied")
    for ax in axes:
        ax.axvline(0.5, color=MUTED, linewidth=0.6)
        ax.set_xlim(0, 1)
    return save(fig, stem)


def third_order_model(theta0, kp, ki, tau, t):
    """Linearised Mahony error loop with the reference transported at the bias-corrected rate:
    e' = -kp (e - r) - b, b' = ki (e - r), r' = -r/tau - b; integrated at 1 ms."""
    n = int(round(t[-1] / 1e-3)) + 1
    e, b, r = theta0, 0.0, 0.0
    out = np.empty(n)
    for k in range(n):
        out[k] = e
        de, db, dr = -kp * (e - r) - b, ki * (e - r), -r / tau - b
        e += de * 1e-3
        b += db * 1e-3
        r += dr * 1e-3
    return np.interp(t, np.arange(n) * 1e-3, out)


def fig_convergence(L, probe_time, stem, kp=1.0, ki=0.25, tau=0.5, title=None):
    """Planted roll error against the third-order loop model (the probe is about body x)."""
    tilt, _ = L.tilt_heading()
    if tilt is None:
        return []
    t = L.t
    rot = L.data.get("MdlOrientationEstimator_error")
    roll = rot[:, 0] if rot is not None else tilt
    near = np.flatnonzero((t >= probe_time - 0.05) & (t <= probe_time + 0.1))
    if len(near) == 0:
        return []
    i0 = near[np.argmax(np.abs(roll[near]))]
    pre = (t >= probe_time - 1.5) & (t <= probe_time - 0.1)
    offset = roll[pre].mean() if pre.any() else 0.0
    sel = np.arange(i0, min(len(t), i0 + int(round(20.0 / L.dt))))
    tau_s = t[sel] - t[i0]
    err = (roll[sel] - offset) * DEG
    theta0 = err[0]
    model = third_order_model(theta0, kp, ki, tau, tau_s)
    fig, ax = plt.subplots(figsize=(FIG_W, 3.4))
    ax.plot(tau_s, err, color=BLUE, label="measured roll error")
    ax.plot(tau_s, model, color=INK, linewidth=0.9, linestyle="--", label=f"model kp={kp:g} ki={ki:g} τ={tau:g}")
    zero_line(ax)

    def cross(x, lvl):
        idx = np.flatnonzero(x <= lvl)
        return tau_s[idx[0]] if len(idx) else float("nan")
    headline(ax, f"θ0 {theta0:.2f} deg  t 1/e {cross(err, theta0/np.e):.2f} s (model {cross(model, theta0/np.e):.2f})\n"
                 f"undershoot {err.min()/theta0:+.3f} θ0 (model {model.min()/theta0:+.3f})  rms vs model {np.sqrt(np.mean((err-model)**2)):.3f} deg", loc="upper right")
    ax.legend(loc="center right")
    ax.set_xlim(0, tau_s[-1])
    ax.set_ylim(min(-1.5, 1.2 * err.min()), max(1.0, 1.1 * theta0))
    ax.set_xlabel("time since the probe [s]")
    ax.set_ylabel("deg")
    ax.set_title((title or L.label) + ": planted tilt error decays as the third-order loop predicts")
    return save(fig, stem)


def fig_before_after(La, Lb, lo, hi, stem, labels=("baseline", "final"), title=None):
    """Two logs of the same scenario: baseline grey, final colour, on the same axes."""
    fig, axes = plt.subplots(5, 1, figsize=(FIG_W, 9.0), sharex=True)
    fig.suptitle((title or f"{La.label} vs {Lb.label}") + f"   window {lo:.1f}-{hi:.1f} s", fontsize=8.5, color=MUTED, x=0.01, ha="left")
    cols = (GREY, BLUE)
    hl = [[], [], [], [], []]
    for L, col, lab in zip((La, Lb), cols, labels):
        t, act = L.t, L.active()
        w = (t >= lo) & (t <= hi)
        V = _velocity_rows(L, lo, hi)
        if V is not None and len(V["seg"]):
            for row, i in ((0, 0), (1, 2)):
                tm, mean, sd = cycle_stats(t, V["ve"][:, i], V["seg"])
                axes[row].plot(tm, np.clip(mean * 1e3, -50, 10), color=col, label=lab)
                hl[row].append(f"{np.nanmean(V['ve'][w & act, i])*1e3:+.1f}")
        tilt, _ = L.tilt_heading()
        if tilt is not None:
            tb, yb = block_mean(t[w], tilt[w] * DEG, max(1, int(round(0.05 / L.dt))))
            axes[2].plot(tb, np.clip(yb, 0, TILT_LIM), color=col, label=lab)
            hl[2].append(f"{np.sqrt(np.mean((tilt[w]*DEG)**2)):.2f}")
        be = L.bias_err()
        if be is not None:
            sel = t >= t[np.flatnonzero(act)[0]] if act.any() else np.ones_like(t, bool)
            tb, yb = block_mean(t[sel], be[sel, 1] * 1e3, max(1, int(round(0.05 / L.dt))))
            axes[3].plot(tb, np.clip(yb, -BIAS_LIM, BIAS_LIM), color=col, label=lab)
            hl[3].append(f"{np.max(np.abs(be[sel, 1]))*1e3:.1f}")
        pe = L.pos_err_world()
        if pe is not None:
            mask = w & act
            leak = L.heading_leak_xy(mask)
            al = np.where(mask, np.linalg.norm(pe[:, :2] - (leak if leak is not None else 0.0), axis=1), np.nan)
            tb, ab = block_mean(t, al, max(1, int(round(0.1 / L.dt))))
            axes[4].plot(tb, ab, color=col, label=lab)
            path = L.col("pe_path")
            last = np.flatnonzero(mask)[-1] if mask.any() else -1
            if path is not None and last >= 0 and path[last, 0] > 0.25:
                hl[4].append(f"{100*al[last]/path[last,0]:.2f} %")
    titles = ["vx error, cycle mean", "vz error, cycle mean", "Tilt error", "Gyro-bias error y (whole run)", "Heading-aligned xy position error"]
    units = ["mm/s", "mm/s", "deg", "mrad/s", "m"]
    lims = [(-50, 10), (-50, 10), (0, TILT_LIM), (-BIAS_LIM, BIAS_LIM), (0, None)]
    kinds = ["mean", "mean", "rms", "peak |err|", "drift"]
    for ax, ti, un, li, k, h in zip(axes, titles, units, lims, kinds, hl):
        ax.set_title(ti)
        ax.set_ylabel(un)
        ax.set_ylim(*li) if li[1] is not None else ax.set_ylim(bottom=0)
        if li[0] < 0:
            zero_line(ax)
        if len(h) == 2:
            headline(ax, f"{k}: {h[0]} → {h[1]} {un if k != 'drift' else ''}")
        ax.legend(loc="lower right", ncol=2)
    axes[-1].set_xlabel("sim time [s]")
    axes[-1].set_xlim(lo, hi)
    return save(fig, stem)


def fig_dumbbell(rows, speeds, stem, labels=("baseline", "final"), title="Scalar metrics across trot speeds"):
    """rows: list of (metric label, unit, {speed: (baseline, final)}) -> dot pairs per speed."""
    n = len(rows)
    ncol = 3
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(FIG_W, 1.6 * nrow + 0.6))
    axes = np.atleast_1d(axes).ravel()
    fig.suptitle(title, fontsize=8.5, color=MUTED, x=0.01, ha="left")
    for ax, (lab, unit, vals) in zip(axes, rows):
        ys = np.arange(len(speeds))[::-1]
        allv = [v for s in speeds for v in vals.get(s, (np.nan, np.nan))]
        lo = min(0.0, np.nanmin(allv)); hi = max(0.0, np.nanmax(allv))
        pad = 0.15 * (hi - lo if hi > lo else 1.0)
        nd = 2 if (hi - lo) < 2.0 else 1
        for y, s in zip(ys, speeds):
            b, f = vals.get(s, (np.nan, np.nan))
            ax.plot([b, f], [y, y], color=GREY, linewidth=0.8, zorder=1)
            ax.plot(b, y, "o", color=GREY, markersize=5, zorder=2, label=labels[0] if y == ys[0] else None)
            ax.plot(f, y, "o", color=BLUE, markersize=5, zorder=3, label=labels[1] if y == ys[0] else None)
            # Labels above (baseline) and below (final); when the dots nearly coincide,
            # push them apart horizontally so the two numbers never overprint.
            close = abs(f - b) < 0.12 * (hi - lo + 2 * pad)
            ax.text(b, y + 0.22, f"{b:.{nd}f}", fontsize=6.5, color=MUTED, ha="right" if close else "center", in_layout=False)
            ax.text(f, y - 0.42, f"{f:.{nd}f}", fontsize=6.5, color=INK, ha="left" if close else "center", in_layout=False)
        ax.set_yticks(ys)
        ax.set_yticklabels([f"{s} m/s" for s in speeds])
        ax.set_xlim(lo - pad, hi + pad)
        ax.set_ylim(-0.8, len(speeds) - 0.2)
        if lo < 0 < hi or lo == 0:
            ax.axvline(0, color=MUTED, linewidth=0.6)
        ax.set_title(f"{lab} [{unit}]")
        ax.grid(axis="y", visible=False)
    handles, lab_ = axes[0].get_legend_handles_labels()
    for ax in axes[n:]:
        ax.axis("off")
    if len(axes) > n:
        axes[n].legend(handles, lab_, loc="center left", ncol=1, fontsize=8)
    else:
        axes[0].legend(handles, lab_, loc="lower right", ncol=2)
    return save(fig, stem)


def _stance_fold(L, lo, hi):
    g = gait_info(L.t, L.col("con"), lo, hi)
    if g is None:
        return None
    rise = g["rise"][(L.t[g["rise"]] >= lo) & (L.t[g["rise"]] + g["stance"] <= hi)]
    seg = cycle_windows(rise, g["n_st"], len(L.t))
    ph = (np.arange(g["n_st"]) + 0.5) / g["n_st"]
    return g, seg, ph


def fig_explain_rolling(L, lo, hi, stem, title=None):
    """Why the forward velocity read low: the sphere centre rolls at r·ω_calf while the filter
    assumed a stationary point."""
    f = _stance_fold(L, lo, hi)
    vf = L.foot_velocity_body(0)
    vr = L.rolling_prediction_body(0)
    if f is None or vf is None or vr is None:
        return []
    g, seg, ph = f
    con = L.col("con")
    mf, sf = fold(vf[:, 0] * 1e3, seg)
    mr, _ = fold(vr[:, 0] * 1e3, seg)
    tr, _ = fold(con[:, 0], seg)
    fig, axes = plt.subplots(3, 1, figsize=(FIG_W, 6.2), sharex=True)
    fig.suptitle((title or L.label) + f"   FL, {len(seg)} stances folded", fontsize=8.5, color=MUTED, x=0.01, ha="left")
    ax = axes[0]
    ax.fill_between(ph, np.clip(mf - sf, -50, 200), np.clip(mf + sf, -50, 200), color=BLUE, alpha=BAND_ALPHA, linewidth=0)
    ax.plot(ph, np.clip(mf, -50, 200), color=BLUE, label="true foot-centre forward velocity")
    ax.plot(ph, np.clip(mr, -50, 200), color=INK, linewidth=0.9, linestyle="--", label="rolling prediction r·(ω_calf × ẑ)")
    full = tr >= 0.999
    headline(ax, f"trusted stance: foot {np.mean(mf[full]):.1f} mm/s, rolling {np.mean(mr[full]):.1f}, residual {np.mean((mf-mr)[full]):+.1f}", loc="upper right")
    ax.set_ylim(-50, 200)
    zero_line(ax)
    ax.set_ylabel("mm/s")
    ax.legend(loc="center right")
    ax.set_title("A stance foot is not stationary: its centre rolls with the calf")
    ax = axes[1]
    ax.plot(ph, np.clip(mf - mr, -30, 30), color=BLUE, label="foot velocity − rolling prediction")
    ax.set_ylim(-30, 30)
    zero_line(ax)
    ax.set_ylabel("mm/s")
    ax.set_title("Residual after the rolling term (slip at touchdown, nothing in mid-stance)")
    ax = axes[2]
    ax.plot(ph, tr, color=BLUE)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("trust")
    ax.set_xlabel("stance phase (touchdown = 0, liftoff = 1)")
    ax.set_title("Contact trust the filter applied")
    for ax in axes:
        ax.set_xlim(0, 1)
    return save(fig, stem)


def fig_explain_touchdown(Lf, lo, hi, stem, Lb=None, title=None):
    """Why vz was biased: the compliant foot sinks and rebounds while the old ramp trusted it."""
    f = _stance_fold(Lf, lo, hi)
    ft = Lf.col("ft")
    if f is None or ft is None:
        return []
    g, seg, ph = f
    vf = Lf.foot_velocity_body(0)
    mz, sz = fold(ft[:, 2] * 1e3, seg)
    mv, sv = fold(vf[:, 2] * 1e3, seg)
    trf, _ = fold(Lf.col("con")[:, 0], seg)
    fig, axes = plt.subplots(4, 1, figsize=(FIG_W, 7.4), sharex=True)
    fig.suptitle((title or Lf.label) + f"   FL, {len(seg)} stances folded", fontsize=8.5, color=MUTED, x=0.01, ha="left")
    ax = axes[0]
    ax.fill_between(ph, mz - sz, mz + sz, color=BLUE, alpha=BAND_ALPHA, linewidth=0)
    ax.plot(ph, mz, color=BLUE, label="true foot-centre height")
    ax.axhline(FOOT_RADIUS * 1e3, color=INK, linewidth=0.8, linestyle="--", label="height datum 22 mm")
    ax.set_ylim(14, 24)
    ax.set_ylabel("mm")
    ax.legend(loc="lower right", ncol=2)
    ax.set_title("The foot sinks ~5 mm at touchdown and rebounds over the first half of stance")
    ax = axes[1]
    ax.fill_between(ph, np.clip(mv - sv, -150, 150), np.clip(mv + sv, -150, 150), color=BLUE, alpha=BAND_ALPHA, linewidth=0)
    ax.plot(ph, np.clip(mv, -150, 150), color=BLUE)
    ax.set_ylim(-150, 150)
    zero_line(ax)
    ax.set_ylabel("mm/s")
    ax.set_title("Foot-centre vertical velocity: a rising trusted foot reads as a sinking body")
    ax = axes[2]
    for L, col, lab in ((Lb, GREY, "baseline"), (Lf, BLUE, "final")):
        if L is None:
            continue
        fl = _stance_fold(L, lo, hi)
        ve = L.vel_err_body()
        if fl is None or ve is None:
            continue
        m, _ = fold(ve[:, 2] * 1e3, fl[1])
        ax.plot(fl[2], np.clip(m, -80, 40), color=col, label=f"{lab}: vz error, mean {np.nanmean(m):+.1f} mm/s")
    ax.set_ylim(-80, 40)
    zero_line(ax)
    ax.set_ylabel("mm/s")
    ax.legend(loc="lower right")
    ax.set_title("Filter vz error over the stance")
    ax = axes[3]
    if Lb is not None:
        flb = _stance_fold(Lb, lo, hi)
        if flb is not None:
            trb, _ = fold(Lb.col("con")[:, 0], flb[1])
            ax.plot(flb[2], trb, color=GREY, label="baseline trust (ramps 0.2 / 0.2)")
    ax.plot(ph, trf, color=BLUE, label=f"final trust (ramps {g['ramp_up']:.2f} / {g['ramp_down']:.2f})")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("trust")
    ax.legend(loc="center right")
    ax.set_xlabel("stance phase (touchdown = 0, liftoff = 1)")
    ax.set_title("Trust ramps: the final one waits out the rebound")
    for ax in axes:
        ax.set_xlim(0, 1)
    return save(fig, stem)


def fig_explain_heading(L, lo, hi, stem, title=None):
    """Why vy looked like it diverged: an unobservable heading error rotates the world frame."""
    qt, qe = L.q_true(), L.col("os_quat")
    vw, vb = L.vel_err_world(), L.vel_err_body()
    if qt is None or qe is None or vw is None or vb is None:
        return []
    t, act = L.t, L.active()
    sel = t >= (t[np.flatnonzero(act)[0]] if act.any() else t[0])
    ht = np.unwrap(heading_of(qt[sel])) * DEG
    he = np.unwrap(heading_of(qe[sel])) * DEG
    ht -= ht[0]; he -= he[0]
    fig, axes = plt.subplots(2, 1, figsize=(FIG_W, 5.2), sharex=True)
    fig.suptitle((title or L.label), fontsize=8.5, color=MUTED, x=0.01, ha="left")
    ax = axes[0]
    nb = max(1, int(round(0.05 / L.dt)))
    tb, hb = block_mean(t[sel], he, nb)
    _, tb2 = block_mean(t[sel], ht, nb)
    ax.plot(tb, hb, color=BLUE, label="estimated heading")
    ax.plot(tb, tb2, color=INK, linewidth=0.9, label="true heading")
    headline(ax, "gravity says nothing about heading: the estimate drifts with the gyro z-bias;\nthe controller holds the estimate, so the robot really turns", loc="upper right")
    ax.set_ylim(min(-2.0, 1.2 * min(ht.min(), he.min())), max(3.0, 1.2 * max(ht.max(), he.max()) + 2.0))
    ax.set_ylabel("deg (relative to start)")
    ax.legend(loc="lower left")
    ax.set_title("Heading: estimate and truth drift apart")
    ax = axes[1]
    w = (t >= lo) & (t <= hi) & act
    vw_y = np.where(w, vw[:, 1] * 1e3, np.nan)
    vb_y = np.where(w, vb[:, 1] * 1e3, np.nan)
    tb, a = block_mean(t, vw_y, max(1, int(round(0.3 / L.dt))))
    _, b = block_mean(t, vb_y, max(1, int(round(0.3 / L.dt))))
    ax.plot(tb, np.clip(a, -20, 60), color=GREY, label=f"world frame: mean {np.nanmean(vw_y):+.1f} mm/s")
    ax.plot(tb, np.clip(b, -20, 60), color=BLUE, label=f"body frame (own frames): mean {np.nanmean(vb_y):+.1f} mm/s")
    zero_line(ax)
    ax.set_ylim(-20, 60)
    ax.set_ylabel("mm/s")
    ax.set_xlabel("sim time [s]")
    ax.legend(loc="upper left")
    ax.set_title("vy error: world-frame comparison grows as v·sin(heading error); body-frame stays flat")
    ax.set_xlim(t[sel][0], hi)
    return save(fig, stem)
