#!/usr/bin/env python3
#
# Copyright (C) 2005-2026 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Verification of the whole-body impulse-control trot ([trot] wbc, Kim et al.
2019) against the per-leg trot on the simulated Go2: every run headless, scored,
gated and plotted.

  /home/ege/legged/iros/.venv/bin/python figs/trot_wbc/run_experiments.py [--only NAME ...] [--out DIR]

  tests        ctest in build/ (build and install first)
  oldpath      wbc = false default trot log against the pre-work baseline (reports/trot_wbc/oldpath_baseline)
  amb_ablation figs/amb_wbc's ablation re-run (--only ablation) against reports/amb_wbc/results.json
  core         in place, forward 0.35/0.70/1.00, lateral 0.20, yaw 0.5, circle, all on ground-truth state;
               forward 0.70 with the position/velocity filter and with both filters in the loop; both
               modes; plus the measurement-chain checks
  diagnosis    targeted runs and analyses of the core logs behind the WBC's pitch offset and early touchdown
               (needs core's logs)
  duty         duty factor 0.30 ... 0.70 at the default period, in place and forward 0.35, both modes
               (below 0.5: flight phases)
  cadence      in place, gait period 0.3 ... 0.8 s, each mode up to its first failure
  push         in place, 0.1 s lateral pulse at the base CoM, both signs, two gait phases, each mode, to the
               first failure

Every run but oldpath and amb_ablation is one bin/sim.sh call driven by the
Supervisor key script "0 s", "2.5 t", "19.3 d": stand, T, 15 s in MdlTrot's TROT
state (after its 1.8 s WAIT + PREP), then the normal stop (ramp-down, centering,
sit). Fixed key times give every run the same gait-phase datum, which the push
times need. Logs: runs/trot_wbc/<experiment>/<mode>_<run>.mat and .log;
results.json, figs/ and the report in reports/trot_wbc/ (--out DIR puts all of
them under DIR; the simulator lock stays runs/.simlock, shared with figs/amb_wbc
because both drive the one fixed-port simulator). Torso state, velocities, ground
reactions, CoM and L come from simulator truth (MdlSimDriver_qpos/qvel/grf
through the MuJoCo model); MPC, WBIC and swing quantities from MdlTrot's log
(layout of MdlTrot_wbc at _logWbc in include/quadruped/MdlTrot.hh).
"""

import argparse
import fcntl
import gzip
import importlib.util
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import tomllib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from scipy.io import loadmat  # noqa: E402

import build_report  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BIN = os.path.join(REPO, "bin")
# figs/amb_wbc holds modules of the same names as this directory's, so its
# run_experiments is loaded by path. Its own "import build_report" then gets
# this directory's (first on sys.path), which it only calls from its main().
_spec = importlib.util.spec_from_file_location("amb_wbc_run_experiments",
                                               os.path.join(REPO, "figs", "amb_wbc", "run_experiments.py"))
amb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(amb)
COL, MASS, FOOT_R, FALL_Z, FALL_ANG, TAU_LIM = amb.COL, amb.MASS, amb.FOOT_R, amb.FALL_Z, amb.FALL_ANG, amb.TAU_LIM
# The pre-work trot log (wbc absent) and its run's stdout, kept in the tracked tree because runs/ is ignored.
BASELINE = os.path.join(REPO, "reports", "trot_wbc", "oldpath_baseline", "trot_a")
AMB_REF = os.path.join(REPO, "reports", "amb_wbc", "results.json")

T_KEY, PREP, TROT_S = 2.5, 1.8, 15.0   # T key; MdlTrot WAIT 0.3 + PREP 1.5 s; time in TROT [s]
D_KEY = T_KEY + PREP + TROT_S          # 19.3 s: the stop key
EXIT = 26.0                            # the sit has settled by ~24.5 s
SCRIPT = ["0 s", f"{T_KEY} t", f"{D_KEY:.1f} d"]
RAMP, SETTLE, CENTER = 2.0, 1.0, 1.5   # trot.toml rampup, settling allowance, centering [s]
LOADED = 5.0                           # [N] a foot counts as loaded above this simulated normal force
CONTACT = 0.5                          # [N] and as free of contact below this one
DUTY = 0.5
MODES = ("baseline", "wbc")
TRUTH = ["MdlSimDriver_qpos", "MdlSimDriver_qvel", "MdlSimDriver_grf", "MdlTrot_torquereq", "MdlTrot_contact",
         "MdlTrot_bodyref"]
# 125 and 122 of the 127 doubles a sample line holds: the baseline has no
# MdlTrot_wbc, so its MPC forces and swing references are logged instead.
VARS = {"baseline": TRUTH + ["MdlConvexMPC_forces", "MdlTrot_footref"], "wbc": TRUTH + ["MdlTrot_wbc"]}
# The measurement-chain run trades the truth GRF and torque for the swing
# references, so both modes' swing errors can be compared on one WBC log.
VARS_CHAIN = ["MdlSimDriver_qpos", "MdlSimDriver_qvel", "MdlTrot_contact", "MdlTrot_footref", "MdlTrot_wbc"]
# Every 1 ms control tick. A 500 Hz log holds alternate ticks, and which half depends on the tick the log thread
# starts on (wall clock): identical runs then gave touchdown timings up to 4 ms apart.
LOG_PERIOD = 1
# name, forward, lateral [m/s], yaw rate [rad/s], extra overrides
CORE = [("inplace", 0.0, 0.0, 0.0, []), ("fwd035", 0.35, 0.0, 0.0, []), ("fwd070", 0.70, 0.0, 0.0, []),
        ("fwd100", 1.00, 0.0, 0.0, []), ("lat020", 0.0, 0.20, 0.0, []), ("yaw050", 0.0, 0.0, 0.5, []),
        ("circle", 0.35, 0.0, 0.5, []),
        # The filters in the loop: position/velocity alone (true attitude), then both (noisy IMU).
        ("fwd070_pv", 0.70, 0.0, 0.0, ["posvelestimator.use_ground_truth = false"]),
        ("fwd070_imu", 0.70, 0.0, 0.0, ["orientationestimator.use_ground_truth = false",
                                        "posvelestimator.use_ground_truth = false"])]
# The controller is judged on the simulator's own state: attitude and base position/velocity come from ground
# truth (shadow mode, the filters keep running), so no estimator error enters its feedback. posvelestimator's
# default is the filter; the runs that put a filter in the loop override this.
GROUND_TRUTH = ["orientationestimator.use_ground_truth = true", "posvelestimator.use_ground_truth = true"]
SUPPORT_RUNS = ("inplace", "fwd070")   # support-line analysis
PERIODS = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
# Duty factor at the default period: above 0.5 the diagonals overlap on four feet for (duty - 0.5) P, below it
# no foot is scheduled down for (0.5 - duty) P per half cycle (a flight phase).
DUTIES, DUTY_SPEEDS = [0.70, 0.60, 0.50, 0.45, 0.40, 0.35, 0.30], [0.0, 0.35]
PULSE, PUSH_AT, COARSE, FINE, PUSH_CAP = 0.1, 27, 20, 5, 400   # [s], gait cycles after TROT entry, [N]
PHASES = {"touchdown": 0.0, "midstance": 0.5 * DUTY * 0.3}       # FL-RR stance start, and half a stance later
# Diagnosis only: a 1 um clamp holds the horizontal body reference on the estimate (the key rejects 0), so the
# WBC sees no horizontal position error; the MPC weights position 0 and does not notice.
NO_POS_ERR = "trot.position_reference_clamp = 1e-6"
COM_SHIFT = 0.002                      # [m] the MPC's CoM belief moved forward: a known nose-up moment
# The defaults every run starts from (config/default shadows the version and robot tiers), for the report.
CONFIG = {"trot": {k: v for k, v in tomllib.load(open(os.path.join(REPO, "config", "default", "trot.toml"), "rb"))
                   ["trot"].items() if k.startswith("wbc") or k == "command_failure_limit"},
          "mpc": {k: v for k, v in tomllib.load(open(os.path.join(REPO, "config", "default", "mpc.toml"), "rb"))
                  ["mpc"].items() if k in ("dt", "horizon", "state_weights", "com_offset_body")}}
COM_OFFSET = np.array(CONFIG["mpc"]["com_offset_body"])   # CoM from the base origin, body axes [m]
EXPERIMENTS = ["tests", "oldpath", "amb_ablation", "core", "diagnosis", "duty", "cadence", "push"]
_M, _D, _FOOT = amb._M, amb._D, amb._FOOT


def check_no_simulation():
    if subprocess.run(["pgrep", "-x", "simulation"], capture_output=True).returncode == 0:
        raise RuntimeError("another simulation process is running (it binds the same port)")


def simulate(path, over, ext=".mat"):
    """One headless bin/sim.sh run with the given overrides -> path+ext, path.log.
    Serialised by amb_wbc's lock: the simulator binds a fixed port."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    for e in (ext, ".log"):
        if os.path.exists(path + e):
            os.remove(path + e)
    t0 = time.time()
    os.makedirs(os.path.dirname(amb.LOCK), exist_ok=True)
    with open(amb.LOCK, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        check_no_simulation()
        with open(path + ".log", "w") as log:
            log.write("overrides: " + json.dumps(over) + "\n")
            log.flush()
            # Own process group: sim.sh does not exec ./simulation, so killing the shell alone on a timeout would
            # leave the simulator running and holding the port. The group is outside the terminal's, so Ctrl-C
            # reaches only this script, which then kills the group too.
            p = subprocess.Popen(["./sim.sh"] + [x for c in over for x in ("-c", c)], cwd=BIN,
                                 stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
            try:
                rc = p.wait(timeout=3 * EXIT + 60)
            except BaseException:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
                raise
    if rc != 0 or not os.path.exists(path + ext):
        raise RuntimeError(f"{path}: exit {rc}, no log (see {path}.log)")
    print(f"  [{time.time() - t0:4.1f} s] {os.path.join(*path.split(os.sep)[-2:])}", flush=True)
    return round(time.time() - t0, 1)


def overrides(path, mode, cmd, extra=(), logvars=None, start=3.0):
    return ["simulation.headless = true", "simulation.realtime = false", f"supervisor.exit_time = {EXIT}",
            f"supervisor.script = {json.dumps(SCRIPT)}", "supervisor.log.enable = true",
            f"supervisor.log.start = {start:.4f}", f"supervisor.log.period = {LOG_PERIOD}",
            'supervisor.log.file_format = "matlab"', f'supervisor.log.file_name = "{path}.mat"',
            f"supervisor.log.vars = {json.dumps(logvars or VARS[mode])}", f"trot.wbc = {str(mode == 'wbc').lower()}",
            f"trot.forward_velocity = {cmd['vx']}", f"trot.lateral_velocity = {cmd['vy']}",
            f"trot.yaw_rate = {cmd['yaw']}", f"trot.period = {cmd['period']}"] + GROUND_TRUTH + list(extra)


def r(x, n=6):
    return None if x is None or not np.isfinite(x) else round(float(x), n)


FRONT, REAR = np.array([0, 1]), np.array([3, 2])   # stance diagonal 0: FL-RR, 1: FR-RL


def about_line(R, sl, a, b):
    """L [kg m^2/s] about the horizontal line through contact points a, b and
    the CoM's offset c_h [m] from it, over samples sl. s runs from the front
    foot to the rear one, so t_h = s x z points to the robot's left on both
    diagonals, and with point feet fixed on the line dL/dt = M g c_h."""
    s = (b - a) * [1.0, 1.0, 0.0]
    s /= np.linalg.norm(s, axis=-1, keepdims=True)
    rc = R.com[sl] - 0.5 * (a + b)
    return (np.sum(s * (R.Lc[sl] + MASS * np.cross(rc, R.vc[sl])), axis=-1),
            s[..., 1] * rc[:, 0] - s[..., 0] * rc[:, 1])


class Run:
    """A loaded trot run, times in absolute simulation seconds. datum is
    MdlTrot's gait-phase datum (TROT entry, or entry + k periods when the log
    starts late): FL and RR are scheduled in stance over [datum, datum +
    duty P) mod P."""

    def __init__(self, path, mode, cmd):
        raw = loadmat(path + ".mat", squeeze_me=False, struct_as_record=True)
        rec = raw["data"].ravel()
        D = {n: np.concatenate([np.atleast_2d(v) for v in rec[n]], axis=1).T.astype(float) for n in rec.dtype.names}
        self.t = D["time"][:, 0] - D["time"][0, 0] + float(np.squeeze(raw["params"]["time_offset"][0, 0])) * 1e-6
        self.D, self.mode, self.cmd, n = D, mode, cmd, len(self.t)
        qpos, qvel = D["MdlSimDriver_qpos"], D["MdlSimDriver_qvel"]
        R, feet = np.empty((n, 3, 3)), np.empty((n, 4, 3))
        self.com, self.Lc, self.vc = np.empty((n, 3)), np.empty((n, 3)), np.empty((n, 3))
        for i in range(n):
            _D.qpos[:], _D.qvel[:] = qpos[i], qvel[i]
            for f in (mujoco.mj_kinematics, mujoco.mj_comPos, mujoco.mj_comVel, mujoco.mj_subtreeVel):
                f(_M, _D)
            R[i], self.com[i], feet[i] = _D.xmat[1].reshape(3, 3), _D.subtree_com[1], _D.geom_xpos[_FOOT]
            self.Lc[i], self.vc[i] = _D.subtree_angmom[1], _D.subtree_linvel[1]   # about the CoM; CoM velocity
        self.qpos, self.z, self.rpy, self.rot = qpos, qpos[:, 2], amb.zyx(qpos[:, 3:7]), R
        # MuJoCo's free joint: linear velocity in world axes, angular in body axes.
        self.v_body = np.einsum("nji,nj->ni", R, qvel[:, 0:3])
        self.yaw_rate = np.einsum("nij,nj->ni", R, qvel[:, 3:6])[:, 2]
        self.foot_body = np.einsum("nji,nkj->nki", R, feet - qpos[:, None, 0:3])   # foot centres, body axes
        self.C, self.B, self.W = D["MdlTrot_contact"], D.get("MdlTrot_bodyref"), D.get("MdlTrot_wbc")
        self.grf, self.tau = D.get("MdlSimDriver_grf"), D.get("MdlTrot_torquereq")
        self.f_des = (self.W[:, [6 * k + a for k in range(4) for a in range(3)]] if self.W is not None
                      else D.get("MdlConvexMPC_forces"))
        # The scheduled stance diagonal (0: FL-RR, 1: FR-RL, -1: other) and L,
        # c_h about the line through its two ground contact points.
        self.pair = np.full(n, -1)
        self.pair[(self.C == [1, 0, 0, 1]).all(1)] = 0
        self.pair[(self.C == [0, 1, 1, 0]).all(1)] = 1
        k, idx = np.maximum(self.pair, 0), np.arange(n)
        self.cp = feet - [0.0, 0.0, FOOT_R]
        L, ch = about_line(self, slice(None), self.cp[idx, FRONT[k]], self.cp[idx, REAR[k]])
        self.L, self.ch = (np.where(self.pair >= 0, v, np.nan) for v in (L, ch))
        self.swing_fz = None if self.grf is None else \
            np.where(self.pair == 0, self.grf[:, [5, 8]].max(1), self.grf[:, [2, 11]].max(1))
        # Gait datum from the first FL stance -> swing switch the log shows.
        fl = np.nonzero((self.C[:-1, 0] == 1) & (self.C[1:, 0] == 0))[0]
        self.datum = self.t[fl[0] + 1] - cmd.get("duty", DUTY) * cmd["period"] if len(fl) else None
        # MdlTrot writes bodyref on every TROT/STOPPING tick and never after:
        # its last change is the end of the stop ramp, or the tick of an ERROR.
        ch = np.nonzero(np.any(np.diff(self.B, axis=0) != 0, axis=1))[0] if self.B is not None else []
        self.t_end = self.t[ch[-1] + 1] if len(ch) else None
        text = re.sub(r"\x1b\[[\d;]*m", "", open(path + ".log", errors="replace").read())
        self.errors = re.findall(r"Supervisor: (ERROR[^\n]*)", text)
        self.warnings = sorted({w.strip() for w in re.findall(r"\[(?:WW|EE)\][^\n]*", text)
                                if "setPriority" not in w})
        self.stopped = "-> S_SIT (from trot)" in text and "sitting settled" in text and not self.errors
        e = re.search(r"MdlTrot: v=\[(\S+) (\S+)\] m/s yaw=(\S+) rad/s period=(\S+) s", text)
        self.echo = [float(x) for x in e.groups()] if e else None
        m = re.search(r"WBC tick mean ([\d.]+) us, max ([\d.]+) us over (\d+) ticks; QP failures (\d+), "
                      r"fallback ticks (\d+), torque saturated on (\d+) ticks", text)
        self.closing = m and {"tick_mean_us": float(m.group(1)), "tick_max_us": float(m.group(2)),
                              "ticks": int(m.group(3)), "qp_failures": int(m.group(4)),
                              "fallback_ticks": int(m.group(5)), "saturated_ticks": int(m.group(6)),
                              "saturated_pct": round(100.0 * int(m.group(6)) / int(m.group(3)), 4)}

    def win(self, a, b):
        return (self.t >= a) & (self.t <= b)

    def first(self, mask):
        i = np.nonzero(mask)[0]
        return r(self.t[i[0]], 4) if len(i) else None

    def trot_window(self, start=None):
        """Start (TROT entry or given) to the end of centering, or to the
        ERROR tick: after it the Supervisor sits the robot below 0.16 m."""
        end = self.t_end if self.errors else self.t_end + CENTER
        return self.win(self.datum if start is None else start, end)

    def fall(self, w):
        return self.first(w & ((self.z < FALL_Z) | np.any(np.abs(self.rpy[:, :2]) > FALL_ANG, axis=1)))

    def outcome(self, start=None):
        w = self.trot_window(start)
        ratio = np.abs(self.tau) / TAU_LIM
        return {"fall_t": self.fall(w), "error": self.errors[0] if self.errors else None,
                "error_t": self.t_end if self.errors else None, "warnings": self.warnings[:6],
                "stop_completed": self.stopped, "first_saturated_t": self.first(w & np.any(ratio > 1.0, axis=1)),
                "peak_tau_over_limit": r(ratio[w].max(), 4),
                "saturated_samples_pct": r(100.0 * np.mean(np.any(ratio[w] > 1.0, axis=1)), 4),
                "wbc_closing": self.closing, "mdltrot_echo": self.echo}


def metrics(R):
    """Every per-run metric of the experiment set over the steady window: TROT
    entry + ramp-up + 1 s to the stop key. A run that falls or errors gets its
    outcome only: its steady window would hold the fall or the sit."""
    c, out = R.cmd, R.outcome()
    if out["fall_t"] is not None or out["error"] is not None:
        return out
    w = R.win(R.datum + RAMP + SETTLE, D_KEY)
    T = R.t[w][-1] - R.t[w][0]
    v, yr = R.v_body[w].mean(0), R.yaw_rate[w].mean()
    verr = np.hypot(v[0] - c["vx"], v[1] - c["vy"])
    deg = np.degrees(R.rpy[w, :2])
    out.update(window_s=[r(R.t[w][0], 3), r(R.t[w][-1], 3)], vx=r(v[0]), vy=r(v[1]), yaw_rate=r(yr),
               v_err=r(verr), yaw_err=r(abs(yr - c["yaw"])),
               v_tol=max(0.1 * np.hypot(c["vx"], c["vy"]), 0.05), yaw_tol=max(0.1 * abs(c["yaw"]), 0.05),
               roll_mean=r(deg[:, 0].mean()), roll_sd=r(deg[:, 0].std()), roll_rms=r(amb.rms(deg[:, 0])),
               pitch_mean=r(deg[:, 1].mean()), pitch_sd=r(deg[:, 1].std()), pitch_rms=r(amb.rms(deg[:, 1])),
               z_mean=r(R.z[w].mean()), z_sd=r(R.z[w].std()),
               # The base height MdlTrot asks for: its CoM reference (bodyref[2]) less the CoM offset, body level.
               z_target=r(R.B[w, 2].mean() - COM_OFFSET[2]))
    out["z_minus_target_mm"] = r(1e3 * (out["z_mean"] - out["z_target"]), 2)
    if c["yaw"] == 0.0:   # off the heading at the window start, less the commanded lateral travel
        psi = R.rpy[w][:, 2]
        d = R.qpos[w][-1, :2] - R.qpos[w][0, :2]
        out["lateral_drift_m"] = r(d @ [-np.sin(psi[0]), np.cos(psi[0])] - c["vy"] * T, 4)
        # The part a heading change makes of it: the forward velocity carried off the initial heading.
        dpsi = np.unwrap(psi) - psi[0]
        out["lateral_drift_heading_m"] = r(np.sum(R.v_body[w, 0] * np.sin(dpsi)) * (R.t[1] - R.t[0]), 4)
        out["yaw_change_deg"] = r(np.degrees(dpsi[-1]), 3)
    # Ground reaction per axis over scheduled stance, leg-samples pooled; sums over the feet.
    st = [w & (R.C[:, k] == 1) for k in range(4)]
    des = np.concatenate([R.f_des[st[k], 3 * k:3 * k + 3] for k in range(4)])
    sim = np.concatenate([R.grf[st[k], 3 * k:3 * k + 3] for k in range(4)])
    g = {"rms_des_sim_N": np.sqrt(np.mean((des - sim) ** 2, 0)).round(3).tolist(),
         "sum_des_mean_N": sum(R.f_des[w, 3 * k:3 * k + 3] * R.C[w, k:k + 1] for k in range(4)).mean(0)
         .round(3).tolist(),
         "sum_sim_mean_N": R.grf[w].reshape(-1, 4, 3).sum(1).mean(0).round(3).tolist(), "weight_N": r(MASS * 9.81, 3)}
    if R.mode == "wbc":
        sol = np.concatenate([R.W[st[k], 6 * k + 3:6 * k + 6] for k in range(4)])
        g.update(rms_des_sol_N=np.sqrt(np.mean((des - sol) ** 2, 0)).round(3).tolist(),
                 rms_sol_sim_N=np.sqrt(np.mean((sol - sim) ** 2, 0)).round(3).tolist(),
                 sum_sol_mean_N=sum(R.W[w, 6 * k + 3:6 * k + 6] for k in range(4)).mean(0).round(3).tolist())
    out["grf"] = g
    # Swing-foot error p_B_des - p_B (ctrl): the WBC logs it; the baseline's
    # reference minus the truth forward kinematics (the same joint angles).
    sw = [w & (R.C[:, k] == 0) for k in range(4)]
    e = np.concatenate([(R.W[:, 31 + 3 * k:34 + 3 * k] if R.mode == "wbc"
                         else R.D["MdlTrot_footref"][:, 9 * k:9 * k + 3] - R.foot_body[:, k])[sw[k]] for k in range(4)])
    en = np.linalg.norm(e, axis=1)
    out["swing_err_mm"] = {"rms": r(1e3 * amb.rms(en), 3), "max": r(1e3 * en.max(), 3),
                           "rms_xyz": (1e3 * np.sqrt(np.mean(e ** 2, 0))).round(3).tolist()}
    fz = R.grf[:, 2::3]
    # Early touchdown: how long each foot has been in contact (above CONTACT, without a break) when its scheduled
    # swing ends. A 5 N threshold would split a bouncing first contact.
    lead = []
    for k in range(4):
        for j in np.nonzero(w[1:] & (R.C[:-1, k] == 0) & (R.C[1:, k] == 1))[0] + 1:
            n = 0
            while R.C[j - 1 - n, k] == 0 and fz[j - 1 - n, k] > CONTACT:
                n += 1
            lead.append(1e3 * n * (R.t[1] - R.t[0]))
    out["touchdown"] = {"swing_loaded_pct": r(100 * np.mean(np.concatenate([fz[sw[k], k] > LOADED for k in range(4)])), 3),
                        "stance_unloaded_pct": r(100 * np.mean(np.concatenate([fz[st[k], k] < LOADED
                                                                               for k in range(4)])), 3),
                        "first_contact_before_scheduled_ms": {"mean": r(np.mean(lead), 2), "max": r(np.max(lead), 2)}}
    if R.mode == "wbc":
        tw = R.trot_window()
        out["wbc_logged"] = {"fallback_samples": int(np.sum(R.W[tw, 43] != 0)),
                             "qp_status_nonzero_samples": int(np.sum(R.W[tw, 24] != 0))}
    return out


def mujoco_contact_points(R, idx):
    """MuJoCo's own contact position of each foot at samples idx, (len(idx), 4, 3); the sphere's lowest point
    (R.cp) where a foot has none. MuJoCo places a contact midway through the penetration."""
    out = R.cp[idx].copy()
    for n, i in enumerate(idx):
        _D.qpos[:] = R.qpos[i]
        mujoco.mj_fwdPosition(_M, _D)
        c = _D.contact
        for k, g in enumerate(_FOOT):
            m = (c.geom1 == g) | (c.geom2 == g)
            if m.any():
                out[n, k] = c.pos[m].mean(0)
    return out


def support_line(R, mujoco_contacts=False):
    """L about the stance diagonal and the CoM offset c_h from it, per complete
    two-foot stance phase in the steady window; and the check dL/dt = M g c_h
    about the instantaneous line and about the line frozen at the phase's start
    (the rolling feet move the contact points), on the samples where the swing
    feet have no contact force. That threshold, not the 5 N 'loaded' one: a
    touchdown impact between two samples reads a few newtons and puts kN m
    spikes into dL/dt. With mujoco_contacts, also the peaks about the line
    through MuJoCo's contact positions instead of the sphere bottoms (the
    convention's effect). Returns the summary and the check's (M g c_h, dL/dt)
    samples."""
    w = R.win(R.datum + RAMP + SETTLE, D_KEY)
    p = np.where(w, R.pair, -2)
    cuts = np.nonzero(np.diff(p) != 0)[0] + 1
    free = R.swing_fz < CONTACT
    free &= np.roll(free, 1) & np.roll(free, -1)        # the difference quotient spans the neighbours
    edge = max(1, round(0.006 / (R.t[1] - R.t[0])))     # 6 ms: no difference quotient across a line switch
    if mujoco_contacts:
        cpm = R.cp.copy()
        cpm[w] = mujoco_contact_points(R, np.nonzero(w)[0])
    phases, alt, pts = [], [], {"instantaneous": [], "frozen": [], "swing_contact": []}
    for i, j in zip(cuts[1:-2], cuts[2:-1]):            # the first and last cuts are the window's edges
        if p[i] < 0:
            continue
        sl, fr, re_ = slice(i, j), FRONT[p[i]], REAR[p[i]]
        L, ch = R.L[sl], R.ch[sl]
        travel = np.linalg.norm((R.cp[j - 1, [fr, re_]] - R.cp[i, [fr, re_]])[:, :2], axis=1).max()
        phases.append([np.abs(L).max(), 1e3 * np.abs(ch).max(), 1e3 * (ch.max() - ch.min()), 1e3 * travel,
                       R.t[j] - R.t[i]])
        if mujoco_contacts:
            Lm, chm = about_line(R, sl, cpm[sl, fr], cpm[sl, re_])
            alt.append([np.abs(Lm).max(), 1e3 * np.abs(chm).max()])
        inner = np.zeros(j - i, bool)
        inner[edge:-edge] = True
        Lf, chf = about_line(R, sl, R.cp[i, fr], R.cp[i, re_])
        for key, LL, cc, m in (("instantaneous", L, ch, free[sl]), ("frozen", Lf, chf, free[sl]),
                               ("swing_contact", L, ch, ~free[sl])):
            pts[key].append(np.c_[MASS * 9.81 * cc, np.gradient(LL, R.t[sl])][inner & m])
    P, pts = np.array(phases), {k: np.concatenate(v) for k, v in pts.items()}
    out = {"n_phases": len(P), "stance_s_mean": r(P[:, 4].mean(), 4)}
    for k, name in enumerate(("peak_L", "peak_ch_mm", "ch_travel_mm", "contact_point_travel_mm")):
        out[name] = {"mean": r(P[:, k].mean()), "max": r(P[:, k].max())}
    if mujoco_contacts:
        A = np.array(alt)
        for k, name in enumerate(("peak_L_mujoco_contact", "peak_ch_mm_mujoco_contact")):
            out[name] = {"mean": r(A[:, k].mean()), "max": r(A[:, k].max())}
    out["check_dLdt_vs_Mgch"] = {"swing_contact_excluded_pct": r(100 * len(pts["swing_contact"]) / (
        len(pts["swing_contact"]) + len(pts["instantaneous"])), 2)}
    for key in ("instantaneous", "frozen"):
        x, y = pts[key].T
        slope, icpt = np.linalg.lstsq(np.c_[x, np.ones_like(x)], y, rcond=None)[0]
        out["check_dLdt_vs_Mgch"][key] = {"slope": r(slope, 4), "intercept_Nm": r(icpt, 4),
                                          "corr": r(np.corrcoef(x, y)[0, 1], 4), "rms_residual_Nm": r(amb.rms(y - x), 4),
                                          "rms_Mgch_Nm": r(amb.rms(x), 4), "samples": len(x)}
    return out, pts


def ratio_ok(a, b):
    return a is not None and b is not None and a <= 1.5 * b


def forward(R):
    """World unit vector of the body x axis on the ground plane, per sample."""
    x = R.rot[:, :, 0] * [1.0, 1.0, 0.0]
    return x / np.linalg.norm(x, axis=1, keepdims=True)


def force_balance(R):
    """Steady-window means along the heading: the MPC's planned force on the scheduled stance feet, the simulated
    force on them, on the scheduled swing feet and on all four [N] (at constant speed momentum balance holds the
    last near zero); the WBC's base position task error (estimator frame) [mm]; CoM height [m]."""
    w = R.win(R.datum + RAMP + SETTLE, D_KEY)
    ex, st = forward(R), (R.C == 1)[:, :, None]
    g, d = R.grf.reshape(-1, 4, 3), R.f_des.reshape(-1, 4, 3)

    def fwd(f):
        return r(np.einsum("nkj,nj->n", f, ex)[w].mean(), 2)
    out = {"stance_desired_N": fwd(d * st), "stance_simulated_N": fwd(g * st), "swing_simulated_N": fwd(g * ~st),
           "all_feet_simulated_N": fwd(g), "com_height_m": r(R.com[w, 2].mean(), 4)}
    if R.W is not None:
        out["position_error_mm"] = r(1e3 * np.einsum("nj,nj->n", R.W[:, 28:31], ex)[w].mean(), 2)
    return out


def touchdown_detail(R):
    """Means over the scheduled touchdowns in the steady window: height of the swing reference's contact point
    above the ground at the scheduled touchdown [mm]; how long before it the reference passes through the ground
    and the foot first touches (above CONTACT without a break) [ms], front and rear legs; the reference's height
    when the foot touches [mm]; the foot's vertical speed just before [m/s]; peak Fz while touching before the
    scheduled touchdown and over the 30 ms after it [N]; the forward impulse while touching before it [N s]."""
    w, dt = R.win(R.datum + RAMP + SETTLE, D_KEY), R.t[1] - R.t[0]
    # p_B_des (foot centre from the base origin, body axes): the baseline logs it, the WBC logs p_B_des - p_B.
    ref = (R.D["MdlTrot_footref"].reshape(-1, 4, 9)[:, :, 0:3] if "MdlTrot_footref" in R.D
           else R.W[:, 31:43].reshape(-1, 4, 3) + R.foot_body)
    zref = R.qpos[:, 2:3] + np.einsum("nij,nkj->nki", R.rot, ref)[:, :, 2] - FOOT_R
    zc, fz = R.cp[:, :, 2], R.grf[:, 2::3]
    vz, fx = np.gradient(zc, R.t, axis=0), np.einsum("nkj,nj->nk", R.grf.reshape(-1, 4, 3), forward(R))
    h, rows = int(0.5 * (1 - DUTY) * R.cmd["period"] / dt), []
    for k in range(4):
        for j in np.nonzero(w[1:] & (R.C[:-1, k] == 0) & (R.C[1:, k] == 1))[0] + 1:
            above = np.nonzero(zref[j - h:j, k] > 0)[0]
            n = 0
            while R.C[j - 1 - n, k] == 0 and fz[j - 1 - n, k] > CONTACT:
                n += 1
            i = j - n
            rows.append([k, 1e3 * zref[j - 1, k], 1e3 * (h - 1 - above[-1]) * dt if len(above) else np.nan,
                         1e3 * n * dt, 1e3 * zref[i, k], vz[i - 1, k], fz[i:j, k].max(initial=0.0),
                         fz[j:j + int(0.03 / dt), k].max(), fx[i:j, k].sum() * dt])
    E = np.array(rows)
    fr = E[:, 0] < 2

    def m(c, s=slice(None)):
        return r(np.nanmean(E[s, c]), 2)
    return {"touchdowns": len(E), "reference_at_touchdown_mm": m(1),
            "reference_through_ground_ms": {"front": m(2, fr), "rear": m(2, ~fr)},
            "first_contact_ms": {"front": m(3, fr), "rear": m(3, ~fr)}, "reference_at_first_contact_mm": m(4),
            "foot_vz_before_contact_m_s": m(5), "peak_fz_early_N": m(6), "peak_fz_30ms_after_N": m(7),
            "early_forward_impulse_Ns": r(m(8), 3)}


def leg_gravity(R):
    """Baseline stance: the per-leg command tau = J^T(-f_des) leaves each stance leg's own gravity torque g(q) to
    its contact, quasi-statically f = f_des + J^-T g(q) (J at the contact point). Steady-window means (10 ms
    steps), summed over the stance feet, of that term and of the measured simulated - desired force: along the
    heading and vertical [N], and the pitch moment about the CoM [N m] (+: nose down)."""
    ex, body = forward(R), [_M.geom_bodyid[g] for g in _FOOT]
    ey = np.cross([0.0, 0.0, 1.0], ex)
    acc = []
    for i in np.nonzero(R.win(R.datum + RAMP + SETTLE, D_KEY))[0][::max(1, round(0.01 / (R.t[1] - R.t[0])))]:
        _D.qpos[:], _D.qvel[:] = R.qpos[i], 0.0
        mujoco.mj_forward(_M, _D)   # qfrc_bias is gravity alone at zero velocity
        a = np.zeros((2, 3))
        for k in np.nonzero(R.C[i] == 1)[0]:
            jp = np.zeros((3, _M.nv))
            mujoco.mj_jac(_M, _D, jp, None, R.cp[i, k], body[k])
            dof = slice(6 + 3 * k, 9 + 3 * k)
            for row, f in enumerate((np.linalg.solve(jp[:, dof].T, _D.qfrc_bias[dof]),
                                     R.grf[i, 3 * k:3 * k + 3] - R.f_des[i, 3 * k:3 * k + 3])):
                a[row] += [f @ ex[i], f[2], np.cross(R.cp[i, k] - R.com[i], f) @ ey[i]]
        acc.append(a)
    a = np.mean(acc, 0).round(3).tolist()
    return {"leg_gravity_term": dict(zip(("fwd_N", "z_N", "pitch_Nm"), a[0])),
            "measured_simulated_minus_desired": dict(zip(("fwd_N", "z_N", "pitch_Nm"), a[1]))}


def estimator_rates(R):
    """Straight forward run, world x: the estimator's position rate against its own mean velocity, the true
    position rate and the body reference's [m/s]; estimated - true v_x rms [m/s]."""
    w = R.win(R.datum + RAMP + SETTLE, D_KEY)
    S, T = R.D["MdlPosVelEstimator_state"], R.t[w][-1] - R.t[w][0]

    def rate(x):
        return r((x[w][-1] - x[w][0]) / T, 4)
    return {"estimated_position_rate": rate(S[:, 0]), "estimated_vx_mean": r(S[w, 3].mean(), 4),
            "true_position_rate": rate(R.qpos[:, 0]), "reference_rate": rate(R.B[:, 0]),
            "estimated_minus_true_vx_rms": r(amb.rms(S[w, 3] - R.D["MdlSimDriver_qvel"][w, 0]), 4)}


class Experiments:
    def __init__(self, logs, rep):
        self.logs, self.rep, self.wall, self.cache = logs, rep, {}, {}

    def save(self, f, name):
        os.makedirs(os.path.join(self.rep, "figs"), exist_ok=True)
        f.tight_layout()
        f.savefig(os.path.join(self.rep, "figs", name))
        plt.close(f)
        return "figs/" + name

    def run(self, exp, name, mode, cmd, extra=(), logvars=None, start=3.0, rerun=True):
        path = os.path.join(self.logs, exp, f"{mode}_{name}")
        if rerun or not os.path.exists(path + ".mat"):
            self.wall[f"{exp}/{mode}_{name}"] = simulate(path, overrides(path, mode, cmd, extra, logvars, start))
        R = self.cache[exp, mode, name] = Run(path, mode, cmd)
        return R

    # -- tests -------------------------------------------------------------------
    def tests(self):
        p = subprocess.run(["ctest", "--output-on-failure"], cwd=os.path.join(REPO, "build"), capture_output=True,
                           text=True, timeout=900)
        m = re.search(r"(\d+)% tests passed, (\d+) tests failed out of (\d+)", p.stdout)
        return {"passed": m and int(m.group(3)) - int(m.group(2)), "total": m and int(m.group(3)),
                "trot_wbc_passed": bool(re.search(r"test_trot_wbc \.+\s+Passed", p.stdout)),
                "returncode": p.returncode, "tail": p.stdout.strip().splitlines()[-4:]}

    # -- old path ----------------------------------------------------------------
    def oldpath(self):
        """The baseline's own command (echoed in its .log) with the file name changed, logged on every tick from
        a little before the baseline's start: its log thread starts on whichever tick the wall clock allows, so
        a period-10 log can sit a tick off with identical physics. Text-identical samples (all fields but the
        relative time) at the stride and offset that match the baseline's first sample, over the samples both
        hold (the log thread also adds or drops a few at shutdown)."""
        for e in (".txt.gz", ".log"):
            if not os.path.exists(BASELINE + e):
                raise RuntimeError(f"old-path baseline {BASELINE + e} missing")
        d = os.path.join(self.logs, "oldpath")
        os.makedirs(d, exist_ok=True)
        base = os.path.join(d, "baseline_trot_a")
        with gzip.open(BASELINE + ".txt.gz", "rb") as src, open(base + ".txt", "wb") as dst:
            shutil.copyfileobj(src, dst)
        shutil.copy(BASELINE + ".log", base + ".log")
        echo = open(base + ".log").read().split("Custom configuration string:\n", 1)[1].split("initHardware", 1)[0]
        path = os.path.join(d, "wbc_absent_trot_a")
        over = [x for x in echo.strip().splitlines() if not x.startswith("supervisor.log.file_name")]
        start = 3.0 - 0.05   # the baseline logged from supervisor.log.start's default, 3 s
        self.wall["oldpath/wbc_absent_trot_a"] = simulate(path, over + [
            f'supervisor.log.file_name = "{path}.txt"', "supervisor.log.period = 1", f"supervisor.log.start = {start}"],
            ".txt")
        nvar = len(json.loads(re.search(r"supervisor.log.vars = (.*)", echo).group(1)))

        def samples(p):   # nvar lines per sample, the first led by the time since the log's first sample
            lines = [x for x in open(p + ".txt") if not x.startswith("#")]
            S = [lines[i:i + nvar] for i in range(0, len(lines) // nvar * nvar, nvar)]
            return [float(x[0].split(" ", 1)[0]) for x in S], [[x[0].split(" ", 1)[1]] + x[1:] for x in S]
        (ta, a), (tb, b) = samples(base), samples(path)
        stride = round((ta[1] - ta[0]) / (tb[1] - tb[0]))
        k = next((j for j in range(len(b)) if b[j] == a[0]), None)
        if k is None:
            return {"command": over, "identical": False, "first_sample_found": False,
                    "samples_baseline": len(a), "samples_common": 0, "samples_new_ticks": len(b)}
        n = min(len(a), (len(b) - k - 1) // stride + 1)
        diff = [i for i in range(n) if a[i] != b[k + stride * i]]
        return {"command": over, "first_sample_found": True, "stride_ticks": stride,
                "baseline_first_sample_tick_after_new_log_start": k, "samples_common": n,
                "samples_baseline": len(a), "samples_new_ticks": len(b), "identical": not diff,
                "first_differing_sample": diff[0] if diff else None,
                "files": [os.path.relpath(BASELINE + ".txt.gz", REPO), os.path.relpath(path + ".txt", REPO)]}

    # -- AMB ablation --------------------------------------------------------------
    def amb_ablation(self):
        """figs/amb_wbc's ablation (its own entry point, --only ablation) into <logs>/amb_ablation, against the
        committed reports/amb_wbc/results.json: the gate's verdict and every per-run physical number. The
        closing line's tick times are wall-clock and left out. That harness logs at 500 Hz, so a run whose log
        starts on the other tick parity than the reference run's (runs/amb_wbc/ablation, when present) samples
        the other half of the ticks: its sampled peaks and event times can differ by a tick with identical
        physics. Unchanged: the same verdict, and every differing value in such a run."""
        out = os.path.join(self.logs, "amb_ablation")
        if os.path.exists(os.path.join(out, "results.json")):
            os.remove(os.path.join(out, "results.json"))   # its main() merges into an existing one
        t0 = time.time()
        subprocess.run([sys.executable, os.path.join(REPO, "figs", "amb_wbc", "run_experiments.py"), "--only",
                        "ablation", "--out", out], check=True, stdin=subprocess.DEVNULL, timeout=1800)
        self.wall["amb_ablation"] = round(time.time() - t0, 1)
        new, ref = (json.load(open(p))["ablation"] for p in (os.path.join(out, "results.json"), AMB_REF))

        def flat(a):
            rows = {f"{g}/{n}": x for g in ("runs", "long") for n, x in a[g].items()}
            return {f"{k}.{q}": v for k, x in rows.items() for q, v in x.items() if q != "closing"} | {
                f"{k}.closing.{q}": v for k, x in rows.items() for q, v in x.get("closing", {}).items()
                if not q.startswith("tick_")}
        fn, fr = flat(new), flat(ref)
        diff = sorted(k for k in fr.keys() | fn.keys() if fn.get(k) != fr.get(k))

        def log_start(path):
            if not os.path.exists(path):
                return None
            return int(str(np.squeeze(loadmat(path, variable_names=["params"])["params"]["time_offset"][0, 0])))
        phase = {}
        for g, n in sorted({tuple(k.split(".")[0].split("/")) for k in diff}):
            f = n + ("_long" if g == "long" else "") + ".mat"
            phase[f"{g}/{n}"] = {"log_start_us_ref": log_start(os.path.join(REPO, "runs", "amb_wbc", "ablation", f)),
                                 "log_start_us_new": log_start(os.path.join(out, "runs", "ablation", f))}
        sampling = all(p["log_start_us_ref"] is not None and p["log_start_us_new"] is not None and
                       (p["log_start_us_ref"] - p["log_start_us_new"]) // 1000 % 2 == 1 for p in phase.values())
        return {"gate_new": new["gate"], "gate_ref": ref["gate"], "values_compared": len(fr),
                "differing": {k: [fr.get(k), fn.get(k)] for k in diff}, "differing_runs_log_start": phase,
                "differences_only_in_other_tick_parity": sampling,
                "unchanged": new["gate"]["result"] == ref["gate"]["result"] and (not diff or sampling),
                "results": os.path.relpath(os.path.join(out, "results.json"), REPO)}

    # -- core --------------------------------------------------------------------
    def core(self):
        res = {"runs": {m: {} for m in MODES}, "support_line": {m: {} for m in MODES}}
        for name, vx, vy, yaw, extra in CORE:
            for mode in MODES:
                R = self.run("core", name, mode, {"vx": vx, "vy": vy, "yaw": yaw, "period": 0.3}, extra)
                res["runs"][mode][name] = dict(metrics(R), cmd=[vx, vy, yaw], extra=extra)
                if name in SUPPORT_RUNS:
                    res["support_line"][mode][name] = support_line(R, mujoco_contacts=True)[0]
        res["chain"] = self.chain()
        res["figs"] = self.core_figs(res)
        return res

    def chain(self):
        """Checks of the measurement chain on the forward 0.70 runs."""
        Wr, Cr = self.cache["core", "wbc", "fwd070"], self.cache["core", "wbc", "circle"]
        w = Wr.win(Wr.datum + RAMP + SETTLE, D_KEY)
        wc = Cr.win(Cr.datum + RAMP + SETTLE, D_KEY)
        # GRF sign and leg order: desired Fz of leg a against simulated Fz of leg b.
        corr = [[r(np.corrcoef(Wr.f_des[w, 3 * a + 2], Wr.grf[w, 3 * b + 2])[0, 1], 3) for b in range(4)]
                for a in range(4)]
        st = w & (Wr.C[:, 0] == 1)
        dpos = np.gradient(Wr.qpos[:, 0:3], Wr.t, axis=0)
        out = {"fz_corr_desired_leg_vs_simulated_leg": corr,
               "fl_stance_mean_desired_vs_simulated_N": [Wr.f_des[st, 0:3].mean(0).round(2).tolist(),
                                                         Wr.grf[st, 0:3].mean(0).round(2).tolist()],
               "qvel_linear_vs_d_qpos_dt_rms": np.sqrt(np.mean((dpos[w] - Wr.D["MdlSimDriver_qvel"][w, 0:3]) ** 2, 0))
               .round(5).tolist(),
               # Body axes: on the circle (0.35 m/s at 0.5 rad/s) the world-axis mean velocity averages out.
               "circle_mean_v_body_xy_vs_world_xy": [Cr.v_body[wc, :2].mean(0).round(4).tolist(),
                                                     Cr.D["MdlSimDriver_qvel"][wc, :2].mean(0).round(4).tolist()]}
        # Logged foot ground reactions against momentum balance: over the steady window, the summed GRF less what
        # the CoM's momentum rate and gravity require [N] (zero for an exact reading; no other external force).
        out["grf_logged_minus_momentum_N"] = {}
        for mode, name in (("baseline", "fwd070"), ("wbc", "fwd070"), ("baseline", "inplace"), ("wbc", "inplace")):
            R = self.cache["core", mode, name]
            w = R.win(R.datum + RAMP + SETTLE, D_KEY)
            need = MASS * (np.gradient(R.vc, R.t, axis=0)[w].mean(0) - _M.opt.gravity)
            out["grf_logged_minus_momentum_N"][f"{mode}_{name}"] = (
                R.grf[w].reshape(-1, 4, 3).sum(1).mean(0) - need).round(3).tolist()
        # Swing error: the WBC's logged p_B_des - p_B against the baseline's
        # definition (footref minus truth FK) on one WBC log carrying both.
        R = self.run("core", "fwd070_chain", "wbc", {"vx": 0.7, "vy": 0.0, "yaw": 0.0, "period": 0.3},
                     logvars=VARS_CHAIN)
        wc = R.win(R.datum + RAMP + SETTLE, D_KEY)
        d = np.concatenate([(R.W[:, 31 + 3 * k:34 + 3 * k] - (R.D["MdlTrot_footref"][:, 9 * k:9 * k + 3]
                                                              - R.foot_body[:, k]))[wc & (R.C[:, k] == 0)]
                            for k in range(4)])
        out["swing_err_logged_minus_footref_fk_mm"] = {"rms": r(1e3 * amb.rms(np.linalg.norm(d, axis=1)), 4),
                                                        "max": r(1e3 * np.abs(d).max(), 4)}
        return out

    def core_figs(self, res):
        names = [c[0] for c in CORE]
        runs = res["runs"]
        f, ax = amb.figure(2, 3, 11.5, sharex=True)
        x = np.arange(len(names))
        for a, get, lab in ((ax[0, 0], lambda m: m["v_err"], "|mean v - command| [m/s]"),
                            (ax[0, 1], lambda m: m["yaw_err"], "|mean yaw rate - command| [rad/s]"),
                            (ax[0, 2], lambda m: m["swing_err_mm"]["rms"], "swing-foot error rms [mm]"),
                            (ax[1, 0], lambda m: m["roll_rms"], "roll rms [deg]"),
                            (ax[1, 1], lambda m: m["pitch_rms"], "pitch rms [deg]"),
                            (ax[1, 2], lambda m: m["pitch_mean"], "pitch mean [deg] (+: nose down)")):
            for k, mode in enumerate(MODES):
                a.plot(x + (k - 0.5) * 0.2, [get(runs[mode][n]) for n in names], "o", ms=4,
                       color=COL["off" if mode == "baseline" else "on"], label=mode)
            a.set_ylabel(lab)
        for a, key in ((ax[0, 0], "v_tol"), (ax[0, 1], "yaw_tol")):
            a.plot(x, [runs["wbc"][n][key] for n in names], "_", ms=12, color=COL["ink"], label="gate tolerance")
            a.set_ylim(0, None)
        ax[1, 2].axhline(0.0, color=COL["ink"], lw=0.6)
        for a in ax[1]:
            a.set_xticks(x, names, rotation=40, ha="right")
        ax[0, 2].legend(loc="center right")
        ax[0, 1].legend(loc="center right", handles=ax[0, 1].get_lines()[-1:])
        figs = [self.save(f, "core_tracking_attitude.png")]
        # support-line check: dL/dt against M g c_h
        f, ax = amb.figure(1, 2, 7)
        R = self.cache["core", "wbc", "fwd070"]
        pts = support_line(R)[1]
        for key, c, lab in (("swing_contact", COL["off"], "a swing foot in contact"),
                            ("instantaneous", COL["on"], "swing feet free"),
                            ("frozen", COL["FL"], "the same, line frozen at stance start")):
            ax[0, 0].plot(*pts[key].T, ".", ms=1.5, color=c, label=lab, rasterized=True)
        ax[0, 0].axline((0, 0), slope=1, color=COL["ink"], lw=0.8, ls="--", label="slope 1")
        ax[0, 0].set(xlabel="$M g c_h$ [N m]", ylabel="$dL/dt$ [N m]", title="wbc, forward 0.70: two-foot stance",
                     ylim=(-8, 8))
        ax[0, 0].legend(loc="upper left", markerscale=6)
        wt = R.win(R.datum + 8.0, R.datum + 8.9)
        ax[0, 1].plot(R.t[wt] - R.datum, R.L[wt], color=COL["on"], label="L [kg m$^2$/s]")
        ax[0, 1].plot(R.t[wt] - R.datum, 10 * R.ch[wt], color=COL["ink"], ls="--", label="$c_h$ [cm]")
        ax[0, 1].set(xlabel="t from TROT entry [s]", title="about the current stance diagonal")
        ax[0, 1].legend(loc="upper right")
        figs.append(self.save(f, "support_line_check.png"))
        return figs

    # -- diagnosis ---------------------------------------------------------------
    def core_run(self, mode, name):
        """A core run from the cache, else from its log (core must have run once)."""
        c = next(c for c in CORE if c[0] == name)
        return self.cache.get(("core", mode, name)) or self.run(
            "core", name, mode, {"vx": c[1], "vy": c[2], "yaw": c[3], "period": 0.3}, c[4], rerun=False)

    def diagnosis(self):
        """Targeted runs and log analyses behind the tracking gate's miss (the WBC's nose-up pitch offset at speed)
        and the early touchdown. The variants set keys outside [trot] wbc_* for diagnosis only; trot.toml keeps
        its values."""
        B = {c[0]: metrics(self.core_run("baseline", c[0])) for c in CORE}
        W = {c[0]: self.core_run("wbc", c[0]) for c in CORE}

        def row(R, base):
            m = metrics(R)
            if m["fall_t"] is not None or m["error"] is not None:
                return {"fall_t": m["fall_t"], "error": m["error"]}
            out = {k: m[k] for k in ("pitch_mean", "pitch_sd", "pitch_rms", "roll_rms", "v_err", "vx", "z_mean")}
            out.update(swing_err_mm=m["swing_err_mm"]["rms"], swing_loaded_pct=m["touchdown"]["swing_loaded_pct"],
                       ratio_to_baseline={k: r(m[k] / base[k], 3) for k in ("pitch_rms", "roll_rms", "v_err")},
                       force_balance=force_balance(R), wbc_closing=m["wbc_closing"])
            return out

        def variant(tag, extra, names=[c[0] for c in CORE], mode="wbc"):
            out = {}
            for c in CORE:
                if c[0] in names:
                    R = self.run("diagnosis", f"{c[0]}_{tag}", mode, {"vx": c[1], "vy": c[2], "yaw": c[3],
                                                                      "period": 0.3}, c[4] + extra)
                    out[c[0]] = row(R, B[c[0]])
            return out

        res = {"variants": {"nopos": NO_POS_ERR, "kppos0": "trot.wbc_kp_pos = 0, trot.wbc_kd_pos = 0",
                            "joint20": "trot.wbc_joint_kp = 20, trot.wbc_joint_kd = 2",
                            "comshift": f"mpc.com_offset_body x + {1e3 * COM_SHIFT:.0f} mm"}}
        # 1. Remove the base position error: the stance joint PD then has no horizontal reference shift to push
        # toward. The baseline never uses the horizontal reference (the MPC weights position 0): control run.
        res["no_position_error"] = variant("nopos", [NO_POS_ERR])
        ctl = row(self.run("diagnosis", "fwd070_nopos", "baseline", {"vx": 0.7, "vy": 0.0, "yaw": 0.0, "period": 0.3},
                           [NO_POS_ERR]), B["fwd070"])
        res["no_position_error_baseline_control"] = dict(fwd070=ctl, identical_to_core=all(
            ctl[k] == B["fwd070"][k] for k in ("pitch_mean", "pitch_sd", "roll_rms", "v_err", "vx", "z_mean")))
        # 2. Acceleration-stage position task off; the kinematic stage still shifts q_des by the position error.
        res["position_task_gains_zero"] = variant("kppos0", ["trot.wbc_kp_pos = 0.0", "trot.wbc_kd_pos = 0.0"],
                                                  ["fwd070"])
        # 3. Tuning round 1 (rejected): a softer joint PD pushes less per unit of position error.
        res["joint_pd_20_2"] = variant("joint20", ["trot.wbc_joint_kp = 20.0", "trot.wbc_joint_kd = 2.0"])
        # 4. Static pitch stiffness: the MPC's CoM belief moved forward puts M g dx nose-up about the true CoM.
        res["pitch_stiffness"] = {}
        for mode in MODES:
            R = self.run("diagnosis", "inplace_comshift", mode, {"vx": 0.0, "vy": 0.0, "yaw": 0.0, "period": 0.3},
                         [f"mpc.com_offset_body = [{COM_OFFSET[0] + COM_SHIFT:.5f}, {COM_OFFSET[1]}, {COM_OFFSET[2]}]"])
            p0, p1 = (B["inplace"] if mode == "baseline" else metrics(W["inplace"]))["pitch_mean"], metrics(R)["pitch_mean"]
            res["pitch_stiffness"][mode] = {"pitch_mean_core": p0, "pitch_mean_shifted": p1,
                                            "moment_Nm": r(-MASS * 9.81 * COM_SHIFT, 4),
                                            "stiffness_Nm_per_deg": r(MASS * 9.81 * COM_SHIFT / (p0 - p1), 3)}
        # 5. Estimator position against its velocity: the position loop holds the estimated position on the
        # reference, the MPC holds the estimated velocity on the command.
        res["estimator"] = {mode: estimator_rates(self.run(
            "diagnosis", "fwd070_est", mode, {"vx": 0.7, "vy": 0.0, "yaw": 0.0, "period": 0.3},
            ["posvelestimator.use_ground_truth = false"], TRUTH + ["MdlPosVelEstimator_state"])) for mode in MODES}
        # 6. Analyses of the core logs.
        fwd = ("inplace", "fwd035", "fwd070", "fwd100")
        speed = {n: next(c[1] for c in CORE if c[0] == n) for n in fwd}
        pm = {m: {n: (B[n] if m == "baseline" else metrics(W[n]))["pitch_mean"] for n in fwd} for m in MODES}
        A = np.array([[speed[n], m == "baseline", 1.0] for m in MODES for n in fwd])
        y = np.array([pm[m][n] for m in MODES for n in fwd])
        k = np.linalg.lstsq(A, y, rcond=None)[0]
        res["pitch_vs_speed"] = {"pitch_mean": pm, "deg_per_m_s": r(k[0], 4), "baseline_offset_deg": r(k[1], 4),
                                 "intercept_deg": r(k[2], 4), "rms_residual_deg": r(amb.rms(A @ k - y), 4)}
        res["force_balance"] = {m: {c[0]: force_balance(self.core_run(m, c[0])) for c in CORE} for m in MODES}
        res["touchdown"] = {m: {n: touchdown_detail(self.core_run(m, n)) for n in fwd} for m in MODES}
        res["touchdown"]["wbc_nopos"] = {n: touchdown_detail(self.cache["diagnosis", "wbc", f"{n}_nopos"]) for n in fwd}
        res["leg_gravity_baseline"] = {n: leg_gravity(self.core_run("baseline", n)) for n in ("inplace", "fwd070")}
        res["figs"] = [self.diagnosis_fig(res, fwd, speed)]
        return res

    def diagnosis_fig(self, res, fwd, speed):
        f, ax = amb.figure(1, 3, 6.5)
        v = [speed[n] for n in fwd]
        for lab, ys, c, ls in (("baseline", [res["pitch_vs_speed"]["pitch_mean"]["baseline"][n] for n in fwd],
                                COL["off"], "-"),
                               ("wbc", [res["pitch_vs_speed"]["pitch_mean"]["wbc"][n] for n in fwd], COL["on"], "-"),
                               ("wbc, no position error", [res["no_position_error"][n]["pitch_mean"] for n in fwd],
                                COL["on"], "--"),
                               ("wbc, joint PD 20/2", [res["joint_pd_20_2"][n]["pitch_mean"] for n in fwd],
                                COL["FL"], ":")):
            ax[0, 0].plot(v, ys, "o" + ls, ms=3.5, color=c, label=lab)
        ax[0, 0].axhline(0.0, color=COL["ink"], lw=0.6)
        ax[0, 0].set(xlabel="commanded forward speed [m/s]", ylabel="pitch mean [deg] (+: nose down)")
        ax[0, 0].legend(loc="upper right", fontsize=6)
        # FL around its scheduled touchdown at 0.70 m/s, averaged over the steady window's touchdowns.
        for mode in MODES:
            R = self.core_run(mode, "fwd070")
            ref = (R.D["MdlTrot_footref"][:, 0:3] if mode == "baseline" else R.W[:, 31:34] + R.foot_body[:, 0])
            zref = R.qpos[:, 2] + np.einsum("nij,nj->ni", R.rot, ref)[:, 2] - FOOT_R
            w = R.win(R.datum + RAMP + SETTLE, D_KEY)
            js = np.nonzero(w[1:] & (R.C[:-1, 0] == 0) & (R.C[1:, 0] == 1))[0] + 1
            dt = R.t[1] - R.t[0]
            sl = np.arange(-round(0.04 / dt), round(0.02 / dt) + 1)   # 40 ms before to 20 ms after
            tt, c = 1e3 * sl * dt, COL["off" if mode == "baseline" else "on"]
            ax[0, 1].plot(tt, 1e3 * np.mean([zref[j + sl] for j in js], 0), "--", color=c, label=f"{mode} reference")
            ax[0, 1].plot(tt, 1e3 * np.mean([R.cp[j + sl, 0, 2] for j in js], 0), color=c, label=f"{mode} foot")
            ax[0, 2].plot(tt, np.mean([R.grf[j + sl, 2] for j in js], 0), color=c, label=mode)
        ax[0, 1].axhline(0.0, color=COL["ink"], lw=0.6)
        ax[0, 1].set(xlabel="t from scheduled touchdown [ms]", ylabel="FL contact point height [mm]", ylim=(-16, 25))
        ax[0, 1].legend(loc="upper right", fontsize=6)
        ax[0, 2].set(xlabel="t from scheduled touchdown [ms]", ylabel="FL simulated $F_z$ [N]")
        ax[0, 2].legend(loc="upper left", fontsize=6)
        return self.save(f, "diagnosis.png")

    # -- duty --------------------------------------------------------------------
    def duty(self):
        res = {m: [] for m in MODES}
        P = PERIODS[0]
        for mode in MODES:
            for vx in DUTY_SPEEDS:
                for d in DUTIES:
                    R = self.run("duty", f"v{vx:.2f}_d{d:.2f}", mode,
                                 {"vx": vx, "vy": 0.0, "yaw": 0.0, "period": P, "duty": d}, [f"trot.duty = {d}"])
                    row = dict(duty=d, vx_cmd=vx, period=P, flight_scheduled_ms=r(1e3 * max(0.5 - d, 0.0) * P, 2),
                               **metrics(R))
                    if "window_s" in row:   # no fall, no ERROR: time without any foot on the ground, per half cycle
                        w = R.win(*row["window_s"])
                        row["airborne_ms"] = r(1e3 * 0.5 * P * np.mean((R.grf[w, 2::3] < CONTACT).all(1)), 2)
                    res[mode].append(row)
        return res

    # -- cadence -----------------------------------------------------------------
    def cadence(self):
        res = {m: [] for m in MODES}
        for mode in MODES:
            for P in PERIODS:
                R = self.run("cadence", f"period{P:.1f}", mode, {"vx": 0.0, "vy": 0.0, "yaw": 0.0, "period": P})
                row = dict(period=P, stance_s=DUTY * P, **metrics(R))
                row["failed"] = row["fall_t"] is not None or row["error"] is not None
                row["support_line"] = None if row["failed"] else support_line(R)[0]
                events = {k: row[k] for k in ("fall_t", "error_t", "first_saturated_t") if row[k] is not None}
                row["first_event"] = min(events, key=events.get) if events else None
                res[mode].append(row)
                if row["failed"]:
                    break
        f, ax = amb.figure(2, 2, 10, sharex=True)
        for mode in MODES:
            rows, c = res[mode], COL["off" if mode == "baseline" else "on"]
            ok = [x for x in rows if not x["failed"]]
            for a, get in ((ax[0, 0], lambda x: x["roll_rms"]), (ax[0, 1], lambda x: x["pitch_rms"]),
                           (ax[1, 0], lambda x: x["support_line"]["peak_L"]["mean"]),
                           (ax[1, 1], lambda x: x["support_line"]["ch_travel_mm"]["mean"])):
                a.plot([x["period"] for x in ok], [get(x) for x in ok], "o-", ms=4, color=c, label=mode)
            for x in rows:
                if x["failed"]:
                    for a in ax.ravel():
                        a.axvline(x["period"], color=c, ls=":", lw=1.0)
        for a, lab in zip(ax.ravel(), ("roll rms [deg]", "pitch rms [deg]", "peak |L| per stance, mean [kg m$^2$/s]",
                                        "$c_h$ travel per stance, mean [mm]")):
            a.set_ylabel(lab)
        for a in ax[1]:
            a.set_xlabel("gait period [s], stance half of it")
        ax[0, 0].legend(loc="upper left")
        res["figs"] = [self.save(f, "cadence_sweep.png")]
        return res

    # -- push --------------------------------------------------------------------
    def push_run(self, mode, name, sign, F, tp, extra=()):
        """One in-place trot with a lateral pulse at tp; failure is a fall or a
        module ERROR after the pulse."""
        R = self.run("push", f"{name}_F{F}", mode, {"vx": 0.0, "vy": 0.0, "yaw": 0.0, "period": 0.3},
                     [f"simulation.push_force = [0.0, {sign * F:.1f}, 0.0]", f"simulation.push_duration = {PULSE}",
                      f"simulation.push_time = {tp:.4f}", "simulation.push_ramp = 0.0"] + list(extra), TRUTH,
                     start=tp - 1.0)
        del self.cache["push", mode, f"{name}_F{F}"]   # read once here; ~170 of them would hold gigabytes
        o, w = R.outcome(start=tp), R.trot_window(tp)
        td = np.nonzero((R.C[:-1, 0] == 0) & (R.C[1:, 0] == 1) & (R.t[1:] <= tp + 1e-9))[0]   # FL-RR touchdowns
        return dict(F=F, success=o["fall_t"] is None and o["error"] is None,
                    kind="fall" if o["fall_t"] else ("error" if o["error"] else None),
                    fall_after_push_s=o["fall_t"] and r(o["fall_t"] - tp, 4), error=o["error"],
                    error_after_push_s=o["error_t"] and r(o["error_t"] - tp, 4), warnings=o["warnings"],
                    peak_roll_deg=r(np.degrees(np.abs(R.rpy[w, 0]).max()), 3), min_z=r(R.z[w].min(), 4),
                    # to the end of the log: after an ERROR the Supervisor sits the robot, which can roll it over
                    peak_roll_to_end_deg=r(np.degrees(np.abs(R.rpy[R.t >= tp, 0]).max()), 3),
                    peak_vy_body=r(np.abs(R.v_body[w, 1]).max(), 4), peak_tau_over_limit=o["peak_tau_over_limit"],
                    first_saturated_after_push_s=o["first_saturated_t"] and r(o["first_saturated_t"] - tp, 4),
                    # signed: a switch logged one tick after tp reads as a whole period minus 1 ms
                    push_s_after_flrr_touchdown=r((tp - R.t[td[-1] + 1] + 0.15) % 0.3 - 0.15, 4) if len(td) else None,
                    wbc_closing=o["wbc_closing"])

    def sweep(self, mode, name, sign, tp, extra=(), lo=0):
        """From lo (a force known to succeed; 0: none), COARSE steps to the first failure, then FINE steps up from
        the last success: bracket [largest success below the failure, first failure]."""
        rows, fail, F = {}, None, lo + COARSE
        while F <= PUSH_CAP and fail is None:
            rows[F] = self.push_run(mode, name, sign, F, tp, extra)
            fail = None if rows[F]["success"] else F
            F += COARSE
        F = max([x for x in rows if x < (fail or 0)], default=lo) + FINE
        while fail and F < fail:
            rows[F] = self.push_run(mode, name, sign, F, tp, extra)
            fail = fail if rows[F]["success"] else F
            F += FINE
        ok = max([x for x in rows if rows[x]["success"] and (fail is None or x < fail)], default=lo or None)
        return {"bracket_N": [ok, fail], "rows": [rows[x] for x in sorted(rows)]}

    def push(self):
        """Per mode, sign and phase: the bracket of the first failure."""
        res = {"datum_s": {}, "push_times_s": {}}
        for mode in MODES:
            # The gait datum comes from the core in-place run (same script, same stand).
            R0 = self.cache.get(("core", mode, "inplace")) or self.run(
                "core", "inplace", mode, {"vx": 0.0, "vy": 0.0, "yaw": 0.0, "period": 0.3}, rerun=False)
            res["datum_s"][mode] = r(R0.datum, 4)
            tps = {ph: R0.datum + PUSH_AT * 0.3 + off for ph, off in PHASES.items()}
            res["push_times_s"][mode] = {ph: r(tp, 4) for ph, tp in tps.items()}
            for sign, sname in ((1, "plus_y"), (-1, "minus_y")):
                for ph, tp in tps.items():
                    name = f"{sname}_{ph}"
                    res.setdefault(mode, {})[name] = self.sweep(mode, name, sign, tp)
        return res


def gates(res):
    G = []

    def gate(name, crit, ok, value, result=None, cause=None):
        G.append({"gate": name, "criterion": crit, "pass": bool(ok), "result": result or ("pass" if ok else "FAIL"),
                  "value": value})
        if cause:
            G[-1]["cause"] = cause

    if "tests" in res:
        t = res["tests"]
        gate("tests", "all ctest tests pass, test_trot_wbc among them: (i) four feet at rest keep consistent forces "
                      "and get the static torques; (ii) diagonal stance holds the contact points, solved forces inside "
                      "the pyramid and normal bounds; (iii) with the body off its reference the swing joint references "
                      "and acceleration command are the body-frame ones; (iv) swing joints accelerate as the "
                      "body-frame law asks, in a flight phase too, where the base falls freely", t["total"] and t["passed"] == t["total"] and t["returncode"] == 0
             and t["trot_wbc_passed"], f"{t['passed']}/{t['total']} passed, test_trot_wbc "
                                       f"{'passed' if t['trot_wbc_passed'] else 'NOT passed'}")
    if "oldpath" in res:
        o, ab = res["oldpath"], res.get("amb_ablation")
        gate("old path", "wbc absent: the default trot run's log text-identical, sample for sample, to the pre-work "
                         "baseline over the samples both hold (the new run logs every tick and is matched at the "
                         "baseline's 10-tick stride from its first sample, because the log thread starts and stops "
                         "on wall-clock-dependent ticks); AMB ablation (figs/amb_wbc, --only ablation) unchanged "
                         "against the committed reports/amb_wbc/results.json",
             o["identical"] and o["samples_common"] > 1000 and bool(ab) and ab["unchanged"],
             f"{o['samples_common']} of the baseline's {o['samples_baseline']} samples " + (
                 "identical" if o["identical"] else "not matched" if not o["first_sample_found"] else
                 f"differ from sample {o['first_differing_sample']}") + "; AMB ablation: " + (
                 "not run" if not ab else
                 f"{ab['values_compared']} per-run values, " + ("all equal" if not ab["differing"] else
                                                                f"{len(ab['differing'])} differ") +
                 f", gate {ab['gate_ref']['result']} before and {ab['gate_new']['result']} now"))
    if "core" in res:
        C = res["core"]["runs"]
        bad = [f"{m} {n} (fall {x['fall_t']}, error {x['error']}, stop {x['stop_completed']})"
               for m in MODES for n, x in C[m].items() if x["fall_t"] is not None or x["error"] or
               not x["stop_completed"]]
        gate("trots", "every core run, both modes: no fall, no module ERROR, stop completes to the sit", not bad,
             f"all {sum(len(C[m]) for m in MODES)} runs clean" if not bad else "failing: " + ", ".join(bad))
        W, B = C["wbc"], C["baseline"]
        miss, tol = [], []
        for n, x in W.items():
            b = B[n]
            if x["v_err"] > x["v_tol"]:
                tol.append(f"{n}: |v err| {x['v_err']:.3f} > {x['v_tol']:.3f} m/s")
            if x["yaw_err"] > x["yaw_tol"]:
                tol.append(f"{n}: |yaw-rate err| {x['yaw_err']:.3f} > {x['yaw_tol']:.3f} rad/s")
            for k in ("roll_rms", "pitch_rms", "v_err") + (("yaw_err",) if x["cmd"][2] else ()):
                if not ratio_ok(x[k], b[k]):
                    miss.append(f"{n}: {k} {x[k]:.3f} vs baseline {b[k]:.3f} ({x[k] / b[k]:.2f}x)")
        cause = None
        if miss and not tol and "diagnosis" in res:
            d, runs = res["diagnosis"], sorted({m.split(":")[0] for m in miss})
            nopos, k0 = d["no_position_error"], d["position_task_gains_zero"]["fwd070"]
            if all(x["ratio_to_baseline"][k] <= 1.5 for x in nopos.values() for k in ("pitch_rms", "roll_rms",
                                                                                         "v_err")):
                cause = ("a static nose-up pitch offset. The joint PD acts on q_des shifted by the base position "
                         "error, which grows with speed (" + ", ".join(
                             f"{d['force_balance']['wbc'][n]['position_error_mm']:.0f} mm {n}" for n in runs) +
                         "). With that error held at zero (" + res["meta"]["logs"] + "/diagnosis, *_nopos) pitch rms "
                         "is " + ", ".join(f"{nopos[n]['ratio_to_baseline']['pitch_rms']:.2f}x ({n})" for n in runs) +
                         " and every core run meets the 1.5x rule; with the position task's gains at zero (*_kppos0) "
                         f"the offset stays ({k0['pitch_mean']:+.2f} deg at fwd070)")
        gate("tracking", "WBC mode: mean velocity within max(10 %, 0.05 m/s), yaw rate within max(10 %, 0.05 rad/s); "
                         "roll rms, pitch rms, velocity error (and yaw-rate error where commanded) at most 1.5x the "
                         "baseline's, or the cause of a miss measured", not miss and not tol,
             "; ".join(tol + miss) or "all within", None if not miss or tol else (
                 "strict rule FAIL; cause measured" if cause else "FAIL"), cause)
        cl = {n: x["wbc_closing"] for n, x in W.items()}
        hl = [f"{n}: {c}" for n, c in cl.items() if not c or c["qp_failures"] or c["fallback_ticks"] or
              c["saturated_pct"] >= 1.0]
        lg = [x["wbc_logged"] for x in W.values() if "wbc_logged" in x]
        hl += [f"logged fallback/QP-status samples {x}" for x in lg if x["fallback_samples"] or
               x["qp_status_nonzero_samples"]]
        cs = [c for c in cl.values() if c]
        gate("health", "WBC mode, every core run: 0 QP failures, 0 fallback ticks (closing line and log), "
                       "torque-saturated ticks < 1 %", not hl, "; ".join(hl) or (
                 f"0 QP failures, 0 fallback ticks over {sum(c['ticks'] for c in cs)} ticks (logged: 0 fallback, 0 "
                 f"nonzero QP status); {sum(c['saturated_ticks'] for c in cs)} saturated ticks; tick mean "
                 f"{min(c['tick_mean_us'] for c in cs):.1f}-{max(c['tick_mean_us'] for c in cs):.1f} us, max "
                 f"{max(c['tick_max_us'] for c in cs):.1f} us against the 1000 us period"))
    if "duty" in res:
        D = res["duty"]
        bad = [f"{m} v{x['vx_cmd']} duty {x['duty']} (fall {x['fall_t']}, error {x['error']}, stop "
               f"{x['stop_completed']})" for m in MODES for x in D[m] if x["fall_t"] is not None or x["error"] or
               not x["stop_completed"]]
        cs = [x["wbc_closing"] for x in D["wbc"]]
        bad += [f"wbc v{x['vx_cmd']} duty {x['duty']}: {c}" for x, c in zip(D["wbc"], cs)
                if not c or c["qp_failures"] or c["fallback_ticks"]]
        gate("duty", f"duty {min(DUTIES)} to {max(DUTIES)} at the default period, in place and forward, both "
                     "modes: no fall, no module ERROR, stop completes; WBC mode: 0 QP failures, 0 fallback ticks "
                     "(below 0.5 the gait has a flight phase, which the whole-body path must solve)", not bad,
             "; ".join(bad) or f"all {sum(len(D[m]) for m in MODES)} runs clean; 0 QP failures, 0 fallback ticks "
                               f"over {sum(c['ticks'] for c in cs)} WBC ticks")
    return G


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", nargs="+", choices=EXPERIMENTS, help="experiments to run (default: all)")
    ap.add_argument("--out", help="root for logs, results and figures (default: runs/trot_wbc, reports/trot_wbc)")
    a = ap.parse_args()
    check_no_simulation()   # before ctest's minute, not at the first run
    out = a.out and os.path.abspath(a.out)
    logs = os.path.join(out, "runs") if out else os.path.join(REPO, "runs", "trot_wbc")
    rep = out or os.path.join(REPO, "reports", "trot_wbc")
    os.makedirs(rep, exist_ok=True)
    only = a.only or EXPERIMENTS
    E = Experiments(logs, rep)
    path = os.path.join(rep, "results.json")
    res = json.load(open(path)) if os.path.exists(path) else {}
    # Wall times per experiment survive a --only re-run of the others.
    walls = res.get("meta", {}).get("experiment_wall_s", {})
    run_walls = {k: v for k, v in res.get("meta", {}).get("run_wall_times_s", {}).items()
                 if k.split("/")[0] not in only}
    for name in EXPERIMENTS:
        if name not in only:
            continue
        print(f"== {name}", flush=True)
        t0 = time.time()
        res[name] = getattr(E, name)()
        walls[name] = round(time.time() - t0, 1)
        res["meta"] = {"script": SCRIPT, "exit_time_s": EXIT, "trot_s": TROT_S, "log_period_ms": LOG_PERIOD,
                       "steady_window": f"TROT entry + {RAMP} s ramp-up + {SETTLE} s to the stop key at {D_KEY} s",
                       "fall": f"base z < {FALL_Z} m or |roll|, |pitch| > 45 deg from TROT entry to the end of "
                               "centering (or to the ERROR tick)", "loaded_foot_N": LOADED, "contact_N": CONTACT,
                       "torque_limit_Nm": TAU_LIM[:3].tolist(),
                       "push": {"pulse_s": PULSE, "at_cycles_after_entry": PUSH_AT, "coarse_N": COARSE,
                                "fine_N": FINE, "cap_N": PUSH_CAP, "phases_s_after_flrr_touchdown": PHASES},
                       "logs": os.path.relpath(logs, REPO), "results": os.path.relpath(path, REPO), "config": CONFIG,
                       "experiments_run": only, "experiment_wall_s": walls,
                       "run_wall_times_s": run_walls | E.wall}
        res["gates"] = gates(res)
        # Saved after every experiment: a later one failing keeps the earlier results.
        with open(path, "w") as f:
            json.dump(res, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print(f"\n{'gate':<12} result\n" + "-" * 60)
    for g in res["gates"]:
        print(f"{g['gate']:<12} {g['result']}\n    {g['value']}")
    print(f"\nwall time {sum(walls[n] for n in only):.0f} s; results in {path}")
    if all(n in res for n in EXPERIMENTS):
        build_report.build(rep)


if __name__ == "__main__":
    main()
