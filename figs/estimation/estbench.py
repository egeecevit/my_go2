#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""State-estimator benchmark: headless simulation scenarios scored against
simulator truth, one table instead of a folder of plots.

  figs/.venv/bin/python figs/estimation/estbench.py all [--only GLOB] [--force]
  figs/.venv/bin/python figs/estimation/estbench.py run|score|report [...]
  figs/.venv/bin/python figs/estimation/estbench.py compare <dirA> <dirB>

Every run is one `bin/sim.sh` invocation driven by a Supervisor key script
(supervisor.script) with the estimator tiers set through -c overrides:

  noise  off|on      [orientationestimator] simnoise.enable
  trust  sched|oracle [posvelestimator] use_contact_oracle (sim foot forces)
  att    filter|truth [orientationestimator] use_ground_truth: who gets the
                      attitude -- the controller AND the position filter
  ctrl   est|truth    [posvelestimator] use_ground_truth: controller on the
                      filter or on simulator truth (shadow mode; the filter
                      runs and is logged either way)

Runs are serialised by a lock (the LogServer binds a fixed port). Output goes
to runs/est/<name>/{run.mat,run.log,manifest.txt,metrics.json,summary.png} and
runs/est/report.md.

Metrics follow the benchmark spec: SO(3) tilt/yaw error, convergence of a
planted tilt, body-frame velocity bias and RMSE with touchdown/liftoff windows,
angular-velocity and gyro-bias error, position drift / ATE / RPE / z RMSE,
foot slip, covariance consistency (3-sigma, NEES, innovations), and the closed
loop (velocity tracking, falls, heading drift).
"""

import argparse
import datetime
import fcntl
import fnmatch
import json
import os
import re
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from plot_log import load_log, quat_rotate, quat_to_rpy  # noqa: E402
except ImportError as exc:  # pragma: no cover
    sys.exit(f"{exc}\nrun this with figs/.venv/bin/python (create it with figs/visualize.sh --setup)")

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "fig-gen"))
from plot_est import plot_log_file  # noqa: E402
from compare_est import compare_dirs  # noqa: E402
BIN = os.path.join(REPO, "bin")
DEFAULT_OUT = os.path.join(REPO, "runs", "est")

# 122 doubles against the hard 128-double ceiling in rtclient::LogClient::addVar
# (overflow or a misspelled name writes NO FILE while the run exits 0):
#   qpos 19 + qvel 18 + orientation state 16 + error 11
#   + posvel state 9 + error 9 + cov 18 + contacts 4 + innov 6 + foottruth 12 = 122
VARS = ["MdlSimDriver_qpos", "MdlSimDriver_qvel",
        "MdlOrientationEstimator_state", "MdlOrientationEstimator_error",
        "MdlPosVelEstimator_state", "MdlPosVelEstimator_error", "MdlPosVelEstimator_cov",
        "MdlPosVelEstimator_contacts", "MdlPosVelEstimator_innov",
        "MdlPosVelEstimator_foottruth"]
LOG_PERIOD = 2          # every 2nd 1 ms cycle: 500 Hz resolves 30 ms trust ramps
FALL_Z = 0.20           # trunk height below which the robot is down (reports/mpc/README.md)
FOOT_RADIUS = 0.022     # go2.xml foot sphere; also posvelestimator.foot_ground_height
TRANSITION_S = 0.015    # half-width of the window around scheduled touchdown and liftoff
GAIT_SETTLE_S = 4.8     # MdlTrot WAIT 0.3 + PREP 1.5 + rampup 2.0 + 1 s margin
STAND_SETTLE_S = 3.0    # MdlStand duration 2.0 + margin

TIER_CFG = {
    "noise": {"off": "orientationestimator.simnoise.enable = false",
              "on": "orientationestimator.simnoise.enable = true"},
    "trust": {"sched": "posvelestimator.use_contact_oracle = false",
              "oracle": "posvelestimator.use_contact_oracle = true"},
    "att": {"filter": "orientationestimator.use_ground_truth = false",
            "truth": "orientationestimator.use_ground_truth = true"},
    "ctrl": {"est": "posvelestimator.use_ground_truth = false",
             "truth": "posvelestimator.use_ground_truth = true"},
}

# script: Supervisor keys; exit: supervisor.exit_time; vx: commanded forward speed.
TROT_SCRIPT = ["2 s", "6 t", "46 d"]
SCENARIOS = {
    "stand60": dict(script=["2 s", "58 d"], exit=64, cfg=[], kind="stand"),
    "sitstand3": dict(script=["2 s", "10 d", "14 s", "22 d", "26 s", "34 d"], exit=40, cfg=[],
                      kind="cycles"),
    "trot_inplace": dict(script=TROT_SCRIPT, exit=52, cfg=["trot.forward_velocity = 0.0"],
                         kind="gait", vx=0.0),
    "trot_v03": dict(script=TROT_SCRIPT, exit=52, cfg=["trot.forward_velocity = 0.3"], kind="gait", vx=0.3),
    "trot_v05": dict(script=TROT_SCRIPT, exit=52, cfg=["trot.forward_velocity = 0.5"], kind="gait", vx=0.5),
    "trot_v08": dict(script=TROT_SCRIPT, exit=52, cfg=["trot.forward_velocity = 0.8"], kind="gait", vx=0.8),
    "trot_yaw": dict(script=TROT_SCRIPT, exit=52,
                     cfg=["trot.forward_velocity = 0.3", "trot.yaw_rate = 0.3"], kind="gait", vx=0.3),
    # Same trot after a 12 s stand: the gyro bias has converged before the gait
    # starts, which is how hardware is used and what the bias gate assumes.
    "trot_v05_long": dict(script=["2 s", "14 t", "54 d"], exit=60, cfg=["trot.forward_velocity = 0.5"],
                          kind="gait", vx=0.5),
    # 5 degrees keeps the integrator peak (0.184 * theta0 = 0.016 rad/s) under the
    # shipped 0.02 rad/s bias clamp, so the analytic second-order decay applies.
    "converge5": dict(script=["2 s", "28 d"], exit=34,
                      cfg=["orientationestimator.simnoise.tilt_probe_deg = 5",
                           "orientationestimator.simnoise.tilt_probe_time = 6"],
                      kind="converge", probe_time=6.0),
}


class Run:
    def __init__(self, scn, noise, trust, att, ctrl, extra=(), tag=""):
        self.scn, self.noise, self.trust, self.att, self.ctrl = scn, noise, trust, att, ctrl
        self.extra = list(extra)
        self.name = f"{scn}-{noise}-{trust}-{att}-{ctrl}" + (f"-{tag}" if tag else "")
        # Shipped values unless a sweep cell overrides them (they label the cell and feed blind_fraction).
        self.H, self.window = shipped_trust()
        self.Rv = None
        for e in self.extra:
            m = re.match(r"posvelestimator\.high_suspect_number\s*=\s*([\d.]+)", e)
            if m:
                self.H = float(m.group(1))
            m = re.match(r"posvelestimator\.trust_window\s*=\s*([\d.]+)", e)
            if m:
                self.window = float(m.group(1))
            m = re.match(r"posvelestimator\.foot_sensor_noise_velocity\s*=\s*([\d.eE+-]+)", e)
            if m:
                self.Rv = float(m.group(1))

    @property
    def scenario(self):
        return SCENARIOS[self.scn]

    def overrides(self):
        s = self.scenario
        return [TIER_CFG["noise"][self.noise], TIER_CFG["trust"][self.trust],
                TIER_CFG["att"][self.att], TIER_CFG["ctrl"][self.ctrl]] + s["cfg"] + self.extra


def run_matrix():
    R = []

    def add(*a, **k):
        R.append(Run(*a, **k))

    add("stand60", "off", "sched", "filter", "truth")
    add("stand60", "on", "sched", "filter", "truth")
    add("sitstand3", "on", "sched", "filter", "truth")
    add("converge5", "off", "sched", "filter", "truth")
    add("converge5", "on", "sched", "filter", "truth")
    add("trot_inplace", "on", "sched", "filter", "truth")
    for s in ("trot_v03", "trot_v05", "trot_v08", "trot_yaw", "trot_v05_long"):
        add(s, "on", "sched", "filter", "truth")
    add("trot_v05", "off", "sched", "filter", "truth")
    add("trot_v05", "off", "oracle", "filter", "truth")
    add("trot_v05", "off", "oracle", "truth", "truth")      # KF alone, on true attitude
    add("trot_v05", "on", "oracle", "filter", "truth")
    for s in ("trot_v03", "trot_v05", "trot_v08"):
        add(s, "on", "sched", "filter", "est")                # closed loop
    for H in (3, 10, 100):
        for w in (0.2, 0.3):
            if H == 100 and w == 0.2:
                continue                                      # that is the baseline cell
            add("trot_v05", "on", "sched", "filter", "truth",
                extra=[f"posvelestimator.high_suspect_number = {H}",
                       f"posvelestimator.trust_window = {w}"], tag=f"H{H}-w{w}")
    # Velocity measurement noise sweep (consistency): the shipped 0.1 is sigma 316 mm/s per leg.
    for Rv in ("1e-2", "1e-3", "1e-4"):
        add("trot_v05", "on", "sched", "filter", "truth",
            extra=[f"posvelestimator.foot_sensor_noise_velocity = {Rv}"], tag=f"Rv{Rv}")
        add("trot_v05", "off", "sched", "filter", "truth",
            extra=[f"posvelestimator.foot_sensor_noise_velocity = {Rv}"], tag=f"Rv{Rv}")
    add("stand60", "on", "sched", "filter", "truth",
        extra=["posvelestimator.foot_sensor_noise_velocity = 1e-3"], tag="Rv1e-3")
    add("trot_v05", "on", "sched", "filter", "est",
        extra=["posvelestimator.foot_sensor_noise_velocity = 1e-3"], tag="Rv1e-3")
    return R


# ---------------------------------------------------------------------------
# Running
# ---------------------------------------------------------------------------

def launch(run, out, force=False):
    d = os.path.join(out, run.name)
    os.makedirs(d, exist_ok=True)
    mat = os.path.join(d, "run.mat")
    if os.path.exists(mat) and not force:
        print(f"[skip] {run.name}: run.mat exists")
        return True
    for f in ("run.mat", "metrics.json"):
        if os.path.exists(os.path.join(d, f)):
            os.remove(os.path.join(d, f))

    s = run.scenario
    cfg = ["simulation.headless = true", "simulation.realtime = false",
           f"supervisor.exit_time = {s['exit']}",
           f"supervisor.script = {json.dumps(s['script'])}",
           "supervisor.log.enable = true", "supervisor.log.start = 0",
           f"supervisor.log.period = {LOG_PERIOD}",
           f'supervisor.log.file_name = "{mat}"',
           f"supervisor.log.vars = {json.dumps(VARS)}"] + run.overrides()
    args = ["./sim.sh"]
    for c in cfg:
        args += ["-c", c]

    with open(os.path.join(d, "manifest.txt"), "w") as f:
        git = lambda *a: subprocess.run(["git", "-C", REPO] + list(a), capture_output=True,  # noqa: E731
                                        text=True).stdout.strip()
        kp, ki, tau_lpf = mahony_gains()
        f.write(f"name       {run.name}\ndate       {datetime.datetime.now().isoformat()}\n"
                f"git        {git('rev-parse', '--short', 'HEAD')} ({git('branch', '--show-current')})\n"
                f"dirty      {git('status', '--porcelain').replace(chr(10), ';')}\n"
                f"overrides  {json.dumps(cfg)}\n"
                f"gains      {kp} {ki} {tau_lpf}\n")

    lock = open(os.path.join(out, ".simlock"), "w")
    fcntl.flock(lock, fcntl.LOCK_EX)
    try:
        with open(os.path.join(d, "run.log"), "w") as log:
            try:
                rc = subprocess.run(args, cwd=BIN, stdin=subprocess.DEVNULL, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=3 * s["exit"] + 30).returncode
            except subprocess.TimeoutExpired:
                rc = -1
                log.write("\nestbench: TIMEOUT\n")
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()

    text = open(os.path.join(d, "run.log"), errors="replace").read()
    # Word match, so the variable name MdlPosVelEstimator_error does not count.
    bad = [l for l in text.splitlines() if re.search(r"(?<![_A-Za-z])(error|fatal)(?![_A-Za-z])", l, re.I)]
    ok = rc == 0 and os.path.exists(mat)
    if "t" in [k.split()[1] for k in s["script"]] and "-> S_TROT" not in text:
        bad.append("estbench: trot never started (script timing?)")
        ok = False
    print(f"[{'ok' if ok else 'FAIL'}] {run.name}: exit={rc}" +
          (f", {os.path.getsize(mat) // 1024} kB" if os.path.exists(mat) else
           "  NO .mat WRITTEN -- suspect the 128-double log ceiling or a bad variable name"))
    for l in bad[:5]:
        print("      " + l.strip())
    return ok


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------

def load(path):
    """load_log() plus the absolute sim time of the first sample, so windows
    can be placed by the script's key times."""
    t, data = load_log(path)
    try:
        from scipy.io import loadmat
        raw = loadmat(path, squeeze_me=True, struct_as_record=False)
        off = float(raw["params"].time_offset) * 1e-6
    except Exception:
        off = 0.0
    return t + off, data


def rms(x, axis=0):
    return np.sqrt(np.nanmean(np.square(x), axis=axis))


def key_times(script):
    return [(float(k.split()[0]), k.split()[1]) for k in script]


def analysis_window(run, t):
    """[t0, t1] in sim time over which steady-state metrics are taken."""
    s = run.scenario
    keys = key_times(s["script"])
    kind = s["kind"]
    if kind == "stand":
        return keys[0][0] + STAND_SETTLE_S, keys[1][0] - 0.5
    if kind == "gait":
        tt = [k for k, c in keys if c == "t"][0]
        td = [k for k, c in keys if c == "d"][-1]
        return tt + GAIT_SETTLE_S, td - 0.2
    if kind == "converge":
        return s["probe_time"], keys[-1][0] - 0.5
    return keys[0][0], s["exit"] - 2.0           # cycles: everything while active


def active_mask(PE):
    """PosVel is deactivated while sitting and its log buffers freeze; drop
    samples identical to their predecessor."""
    moving = np.ones(len(PE), bool)
    moving[1:] = np.any(np.diff(PE, axis=0) != 0.0, axis=1)
    return moving


def shipped_trust():
    """high_suspect_number and trust_window as shipped in config/default/stateestimator.toml."""
    text = open(os.path.join(REPO, "config", "default", "stateestimator.toml")).read()
    get = lambda k, d: float(re.search(rf"^{k}\s*=\s*([\d.eE+-]+)", text, re.M).group(1)) if re.search(rf"^{k}\s*=", text, re.M) else d  # noqa: E731
    return get("high_suspect_number", 100.0), get("trust_window", 0.2)


def mahony_gains(run_dir=None):
    """mahony_kp, mahony_ki, accel_filter_tau: from the run's manifest when it
    has them (the gains the run actually used), else from the shipped toml."""
    if run_dir:
        try:
            for line in open(os.path.join(run_dir, "manifest.txt")):
                if line.startswith("gains"):
                    return tuple(float(x) for x in line.split()[1:4])
        except OSError:
            pass
    text = open(os.path.join(REPO, "config", "default", "stateestimator.toml")).read()
    get = lambda k: float(re.search(rf"^{k}\s*=\s*([\d.eE+-]+)", text, re.M).group(1))  # noqa: E731
    return get("mahony_kp"), get("mahony_ki"), get("accel_filter_tau")


def predicted_decay(theta0, kp, ki, tau, t):
    """Linearised Mahony error loop for a stationary body: tilt error e, bias
    integrator b, and the low-passed gravity reference's own error r, which the
    filter drags along at -b because it transports the reference with the
    bias-corrected rate:  e' = -kp (e - r) - b,  b' = ki (e - r),  r' = -r/tau - b.
    Integrated at 1 ms; matches the C++ filter to 0.1 % of theta0."""
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


def first_crossing(t, x, level):
    idx = np.nonzero(x <= level)[0]
    return float(t[idx[0]]) if len(idx) else float("nan")


def score(run, out):
    d = os.path.join(out, run.name)
    mat = os.path.join(d, "run.mat")
    if not os.path.exists(mat):
        return None
    t, data = load(mat)
    need = {"qpos": "MdlSimDriver_qpos", "qvel": "MdlSimDriver_qvel",
            "OS": "MdlOrientationEstimator_state", "OE": "MdlOrientationEstimator_error",
            "PS": "MdlPosVelEstimator_state", "PE": "MdlPosVelEstimator_error",
            "COV": "MdlPosVelEstimator_cov", "CON": "MdlPosVelEstimator_contacts",
            "INN": "MdlPosVelEstimator_innov", "FT": "MdlPosVelEstimator_foottruth"}
    V = {k: data[v] for k, v in need.items() if v in data}
    missing = [v for k, v in need.items() if v not in data]
    M = {"name": run.name, "scenario": run.scn, "noise": run.noise, "trust": run.trust,
         "att": run.att, "ctrl": run.ctrl, "H": run.H, "trust_window": run.window,
         "missing": missing}
    if missing:
        print(f"[score] {run.name}: missing {missing}")
        return M

    t0, t1 = analysis_window(run, t)
    w = (t >= t0) & (t <= t1)
    M["window"] = [t0, t1]
    dt = float(np.median(np.diff(t)))
    deg = 180.0 / np.pi
    qpos, qvel, OS, OE, PS, PE, COV, CON, INN, FT = (V[k] for k in
                                                   ("qpos", "qvel", "OS", "OE", "PS", "PE", "COV", "CON", "INN", "FT"))
    q_true = qpos[:, 3:7]
    act = active_mask(PE) & w

    # -- orientation (filter vs truth, in-module, same tick) -----------------
    # OE: [0:3] rotation vector, [3] tilt angle (what gravity observes),
    # [4] heading error about the vertical, [5:8] body-rate error, [8:11] bias error.
    tilt = OE[:, 3]
    M["tilt_rms_deg"] = float(rms(tilt[w]) * deg)
    M["tilt_max_deg"] = float(tilt[w].max() * deg)
    slope = np.polyfit(t[w], np.unwrap(OE[w, 4]), 1)[0]
    M["yaw_drift_deg_per_min"] = float(slope * deg * 60.0)
    # What the unlearned bias predicts for the heading drift: the bias error
    # projected on the vertical (the body is pitched in the stand pose).
    up_true = quat_rotate(q_true, np.array([0.0, 0.0, 1.0]), inverse=True)
    M["yaw_drift_expected_deg_per_min"] = float(-np.mean(np.sum(OE[w, 8:11] * up_true[w], axis=1)) * deg * 60.0)
    b_inj = OS[:, 13:16] - OE[:, 8:11]
    M["injected_bz_deg_per_min"] = float(np.mean(b_inj[w, 2]) * deg * 60.0)
    # Tilt drift: slope of the tilt angle over the window, ~0 once the bias is learned.
    M["tilt_slope_deg_per_min"] = float(np.polyfit(t[w], tilt[w], 1)[0] * deg * 60.0)

    # -- angular velocity and gyro bias --------------------------------------
    M["omega_err_mean"] = OE[w, 5:8].mean(0).tolist()
    M["omega_err_rms"] = rms(OE[w, 5:8]).tolist()
    tail = w & (t >= t1 - 0.25 * (t1 - t0))
    M["bias_err_steady"] = OE[tail, 8:11].mean(0).tolist()
    M["bias_err_peak"] = np.abs(OE[w, 8:11]).max(0).tolist()
    # Transient kicks: peak |bias error| over the whole active run (trot start/stop included).
    first_key = key_times(run.scenario["script"])[0][0]
    last_down = [k for k, c in key_times(run.scenario["script"]) if c == "d"][-1]
    # The stop transient (gait ramp-down after the last 'd') carries the larger kick.
    run_span = (t >= first_key + 1.0) & (t <= last_down + 5.0)
    M["bias_err_peak_run"] = np.abs(OE[run_span, 8:11]).max(0).tolist()
    M["tilt_max_run_deg"] = float(tilt[run_span].max() * deg)

    # -- velocity (body frame, in-module) ------------------------------------
    M["vel_err_mean"] = PE[act, 3:6].mean(0).tolist()
    M["vel_err_rms"] = rms(PE[act, 3:6]).tolist()
    # Scheduled touchdown/liftoff = trust leaving or entering zero (the ramps start and end
    # exactly at the stance boundaries whatever their width).
    events = np.zeros(len(t), bool)
    for i in range(4):
        c = CON[:, i] > 0.0
        events[1:] |= c[1:] != c[:-1]
    half = int(round(TRANSITION_S / dt))
    trans = np.convolve(events.astype(float), np.ones(2 * half + 1), "same") > 0
    M["n_contact_events"] = int(events[w].sum())
    if events[w].any():
        M["vel_err_rms_transition"] = rms(PE[act & trans, 3:6]).tolist()
        M["vel_err_rms_midstance"] = rms(PE[act & ~trans, 3:6]).tolist()
    # Weighted blind fraction: no leg with effective weight trust/(1+(1-trust)H) >= 0.5.
    # For a scheduled gait this is the closed form 2w(1+H)/(2+H), a property of
    # the (trust_window, H) cell and not of the filter; kept as a descriptor only.
    # (The covariance of this linear KF is data-independent too, so no P-derived
    # quantity can replace it as a performance metric.)
    weff = CON / (1.0 + (1.0 - CON) * run.H)
    M["blind_fraction"] = float(np.mean(weff[w].max(1) < 0.5))
    M["vel_err_max_midstance"] = np.abs(PE[act & ~trans, 3:6]).max(0).tolist() if (act & ~trans).any() else None
    if run.scenario.get("vx", 0.0) > 0.0:
        M["vel_bias_pct_of_speed"] = (100.0 * np.abs(PE[act, 3:6].mean(0)) / run.scenario["vx"]).tolist()

    # -- position -------------------------------------------------------------
    last = np.nonzero(act)[0][-1] if act.any() else len(t) - 1
    dist = PE[last, 7]
    M["path_m"] = float(dist)
    M["xy_err_end_m"] = float(np.linalg.norm(PE[last, 0:2]))
    M["xy_drift_percent"] = float(100.0 * M["xy_err_end_m"] / dist) if dist > 0.25 else float("nan")
    M["ate_m"] = float(rms(np.linalg.norm(PE[act, 0:3], axis=1)))
    M["z_rms_m"] = float(rms(PE[act, 2]))
    # Heading-aligned position error: subtract the integral of (R(psi) - I) v_true,
    # the dead-reckoning leak of an unobservable heading error psi. The raw
    # number is what the controller sees; the aligned one is what a change to
    # the position/velocity filter can move (figs/plot_log.py heading_leak).
    psi = OE[:, 4]
    vt = qvel[:, 0:3]
    dvx = (np.cos(psi) - 1.0) * vt[:, 0] - np.sin(psi) * vt[:, 1]
    dvy = np.sin(psi) * vt[:, 0] + (np.cos(psi) - 1.0) * vt[:, 1]
    leak = np.column_stack([np.cumsum(dvx * act * dt), np.cumsum(dvy * act * dt)])
    aligned_xy = PE[:, 0:2] - leak
    M["xy_err_end_aligned_m"] = float(np.linalg.norm(aligned_xy[last]))
    M["xy_drift_aligned_percent"] = float(100.0 * M["xy_err_end_aligned_m"] / dist) if dist > 0.25 else float("nan")
    M["heading_leak_percent"] = float(100.0 * np.linalg.norm(leak[last]) / dist) if dist > 0.25 else float("nan")
    M["z_err_mean_m"] = float(PE[act, 2].mean())
    k1 = int(round(1.0 / dt))
    idx = np.nonzero(act)[0]
    idx = idx[idx + k1 < len(t)]
    if len(idx):
        d_est = PS[idx + k1, 0:3] - PS[idx, 0:3]
        d_true = qpos[idx + k1, 0:3] - qpos[idx, 0:3]
        M["rpe_xy_1s_m"] = float(rms(np.linalg.norm((d_est - d_true)[:, :2], axis=1)))

    # -- foot slip in trusted stance (truth foot centre, body frame) ----------
    # Raw: what the filter's velocity rows assume is zero. Rolling-corrected:
    # minus r * (omega_calf x z) for a sphere rolling with the calf.
    # MuJoCo joint columns already follow the kinematics' internal sign
    # convention (the driver's polarity and joint_directions cancel), so no
    # JOINT_DIRECTION here; that table applies to the ctrl columns only.
    qj = qpos[:, 7:19].reshape(-1, 4, 3)
    qdj = qvel[:, 6:18].reshape(-1, 4, 3)
    wb = qvel[:, 3:6]
    slip_raw, slip_roll, n_slip = [], [], 0
    for i in range(4):
        trusted = (CON[:, i] > 0.99) & act
        if trusted.sum() < 10:
            continue
        vf = np.gradient(FT[:, 3 * i:3 * i + 3], dt, axis=0)
        q1 = qj[:, i, 0]
        omega_rel = np.column_stack([qdj[:, i, 0], (qdj[:, i, 1] + qdj[:, i, 2]) * np.cos(q1),
                                     (qdj[:, i, 1] + qdj[:, i, 2]) * np.sin(q1)])
        omega_calf_w = quat_rotate(q_true, wb + omega_rel)
        v_roll = FOOT_RADIUS * np.cross(omega_calf_w, np.array([0.0, 0.0, 1.0]))
        vf_b = quat_rotate(q_true, vf, inverse=True)
        vr_b = quat_rotate(q_true, vf - v_roll, inverse=True)
        slip_raw.append(vf_b[trusted]); slip_roll.append(vr_b[trusted]); n_slip += int(trusted.sum())
    if slip_raw:
        sr, sl = np.vstack(slip_raw), np.vstack(slip_roll)
        M["foot_vel_stance_mean"] = sr.mean(0).tolist()
        M["foot_vel_stance_rms"] = rms(sr).tolist()
        M["foot_slip_rollcorr_mean"] = sl.mean(0).tolist()
        M["foot_slip_rollcorr_rms"] = rms(sl).tolist()

    # -- consistency ----------------------------------------------------------
    e_w = quat_rotate(q_true, PE[:, 3:6])             # body-frame error back to world
    var_v = np.maximum(COV[:, 3:6], 1e-12)
    var_z = np.maximum(COV[:, 2], 1e-12)
    M["frac_in_3sigma_v"] = float(np.mean(np.all(np.abs(e_w[act]) <= 3.0 * np.sqrt(var_v[act]), axis=1)))
    M["frac_in_3sigma_z"] = float(np.mean(np.abs(PE[act, 2]) <= 3.0 * np.sqrt(var_z[act])))
    nees_v = np.sum(np.square(e_w) / var_v, axis=1)
    nees_z = np.square(PE[:, 2]) / var_z
    M["nees_v_mean"] = float(nees_v[act].mean())
    M["nees_z_mean"] = float(nees_z[act].mean())
    # Bias-removed NEES: a biased, underconfident filter can land near 3 by
    # accident; the centred value judges the covariance against the noise.
    e_c = e_w - e_w[act].mean(0)
    M["nees_v_centered"] = float(np.sum(np.square(e_c[act]) / var_v[act], axis=1).mean())
    M["sigma_v_mean"] = np.sqrt(var_v[act]).mean(0).tolist()
    tr = act & (INN[:, 5] > 0)
    if tr.any():
        n = int(tr.sum())
        M["innov_v_mean"] = INN[tr, 0:3].mean(0).tolist()
        M["innov_v_std"] = INN[tr, 0:3].std(0).tolist()
        M["innov_v_zscore"] = (INN[tr, 0:3].mean(0) / (INN[tr, 0:3].std(0) / np.sqrt(n) + 1e-12)).tolist()
        M["innov_z_mean"] = float(INN[tr, 3].mean())
        M["nis_v_mean"] = float(INN[tr, 4].mean())

    # -- closed loop ------------------------------------------------------------
    vb_true = quat_rotate(q_true, qvel[:, 0:3], inverse=True)
    yaw_true = quat_to_rpy(q_true)[:, 2]
    # Up on its feet from each stand until its scripted sit; a sit is not a fall.
    up = np.zeros(len(t), bool)
    keys = key_times(run.scenario["script"])
    for j, (ts, c) in enumerate(keys):
        if c != "s":
            continue
        td = next((k for k, cc in keys[j + 1:] if cc == "d"), run.scenario["exit"])
        up |= (t >= ts + 2.5) & (t <= td)
    M["fall"] = bool(np.any(qpos[up, 2] < FALL_Z))
    M["heading_change_deg"] = float(np.unwrap(yaw_true[w])[-1] - np.unwrap(yaw_true[w])[0]) * deg
    M["yaw_err_end_deg"] = float(OE[w, 4][-1] * deg)
    if "vx" in run.scenario:
        M["vx_cmd"] = run.scenario["vx"]
        M["vx_track_mean"] = float(vb_true[w, 0].mean() - run.scenario["vx"])
        M["vx_track_rms"] = float(rms(vb_true[w, 0] - run.scenario["vx"]))
        M["vy_true_mean"] = float(vb_true[w, 1].mean())

    # -- convergence ------------------------------------------------------------
    if run.scenario["kind"] == "converge":
        tp = run.scenario["probe_time"]
        # The probe lands on the first cycle after tp; take the largest error
        # within 100 ms of it as the onset so the clock starts on the probe.
        near = np.nonzero((t >= tp - 0.05) & (t <= tp + 0.1))[0]
        i0 = near[np.argmax(np.abs(OE[near, 0]))]
        sel = np.zeros(len(t), bool)
        sel[i0:] = t[i0:] <= t1
        tau = t[sel] - t[i0]
        # The probe is about body x; remove the static pre-probe offset (accel
        # bias tilt) so the decay is measured against where the loop was sitting.
        pre = (t >= tp - 1.5) & (t <= tp - 0.1)
        offset = OE[pre, 0].mean() if pre.any() else 0.0
        roll_err = OE[sel, 0] - offset
        theta0 = roll_err[0]
        M["probe_pre_offset_deg"] = float(offset * deg)
        kp, ki, tau_lpf = mahony_gains(d)
        pred = predicted_decay(theta0, kp, ki, tau_lpf, tau)
        M["probe_theta0_deg"] = float(theta0 * deg)
        M["probe_t_1e_s"] = first_crossing(tau, roll_err, theta0 / np.e)
        M["probe_t_1e_pred_s"] = first_crossing(tau, pred, theta0 / np.e)
        M["probe_t_10pct_s"] = first_crossing(tau, roll_err, 0.1 * theta0)
        M["probe_t_10pct_pred_s"] = first_crossing(tau, pred, 0.1 * theta0)
        M["probe_undershoot_frac"] = float(roll_err.min() / theta0)
        M["probe_undershoot_pred_frac"] = float(pred.min() / theta0)
        M["probe_resid_end_frac"] = float(abs(roll_err[-1]) / theta0)
        M["probe_rms_vs_pred_deg"] = float(rms(roll_err - pred) * deg)
        M["kp_ki_tau"] = [kp, ki, tau_lpf]
    with open(os.path.join(d, "metrics.json"), "w") as f:
        json.dump(M, f, indent=1, default=float)
    return M


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def f3(x, scale=1.0, nd=3):
    if x is None:
        return "n/a"
    if isinstance(x, (list, tuple)):
        return " / ".join(f3(v, scale, nd) for v in x)
    try:
        x = float(x) * scale
    except (TypeError, ValueError):
        return str(x)
    return "nan" if np.isnan(x) else f"{x:.{nd}f}"


def table(rows, cols):
    head = "| run | " + " | ".join(c[0] for c in cols) + " |"
    sep = "|---|" + "---|" * len(cols)
    body = []
    for M in rows:
        body.append("| " + M["name"] + " | " + " | ".join(
            f3(M.get(c[1]), *(c[2:] or ())) for c in cols) + " |")
    return "\n".join([head, sep] + body)


def report(metrics, out):
    L = [f"# Estimator benchmark report\n\ngenerated {datetime.datetime.now().isoformat(timespec='seconds')}; "
         f"{len(metrics)} runs under `{out}`\n",
         "Velocity and position errors are the estimator's own in-module comparison with simulator truth; "
         "velocity errors are body-frame velocities, each in its own body frame (heading is unobservable, so a "
         "world-frame velocity difference grows with yaw drift). Window = steady segment of each scenario. In the "
         "ctrl=truth tier the controller gets the true body velocity through the attitude it uses; its heading is "
         "still the filter's.\n"]
    L += ["## Orientation\n", table(metrics, [
        ("tilt rms [°]", "tilt_rms_deg"), ("tilt max [°]", "tilt_max_deg"),
        ("tilt slope [°/min]", "tilt_slope_deg_per_min"),
        ("heading drift [°/min]", "yaw_drift_deg_per_min"), ("expected from unlearned bias [°/min]", "yaw_drift_expected_deg_per_min"),
        ("tilt max (run) [°]", "tilt_max_run_deg"),
        ("ω err mean x/y/z [mrad/s]", "omega_err_mean", 1e3, 2), ("ω err rms [mrad/s]", "omega_err_rms", 1e3, 2),
        ("bias err steady [mrad/s]", "bias_err_steady", 1e3, 2), ("bias err peak (run) [mrad/s]", "bias_err_peak_run", 1e3, 2)]), ""]
    conv = [M for M in metrics if M.get("probe_theta0_deg") is not None]
    if conv:
        L += ["## Convergence of a planted tilt\n", table(conv, [
            ("θ0 [°]", "probe_theta0_deg", 1, 2), ("t 1/e meas/pred [s]", "probe_t_1e_s", 1, 2),
            ("", "probe_t_1e_pred_s", 1, 2), ("t 10 % meas/pred [s]", "probe_t_10pct_s", 1, 2),
            ("", "probe_t_10pct_pred_s", 1, 2), ("undershoot meas/pred", "probe_undershoot_frac", 1, 3),
            ("", "probe_undershoot_pred_frac", 1, 3), ("residual at end", "probe_resid_end_frac", 1, 3),
            ("rms vs pred [°]", "probe_rms_vs_pred_deg", 1, 3), ("pre-probe offset [°]", "probe_pre_offset_deg", 1, 3)]), ""]
    L += ["## Velocity (true body frame)\n", table(metrics, [
        ("bias x/y/z [mm/s]", "vel_err_mean", 1e3, 1), ("rms x/y/z [mm/s]", "vel_err_rms", 1e3, 1),
        ("bias % of speed x/y/z", "vel_bias_pct_of_speed", 1, 2),
        ("rms at transitions [mm/s]", "vel_err_rms_transition", 1e3, 1), ("rms mid-stance [mm/s]", "vel_err_rms_midstance", 1e3, 1),
        ("max mid-stance [mm/s]", "vel_err_max_midstance", 1e3, 0),
        ("blind fraction (cell)", "blind_fraction", 1, 3),
        ("events", "n_contact_events", 1, 0)]), ""]
    L += ["## Foot slip in trusted stance (truth foot centre, body frame)\n", table(metrics, [
        ("raw mean x/y/z [mm/s]", "foot_vel_stance_mean", 1e3, 1), ("raw rms [mm/s]", "foot_vel_stance_rms", 1e3, 1),
        ("rolling-corrected mean [mm/s]", "foot_slip_rollcorr_mean", 1e3, 1), ("rolling-corrected rms [mm/s]", "foot_slip_rollcorr_rms", 1e3, 1)]), ""]
    L += ["## Position\n", table(metrics, [
        ("path [m]", "path_m", 1, 2), ("xy err end [m]", "xy_err_end_m", 1, 3), ("xy drift [%]", "xy_drift_percent", 1, 2),
        ("heading leak [%]", "heading_leak_percent", 1, 2), ("xy drift aligned [%]", "xy_drift_aligned_percent", 1, 2),
        ("ATE [m]", "ate_m", 1, 3), ("RPE xy 1 s [mm]", "rpe_xy_1s_m", 1e3, 1),
        ("z rms [mm]", "z_rms_m", 1e3, 1), ("z err mean [mm]", "z_err_mean_m", 1e3, 1)]), ""]
    L += ["## Consistency\n", table(metrics, [
        ("in 3σ (v)", "frac_in_3sigma_v", 1, 3), ("in 3σ (z)", "frac_in_3sigma_z", 1, 3),
        ("mean NEES v (≈3)", "nees_v_mean", 1, 2), ("NEES v centred", "nees_v_centered", 1, 2), ("mean NEES z (≈1)", "nees_z_mean", 1, 2),
        ("σv mean x/y/z [mm/s]", "sigma_v_mean", 1e3, 1),
        ("innov v mean [mm/s]", "innov_v_mean", 1e3, 2), ("innov v std [mm/s]", "innov_v_std", 1e3, 1),
        ("innov height mean [mm]", "innov_z_mean", 1e3, 2), ("mean NIS v (≈3)", "nis_v_mean", 1, 2)]), ""]
    L += ["## Closed loop\n", table(metrics, [
        ("vx cmd", "vx_cmd", 1, 2), ("vx tracking mean [mm/s]", "vx_track_mean", 1e3, 1), ("vx tracking rms [mm/s]", "vx_track_rms", 1e3, 1),
        ("true vy mean [mm/s]", "vy_true_mean", 1e3, 1), ("heading change [°]", "heading_change_deg", 1, 2),
        ("yaw err end [°]", "yaw_err_end_deg", 1, 2), ("fall", "fall", 1, 0)]), ""]
    L += ["## Figures\n", "Comparison figures in `figures/` (before_after-*, dumbbell-trot_speeds, explain-*).\n"] + \
         [f"- [{M['name']}]({M['name']}/story.png) · [phase]({M['name']}/phase.png)" for M in metrics]
    path = os.path.join(out, "report.md")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"wrote {path}")


def compare(a, b):
    """Side-by-side of two output directories' metrics.json files."""
    keys = ["tilt_rms_deg", "tilt_max_deg", "tilt_max_run_deg", "yaw_drift_deg_per_min", "bias_err_peak_run", "bias_err_steady",
            "vel_err_mean", "vel_err_rms", "vel_err_rms_transition", "xy_drift_percent",
            "xy_drift_aligned_percent", "z_rms_m", "z_err_mean_m", "nees_v_mean", "nees_v_centered", "nis_v_mean",
            "fall", "vx_track_mean", "vx_track_rms"]
    names = sorted(set(os.listdir(a)) & set(os.listdir(b)))
    for n in names:
        pa, pb = (os.path.join(x, n, "metrics.json") for x in (a, b))
        if not (os.path.exists(pa) and os.path.exists(pb)):
            continue
        A, B = json.load(open(pa)), json.load(open(pb))
        print(f"\n== {n}")
        for k in keys:
            if k in A or k in B:
                print(f"  {k:<26} {f3(A.get(k), 1, 4):>28}  ->  {f3(B.get(k), 1, 4)}")


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["run", "score", "figures", "report", "all", "compare", "list"])
    ap.add_argument("--baseline", default=os.path.join(REPO, "runs", "est_baseline"),
                    help="figures: directory of the unfixed runs for the before/after figures")
    ap.add_argument("paths", nargs="*", help="compare: two output directories")
    ap.add_argument("--only", default="*", help="glob on run names")
    ap.add_argument("--force", action="store_true", help="re-run even if run.mat exists")
    ap.add_argument("--out", default=DEFAULT_OUT)
    a = ap.parse_args()

    if a.cmd == "compare":
        if len(a.paths) != 2:
            sys.exit("compare needs two directories")
        compare(*a.paths)
        return

    runs = [r for r in run_matrix() if fnmatch.fnmatch(r.name, a.only)]
    if a.cmd == "list":
        for r in runs:
            print(r.name, " ".join(r.overrides()))
        return
    os.makedirs(a.out, exist_ok=True)
    if a.cmd in ("run", "all"):
        for r in runs:
            launch(r, a.out, a.force)
    metrics = []
    if a.cmd in ("score", "all", "report"):
        for r in runs:
            if a.cmd == "report" and os.path.exists(os.path.join(a.out, r.name, "metrics.json")):
                metrics.append(json.load(open(os.path.join(a.out, r.name, "metrics.json"))))
                continue
            M = score(r, a.out)
            if M and not M.get("missing"):
                metrics.append(M)
                print(f"[score] {r.name}: tilt rms {M['tilt_rms_deg']:.3f}°, vel bias {f3(M['vel_err_mean'], 1e3, 1)} mm/s, "
                      f"xy drift {f3(M['xy_drift_percent'], 1, 2)} %")
    if a.cmd in ("figures", "all"):
        # Per-run figures come from fig-gen/plot_est.py (works on any .mat log); the
        # before/after, dumbbell and explainer figures from fig-gen/compare_est.py.
        for r in runs:
            mat = os.path.join(a.out, r.name, "run.mat")
            if os.path.exists(mat):
                probe = r.scenario.get("probe_time")
                gains = mahony_gains(os.path.join(a.out, r.name))
                plot_log_file(mat, os.path.join(a.out, r.name), None, r.name, probe, gains)
        if os.path.isdir(a.baseline):
            compare_dirs(a.baseline, a.out, os.path.join(a.out, "figures"),
                         ("baseline", "final"), a.only if a.only != "*" else None)
    if metrics:
        report(metrics, a.out)


if __name__ == "__main__":
    main()
