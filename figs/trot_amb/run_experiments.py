#!/usr/bin/env python3
#
# Copyright (C) 2005-2026 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Angular-momentum balancing inside the whole-body trot ([trot] amb, TrotAMB)
on the simulated Go2, in place: every run headless, scored and plotted.

  /home/ege/legged/iros/.venv/bin/python figs/trot_amb/run_experiments.py [--only NAME ...] [--out DIR]

  tests         ctest in build/ (build and install first)
  depth         swing touchdown depth at the headline slow gait, AMB on
  baseline      AMB off, the trot as it was: how long a two-foot phase survives, and whether those phases
                are two-foot stands at all; then the same with swing_lift_time
  static        an 8 s two-foot phase on the trot's own footprint: how far off the support line the CoM may
                start (both realizations of AMB, no centring), then both diagonals with the four-foot CoM
                centring on the default footprint
  inplace       trot in place with two-foot phases of 0.75 to 2.0 s: nothing, centring alone, and AMB with
                centring in both realizations; AMB without centring; the attitude integral off; the filters
                in the loop; a lateral push
  forward       the 1.0 s / 0.5 s gait commanded forward at 0.02 to 0.20 m/s: nothing, the zig-zag alone
                (centring and foot placement without the law) and AMB with it; other phase lengths, the
                pass-through, the filters, a lateral and a yaw command

A gait is given by its two-foot time t_two (one diagonal carries the robot, the
other swings) and its four-foot time t_four: period = 2 (t_two + t_four), duty =
1 - t_two / period. Every run is one bin/sim.sh call on the whole-body trot,
driven by the Supervisor key script: stand, T at 2.5 s, the trot, then the
normal stop. Up to six simulations run side by side, each on its own LogServer
port (5510 and up), so an interactive bin/sim.sh on the default port is not in
the way. Controller state is simulator ground truth on every run but the
marked filter runs. The attitude integral is on (its default) unless a run
says otherwise. Logs: runs/trot_amb/<experiment>/<run>.mat and .log; results.json,
figs/ and the report in reports/trot_amb/ (--out DIR puts all under DIR).
Torso state, CoM and L come from simulator truth through figs/trot_wbc's Run;
the law's own quantities from MdlTrot_amb (layout at TrotAMB::_log in
include/quadruped/TrotAMB.hh).
"""

import argparse
import concurrent.futures
import importlib.util
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import build_report  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# figs/trot_wbc holds modules of the same names as this directory's, so its
# run_experiments is loaded by path (its "import build_report" gets this
# directory's, which it only calls from its own main()).
_spec = importlib.util.spec_from_file_location("trot_wbc_run_experiments",
                                               os.path.join(REPO, "figs", "trot_wbc", "run_experiments.py"))
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
amb_wbc, r = T.amb, T.r

T_KEY, PREP, RAMP = T.T_KEY, T.PREP, T.RAMP   # T key; MdlTrot WAIT + PREP; its ramp-up [s]
STOP = 6.5                                    # ramp-down, centering and sit after the stop key [s]
# 124 of the 127 doubles a sample line holds. Runs with AMB on trade the truth
# ground reactions and the torque request of figs/trot_wbc's list for MdlTrot_amb.
VARS_AMB = ["MdlSimDriver_qpos", "MdlSimDriver_qvel", "MdlTrot_contact", "MdlTrot_bodyref", "MdlTrot_wbc",
            "MdlTrot_amb"]
MODES = ("offset", "passthrough")
# Settings every slow-gait run shares. LIFT: swing_lift_time, without which a long swing creeps off the ground.
# DEPTH: touchdown depth below the lift-off height (the handoff's value for slow gaits; `depth` re-checks it).
LIFT, DEPTH, CENTRE = "trot.swing_lift_time = 0.2", "trot.wbc_touchdown_depth = 0.008", "trot.amb_centre = true"
# The law's switch on but no swing long enough for it to act: the four-foot CoM centring alone.
CENTRE_ONLY = ["trot.amb = true", "trot.amb_min_swing = 99.0", CENTRE]
SWAP = "trot.phase_offset = [0.5, 0.0, 0.0, 0.5]"   # the FR-RL diagonal carries the first two-foot phase
TWO, FOUR = [0.50, 0.75, 1.0, 1.5, 2.0], [0.0, 0.2, 0.5]
SLOW_TWO, SLOW_FOUR = [0.75, 1.0, 1.5, 2.0], 0.5
STATIC_TWO, STATIC_FOUR = 8.0, 0.5
HOLD_TWO, HOLD_FOUR = 30.0, 2.0                           # one long hold, after the centring has settled
# Simulations run side by side, each with its own LogServer port (the default
# 5500 is left to an interactive bin/sim.sh): two runs at once log the same
# values as either alone (checked 2026-11-27). figs/trot_wbc's Run works on one
# shared MuJoCo model, so loading and scoring a log is serialised.
PORTS, JOBS = range(5510, 5540), 6
_ports, _analysis = queue.Queue(), threading.Lock()
for _p in PORTS:
    _ports.put(_p)
ORIGINS = [0.0, -0.005, -0.009, -0.013, -0.017, -0.021]   # footprint fore-aft origin [m]: moves the CoM across the diagonals
HEAD = (1.0, 0.5)                                         # the gait of the push, integral and filter runs
PUSH_STEP, PUSH_CAP = 10, 150                             # [N]
EXPERIMENTS = ["tests", "depth", "baseline", "static", "inplace", "forward"]
SPEEDS = [0.02, 0.05, 0.10, 0.15, 0.20]                   # commanded forward speed of the slow trot [m/s]
CONFIG_KEYS = ("amb_mode", "amb_gains", "amb_kp3", "amb_kd3", "amb_observer_tc", "amb_template_m",
               "amb_template_l", "amb_template_d", "amb_ramp", "amb_return", "amb_min_swing", "amb_alpha_max",
               "amb_foot_acc_max", "wbc_kp_ori", "wbc_kd_ori", "wbc_kp_foot", "wbc_kd_foot", "wbc_joint_kp",
               "wbc_joint_kd", "wbc_touchdown_depth", "wbc_attitude_integral_gain", "swing_height")


def gait(t_two, t_four):
    """(period, duty) of a trot with the given two-foot and four-foot times."""
    period = 2.0 * (t_two + t_four)
    return period, 1.0 - t_two / period


def trot_time(t_two, t_four):
    """Time in TROT [s]: at least 15 s and four gait cycles."""
    return max(15.0, 4.0 * gait(t_two, t_four)[0])


def overrides(path, t_two, t_four, trot_s, extra=(), logvars=None, push=None):
    period, duty = gait(t_two, t_four)
    d_key = T_KEY + PREP + trot_s
    over = ["simulation.headless = true", "simulation.realtime = false",
            f"supervisor.exit_time = {d_key + STOP:.3f}",
            f"supervisor.script = {json.dumps(['0 s', f'{T_KEY} t', f'{d_key:.3f} d'])}",
            "supervisor.log.enable = true", "supervisor.log.start = 3.0", f"supervisor.log.period = {T.LOG_PERIOD}",
            'supervisor.log.file_format = "matlab"', f'supervisor.log.file_name = "{path}.mat"',
            f"supervisor.log.vars = {json.dumps(logvars or VARS_AMB)}", "trot.wbc = true",
            "trot.forward_velocity = 0.0", "trot.lateral_velocity = 0.0", "trot.yaw_rate = 0.0",
            f"trot.period = {period:.6f}", f"trot.duty = {duty:.8f}"] + T.GROUND_TRUTH
    if push:
        over += [f"simulation.push_force = [0.0, {push[0]:.1f}, 0.0]", f"simulation.push_duration = {T.PULSE}",
                 f"simulation.push_time = {push[1]:.4f}", "simulation.push_ramp = 0.0"]
    return over + list(extra)


def simulate(path, over, exit_time):
    """One headless bin/sim.sh run with the given overrides -> path.mat, path.log; its wall time [s]."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    for e in (".mat", ".log"):
        if os.path.exists(path + e):
            os.remove(path + e)
    port, t0 = _ports.get(), time.time()
    over = over + [f"logserver.server_port = {port}"]
    try:
        with open(path + ".log", "w") as log:
            log.write("overrides: " + json.dumps(over) + "\n")
            log.flush()
            # Own process group: sim.sh does not exec ./simulation, so killing the shell alone would leave the
            # simulator running.
            p = subprocess.Popen(["./sim.sh"] + [x for c in over for x in ("-c", c)], cwd=T.BIN,
                                 stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
            try:
                rc = p.wait(timeout=3 * exit_time + 60)
            except BaseException:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
                raise
    finally:
        _ports.put(port)
    if rc != 0 or not os.path.exists(path + ".mat"):
        raise RuntimeError(f"{path}: exit {rc}, no log (see {path}.log)")
    print(f"  [{time.time() - t0:4.1f} s] {os.path.join(*path.split(os.sep)[-2:])}", flush=True)
    return round(time.time() - t0, 1)


def amb_on(mode):
    return ["trot.amb = true", f'trot.amb_mode = "{mode}"'] if mode else []


def summary(R, trot_s):
    """Scores of one run. The window runs from RAMP after TROT entry to the
    stop key; a run that falls or errors is scored up to that event only for
    the time it lasted."""
    d_key = T_KEY + PREP + trot_s
    moving = np.nonzero(np.any(np.diff(R.B, axis=0) != 0, axis=1))[0]
    entry = R.t[moving[0] + 1]
    out = {"period": r(R.cmd["period"], 4), "duty": r(R.cmd["duty"], 5), "entry_t": r(entry, 3),
           "v_cmd": [R.cmd["vx"], R.cmd["vy"], R.cmd["yaw"]],
           "fall_t": R.fall(R.trot_window(entry)), "error": R.errors[0] if R.errors else None,
           "stop_completed": R.stopped, "warnings": R.warnings[:4], "wbc_closing": R.closing}
    failed = out["fall_t"] is not None or out["error"] is not None
    out["failed"] = failed
    end = min(x for x in (out["fall_t"], R.t_end if R.errors else None, d_key) if x is not None)
    out["survived_s"] = r(end - entry, 3)
    w = R.win(entry + RAMP, end)
    two = w & (R.pair >= 0)
    deg = np.degrees(R.rpy[:, :2])
    if not failed:
        out.update(roll_peak=r(np.abs(deg[w, 0]).max(), 3), pitch_peak=r(np.abs(deg[w, 1]).max(), 3),
                   roll_rms=r(amb_wbc.rms(deg[w, 0]), 3), pitch_rms=r(amb_wbc.rms(deg[w, 1]), 3),
                   roll_mean=r(deg[w, 0].mean(), 3), pitch_mean=r(deg[w, 1].mean(), 3),
                   z_min=r(R.z[w].min(), 4))
    if (R.t[w][-1] - R.t[w][0] if w.any() else 0.0) > 3.0:   # mean body-frame velocity and yaw rate achieved
        out.update(vx=r(R.v_body[w, 0].mean(), 4), vy=r(R.v_body[w, 1].mean(), 4), yaw_rate=r(R.yaw_rate[w].mean(), 4))
    if two.any():
        out.update(L_rms=r(amb_wbc.rms(R.L[two]), 4), L_peak=r(np.abs(R.L[two]).max(), 4),
                   ch_rms_mm=r(1e3 * amb_wbc.rms(R.ch[two]), 3), ch_peak_mm=r(1e3 * np.abs(R.ch[two]).max(), 3))
    # A two-foot phase counts as held when neither swing foot came back to the ground between its lift and
    # its descent and the body stayed within 10 deg: without that the body has tipped onto a swing foot and
    # stands on three. Phases cut short by the stop, or begun before the lift scale reached 1, are not counted.
    starts = np.nonzero((R.pair[1:] >= 0) & (R.pair[:-1] != R.pair[1:]))[0] + 1
    held, phases, ch0, zmin, zmax = 0, 0, [], [], []
    for i0 in starts:
        j = i0
        while j < len(R.t) and R.pair[j] == R.pair[i0]:
            j += 1
        if R.t[j - 1] - R.t[i0] < (1.0 - R.cmd["duty"]) * R.cmd["period"] - 0.01 or R.B[i0, 10] < 0.99 or \
                R.t[i0] > end:
            continue
        phases += 1
        ch0.append(1e3 * R.ch[i0])
        sw = [1, 2] if R.pair[i0] == 0 else [0, 3]
        mid = np.arange(i0, j)[(R.t[i0:j] > R.t[i0] + 0.25) & (R.t[i0:j] < R.t[j - 1] - 0.25)]
        held += int(len(mid) > 0 and R.cp[mid][:, sw, 2].min() > 0.005
                    and np.degrees(np.abs(R.rpy[i0:j, :2]).max()) < 10.0)
        if len(mid):
            zmin.append(1e3 * R.cp[mid][:, sw, 2].min())
            zmax.append(1e3 * R.cp[mid][:, sw, 2].max())
    out.update(phases=phases, phases_held=held, ch_at_liftoff_mm=[r(x, 2) for x in ch0],
               # lowest and highest a swing foot was between its lift and its descent, over the counted phases
               swing_height_mm=[r(min(zmin), 1), r(max(zmax), 1)] if zmin else None)
    A = R.D.get("MdlTrot_amb")
    if A is not None and w.any() and np.any(A[w, 1] > 0):
        act = w & (A[:, 1] == 1)
        out["amb"] = {"acting_pct_of_two_foot": r(100.0 * act.sum() / max(two.sum(), 1), 2),
                      "invalid_ticks": int(A[w, 21].max()),
                      "alpha_peak_deg": r(np.degrees(np.abs(A[w, 16:18]).max()), 3),
                      "leg_travel_peak_deg": r(np.degrees(np.abs(A[w, 14:16]).max()), 3)}
        if act.any():
            # Mode offset: the swing task's PD part of the tangential acceleration, kp rho (alpha - measured),
            # against the feedforward rho thdd. Small means the legs follow the commanded motion as the
            # pass-through assumes they do.
            kp = T.CONFIG["trot"]["wbc_kp_foot"]
            pd = kp * A[act, 22:24] * (A[act, 16:18] - A[act, 14:16])
            ff = A[act, 22:24] * A[act, 12:14]
            out["amb"].update(thdd_rms=r(amb_wbc.rms(A[act, 12:14]), 3), ff_acc_rms=r(amb_wbc.rms(ff), 4),
                              pd_acc_rms=r(amb_wbc.rms(pd), 4), tau_hat_peak=r(np.abs(A[act, 5]).max(), 4))
    return out


class Experiments:
    def __init__(self, logs, rep):
        self.logs, self.rep, self.wall = logs, rep, {}

    def save(self, f, name):
        os.makedirs(os.path.join(self.rep, "figs"), exist_ok=True)
        f.tight_layout()
        f.savefig(os.path.join(self.rep, "figs", name))
        plt.close(f)
        return "figs/" + name

    def run(self, exp, name, t_two, t_four, mode=None, extra=(), trot_s=None, logvars=None, push=None,
            keep=False, cmd=(0.0, 0.0, 0.0)):
        """One run -> its summary (and the loaded Run when keep). mode: None
        (AMB off), "offset" or "passthrough"."""
        trot_s = trot_s or trot_time(t_two, t_four)
        path = os.path.join(self.logs, exp, name)
        over = overrides(path, t_two, t_four, trot_s, amb_on(mode) + list(extra), logvars, push)
        over += [f"trot.forward_velocity = {cmd[0]}", f"trot.lateral_velocity = {cmd[1]}", f"trot.yaw_rate = {cmd[2]}"]
        self.wall[f"{exp}/{name}"] = simulate(path, over, T_KEY + PREP + trot_s + STOP)
        period, duty = gait(t_two, t_four)
        with _analysis:
            R = T.Run(path, "wbc", {"vx": cmd[0], "vy": cmd[1], "yaw": cmd[2], "period": period, "duty": duty})
            row = dict(name=name, t_two=t_two, t_four=t_four, mode=mode or "off", trot_s=trot_s,
                       **summary(R, trot_s))
        return (row, R) if keep else row

    def many(self, calls):
        """Rows of several runs made side by side, in the order given. Each
        call: (tags merged into the row, self.slow/self.run, args, kwargs)."""
        with concurrent.futures.ThreadPoolExecutor(JOBS) as pool:
            futs = [pool.submit(fn, *args, **kw) for _tags, fn, args, kw in calls]
            return [dict(tags, **f.result()) for (tags, *_), f in zip(calls, futs)]

    # -- tests -------------------------------------------------------------------
    def tests(self):
        p = subprocess.run(["ctest", "--output-on-failure"], cwd=os.path.join(REPO, "build"), capture_output=True,
                           text=True)
        tail = [x for x in p.stdout.splitlines() if x.strip()][-3:]
        return {"returncode": p.returncode, "summary": tail}

    def slow(self, exp, name, t_two, t_four, mode=None, extra=(), **kw):
        """A run with the slow-gait settings (lift profile, touchdown depth)."""
        return self.run(exp, name, t_two, t_four, mode, [LIFT, self.depth_key] + list(extra), **kw)

    depth_key = DEPTH

    # -- depth -------------------------------------------------------------------
    def depth(self, res):
        """The swing lands `wbc_touchdown_depth` below its lift-off height: too
        little leaves the foot short of the ground at the scheduled touchdown,
        too much lands it early and hard. Scored on the headline gait with AMB
        and centring on, where every phase is a two-foot stand."""
        rows = self.many([({"depth": d}, self.run, ("depth", f"d{1e3 * d:.0f}", *HEAD, "offset",
                                                    [LIFT, CENTRE, f"trot.wbc_touchdown_depth = {d}"]), {})
                          for d in (0.003, 0.008, 0.012)])
        ok = [x for x in rows if not x["failed"]]
        best = min(ok, key=lambda x: x["roll_rms"] + x["pitch_rms"])["depth"] if ok else 0.008
        return {"rows": rows, "chosen": best}

    # -- baseline ----------------------------------------------------------------
    def baseline(self, res):
        """AMB off. `default`: the trot as it was before this work, at the
        handoff's 8 mm touchdown depth. `lift`: the same with swing_lift_time."""
        return {"default": self.many([({}, self.run, ("baseline", f"default_two{t_two:.2f}_four{t_four:.1f}", t_two,
                                                       t_four), {"extra": [DEPTH]})
                                      for t_four in FOUR for t_two in TWO]),
                "lift": self.many([({}, self.slow, ("baseline", f"lift_two{t_two:.2f}", t_two, SLOW_FOUR), {})
                                   for t_two in SLOW_TWO])}

    # -- static ------------------------------------------------------------------
    def static(self, res):
        """Step 1. `basin`: the first diagonal alone (the stop falls in the
        four-foot phase after it), the footprint moved fore-aft so the CoM
        starts at different distances from the support line, no centring.
        `centred`: both diagonals in one run on the default footprint with the
        CoM centring, and again with the other diagonal first. `hold`: one
        30 s two-foot phase after 2 s on four feet."""
        one = STATIC_TWO + 1.5 * STATIC_FOUR            # stop key in the four-foot phase after the first swing
        both = 2.0 * STATIC_TWO + 3.5 * STATIC_FOUR     # ... after the second
        ox = f"trot.origin = [{ORIGINS[3]}, 0.10, -0.28]"
        PV = "posvelestimator.use_ground_truth = false"
        hold_s = HOLD_FOUR + HOLD_TWO + 0.5 * HOLD_FOUR
        return {
            "basin": self.many([({"origin_x": o}, self.slow,
                                 ("static", f"basin_{mode}_ox{1e3 * o:+.0f}", STATIC_TWO, STATIC_FOUR, mode,
                                  [f"trot.origin = [{o}, 0.10, -0.28]"]), {"trot_s": one})
                                for mode in MODES for o in ORIGINS]),
            "aids": self.many([({"case": c}, self.slow, ("static", f"aids_{c}", STATIC_TWO, STATIC_FOUR, m, [ox] + e),
                                {"trot_s": one})
                               for c, m, e in (("off", None, []),
                                               ("passthrough_no_friction", "passthrough",
                                                ["trot.amb_joint_friction = 0.0"]),
                                               ("passthrough_joint_pd_on", "passthrough",
                                                ["trot.amb_swing_joint_pd = true"]),
                                               ("passthrough_both_aids", "passthrough", []))]),
            "centred": self.many([({"case": case, "first": first}, self.slow,
                                   ("static", f"centred_{case.replace(' ', '_')}_{first}", STATIC_TWO, STATIC_FOUR,
                                    mode, extra + swap), {"trot_s": both})
                                  for first, swap in (("FL-RR", []), ("FR-RL", [SWAP]))
                                  for case, mode, extra in (("centring only", None, CENTRE_ONLY),
                                                            ("offset", "offset", [CENTRE]),
                                                            ("passthrough", "passthrough", [CENTRE]))]),
            "hold": self.many([({"case": case}, self.slow,
                                ("static", "hold_" + case.replace(",", "").replace(" ", "_").replace("/", "_"),
                                 HOLD_TWO, HOLD_FOUR, mode, extra), {"trot_s": hold_s})
                               for case, mode, extra in (
                                   ("nothing", None, []), ("offset", "offset", [CENTRE]),
                                   ("passthrough", "passthrough", [CENTRE]),
                                   ("offset, position/velocity filter in the loop", "offset", [CENTRE, PV]),
                                   ("nothing, position/velocity filter in the loop", None, [PV]))])}

    # -- inplace -----------------------------------------------------------------
    def inplace(self, res):
        """Step 2: trot in place with long two-foot phases."""
        out = {"push": {}}
        cases = (("nothing", None, []), ("centring only", None, CENTRE_ONLY), ("offset", "offset", [CENTRE]),
                 ("passthrough", "passthrough", [CENTRE]))
        out["grid"] = self.many([({"case": case}, self.slow,
                                  ("inplace", f"grid_two{t_two:.2f}_{case.replace(' ', '_')}", t_two, SLOW_FOUR, mode,
                                   extra), {}) for t_two in SLOW_TWO for case, mode, extra in cases])
        PV = "posvelestimator.use_ground_truth = false"
        out["variants"] = self.many([
            ({"case": name}, self.slow,
             ("inplace", "var_" + name.replace(",", "").replace(" ", "_").replace("/", "_"), t_two, t_four, "offset",
              extra), {"trot_s": trot_s})
            for name, t_two, t_four, extra, trot_s in (
                ("no centring", HEAD[0], HEAD[1], [], None),
                ("four-foot phase 0.3 s", HEAD[0], 0.3, [CENTRE], None),
                ("four-foot phase 0.2 s", HEAD[0], 0.2, [CENTRE], None),
                ("attitude integral off", HEAD[0], HEAD[1], [CENTRE, T.KI_OFF], None),
                ("attitude integral off, two-foot 1.5 s", 1.5, HEAD[1], [CENTRE, T.KI_OFF], None),
                ("position/velocity filter in the loop", HEAD[0], HEAD[1], [CENTRE, PV], None),
                ("position/velocity filter in the loop, two-foot 2.0 s, 60 s", 2.0, HEAD[1], [CENTRE, PV], 60.0),
                ("ground truth, two-foot 2.0 s, 60 s", 2.0, HEAD[1], [CENTRE], 60.0),
                ("both filters in the loop", HEAD[0], HEAD[1],
                 [CENTRE, "orientationestimator.use_ground_truth = false", PV], None))])
        # Lateral 0.1 s pulse at the base CoM in the middle of the fifth two-foot phase (FL-RR stance), both
        # signs, from PUSH_STEP up to the first run that falls. The six sweeps run side by side.
        entry = next(x["entry_t"] for x in out["grid"] if x["t_two"] == HEAD[0] and x["case"] == "offset")
        tp = entry + HEAD[1] + 4.0 * sum(HEAD) + 0.5 * HEAD[0]
        out["push_time_s"] = r(tp, 3)

        def sweep(case, mode, extra, sign):
            rows, F = [], PUSH_STEP
            while F <= PUSH_CAP:
                row = self.slow("inplace", f"push_{case}_{'p' if sign > 0 else 'm'}{F}", *HEAD, mode, extra,
                                push=(sign * F, tp))
                rows.append(dict(F=sign * F, failed=row["failed"], phases=row["phases"],
                                 phases_held=row["phases_held"], survived_s=row["survived_s"]))
                if row["failed"]:
                    break
                F += PUSH_STEP
            ok = [abs(x["F"]) for x in rows if not x["failed"]]
            return {"largest_survived_N": max(ok) if ok else None, "rows": rows}

        specs = [(case, mode, extra, sign) for case, mode, extra in
                 (("nothing", None, []), ("offset", "offset", [CENTRE]), ("passthrough", "passthrough", [CENTRE]))
                 for sign in (1, -1)]
        with concurrent.futures.ThreadPoolExecutor(len(specs)) as pool:
            for (case, _m, _e, sign), f in zip(specs, [pool.submit(sweep, *sp) for sp in specs]):
                out["push"].setdefault(case, {})["plus_y" if sign > 0 else "minus_y"] = f.result()
        return out

    # -- forward -----------------------------------------------------------------
    def forward(self, res):
        """The slow trot commanded forward. With the centring on the body
        follows the command along the current support line only and the
        landing feet are placed so their line passes under the CoM: a zig-zag."""
        PV = "posvelestimator.use_ground_truth = false"
        cases = (("nothing", None, []), ("zig-zag only", None, CENTRE_ONLY), ("offset", "offset", [CENTRE]))
        out = {"speeds": self.many([({"case": case}, self.slow,
                                     ("forward", f"v{v:.2f}_{case.replace(' ', '_')}", *HEAD, mode, extra),
                                     {"cmd": (v, 0.0, 0.0)}) for v in SPEEDS for case, mode, extra in cases])}
        out["variants"] = self.many([
            ({"case": name}, self.slow,
             ("forward", "var_" + name.replace(",", "").replace(" ", "_").replace("/", "_"), t_two, t_four, mode,
              extra), {"cmd": cmd})
            for name, t_two, t_four, mode, extra, cmd in (
                ("two-foot 0.75 s, four-foot 0.2 s, 0.10 m/s", 0.75, 0.2, "offset", [CENTRE], (0.10, 0.0, 0.0)),
                ("four-foot 0.2 s, 0.10 m/s", HEAD[0], 0.2, "offset", [CENTRE], (0.10, 0.0, 0.0)),
                ("two-foot 2.0 s, 0.05 m/s", 2.0, HEAD[1], "offset", [CENTRE], (0.05, 0.0, 0.0)),
                ("pass-through, 0.10 m/s", *HEAD, "passthrough", [CENTRE], (0.10, 0.0, 0.0)),
                ("position/velocity filter in the loop, 0.10 m/s", *HEAD, "offset", [CENTRE, PV], (0.10, 0.0, 0.0)),
                ("both filters in the loop, 0.10 m/s", *HEAD, "offset",
                 [CENTRE, PV, "orientationestimator.use_ground_truth = false"], (0.10, 0.0, 0.0)),
                ("lateral 0.05 m/s", *HEAD, "offset", [CENTRE], (0.0, 0.05, 0.0)),
                ("yaw 0.1 rad/s", *HEAD, "offset", [CENTRE], (0.0, 0.0, 0.1)))])
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", nargs="+", choices=EXPERIMENTS, help="experiments to run (default: all)")
    ap.add_argument("--out", help="root for logs, results and figures (default: runs/trot_amb, reports/trot_amb)")
    a = ap.parse_args()
    out = a.out and os.path.abspath(a.out)
    logs = os.path.join(out, "runs") if out else os.path.join(REPO, "runs", "trot_amb")
    rep = out or os.path.join(REPO, "reports", "trot_amb")
    os.makedirs(rep, exist_ok=True)
    only = a.only or EXPERIMENTS
    E = Experiments(logs, rep)
    path = os.path.join(rep, "results.json")
    res = json.load(open(path)) if os.path.exists(path) else {}
    walls = res.get("meta", {}).get("experiment_wall_s", {})
    for name in EXPERIMENTS:
        if name not in only:
            continue
        print(f"== {name}", flush=True)
        t0 = time.time()
        if "depth" in res and name != "depth":
            E.depth_key = f"trot.wbc_touchdown_depth = {res['depth']['chosen']}"
        res[name] = getattr(E, name)() if name == "tests" else getattr(E, name)(res)
        walls[name] = round(time.time() - t0, 1)
        import tomllib
        trot = tomllib.load(open(os.path.join(REPO, "config", "default", "trot.toml"), "rb"))["trot"]
        res["meta"] = {"t_key_s": T_KEY, "prep_s": PREP, "window": f"TROT entry + {RAMP} s ramp-up to the stop key",
                       "fall": f"base z < {T.FALL_Z} m or |roll|, |pitch| > 45 deg",
                       "state": "simulator ground truth (orientation and position/velocity) unless a run says "
                                "otherwise", "config": {k: trot[k] for k in CONFIG_KEYS if k in trot},
                       "logs": os.path.relpath(logs, REPO), "experiment_wall_s": walls, "parallel_runs": JOBS}
        with open(path, "w") as f:
            json.dump(res, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    print(f"\nwall time {sum(walls[n] for n in only):.0f} s; results in {path}")
    if all(n in res for n in EXPERIMENTS):
        build_report.build(rep)


if __name__ == "__main__":
    main()
