#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Prints one scorecard for a Supervisor .mat log (plus optionally an
MdlConvexMPC prediction dump), so "is this MPC good?" is a table of numbers
instead of a look at a plot.

  score_run.py <log.mat> [--dump FILE] [--start S] [--stop S]

Every metric needs specific log variables (see supervisor.log.vars); a log
recorded with a shorter list is normal, not an error. A metric whose input is
absent from this log prints "n/a (missing '<var>')" naming exactly the
variable that would be needed, rather than crashing the whole table.

Without --start/--stop, the analysis window is derived from the data rather
than defaulting to the whole log: it is bracketed by the first and last
transition of the gait contact signal, then a settling margin
(SETTLE_MARGIN_S) is dropped off the front, since the trot command is not at
full scale until rampup + WAIT + PREP have elapsed. The window actually used,
and how it was chosen, is always printed on the header -- pass --start/--stop
explicitly to override either edge.

See figs/mpc/README.md for how to record a log and what supervisor.log.vars
to use, and src/quadruped/MdlConvexMPC.cc:644-686 for the dump format
(figs/mpc/mpcdump.py parses it).
"""

import argparse
import os
import sys

import numpy as np

from plot_mpc import load_log, quat_rotate, JOINT_DIRECTION, TORQUE_LIMIT, \
    JOINT_NAMES, LEG_NAMES
from mpcdump import read_dump

# Compile-time hip positions in body frame, (x, y, z), Go2. From
# createGo2Config() in src/quadruped/QuadrupedConfigs.cc. Not logged, same
# reasoning as plot_mpc.py's TORQUE_LIMIT: cheaper to hard-code than to spend
# log budget on a constant.
HIP_POS = np.array([
    [0.1934, 0.0465, 0.0],    # FL
    [0.1934, -0.0465, 0.0],   # FR
    [-0.1934, 0.0465, 0.0],   # RL
    [-0.1934, -0.0465, 0.0],  # RR
])

# Gait timing, config/default/trot.toml defaults (period, duty). Not logged.
GAIT_PERIOD = 0.5
GAIT_DUTY = 0.5
T_STANCE = GAIT_DUTY * GAIT_PERIOD

BODY_WEIGHT_N = 149.2

# Contact debounce: a bare instantaneous grf_z > GRF_OFF_N re-triggers on
# touchdown bounce and force ripple, fragmenting one true stance into
# several -- see debounce_contact()'s docstring. GRF_ON_N/GRF_OFF_N are a
# Schmitt trigger (latch stance above ON, latch swing only below OFF);
# MIN_EVENT_S then merges any stance or swing shorter than 30% of the
# nominal 0.25s half-cycle into its neighbours.
GRF_ON_N = 20.0
GRF_OFF_N = 5.0
MIN_EVENT_S = 0.30 * T_STANCE

# Default settling margin dropped off the front of an auto-detected window:
# trot.rampup_duration (2.0s) plus the WAIT (0.3s) and PREP (1.5s) states
# before the gait is commanded at full scale, plus headroom. Every statistic
# in the table is contaminated by stand-up/ramp-up/sit-down transients if
# this isn't applied, which is why it is the default rather than something
# the caller has to remember to pass.
SETTLE_MARGIN_S = 10.0

# Below this many gait cycles, edge effects (partial stances, one or two
# footfalls) dominate every statistic; the header warns rather than silently
# printing numbers that look precise but aren't.
MIN_WINDOW_CYCLES = 5

# MdlConvexMPC::input_t::body_state_t layout (13 states), see
# include/quadruped/MdlConvexMPC.hh.
ST_ROLL, ST_PITCH, ST_YAW = 0, 1, 2
ST_PX, ST_PY, ST_PZ = 3, 4, 5
ST_VX = 9


class Missing(Exception):
    """Raised by col() for a metric to report as n/a rather than crash."""

    def __init__(self, name):
        super().__init__(name)
        self.name = name


def col(data, name):
    if name not in data:
        raise Missing(name)
    return data[name]


def section(title):
    print(f"-- {title} --")


def show(label, fn):
    """Prints one metric line, or 'n/a (missing ...)' if fn hits a Missing var."""
    try:
        print(f"  {label:<38}{fn()}")
    except Missing as e:
        print(f"  {label:<38}n/a (missing '{e.name}')")


def rpy_from_quat(q):
    """roll, pitch, yaw from a (N,4) body-to-world quaternion (w,x,y,z).

    Copied from MdlOrientationEstimator::rpyFromQuat
    (src/quadruped/MdlOrientationEstimator.cc:64-86), ZYX order, NOT
    scipy.spatial.transform's convention. R_bw = Rz*Ry*Rx is body->world in
    this codebase's +X-forward/+Y-left/+Z-up frame, so positive pitch here
    means NOSE DOWN.
    """
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(s)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def edges(contact_bool):
    """(touchdown sample indices, liftoff sample indices) from a bool series."""
    d = np.diff(contact_bool.astype(int))
    td = np.flatnonzero(d == 1) + 1
    lo = np.flatnonzero(d == -1) + 1
    return td, lo


def stance_pairs(td, lo):
    """[(touchdown_idx, liftoff_idx), ...] for each *completed* stance phase."""
    pairs = []
    j = 0
    for i in td:
        while j < len(lo) and lo[j] <= i:
            j += 1
        if j < len(lo):
            pairs.append((i, lo[j]))
    return pairs


def nearest_index(t, target):
    """For each value in target, the index into sorted t nearest to it."""
    idx = np.clip(np.searchsorted(t, target), 1, len(t) - 1)
    left = idx - 1
    choose_left = np.abs(t[left] - target) <= np.abs(t[idx] - target)
    return np.where(choose_left, left, idx)


def debounce_contact(t, fz, on_thresh=GRF_ON_N, off_thresh=GRF_OFF_N,
                     min_interval_s=MIN_EVENT_S):
    """Hysteresis + minimum-duration merge on one leg's grf_z trace.

    Pass 1, a Schmitt trigger: latches stance (True) once fz rises above
    on_thresh, and only unlatches back to swing once fz falls below
    off_thresh; a sample in the dead zone holds the previous state. This
    alone absorbs ripple that never leaves the dead zone, but NOT a real
    excursion that crosses both thresholds and back within one touchdown
    bounce, which is exactly what a MuJoCo foot does on impact.

    Pass 2 handles that: run-length encode the latched signal, then
    iteratively merge any interior run (stance or swing) shorter than
    min_interval_s into its neighbours, which -- by construction of run
    alternation -- already share a value once the short run between them is
    removed. A run at either end of the window is left alone rather than
    merged, since it may be a genuine partial stance/swing truncated by the
    window edge, not a bounce; downstream stance_pairs() already discards
    incomplete edge stances.

    Returns (clean bool (N,), raw_crossings, accepted_events): raw_crossings
    is the transition count in the latched-only signal (pass 1), so the
    fragmentation the merge step removes is visible rather than silent.
    """
    n = len(fz)
    state = np.zeros(n, dtype=bool)
    cur = False
    for i in range(n):
        if fz[i] > on_thresh:
            cur = True
        elif fz[i] < off_thresh:
            cur = False
        state[i] = cur
    raw_crossings = int(np.sum(state[1:] != state[:-1]))

    runs = []
    i = 0
    while i < n:
        j = i
        while j < n and state[j] == state[i]:
            j += 1
        runs.append([i, j, bool(state[i])])
        i = j

    changed = True
    while changed:
        changed = False
        for k in range(1, len(runs) - 1):
            s, e, _ = runs[k]
            dur = (t[e] if e < n else t[-1]) - t[s]
            if dur < min_interval_s:
                runs[k - 1][1] = runs[k + 1][1]
                del runs[k:k + 2]
                changed = True
                break

    clean = np.zeros(n, dtype=bool)
    for s, e, val in runs:
        clean[s:e] = val
    accepted_events = max(0, len(runs) - 1)
    return clean, raw_crossings, accepted_events


def _contact_signal(data, t):
    """(contact bool (N,4), description, is_measured); raises Missing.

    Prefers measured stance from MdlSimDriver_grf, debounced per leg by
    debounce_contact() (a bare instantaneous threshold fragments one true
    stance into several on touchdown bounce -- see its docstring);
    MdlTrot_contact is a fallback and is the open-loop *schedule*, not a
    measurement. Silent -- get_contact() below is the version gait/force
    reporting should call, which additionally prints the fallback note.
    """
    if "MdlSimDriver_grf" in data:
        grf = data["MdlSimDriver_grf"]
        contact = np.zeros((len(t), 4), dtype=bool)
        for leg in range(4):
            contact[:, leg], _, _ = debounce_contact(t, grf[:, 3 * leg + 2])
        return contact, (f"measured (MdlSimDriver_grf, hysteresis "
                         f"{GRF_OFF_N:.0f}/{GRF_ON_N:.0f} N, "
                         f"min event {MIN_EVENT_S * 1000:.0f} ms)"), True
    if "MdlTrot_contact" in data:
        return data["MdlTrot_contact"] > 0.5, "scheduled (MdlTrot_contact)", False
    raise Missing("MdlSimDriver_grf or MdlTrot_contact")


def get_contact(data, t):
    """(contact bool (N,4), description, is_measured), noting a schedule fallback."""
    contact, label, measured = _contact_signal(data, t)
    if not measured:
        print("  note: 'MdlSimDriver_grf' is not in this log; falling back to "
              "the SCHEDULED gait (MdlTrot_contact). This is open-loop and is "
              "NOT a measurement of actual ground contact.")
        label += ", not measured"
    return contact, label, measured


def detect_gait_window(t, data):
    """(start, stop, label) bracketing the gait, from the full unwindowed log.

    Deliberately brackets using the SCHEDULE (MdlTrot_contact), never
    measured contact, even when grf is available: the schedule is exactly
    constant whenever MdlTrot is not the active behavior, so its first/last
    transition marks precisely when the gait was running. Measured contact
    is unsuitable for this specifically -- a stance foot stays loaded into a
    following Sit, so bracketing by "last measured contact transition" runs
    the stop edge past the end of the gait and into the sit-down transient.
    Falls back to measured grf only when the log has no schedule at all.
    The caller is responsible for applying SETTLE_MARGIN_S on top. Returns
    (None, None, reason) when there's nothing to bracket from.
    """
    if "MdlTrot_contact" in data:
        sched = data["MdlTrot_contact"] > 0.5
        label = "scheduled (MdlTrot_contact)"
    elif "MdlSimDriver_grf" in data:
        grf = data["MdlSimDriver_grf"]
        sched = np.zeros((len(t), 4), dtype=bool)
        for leg in range(4):
            sched[:, leg], _, _ = debounce_contact(t, grf[:, 3 * leg + 2])
        label = ("measured (MdlSimDriver_grf); this log has no schedule, so "
                "the window may run into a trailing sit")
    else:
        return None, None, "no contact signal available"
    changed = np.any(np.diff(sched.astype(int), axis=0) != 0, axis=1)
    idx = np.flatnonzero(changed)
    if len(idx) == 0:
        return None, None, f"{label} never transitions"
    return t[idx[0] + 1], t[idx[-1] + 1], label


def duty_from_stances(t, pairs):
    """Duty factor from complete stance intervals only.

    mean(stance duration) / mean(touchdown-to-touchdown period) -- never the
    sample mean of the contact flag over the window. That mean is biased by
    whatever fraction of a cycle sits at each window edge: a window that
    isn't an integer number of gait cycles makes an identical, symmetric
    schedule look like it has a different duty factor per leg, which is a
    windowing artefact, not a gait asymmetry. None if there are fewer than 2
    complete stances to measure a period from.
    """
    if len(pairs) < 2:
        return None
    stance_durations = np.array([t[j] - t[i] for i, j in pairs])
    td_times = np.array([t[i] for i, _ in pairs])
    periods = np.diff(td_times)
    return float(np.mean(stance_durations) / np.mean(periods))


def foot_body_frame(data):
    """(N,4,3) foot position per leg (body frame), foottruth (world), qpos.

    Same rotation plot_mpc.py uses for its foot_reference_<LEG> figures:
    world-frame foot minus base position, rotated into the body frame with
    R_bw = R_wb^T via quat_rotate(..., inverse=True). qpos is returned
    alongside since callers doing an exact per-stance body-travel
    calculation need the raw world position and quaternion too.
    """
    foottruth = col(data, "MdlPosVelEstimator_foottruth")
    qpos = col(data, "MdlSimDriver_qpos")
    base = qpos[:, 0:3]
    q = qpos[:, 3:7]
    out = np.zeros((foottruth.shape[0], 4, 3))
    for leg in range(4):
        rel_world = foottruth[:, 3 * leg:3 * leg + 3] - base
        out[:, leg, :] = quat_rotate(q, rel_world, inverse=True)
    return out, foottruth, qpos


def stance_metrics(qpos, foot_b, foottruth, hip_x, leg, pairs):
    """Per-stance arrays (one row per completed stance): x_td, x_lo (m),
    excess_bf (m, exact body-frame sweep excess), slip_xyz (m, world-frame
    signed foot displacement touchdown->liftoff).

    excess_bf is L - body_travel computed exactly per stance, with no mean
    velocity anywhere: body_travel is the body's own world displacement
    between this stance's touchdown and liftoff samples (MdlSimDriver_qpos),
    rotated into the body frame at touchdown (R_bw(q_td)^T), so it uses the
    same rotation convention x_td/x_lo already use. If the foot does not
    slip, L equals this body travel exactly and excess_bf is 0; a positive
    slip_xyz[...,0] (foot moves forward in world during stance) shows up as
    a negative excess_bf, up to the body's own rotation over the stance --
    see the printed consistency check in gait_section().
    """
    n = len(pairs)
    x_td = np.empty(n)
    x_lo = np.empty(n)
    excess_bf = np.empty(n)
    slip_xyz = np.empty((n, 3))
    for k, (i, j) in enumerate(pairs):
        x_td[k] = foot_b[i, leg, 0] - hip_x
        x_lo[k] = foot_b[j, leg, 0] - hip_x
        world_disp = qpos[j, 0:3] - qpos[i, 0:3]
        q_td = qpos[i, 3:7]
        body_travel_x = quat_rotate(q_td[None, :], world_disp[None, :], inverse=True)[0, 0]
        excess_bf[k] = (x_td[k] - x_lo[k]) - body_travel_x
        slip_xyz[k] = foottruth[j, 3 * leg:3 * leg + 3] - foottruth[i, 3 * leg:3 * leg + 3]
    return x_td, x_lo, excess_bf, slip_xyz


# ---------------------------------------------------------------------------
#  Sections
# ---------------------------------------------------------------------------

def solver_section(data):
    section("Solver")

    def solve_times_ms():
        return col(data, "MdlConvexMPC_solver")[:, 1] * 1000.0

    def status_rate():
        solver = col(data, "MdlConvexMPC_solver")
        rate = float(np.mean(solver[:, 0] != 0))
        return f"{100 * rate:.2f}% of {len(solver)} solves"
    show("nonzero qpOASES status rate", status_rate)
    show("solve time p50", lambda: f"{np.percentile(solve_times_ms(), 50):.3f} ms")
    show("solve time p95", lambda: f"{np.percentile(solve_times_ms(), 95):.3f} ms")
    show("solve time max", lambda: f"{np.max(solve_times_ms()):.3f} ms")
    show("fraction over 1 ms budget",
        lambda: f"{100 * np.mean(solve_times_ms() > 1.0):.2f}%")


def body_section(t, data):
    section("Body")
    print("  sign convention: R_bw = Rz*Ry*Rx (body->world) => positive pitch = NOSE DOWN")

    def v_x_tracking():
        achieved = col(data, "MdlSimDriver_qvel")[:, 0]
        commanded = col(data, "MdlTrot_bodyref")[:, 4]
        bias = float(np.mean(achieved - commanded))
        cmd_mean = float(np.mean(commanded))
        pct = f"{100 * bias / cmd_mean:+.1f}%" if abs(cmd_mean) > 1e-6 else "n/a, commanded ~0"
        return (f"achieved {float(np.mean(achieved)):.4f} m/s, commanded {cmd_mean:.4f} m/s, "
                f"bias {bias:+.4f} m/s ({pct})")
    show("v_x mean achieved vs commanded", v_x_tracking)

    def pitch_stats():
        _, pitch, _ = rpy_from_quat(col(data, "MdlSimDriver_qpos")[:, 3:7])
        return (f"mean {np.degrees(np.mean(pitch)):+.3f} deg "
                f"({1000 * np.mean(pitch):+.2f} mrad), "
                f"p2p {np.degrees(np.ptp(pitch)):.3f} deg ({1000 * np.ptp(pitch):.2f} mrad)")
    show("pitch (+ = nose down)", pitch_stats)

    def roll_stats():
        roll, _, _ = rpy_from_quat(col(data, "MdlSimDriver_qpos")[:, 3:7])
        return (f"mean {np.degrees(np.mean(roll)):+.3f} deg "
                f"({1000 * np.mean(roll):+.2f} mrad), "
                f"p2p {np.degrees(np.ptp(roll)):.3f} deg ({1000 * np.ptp(roll):.2f} mrad)")
    show("roll", roll_stats)

    def height_stats():
        z = col(data, "MdlSimDriver_qpos")[:, 2]
        return f"mean {np.mean(z):.4f} m, p2p {np.ptp(z):.4f} m"
    show("height", height_stats)

    def yaw_drift():
        if "MdlTrot_bodyref" in data:
            yaw = data["MdlTrot_bodyref"][:, 8]
        else:
            _, _, yaw_wrapped = rpy_from_quat(col(data, "MdlSimDriver_qpos")[:, 3:7])
            yaw = np.unwrap(yaw_wrapped)
        slope = float(np.polyfit(t, yaw, 1)[0])
        return f"{slope:+.5f} rad/s (linear fit)"
    show("yaw drift rate", yaw_drift)


def gait_section(t, data):
    section("Gait geometry")
    try:
        contact, contact_label, measured = get_contact(data, t)
    except Missing as e:
        print(f"  n/a (missing '{e.name}'): cannot detect touchdown/liftoff "
              "without some form of ground contact")
        return
    try:
        foot_b, foottruth, qpos = foot_body_frame(data)
    except Missing as e:
        print(f"  n/a (missing '{e.name}'): cannot locate the feet")
        return

    print(f"  contact source: {contact_label}")
    if measured:
        grf = data["MdlSimDriver_grf"]
        print("  contact debounce audit (raw threshold crossings -> accepted events):")
        for leg, name in enumerate(LEG_NAMES):
            _, raw_c, acc_e = debounce_contact(t, grf[:, 3 * leg + 2])
            print(f"    {name}: {raw_c} raw -> {acc_e} accepted")

    sched = data.get("MdlTrot_contact")
    S_all = []
    for leg, name in enumerate(LEG_NAMES):
        td, lo = edges(contact[:, leg])
        pairs = stance_pairs(td, lo)
        if not pairs:
            print(f"  {name}: no complete stance phase in this window")
            continue
        x_td, x_lo, excess_bf, slip_xyz = stance_metrics(
            qpos, foot_b, foottruth, HIP_POS[leg, 0], leg, pairs)
        S = (x_td + x_lo) / 2.0
        L_mm = (x_td - x_lo) * 1000.0
        duty = duty_from_stances(t, pairs)
        duty_str = f"{duty:.3f}" if duty is not None else "n/a (<2 complete stances)"
        S_all.append(np.mean(S))

        timing = ""
        if measured and sched is not None:
            sched_td, _ = edges(sched[:, leg] > 0.5)
            if len(sched_td):
                nearest_sched_t = t[sched_td][nearest_index(t[sched_td], t[td])]
                err_ms = (t[td] - nearest_sched_t) * 1000.0
                timing = f"  td_err(mean/rms)={np.mean(err_ms):+.1f}/{np.sqrt(np.mean(err_ms**2)):.1f} ms"
        elif not measured:
            timing = "  td_err=n/a (contact IS the schedule)"
        else:
            timing = "  td_err=n/a (missing 'MdlTrot_contact')"

        print(f"  {name}: x_td={np.mean(x_td):+.4f} m  x_lo={np.mean(x_lo):+.4f} m  "
              f"S={np.mean(S) * 1000:+.1f} mm  L={np.mean(L_mm):+.1f} mm  "
              f"duty={duty_str}  n={len(pairs)}{timing}")

        # Sweep excess computed two independent ways -- exactly per stance,
        # no mean velocity anywhere. (a) body-frame: L minus this stance's
        # own exact body displacement (qpos, rotated into the body frame at
        # touchdown). (b) world-frame: the foot's own signed displacement,
        # negated (see stance_metrics docstring for why these should agree
        # up to the body's rotation over one ~250ms stance).
        excess_mm = excess_bf * 1000.0
        slip_mm = slip_xyz * 1000.0
        neg_slip_x_mm = -slip_mm[:, 0]
        agree_mm = float(np.mean(excess_mm) - np.mean(neg_slip_x_mm))
        flag = "" if abs(agree_mm) <= 2.0 else "  ** MISMATCH > 2mm, check frame handling **"
        print(f"    sweep excess: body-frame(exact) {np.mean(excess_mm):+.1f} "
              f"+/-{np.std(excess_mm):.1f} mm   "
              f"vs -slip_x {np.mean(neg_slip_x_mm):+.1f} +/-{np.std(neg_slip_x_mm):.1f} mm   "
              f"agree within {agree_mm:+.1f} mm{flag}")
        print(f"    foot slip world (mm): "
              f"x={np.mean(slip_mm[:, 0]):+.1f}+/-{np.std(slip_mm[:, 0]):.1f}  "
              f"y={np.mean(slip_mm[:, 1]):+.1f}+/-{np.std(slip_mm[:, 1]):.1f}  "
              f"z={np.mean(slip_mm[:, 2]):+.1f}+/-{np.std(slip_mm[:, 2]):.1f}  "
              f"norm(mean/max)={np.mean(np.linalg.norm(slip_mm, axis=1)):.1f}/"
              f"{np.max(np.linalg.norm(slip_mm, axis=1)):.1f}")

    if S_all:
        print(f"  S averaged over legs: {np.mean(S_all) * 1000:+.1f} mm "
              "(0 = sweep centred under hip; >0 = foot ahead of hip through stance)")


def effort_section(data):
    section("Effort")

    def per_joint(torque):
        lines = []
        for j, name in enumerate(JOINT_NAMES):
            limit = TORQUE_LIMIT[j]
            cols = torque[:, j::3]
            rms_pct = 100.0 * np.sqrt(np.mean(cols ** 2)) / limit
            peak_pct = 100.0 * np.max(np.abs(cols)) / limit
            over_frac = 100.0 * np.mean(np.abs(cols) > limit)
            lines.append(f"{name:<10} rms {rms_pct:5.1f}%  peak {peak_pct:6.1f}%  "
                         f"over-limit {over_frac:5.2f}% of samples")
        return lines

    try:
        req = col(data, "MdlTrot_torquereq")
        print("  requested (pre-saturation):")
        for line in per_joint(req):
            print(f"    {line}")
    except Missing as e:
        print(f"  requested: n/a (missing '{e.name}')")

    try:
        ctrl = col(data, "MdlSimDriver_ctrl").copy()
        for leg in range(4):
            ctrl[:, 3 * leg:3 * leg + 3] *= np.asarray(JOINT_DIRECTION[leg])
        print("  applied (post-saturation, controller sign convention):")
        for line in per_joint(ctrl):
            print(f"    {line}")
    except Missing as e:
        print(f"  applied: n/a (missing '{e.name}')")

    try:
        scale = col(data, "MdlTrot_torquescale")
        print("  saturation fraction (MdlTrot_torquescale, <1 means clipping):")
        for leg, name in enumerate(LEG_NAMES):
            s = scale[:, leg]
            print(f"    {name}: mean={np.mean(s):.3f}  min={np.min(s):.3f}  "
                  f"clipped={100 * np.mean(s < 1.0):.2f}% of samples")
    except Missing as e:
        print(f"  saturation: n/a (missing '{e.name}')")


def peak_at_touchdown(force_z, contact_leg, t):
    """Peak vertical force in a short window after each touchdown."""
    td, _ = edges(contact_leg)
    if len(td) == 0:
        return np.array([])
    dt_med = np.median(np.diff(t)) if len(t) > 1 else 1e-3
    win = max(1, int(round(0.03 / dt_med)))
    return np.array([np.max(force_z[i:min(i + win, len(t))]) for i in td])


def force_section(t, data):
    section("Force tracking")
    try:
        cmd = col(data, "MdlConvexMPC_forces")
        act = col(data, "MdlSimDriver_grf")
    except Missing as e:
        print(f"  n/a (missing '{e.name}')")
        return
    try:
        contact, _, _ = get_contact(data, t)
    except Missing as e:
        print(f"  n/a (missing '{e.name}'): no stance mask to select stance samples")
        return

    for leg, name in enumerate(LEG_NAMES):
        stance = contact[:, leg]
        if not stance.any():
            print(f"  {name}: no stance samples in this window")
            continue
        err = act[stance][:, 3 * leg:3 * leg + 3] - cmd[stance][:, 3 * leg:3 * leg + 3]
        bias = np.mean(err, axis=0)
        rms = np.sqrt(np.mean(err ** 2, axis=0))
        print(f"  {name} f_actual-f_cmd (x,y,z) N: "
              f"bias=({bias[0]:+.2f},{bias[1]:+.2f},{bias[2]:+.2f})  "
              f"rms=({rms[0]:.2f},{rms[1]:.2f},{rms[2]:.2f})")

    print(f"  peak vertical force at touchdown, body weights (BW={BODY_WEIGHT_N:.1f} N):")
    for leg, name in enumerate(LEG_NAMES):
        peaks = peak_at_touchdown(act[:, 3 * leg + 2], contact[:, leg], t)
        if len(peaks) == 0:
            print(f"    {name}: no touchdown events in this window")
            continue
        print(f"    {name}: mean {np.mean(peaks) / BODY_WEIGHT_N:.2f} BW  "
              f"max {np.max(peaks) / BODY_WEIGHT_N:.2f} BW  n={len(peaks)}")


def validity_section(dump_path):
    section("Plan-vs-reference convergence (dump)")
    header, records = read_dump(dump_path)
    t = records["t"]
    n = len(t)
    if n < 2:
        print("  n/a: dump has fewer than 2 usable records")
        return
    dt = header["dt"]
    H = header["horizon"]
    print(f"  dump: {n} records, horizon {H}, dt {dt * 1000:.1f} ms")
    print("  NOT a model-validity metric -- read the caveat before quoting any of this.")
    print("  The crossover step below tracks the CONFIGURED HORIZON, not the robot:")
    print("  measured 14 at H15, 19 at H20, 27 at H30, 37 at H40 (all dt=0.01). The QP")
    print("  minimises deviation from the reference, so its plan converges toward that")
    print("  reference across the horizon and the two error curves meet asymptotically --")
    print("  where they meet is a property of the horizon. At dt=0.05 this reports a")
    print("  crossover at step 0 for configurations that then walk for 40 s. It is useful")
    print("  only to compare two runs at IDENTICAL horizon and dt; it does not bound the")
    print("  usable lookahead, and there is no horizon*dt <= duty*period rule (falsified).")

    # reference[k]/Xpred[k] describe now+(k+1)*dt (include/quadruped/MdlConvexMPC.hh:108-113):
    # one step later than contact[k]/moment arms, which describe the interval
    # STARTING at now+k*dt. Scored here against the dump's own x0 (ground
    # truth, since the sim runs with ground truth in the loop) from whichever
    # later record's t is nearest now+(k+1)*dt.
    channels = {"roll": ST_ROLL, "pitch": ST_PITCH, "height": ST_PZ, "v_x": ST_VX}
    for name, idx in channels.items():
        pred_rms = np.full(H, np.nan)
        ref_rms = np.full(H, np.nan)
        for k in range(H):
            target = t + (k + 1) * dt
            valid = target <= t[-1] + 1e-9
            if not valid.any():
                continue
            m = nearest_index(t, target[valid])
            x_true = records["x0"][m, idx]
            pred_err = records["Xpred"][valid, k, idx] - x_true
            ref_err = records["xref"][valid, k, idx] - x_true
            pred_rms[k] = np.sqrt(np.mean(pred_err ** 2))
            ref_rms[k] = np.sqrt(np.mean(ref_err ** 2))

        crossover = next((k for k in range(H)
                          if not np.isnan(pred_rms[k]) and not np.isnan(ref_rms[k])
                          and pred_rms[k] > ref_rms[k]), None)
        if crossover is None:
            print(f"  {name}: prediction stays at or below reference error through "
                 f"the whole horizon ({H} steps, {H * dt:.3f} s lookahead)")
        else:
            print(f"  {name}: plan/reference crossover at step {crossover} "
                 f"({(crossover + 1) * dt:.3f} s lookahead) -- pred_rms="
                 f"{pred_rms[crossover]:.4g} exceeds ref_rms={ref_rms[crossover]:.4g} "
                 f"at step {crossover}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", help="Supervisor .mat log to score")
    ap.add_argument("--dump", help="MdlConvexMPC prediction dump (enables plan/reference convergence)")
    ap.add_argument("--start", type=float, default=None,
                    help="drop samples before this time [s] (default: auto, see above)")
    ap.add_argument("--stop", type=float, default=None,
                    help="drop samples after this time [s] (default: auto, see above)")
    args = ap.parse_args()

    if not os.path.isfile(args.logfile):
        raise SystemExit(f"{args.logfile}: no such file")
    if args.dump is not None and not os.path.isfile(args.dump):
        raise SystemExit(f"{args.dump}: no such file")

    t_full, data_full = load_log(args.logfile)
    auto_start, auto_stop, auto_note = detect_gait_window(t_full, data_full)

    if args.stop is not None:
        stop, stop_note = args.stop, "explicit --stop"
    elif auto_stop is not None:
        stop = auto_stop
        stop_note = f"auto: last {auto_note} transition ({auto_stop:.2f}s)"
    else:
        stop, stop_note = t_full[-1], f"auto: {auto_note}, using full log"

    if args.start is not None:
        start, start_note = args.start, "explicit --start"
    elif auto_start is not None:
        margined = auto_start + SETTLE_MARGIN_S
        if margined < stop:
            start = margined
            start_note = (f"auto: first {auto_note} transition ({auto_start:.2f}s) "
                          f"+ {SETTLE_MARGIN_S:.1f}s settling margin")
        else:
            start = auto_start
            start_note = (f"auto: first {auto_note} transition ({auto_start:.2f}s); "
                          f"{SETTLE_MARGIN_S:.1f}s settling margin dropped, not enough "
                          "trot time in this log")
    else:
        start, start_note = t_full[0], f"auto: {auto_note}, using full log"

    keep = (t_full >= start) & (t_full <= stop)
    if not keep.any():
        raise SystemExit(f"{args.logfile}: no samples in [{start}, {stop}]; "
                         f"log spans 0 to {t_full[-1]:.1f}s")
    t = t_full[keep]
    data = {k: v[keep] for k, v in data_full.items()}

    print(f"score_run: {args.logfile}")
    print(f"  window [{t[0]:.2f}, {t[-1]:.2f}] s, {len(t)} samples")
    print(f"    start: {start_note}")
    print(f"    stop:  {stop_note}")
    duration = t[-1] - t[0]
    if duration < MIN_WINDOW_CYCLES * GAIT_PERIOD:
        print(f"  WARNING: window is {duration:.2f}s, shorter than "
              f"{MIN_WINDOW_CYCLES} gait cycles ({MIN_WINDOW_CYCLES * GAIT_PERIOD:.2f}s) "
              "-- every statistic below is edge-dominated")
    print(f"  present: {', '.join(sorted(data))}")
    if "MdlTrot_bodyref" in data:
        print(f"  commanded v_x (window mean, MdlTrot_bodyref[4]): "
              f"{np.mean(data['MdlTrot_bodyref'][:, 4]):.4f} m/s")
    else:
        print("  commanded v_x: n/a (missing 'MdlTrot_bodyref')")
    print()

    solver_section(data)
    print()
    body_section(t, data)
    print()
    gait_section(t, data)
    print()
    effort_section(data)
    print()
    force_section(t, data)

    if args.dump:
        print()
        validity_section(args.dump)


if __name__ == "__main__":
    sys.exit(main())
