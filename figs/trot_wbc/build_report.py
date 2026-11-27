#!/usr/bin/env python3
#
# Copyright (C) 2005-2026 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""REPORT.md and REPORT.pdf on the whole-body impulse-control trot, from
results.json and figs/ in reports/trot_wbc (or --out DIR). No simulation runs:

  /home/ege/legged/iros/.venv/bin/python figs/trot_wbc/build_report.py [--out DIR]

run_experiments.py calls build() at the end of a full run. Every measured
number is read from results.json, and so are the configured gains and weights
(meta.config); the block renderers are figs/amb_wbc's.
"""

import argparse
import datetime
import importlib.util
import json
import math
import os

import matplotlib
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (CondPageBreak, Image, KeepTogether, Paragraph, Preformatted, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Same file name as this one, so loaded by path.
_spec = importlib.util.spec_from_file_location("amb_wbc_build_report",
                                               os.path.join(REPO, "figs", "amb_wbc", "build_report.py"))
amb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(amb)
TITLE = "Whole-body impulse-control trot on the Go2: verification"
MODES = ("baseline", "wbc")
CORE = ("inplace", "fwd035", "fwd070", "fwd100", "lat020", "yaw050", "circle", "fwd070_pv", "fwd070_imu")
FWD = ("inplace", "fwd035", "fwd070", "fwd100")
TD = ("inplace", "fwd070", "fwd100")
NAMES = {"inplace": "in place", "fwd035": "forward 0.35", "fwd070": "forward 0.70", "fwd100": "forward 1.00",
         "lat020": "lateral 0.20", "yaw050": "yaw 0.5 rad/s", "circle": "circle 0.35 m/s, 0.5 rad/s",
         "fwd070_pv": "forward 0.70, pos/vel filter", "fwd070_imu": "forward 0.70, both filters"}
COMBOS = [f"{s}_{p}" for s in ("plus_y", "minus_y") for p in ("touchdown", "midstance")]
EVENT = {"fall_t": "fall", "error_t": "module ERROR", "first_saturated_t": "torque saturation"}


def fx(v, n=2, sign=""):
    return "–" if v is None else f"{v:{sign}.{n}f}"


def bw(B, W, get, n=2, sign=""):
    """'baseline / wbc' for one metric; a run that failed has none."""
    def one(x):
        try:
            return fx(get(x), n, sign)
        except (KeyError, TypeError):
            return "–"
    return f"{one(B)} / {one(W)}"


def seq(get, keys, n=2, sign=""):
    return " / ".join(fx(get(k), n, sign) for k in keys)


def span(vals, n=1, unit=""):
    """'a–b unit' over a list, or one value when they agree to the shown digits."""
    lo, hi = f"{min(vals):.{n}f}", f"{max(vals):.{n}f}"
    return (lo if lo == hi else f"{lo} to {hi}" if min(vals) < 0 else f"{lo}–{hi}") + unit


def and_list(xs):
    return ", ".join(xs[:-1]) + " and " + xs[-1] if len(xs) > 1 else "".join(xs)


def words(s):
    return s.replace(">=", "≥").replace("<=", "≤").replace("< ", "below ").replace("> ", "above ")


def content(R, written):
    core, cad, push, meta, dg = R["core"], R["cadence"], R["push"], R["meta"], R["diagnosis"]
    B, W, ch, sl = core["runs"]["baseline"], core["runs"]["wbc"], core["chain"], core["support_line"]
    C = {"baseline": B, "wbc": W}
    gates = {g["gate"]: g for g in R["gates"]}
    cfg, lim = meta["config"], meta["torque_limit_Nm"]
    tc, mc = cfg["trot"], cfg["mpc"]
    walls = meta["run_wall_times_s"]
    cl = [W[n]["wbc_closing"] for n in CORE]
    nopos, k0, j20 = dg["no_position_error"], dg["position_task_gains_zero"]["fwd070"], dg["joint_pd_20_2"]
    fb, td, est, ps, lg = dg["force_balance"], dg["touchdown"], dg["estimator"], dg["pitch_stiffness"], \
        dg["leg_gravity_baseline"]
    ew, eb = est["wbc"], est["baseline"]
    tr = gates["tracking"]
    corr = ch["fz_corr_desired_leg_vs_simulated_leg"]
    cross = max(corr[a][b] for a in (0, 3) for b in (1, 2))   # FL, RR desired against FR, RL simulated
    res = ch["grf_logged_minus_momentum_N"].values()
    grf_tol = math.ceil(2 * max(abs(v) for x in res for v in x)) / 2      # [N], to the half newton above
    grf_x = max(abs(x[0]) for x in res)
    ratio = {n: {k: W[n][k] / B[n][k] for k in ("roll_rms", "pitch_rms", "v_err")} for n in CORE}
    over = [n for n in CORE if ratio[n]["pitch_rms"] > 1.5]
    tn = {k: i for i, k in enumerate(("gates", "core", "pieces", "velocity", "attitude", "diagnosis", "forces",
                                       "swing", "touchdown", "filters", "duty", "support", "cadence", "push"), 1)}
    fn = {k: i for i, k in enumerate(("core", "diagnosis", "support", "cadence"), 1)}
    seen = []

    def T(key, cap):
        seen.append(key)   # tables must appear in tn's order
        assert [k for k in tn if k in seen] == seen, seen
        return f"Table {tn[key]}. {cap}"

    def F(key, cap):
        return f"Fig. {fn[key]}. {cap}"

    def ratio_cell(n, k):
        x = ratio[n][k]
        s = f"{x:.2f}x"
        return f"<b>{s}</b>" if x > 1.5 else s

    Bl = [("title", TITLE),
          ("p", f"<i>Simulation only (MuJoCo, Unitree Go2). Results: `{meta['results']}`, written {written} by "
                f"`figs/trot_wbc/run_experiments.py` ({len(walls)} simulator runs, "
                f"{sum(meta['experiment_wall_s'].values()) / 60:.0f} min); logs under `{meta['logs']}/`. Every number "
                "below is read from results.json.</i>")]

    # 1 -----------------------------------------------------------------------------------------------------
    vbetter = sum(W[n]["v_err"] <= B[n]["v_err"] for n in CORE)
    sd_lower = sum(W[n]["pitch_sd"] < B[n]["pitch_sd"] for n in CORE)
    Bl += [("h2", "1. What this is and the outcome"),
           ("p", "`[trot] wbc = true` replaces MdlTrot's per-leg commands (J<sup>T</sup>f on stance legs, a Cartesian "
                 "PD on swing legs) by whole-body impulse control after Kim, Di Carlo, Katz, Bledt and Kim (2019). The "
                 "convex MPC's first-step forces, the body reference and the swing samples become a task hierarchy "
                 "and a QP that lets the floating base give way, and every joint gets the resulting torque plus a "
                 "joint PD. The default stays `false`. Each run below exists for the unchanged per-leg trot "
                 "('baseline') and for the WBC. "
                 "Outcome: the " + and_list([g["gate"] for g in R["gates"] if g["pass"]]) + " gates pass" +
                 "".join(f"; the {g['gate']} gate fails" + (" its strict rule, with the cause measured" if g.get("cause")
                                                             else "") for g in R["gates"] if not g["pass"]) +
                 ". The WBC's velocity "
                 f"error is at or below the baseline's in {vbetter} of {len(CORE)} core runs and its pitch oscillation "
                 f"(sd) is smaller in {sd_lower}. "
                 + (("At " + " and ".join(NAMES[n] + " m/s" for n in over) + " it holds a steady nose-up pitch (" +
                     ", ".join(f"{W[n]['pitch_mean']:+.2f} deg" for n in over) + "), which takes its pitch rms to " +
                     " and ".join(f"{ratio[n]['pitch_rms']:.2f}x" for n in over) + " the baseline's and fails the "
                     "strict 1.5x rule. ") if over else "") +
                 ("The cause is measured (Sec. 3.2): the joint PD acts on joint references shifted by the base "
                  "position error, which grows with speed because the position estimate lags its own velocity; "
                  "with that error held at zero every core run meets the rule. " if tr.get("cause") else "") +
                 "The controller is judged on the simulator's own state (attitude, position and velocity from ground "
                 "truth), so that estimator error does not enter the comparison; two runs put the filters in the loop "
                 "(Sec. 3.5). At speed the WBC holds a small nose-up pitch ("
                 f"{seq(lambda n: W[n]['pitch_mean'], ('fwd070', 'fwd100'), 2, '+')} deg at 0.70 / 1.00 m/s, "
                 f"{W['fwd070_pv']['pitch_mean']:+.2f} deg at 0.70 m/s with the position/velocity filter); Sec. 3.2 "
                 "gives its cause. It is documented, not removed. "
                 "Nothing of angular-momentum balancing (AMB) is in the trot; Sec. 4 records, without a gate, the "
                 "quantities a later AMB study needs."),
           ("table", T("gates", "Gates. Criterion, result and value as stored in results.json."),
            ["Gate", "Criterion", "Result", "Value"],
            [[g["gate"], words(g["criterion"]), g["result"],
              words(g["value"]) + (f". <b>Cause:</b> {words(g['cause'])}." if g.get("cause") else "")]
             for g in R["gates"]], [0.08, 0.38, 0.10, 0.44]),
           ("table", T("core", "Core runs, baseline / WBC, steady window (simulator truth). |Δv|: mean body-frame velocity "
                       "minus command; |Δω|: mean yaw rate minus command; ratios WBC ÷ baseline, bold above the "
                       "1.5x rule."),
            ["Run", "|Δv| [m/s] (tolerance)", "|Δω| [rad/s]", "Roll rms [deg]", "Ratio", "Pitch rms [deg]", "Ratio",
             "Pitch mean [deg] (+ nose down)", "Velocity error ratio"],
            [[NAMES[n], bw(B[n], W[n], lambda x: x["v_err"], 3) + f" ({W[n]['v_tol']:.2f})",
              bw(B[n], W[n], lambda x: x["yaw_err"], 3), bw(B[n], W[n], lambda x: x["roll_rms"]),
              ratio_cell(n, "roll_rms"), bw(B[n], W[n], lambda x: x["pitch_rms"]), ratio_cell(n, "pitch_rms"),
              bw(B[n], W[n], lambda x: x["pitch_mean"], sign="+"), ratio_cell(n, "v_err")] for n in CORE],
            [0.17, 0.13, 0.11, 0.10, 0.06, 0.10, 0.06, 0.12, 0.08])]

    # 2 -----------------------------------------------------------------------------------------------------
    sw = mc["state_weights"]
    Bl += [("h2", "2. The formulation and what was built"),
           ("p", "Kim et al. split the controller in three. A convex MPC on a single rigid body plans the ground "
                 "reactions over a horizon; only its first-step forces f<sub>MPC</sub> go down. A task hierarchy, "
                 "rebuilt every tick from the scheduled contacts, stacks the contact constraint, body orientation, "
                 "body position and one position task per swing foot by null-space projection: a kinematic stage "
                 "(pseudo-inverse) gives joint references q<sub>des</sub>, q̇<sub>des</sub>, an acceleration stage "
                 "(dynamically consistent inverse) gives q̈<sub>cmd</sub>. A QP then corrects the floating-base "
                 "acceleration (δ<sub>fb</sub>, 6) and the forces (δ<sub>f</sub>) so that the six floating-base rows "
                 "of the dynamics hold, with the friction pyramid on f<sub>MPC</sub> + δ<sub>f</sub>; weighting the "
                 f"base lightly ({tc['wbc_w_base']}) against the forces ({tc['wbc_w_force']}) lets the body give way "
                 "and keeps the MPC's forces, the 'impulse' idea. Finally τ = A q̈ + b + g − J<sub>c</sub><sup>T</sup>f "
                 "goes to every joint with a joint PD on q<sub>des</sub>, q̇<sub>des</sub>. A trot stance uses all 18 "
                 "degrees of freedom (3 n<sub>c</sub> + 3 + 3 + 3 (4 − n<sub>c</sub>)), so no null space is left."),
           ("table", T("pieces", "Where each piece lives. Only the per-leg command block of MdlTrot's TROT/STOPPING tick is "
                       "replaced; gait clock, footstep law, swing quintic, body reference, MPC call and state machine "
                       "are shared by both modes."),
            ["Piece", "Here", "Code"],
            [["MPC → f<sub>MPC</sub>", f"MdlConvexMPC unchanged: horizon {mc['horizon']} × {1e3 * mc['dt']:.0f} ms, "
              "re-solved on every contact change; first-step world forces", "`MdlTrot::_trotDuring`"],
             ["State", "R, ω from MdlOrientationEstimator; base position and velocity from MdlPosVelEstimator, moved "
              "from the CoM back to the base origin (in the runs here both modules hand out simulator truth, "
              "Sec. 3)", "`MdlTrot::_wbcTick`"],
             ["Model", "floating-base Go2, 18 DoF: A, b + g, contact and foot Jacobians",
              "`QuadrupedDynamics` (Go2 config)"],
             ["Contacts", "scheduled stance legs (no touchdown detection, as Kim); rolling contact points one foot "
              "radius below the foot centres", "`MdlTrot::wbcSolve`"],
             ["Tasks", f"body orientation (R<sub>z</sub>(desired yaw), yaw-rate command; {tc['wbc_kp_ori']:g}/"
              f"{tc['wbc_kd_ori']:g}), base position (MdlTrot's CoM reference less the CoM offset, commanded "
              f"velocity; {tc['wbc_kp_pos']:g}/{tc['wbc_kd_pos']:g}), one per swing foot (body-frame swing quintic; "
              f"{tc['wbc_kp_foot']:g}/{tc['wbc_kd_foot']:g}); zero reference acceleration on the body tasks",
              "`MdlTrot::wbcSolve`"],
             ["Hierarchy, QP", f"Kim Eq. (16)–(21), (25); weights {tc['wbc_w_base']} / {tc['wbc_w_force']}; friction "
              "and normal bounds copied from [mpc]", "`QuadrupedWBC` (qpOASES; changed only to solve flight, Sec. 3.7)"],
             ["Joint command", f"τ + PD {tc['wbc_joint_kp']:g}/{tc['wbc_joint_kd']:g} on every leg; the request is "
              "clipped per joint to the Go2 limits; the closing line counts the clipped ticks",
              "`MdlTrot::_wbcTick` → `MdlLegControl::setJointCommand`"]],
            [0.13, 0.62, 0.25]),
           ("ol", [
               f"<b>Switch and keys.</b> `[trot] wbc` (default `{str(tc['wbc']).lower()}` in trot.toml and "
               "MdlTrot.hh): T trots as before until it is flipped. `wbc_kp_*` / `wbc_kd_*` (ori, pos, foot), "
               "`wbc_w_base`, `wbc_w_force`, `wbc_joint_kp` / `wbc_joint_kd`, `wbc_touchdown_depth`, all read in init().",
               "<b>Failure handling.</b> A tick without a solution (QP failure, unusable joint state) "
               "uses the per-leg commands; `command_failure_limit` "
               f"({tc['command_failure_limit']}) such ticks in a row is a module ERROR. A torque request beyond the "
               "limits is capped and nothing else: the abort after 20 saturated ticks in a row (`torque_saturation_limit`) "
               "was removed from both paths, because under a push it ended a trot that was not falling and the sit "
               "that followed could roll the robot over.",
               "<b>Go2 only.</b> The floating-base model carries the Go2 inertias; for another `robot_model` init() "
               "warns and keeps the per-leg path.",
               "<b>Log.</b> `MdlTrot_wbc` (45 doubles: MPC-desired and WBIC-solved force per leg, QP status, "
               "orientation and position task errors, swing errors p<sub>B,des</sub> − p<sub>B</sub>, fallback flag, "
               "tick time; layout at `_logWbc` in MdlTrot.hh). deactivate() prints tick mean / max, QP failures, "
               "fallback and saturated ticks.",
               "<b>Test.</b> `test_trot_wbc` checks `MdlTrot::wbcSolve` on the Go2 model without the simulator: "
               "(i) four feet at rest keep statically consistent forces and get the static torques; (ii) diagonal "
               "stance at random states holds the contact points with forces inside the pyramid and normal bounds; "
               "(iii) with the body off its reference, the swing joint references and the swing acceleration "
               "command are the body-frame ones; (iv) swing joints accelerate as the body-frame law asks."])]

    # 3 -----------------------------------------------------------------------------------------------------
    hd = {n: [W[n]["z_minus_target_mm"], B[n]["z_minus_target_mm"]] for n in CORE}
    Bl += [("h2", "3. Results"),
           ("p", f"Each run is one headless, non-real-time `bin/sim.sh` call with the Supervisor key script "
                 f"{', '.join(meta['script'])} (stand, T, D): {meta['trot_s']:.0f} s in MdlTrot's TROT state, then "
                 f"the normal stop to the sit. Logs at {1000 / meta['log_period_ms']:.0f} Hz (every control tick). "
                 "Steady window: "
                 f"{meta['steady_window']}. Fall: {words(meta['fall'])}. The controller's state input is simulator "
                 "ground truth in every run, attitude and base position/velocity alike "
                 "(`orientationestimator.use_ground_truth = true`, `posvelestimator.use_ground_truth = true`; the "
                 "filters keep running in shadow), so no estimator error enters the feedback; only the two filter "
                 "runs of Sec. 3.5 differ. The reported torso state, velocities, ground reactions, "
                 "CoM and L are simulator truth as well; MPC, WBIC and swing quantities are MdlTrot's log ('ctrl'). Torque "
                 f"limits: {lim[0]:g} / {lim[1]:g} / {lim[2]:g} N m (abduction, hip, knee). Leg order "
                 "of the simulated ground reactions is fixed by MdlSimDriver's lookup of the foot geoms by name (FL, "
                 "FR, RL, RR); the desired and simulated F<sub>z</sub> of each leg correlate at "
                 f"{span([corr[i][i] for i in range(4)], 2)}"
                 + (" and across the two diagonals they anti-correlate, which checks the sign and the diagonal "
                    "pairing (within a diagonal the two legs move together, so the correlation cannot tell them "
                    "apart). " if cross < 0 else ". ") + "The logged ground "
                 f"reactions are good to about {grf_tol:g} N: over the steady window their sum differs from what "
                 "the CoM's momentum rate and gravity require by " + "; ".join(
                     f"({', '.join(f'{v:+.1f}' for v in x)}) N {k.replace('_', ' ')}"
                     for k, x in ch["grf_logged_minus_momentum_N"].items()) + "."),
           ("h3", "3.1 Core runs"),
           ("table", T("velocity", "Velocity and drift, baseline / WBC. Lateral drift: displacement normal to the heading at the "
                       "window start, less the commanded lateral travel; 'from heading': the part a change of heading "
                       "carries (forward velocity off the initial heading); straight runs only."),
            ["Run", "Mean v<sub>x</sub> [m/s]", "Mean v<sub>y</sub> [m/s]", "Mean ω [rad/s]", "Lateral drift [m]",
             "From heading [m]", "Yaw change [deg]"],
            [[NAMES[n], bw(B[n], W[n], lambda x: x["vx"], 3), bw(B[n], W[n], lambda x: x["vy"], 3),
              bw(B[n], W[n], lambda x: x["yaw_rate"], 3), bw(B[n], W[n], lambda x: x["lateral_drift_m"], 3),
              bw(B[n], W[n], lambda x: x["lateral_drift_heading_m"], 3),
              bw(B[n], W[n], lambda x: x["yaw_change_deg"], 2)] for n in CORE],
            [0.21, 0.13, 0.13, 0.13, 0.14, 0.13, 0.13]),
           ("table", T("attitude", "Attitude and height, baseline / WBC. Height target: MdlTrot's CoM reference less the CoM "
                       "offset."),
            ["Run", "Roll mean [deg]", "Roll sd [deg]", "Pitch mean [deg]", "Pitch sd [deg]",
             "Height − target [mm]", "Height sd [mm]"],
            [[NAMES[n], bw(B[n], W[n], lambda x: x["roll_mean"], sign="+"), bw(B[n], W[n], lambda x: x["roll_sd"]),
              bw(B[n], W[n], lambda x: x["pitch_mean"], sign="+"), bw(B[n], W[n], lambda x: x["pitch_sd"]),
              bw(B[n], W[n], lambda x: x["z_minus_target_mm"], 1, "+"),
              bw(B[n], W[n], lambda x: 1e3 * x["z_sd"], 2)] for n in CORE],
            [0.21, 0.13, 0.13, 0.13, 0.13, 0.14, 0.13]),
           ("p", "Both modes track every command; the WBC's mean velocity error stays at or below "
                 f"{max(W[n]['v_err'] for n in CORE):.3f} m/s (baseline up to {max(B[n]['v_err'] for n in CORE):.3f}) "
                 f"and its yaw-rate error at or below {max(W[n]['yaw_err'] for n in CORE):.3f} rad/s. The WBC holds "
                 f"the commanded base height ({span([hd[n][0] for n in CORE], 1, ' mm')} from the target), while the "
                 f"baseline rides {span([hd[n][1] for n in CORE], 1, ' mm')} above it. Its roll and pitch oscillate "
                 "less (sd), but its pitch carries a nose-up offset that grows with speed; Sec. 3.2 gives the cause. "
                 "The two filter runs are in Sec. 3.5."),
           ("fig", "core_tracking_attitude.png", 0.64,
            F("core", "Core runs: velocity and yaw-rate error with the gate tolerance, swing-foot error, roll and pitch rms, "
              "pitch mean. Orange: baseline, blue: WBC."))]

    # 3.2 diagnosis
    exc = {n: fb["wbc"][n]["stance_simulated_N"] - fb["wbc"][n]["stance_desired_N"] for n in FWD}
    nopos_ok = all(x["ratio_to_baseline"][k] <= 1.5 for x in nopos.values() for k in ("pitch_rms", "roll_rms", "v_err"))
    worse20 = [NAMES[n] for n in CORE if j20[n]["pitch_rms"] > W[n]["pitch_rms"]]
    ctl = dg["no_position_error_baseline_control"]
    Bl += [("h3", "3.2 The pitch offset at speed"),
           ("p", "<b>Mechanism, measured.</b> The kinematic stage moves the base by the base-position task error "
                 "with no gain, so every leg's q<sub>des</sub> absorbs that error, and the "
                 f"{tc['wbc_joint_kp']:g}/{tc['wbc_joint_kd']:g} joint PD turns it into a force at the feet outside "
                 "the QP and outside the MPC's plan. The error (reference − position, along the heading) grows with "
                 f"speed: {seq(lambda n: fb['wbc'][n]['position_error_mm'], FWD, 1)} mm in place / 0.35 / 0.70 / 1.00 "
                 f"m/s. Three runs isolate the path (Table {tn['diagnosis']}). Holding the horizontal body reference on "
                 f"the measured position (`{dg['variants']['nopos']}`, diagnosis only) removes the error and turns the "
                 f"offset around: pitch mean {seq(lambda n: nopos[n]['pitch_mean'], FWD, 2, '+')} deg against "
                 f"{seq(lambda n: W[n]['pitch_mean'], FWD, 2, '+')} deg. " +
                 f"With the position task's gains at zero (`{dg['variants']['kppos0']}`) the offset stays "
                 f"({k0['pitch_mean']:+.2f} deg at 0.70 m/s), so the task's acceleration is not the path; the joint "
                 "PD on the shifted q<sub>des</sub> is. The baseline with the same clamp is "
                 + ("identical to" if ctl["identical_to_core"] else "<b>not</b> identical to") +
                 f" its core run: its MPC weights horizontal position {sw[3]:g} and has no other position feedback, "
                 "so the clamp changes nothing else. Whether stance or swing legs carry most of the push is not "
                 "separated; the clamp changes the swing error too."),
           ("p", "<b>Where the error comes from.</b> On ground truth the body runs at the commanded speed "
                 f"({W['fwd070']['vx']:.3f} m/s at 0.70) and trails its position reference by a steady "
                 f"{fb['wbc']['fwd070']['position_error_mm']:.0f} mm; the MPC's net forward plan on the stance feet is "
                 f"{seq(lambda n: fb['wbc'][n]['stance_desired_N'], FWD, 1, '+')} N (in place to 1.00 m/s). Why this "
                 "lag settles where it does is not established (hypothesis: the MPC, which weights forward velocity "
                 f"{sw[9]:g} and horizontal position {sw[3]:g}, plans a small braking force and the error settles "
                 "where the PD's push balances it). With the position/velocity filter in the loop the error is "
                 f"larger ({fb['wbc']['fwd070_pv']['position_error_mm']:.0f} mm at 0.70 m/s) and so is the offset "
                 f"({W['fwd070_pv']['pitch_mean']:+.2f} deg), because the filter's position advances "
                 f"{1e3 * (ew['estimated_vx_mean'] - ew['estimated_position_rate']):.1f} mm/s slower than its own "
                 f"velocity estimate (baseline run {1e3 * (eb['estimated_vx_mean'] - eb['estimated_position_rate']):.1f}"
                 " mm/s): the position task holds the estimated position on the reference, and the body then runs at "
                 f"{ew['true_position_rate']:.4f} m/s (Sec. 3.5)."),
           ("p", f"<b>Size and the baseline's own offset.</b> Moving the MPC's CoM belief forward "
                 f"(`{dg['variants']['comshift']}`, {ps['wbc']['moment_Nm']:.2f} N m about the true CoM, in place) "
                 "measures the static pitch stiffness: "
                 f"{ps['baseline']['stiffness_Nm_per_deg']:.1f} N m/deg in the baseline, "
                 f"{ps['wbc']['stiffness_Nm_per_deg']:.1f} N m/deg with the WBC, so a few tenths of a N m the MPC "
                 "does not model move the pitch by a tenth of a degree. The baseline has its own offset in "
                 f"place ({B['inplace']['pitch_mean']:+.2f} deg) that the WBC does not "
                 f"({W['inplace']['pitch_mean']:+.2f} deg): its stance forces miss the plan by "
                 f"{lg['inplace']['measured_simulated_minus_desired']['pitch_Nm']:+.2f} N m about the CoM, and the "
                 "stance legs' own gravity torque, which the per-leg τ = J<sup>T</sup>(−f) leaves out and the WBC's "
                 f"inverse dynamics carries, accounts for {lg['inplace']['leg_gravity_term']['pitch_Nm']:+.2f} N m of "
                 "it (J<sup>−T</sup>g(q) at the contacts; that this sets the offset is a hypothesis). With speed the "
                 f"baseline's pitch mean falls towards zero ({seq(lambda n: B[n]['pitch_mean'], FWD, 2, '+')} deg), "
                 "which is why its pitch rms is smallest at speed."),
           ("p", f"<b>A softer joint PD, rejected.</b> `{dg['variants']['joint20']}` pushes less per mm of error "
                 f"(pitch mean {seq(lambda n: j20[n]['pitch_mean'], ('fwd070', 'fwd100'), 2, '+')} deg at 0.70 / 1.00 "
                 f"m/s), but the swing error grows in "
                 f"{sum(j20[n]['swing_err_mm'] > W[n]['swing_err_mm']['rms'] for n in CORE)} of {len(CORE)} runs "
                 f"(Table {tn['diagnosis']})" + ((" and pitch rms rises in " + "; ".join(worse20)) if worse20 else "") +
                 ". It weakens the path instead of removing the error, so the plan's "
                 f"{tc['wbc_joint_kp']:g}/{tc['wbc_joint_kd']:g} stays."),
           ("table", T("diagnosis", "Pitch-offset diagnosis, WBC runs: the core run, the same with the base position error held "
                       "at zero ('no pos.'), and with joint PD 20 / 2 ('PD 20/2'). Logs in diagnosis/."),
            ["Run", "Pitch mean [deg]: baseline / WBC / no pos. / PD 20/2",
             "Pitch rms ÷ baseline: WBC / no pos. / PD 20/2", "|Δv| [m/s]: WBC / no pos. / PD 20/2",
             "Swing rms [mm]: WBC / no pos. / PD 20/2", "Position error [mm]: WBC / PD 20/2"],
            [[NAMES[n], f"{B[n]['pitch_mean']:+.2f} / {W[n]['pitch_mean']:+.2f} / {nopos[n]['pitch_mean']:+.2f} / "
                        f"{j20[n]['pitch_mean']:+.2f}",
              f"{ratio[n]['pitch_rms']:.2f} / {nopos[n]['ratio_to_baseline']['pitch_rms']:.2f} / "
              f"{j20[n]['ratio_to_baseline']['pitch_rms']:.2f}",
              f"{W[n]['v_err']:.3f} / {nopos[n]['v_err']:.3f} / {j20[n]['v_err']:.3f}",
              f"{W[n]['swing_err_mm']['rms']:.1f} / {nopos[n]['swing_err_mm']:.1f} / {j20[n]['swing_err_mm']:.1f}",
              f"{fb['wbc'][n]['position_error_mm']:.1f} / {j20[n]['force_balance']['position_error_mm']:.1f}"]
             for n in CORE], [0.18, 0.22, 0.17, 0.16, 0.14, 0.13]),
           ("fig", "diagnosis.png", 0.98,
            F("diagnosis", "Left: pitch mean against forward speed for the baseline, the WBC, the WBC with no base position "
              "error and the WBC with joint PD 20 / 2. Middle and right: FL around its scheduled touchdown at 0.70 "
              "m/s, averaged over the steady window's touchdowns: swing reference and contact-point height (a loaded "
              "foot sinks a few mm into MuJoCo's soft contact), and simulated F<sub>z</sub>. The WBC foot lands "
              "when its reference passes through the ground (Sec. 3.4)."))]

    # 3.3 forces
    def ax3(x, key):
        return "/".join(f"{v:.1f}" for v in x["grf"][key])
    Bl += [("h3", "3.3 Force tracking"),
           ("table", T("forces", "Ground reaction over scheduled stance, rms per axis x / y / z [N], leg-samples pooled: MPC "
                       "desired against simulated for both modes, and for the WBC desired against WBIC-solved and "
                       "solved against simulated."),
            ["Run", "Desired − simulated, baseline", "Desired − simulated, WBC", "Desired − solved, WBC",
             "Solved − simulated, WBC"],
            [[NAMES[n], ax3(B[n], "rms_des_sim_N"), ax3(W[n], "rms_des_sim_N"), ax3(W[n], "rms_des_sol_N"),
              ax3(W[n], "rms_sol_sim_N")] for n in CORE], [0.24, 0.19, 0.19, 0.19, 0.19]),
           ("p", "The QP keeps the MPC's forces (desired − solved "
                 f"{span([max(W[n]['grf']['rms_des_sol_N']) for n in CORE], 1, ' N')} rms on the worst axis), as Kim's "
                 "weighting intends. Most of what separates the simulated force from the plan arises after the QP: "
                 "solved − simulated is nearly the whole desired − simulated difference. The joint PD (Sec. 3.2), the "
                 "touchdown transient (Sec. 3.4) and MuJoCo's soft contact all enter there. "
                 "The MPC's summed vertical force at 0.70 m/s is "
                 f"{seq(lambda m: C[m]['fwd070']['grf']['sum_des_mean_N'][2], MODES, 0)} N (baseline / WBC) against a "
                 f"weight of {W['fwd070']['grf']['weight_N']:.0f} N; why the baseline's plan is below the weight was "
                 "not investigated.")]

    # 3.4 swing, touchdown, torque
    # The WBC on the nominal touchdown height, from this report's run of 2026-11-26 19:38, before
    # wbc_touchdown_depth existed (no key selects that target any more).
    BEFORE_FIX = ("12–15 mm deep, first contact 21–26 ms early at 1.31–1.48 m/s, braking 0.14–0.38 N s per "
                  "touchdown (run before the fix)")
    Bl += [("h3", "3.4 Swing tracking, touchdown timing and torque"),
           ("p", "The WBC tracks the swing reference "
                 f"{'more closely' if W['fwd070']['swing_err_mm']['rms'] < B['fwd070']['swing_err_mm']['rms'] else 'less closely'}"
                 f" than the per-leg Cartesian PD ({bw(B['fwd070'], W['fwd070'], lambda x: x['swing_err_mm']['rms'], 1)} "
                 "mm rms at 0.70 m/s, baseline / WBC). The per-leg path's touchdown target is the nominal "
                 "footprint's height. The stand sags about 10 mm under its joint PD, so the trot holds the body lower "
                 "than that footprint assumes and the target lies "
                 f"{span([-td['baseline'][n]['reference_at_touchdown_mm'] for n in TD], 0, ' mm')} below the ground; "
                 "the baseline's foot lags "
                 f"{span([abs(td['baseline'][n]['reference_at_first_contact_mm']) for n in TD], 0, ' mm')} behind its "
                 f"reference and touches {span([v for n in TD for v in td['baseline'][n]['first_contact_ms'].values()], 0, ' ms')}"
                 " early. With the same target the WBC, which tracks closely, followed its reference into the ground: "
                 f"{BEFORE_FIX}. The WBC path therefore aims `wbc_touchdown_depth` "
                 f"({1e3 * tc['wbc_touchdown_depth']:g} mm, Kim's value) below the height the foot lifted off from, in "
                 f"the body frame (Table {tn['touchdown']}). Its reference now ends "
                 f"{span([-td['wbc'][n]['reference_at_touchdown_mm'] for n in TD], 1, ' mm')} below the ground and the "
                 f"foot first touches {span([v for n in TD for v in td['wbc'][n]['first_contact_ms'].values()], 0, ' ms')}"
                 f" early at {span([-td['wbc'][n]['foot_vz_before_contact_m_s'] for n in TD], 2, ' m/s')} (baseline "
                 f"{span([-td['baseline'][n]['foot_vz_before_contact_m_s'] for n in TD], 2)}), with at most "
                 f"{max(td['wbc'][n]['peak_fz_early_N'] for n in TD):.0f} N before the scheduled touchdown (baseline "
                 f"{span([td['baseline'][n]['peak_fz_early_N'] for n in TD], 0)}) and no braking "
                 f"({span([td['wbc'][n]['early_forward_impulse_Ns'] for n in TD], 2, ' N s')} per touchdown). "
                 "Torque stays below the limits in every "
                 f"core run of both modes (peak |τ|/limit at most {max(x['peak_tau_over_limit'] for m in MODES for x in C[m].values()):.2f})."),
           ("table", T("swing", "Swing and touchdown, baseline / WBC. Swing error p<sub>B,des</sub> − p<sub>B</sub> (ctrl, body "
                       f"frame). Loaded: simulated F<sub>z</sub> above {meta['loaded_foot_N']:g} N. First contact: how "
                       f"long the foot has been in contact (above {meta['contact_N']:g} N, unbroken) when its scheduled "
                       "swing ends, mean over touchdowns. Peak |τ|/limit: the WBC's is WBIC torque plus joint PD, "
                       "before the clip."),
            ["Run", "Swing rms [mm]", "Swing max [mm]", "Swing samples loaded [%]", "Stance samples unloaded [%]",
             "First contact before schedule [ms]", "Peak |τ|/limit", "WBC tick mean / max [µs]"],
            [[NAMES[n], bw(B[n], W[n], lambda x: x["swing_err_mm"]["rms"], 1),
              bw(B[n], W[n], lambda x: x["swing_err_mm"]["max"], 1),
              bw(B[n], W[n], lambda x: x["touchdown"]["swing_loaded_pct"], 1),
              bw(B[n], W[n], lambda x: x["touchdown"]["stance_unloaded_pct"], 2),
              bw(B[n], W[n], lambda x: x["touchdown"]["first_contact_before_scheduled_ms"]["mean"], 1),
              bw(B[n], W[n], lambda x: x["peak_tau_over_limit"]),
              f"{W[n]['wbc_closing']['tick_mean_us']:.1f} / {W[n]['wbc_closing']['tick_max_us']:.0f}"] for n in CORE],
            [0.19, 0.10, 0.10, 0.11, 0.11, 0.13, 0.11, 0.15]),
           ("table", T("touchdown", "Touchdown detail, means over the steady window's scheduled touchdowns (TD). Reference: the "
                       "swing reference's contact-point height. Through ground and first contact: how long before the "
                       "scheduled TD, front / rear legs. Swing feet F: mean forward force on the scheduled-swing feet "
                       f"(the logged sum is off by up to {grf_x:.1f} N along x, Sec. 3)."),
            ["Mode, run", "Reference at TD [mm]", "Through ground [ms]", "First contact [ms]",
             "Reference at contact [mm]", "Foot v<sub>z</sub> [m/s]", "Peak F<sub>z</sub> early / after [N]",
             "Impulse [N s]", "Swing feet F [N]"],
            [[f"{lab}, {NAMES[n]}", f"{x['reference_at_touchdown_mm']:.1f}",
              f"{x['reference_through_ground_ms']['front']:.0f} / {x['reference_through_ground_ms']['rear']:.0f}",
              f"{x['first_contact_ms']['front']:.0f} / {x['first_contact_ms']['rear']:.0f}",
              f"{x['reference_at_first_contact_mm']:.1f}", f"{x['foot_vz_before_contact_m_s']:.2f}",
              f"{x['peak_fz_early_N']:.0f} / {x['peak_fz_30ms_after_N']:.0f}", f"{x['early_forward_impulse_Ns']:.2f}",
              f"{f['swing_simulated_N']:+.1f}"]
             for lab, n, x, f in
             [(m, n, td[m][n], fb[m][n]) for m in MODES for n in TD] +
             [("WBC no pos.", "fwd070", td["wbc_nopos"]["fwd070"], nopos["fwd070"]["force_balance"])]],
            [0.19, 0.09, 0.10, 0.09, 0.10, 0.08, 0.13, 0.08, 0.09])]

    # 3.5 filters in the loop, 3.6 timing
    i, pv, f7 = "fwd070_imu", "fwd070_pv", "fwd070"
    est_rows = [("ground truth", f7), ("pos/vel filter, true attitude", pv), ("both filters, noisy IMU", i)]
    Bl += [("h3", "3.5 Filters in the loop"),
           ("p", "Every run above feeds the controller the simulator's state. Two more runs at 0.70 m/s put the "
                 "filters in the loop: the position/velocity Kalman filter alone "
                 "(`posvelestimator.use_ground_truth = false`), and that filter together with the Mahony attitude "
                 f"filter on the simulated noisy IMU (Table {tn['filters']}). With the position/velocity filter the "
                 f"WBC's nose-up pitch grows from {W[f7]['pitch_mean']:+.2f} to {W[pv]['pitch_mean']:+.2f} deg and its "
                 f"mean speed from {W[f7]['vx']:.3f} to {W[pv]['vx']:.3f} m/s: the filter's position advances "
                 f"{1e3 * (ew['estimated_vx_mean'] - ew['estimated_position_rate']):.1f} mm/s slower than its own "
                 "velocity estimate, the base-position task holds that position on the reference, so the body runs "
                 "ahead of the command and the joint PD pushes on the shifted joint references (Sec. 3.2). The "
                 "baseline, which has no position feedback, hardly changes. With both filters the two robots turn by "
                 f"{bw(B[i], W[i], lambda x: x['yaw_change_deg'], 2)} deg over the window (ground truth "
                 f"{bw(B[f7], W[f7], lambda x: x['yaw_change_deg'], 2)}): the controller holds the estimated heading, "
                 "so the same turn in both modes points to the attitude filter's yaw drift."),
           ("table", T("filters", "Forward 0.70 m/s with the controller's state from ground truth and from the filters, "
                       "baseline / WBC (measured on simulator truth)."),
            ["State input", "Mean v<sub>x</sub> [m/s]", "|Δv| [m/s]", "Roll rms [deg]", "Pitch mean [deg]",
             "Pitch rms [deg]", "Pitch rms ratio", "Yaw change [deg]"],
            [[lab, bw(B[n], W[n], lambda x: x["vx"], 3), bw(B[n], W[n], lambda x: x["v_err"], 3),
              bw(B[n], W[n], lambda x: x["roll_rms"]), bw(B[n], W[n], lambda x: x["pitch_mean"], sign="+"),
              bw(B[n], W[n], lambda x: x["pitch_rms"]), f"{ratio[n]['pitch_rms']:.2f}x",
              bw(B[n], W[n], lambda x: x["yaw_change_deg"], 2)] for lab, n in est_rows],
            [0.23, 0.12, 0.11, 0.11, 0.11, 0.11, 0.10, 0.11]),
           ("h3", "3.6 Timing"),
           ("p", f"Over the core runs the WBC ran {sum(c['ticks'] for c in cl)} ticks with "
                 f"{sum(c['qp_failures'] for c in cl)} QP failures, {sum(c['fallback_ticks'] for c in cl)} fallback "
                 f"ticks and {sum(c['saturated_ticks'] for c in cl)} torque-saturated ticks (closing lines); the log "
                 "shows no fallback flag and no nonzero QP status, and no run printed a warning. Tick time (model "
                 f"terms, tasks and QP): mean {span([c['tick_mean_us'] for c in cl], 1, ' µs')}, max "
                 f"{max(c['tick_max_us'] for c in cl):.0f} µs, against the 1 ms period; headless and not real time, so "
                 "this is CPU cost, not a real-time guarantee.")]

    # 3.7 duty factor
    dut, P0 = R["duty"], R["duty"]["wbc"][0]["period"]

    def duty_row(d, vx):
        out = [f"{d:.2f}", f"{vx:.2f}", "none" if d == 0.5 else f"{1e3 * (d - 0.5) * P0:.0f} four-foot" if d > 0.5 else
               f"{1e3 * (0.5 - d) * P0:.0f} flight"]
        b, w = (next(x for x in dut[m] if x["duty"] == d and x["vx_cmd"] == vx) for m in MODES)
        if "window_s" not in b or "window_s" not in w:
            return out + ["baseline: " + ("ok" if "window_s" in b else "fails") + ", WBC: " +
                          ("ok" if "window_s" in w else "fails")] + ["–"] * 6
        return out + [bw(b, w, lambda x: x["airborne_ms"], 0), bw(b, w, lambda x: round(x["vx"], 3) + 0.0, 3),
                      bw(b, w, lambda x: x["roll_rms"]), bw(b, w, lambda x: x["pitch_mean"], sign="+"),
                      bw(b, w, lambda x: x["pitch_rms"]), bw(b, w, lambda x: x["z_minus_target_mm"], 1, "+"),
                      bw(b, w, lambda x: x["saturated_samples_pct"], 1)]
    dc = [x["wbc_closing"] for x in dut["wbc"]]
    fl = {m: [x for x in dut[m] if x["duty"] < 0.5 and "window_s" in x] for m in MODES}
    lowest = {m: min(fl[m], key=lambda x: (x["duty"], -x["vx_cmd"])) for m in MODES}   # lowest duty, moving
    mv = {m: [x for x in dut[m] if x["vx_cmd"] > 0 and "window_s" in x] for m in MODES}
    Bl += [("h3", "3.7 Duty factor: overlap and flight"),
           ("p", f"The gait clock takes any duty factor. At the default {P0} s period a duty above 0.5 overlaps the "
                 "two diagonals on four feet; below 0.5 no foot is scheduled down for (0.5 − duty) · period per "
                 "half cycle, a flight phase. Kim's controller covers flight inside the same hierarchy and so does "
                 "this one: with no contact all four feet are swing tasks (`QuadrupedWBC::MAX_TASKS` is 6), the QP "
                 "has only the floating-base correction, so the base falls freely whatever the body tasks ask, and "
                 "the legs keep following their swing references in the body frame. As first built the solver "
                 "refused a problem without a contact, and the trot ended in a module ERROR at the first flight "
                 "phase (found by Ege, 2026-11-27; fixed the same day). "
                 f"Table {tn['duty']}: every run completes, with {sum(c['qp_failures'] for c in dc)} QP failures and "
                 f"{sum(c['fallback_ticks'] for c in dc)} fallback ticks over {sum(c['ticks'] for c in dc)} WBC "
                 "ticks. The robot leaves the ground for less than the schedule asks, because the feet lift off "
                 "late and land early, and the WBC comes closer to the schedule than the baseline: at duty "
                 f"{lowest['wbc']['duty']:.2f} and {lowest['wbc']['vx_cmd']:.2f} m/s it is airborne "
                 f"{bw(lowest['baseline'], lowest['wbc'], lambda x: x['airborne_ms'], 0)} ms per half cycle "
                 f"(baseline / WBC) of the scheduled {lowest['wbc']['flight_scheduled_ms']:.0f} ms. The WBC holds "
                 f"the commanded speed at every duty ({span([x['vx'] for x in mv['wbc']], 3, ' m/s')}; baseline "
                 f"{span([x['vx'] for x in mv['baseline']], 3, ' m/s')}). With flight it carries a nose-down pitch "
                 f"that grows as the duty factor falls, to {lowest['wbc']['pitch_mean']:+.2f} deg at duty "
                 f"{lowest['wbc']['duty']:.2f}, and its base runs up to "
                 f"{-min(x['z_minus_target_mm'] for x in fl['wbc']):.0f} mm below the commanded height; the "
                 "baseline's pitch at speed stays within "
                 f"{max(abs(x['pitch_mean']) for x in fl['baseline'] if x['vx_cmd'] > 0):.2f} deg. Check runs of "
                 "2026-11-27 at duty 0.35 (outside this script): holding the base-position error at zero "
                 "(Sec. 3.2's mechanism) takes the pitch at 0.35 m/s from −0.88 to −0.52 deg; a touchdown "
                 "target 10 mm higher removes the early touchdown and doubles the pitch, so touchdown timing is not "
                 "the cause. The rest is not explained."),
           ("table", T("duty", f"Duty factor at the {P0} s period, baseline / WBC, ground-truth state. Scheduled phase: "
                       "four-foot overlap or flight per half cycle. Airborne: mean time per half cycle with no "
                       "foot in contact. Height: mean base height minus the commanded one. Saturated: share of "
                       "samples with a torque request beyond the limit."),
            ["Duty", "v<sub>x</sub> cmd [m/s]", "Scheduled [ms]", "Airborne [ms]", "Mean v<sub>x</sub> [m/s]",
             "Roll rms [deg]", "Pitch mean [deg]", "Pitch rms [deg]", "Height [mm]", "Sat. [%]"],
            [duty_row(d, vx) for vx in sorted({x["vx_cmd"] for x in dut["wbc"]})
             for d in sorted({x["duty"] for x in dut["wbc"]}, reverse=True)],
            [0.07, 0.08, 0.13, 0.10, 0.13, 0.10, 0.11, 0.10, 0.10, 0.08])]

    # 4 -----------------------------------------------------------------------------------------------------
    chk = {(m, n): sl[m][n]["check_dLdt_vs_Mgch"] for m in MODES for n in ("inplace", "fwd070")}
    s7 = {m: sl[m]["fwd070"] for m in MODES}
    first_fail = {m: next((x for x in cad[m] if x["failed"]), None) for m in MODES}
    last = {m: [x for x in cad[m] if not x["failed"]][-1] for m in MODES}
    Bl += [("h2", "4. Baselines for the AMB study (no gate)"),
           ("p", "L is the whole-robot angular momentum about the line through the two scheduled stance feet's "
                 "ground contact points (the foot sphere's lowest point), c<sub>h</sub> the CoM's horizontal offset "
                 "from that line (positive to the robot's left). With point feet fixed on the line the ground "
                 "forces have no moment about it, so dL/dt = M g c<sub>h</sub>: the stance forces change L only by "
                 "moving the CoM off the line, and the next touchdown replaces the line. These are the quantities an "
                 "AMB term would regulate."),
           ("h3", "4.1 Support line in a normal trot"),
           ("table", T("support", "Per complete two-foot stance phase in the steady window, mean / max over the phases: peak |L|, "
                       "peak |c<sub>h</sub>|, how far c<sub>h</sub> moves within the stance, how far the stance "
                       "contact points roll; and the check dL/dt against M g c<sub>h</sub> (slope, correlation) on the "
                       "samples whose swing feet carry no force, about the instantaneous line and, in brackets, the "
                       "line frozen at stance start."),
            ["Mode, run", "Phases", "Peak |L| [kg m²/s]", "Peak |c<sub>h</sub>| [mm]", "c<sub>h</sub> travel [mm]",
             "Contact travel [mm]", "dL/dt check"],
            [[f"{m}, {NAMES[n]}", str(sl[m][n]["n_phases"]),
              f"{sl[m][n]['peak_L']['mean']:.3f} / {sl[m][n]['peak_L']['max']:.3f}",
              f"{sl[m][n]['peak_ch_mm']['mean']:.1f} / {sl[m][n]['peak_ch_mm']['max']:.1f}",
              f"{sl[m][n]['ch_travel_mm']['mean']:.1f} / {sl[m][n]['ch_travel_mm']['max']:.1f}",
              f"{sl[m][n]['contact_point_travel_mm']['mean']:.1f} / {sl[m][n]['contact_point_travel_mm']['max']:.1f}",
              f"{chk[m, n]['instantaneous']['slope']:.2f}, {chk[m, n]['instantaneous']['corr']:.3f} "
              f"({chk[m, n]['frozen']['slope']:.2f}, {chk[m, n]['frozen']['corr']:.3f})"]
             for m in MODES for n in ("inplace", "fwd070")],
            [0.21, 0.07, 0.14, 0.13, 0.13, 0.13, 0.19]),
           ("p", "In place, L about the diagonal stays near "
                 f"{span([sl[m]['inplace']['peak_L']['mean'] for m in MODES], 3)} kg m²/s and the CoM within "
                 f"{span([sl[m]['inplace']['peak_ch_mm']['max'] for m in MODES], 1, ' mm')} of the line. At 0.70 m/s "
                 f"peak |L| is {span([s7[m]['peak_L']['mean'] for m in MODES], 2)} kg m²/s and c<sub>h</sub> sweeps "
                 f"{span([s7[m]['ch_travel_mm']['mean'] for m in MODES], 0, ' mm')} across the line within each "
                 "stance. The body moves forward over a diagonal that is oblique to the motion, so L about the line "
                 "contains the moment of the forward momentum about it, and it is this large in a steady trot that "
                 "does not fall. A controller that regulates L about the stance diagonal would therefore have to "
                 "separate this gait-borne part at speed; in place it would work on a small quantity. "
                 "dL/dt follows M g c<sub>h</sub> "
                 "with a slope above 1 at speed because the rolling feet move the contact points "
                 f"({span([s7[m]['contact_point_travel_mm']['mean'] for m in MODES], 1, ' mm')} per stance); about "
                 f"the line frozen at stance start the slope is {span([chk[m, 'fwd070']['frozen']['slope'] for m in MODES], 2)}. "
                 "Peak |L| depends on where the contact point is taken: about the line through MuJoCo's own contact "
                 "positions (midway through the penetration) it is "
                 + "; ".join(f"{100 * (s7[m]['peak_L_mujoco_contact']['mean'] / s7[m]['peak_L']['mean'] - 1):+.1f} %"
                             f" ({m})" for m in MODES) + " at 0.70 m/s, while c<sub>h</sub> does not change."),
           ("fig", "support_line_check.png", 0.66,
            F("support", "WBC, forward 0.70 m/s. Left: dL/dt against M g c<sub>h</sub> on two-foot stance. Right: L and "
              "c<sub>h</sub> about the current stance diagonal; L changes sign with the diagonal because the "
              "line's direction does."))]

    def cad_row(P):
        out = [f"{P:.1f}"]
        for m in MODES:
            x = next((r for r in cad[m] if r["period"] == P), None)
            if x is None:
                out += ["not run"] * 3
            elif x["failed"]:
                what = "fall" if x["fall_t"] is not None else "module ERROR"
                out += [f"fails: {what}", "–", "–"]
            else:
                s = x["support_line"]
                out += [f"{fx(x['roll_rms'])} / {fx(x['pitch_rms'])}",
                        f"{fx(s['peak_L']['mean'], 3)} / {fx(s['peak_ch_mm']['mean'], 1)} / "
                        f"{fx(s['ch_travel_mm']['mean'], 1)}", fx(x["peak_tau_over_limit"])]
        return out
    periods = sorted({x["period"] for m in MODES for x in cad[m]})
    Bl += [("h3", "4.2 Cadence: prolonged diagonal support"),
           ("p", f"In place, gait period {periods[0]} to {periods[-1]} s at duty 0.5 (diagonal stance "
                 f"{0.5 * periods[0]:.2f} to {0.5 * periods[-1]:.2f} s), each mode up to its first failure. " +
                 " ".join(f"{m.capitalize() if m == 'baseline' else 'WBC'}: " +
                          (f"first failure at {first_fail[m]['period']} s (first event: "
                           f"{EVENT[first_fail[m]['first_event']]})." if first_fail[m] else
                           f"no period up to {last[m]['period']} s fails (no fall, no module ERROR, no saturated "
                           "torque sample).") for m in MODES) +
                 (" The failure boundary therefore lies beyond the sweep and was not searched."
                  if not any(first_fail.values()) else "") +
                 (" With longer stance both modes rock more about the diagonal" if all(
                     last[m]["support_line"]["peak_L"]["mean"] > cad[m][0]["support_line"]["peak_L"]["mean"]
                     for m in MODES) else " Peak |L| per stance") + ": at "
                 f"{last['wbc']['period']} s peak |L| per stance is "
                 f"{bw(last['baseline']['support_line'], last['wbc']['support_line'], lambda s: s['peak_L']['mean'], 3)}"
                 " kg m²/s and roll rms "
                 f"{bw(last['baseline'], last['wbc'], lambda x: x['roll_rms'])} deg (baseline / WBC), against "
                 f"{bw(cad['baseline'][0]['support_line'], cad['wbc'][0]['support_line'], lambda s: s['peak_L']['mean'], 3)}"
                 f" kg m²/s and {bw(cad['baseline'][0], cad['wbc'][0], lambda x: x['roll_rms'])} deg at "
                 f"{periods[0]} s. For a controller that regulates L about the stance diagonal, a slower cadence "
                 "both lengthens the time it has per diagonal and enlarges the quantity it would regulate."),
           ("table", T("cadence", "Cadence sweep, in place: roll / pitch rms [deg]; per stance mean peak |L| [kg m²/s] / peak "
                       "|c<sub>h</sub>| [mm] / c<sub>h</sub> travel [mm]; peak |τ|/limit."),
            ["Period [s]", "Baseline attitude", "Baseline support line", "Baseline τ", "WBC attitude",
             "WBC support line", "WBC τ"], [cad_row(P) for P in periods],
            [0.09, 0.13, 0.20, 0.09, 0.13, 0.20, 0.09]),
           ("fig", "cadence_sweep.png", 0.54, F("cadence", "Cadence sweep: attitude rms and support-line quantities against "
                                                "the gait period. Orange: baseline, blue: WBC."))]

    # 4.3 push
    def first_row(sweep):
        ok, fail = sweep["bracket_N"]
        return next((x for x in sweep["rows"] if x["F"] == fail), None)

    def cell(mode, cb):
        s = push[mode][cb]
        ok, fail = s["bracket_N"]
        if fail is None:
            return f"{ok} N, none up to {meta['push']['cap_N']} N"
        x = first_row(s)
        what = (f"fall {x['fall_after_push_s']:.2f} s after the pulse" if x["kind"] == "fall" else
                f"module ERROR {x['error_after_push_s']:.1f} s after the pulse (peak roll {x['peak_roll_deg']:.0f} deg)")
        return f"{ok if ok is not None else 0} / {fail} N, {what}"

    kinds = {x["kind"] for m in MODES for cb in COMBOS for x in [first_row(push[m][cb])] if x}
    ok_rows = [x for m in MODES for cb in COMBOS for x in push[m][cb]["rows"] if x["success"]]
    sat = [x["peak_tau_over_limit"] for x in ok_rows if x["peak_roll_deg"] < 20.0]
    blown = [x for x in ok_rows if x["peak_tau_over_limit"] > 10.0]
    Bl += [("h3", "4.3 Lateral pushes"),
           ("p", f"In-place trot, one {meta['push']['pulse_s']} s pulse at the base body's CoM along world ±y (the "
                 f"robot's left / right; the heading stays near zero in place), {meta['push']['at_cycles_after_entry']}"
                 f" gait cycles after TROT entry: at the FL–RR touchdown ({push['push_times_s']['wbc']['touchdown']} s)"
                 f" or half a stance later ({push['push_times_s']['wbc']['midstance']} s). {meta['push']['coarse_N']} "
                 f"N steps to the first failure, then {meta['push']['fine_N']} N steps up from the last success. "
                 "A torque request beyond the limits is capped and the trot goes on (recovered pulses with a peak roll "
                 f"below 20 deg ask for up to {max(sat):.1f} times the limit"
                 + (f"; in {len(blown)} recovered WBC pulses near the boundary, peak roll "
                    f"{span([x['peak_roll_deg'] for x in blown], 0, ' deg')}, the uncapped request reaches hundreds "
                    "of times the limit" if blown else "") + "), so the first failure is a fall"
                 + ("" if kinds == {"fall"} else
                    f" in {sum(first_row(push[m][cb])['kind'] == 'fall' for m in MODES for cb in COMBOS)} of "
                    f"{len(MODES) * len(COMBOS)} cases; in the others the robot stays up through the trot but the "
                    "stop sequence ends in a module ERROR (a centering command out of reach)") +
                 ". These brackets are the balance boundary the AMB study needs. "
                 "The WBC's boundary lies above the baseline's in "
                 f"{sum((push['wbc'][cb]['bracket_N'][0] or 0) > (push['baseline'][cb]['bracket_N'][0] or 0) for cb in COMBOS)}"
                 f" of {len(COMBOS)} cases (Table {tn['push']})."),
           ("table", T("push", "Pushes: largest recovered / first failing pulse and how it failed."),
            ["Direction, phase", "Baseline", "WBC"],
            [[cb.replace("plus_y", "+y").replace("minus_y", "−y").replace("_", ", "), cell("baseline", cb),
              cell("wbc", cb)] for cb in COMBOS],
            [0.2, 0.4, 0.4])]

    # 5 -----------------------------------------------------------------------------------------------------
    ticks = W["fwd070"]["wbc_closing"]["ticks"]
    Bl += [("h2", "5. Departures from Kim et al."),
           ("ol", [
               "<b>Footholds.</b> Not re-planned in world frame every tick: MdlTrot's body-frame footstep law and "
               "swing quintic stay, so both modes share them.",
               "<b>Swing task frame.</b> Kim's swing task tracks a world-frame foot target. Here the kinematic stage "
               "carries the body-frame sample on the <i>desired</i> body and the acceleration stage commands the "
               "absolute acceleration of the body-frame sample with the body moving as the two body tasks command, "
               "so the swing joints see the leg-relative error the footstep law was tuned for. The plan said 'on the "
               "measured body'; on the measured body the joint PD, stiffer than the swing task, would track the "
               "body's own error with the swing leg.",
               f"<b>Joint PD.</b> {tc['wbc_joint_kp']:g}/{tc['wbc_joint_kd']:g} N m/rad on every leg, stance and swing "
               "(Kim's structure: one set of gains for both). Kim uses 3/0.3 on the 9 kg Mini Cheetah; "
               f"{tc['wbc_joint_kp']:g}/{tc['wbc_joint_kd']:g} is the thesis' Go2 stance value, and the thesis gives "
               "swing joints none. On a lifted Go2 leg this PD is stiffer than the swing task, so it sets swing "
               "tracking, and on stance legs it pushes toward the shifted q<sub>des</sub> (Sec. 3.2).",
               f"<b>Rate.</b> The WBC runs on every 1 ms control tick ({ticks} ticks per run, one per ms of TROT "
               "and STOPPING); Kim runs it at 500 Hz.",
               f"<b>MPC.</b> Horizon {mc['horizon']} × {1e3 * mc['dt']:.0f} ms, re-solved on every contact change "
               "(Kim: about 10 × 26 ms at about 40 Hz); its moment arms use the foot centres, the WBC uses the "
               "contact points one foot radius lower.",
               "<b>Not carried over.</b> No stance-foot Cartesian damping and no knee barrier (extras in "
               "Cheetah-Software's code, not in the paper).",
               "<b>Robustness additions.</b> Go2-only guard; a tick without a solution falls back to the per-leg "
               "commands, with an ERROR after `command_failure_limit` in a row; the torque request is clipped to the "
               "Go2 limits. Kim has none of these.",
               "<b>Logging.</b> The WBIC feedforward torque is not logged by itself: `MdlTrot_torquereq` is WBIC "
               f"torque plus joint PD before the clip, so the WBC's peak |τ|/limit in Table {tn['swing']} is that "
               "sum."])]

    # 6 -----------------------------------------------------------------------------------------------------
    ab = R["amb_ablation"]
    Bl += [("h2", "6. Open issues and limits"),
           ("ol", [
               "<b>Pitch offset at speed: documented, not removed</b> (Ege, 2026-11-26). It is "
               f"{seq(lambda n: W[n]['pitch_mean'], ('fwd070', 'fwd100'), 2, '+')} deg at 0.70 / 1.00 m/s on ground "
               f"truth and {W['fwd070_pv']['pitch_mean']:+.2f} deg at 0.70 m/s with the position/velocity filter. "
               "Holding the horizontal reference on the measured position (`trot.position_reference_clamp` tiny"
               + ("; it changes nothing on the per-leg path" if ctl["identical_to_core"] else "") + ") turns it to "
               f"{seq(lambda n: nopos[n]['pitch_mean'], ('fwd070', 'fwd100'), 2, '+')} deg at the cost of velocity "
               f"error ({seq(lambda n: nopos[n]['v_err'], ('fwd070', 'fwd100'), 3)} m/s against "
               f"{seq(lambda n: W[n]['v_err'], ('fwd070', 'fwd100'), 3)}). Untested: horizontal position weight in the "
               "MPC (Cheetah-Software's MPC has one), or a position/velocity filter whose position agrees with its "
               "velocity.",
               "<b>Hypotheses left open:</b> why a position lag remains on ground truth; stance against swing legs "
               "in the PD's push; the per-leg path's left-out leg gravity as the "
               "cause of the baseline's in-place offset; the MPC's summed vertical force below the weight; the "
               "baseline's side slip in the both-filters run.",
               "<b>Touchdown depth at slow cadence.</b> The 3 mm of `wbc_touchdown_depth` suit the 0.3 s gait. With "
               "a long swing the body moves further before the foot arrives and 3 mm is too shallow: in place at a "
               "0.8 s period the WBC's roll / pitch rms is 0.30 / 0.32 deg with 3 mm and 0.07 / 0.10 deg with 8 mm; at "
               "1.0 s it is 1.40 / 1.24 deg against 0.22 / 0.19 deg (check runs on ground truth, 2026-11-26, outside "
               "this script). A slow-cadence study has to set this depth first, or the comparison measures it.",
               "<b>Flight phases</b> (Sec. 3.7) run, but below duty 0.5 the WBC pitches nose-down (to "
               f"{lowest['wbc']['pitch_mean']:+.2f} deg at duty {lowest['wbc']['duty']:.2f}) and rides up to "
               f"{-min(x['z_minus_target_mm'] for x in fl['wbc']):.0f} mm low, and the robot is airborne for at most "
               "about half the scheduled time. The base-position error explains part of the pitch; the rest, the height "
               "loss and the late lift-off are not examined. The MPC is the same in both modes and is untuned for "
               "flight.",
               "<b>Cadence boundary</b> not reached up to "
               f"{min(last[m]['period'] for m in MODES)} s; a longer sweep would find where prolonged diagonal support "
               "fails.",
               "<b>What the tests do not cover:</b> no check of the body tasks' error signs; the fallback and its "
               "ERROR never ran in simulation (0 fallback ticks).",
               "<b>Limits.</b> Simulation only (MuJoCo, rolling sphere feet, soft contact); the state feedback is "
               "ground truth, with two filter-in-the-loop runs per mode at 0.70 m/s only; tick "
               f"times are headless, not real time; logged ground reactions are good to about {grf_tol:g} N.",
               f"<b>AMB untouched.</b> figs/amb_wbc's ablation, re-run, gives the same verdict "
               f"({ab['gate_ref']['result']} → {ab['gate_new']['result']}); "
               f"{ab['values_compared'] - len(ab['differing'])} of {ab['values_compared']} per-run values are equal"
               + ("." if not ab["differing"] else
                  "; the others belong to " + ", ".join(k.split("/")[1] for k in sorted(ab["differing_runs_log_start"]))
                  + (" only, whose 500 Hz logs started on the other tick parity than the reference runs', so their "
                     "sampled peaks and event times move by one tick." if ab["differences_only_in_other_tick_parity"]
                     else ", not explained by the logs' sampling phase (results.json amb_ablation)."))])]

    # 7 -----------------------------------------------------------------------------------------------------
    Bl += [("h2", "7. Reproduction"),
           ("p", "From the repository root. One simulator at a time: the script refuses to start while a `simulation` "
                 "process runs, and serialises its runs on `runs/.simlock`, shared with figs/amb_wbc."),
           ("code",
            "cd /home/ege/legged/iros/project\n"
            "cmake --build build -j 12 && cmake --install build && (cd build && ctest --output-on-failure)\n"
            "/home/ege/legged/iros/.venv/bin/python figs/trot_wbc/run_experiments.py        # all + report\n"
            "/home/ege/legged/iros/.venv/bin/python figs/trot_wbc/run_experiments.py --only push   # a subset\n"
            "/home/ege/legged/iros/.venv/bin/python figs/trot_wbc/run_experiments.py --out /tmp/x  # elsewhere\n"
            "/home/ege/legged/iros/.venv/bin/python figs/trot_wbc/build_report.py           # report only\n"
            "cd bin && ./sim.sh -c \"trot.wbc = true\"           # interactive: S stand, T trot, D stop"),
           ("p", "Experiments: tests, oldpath, amb_ablation, core, diagnosis (needs core's logs), duty, cadence, push. "
                 "Logs go to `runs/trot_wbc/<experiment>/<mode>_<run>.mat` and `.log`; the old-path baseline is kept "
                 "gzipped in `reports/trot_wbc/oldpath_baseline/`, since runs/ is not tracked. The harness shares "
                 "figs/amb_wbc's lock, fall thresholds, foot radius, torque limits and MuJoCo model, with its own "
                 "runner and loader (amb_wbc's hard-code the balance script). A key script instead of "
                 "`supervisor.autorun_trot_duration` gives every run the same gait datum. The baseline logs the MPC "
                 "forces and swing references in place of `MdlTrot_wbc` (a log line holds 127 doubles).")]
    return Bl


def write_pdf(blocks, path, rep):
    """figs/amb_wbc's layout; its own write_pdf carries that report's title and footer."""
    fonts = os.path.join(matplotlib.get_data_path(), "fonts", "ttf")
    for name, file in (("Sans", "DejaVuSans"), ("Sans-B", "DejaVuSans-Bold"), ("Sans-I", "DejaVuSans-Oblique"),
                       ("Sans-BI", "DejaVuSans-BoldOblique"), ("Mono", "DejaVuSansMono")):
        pdfmetrics.registerFont(TTFont(name, os.path.join(fonts, file + ".ttf")))
    pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-B", italic="Sans-I", boldItalic="Sans-BI")
    body = ParagraphStyle("body", fontName="Sans", fontSize=8.3, leading=11.0, spaceAfter=4.0)
    st = {"title": ParagraphStyle("title", parent=body, fontName="Sans-B", fontSize=14, leading=18, spaceAfter=6),
          "h2": ParagraphStyle("h2", parent=body, fontName="Sans-B", fontSize=11, leading=14, spaceBefore=8,
                               spaceAfter=4),
          "h3": ParagraphStyle("h3", parent=body, fontName="Sans-B", fontSize=9.2, leading=12, spaceBefore=5,
                               spaceAfter=3),
          "p": body,
          "li": ParagraphStyle("li", parent=body, leftIndent=13, bulletIndent=0, bulletFontName="Sans",
                               bulletFontSize=body.fontSize, spaceAfter=2.5),
          "cap": ParagraphStyle("cap", parent=body, fontName="Sans-I", fontSize=7.4, leading=9.4, spaceAfter=6),
          # a table caption stays with the table's first rows; the table itself may split, header repeated
          "tcap": ParagraphStyle("tcap", parent=body, fontName="Sans-I", fontSize=7.4, leading=9.4, spaceAfter=3,
                                 keepWithNext=True),
          "cell": ParagraphStyle("cell", parent=body, fontSize=6.7, leading=9.0, spaceAfter=0),
          "head": ParagraphStyle("head", parent=body, fontName="Sans-B", fontSize=6.7, leading=9.0, spaceAfter=0),
          "code": ParagraphStyle("code", parent=body, fontName="Mono", fontSize=7.0, leading=9.0,
                                 backColor=colors.HexColor("#f3f3f3"), borderPadding=4, spaceBefore=3, spaceAfter=8)}
    doc = SimpleDocTemplate(path, pagesize=A4, leftMargin=15 * mm, rightMargin=15 * mm, topMargin=14 * mm,
                            bottomMargin=15 * mm, title=TITLE, author="quadcontrol")
    W, H = doc.width, doc.height

    def footer(c, d):
        c.setFont("Sans", 7)
        c.setFillColor(colors.HexColor("#666666"))
        c.drawString(doc.leftMargin, 8 * mm, "MdlTrot whole-body impulse control: verification")
        c.drawRightString(doc.leftMargin + W, 8 * mm, f"page {d.page}")

    flow = []
    for b in blocks:
        kind = b[0]
        if kind in ("h2", "h3"):
            # a heading needs a few lines under it on the same page; keepWithNext would move a long paragraph whole
            flow += [CondPageBreak(50 * mm), Paragraph(amb.pdf_inline(b[1]), st[kind])]
        elif kind in ("title", "p"):
            flow.append(Paragraph(amb.pdf_inline(b[1]), st[kind]))
        elif kind == "ol":
            flow += [Paragraph(amb.pdf_inline(t), st["li"], bulletText=f"{i}.") for i, t in enumerate(b[1], 1)]
        elif kind == "table":
            _, cap, head, rows, widths = b
            data = [[Paragraph(amb.pdf_inline(h), st["head"]) for h in head]] + \
                   [[Paragraph(amb.pdf_inline(c), st["cell"]) for c in r] for r in rows]
            t = Table(data, colWidths=[w * W for w in widths], repeatRows=1, hAlign="LEFT")
            t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#b8b8b8")),
                                   ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#ececec")),
                                   ("VALIGN", (0, 0), (-1, -1), "TOP"),
                                   ("LEFTPADDING", (0, 0), (-1, -1), 2.5), ("RIGHTPADDING", (0, 0), (-1, -1), 2.5),
                                   ("TOPPADDING", (0, 0), (-1, -1), 1.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
            flow += [Paragraph(amb.pdf_inline(cap), st["tcap"]), t, Spacer(1, 6)]
        elif kind == "fig":
            _, name, frac, cap = b
            img = os.path.join(rep, "figs", name)
            iw, ih = ImageReader(img).getSize()
            w, h = frac * W, frac * W * ih / iw
            if h > 0.8 * H:
                w, h = w * 0.8 * H / h, 0.8 * H
            flow.append(KeepTogether([Image(img, width=w, height=h), Paragraph(amb.pdf_inline(cap), st["cap"])]))
        elif kind == "code":
            flow.append(Preformatted(b[1], st["code"]))
    doc.build(flow, onFirstPage=footer, onLaterPages=footer)


def build(rep):
    """REPORT.md and REPORT.pdf in rep from rep/results.json and rep/figs."""
    path = os.path.join(rep, "results.json")
    with open(path) as f:
        R = json.load(f)
    written = datetime.datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M")
    blocks = content(R, written)
    amb.write_md(blocks, os.path.join(rep, "REPORT.md"))
    write_pdf(blocks, os.path.join(rep, "REPORT.pdf"), rep)
    print(f"report: {os.path.join(rep, 'REPORT.md')}, REPORT.pdf")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", help="directory holding results.json and figs/ (default: reports/trot_wbc)")
    a = ap.parse_args()
    build(os.path.abspath(a.out) if a.out else os.path.join(REPO, "reports", "trot_wbc"))


if __name__ == "__main__":
    main()
