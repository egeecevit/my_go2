#!/usr/bin/env python3
#
# Copyright (C) 2005-2026 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""REPORT.md and REPORT.pdf on the thesis Ch. 6 work, from results.json and
figs/ in reports/amb_wbc (or --out DIR). No simulation runs:

  /home/ege/legged/iros/.venv/bin/python figs/amb_wbc/build_report.py [--out DIR]

run_experiments.py calls build() at the end of a run. Both files are rendered
from one block list (content()), so they say the same thing. Every measured
number, and every thesis number results.json stores, is read from
results.json here; numbers typed in the text come from the stage records
(source in the text) or from the thesis.
"""

import argparse
import datetime
import json
import os
import re

import matplotlib
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, KeepTogether, Paragraph, Preformatted, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TITLE = "Angular-momentum balancing as a whole-body task on the Go2 (thesis Chapter 6)"
CODE_COLS = 100      # mono 7.5 pt: ~110 columns fit the text width; longer lines would be clipped


def content(R, written):
    """The report as blocks: (kind, ...). Inline markup: <b>, <i>, <sub>,
    <sup> and `code`."""
    abl, tr, x3, ori, fo, pu, ws, meta = (R[k] for k in ("ablation", "transfer", "x3", "orientation", "force",
                                                        "push", "workspace", "meta"))
    on, off, jon, joff, tg = (abl["runs"][k] for k in ("wbc_on", "wbc_off", "joint_on", "joint_off",
                                                       "wbc_thesis_gains"))
    t64, prin, t66, th66 = abl["thesis_table_6_4"], tr["principal_two_foot"], tr["table_6_6"], tr["thesis_table_6_6"]
    p5, y3, thp = ori["pitch5"], ori["yaw3"], ori["thesis_peaks_ref_meas_deg"]
    steps = x3["steps"]
    frun = {int(F): r for F, r in fo["runs"].items()}
    f_ok = sorted(F for F, r in frun.items() if r["bounded"])
    f_bad = sorted(F for F, r in frun.items() if not r["bounded"])
    f_th = sorted(int(F) for F in fo["thesis"]["x2"])
    t65 = {int(F): v for F, v in pu["thesis_table_6_5"].items() if F != "columns"}
    th_ok, th_bad = max(F for F in t65 if t65[F][4]), min(F for F in t65 if not t65[F][4])
    rows = {p: {r["F"]: r for r in pu[p]["table_6_5"]} for p in ("wbc", "joint")}
    rows["minus"] = {r["F"]: r for r in pu["minus_t_h"]["wbc"]["table_6_5"]}
    bw, bj, bm, bmj = (pu["wbc"]["boundary_N"], pu["joint"]["boundary_N"], pu["minus_t_h"]["wbc"]["boundary_N"],
                       pu["minus_t_h"]["joint"]["boundary_N"])
    rec, fail = ws["recovering"], ws["failing"]
    ev, evr = fail["events_s_after_activation"], rec["events_s_after_activation"]
    lim = fail["xi_t_lim_m"]
    peak_abd = {int(F): v for F, v in ws["sweep_facts"]["recovering_peak_abduction_deg"].items()}
    past_stop = sorted(F for F, v in peak_abd.items() if v > ws["sweep_facts"]["abduction_stop_deg"])
    minus_nl = sorted(F for F, r in rows["minus"].items() if r["success"] and r["feedforward_norm_limited_samples"])
    errs_post = [fail[f"{g}_max_{c}_error_mm"]["post"] for g in ("FR", "RL") for c in ("normal", "radial")]
    ph = tr["phases"]
    fz = tr["final_sim_fz_N"]
    legs = ("FL", "FR", "RL", "RR")
    worst = max(t66[k][g]["rmse_des_sol"] for k in t66 for g in legs)
    worst_th = max(th66[k][g][2] for k in ("transfer", "final") for g in legs)
    worst_sim = max(t66[k][g]["rmse_des_sim"] for k in t66 for g in legs)
    th_dev = [abs(th66[k][g][1] - th66[k][g][0]) for k in ("transfer", "final") for g in legs]
    pmax = max(prin[g][f"{c}_max_mm"] for g in ("FR", "RL") for c in ("normal", "radial"))
    hand = meta["handoff_s"]
    t_b = float(meta["script"][1].split()[0])
    pd = meta["push_direction"]
    hz = 1000 / meta["log_period_ms"]
    # computeTick wall time from each run's closing line (the joint path prints none)
    wbc_runs = ([on, off, tg, tr, x3, p5, y3] + list(frun.values()) + list(rows["wbc"].values())
                + list(rows["minus"].values()))
    bal2 = [r["closing"]["tick_max_us"] for r in [on, x3, p5, y3] + [frun[F] for F in f_ok]
            + [r for p in ("wbc", "minus") for r in rows[p].values() if r["success"]]]
    falling = [r["closing"]["tick_max_us"] for r in wbc_runs if r["amb_error"]]
    means = [r["closing"]["tick_mean_us"] for r in wbc_runs]
    # how long before each fall the theta branch-cut jump came, where both exist
    lon, lth = abl["long"]["wbc_on"], abl["long"]["wbc_thesis_ori_gains"]
    falls = ([(r["time_to_fall"], r) for r in (tg, lth)] + [(r["fall"], r) for r in frun.values()]
             + [(r["fall_t"], r) for r in rows["wbc"].values()])
    lead = [f - r["theta_wrap_t"] for f, r in falls if f is not None and r["theta_wrap_t"] is not None]
    th_rec_L = [t65[F][2] for F in t65 if t65[F][4]]
    margins = [("ablation, AMB on, 12 s", on), ("the same, 60 s", lon), ("x3 schedule", x3), ("pitch", p5),
               ("yaw", y3), (f"{f_ok[-1]} N force", frun[f_ok[-1]]), (f"{bw[0] - 1} N push", rows["wbc"][bw[0] - 1]),
               ("ablation, joint space", jon)]
    angle = pd["support_line_angle_to_body_x_deg"]

    def fx(v, n=2):
        return "–" if v is None else f"{v:.{n}f}"

    def yn(b):
        return "yes" if b else "no"

    def mrg(r):      # abduction margin at the end of a window and its trend (+: moving away from the stop)
        return f"{fx(r['abduction_margin_end_deg'], 1)} deg ({r['abduction_margin_rate_deg_per_s']:+.2f} deg/s)"

    def words(s):    # results.json text: comparison signs would read as markup
        return s.replace(">=", "≥").replace("<=", "≤").replace("< ", "below ").replace("> ", "above ")

    def crit(g):     # the gate's own criterion text from results.json
        return words(g["gate"].split(": ", 1)[1])

    B = []
    B += [("title", TITLE),
          ("p", f"<i>Report on the whole-body realization of MdlAMB in quadcontrol and on the thesis Sec. 6.6 "
                f"experiments, simulation only. Results: `reports/amb_wbc/results.json`, written {written} by "
                f"`figs/amb_wbc/run_experiments.py` ({len(meta['run_wall_times_s'])} simulator runs, "
                f"{meta['wall_time_s']:.0f} s wall time). Thesis: Ozden (2026), Chapter 6.</i>")]

    # 1 ------------------------------------------------------------------------
    B += [("h2", "1. What this is and the outcome"),
          ("p", "MdlAMB balances the Go2 on its FL–RR diagonal by swinging the two free legs, FR and RL, under the "
                "reduced-order angular-momentum (AMB) law of the thesis. This work adds the thesis Chapter 6 "
                "realization of that law. The AMB output becomes a tangential acceleration of each swing foot in a "
                "hip-centred principal frame. A prioritized whole-body controller (WBC) realizes it together with "
                "body-orientation and body-position tasks, and a whole-body impulse-control QP (WBIC) resolves the "
                "floating-base dynamics against stance forces planned by the existing convex MPC. The earlier "
                "joint-space realization stays, selected by `[amb] wbc = false`. One script runs every Sec. 6.6 "
                "experiment headless in MuJoCo. The robot differs from the thesis model in ways that matter "
                "(rolling 22 mm sphere feet, real Go2 torque limits, a different entry posture, tuned AMB gains and a "
                "stiffer body-orientation gain), "
                "so the comparison is qualitative. The mechanism, the ablation, x<sub>3</sub> and orientation "
                "tracking and the load transfer reproduce the thesis' statements. The operating range does not: "
                f"a persistent force is held only up to {f_ok[-1]} N (thesis: {f_th[-1]} N), and the push boundary "
                f"along the direction inferred from the thesis' figures is {bw[0]}–{bw[1]} N (thesis: "
                f"{th_ok}–{th_bad} N; {bm[0]}–{bm[1]} N here along the opposite direction)."),
          ("p", "<b>Symbols.</b> L: whole-robot angular momentum about the FL–RR support line through the ground "
                "contact points [kg m²/s]. t<sub>h</sub>: horizontal unit normal of that line (here pointing to the "
                "robot's left). c<sub>x</sub>: horizontal offset of the centre of mass (CoM) from the line along "
                "t<sub>h</sub>. x<sub>1</sub> = θ<sub>1</sub>: template tip angle about the line; x<sub>2</sub> = "
                "θ<sub>2</sub> + θ<sub>3</sub> and x<sub>3</sub> = θ<sub>2</sub> − θ<sub>3</sub>: symmetric swing "
                "and separation of the two swing legs (θ<sub>2</sub>, θ<sub>3</sub> their template angles). "
                "ξ<sub>n</sub>, ρ, ξ<sub>t</sub>: normal, radial and tangential coordinates of a swing foot "
                "relative to its hip in the principal frame. Times are from MdlAMB activation, the hand-off from "
                "MdlDiagonalStance."),
          ("table", "Table 1. Sec. 6.6 experiments: gate, result, headline numbers here and in the thesis.",
           ["Experiment (thesis)", "Gate", "Result", "Here", "Thesis"],
           [["Principal-frame tracking (6.6.2)", crit(tr["gate_principal"]), tr["gate_principal"]["result"],
             f"RMSE normal {fx(prin['both_rmse_mm']['normal'])} mm, radial {fx(prin['both_rmse_mm']['radial'])} "
             f"mm (both legs); largest error {fx(pmax, 1)} mm",
             f"RMSE normal {prin['thesis_rmse_mm']['normal']} mm, radial {prin['thesis_rmse_mm']['radial']} mm"],
            ["x<sub>3</sub> tracking (6.6.3)", crit(x3["gate"]), x3["gate"]["result"],
             f"steady error at most {fx(max(abs(s['steady_error_deg']) for s in steps[1:]))} deg on the last "
             f"{len(steps) - 1} steps, {fx(steps[0]['steady_error_deg'])} deg on the "
             f"{steps[0]['to_s'] - steps[0]['from_s']:.1f} s first step; no fall in {steps[-1]['to_s']:.0f} s",
             "follows each step with a small offset (Fig. 6.4)"],
            ["AMB ablation (6.6.4)", crit(abl["gate"]), abl["gate"]["result"],
             f"on: no fall in {on['observed']:.0f} s; off: falls at {off['time_to_fall']} s",
             f"on: no fall in {t64['enabled']['observed']} s; off: falls at {t64['disabled']['time_to_fall']} s"],
            ["Orientation (6.6.5.1)", crit(ori["gate"]), ori["gate"]["result"],
             f"amplitude ratio {fx(p5['amplitude_ratio'], 3)} (pitch) / {fx(y3['amplitude_ratio'], 3)} (yaw), "
             f"lag {fx(p5['lag_s'], 3)} / {fx(y3['lag_s'], 3)} s",
             f"ratio about {thp['pitch'][1] / thp['pitch'][0]:.2f} / {thp['yaw'][1] / thp['yaw'][0]:.2f} "
             "(peaks of Fig. 6.11)"],
            ["Persistent force (6.6.5.2)", crit(fo["gate"]),
             fo["gate"]["result"] + (" via the 'boundary reported' clause; thesis result not reproduced"
                                     if f_bad else ""),
             f"bounded at {', '.join(map(str, f_ok))} N; falls at {', '.join(map(str, f_bad))} N",
             f"bounded at {', '.join(map(str, f_th))} N"],
            ["Push recovery (6.6.5.3)", crit(pu["gate"]), pu["gate"]["result"],
             f"+t<sub>h</sub>: WBC recovers up to {bw[0]} N, falls from {bw[1]} N; joint space up to {bj[0]} N, "
             f"from {bj[1]} N. −t<sub>h</sub>: WBC up to {bm[0]} N, from {bm[1]} N",
             f"recovers up to {th_ok} N, falls from {th_bad} N (one direction)"],
            ["Workspace limit (6.6.6)", crit(ws["gate"]), ws["gate"]["result"],
             f"{rec['F']} N recovers, {fail['F']} N fails by exhausted tangential travel, normal/radial errors "
             f"{min(errs_post):.0f}–{max(errs_post):.0f} mm; recovering pushes from {past_stop[0]} N also pass "
             "the abduction stop",
             "recovering case inside ±ξ<sub>t,lim</sub>, failing case crosses it with normal/radial collapse"],
            ["Load transfer (6.6.7)", crit(tr["gate"]), tr["gate"]["result"],
             f"four-foot support, no fall; worst per-foot desired−solved RMSE {fx(worst)} N",
             f"worst per-foot RMSE {worst_th} N"]],
           [0.15, 0.22, 0.11, 0.30, 0.22]),
          ("p", "<b>Code regression checks</b> (run by hand, not stored in results.json): ctest passes 20 of 20 "
                "tests, the pre-existing ones unchanged. With `amb.wbc = false` the headless 10 s balance log is "
                "identical, sample for sample (4674 samples), to the one recorded before any of this work: the "
                "joint-space path is untouched.")]

    # 2 ------------------------------------------------------------------------
    B += [("h2", "2. What was built"),
          ("p", "Thesis (6.10) runs once per 1 ms tick in `MdlAMB::computeTick`: (q, dq/dt, contact pair) → support "
                "line and template plane → reduced state (x, dx/dt, L, dL/dt) → template accelerations "
                "d²θ/dt² → swing-foot feedforward a<sub>f</sub> → swing-foot task command → WBIC (generalized "
                "acceleration, contact forces F<sub>c</sub>, torques τ). The first three arrows are MdlAMB code "
                "shared with the joint-space path: `project()` builds the support line from the FL and RR ground "
                "contact points, the template angles and the CoM offset; `robotMomentum()` gives L, with dL/dt = "
                "M g c<sub>x</sub> about the line and an external-torque observer; `ambLaw()` is the Chapter 4 law. "
                "From d²θ<sub>2</sub>/dt², d²θ<sub>3</sub>/dt² the two realizations part. In WBC mode "
                "`MdlAMB::_wbcTick` builds, per swing leg, the principal frame (6.13)–(6.16), the feedforward "
                "a<sub>f</sub> = ρ (d²θ/dt²) e<sub>t</sub> with its norm limit (6.18), the tangential pass-through "
                "ξ<sub>t</sub><sup>d</sup> = ξ<sub>t</sub> (6.23) and the desired foot position and velocity "
                "(6.20)–(6.25). It hands four tasks, in priority order body orientation, body position, FR foot, "
                "RL foot, to `QuadrupedWBC`. The stance-force objective F<sub>c</sub><sup>d</sup> comes from "
                "`MdlConvexMPC`, re-solved every 10th tick on the latched pose and clamped to 0.10 tangential over "
                "normal. `QuadrupedWBC` computes the task hierarchy (6.4)–(6.6), the WBIC QP (6.7)–(6.8) in qpOASES "
                "and the torque (6.9) on `QuadrupedDynamics`, an 18-DoF floating-base model whose contact Jacobian "
                "is that of a rolling sphere. Joint commands follow (6.28): stance legs track the WBC joint "
                "references under a 30/3 joint PD plus the WBIC torque; swing joints get the WBIC torque alone, "
                "plus cancellation of the simulator's joint damping and dry friction."),
          ("p", "<b>Phases.</b> The Supervisor goes S_STAND → (key B) S_DIAG, where MdlDiagonalStance moves the load "
                "onto FL and RR, → S_AMB. MdlAMB lifts FR and RL over 0.4 s (quintic: normal and radial references "
                "to a lift target, x<sub>3</sub> reference to −304 deg) and balances. Key D requests a stop (WBC "
                "mode only, state S_AMB_STOPPING): the swing feet are placed over 2.5 s with AMB still acting; each "
                "foot becomes a WBC contact once its kinematic contact point is within 5 mm of the ground and no "
                "longer sinking faster than 1 cm/s; the desired normal forces then blend linearly over 3 s from the "
                "frozen two-foot plan to the four-foot MPC plan; after a 3 s four-foot hold MdlAMB reports SETTLED "
                "and the Supervisor sits. In joint-space mode D sits at once, as before."),
          ("table", "Table 2. Thesis items and where they live.", ["Thesis", "Implementation", "Where"],
           [["(6.1)–(6.2) floating-base dynamics, rigid contacts",
             "M, h, foot, contact-point and hip Jacobians with their bias terms, CoM",
             "`QuadrupedDynamics.hh/.cc`"],
            ["(6.3)–(6.9) task feedback, hierarchy, WBIC QP, torque",
             "dynamically consistent hierarchy and kinematic stage; QP over base-acceleration and force "
             "corrections with friction pyramid and normal-force bounds", "`QuadrupedWBC.hh/.cc`"],
            ["(6.10)–(6.12) signal path, support line, template plane",
             "`computeTick`: project, L, ambLaw, d²θ/dt², then `_wbcTick`", "`MdlAMB.cc`"],
            ["(6.13)–(6.19) principal frame, arc coordinate, feedforward and norm limit",
             "static helpers `principalFrame`, `tangentialArc`, `tangentialFeedforward`", "`MdlAMB.hh/.cc`"],
            ["(6.20)–(6.27) desired axes, pass-through, swing task",
             "`swingReference`; task assembly in `_wbcTick`", "`MdlAMB.cc`"],
            ["(6.28) joint commands", "per-leg MIT command, friction cancellation on the swing joints",
             "`MdlAMB.cc`"],
            ["F<sub>c</sub><sup>d</sup>, Sec. 6.4.3 force ratio", "MPC grabbed as MdlTrot does; tangential clamp",
             "`MdlConvexMPC` (unchanged), `MdlAMB.cc`"],
            ["Sec. 6.5 feasibility diagnostics", "logs `MdlAMB_wbc` (layout at `_logWbc`), `MdlAMB_tau`, closing "
             "messages (saturation, tick time)", "`MdlAMB.hh/.cc`"],
            ["Sec. 6.6 inputs", "`x3_schedule`, `pitch_amp_deg`, `yaw_amp_deg`, `place_duration`; `push_ramp`",
             "`amb.toml`, `simulation.toml`, `MdlSimDriver.cc`"],
            ["Sec. 6.6.7 stop sequence", "`stopBalancing()`, contact promotion, transfer blend; S_AMB_STOPPING",
             "`MdlAMB.cc`, `Supervisor.cc`"],
            ["Tests", "model against MuJoCo; WBC invariants; full-rig balance, ablation, x<sub>3</sub> step, "
             "orientation, stop and transfer", "`test_quadruped_dynamics`, `test_wbc`, `test_amb_wbc_sim`"],
            ["Sec. 6.6 experiments", "single entry point: runs, metrics, figures, this report",
             "`figs/amb_wbc/run_experiments.py`"]],
           [0.30, 0.42, 0.28])]

    # 3 ------------------------------------------------------------------------
    B += [("h2", "3. Results"),
          ("p", f"<b>Common setup.</b> Each run is one headless, non-real-time `bin/sim.sh` call with the key script "
                f"S at 0 s and B at {t_b} s; MdlDiagonalStance hands over to MdlAMB at {hand} s of simulation time "
                f"in every run. Disturbances start 2 s after the hand-off. Logs are at {hz:.0f} Hz. Torso state, "
                f"CoM, L and c<sub>x</sub> are simulator truth (logged qpos and qvel through the MuJoCo model); "
                f"reduced and principal-frame quantities are the controller's own log. Fall criteria, as Sec. "
                f"6.6.1: ablation {words(meta['fall_criteria']['ablation'])}; all other trials "
                f"{words(meta['fall_criteria']['others'])}. Pushes and persistent forces act at the base body's CoM "
                f"along +t<sub>h</sub> = world (x, y) = ({pd['push_direction_xy'][0]}, {pd['push_direction_xy'][1]}), "
                f"the sign for which thesis Fig. 6.18 shows L and dL/dt positive during the pulse. The support line "
                f"runs {angle:.2f} deg from body x."),

          ("h3", "3.1 Principal-frame tracking (Sec. 6.6.2, Fig. 6.3)"),
          ("p", "Applied: the stop sequence of Sec. 3.8 (balance 5 s, then D). During balance the normal and radial "
                "coordinates of both swing feet hold their references, and the tangential reference equals the "
                "measurement by construction (6.23), so the tangential panel shows the motion AMB commands. At the "
                "stop the normal reference returns to its hand-off value and the radial one rises to the ground. "
                f"From the end of the lift to each foot's promotion the RMSE is {fx(prin['FR']['normal_rmse_mm'])} / "
                f"{fx(prin['RL']['normal_rmse_mm'])} mm normal and {fx(prin['FR']['radial_rmse_mm'])} / "
                f"{fx(prin['RL']['radial_rmse_mm'])} mm radial (FR / RL), largest error {fx(pmax, 1)} mm; with the "
                f"0.4 s lift included, {fx(prin['FR']['with_lift']['normal_rmse_mm'])} / "
                f"{fx(prin['RL']['with_lift']['normal_rmse_mm'])} mm and "
                f"{fx(prin['FR']['with_lift']['radial_rmse_mm'])} / {fx(prin['RL']['with_lift']['radial_rmse_mm'])}"
                f" mm. Thesis: {prin['thesis_rmse_mm']['normal']} mm normal, {prin['thesis_rmse_mm']['radial']} mm "
                "radial. Why smaller: the swing task here feeds the lift and placement ramps' reference rate and "
                "acceleration ((6.3)), which the thesis' held references do not need; before the acceleration term "
                "was added, RL's normal error peaked at 56.8 mm just after the lift."),
          ("fig", "fig6_03_principal_frame.png", 0.62,
           "Fig. 6.3 counterpart: principal-frame coordinates of FR (blue) and RL (orange) and x<sub>3</sub> "
           "through balance, placement, promotion of both feet ('balance stop'), load transfer and final hold. "
           "Dashed: references, hidden once a foot is a contact."),

          ("h3", "3.2 x<sub>3</sub> tracking (Sec. 6.6.3, Fig. 6.4–6.5)"),
          ("p", "Applied: the full thesis schedule as raw steps, " +
                ", ".join(f"{s['x3d_deg']:.0f} deg at {s['from_s']} s" for s in steps) +
                ". The lift itself ramps x<sub>3</sub> to −304 deg, so the thesis' first value is given at the end of "
                "the lift. x<sub>3</sub> follows every step; the steady error (mean over each step's last 1 s) is " +
                ", ".join(f"{s['steady_error_deg']:+.2f}" for s in steps) +
                f" deg. The first step lasts only {steps[0]['to_s'] - steps[0]['from_s']:.1f} s and is still "
                f"settling. No fall in {steps[-1]['to_s']:.0f} s; the CoM stays within "
                f"{fx(x3['max_cx_mm'])} mm of the support line, the offset it starts with at the hand-off. The "
                f"torso stays near its WBC references: largest deviation {fx(x3['torso_max_dev']['height_mm'])} mm "
                f"in height, {fx(x3['torso_max_dev']['roll_deg'])} deg roll, "
                f"{fx(x3['torso_max_dev']['pitch_deg'])} deg pitch, {fx(x3['torso_max_dev']['yaw_deg'])} deg yaw. "
                f"The roll deviation peaks at {x3['torso_max_dev']['roll_max_at_s']:.1f} s; the swing abduction ends "
                f"{mrg(x3)} from its stop. Thesis: an offset of about 1–2 deg and sub-degree, millimetre torso "
                "deviations. The offset is smaller here; the template masses and gains differ, and the cause was "
                "not investigated."),
          ("fig", "fig6_04_x3_tracking.png", 0.78,
           "Fig. 6.4 counterpart: top-down CoM path (colour: time) against the hand-off support line, and "
           "x<sub>3</sub> against the scheduled reference. Torso deviations: `figs/fig6_05_x3_torso.png`."),

          ("h3", "3.3 AMB ablation (Sec. 6.6.4, Table 6.4, Fig. 6.7–6.8)"),
          ("p", "Applied: the same run with `amb.enabled = false`, which keeps the WBC, its hierarchy and the lift and "
                "removes the AMB feedforward and the tangential pass-through. AMB on stays up for the whole "
                f"{on['observed']:.0f} s window; AMB off crosses 0.16 m at {off['time_to_fall']} s (thesis "
                f"{t64['disabled']['time_to_fall']} s). The joint-space pair behaves the same way (on balances, off "
                f"falls at {joff['time_to_fall']} s). The AMB-on peaks of |L|, |c<sub>x</sub>| and angular speed are "
                "the values at the hand-off instant, the posture MdlDiagonalStance leaves; after the lift they are "
                "an order of magnitude smaller (0.0007 kg m²/s and 0.08 mm, from the wbc_on log). The thesis' peaks "
                "include its trot-to-balance transition, so the AMB-on rows are not comparable in size. The Table "
                "6.2 AMB gains and template masses "
                + ("also balance" if abl["thesis_gains_balance"] else f"fall at {tg['time_to_fall']} s")
                + f" over the {on['observed']:.0f} s window, with larger excursions (peak |L| {tg['peak_L']:.4f} "
                f"against {on['peak_L']:.4f} kg m²/s; a one-off 60 s run held, max |θ<sub>1</sub>| 0.66 deg against "
                f"{lon['closing']['max_theta1_deg']:.2f} deg). The tuned values stay: the joint-space path shares "
                "them."),
          ("p", "<b>Is the balance a settled state?</b> The law feeds back L, its derivatives and x<sub>3</sub>; "
                "nothing in it restores x<sub>2</sub>, the symmetric swing. The measured lever is the trunk's "
                "orientation stiffness (probably: with the roll about the support line held, the CoM offset depends "
                "on x<sub>2</sub>, and regulating L then fixes it). With Table 6.2's body-orientation gains "
                "(100, 10) the roll yields and "
                f"x<sub>2</sub> runs away slowly: {lth['x2_deg_at_5s']:.0f}, {lth['x2_deg_at_20s']:.0f}, "
                f"{lth['x2_deg_at_35s']:.0f} deg at 5, 20, 35 s, then the swing legs reach their stops and the robot "
                f"falls at {lth['time_to_fall']:.1f} s. With the default (400, 40) x<sub>2</sub> settles: "
                f"{lon['x2_deg_at_5s']:.1f}, {lon['x2_deg_at_20s']:.1f}, {lon['x2_deg_at_35s']:.1f} deg at the same "
                f"times, no fall in {lon['observed']:.0f} s, max |θ<sub>1</sub>| "
                f"{lon['closing']['max_theta1_deg']:.2f} deg, abduction margin at the end {mrg(lon)}; a one-off run "
                "held 120 s. From kp 200 up it settles; kd alone changes nothing. This is why the default departs "
                "from Table 6.2 (departure 6). At 12 s the default is still settling (x<sub>2</sub> rate "
                f"{on['x2_drift_last_2s_deg_per_s']:+.2f} deg/s); the joint-space path does not move "
                f"({jon['x2_drift_last_2s_deg_per_s']:+.2f} deg/s)."),
          ("table", "Table 3. Table 6.4 metrics, AMB off and on, here (whole-body path) and in the thesis.",
           ["Metric", "Off, here", "Off, thesis", "On, here", "On, thesis"],
           [[name, *(conv(r.get(key)) for r in (off, t64["disabled"], on, t64["enabled"]))]
            for name, key, conv in (
                ("Fall detected", "fall", yn),
                ("Time to fall [s]", "time_to_fall", lambda v: fx(v, 3)),
                ("Minimum torso height [m]", "min_z", lambda v: fx(v, 4)),
                ("Peak |roll| [deg]", "peak_roll", lambda v: fx(v, 2)),
                ("Peak |pitch| [deg]", "peak_pitch", lambda v: fx(v, 2)),
                ("Peak angular speed [rad/s]", "peak_w", lambda v: fx(v, 3)),
                ("Peak |L| [kg m²/s]", "peak_L", lambda v: fx(v, 4)),
                ("Peak |c<sub>x</sub>| [mm]", "peak_cx", lambda v: fx(None if v is None else 1e3 * v, 2)),
                ("Observed duration [s]", "observed", lambda v: fx(v, 3)))],
           [0.32, 0.17, 0.17, 0.17, 0.17]),
          ("fig", "fig6_07_ablation_height.png", 0.78,
           "Fig. 6.7 counterpart: torso height, whole-body pair (left) and the joint-space pair and Table 6.2 AMB "
           "gains (right); dotted: 0.16 m fall threshold. Top-down CoM: `figs/fig6_08_ablation_com.png`."),

          ("h3", "3.4 Orientation commands (Sec. 6.6.5.1, Fig. 6.11–6.12)"),
          ("p", f"Applied: {p5['amplitude_ref_deg']:.0f} deg pitch and {y3['amplitude_ref_deg']:.0f} deg yaw "
                "sinusoids at 0.1 Hz in separate runs, after a 2 s delay and a 2 s envelope, on the orientation "
                "task and on the MPC reference. The measured angle follows with amplitude ratio "
                f"{fx(p5['amplitude_ratio'], 3)} and {fx(y3['amplitude_ratio'], 3)}, lag {fx(p5['lag_s'], 3)} s "
                f"and {fx(y3['lag_s'], 3)} s, tracking RMS {fx(p5['tracking_rms_deg'])} and "
                f"{fx(y3['tracking_rms_deg'])} deg. During the sinusoids max |L| is "
                f"{fx(p5['max_L_sinusoid'], 4)} and {fx(y3['max_L_sinusoid'], 4)} kg m²/s; no fall. Thesis: peaks "
                f"{thp['pitch'][0]} → {thp['pitch'][1]} deg (pitch) and {thp['yaw'][0]} → {thp['yaw'][1]} deg "
                "(yaw), L within ±0.02 kg m²/s. Why better here: the MPC reference carries the sinusoid, and the "
                "orientation gain is four times the thesis' (with (100, 10) the ratios were 0.991 and 0.994, and "
                "0.881 for pitch without the MPC reference). Abduction margin at the end "
                f"{mrg(p5)} and {mrg(y3)}; x<sub>2</sub> follows the sinusoid."),
          ("fig", "fig6_11_orientation.png", 0.60,
           "Fig. 6.11 counterpart: pitch and yaw against their references, L and dL/dt (the early spike is the "
           "lift). Top-down CoM: `figs/fig6_12_orientation_com.png`."),

          ("h3", "3.5 Persistent force (Sec. 6.6.5.2, Fig. 6.14–6.15)"),
          ("p", f"Applied: {', '.join(map(str, f_th))} N along +t<sub>h</sub>, a 2 s linear ramp from 2 s, held to "
                "12 s; then 1 N steps below the first failure. Only "
                f"{', '.join(map(str, f_ok))} N stays bounded; " +
                ", ".join(f"{F}" for F in f_bad) + " N fall at " +
                ", ".join(f"{frun[F]['fall']:.2f}" for F in f_bad) +
                f" s. Thesis: all four bounded. At {f_ok[-1]} N the CoM settles at c<sub>x</sub> = "
                f"{frun[f_ok[-1]]['steady_cx_mm']:.3f} mm, the static requirement −F h/(M g) = "
                f"{frun[f_ok[-1]]['required_cx_mm']:.3f} mm (h: height of the base CoM, where the force acts). The "
                "WBC holds the trunk pose, so this offset has to come from the swing legs, and they supply it by "
                f"abduction: x<sub>2</sub> is at {frun[f_ok[-1]]['steady_x_deg'][1]:.0f} deg over the last second, "
                f"still moving at {frun[f_ok[-1]]['x2_drift_deg_per_s']:+.2f} deg/s, and the abduction margin at "
                f"12 s is {mrg(frun[f_ok[-1]])}. In every failing run the swing "
                "abductions run into their ±60 deg stops (smallest joint-limit distance "
                f"{min(frun[F]['min_joint_limit_distance_rad'] for F in f_bad):.3f} rad: past the soft limit), then "
                "the trunk tips. "
                f"Why: with the stance feet on the body centreline the support line is {angle:.1f} deg from body x, "
                "so the swing feet's tangential channel is almost pure hip abduction, which moves the CoM little. "
                "With the trunk fixed, both abductions at −60 deg move c<sub>x</sub> by −4.30 mm (FR −0.19 mm, "
                "RL −4.11 mm) and at +60 deg by +6.85 mm (review check on an earlier run's posture), while " +
                ", ".join(f"{F} N needs {abs(frun[F]['required_cx_mm']):.2f} mm" for F in f_bad[:1] + f_bad[-1:]) +
                ". For feet under the hips the thesis' diagonal would be about 36 deg from body x, which brings hip "
                "flexion in (a hypothesis, not re-derived). The stiffer orientation gain does not change this "
                "boundary. Along −t<sub>h</sub>, 2 N is bounded and 3 N fails (one-off runs)."),
          ("fig", "fig6_14_force_states.png", 0.60,
           "Fig. 6.14 counterpart: x<sub>1</sub>, x<sub>2</sub>, x<sub>3</sub> under the ramped forces (grey: "
           "ramp); failing runs end at MdlAMB's error, angles unwrapped across the θ branch cut (Sec. 5). Top-down "
           "CoM: `figs/fig6_15_force_com.png`."),

          ("h3", "3.6 Push recovery (Sec. 6.6.5.3, Table 6.5, Fig. 6.16–6.18)"),
          ("p", "Applied: 0.1 s pulses along +t<sub>h</sub> at 2 s, from 1 N in 1 N steps to two past the first "
                "failure, on the whole-body and on the joint-space path; success is judged after the pulse; the "
                "same sweep along −t<sub>h</sub> is a side result. The WBC recovers up to "
                f"{bw[0]} N and falls from {bw[1]} N; the joint-space path recovers up to {bj[0]} N and falls from "
                f"{bj[1]} N. Along −t<sub>h</sub>: WBC {bm[0]} / {bm[1]} N, the thesis' boundary, joint space "
                f"{bmj[0]} / {bmj[1]} N. No push above a first failure recovered. In the recovering cases near the "
                f"boundary, max |L| is {rows['wbc'][bw[0] - 2]['max_L']:.2f}–{rows['wbc'][bw[0]]['max_L']:.2f} "
                f"kg m²/s (thesis {min(th_rec_L)}–{max(th_rec_L)}) and the mean dL/dt during the pulse 1.8–2.4 N m "
                "(thesis about 3 N m). Why lower along +t<sub>h</sub>: probably the asymmetry of the abduction-only "
                "tangential channel (+6.85 / −4.30 mm above); +t<sub>h</sub> calls on the weaker side. The boundary "
                "is sensitive to the posture at the pulse: a 0.5 deg difference in abduction moved it by 1 N in an "
                "earlier build, and with the hand-off 0.5 s earlier (the autorun) 10 N recovers. Read it as ±1 N."),
          ("table", "Table 4. Table 6.5 quantities after the pulse (WBC, +t<sub>h</sub>) with the thesis rows, and "
                    "success on the other sweeps; –: not sampled. Plots: `figs/table6_05_push_sweep.png` (both "
                    "paths), `fig6_16_push_com_recovering.png`, `fig6_17_push_com_failing.png` (CoM paths), "
                    "`fig6_18_push_dynamics.png` (v<sub>⊥</sub>, L, dL/dt of the recovering cases).",
           ["F [N]", "max |c<sub>x</sub>| [mm]", "max |L| [kg m²/s]", "z<sub>min</sub> [m]", "Success",
            "Success −t<sub>h</sub>", "Success joint", "Thesis max |c<sub>x</sub>| [mm]", "Thesis max |L|",
            "Thesis z<sub>min</sub> [m]", "Thesis success"],
           [[str(F)] + ([fx(1e3 * rows["wbc"][F]["max_cx"], 2), fx(rows["wbc"][F]["max_L"], 3),
                         fx(rows["wbc"][F]["z_min"], 4), yn(rows["wbc"][F]["success"])]
                        if F in rows["wbc"] else ["–"] * 4)
            + [yn(rows[p][F]["success"]) if F in rows[p] else "–" for p in ("minus", "joint")]
            + ([fx(1e3 * t65[F][1], 2), fx(t65[F][2], 3), fx(t65[F][3], 4), yn(t65[F][4])]
               if F in t65 else ["–"] * 4)
            for F in range(bj[0], max(max(rows["minus"]), max(t65)) + 1)],
           [0.06, 0.10, 0.10, 0.09, 0.08, 0.09, 0.08, 0.11, 0.10, 0.10, 0.09]),

          ("h3", "3.7 Workspace limit (Sec. 6.6.6, Fig. 6.19–6.20)"),
          ("p", f"Applied: the recovering push 2 N below the boundary ({rec['F']} N) and the failing push 2 N above "
                f"it ({fail['F']} N), as the thesis pairs 9 and 14 N around its 11–12 N. The tangential bounds "
                "±ξ<sub>t,lim</sub> come from MuJoCo forward kinematics of the logged posture with only that leg's "
                f"abduction moved to its limits: FR [{lim['FR'][0]:.3f}, {lim['FR'][1]:.3f}] m, RL "
                f"[{lim['RL'][0]:.3f}, {lim['RL'][1]:.3f}] m before the pulse (the thesis draws about ±0.5 m). The "
                f"{fail['F']} N failure follows the thesis' mechanism, now with the logs the thesis lacked (its "
                "Sec. 6.7, limitation 4). In order: AMB feedforward norm-limited at "
                f"{ev['feedforward_norm_limited']} s (during the pulse), abduction within 0.01 rad of its stop at "
                f"{ev['joint_limit_within_0.01rad']} s, ξ<sub>t</sub> past its bound at "
                f"{ev['RL_xi_t_beyond_bound']} s (RL) and {ev['FR_xi_t_beyond_bound']} s (FR), torque saturation "
                f"at {ev['torque_saturated']} s, normal and radial errors growing to {min(errs_post):.0f}–"
                f"{max(errs_post):.0f} mm, fall at {ev['fall']} s, then MdlAMB's θ<sub>1</sub> guard at "
                f"{ev['amb_error']['t']} s. The QP never failed. Unlike the thesis' recovering case, the recovering "
                f"pushes here also use up the tangential travel: every WBC push from {past_stop[0]} to "
                f"{past_stop[-1]} N passes the ±60 deg stop (peak "
                f"{peak_abd[past_stop[0]]:.1f}–{peak_abd[past_stop[-1]]:.1f} deg; the simulator's joint limit is "
                f"soft) and still recovers, and {rec['F']} N passes RL's ξ<sub>t</sub> bound at "
                f"{evr['RL_xi_t_beyond_bound']} s. Along +t<sub>h</sub> the 50 m/s² feedforward norm limit first "
                f"acts at {ws['sweep_facts']['first_force_norm_limited_N']} N, the first failing push; but along "
                f"−t<sub>h</sub> pushes of {minus_nl[0]}–{minus_nl[-1]} N recover with it active, and raising it to "
                "100 m/s² does not save 10 or 11 N (one-off diagnostic). The limit marks where failures start; it "
                "does not set the boundary."
                + (f" At {ev['theta_wrap']} s θ<sub>2</sub> crosses its branch cut (a swing leg passing straight "
                   "up), after the divergence; the plots unwrap it." if ev["theta_wrap"] is not None else "")),
          ("fig", "fig6_19_workspace_tangential.png", 0.78,
           f"Fig. 6.19 counterpart: ξ<sub>t</sub> of FR and RL for the {rec['F']} N and {fail['F']} N pushes; "
           "dashed ±ξ<sub>t,lim</sub> before the pulse, dotted the instantaneous bound. Normal and radial channels: "
           "`figs/fig6_20_workspace_normal_radial.png`; abduction, feedforward status, QP status and torque: "
           "`figs/fig6_19b_workspace_diagnostics.png`."),

          ("h3", "3.8 Load transfer (Sec. 6.6.7, Table 6.6, Fig. 6.21–6.22)"),
          ("p", f"Applied: balance for 5 s, then D. Placement starts at {ph['placement']} s, both feet are promoted "
                f"('balance stop') at {ph['transfer']} s, the final hold starts at {ph['final']} s and SETTLED "
                f"comes at {ph['settled']} s; the Supervisor then sits. The robot ends in four-foot support without "
                f"falling; final simulated normal forces FL / FR / RL / RR "
                f"{' / '.join(f'{fz[g]:.1f}' for g in legs)} N, sum {sum(fz.values()):.1f} N, weight "
                f"{tr['weight_N']:.1f} N. The worst per-foot desired−solved RMSE is {fx(worst)} N (thesis "
                f"{worst_th} N), the worst desired−simulated RMSE {fx(worst_sim)} N. Final body-position RMSE: "
                f"{fx(tr['final_body_rmse_mm'])} mm in the controller's frame and "
                f"{fx(tr['final_body_rmse_truth_mm'])} mm in simulator truth (thesis 7.13 mm, under its "
                "simulated-state override). Why the force errors are smaller: the thesis' desired forces are a "
                "pre-set ramp to equal shares, which its mean solved forces leave by "
                f"{min(th_dev):.1f}–{max(th_dev):.1f} N; here the desired forces are the MPC's plan for the actual "
                "CoM and footholds, which the WBIC changes little. That is also why the final split here is "
                "unequal."),
          ("table", "Table 5. Table 6.6 normal forces [N]: means and RMSE here (desired, solved, simulated) and in "
                    "the thesis (desired, solved).",
           ["Phase", "Foot", "Desired", "Solved", "Simulated", "RMSE des−sol", "RMSE des−sim", "Thesis desired",
            "Thesis solved", "Thesis RMSE"],
           [[ph_, g, fx(t66[k][g]["mean_desired"]), fx(t66[k][g]["mean_solved"]), fx(t66[k][g]["mean_sim"]),
             fx(t66[k][g]["rmse_des_sol"]), fx(t66[k][g]["rmse_des_sim"]), *(fx(v) for v in th66[k][g])]
            for k, ph_ in (("transfer", "transfer"), ("final", "final hold")) for g in legs],
           [0.12, 0.07, 0.10, 0.10, 0.10, 0.10, 0.10, 0.11, 0.10, 0.10]),
          ("fig", "fig6_21_normal_forces.png", 0.72,
           "Fig. 6.21 counterpart: desired (dashed), solved (solid) and simulated (dotted) normal forces; grey: "
           "3 s transfer, blue: final hold. Body position and roll: `figs/fig6_22_body.png`.")]

    # 4 ------------------------------------------------------------------------
    B += [("h2", "4. Departures from the thesis"),
          ("ol", [
              "<b>Feet.</b> The Go2 model's 22 mm sphere feet roll; contacts, the support line and L are taken at "
              "the ground contact points. The thesis treats point feet. Numbers are compared qualitatively.",
              f"<b>Entry.</b> MdlDiagonalStance puts FL and RR on the body centreline (support line {angle:.2f} deg "
              "from body x) and MdlAMB lifts FR and RL over 0.4 s; the thesis hands over from a trot. Disturbances "
              "start 2 s after the hand-off (the thesis' 5.5 s warm-up less its ≈3.5 s balance start).",
              "<b>State.</b> Base position and velocity come from the planted stance feet, attitude from "
              "MdlOrientationEstimator with its simulated sensor noise (`attitude_source = \"estimator\"`); the "
              "thesis uses the simulated-state override.",
              "<b>Period and torque.</b> 1 ms control period (thesis 2 ms); the real Go2 torque limits 23.7 / 23.7 "
              "/ 45.43 N m (thesis 65 N m).",
              "<b>AMB gains and template masses.</b> Tuned (950, 360, 41) and momentum-matched masses (14.954, "
              "0.2296, 0.0594 kg) instead of Table 6.2's (−3430, −1470, −210) and (10.5, 2.26, 2.26 kg). The "
              "thesis values were run and balance too, with larger excursions (Sec. 3.3); the tuned ones stay "
              "because the joint-space path shares them. The code's law takes the gains as magnitudes.",
              "<b>Body-orientation gains.</b> (400, 40) instead of Table 6.2's (100, 10): with the thesis values "
              f"the balance diverges slowly and falls {lth['time_to_fall']:.1f} s after the hand-off; with the "
              "default it settles (Sec. 3.3). The other WBC gains are Table 6.2's. Every experiment here runs "
              "with (400, 40).",
              "<b>Not implemented.</b> Pair switching (FL–RR only); the stance-foot horizontal anti-slip action of "
              "Sec. 6.4.3 (stance legs follow the WBC joint references under the 30/3 joint PD instead); the "
              "Chapter 5 vertical-acceleration bound (Table 6.2: 'when enabled').",
              "<b>Swing joints.</b> The simulator model's joint damping and dry friction (0.2 N m s/rad, 0.2 N m) "
              "are cancelled in the swing commands, and the swing kd is 0.001 instead of 0 because the simulator "
              "replaces kd ≤ 0 with 5 N m s/rad.",
              "<b>Swing task during ramps.</b> In the lift and placement the task uses the ramped "
              "ξ<sub>n</sub><sup>d</sup>, ρ<sup>d</sup> with their rates and the reference acceleration of (6.3), "
              "and ρ<sup>d</sup> in (6.20); the thesis holds these references. Without the acceleration term RL's "
              "normal error after the lift was 56.8 mm.",
              "<b>AMB-off ablation.</b> Same WBC, hierarchy and lift; the AMB feedforward and the tangential "
              "pass-through (6.23) are removed and the swing task holds the tangential channel at the lift "
              "target's angle. The thesis says only that the run removes the tangential balance acceleration.",
              "<b>Disturbance direction.</b> +t<sub>h</sub>, the sign of thesis Fig. 6.18 (L, dL/dt positive in the "
              "pulse) and Fig. 6.15 (CoM leaning to −t<sub>h</sub> under the ramp); the thesis says only "
              "'support-normal'. The −t<sub>h</sub> push sweep is a side result.",
              "<b>x<sub>3</sub> schedule.</b> The lift ramps x<sub>3</sub> to −304 deg, so the thesis' first value "
              "(−340 deg) is a schedule entry at 0.4 s; schedule times count from MdlAMB activation.",
              "<b>Orientation input.</b> Quintic envelope (the thesis does not give its shape); the reference rate "
              "feeds the orientation task and the MPC reference; no feedforward acceleration (measured to make no "
              "difference).",
              "<b>Stop sequence.</b> The radial placement reference ramps to the radius that puts the contact point "
              "10 mm into the ground along the current radial axis, not back to the hand-off radius, because "
              "x<sub>2</sub> does not return to its hand-off value (the literal target left FR 19.5 mm above the "
              "ground in an earlier build). Promotion is kinematic (within 5 mm of the ground, sinking at most "
              "1 cm/s; the thesis gives no criterion). The MPC adds the former swing feet at the second "
              "touchdown, so the first-promoted foot is a WBC contact with zero desired force for some tens of "
              "ms (25 ms in the transfer run). The final split is the four-foot MPC plan, not equal shares.",
              "<b>Metrics.</b> Table 6.6's 'total' RMSE is not defined in the thesis; pooled and summed-force RMSE "
              "are in results.json. ξ<sub>t,lim</sub> is computed by forward kinematics with only the abduction "
              "moved to its limits, at the held radius."])]

    # 5 ------------------------------------------------------------------------
    B += [("h2", "5. Open issues and limits"),
          ("ol", [
              "<b>MPC solve time against the 1 ms period (needs a decision).</b> Two-foot MPC re-solves (every 10th "
              "tick) take about 0.75 ms on average and up to 1.0–1.4 ms, the same cost MdlTrot pays; this is "
              "accepted. With four contacts (load transfer and final hold, 6 s per stop) they take 2.0–2.2 ms on "
              "average with peaks of 6.2–6.6 ms in the first ≈0.15 s after the contact switch (test-rig "
              "instrumentation). From the runs' closing lines: computeTick mean "
              f"{min(means):.0f}–{max(means):.0f} µs over all ticks; max {min(bal2) / 1e3:.2f}–"
              f"{max(bal2) / 1e3:.2f} ms in balanced two-foot runs, {tr['closing']['tick_max_us'] / 1e3:.2f} ms in "
              f"the transfer run, up to {max(falling) / 1e3:.2f} ms in runs that fall. Timing varies between "
              "identical runs, and headless non-real-time runs hide overruns; on hardware the four-foot solve "
              "would miss deadlines.",
              "<b>Dependence on cancelling joint friction.</b> Balance needs the swing joints' simulated damping and "
              "dry friction cancelled: uncancelled, the swing legs creep into the hip-roll stop and the robot falls "
              "4.6 s after the hand-off (one-off run). An error of ±0.2 N m in the cancelled friction also made it "
              "fall in an earlier build. Real joint friction is unknown and will differ.",
              f"<b>Operating range.</b> Persistent force bounded only up to {f_ok[-1]} N and the push boundary "
              f"{bw[0]}–{bw[1]} N along +t<sub>h</sub> (Sec. 3.5–3.7). The measured cause is the little CoM "
              "authority of an abduction-only tangential channel in the centreline stance. Reaching the thesis' "
              "range needs a structural change (a wider diagonal from MdlDiagonalStance, or trunk tasks that yield "
              "about the support line), not a gain.",
              "<b>x<sub>2</sub> has no restoring action of its own.</b> It settles only through the trunk's "
              "orientation stiffness (Sec. 3.3), slowly (about 20 s), and a 'no fall' or 'bounded' verdict over a "
              "window is not by itself a settled state. Abduction margin at the end of each window (rate; +: "
              "moving away from the stop): " + "; ".join(f"{n} {mrg(r)}" for n, r in margins) +
              ". The joint-space path shows no such motion; why was not investigated. An x<sub>2</sub> term in the "
              "law would be an addition to the thesis (decision).",
              "<b>Branch cut in the reduced state.</b> θ<sub>2</sub>, θ<sub>3</sub> (and ξ<sub>t</sub>) jump by "
              "2π when a swing leg passes straight up. "
              + (f"In the logged runs this happens only after divergence, {min(lead):.2f}–{max(lead):.2f} s before "
                 "the fall. " if lead else "No logged run reaches it before its fall. ")
              + "Not fixed; it would matter if a recovery ever passed through that posture.",
              "<b>Rig-only light contact.</b> In `test_amb_wbc_sim` the promoted FR's contact point reads up to "
              "3.2 mm above the ground while the QP asks it for up to 2.1 N more than the simulator gives; the "
              "full stack does not show this. A per-foot minimum normal force in QuadrupedWBC would probably "
              "prevent it (decision).",
              "<b>Hardware-dependent thresholds.</b> The touchdown criterion (5 mm, 1 cm/s) was set from MuJoCo "
              "soft-contact readings and needs checking on the robot.",
              "<b>Test coverage.</b> `test_amb_wbc_sim` has no push block, and the stop sequence's failure paths (a "
              "foot that never lands fails the tick; a stop during the lift waits for its end) were checked only "
              "in one-off full-stack runs.",
              "<b>Joint-space stop.</b> D still sits at once and the swing feet drop; a sit after a balance ending "
              "near the abduction stops failed in an earlier stage. The stop sequence fixes this in WBC mode only.",
              "<b>Scope.</b> Simulation only, one diagonal, one direction per sweep (plus the −t<sub>h</sub> side "
              "result). Runs repeat but not bit for bit: in a repeat of the whole set every gate and boundary "
              "was the same and the metrics agreed to about three digits (the hand-off can land 1 ms apart); "
              "computeTick timing varied."])]

    # 6 ------------------------------------------------------------------------
    B += [("h2", "6. Reproduction"),
          ("p", "From the repository root; one simulator at a time (it binds a fixed port; the script refuses to "
                "start while another `simulation` process runs)."),
          ("code",
           "cd /home/ege/legged/iros/project\n"
           "# all Sec. 6.6 experiments, headless, then results.json, figures and this report\n"
           "/home/ege/legged/iros/.venv/bin/python figs/amb_wbc/run_experiments.py\n"
           "# a subset, merged into the existing results.json (names: ablation transfer x3\n"
           "# orientation force push workspace); --out DIR writes runs/, results.json, figs/ under DIR\n"
           "/home/ege/legged/iros/.venv/bin/python figs/amb_wbc/run_experiments.py --only push workspace\n"
           "# this report alone, from the existing results.json and figures (no simulation)\n"
           "/home/ege/legged/iros/.venv/bin/python figs/amb_wbc/build_report.py\n"
           "\n"
           "# build, install (needed before running from bin/), tests\n"
           "cmake --build build -j 12 && cmake --install build\n"
           "cd build && ctest --output-on-failure\n"
           "# one headless balance with the autorun (stand, diagonal stance, balance, stop, sit)\n"
           "cd ../bin && ./sim.sh -c \"simulation.headless = true\" -c \"simulation.realtime = false\" \\\n"
           "                      -c \"supervisor.autorun_amb_duration = 10\" < /dev/null"),
          ("p", "<b>Interactive.</b> `cd bin && ./sim.sh`, then keys in the terminal or the simulator window: S "
                "(stand; wait for it to settle), B (diagonal stance, which hands over to balancing by itself "
                f"{hand - t_b:.1f} s later), P in the simulator window (push: `simulation.push_force` for "
                "`push_duration`, ramped over `push_ramp`), D (WBC: place the feet, transfer the load, sit; joint "
                "space: sit), Q (quit). Useful overrides: `-c \"amb.wbc = false\"` (joint-space path), "
                "`-c \"amb.enabled = false\"` (ablation), `-c \"amb.x3_schedule = [[0.4, -340.0], [2.0, -220.0]]\"`, "
                "`-c \"amb.pitch_amp_deg = 5\"`, `-c \"simulation.push_ramp = 2\"`."),
          ("p", "<b>Outputs.</b> Logs: `runs/amb_wbc/<experiment>/<run>.mat` and `.log` "
                f"({hz:.0f} Hz Supervisor logs and "
                "the simulator's stdout; layouts of `MdlAMB_state` and `MdlAMB_wbc` at `_log` and `_logWbc` in "
                "`include/quadruped/MdlAMB.hh`). Results: `reports/amb_wbc/results.json` (metrics, thesis values "
                "where stored, gates). Figures: `reports/amb_wbc/figs/*.png`, named after the thesis figure or "
                "table they reproduce. Report: `reports/amb_wbc/REPORT.md` and `REPORT.pdf`, both from "
                "`figs/amb_wbc/build_report.py`.")]
    return B


# -- rendering -------------------------------------------------------------------

def check_markup(s):
    """Only the five inline forms; a stray '<' or '>' would break the PDF
    parser or show raw in Markdown."""
    bare = re.sub(r"`[^`]*`", "", re.sub(r"</?(b|i|sub|sup)>", "", s))
    if "<" in bare or ">" in bare:
        raise ValueError(f"stray angle bracket in: {s[:80]}")
    return s


def md_inline(s, cell=False):
    s = check_markup(s).replace("<b>", "**").replace("</b>", "**").replace("<i>", "*").replace("</i>", "*")
    return s.replace("|", "\\|") if cell else s


def pdf_inline(s):
    def code(m):
        return '<font name="Mono">' + m.group(1).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;") \
               + "</font>"
    parts = re.split(r"(`[^`]*`)", check_markup(s))
    s = "".join(code(re.match(r"`([^`]*)`", p)) if p.startswith("`") else p.replace("&", "&amp;") for p in parts)
    # reportlab shifts sub/superscripts by half the font size, onto the neighbouring lines
    return s.replace("<sub>", '<sub rise="30%">').replace("<sup>", '<sup rise="40%">')


def write_md(blocks, path):
    out = []
    for b in blocks:
        kind = b[0]
        if kind == "title":
            out.append(f"# {b[1]}\n")
        elif kind in ("h2", "h3"):
            out.append(f"{'#' * int(kind[1])} {md_inline(b[1])}\n")
        elif kind == "p":
            out.append(md_inline(b[1]) + "\n")
        elif kind == "ol":
            out.append("\n".join(f"{i}. {md_inline(t)}" for i, t in enumerate(b[1], 1)) + "\n")
        elif kind == "table":
            _, cap, head, rows, _w = b
            out.append(f"*{md_inline(cap)}*\n")
            out.append("| " + " | ".join(md_inline(h, True) for h in head) + " |")
            out.append("|" + "---|" * len(head))
            out += ["| " + " | ".join(md_inline(c, True) for c in r) + " |" for r in rows]
            out.append("")
        elif kind == "fig":
            _, name, _w, cap = b
            out.append(f"![{name}](figs/{name})\n\n*{md_inline(cap)}*\n")
        elif kind == "code":
            out.append("```bash\n" + b[1] + "\n```\n")
    with open(path, "w") as f:
        f.write("\n".join(out))


def write_pdf(blocks, path, rep):
    fonts = os.path.join(matplotlib.get_data_path(), "fonts", "ttf")   # DejaVu: Greek, math, arrows
    for name, file in (("Sans", "DejaVuSans"), ("Sans-B", "DejaVuSans-Bold"), ("Sans-I", "DejaVuSans-Oblique"),
                       ("Sans-BI", "DejaVuSans-BoldOblique"), ("Mono", "DejaVuSansMono")):
        pdfmetrics.registerFont(TTFont(name, os.path.join(fonts, file + ".ttf")))
    pdfmetrics.registerFontFamily("Sans", normal="Sans", bold="Sans-B", italic="Sans-I", boldItalic="Sans-BI")
    body = ParagraphStyle("body", fontName="Sans", fontSize=8.6, leading=11.6, spaceAfter=4.5)
    st = {"title": ParagraphStyle("title", parent=body, fontName="Sans-B", fontSize=14, leading=18, spaceAfter=6),
          "h2": ParagraphStyle("h2", parent=body, fontName="Sans-B", fontSize=11, leading=14, spaceBefore=8,
                               spaceAfter=4),
          "h3": ParagraphStyle("h3", parent=body, fontName="Sans-B", fontSize=9.4, leading=12, spaceBefore=6,
                               spaceAfter=3),
          "p": body,
          "li": ParagraphStyle("li", parent=body, leftIndent=13, bulletIndent=0, bulletFontName="Sans",
                               bulletFontSize=8.6, spaceAfter=2.5),
          "cap": ParagraphStyle("cap", parent=body, fontName="Sans-I", fontSize=7.6, leading=9.6, spaceAfter=7),
          # leading 9.6 at 7 pt: a subscript must not touch the next line
          "cell": ParagraphStyle("cell", parent=body, fontSize=7.0, leading=9.6, spaceAfter=0),
          "head": ParagraphStyle("head", parent=body, fontName="Sans-B", fontSize=7.0, leading=9.6, spaceAfter=0),
          "code": ParagraphStyle("code", parent=body, fontName="Mono", fontSize=7.5, leading=9.4,
                                 backColor=colors.HexColor("#f3f3f3"), borderPadding=4, spaceBefore=3,
                                 spaceAfter=8)}
    doc = SimpleDocTemplate(path, pagesize=A4, leftMargin=17 * mm, rightMargin=17 * mm, topMargin=15 * mm,
                            bottomMargin=16 * mm, title=TITLE, author="quadcontrol")
    W, H = doc.width, doc.height

    def footer(c, d):
        c.setFont("Sans", 7)
        c.setFillColor(colors.HexColor("#666666"))
        c.drawString(doc.leftMargin, 9 * mm, "MdlAMB whole-body realization: thesis Sec. 6.6 experiments")
        c.drawRightString(doc.leftMargin + W, 9 * mm, f"page {d.page}")

    flow = []
    for b in blocks:
        kind = b[0]
        if kind in ("title", "h2", "h3", "p"):
            flow.append(Paragraph(pdf_inline(b[1]), st[kind]))
        elif kind == "ol":
            flow += [Paragraph(pdf_inline(t), st["li"], bulletText=f"{i}.") for i, t in enumerate(b[1], 1)]
        elif kind == "table":
            _, cap, head, rows, widths = b
            data = [[Paragraph(pdf_inline(h), st["head"]) for h in head]] + \
                   [[Paragraph(pdf_inline(c), st["cell"]) for c in r] for r in rows]
            t = Table(data, colWidths=[w * W for w in widths], repeatRows=1, hAlign="LEFT")
            t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#b8b8b8")),
                                   ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#ececec")),
                                   ("VALIGN", (0, 0), (-1, -1), "TOP"),
                                   ("LEFTPADDING", (0, 0), (-1, -1), 2.5), ("RIGHTPADDING", (0, 0), (-1, -1), 2.5),
                                   ("TOPPADDING", (0, 0), (-1, -1), 1.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]))
            flow.append(KeepTogether([Paragraph(pdf_inline(cap), st["cap"]), t, Spacer(1, 7)]))
        elif kind == "fig":
            _, name, frac, cap = b
            img = os.path.join(rep, "figs", name)
            iw, ih = ImageReader(img).getSize()
            w = frac * W
            h = w * ih / iw
            if h > 0.8 * H:          # a tall figure must still fit one frame with its caption
                w, h = w * 0.8 * H / h, 0.8 * H
            flow.append(KeepTogether([Image(img, width=w, height=h), Paragraph(pdf_inline(cap), st["cap"])]))
        elif kind == "code":
            if max(len(line) for line in b[1].splitlines()) > CODE_COLS:
                raise ValueError("code line too long for the page width")
            flow.append(Preformatted(b[1], st["code"]))
    doc.build(flow, onFirstPage=footer, onLaterPages=footer)


def build(rep):
    """REPORT.md and REPORT.pdf in rep from rep/results.json and rep/figs."""
    path = os.path.join(rep, "results.json")
    with open(path) as f:
        R = json.load(f)
    written = datetime.datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M")
    blocks = content(R, written)
    write_md(blocks, os.path.join(rep, "REPORT.md"))
    write_pdf(blocks, os.path.join(rep, "REPORT.pdf"), rep)
    print(f"report: {os.path.join(rep, 'REPORT.md')}, REPORT.pdf")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", help="directory holding results.json and figs/ (default: reports/amb_wbc)")
    a = ap.parse_args()
    build(os.path.abspath(a.out) if a.out else os.path.join(REPO, "reports", "amb_wbc"))


if __name__ == "__main__":
    main()
