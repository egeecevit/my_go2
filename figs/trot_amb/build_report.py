#!/usr/bin/env python3
#
# Copyright (C) 2005-2026 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""REPORT.md and REPORT.pdf on AMB inside the whole-body trot, from results.json
and the logs it names, in reports/trot_amb (or --out DIR). No simulation runs:

  /home/ege/legged/iros/.venv/bin/python figs/trot_amb/build_report.py [--out DIR]

run_experiments.py calls build() at the end of a full run. Every measured number
is read from results.json; the block renderers are figs/amb_wbc's and the PDF
layout figs/trot_wbc's.
"""

import argparse
import datetime
import importlib.util
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TITLE = "Angular-momentum balancing inside the whole-body trot: static hold and trot in place"


def _load(name, *path):
    spec = importlib.util.spec_from_file_location(name, os.path.join(REPO, "figs", *path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def fx(v, n=2):
    return "–" if v is None else f"{v:.{n}f}"


def outcome(x):
    return f"falls after {x['survived_s']:.1f} s" if x["failed"] else "no fall"


def held(x):
    return f"{x['phases_held']} of {x['phases']}"


def tilt(x):
    return "–" if x["failed"] else f"{fx(x['roll_peak'], 1)} / {fx(x['pitch_peak'], 1)}"


def headline_fig(R, rep, logs):
    """Roll, pitch, CoM offset and swing-foot height over five gait cycles of
    the headline gait, nothing against AMB with centring."""
    T = _load("trot_wbc_run_experiments", "trot_wbc", "run_experiments.py")
    rows = {x["case"]: x for x in R["inplace"]["grid"] if x["t_two"] == 1.0}
    f, ax = plt.subplots(3, 1, figsize=(7.2, 6.0), sharex=True)
    for case, col in (("nothing", "#d9534f"), ("offset", "#257de0")):
        x = rows[case]
        run = T.Run(os.path.join(logs, "inplace", x["name"]), "wbc",
                    {"vx": 0.0, "vy": 0.0, "yaw": 0.0, "period": x["period"], "duty": x["duty"]})
        w = run.win(x["entry_t"] + 3.0, x["entry_t"] + 12.0)
        t = run.t[w] - x["entry_t"]
        lab = "AMB off" if case == "nothing" else "AMB (offset) with centring"
        ax[0].plot(t, np.degrees(run.rpy[w, 0]), color=col, lw=1.0, label=lab + ", roll")
        ax[0].plot(t, np.degrees(run.rpy[w, 1]), color=col, lw=1.0, ls="--", label="pitch")
        ax[1].plot(t, 1e3 * run.ch[w], color=col, lw=1.0)
        # the lower of the two scheduled swing feet; four-foot samples left out
        z = np.where(run.pair[w] == 0, run.cp[w][:, [1, 2], 2].min(1), run.cp[w][:, [0, 3], 2].min(1))
        ax[2].plot(t, np.where(run.pair[w] >= 0, 1e3 * z, np.nan), color=col, lw=1.0)
    ax[0].set_ylabel("roll, pitch [deg]")
    ax[1].set_ylabel("CoM offset from the\nsupport line $c_h$ [mm]")
    ax[2].set_ylabel("lower swing foot,\nheight [mm]")
    ax[2].set_xlabel("time since TROT entry [s]")
    ax[0].legend(loc="upper right", ncol=2, fontsize=7)
    for a in ax:
        a.grid(alpha=0.3)
    os.makedirs(os.path.join(rep, "figs"), exist_ok=True)
    f.tight_layout()
    f.savefig(os.path.join(rep, "figs", "headline.png"), dpi=150)
    plt.close(f)


def content(R, written):
    base, st, ip, meta = R["baseline"], R["static"], R["inplace"], R["meta"]
    cfg = meta["config"]
    grid = {(x["t_two"], x["case"]): x for x in ip["grid"]}
    var = {x["case"]: x for x in ip["variants"]}
    basin = {(x["mode"], x["origin_x"]): x for x in st["basin"]}
    aids = {x["case"]: x for x in st["aids"]}
    cen = {(x["case"], x["first"]): x for x in st["centred"]}
    two = sorted({x["t_two"] for x in ip["grid"]})
    origins = sorted({x["origin_x"] for x in st["basin"]}, reverse=True)
    amb_rows = [grid[t, m] for t in two for m in ("offset", "passthrough")]
    worst = max(max(x["roll_peak"], x["pitch_peak"]) for x in amb_rows)
    hold = {m: [basin[m, o]["ch_at_liftoff_mm"][0] for o in origins if basin[m, o]["phases_held"] == 1] for m in
            ("offset", "passthrough")}
    lost = {m: [basin[m, o]["ch_at_liftoff_mm"][0] for o in origins if basin[m, o]["phases_held"] == 0] for m in
            ("offset", "passthrough")}
    default_falls = sorted({x["t_two"] for x in base["default"] if x["failed"]})
    push = ip["push"]
    h = grid[1.0, "offset"]

    def pushed(c):
        return " / ".join("none" if push[c][s]["largest_survived_N"] is None else f"{push[c][s]['largest_survived_N']} N"
                          for s in ("plus_y", "minus_y"))

    B = [("title", TITLE),
         ("p", f"Generated {written} from `reports/trot_amb/results.json`. Branch `amb-trot`. Simulated Go2, trot in "
               "place, whole-body path (`trot.wbc = true`). The controller's state input is simulator ground truth "
               "(attitude and position/velocity) on every run except the two marked filter runs. The attitude "
               "integral is on unless a row says otherwise."),
         ("h2", "1. Result"),
         ("ol", [
             f"<b>AMB makes a slow trot in place possible, with one addition.</b> With two-foot phases of "
             f"{two[0]:.2f} to {two[-1]:.1f} s and a 0.5 s four-foot phase, every two-foot phase is a real two-foot "
             f"stand and roll and pitch stay within {worst:.1f} deg, in both realizations of AMB (Table 4). Without "
             "AMB the same gaits never stand on two feet: the body tips about 15 deg onto a swing foot in every "
             "phase, or falls.",
             "<b>The addition is CoM centring in the four-foot phase.</b> The swing legs can only absorb a CoM "
             f"that starts within about 3 mm of the support line (held from {min(hold['passthrough']):.1f} to "
             f"{max(hold['passthrough']):.1f} mm, lost at {min(x for x in lost['passthrough'] if x > 0):.1f} mm "
             f"and at {max(x for x in lost['passthrough'] if x < 0):.1f} mm; Table 2). On the trot's own footprint the CoM "
             f"is {basin['offset', 0.0]['ch_at_liftoff_mm'][0]:.1f} mm off each diagonal at the first lift-off. "
             "While all four feet are down, the body reference is therefore shifted until the CoM is over the next "
             "support line. Centring without AMB does not hold a phase; AMB without centring falls after "
             f"{var['offset, no centring']['survived_s']:.1f} s.",
             "<b>Both realizations work, and they are equivalent in quality.</b> The pass-through (thesis Eqs. "
             "6.23, 6.26) needs the joint friction cancelled, as `MdlAMB` did; the reference offset does not.",
             f"<b>Limits.</b> It needs a four-foot phase of about 0.5 s (0.3 s and 0.2 s fall). A 0.1 s lateral "
             f"push in the middle of a two-foot phase is survived up to {pushed('offset')} (offset, +y / -y) and "
             f"{pushed('passthrough')} (pass-through). With the attitude filter in the loop it falls after "
             f"{var['offset, both filters in the loop']['survived_s']:.1f} s. Forward speed was not attempted.",
             "<b>Two things had to change in the trot itself</b> before any of this could be measured: the swing "
             "has to leave the ground quickly (`swing_lift_time`), and AMB has to be back on the planned "
             "trajectory before the foot comes down (Section 3)."]),
         ("h2", "2. What was built"),
         ("table", "Table 1. The pieces. All keys are in `[trot]`; the defaults leave the trot unchanged.",
          ["Piece", "Where", "What it does"],
          [["Support pair as a parameter", "`MdlAMB::geometry`, `robotMomentum`, `project` (`pair_t`)",
            "The projection onto the support line for either diagonal. The second diagonal is ordered RL to FR: it "
            "is the mirror image of FL to RR, and reversing the line keeps every sign convention. Checked against "
            "the mirrored state in `test_trot_amb`."],
           ["`TrotAMB`", "`TrotAMB.{hh,cc}`, called from `MdlTrot::_wbcTick`",
            "The thesis law (`MdlAMB::ambLaw`, same template and gains) on the current stance pair in every "
            "two-foot phase, state reset at each phase. `amb`, `amb_mode`."],
           ["Mode `offset`", "`TrotAMB::rotateReference`",
            "The law's angular acceleration is integrated into a rotation of each planned swing-foot reference "
            "about its hip, axis parallel to the support line; returned to zero before the descent."],
           ["Mode `passthrough`", "`MdlTrot::wbcSolve` (`amb_dir`, `amb_share`, `amb_acc`)",
            "Thesis (6.23), (6.26): the swing task's error is removed along the tangential direction and the "
            "acceleration is a pure feedforward. Friction cancellation and joint PD off on that leg (6.28)."],
           ["Swing lift time", "`SwingTrajectory::sampleLifted`, `swing_lift_time`",
            "The foot rises in a fixed time, holds, and comes down in the same time."],
           ["CoM centring", "`MdlTrot::_wbcTick`, `amb_centre_tc`",
            "In four-foot phases an integrator shifts the base position reference until the CoM is over the next "
            "support line."]],
          [0.20, 0.30, 0.50]),
         ("p", f"Settings of the runs below: `swing_lift_time = 0.2`, `wbc_touchdown_depth = {R['depth']['chosen']}` "
               f"(Section 6), `amb_centre_tc = 0.3` where centring is on, law gains {cfg['amb_gains']}, template as "
               f"`[amb]` (the centreline fit was not redone: it held), `amb_alpha_max = {cfg['amb_alpha_max']}`, "
               f"`amb_return = {cfg['amb_return']}`."),
         ("p", f"Checks: `ctest` {R['tests']['summary'][1].strip()} (22 with the new `test_trot_amb`; "
               "`test_trot_wbc` gained a pass-through case). With `amb = false` and the other new keys at their "
               "defaults, the default trot on both paths, a slow whole-body trot and the key-B `MdlAMB` balance log "
               "the same values tick for tick as before the work (compared on 2026-11-27 with the final binary)."),
         ("h2", "3. Three findings on the way"),
         ("ol", [
             "<b>A long swing did not leave the ground.</b> The trot's clearance profile grows with the cube of the "
             "swing's progress: an 8 s swing is 0.7 mm up after 0.4 s. The foot scheduled to swing stayed on the "
             "ground, was loaded as the body began to tip (46 N for 70 ms measured) and kicked the body about the "
             "support line. The \"two-foot phases\" of the earlier baseline were three-foot phases: in Table 3 no "
             "phase of any gait without AMB counts as a two-foot stand.",
             "<b>The CoM is not over the diagonals.</b> On the default footprint it is about 9 mm from each "
             "support line at the first lift-off, which is three times what the swing legs can absorb.",
             "<b>A foot must not land with AMB's offset on it.</b> With the lift profile the descent starts before "
             "the touchdown; a foot that met the ground while still rotated by AMB landed 3 to 7 cm off its target. "
             "AMB now finishes its return before the descent begins."]),
         ("h2", "4. Static hold on the trot's own footprint (step 1)"),
         ("p", f"One two-foot phase of {st['basin'][0]['t_two']:.0f} s. The footprint is moved fore-aft "
               "(`trot.origin`) so the CoM starts at different distances from the support line; no centring."),
         ("table", "Table 2. Static hold against the CoM's starting offset from the support line. Held: neither "
                   "swing foot came back to the ground and the body stayed within 10 deg.",
          ["Footprint origin x [mm]", "CoM offset at lift-off [mm]", "offset: held", "offset: peak roll / pitch [deg]",
           "offset: leg travel [deg]", "pass-through: held", "pass-through: peak roll / pitch [deg]",
           "pass-through: leg travel [deg]"],
          [[f"{1e3 * o:+.0f}", fx(basin["offset", o]["ch_at_liftoff_mm"][0], 1)] +
           [c for m in ("offset", "passthrough") for c in
            ("yes" if basin[m, o]["phases_held"] else "no", tilt(basin[m, o]),
             fx((basin[m, o].get("amb") or {}).get("leg_travel_peak_deg"), 0) if basin[m, o]["phases_held"] else "–")]
           for o in origins],
          [0.11, 0.12, 0.10, 0.15, 0.12, 0.12, 0.15, 0.13]),
         ("p", f"The offset mode held the phase started at {basin['offset', -0.017]['ch_at_liftoff_mm'][0]:.1f} mm "
               "and fell afterwards, at the stop. At the origin of -13 mm, AMB off tips onto a swing foot "
               f"({tilt(aids['off'])} deg, {held(aids['off'])} held). The pass-through without friction cancellation "
               f"{outcome(aids['passthrough_no_friction'])}; with the cancellation it holds whether the swing joint "
               f"PD is on ({tilt(aids['passthrough_joint_pd_on'])} deg) or off ({tilt(aids['passthrough_both_aids'])} "
               "deg)."),
         ("table", "Table 3. Both diagonals in one run on the default footprint, with centring: 8 s on one "
                   "diagonal, 0.5 s on four feet, 8 s on the other.",
          ["Case", "First diagonal", "Outcome", "Two-foot phases held", "Peak roll / pitch [deg]",
           "CoM offset at the lift-offs [mm]"],
          [[c, f, outcome(cen[c, f]), held(cen[c, f]), tilt(cen[c, f]),
            ", ".join(fx(v, 1) for v in cen[c, f]["ch_at_liftoff_mm"])]
           for c in ("centring only", "offset", "passthrough") for f in ("FL-RR", "FR-RL")],
          [0.16, 0.13, 0.19, 0.16, 0.18, 0.18]),
         ("p", "The first lift-off comes 0.5 s after TROT entry, when the centring has brought the CoM from 9 to "
               "5.5 mm only. The offset mode absorbed that start here; the pass-through did not. In the gait runs "
               "below, with shorter phases, both get through the first phase."),
         ("h2", "5. Trot in place with long two-foot phases (step 2)"),
         ("table", "Table 4. Four-foot phase 0.5 s. Nothing: AMB and centring off, same lift profile.",
          ["Two-foot phase [s]", "Case", "Outcome", "Two-foot phases held", "Peak roll / pitch [deg]",
           "Roll / pitch rms [deg]", "CoM offset rms [mm]", "Leg travel [deg]"],
          [[f"{t:.2f}", c, outcome(grid[t, c]), held(grid[t, c]), tilt(grid[t, c]),
            "–" if grid[t, c]["failed"] else f"{fx(grid[t, c]['roll_rms'])} / {fx(grid[t, c]['pitch_rms'])}",
            fx(grid[t, c].get("ch_rms_mm"), 1), fx((grid[t, c].get("amb") or {}).get("leg_travel_peak_deg"), 0)]
           for t in two for c in ("nothing", "centring only", "offset", "passthrough")],
          [0.09, 0.13, 0.17, 0.11, 0.14, 0.14, 0.11, 0.11]),
         ("fig", "headline.png", 0.80,
          "Figure 1. Two-foot phase 1.0 s, four-foot phase 0.5 s. Without AMB the body tips until a swing foot is "
          "back on the ground (height 0) in every phase. With AMB and centring the swing feet stay up and the CoM "
          "stays on the support line."),
         ("table", "Table 5. Variants of the 1.0 s / 0.5 s gait, AMB in offset mode.",
          ["Variant", "Outcome", "Two-foot phases held", "Peak roll / pitch [deg]", "Roll / pitch rms [deg]"],
          [["as Table 4", outcome(h), held(h), tilt(h), f"{fx(h['roll_rms'])} / {fx(h['pitch_rms'])}"]] +
          [[x["case"].replace("offset, ", ""), outcome(x), held(x), tilt(x),
            "–" if x["failed"] else f"{fx(x['roll_rms'])} / {fx(x['pitch_rms'])}"] for x in ip["variants"]],
          [0.36, 0.18, 0.14, 0.16, 0.16]),
         ("p", "The attitude integral does not disturb AMB: with it off the result is the same or slightly "
               "smoother. The position/velocity filter in the loop costs a factor of two to three in rms; the "
               "attitude filter's noise is not tolerated."),
         ("table", f"Table 6. Lateral 0.1 s push at the base CoM in the middle of a two-foot phase (t = "
                   f"{ip['push_time_s']} s), in {10} N steps to the first fall.",
          ["Case", "Largest push survived, +y [N]", "Largest push survived, -y [N]", "Note"],
          [["nothing", fx(push["nothing"]["plus_y"]["largest_survived_N"], 0),
            fx(push["nothing"]["minus_y"]["largest_survived_N"], 0) + " (sweep cap)",
            "stands on three feet in every phase, pushed or not"],
           ["offset", fx(push["offset"]["plus_y"]["largest_survived_N"], 0),
            fx(push["offset"]["minus_y"]["largest_survived_N"], 0), "all phases held at the surviving force"],
           ["passthrough", fx(push["passthrough"]["plus_y"]["largest_survived_N"], 0),
            fx(push["passthrough"]["minus_y"]["largest_survived_N"], 0), ""]],
          [0.16, 0.22, 0.22, 0.40]),
         ("p", "A two-foot stand held by the swing legs has a small basin: 1 to 2 N s. The trot without AMB shrugs "
               "off far larger pushes because it is already leaning on a third foot."),
         ("h2", "6. Baseline without AMB, and the touchdown depth (step 0)"),
         ("table", "Table 7. The trot as it was before this work (AMB off, default swing profile, 8 mm touchdown "
                   "depth), attitude integral on.",
          ["Two-foot phase [s]"] + [f"four-foot {f:.1f} s" for f in sorted({x["t_four"] for x in base["default"]})],
          [[f"{t:.2f}"] + [next(f"{outcome(x)}" + ("" if x["failed"] else f", peak {tilt(x)} deg")
                                for x in base["default"] if x["t_two"] == t and x["t_four"] == f)
                           for f in sorted({x["t_four"] for x in base["default"]})]
           for t in sorted({x["t_two"] for x in base["default"]})],
          [0.16, 0.28, 0.28, 0.28]),
         ("p", f"At least one column falls at two-foot phases of {', '.join(f'{t:.2f}' for t in default_falls)} s; "
               "the runs that do not fall survive by resting on a swing foot. No phase in this table is a two-foot "
               "stand. Touchdown "
               "depth on the 1.0 s / 0.5 s gait with AMB: " +
               "; ".join(f"{1e3 * x['depth']:.0f} mm: {outcome(x)}" +
                         ("" if x["failed"] else f", rms {fx(x['roll_rms'])} / {fx(x['pitch_rms'])} deg")
                         for x in R["depth"]["rows"]) +
               f". {1e3 * R['depth']['chosen']:.0f} mm is used; the 3 mm run fell at the stop."),
         ("h2", "7. Departures from the thesis and from the plan"),
         ("ol", [
             "Mode `offset` replaces Eqs. (6.23) and (6.28): the tangential references are the integrated motion, "
             "not the measured one, and the joint PD stays on. Its PD share of the tangential acceleration was "
             f"{fx(h['amb']['pd_acc_rms'], 2)} m/s<sup>2</sup> rms against {fx(h['amb']['ff_acc_rms'], 2)} "
             "m/s<sup>2</sup> of feedforward on the headline gait.",
             "The second diagonal is ordered RL to FR, not RF to LH as in Sec. 6.3.1.",
             "CoM centring and the swing lift time are not in the thesis. The centring has the role "
             "`MdlDiagonalStance` has before `MdlAMB`.",
             "The template was not refitted for the trot posture: the centreline parameters held every case here.",
             "The stop key is placed in a four-foot phase. A stop during a long two-foot phase can fall (seen at "
             "the 3 mm depth and after one static hold); the stop ramp takes AMB's authority away with the lift "
             "scale."]),
         ("h2", "8. Open"),
         ("ol", [
             "Four-foot phases shorter than 0.5 s: the centring integrator (0.3 s) does not converge in time, and "
             "0.1 s is unstable against the base position task.",
             "The small push basin and the attitude-noise failure; both are the swing legs' authority.",
             "Forward speeds (step 3): the body shift used by the centring is not what a moving trot wants; "
             "foot placement relative to the CoM is the natural replacement.",
             "Two-foot phases of 0.5 s: with a 0.2 s lift and descent AMB has no window left "
             "(`amb_min_swing`)."]),
         ("h2", "9. Reproduce"),
         ("code", "cd /home/ege/legged/iros/project\n"
                  "cmake --build build -j 12 && cmake --install build && (cd build && ctest)\n"
                  "/home/ege/legged/iros/.venv/bin/python figs/trot_amb/run_experiments.py     # all runs, 15 min\n"
                  "cd bin && ./sim.sh -c \"trot.wbc = true\" -c \"trot.amb = true\" -c \"trot.amb_centre_tc = 0.3\" \\\n"
                  "  -c \"trot.swing_lift_time = 0.2\" -c \"trot.wbc_touchdown_depth = 0.012\" \\\n"
                  "  -c \"trot.forward_velocity = 0.0\" -c \"trot.period = 3.0\" -c \"trot.duty = 0.6666667\"   # S, T, D")]
    return B


def build(rep):
    """REPORT.md and REPORT.pdf in rep from rep/results.json and the logs."""
    amb = _load("amb_wbc_build_report", "amb_wbc", "build_report.py")
    trot = _load("trot_wbc_build_report", "trot_wbc", "build_report.py")
    path = os.path.join(rep, "results.json")
    with open(path) as f:
        R = json.load(f)
    written = datetime.datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M")
    headline_fig(R, rep, os.path.join(REPO, R["meta"]["logs"]))
    blocks = content(R, written)
    amb.write_md(blocks, os.path.join(rep, "REPORT.md"))
    trot.write_pdf(blocks, os.path.join(rep, "REPORT.pdf"), rep, title=TITLE,
                   foot="AMB inside the whole-body trot: static hold and trot in place")
    print(f"report: {os.path.join(rep, 'REPORT.md')}, REPORT.pdf")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", help="directory holding results.json (default: reports/trot_amb)")
    a = ap.parse_args()
    build(os.path.abspath(a.out) if a.out else os.path.join(REPO, "reports", "trot_amb"))


if __name__ == "__main__":
    main()
