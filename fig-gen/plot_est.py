#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Estimator figures for one Supervisor .mat log (a quick run from bin/ or a benchmark run).

  fig-gen/plot_est.py LOG.mat [LOG2.mat ...] [--out DIR] [--window LO HI] [--label NAME]
                      [--probe T [--gains KP KI TAU]]

Writes <out>/story.{png,pdf} (velocity error per gait cycle, zoom, attitude, gyro bias,
consistency), <out>/phase.{png,pdf} when the log holds a trot, and <out>/convergence.{png,pdf}
when --probe gives the time a tilt error was planted. The default output directory is
<LOG dir>/<LOG name>_figs/. The analysis window defaults to the steady part of the log
(gait minus its ramp-up, or the active span); pass --window to choose it. Variables the log
does not carry are skipped, so the default supervisor.toml variable list works too.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from estplots import Log, default_window, fig_convergence, fig_phase, fig_story, setup_style, DEG  # noqa: E402


def summary(L, lo, hi):
    """A few headline numbers, printed so a quick run can be judged without opening the figures."""
    w = (L.t >= lo) & (L.t <= hi) & L.active()
    out = [f"{L.label}: window {lo:.1f}-{hi:.1f} s, {L.dt*1e3:.0f} ms samples"]
    ve = L.vel_err_body()
    if ve is not None:
        out.append("  velocity error, body frame  mean %s  rms %s mm/s" % (
            " / ".join(f"{x*1e3:+.1f}" for x in np.nanmean(ve[w], axis=0)),
            " / ".join(f"{x*1e3:.1f}" for x in np.sqrt(np.nanmean(ve[w]**2, axis=0)))))
    tilt, head = L.tilt_heading()
    if tilt is not None:
        slope = np.polyfit(L.t[w], np.unwrap(head[w]) * DEG, 1)[0] * 60
        exp = L.expected_heading_slope(w)
        out.append(f"  tilt error rms {np.sqrt(np.mean((tilt[w]*DEG)**2)):.2f} deg, max {np.max(tilt[w]*DEG):.2f}; heading drift {slope:+.2f} deg/min"
                   + (f" (bias predicts {exp*DEG*60:+.2f})" if exp is not None else ""))
    be = L.bias_err()
    if be is not None:
        out.append("  gyro-bias error  mean %s  peak %s mrad/s" % (
            " / ".join(f"{x*1e3:+.2f}" for x in np.nanmean(be[w], axis=0)),
            " / ".join(f"{x*1e3:.2f}" for x in np.max(np.abs(be[L.active()]), axis=0))))
    return "\n".join(out)


def plot_log_file(path, out=None, window=None, label=None, probe=None, gains=(1.0, 0.25, 0.5)):
    setup_style()
    if label is None and os.path.splitext(os.path.basename(path))[0] == "run":
        label = os.path.basename(os.path.dirname(os.path.abspath(path)))
    L = Log(path, label)
    lo, hi = window if window else default_window(L)
    out = out or os.path.join(os.path.dirname(os.path.abspath(path)), os.path.splitext(os.path.basename(path))[0] + "_figs")
    files = fig_story(L, lo, hi, os.path.join(out, "story"))
    files += fig_phase(L, lo, hi, os.path.join(out, "phase"))
    if probe is not None:
        files += fig_convergence(L, probe, os.path.join(out, "convergence"), *gains)
    print(summary(L, lo, hi))
    for f in files:
        print("  wrote", f)
    return files


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="Supervisor .mat log(s)")
    ap.add_argument("--out", help="output directory (default <log>_figs next to the log; with several logs a subfolder per log)")
    ap.add_argument("--window", nargs=2, type=float, metavar=("LO", "HI"), help="analysis window in sim seconds")
    ap.add_argument("--label", help="title label (default: file name)")
    ap.add_argument("--probe", type=float, help="sim time at which a tilt error was planted (draws convergence)")
    ap.add_argument("--gains", nargs=3, type=float, default=(1.0, 0.25, 0.5), metavar=("KP", "KI", "TAU"))
    a = ap.parse_args()
    for p in a.logs:
        out = a.out
        if out and len(a.logs) > 1:
            out = os.path.join(out, os.path.splitext(os.path.basename(p))[0])
        plot_log_file(p, out, tuple(a.window) if a.window else None, a.label if len(a.logs) == 1 else None, a.probe, tuple(a.gains))


if __name__ == "__main__":
    main()
