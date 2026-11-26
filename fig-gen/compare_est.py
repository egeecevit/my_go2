#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Compare estimator configurations: two logs, or two benchmark directories.

  fig-gen/compare_est.py A.mat B.mat [--out DIR] [--labels LA LB] [--window LO HI]
      -> before_after.{png,pdf}: the same quantities, A grey, B colour, on the same axes

  fig-gen/compare_est.py BASE_DIR FINAL_DIR [--out DIR] [--runs GLOB] [--labels LA LB]
      -> before_after-<run>.{png,pdf} for every run present in both directories that matches
         --runs (default: the standard set below), dumbbell-trot_speeds.{png,pdf} from the
         metrics.json files of trot_v03/v05/v08, and the mechanism explainers
         explain-{rolling,touchdown,heading}.{png,pdf}.

Benchmark directories follow the estbench layout: <dir>/<run>/run.mat (+ metrics.json).
"""

import argparse
import fnmatch
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from estplots import (Log, default_window, fig_before_after, fig_dumbbell, fig_explain_heading,  # noqa: E402
                      fig_explain_rolling, fig_explain_touchdown, setup_style)

STANDARD_PAIRS = ("trot_v05-on-sched-filter-truth", "trot_v05-off-sched-filter-truth",
                  "trot_v05-on-sched-filter-est", "trot_yaw-on-sched-filter-truth",
                  "stand60-on-sched-filter-truth")
EXPLAIN_RUN = "trot_v05-on-sched-filter-truth"
HEADING_RUN = "trot_v05-on-sched-filter-est"
SPEED_RUNS = {"0.3": "trot_v03-on-sched-filter-truth", "0.5": "trot_v05-on-sched-filter-truth", "0.8": "trot_v08-on-sched-filter-truth"}


def compare_logs(a, b, out, labels, window=None, title=None):
    setup_style()
    La, Lb = Log(a, labels[0]), Log(b, labels[1])
    lo, hi = window if window else default_window(Lb)
    return fig_before_after(La, Lb, lo, hi, os.path.join(out, "before_after"), labels, title)


def dumbbell_rows(base, final):
    """Scalar metrics from metrics.json across the three speeds."""
    def M(d, r):
        p = os.path.join(d, r, "metrics.json")
        return json.load(open(p)) if os.path.exists(p) else None
    spec = [("vx bias", "mm/s", lambda m: m["vel_err_mean"][0] * 1e3),
            ("vy bias", "mm/s", lambda m: m["vel_err_mean"][1] * 1e3),
            ("vz bias", "mm/s", lambda m: m["vel_err_mean"][2] * 1e3),
            ("velocity rms, norm", "mm/s", lambda m: float(np.linalg.norm(m["vel_err_rms"])) * 1e3),
            ("xy drift, aligned", "%", lambda m: m.get("xy_drift_aligned_percent", float("nan"))),
            ("tilt rms", "deg", lambda m: m["tilt_rms_deg"]),
            ("bias error peak, run", "mrad/s", lambda m: max(m["bias_err_peak_run"]) * 1e3),
            ("vz rms at TD/LO", "mm/s", lambda m: m["vel_err_rms_transition"][2] * 1e3)]
    rows, speeds = [], []
    for lab, unit, f in spec:
        vals = {}
        for s, r in SPEED_RUNS.items():
            mb, mf = M(base, r), M(final, r)
            if mb and mf:
                vals[s] = (f(mb), f(mf))
        rows.append((lab, unit, vals))
        speeds = sorted(vals)
    return rows, speeds


def compare_dirs(base, final, out, labels, runs_glob=None):
    setup_style()
    files = []
    names = sorted(set(os.listdir(base)) & set(os.listdir(final)))
    wanted = [n for n in names if (fnmatch.fnmatch(n, runs_glob) if runs_glob else n in STANDARD_PAIRS)]
    cache = {}

    def log(d, n):
        key = (d, n)
        if key not in cache:
            p = os.path.join(d, n, "run.mat")
            cache[key] = Log(p, f"{labels[0 if d == base else 1]}") if os.path.exists(p) else None
        return cache[key]

    for n in wanted:
        La, Lb = log(base, n), log(final, n)
        if La is None or Lb is None:
            print(f"[skip] {n}: run.mat missing in one directory")
            continue
        lo, hi = default_window(Lb)
        files += fig_before_after(La, Lb, lo, hi, os.path.join(out, f"before_after-{n}"), labels, title=n)
    rows, speeds = dumbbell_rows(base, final)
    if speeds:
        files += fig_dumbbell(rows, speeds, os.path.join(out, "dumbbell-trot_speeds"), labels,
                              title=f"Scalar metrics across trot speeds: {labels[0]} → {labels[1]} (controller on truth, noisy)")
    Lf, Lb0 = log(final, EXPLAIN_RUN), log(base, EXPLAIN_RUN)
    if Lf is not None:
        lo, hi = default_window(Lf)
        files += fig_explain_rolling(Lf, lo, hi, os.path.join(out, "explain-rolling"), title=f"{EXPLAIN_RUN} ({labels[1]})")
        files += fig_explain_touchdown(Lf, lo, hi, os.path.join(out, "explain-touchdown"), Lb=Lb0, title=f"{EXPLAIN_RUN}")
    Lh = log(base, HEADING_RUN)
    if Lh is not None:
        lo, hi = default_window(Lh)
        files += fig_explain_heading(Lh, lo, hi, os.path.join(out, "explain-heading"), title=f"{HEADING_RUN} ({labels[0]}, controller on the estimator)")
    for f in files:
        print("  wrote", f)
    return files


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", help="baseline .mat log or benchmark directory")
    ap.add_argument("b", help="final .mat log or benchmark directory")
    ap.add_argument("--out", help="output directory (default: <b>/figures for directories, <b dir>/compare_figs for logs)")
    ap.add_argument("--labels", nargs=2, default=None, metavar=("LA", "LB"))
    ap.add_argument("--runs", help="glob on run names for the directory mode (default: the standard pairs)")
    ap.add_argument("--window", nargs=2, type=float, metavar=("LO", "HI"))
    a = ap.parse_args()
    if os.path.isdir(a.a) and os.path.isdir(a.b):
        labels = tuple(a.labels) if a.labels else (os.path.basename(os.path.normpath(a.a)), os.path.basename(os.path.normpath(a.b)))
        compare_dirs(a.a, a.b, a.out or os.path.join(a.b, "figures"), labels, a.runs)
    else:
        labels = tuple(a.labels) if a.labels else ("A", "B")
        out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.b)), "compare_figs")
        for f in compare_logs(a.a, a.b, out, labels, tuple(a.window) if a.window else None):
            print("  wrote", f)


if __name__ == "__main__":
    main()
