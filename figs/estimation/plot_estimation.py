#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Plots the state estimate against simulator ground truth.

Reads a .mat file written by the Supervisor's logging thread and produces one
figure per estimated quantity, each with an x, y and z subplot. Ground truth is
solid blue and the estimate is solid red throughout, so the two are the same two
colours on every axis of every figure.

  body_position.png      r,      against qpos[0:3]
  body_velocity.png      v,      against qvel[0:3]
  body_orientation.png   rpy,    against qpos[3:7] converted to rpy
  foot_position_<LEG>.png  foothold p_i, against the simulator's foot geom, one
                           figure per leg in FL, FR, RL, RR order

See README.md in this directory for how to record the log.
"""

import argparse
import os
import sys

import numpy as np
from scipy.io import loadmat

import matplotlib

# Chosen before pyplot is imported. Writing files is the normal use and the
# script must work over ssh with no display; --show overrides it below.
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Leg order is FL, FR, RL, RR throughout the code base
# (QuadrupedKinematics::LegIndex). Foot state i occupies footholds[3i:3i+3].
LEG_NAMES = ["FL", "FR", "RL", "RR"]

TRUTH_STYLE = dict(color="blue", linestyle="-", linewidth=1.2, label="ground truth")
EST_STYLE = dict(color="red", linestyle="-", linewidth=1.0, label="estimate")

AXES = ["x", "y", "z"]


def load_log(path):
    """Returns a time vector in seconds plus {varname: (N, width) array}.

    rtclient::WriteML writes a MATLAB v5 *struct array* named `data`, one
    element per control cycle, each field a column vector. That is the transpose
    of the array-of-samples layout a plot wants, so this stacks it once here and
    everything downstream indexes by sample.

    Which fields exist depends on supervisor.log.vars, so nothing is assumed
    beyond `time`; a caller asks for what it needs through require().
    """
    raw = loadmat(path, squeeze_me=False, struct_as_record=True)

    rec = None
    for key, val in raw.items():
        if key.startswith("__"):
            continue
        # `params` is also present and carries only metadata. The data struct is
        # the one with a time field.
        if getattr(val, "dtype", None) is not None and val.dtype.names \
                and "time" in val.dtype.names:
            rec = val
            break
    if rec is None:
        raise SystemExit(f"{path}: no struct with a time field; is this a Supervisor log?")

    rec = rec.ravel()
    if rec.size == 0:
        raise SystemExit(f"{path}: log is empty. Did the run end before "
                         "supervisor.log.start?")

    out = {}
    for name in rec.dtype.names:
        # Each element is a (width, 1) array. concatenate along axis 1 and
        # transpose is markedly faster than stacking per sample, which matters:
        # a minute of logging at 1 kHz is 60k samples.
        out[name] = np.concatenate([np.atleast_2d(v) for v in rec[name]], axis=1).T

    # Already seconds, and already relative to when logging started rather than
    # to when the controller did: the offset sits in the file's `params` struct
    # as time_offset, in microseconds. Rebased to zero so --start and --stop read
    # off the plotted axis directly.
    t = out.pop("time")[:, 0].astype(float)
    return t - t[0], out


def require(data, name, width, what):
    if name not in data:
        raise SystemExit(
            f"missing '{name}', needed for {what}.\n"
            f"Add it to supervisor.log.vars and re-record; see README.md.\n"
            f"Present: {', '.join(sorted(data))}"
        )
    if data[name].shape[1] < width:
        raise SystemExit(
            f"'{name}' has {data[name].shape[1]} columns, expected at least "
            f"{width}. The log predates a change to that variable's layout."
        )
    return data[name]


def quat_to_rpy(q):
    """(N,4) w-first quaternion to (N,3) roll, pitch, yaw in ZYX order.

    Reproduces MdlOrientationEstimator::rpyFromQuat so the truth trace is built
    the same way the estimate is, and a difference between the two curves is a
    difference in attitude rather than in Euler convention.
    """
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Clipped for the same reason the C++ does it: a normalisation error of a
    # few ulp pushes the argument past one and arcsin returns NaN.
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.column_stack([roll, pitch, yaw])


def heading_leak(t, dyaw, v_truth):
    """Position error attributable to the drifted heading alone, (N,3).

    Gravity says nothing about heading, so the yaw gyroscope bias is
    unobservable and the estimated heading drifts. That drift is accepted; what
    it does to the *position* estimate is the reason for this view. The filter's
    kinematic velocity measurement is expressed through the estimated attitude,
    so a body moving at v is reconstructed as moving at R_z(dyaw) v. The
    difference integrates:

        leak(T) = integral of (R_z(dyaw(t)) - I) v_true(t) dt

    Note that this is an integral and not a rotation of the final error. For a
    straight run with dyaw growing linearly to dyaw_f, it comes to about
    dyaw_f * x_f / 2, which on a 0.1 rad drift over 0.64 m is 32 mm -- larger
    than everything else in the y error put together. Rotating the endpoint
    instead would have removed 0.1 * (a few mm), which is nothing, and would
    have left the whole leak sitting in the plot looking like translation error.
    """
    c, s = np.cos(dyaw), np.sin(dyaw)
    dvx = (c - 1.0) * v_truth[:, 0] - s * v_truth[:, 1]
    dvy = s * v_truth[:, 0] + (c - 1.0) * v_truth[:, 1]
    dt = np.gradient(t)
    return np.column_stack([np.cumsum(dvx * dt), np.cumsum(dvy * dt),
                            np.zeros_like(dvx)])


def heading_align(est, leak, dyaw, lever=None):
    """Removes the heading contribution from an estimate.

    The body carries the integrated leak. A foot carries that plus the static
    fan-out of its own lever arm, (R_z(dyaw) - I) p_rel, which is what splays
    the front and rear legs to opposite sides in the raw plots.

    What survives is the error heading drift does not explain -- the part a
    change to the position filter would actually move.
    """
    out = est - leak
    if lever is not None:
        c, s = np.cos(dyaw), np.sin(dyaw)
        out[:, 0] -= (c - 1.0) * lever[:, 0] - s * lever[:, 1]
        out[:, 1] -= s * lever[:, 0] + (c - 1.0) * lever[:, 1]
    return out


def triple_plot(t, truth, est, title, ylabels, outpath, show):
    """One figure, three stacked subplots, truth and estimate overlaid."""
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(3):
        axs[i].plot(t, truth[:, i], **TRUTH_STYLE)
        axs[i].plot(t, est[:, i], **EST_STYLE)
        axs[i].set_ylabel(ylabels[i])
        axs[i].grid(True, alpha=0.3)

        # Error as an annotation rather than a fourth panel: it keeps the three
        # subplots the user asked for while still answering "how far off is it".
        rms = float(np.sqrt(np.mean((est[:, i] - truth[:, i]) ** 2)))
        axs[i].text(
            0.995,
            0.04,
            f"rms {rms:.4g}",
            transform=axs[i].transAxes,
            ha="right",
            va="bottom",
            fontsize=8,
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.7", alpha=0.85),
        )

    axs[0].legend(loc="upper left", fontsize=9)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(outpath, dpi=140)
    print(f"wrote {outpath}")
    if not show:
        plt.close(fig)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", nargs="?",
                    default=os.path.join(here, "..", "..", "bin", "estrun.mat"),
                    help="Supervisor .mat log (default: bin/estrun.mat)")
    ap.add_argument("-o", "--outdir", default=here,
                    help="where to write the figures (default: this directory)")
    ap.add_argument("--start", type=float, default=0.0,
                    help="drop samples before this time [s]")
    ap.add_argument("--stop", type=float, default=None,
                    help="drop samples after this time [s]")
    ap.add_argument("--show", action="store_true",
                    help="also open the figures in a window")
    ap.add_argument("-a", "--heading-aligned", action="store_true",
                    help="rotate position errors out of the drifted heading "
                         "before plotting; see heading_align()")
    args = ap.parse_args()

    if args.show:
        matplotlib.use("TkAgg", force=True)

    if not os.path.exists(args.logfile):
        raise SystemExit(f"{args.logfile}: not found. See README.md for how to record one.")

    t, data = load_log(args.logfile)

    # Window before anything else, so the rms annotations describe the interval
    # actually plotted. The default log starts at supervisor.log.start, several
    # seconds before the robot stands, and that idle stretch flattens the scale
    # of every trace.
    keep = t >= args.start
    if args.stop is not None:
        keep &= t <= args.stop
    if not keep.any():
        raise SystemExit(f"no samples in [{args.start}, {args.stop}]; log spans "
                         f"0 to {t[-1]:.1f}s")
    t = t[keep]
    data = {k: v[keep] for k, v in data.items()}

    os.makedirs(args.outdir, exist_ok=True)
    out = lambda n: os.path.join(args.outdir, n)  # noqa: E731

    qpos = require(data, "MdlSimDriver_qpos", 7, "body position and orientation truth")
    qvel = require(data, "MdlSimDriver_qvel", 3, "body velocity truth")
    pv = require(data, "MdlPosVelEstimator_state", 6, "body position and velocity")
    ori = require(data, "MdlOrientationEstimator_state", 7, "estimated orientation")

    rpy_truth = quat_to_rpy(qpos[:, 3:7])
    if args.heading_aligned:
        # Wrapped, so a heading error that crosses pi does not read as 2*pi.
        dyaw = np.arctan2(np.sin(ori[:, 6] - rpy_truth[:, 2]),
                          np.cos(ori[:, 6] - rpy_truth[:, 2]))
        leak = heading_leak(t, dyaw, qvel[:, 0:3])
        align = lambda es, lever=None: heading_align(es, leak, dyaw, lever)  # noqa: E731
        suffix = "\n(heading aligned: the drifted yaw's contribution removed)"
    else:
        align = lambda es, lever=None: es  # noqa: E731
        suffix = ""

    # -- Body position. qpos[0:3] is the free joint origin, which is the same
    #    point the filter's r describes, so no offset correction is needed.
    triple_plot(t, qpos[:, 0:3], align(pv[:, 0:3]),
                "Body position: estimate vs ground truth" + suffix,
                ["x [m]", "y [m]", "z [m]"], out("body_position.png"), args.show)

    # -- Body velocity, world frame. The filter also carries a body frame
    #    velocity in columns 6:9; the world frame one is what qvel reports.
    triple_plot(t, qvel[:, 0:3], pv[:, 3:6],
                "Body velocity (world frame): estimate vs ground truth",
                ["vx [m/s]", "vy [m/s]", "vz [m/s]"], out("body_velocity.png"),
                args.show)

    # -- Body orientation.
    #
    # What this shows depends entirely on orientationestimator.attitude_source.
    # Under "imu" the two curves are the same numbers: in simulation the IMU
    # quaternion is qpos[3:7], the very array plotted as truth here, and the
    # module passes it through untouched. Do not read that agreement as the
    # estimator performing well; nothing is being tested. Under "filter" the
    # attitude is estimated from the rates and gravity and the curves are
    # genuinely independent -- expect roll and pitch to track within a couple of
    # milliradians and yaw to drift, since gravity cannot observe heading.
    #
    # Which of those two it is cannot be read from the log, so it is detected:
    # a passthrough agrees with truth to within the one-cycle logging skew.
    passthrough = np.max(np.abs(ori[:, 4:6] - rpy_truth[:, 0:2])) < 1e-3
    triple_plot(t, rpy_truth, ori[:, 4:7],
                "Body orientation: estimate vs ground truth\n"
                + ("(attitude_source = \"imu\": identical by construction, the IMU "
                   "quaternion is the simulator's own)" if passthrough else
                   "(attitude_source = \"filter\": estimated here, so these curves "
                   "can and do differ)"),
                ["roll [rad]", "pitch [rad]", "yaw [rad]"],
                out("body_orientation.png"), args.show)

    # -- Foot positions, one figure per leg.
    fh = require(data, "MdlPosVelEstimator_footholds", 12, "foot position estimates")
    ft = require(data, "MdlPosVelEstimator_foottruth", 12, "foot position truth")
    for leg, name in enumerate(LEG_NAMES):
        s = slice(3 * leg, 3 * leg + 3)
        triple_plot(t, ft[:, s], align(fh[:, s], ft[:, s] - qpos[:, 0:3]),
                    f"{name} foot position (world frame): estimate vs ground truth"
                    + suffix,
                    [f"{a} [m]" for a in AXES],
                    out(f"foot_position_{name}.png"), args.show)

    if args.show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
