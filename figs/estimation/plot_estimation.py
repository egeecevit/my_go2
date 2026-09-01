#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Plots the state estimate against simulator ground truth.

Reads one or two .mat files written by the Supervisor's logging thread and
produces one figure per estimated quantity for each log, each with an x, y and z
subplot. Ground truth is a dashed blue line drawn on top of the estimate's solid
vermillion one; the angular-velocity and gravity-reference figures add the
unprocessed sensor reading as a pale orange band beneath both. The palette and
line weights are the colourblind-safe set figs/mpc/plot_mpc.py uses.
With two logs, each log's filename appears in both its figure titles and output
filenames. With one log, the original unsuffixed output filenames are retained.

  body_position[_<LOG>].png      r,      against qpos[0:3]
  body_velocity[_<LOG>].png      v,      against qvel[0:3]
  body_orientation[_<LOG>].png   rpy,    against qpos[3:7] converted to rpy
  body_angular_velocity[_<LOG>].png  corrected and raw gyro against qvel[3:6]
  foot_position_<LEG>[_<LOG>].png  foothold p_i, against the simulator's foot
                                   geom, one figure per leg in FL, FR, RL, RR order

The last two need MdlOrientationEstimator_filter, which older logs predate. They
are skipped with a note if it is absent rather than being a hard requirement:

  gyro_bias[_<LOG>].png          the attitude filter's bias estimate against the
                                 injected true bias
  filtered_acceleration[_<LOG>].png  the low-passed gravity reference against the
                                 ideal one, with the raw specific force overlaid

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

# Okabe-Ito, the colourblind-safe pair figs/mpc/plot_mpc.py uses, and the same
# convention: the curve being compared *against* is dashed and drawn on top, the
# curve under test is solid beneath it. Two solid lines of near-equal weight,
# which is what this file drew before, cannot be told apart wherever they agree
# -- and agreeing is the normal case here, so the trace drawn second simply
# erased the one below it. Weights are down from 1.2/1.0 as well: at 1 kHz a
# 35 s log puts 35k samples on a 10 inch axis, and a 1.2 pt line fills the
# envelope solid.
TRUTH_STYLE = dict(color="#0072B2", linestyle="--", linewidth=1.0, zorder=3,
                   label="ground truth")
EST_STYLE = dict(color="#D55E00", linestyle="-", linewidth=0.9, zorder=2,
                 label="estimate")

# The unprocessed sensor goes underneath both as a wide pale band rather than a
# third line of the same weight. Raw gyro, corrected gyro and truth agree to
# about 5 mrad/s on a signal spanning 4 rad/s, so as an equal-weight line the
# raw trace was covered completely and appeared only in the legend. As a halo it
# stays visible where all three coincide and still separates where they do not.
RAW_GYRO_STYLE = dict(color="#E69F00", linestyle="-", linewidth=2.2, alpha=0.45,
                      zorder=1, label="raw gyro (reconstructed)")
CORRECTED_GYRO_STYLE = dict(color="#D55E00", linestyle="-", linewidth=0.9,
                            zorder=2, label="bias-corrected gyro")
# Same colour and layer as the raw gyro -- orange is the unprocessed sensor on
# both figures -- but thin rather than wide, and the reason is the opposite one.
# The raw specific force swings +/-8 m/s^2 about a gravity reference that moves
# by tenths, so it is in no danger of being hidden; drawn as a wide band it
# instead swamps the two traces the figure exists to compare.
RAW_ACCEL_STYLE = dict(color="#E69F00", linestyle="-", linewidth=0.8, alpha=0.35,
                       zorder=1, label="raw specific force")

AXES = ["x", "y", "z"]

# Magnitude of gravity, matching orientationestimator.gravity_magnitude and
# posvelestimator.gravity in config/default/stateestimator.toml. Only used to
# build the ideal gravity reference the filtered accelerometer is compared with.
GRAVITY = 9.81


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


def quat_rotate(q, v, inverse=False):
    """Rotates (N,3) vectors by (N,4) w-first quaternions; (3,) v broadcasts.

    q is body to world throughout this code base, so `inverse=True` is the world
    to body direction -- the one both accelerometer traces need, since the
    specific force is a body frame quantity.

    Written out as v + 2w(u x v) + 2u x (u x v) rather than by building rotation
    matrices, so a minute of logging at 1 kHz stays a handful of array
    operations instead of 60k small matmuls.
    """
    v = np.broadcast_to(np.asarray(v, dtype=float), (q.shape[0], 3))
    w = q[:, 0:1]
    u = q[:, 1:4]
    if inverse:
        u = -u
    uxv = np.cross(u, v)
    return v + 2.0 * (w * uxv + np.cross(u, uxv))


def wrapped_angle_error(est, truth):
    """Shortest signed Euler-angle difference, component by component."""
    delta = est - truth
    return np.arctan2(np.sin(delta), np.cos(delta))


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


def _annotate(ax, lines, loc=(0.995, 0.04)):
    """Small boxed text in a subplot corner. Shared by all four figure kinds."""
    ax.text(
        loc[0], loc[1], "\n".join(lines),
        transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.7", alpha=0.85),
    )


def triple_plot(t, truth, est, title, ylabels, outpath, show, dpi, error_fn=None):
    """One figure, three stacked subplots, truth and estimate overlaid."""
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    error = est - truth if error_fn is None else error_fn(est, truth)

    for i in range(3):
        axs[i].plot(t, truth[:, i], **TRUTH_STYLE)
        axs[i].plot(t, est[:, i], **EST_STYLE)
        axs[i].set_ylabel(ylabels[i])
        axs[i].grid(True, alpha=0.3)

        # Error as an annotation rather than a fourth panel: it keeps the three
        # subplots the user asked for while still answering "how far off is it".
        rms = float(np.sqrt(np.mean(error[:, i] ** 2)))
        _annotate(axs[i], [f"rms {rms:.4g}"])

    axs[0].legend(loc="upper left", fontsize=9)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(outpath, dpi=dpi)
    print(f"wrote {outpath}")
    if not show:
        plt.close(fig)


def body_rate_series(t, qvel, ori):
    """Returns time-aligned body-rate truth, raw gyro and corrected gyro.

    MdlSimDriver runs after MdlOrientationEstimator. Consequently qvel at log
    index i is one control sample newer than the IMU reading represented by the
    orientation state at index i. Comparing ori[i] with qvel[i-1] removes that
    deterministic logger skew. The estimator logs corrected gyro and its bias
    estimate separately, so their sum reconstructs the raw gyro exactly.
    """
    if len(t) < 2:
        raise SystemExit("at least two samples are needed for angular velocity")
    corrected = ori[1:, 7:10]
    raw = corrected + ori[1:, 13:16]
    return t[1:], qvel[:-1, 3:6], raw, corrected


def angular_velocity_plot(t, truth, raw, corrected, title, outpath, show, dpi):
    """Body angular velocity with raw and corrected gyro RMSE per axis."""
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(3):
        axs[i].plot(t, truth[:, i], **TRUTH_STYLE)
        axs[i].plot(t, raw[:, i], **RAW_GYRO_STYLE)
        axs[i].plot(t, corrected[:, i], **CORRECTED_GYRO_STYLE)
        axs[i].set_ylabel(f"omega_{AXES[i]} [rad/s]")
        axs[i].grid(True, alpha=0.3)

        raw_rms = float(np.sqrt(np.mean((raw[:, i] - truth[:, i]) ** 2)))
        corrected_rms = float(np.sqrt(np.mean((corrected[:, i] - truth[:, i]) ** 2)))
        _annotate(axs[i], [f"raw rms {raw_rms:.4g}",
                           f"corrected rms {corrected_rms:.4g}"])

    axs[0].legend(loc="upper left", fontsize=9)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(outpath, dpi=dpi)
    print(f"wrote {outpath}")
    if not show:
        plt.close(fig)


def filtered_acceleration_plot(t, ideal, raw, filtered, title, outpath, show, dpi):
    """The attitude filter's gravity reference against the ideal one.

    Three traces per axis: the ideal body-frame gravity from ground truth, the
    raw specific force the accelerometer actually reports, and `_accFilt`, the
    low-passed version the Mahony correction is built from. The gap between
    orange and red is exactly what accel_filter_tau removes.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(3):
        axs[i].plot(t, ideal[:, i], **TRUTH_STYLE)
        axs[i].plot(t, raw[:, i], **RAW_ACCEL_STYLE)
        axs[i].plot(t, filtered[:, i], **EST_STYLE)
        axs[i].set_ylabel(f"a_{AXES[i]} [m/s^2]")
        axs[i].grid(True, alpha=0.3)

        raw_rms = float(np.sqrt(np.mean((raw[:, i] - ideal[:, i]) ** 2)))
        filt_rms = float(np.sqrt(np.mean((filtered[:, i] - ideal[:, i]) ** 2)))
        _annotate(axs[i], [f"raw rms {raw_rms:.4g}",
                           f"filtered rms {filt_rms:.4g}"])

    axs[0].legend(loc="upper left", fontsize=9)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(outpath, dpi=dpi)
    print(f"wrote {outpath}")
    if not show:
        plt.close(fig)


def reference_tilt(filtered, ideal):
    """Angle between the filtered gravity reference and the true one [rad].

    This is rho, the attitude error the gravity reference itself asserts, and it
    is the whole reason accel_filter_tau exists. The Mahony loop can do no better
    than its reference: with mahony_ki > 0 the estimate settles at exactly the
    mean of this signal, so its DC is the estimator's steady tilt error and its
    oscillation is not. See reports/mahony_error_decomposition.md.
    """
    cross = np.linalg.norm(np.cross(filtered, ideal), axis=1)
    dot = np.sum(filtered * ideal, axis=1)
    return np.arctan2(cross, dot)


def logfile_tag(path):
    """Returns a filesystem-friendly label based on a log's filename."""
    stem = os.path.splitext(os.path.basename(path))[0]
    tag = "".join(c if c.isalnum() or c in "-_." else "_" for c in stem)
    return tag or "log"


def unique_log_tags(paths):
    """Returns stable output tags, disambiguating equal/sanitized basenames."""
    base_tags = [logfile_tag(path) for path in paths]
    totals = {tag: base_tags.count(tag) for tag in base_tags}
    seen = {}
    tags = []
    for tag in base_tags:
        seen[tag] = seen.get(tag, 0) + 1
        tags.append(f"{tag}_{seen[tag]}" if totals[tag] > 1 else tag)
    return tags


def plot_log(logfile, log_tag, outdir, start, stop, show, dpi, heading_aligned):
    """Loads one log and writes its complete set of estimator figures."""
    t, data = load_log(logfile)

    # Window before anything else, so the rms annotations describe the interval
    # actually plotted. The default log starts at supervisor.log.start, several
    # seconds before the robot stands, and that idle stretch flattens the scale
    # of every trace.
    keep = t >= start
    if stop is not None:
        keep &= t <= stop
    if not keep.any():
        raise SystemExit(f"{logfile}: no samples in [{start}, {stop}]; log spans "
                         f"0 to {t[-1]:.1f}s")
    t = t[keep]
    data = {k: v[keep] for k, v in data.items()}

    filename_suffix = f"_{log_tag}" if log_tag else ""
    out = lambda n: os.path.join(outdir, f"{n}{filename_suffix}.png")  # noqa: E731
    log_title = f"\nLog: {os.path.basename(logfile)}" if log_tag else ""

    qpos = require(data, "MdlSimDriver_qpos", 7, "body position and orientation truth")
    qvel = require(data, "MdlSimDriver_qvel", 6, "body velocity and angular velocity truth")
    pv = require(data, "MdlPosVelEstimator_state", 6, "body position and velocity")
    ori = require(data, "MdlOrientationEstimator_state", 16,
                  "estimated orientation, angular velocity and gyro bias")

    rpy_truth = quat_to_rpy(qpos[:, 3:7])
    if heading_aligned:
        # Wrapped, so a heading error that crosses pi does not read as 2*pi.
        dyaw = np.arctan2(np.sin(ori[:, 6] - rpy_truth[:, 2]),
                          np.cos(ori[:, 6] - rpy_truth[:, 2]))
        leak = heading_leak(t, dyaw, qvel[:, 0:3])
        align = lambda es, lever=None: heading_align(es, leak, dyaw, lever)  # noqa: E731
        position_suffix = "\n(heading aligned: the drifted yaw's contribution removed)"
    else:
        align = lambda es, lever=None: es  # noqa: E731
        position_suffix = ""

    # -- Body position. qpos[0:3] is the free joint origin, which is the same
    #    point the filter's r describes, so no offset correction is needed.
    triple_plot(t, qpos[:, 0:3], align(pv[:, 0:3]),
                "Body position: estimate vs ground truth" + position_suffix + log_title,
                ["x [m]", "y [m]", "z [m]"], out("body_position"), show, dpi)

    # -- Body velocity, world frame. The filter also carries a body frame
    #    velocity in columns 6:9; the world frame one is what qvel reports.
    triple_plot(t, qvel[:, 0:3], pv[:, 3:6],
                "Body velocity (world frame): estimate vs ground truth" + log_title,
                ["vx [m/s]", "vy [m/s]", "vz [m/s]"], out("body_velocity"),
                show, dpi)

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
                   "can and do differ)") + log_title,
                ["roll [rad]", "pitch [rad]", "yaw [rad]"],
                out("body_orientation"), show, dpi, wrapped_angle_error)

    # -- Body angular velocity. Both qvel[3:6] and the gyroscope use the body
    #    frame. The proportional Mahony term corrects quaternion propagation but
    #    is not a physical rate; the published rate therefore subtracts only the
    #    integral bias estimate. Plotting raw and corrected gyro separately makes
    #    it visible whether that bias estimate actually improves the measurement.
    rate_t, rate_truth, raw_gyro, corrected_gyro = body_rate_series(t, qvel, ori)
    angular_velocity_plot(
        rate_t, rate_truth, raw_gyro, corrected_gyro,
        "Body angular velocity (body frame): gyro vs ground truth\n"
        "(ground truth shifted by one sample to match estimator logging)" + log_title,
        out("body_angular_velocity"), show, dpi)

    # -- Attitude filter internals. Optional: MdlOrientationEstimator_filter was
    #    added after these plots existed, so any log recorded before it is
    #    complete without these two figures and must still plot the rest.
    #    Deliberately not require(), which exits.
    if "MdlOrientationEstimator_filter" not in data:
        print("note: 'MdlOrientationEstimator_filter' is not in this log; "
              "skipping gyro_bias and filtered_acceleration. Add it to "
              "supervisor.log.vars and re-record to get them.")
    else:
        filt = require(data, "MdlOrientationEstimator_filter", 10,
                       "the attitude filter's correction, gain, filtered "
                       "acceleration and injected true gyro bias")

        # -- Gyro bias. The only figure here whose "truth" is injected rather
        #    than physical: filt[7:10] is the synthetic bias simnoise added to
        #    the simulator's clean gyro, so this measures the estimator against
        #    the exact quantity it is trying to find. Note it is not constant --
        #    gyro_bias_walk keeps it wandering for the whole run, which is why it
        #    is plotted as a trace and not quoted as a number.
        #
        #    With mahony_ki = 0 the red trace is identically zero by
        #    construction: the ki line is the only write to _gyroBias. That flat
        #    line is the correct output, not a broken plot.
        triple_plot(t, filt[:, 7:10], ori[:, 13:16],
                    "Gyroscope bias: attitude filter estimate vs injected truth"
                    + log_title,
                    [f"b_{a} [rad/s]" for a in AXES], out("gyro_bias"), show, dpi)

        # -- The gravity reference. The Mahony correction is built from
        #    _accFilt, so how well it can possibly do is bounded by how far that
        #    vector sits from true gravity; the angle between the red and blue
        #    traces is rho, quoted in the title.
        #
        #    Ideal reference: gravity carried into the true body frame. The raw
        #    overlay is reconstructed rather than logged -- aWorld is exactly
        #    R_est * aBody, so rotating it back by the estimate quaternion
        #    recovers the accelerometer reading bit for bit.
        ideal_acc = quat_rotate(qpos[:, 3:7], [0.0, 0.0, GRAVITY], inverse=True)
        raw_acc = quat_rotate(ori[:, 0:4], ori[:, 10:13], inverse=True)
        rho = reference_tilt(filt[:, 4:7], ideal_acc)
        filtered_acceleration_plot(
            t, ideal_acc, raw_acc, filt[:, 4:7],
            "Gravity reference (body frame): _accFilt vs true gravity\n"
            f"(reference tilt rho: mean {1e3 * float(np.mean(rho)):.2f} mrad, "
            f"rms {1e3 * float(np.sqrt(np.mean(rho ** 2))):.2f} mrad)" + log_title,
            out("filtered_acceleration"), show, dpi)

    # -- Foot positions, one figure per leg.
    fh = require(data, "MdlPosVelEstimator_footholds", 12, "foot position estimates")
    ft = require(data, "MdlPosVelEstimator_foottruth", 12, "foot position truth")
    for leg, name in enumerate(LEG_NAMES):
        s = slice(3 * leg, 3 * leg + 3)
        triple_plot(t, ft[:, s], align(fh[:, s], ft[:, s] - qpos[:, 0:3]),
                    f"{name} foot position (world frame): estimate vs ground truth"
                    + position_suffix + log_title,
                    [f"{a} [m]" for a in AXES],
                    out(f"foot_position_{name}"), show, dpi)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, "..", ".."))
    default_logfile = os.path.join(repo_root, "bin", "estrun.mat")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", nargs="?", metavar="LOGFILE1",
                    help="Supervisor .mat log to plot (default: bin/estrun.mat)")
    ap.add_argument("logfile2", nargs="?", metavar="LOGFILE2",
                    help="optional second Supervisor .mat log to plot")
    ap.add_argument("-o", "--outdir", default=here,
                    help="where to write the figures (default: this directory)")
    ap.add_argument("--start", type=float, default=0.0,
                    help="drop samples before this time [s] in both logs")
    ap.add_argument("--stop", type=float, default=None,
                    help="drop samples after this time [s] in both logs")
    ap.add_argument("--show", action="store_true",
                    help="also open the figures in a window")
    ap.add_argument("--dpi", type=int, default=200,
                    help="figure resolution in dots per inch (default: 200)")
    ap.add_argument("-a", "--heading-aligned", action="store_true",
                    help="rotate position errors out of the drifted heading "
                         "before plotting; see heading_align()")
    args = ap.parse_args()

    logfiles = [path for path in (args.logfile, args.logfile2) if path is not None]
    if not logfiles:
        logfiles = [default_logfile]

    if args.show:
        matplotlib.use("TkAgg", force=True)

    resolved_logfiles = []
    for logfile in logfiles:
        candidates = [logfile]
        if not os.path.isabs(logfile):
            candidates.append(os.path.join(repo_root, logfile))
        resolved = next((path for path in candidates if os.path.exists(path)), None)
        if resolved is None:
            checked = " and ".join(os.path.abspath(path) for path in candidates)
            raise SystemExit(f"{logfile}: not found (checked {checked}). "
                             "See README.md for how to record one.")
        resolved_logfiles.append(os.path.abspath(resolved))

    os.makedirs(args.outdir, exist_ok=True)
    log_tags = unique_log_tags(resolved_logfiles) if len(resolved_logfiles) == 2 \
        else [None]
    for logfile, log_tag in zip(resolved_logfiles, log_tags):
        plot_log(logfile, log_tag, args.outdir, args.start, args.stop,
                 args.show, args.dpi, args.heading_aligned)

    if args.show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
