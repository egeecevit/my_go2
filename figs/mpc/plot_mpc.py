#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Plots MdlTrot / MPC tracking and torque saturation against a Supervisor log.

Reads one or two .mat files written by the Supervisor's logging thread and
produces the following figures for each log. With two logs, each log's
filename appears in both its figure titles and output filenames. With one
log, the original unsuffixed output filenames are retained.

  torque_<LEG>[_<LOG>].png             requested vs applied motor torque, one
                                        figure per leg (abduction, hip, knee),
                                        with the compile-time torque limit
  body_velocity_tracking[_<LOG>].png   commanded vs actual body linear
                                        velocity, world frame
  body_angular_velocity_tracking[_<LOG>].png
                                        commanded vs actual body angular
                                        velocity, body frame
  body_reference[_<LOG>].png           desired vs measured body position and
                                        yaw, with the yaw clamp band shaded
  foot_reference_<LEG>[_<LOG>].png     commanded foot position (body frame)
                                        vs the measured foot rotated into the
                                        body frame, one figure per leg, with
                                        scheduled swing shaded

A log missing a variable is not fatal. When only the commanded half of a
pair is absent -- MdlTrot_torquereq, MdlTrot_bodyref, MdlTrot_footref -- the
figure is still written from the measured half alone, and the note names
exactly which variable was missing rather than every variable the figure
could use.

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
# (QuadrupedKinematics::LegIndex). Joints are abduction, hip, knee.
LEG_NAMES = ["FL", "FR", "RL", "RR"]
JOINT_NAMES = ["abduction", "hip", "knee"]
AXES = ["x", "y", "z"]

# Per-joint torque limit, N*m. Compile-time constants from
# createGo2DynamicsConfig() in src/quadruped/QuadrupedConfigs.cc, applied in
# MdlLegControl.cc:63. Not logged, so hard-coded here rather than spent out of
# the 128-double log budget.
TORQUE_LIMIT = (23.7, 23.7, 45.43)

# Controller joint sign per leg, from kDirection in
# src/hardware/mujocohw/tests/test_go2_leg_dynamics.cc. MdlTrot_torquereq is
# logged in the controller's convention, straight out of command_result_t,
# while MdlSimDriver_ctrl is what the simulator applies -- and the two differ
# in the sign of the abduction joint on the right-hand legs. Comparing them
# without this makes FR and RR abduction look like mirror images of each other
# (rms 7.6 N*m against 0.01 on every other motor), which is a plotting artefact
# and not a controller fault. Applied torque is mapped into the controller
# convention below so both traces mean the same thing.
JOINT_DIRECTION = ((1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
                   (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0))

# Colourblind-safe, thin lines: this plot set is about distinguishing a
# commanded/requested trace from an actual/applied one at a glance, not about
# the estimator trio's truth-vs-estimate convention, so it gets its own
# palette per the house style requested for this tool.
CMD_STYLE = dict(color="#0072B2", linestyle="--", linewidth=0.9, label="commanded")
ACT_STYLE = dict(color="#D55E00", linestyle="-", linewidth=0.9, label="actual")
REQ_STYLE = dict(color="#0072B2", linestyle="--", linewidth=0.9, label="requested")
APP_STYLE = dict(color="#D55E00", linestyle="-", linewidth=0.9, label="applied")
LIMIT_STYLE = dict(color="#999999", linestyle=":", linewidth=0.8)
SWING_SHADE = dict(color="grey", alpha=0.07, linewidth=0)


def load_log(path):
    """Returns a time vector in seconds plus {varname: (N, width) array}.

    Copied verbatim from figs/estimation/plot_estimation.py: rtclient::WriteML
    writes a MATLAB v5 *struct array* named `data`, one element per control
    cycle, each field a column vector. That is the transpose of the
    array-of-samples layout a plot wants, so this stacks it once here and
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
    """Copied verbatim from figs/estimation/plot_estimation.py."""
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


def missing_note(data, *names):
    """"'X' is not in this log" for the absent members of names, or None.

    Only the names actually absent are reported: a note reading "'A', 'B' or
    'C' is not in this log" sends the reader hunting through three variables
    when one of them is the problem.
    """
    absent = [f"'{n}'" for n in names if n not in data]
    if not absent:
        return None
    if len(absent) == 1:
        return f"{absent[0]} is not in this log"
    return ", ".join(absent[:-1]) + f" and {absent[-1]} are not in this log"


def optional(data, name, width, what, note):
    """require() for a variable a figure can do without; None if absent."""
    if name not in data:
        print(f"note: '{name}' is not in this log; {note}")
        return None
    return require(data, name, width, what)


def gait_contact(data):
    """(contact signal, what it means) for swing shading; (None, None) if neither.

    MdlTrot_contact is the gait schedule, which is what the shading is meant
    to show. Failing that, MdlPosVelEstimator_contacts carries the
    estimator's contact trust ramp over the same footfalls -- close enough to
    read gait phase off, but a different quantity, so its name is carried
    along and printed on the figure rather than passing it off as the
    schedule.
    """
    if "MdlTrot_contact" in data:
        return require(data, "MdlTrot_contact", 4, "gait contact schedule"), \
            "scheduled swing"
    if "MdlPosVelEstimator_contacts" in data:
        return require(data, "MdlPosVelEstimator_contacts", 4,
                       "estimator contact trust"), "estimator contact trust < 0.5"
    return None, None


def quat_yaw(q):
    """Yaw about +Z from (N,4) w-first quaternions, radians.

    Only needed when MdlTrot_bodyref is absent: bodyref[8] is the measured
    yaw the controller itself saw, and is preferred when it is there.
    """
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_rotate(q, v, inverse=False):
    """Rotates (N,3) vectors by (N,4) w-first quaternions; (3,) v broadcasts.

    Copied verbatim from figs/estimation/plot_estimation.py. q is body to
    world throughout this code base, so `inverse=True` is the world to body
    direction, the one the foot-reference figure needs to bring the world
    frame foot truth into the body frame footref lives in.

    Written out as v + 2w(u x v) + 2u x (u x v) rather than by building
    rotation matrices, so a minute of logging at 1 kHz stays a handful of
    array operations instead of 60k small matmuls.
    """
    v = np.broadcast_to(np.asarray(v, dtype=float), (q.shape[0], 3))
    w = q[:, 0:1]
    u = q[:, 1:4]
    if inverse:
        u = -u
    uxv = np.cross(u, v)
    return v + 2.0 * (w * uxv + np.cross(u, uxv))


def logfile_tag(path):
    """Copied verbatim from figs/estimation/plot_estimation.py."""
    stem = os.path.splitext(os.path.basename(path))[0]
    tag = "".join(c if c.isalnum() or c in "-_." else "_" for c in stem)
    return tag or "log"


def unique_log_tags(paths):
    """Copied verbatim from figs/estimation/plot_estimation.py."""
    base_tags = [logfile_tag(path) for path in paths]
    totals = {tag: base_tags.count(tag) for tag in base_tags}
    seen = {}
    tags = []
    for tag in base_tags:
        seen[tag] = seen.get(tag, 0) + 1
        tags.append(f"{tag}_{seen[tag]}" if totals[tag] > 1 else tag)
    return tags


def _annotate(ax, lines, loc=(0.995, 0.04)):
    """Small boxed text in a subplot corner, matching triple_plot's style."""
    ax.text(
        loc[0], loc[1], "\n".join(lines),
        transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.7", alpha=0.85),
    )


def _shade_swing(ax, t, contact):
    """Shades intervals where the gait schedule says this foot is airborne.

    contact is the SCHEDULED stance signal (MdlTrot_contact), not measured
    ground contact, so this reads gait phase, not actual touchdown -- useful
    for judging tracking lag against where in the step the foot is supposed
    to be. None when the log carries no contact signal at all, in which case
    nothing is shaded.
    """
    if contact is None:
        return
    swing = contact < 0.5
    if not swing.any():
        return
    # Turn the boolean mask into contiguous [start, end) runs so axvspan is
    # called once per swing phase instead of once per sample.
    edges = np.flatnonzero(np.diff(swing.astype(int)))
    starts = [0] if swing[0] else []
    starts += [e + 1 for e in edges if swing[e + 1]]
    ends = [e + 1 for e in edges if not swing[e + 1]]
    if swing[-1]:
        ends.append(len(t))
    for s, e in zip(starts, ends):
        ax.axvspan(t[s], t[min(e, len(t) - 1)], **SWING_SHADE)


def torque_plot(t, req, app, contact, leg_name, title, outpath, show, dpi):
    """One figure per leg: requested vs applied torque, 3 stacked subplots.

    Requested is pre-saturation (MdlTrot_torquereq); applied is what
    MdlLegControl actually sent (MdlSimDriver_ctrl). The gap between them,
    quantified in the annotation, is the whole reason the requested signal is
    logged at all -- it is invisible from the applied trace alone, which is
    clipped at the limit by construction.

    req is None when the log has no MdlTrot_torquereq: the figure then shows
    the applied trace alone and the annotation counts samples sitting on the
    limit, which is all the saturation a clipped signal can still reveal.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(3):
        limit = TORQUE_LIMIT[i]
        _shade_swing(axs[i], t, contact)
        if req is not None:
            axs[i].plot(t, req[:, i], **REQ_STYLE)
        axs[i].plot(t, app[:, i], **APP_STYLE)
        axs[i].axhline(limit, **LIMIT_STYLE)
        axs[i].axhline(-limit, **LIMIT_STYLE)
        axs[i].set_ylabel(f"{JOINT_NAMES[i]} tau [N*m]")
        axs[i].grid(True, alpha=0.3)

        if req is not None:
            rms = float(np.sqrt(np.mean((req[:, i] - app[:, i]) ** 2)))
            over_frac = float(np.mean(np.abs(req[:, i]) > limit))
            max_pct = 100.0 * float(np.max(np.abs(req[:, i]))) / limit
            _annotate(axs[i], [
                f"rms(req-app) {rms:.4g}",
                f"|req|>limit: {100.0 * over_frac:.1f}% of samples",
                f"max|req|: {max_pct:.1f}% of limit",
            ])
        else:
            # Equality against the clip value, not a > test: a saturated
            # sample lands exactly on the limit, so a strict inequality
            # counts nothing.
            at_limit = float(np.mean(np.abs(app[:, i]) >= limit - 1e-9))
            max_pct = 100.0 * float(np.max(np.abs(app[:, i]))) / limit
            _annotate(axs[i], [
                f"|app| at limit: {100.0 * at_limit:.1f}% of samples",
                f"max|app|: {max_pct:.1f}% of limit",
            ])

    axs[0].legend(loc="upper left", fontsize=9)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(outpath, dpi=dpi)
    print(f"wrote {outpath}")
    if not show:
        plt.close(fig)


def triple_plot(t, cmd, act, title, ylabels, outpath, show, dpi, shade_contact=None):
    """One figure, three stacked subplots, commanded and actual overlaid.

    Structurally figs/estimation's triple_plot with the truth/estimate
    styling swapped for this tool's commanded/actual palette, and an optional
    per-subplot swing shading for the foot-reference figures.

    cmd is None when the log carries no command for this quantity; the actual
    trace is then plotted on its own, annotated with its mean and rms since
    there is no error to report.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(3):
        _shade_swing(axs[i], t, shade_contact)
        if cmd is not None:
            axs[i].plot(t, cmd[:, i], **CMD_STYLE)
        axs[i].plot(t, act[:, i], **ACT_STYLE)
        axs[i].set_ylabel(ylabels[i])
        axs[i].grid(True, alpha=0.3)

        if cmd is not None:
            rms = float(np.sqrt(np.mean((act[:, i] - cmd[:, i]) ** 2)))
            _annotate(axs[i], [f"rms {rms:.4g}"])
        else:
            _annotate(axs[i], [f"mean {float(np.mean(act[:, i])):.4g}",
                               f"rms {float(np.sqrt(np.mean(act[:, i] ** 2))):.4g}"])

    axs[0].legend(loc="upper left", fontsize=9)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(outpath, dpi=dpi)
    print(f"wrote {outpath}")
    if not show:
        plt.close(fig)


def body_reference_plot(t, des_pos, meas_pos, des_yaw, meas_yaw, clamp, title,
                        outpath, show, dpi):
    """4 subplots: desired vs measured body x, y, z and yaw.

    The yaw panel additionally shades measured +/- clamp, the band
    bodyref[11] actually enforces, so it is visible on the plot whether the
    controller's yaw command is being clamped rather than having to cross
    -reference against the config.

    des_pos, des_yaw and clamp are all None when the log has no
    MdlTrot_bodyref: the measured position and yaw are then plotted alone,
    with no clamp band to draw.
    """
    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    labels = ["x [m]", "y [m]", "z [m]"]

    for i in range(3):
        if des_pos is not None:
            axs[i].plot(t, des_pos[:, i], **CMD_STYLE)
        axs[i].plot(t, meas_pos[:, i], **ACT_STYLE)
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True, alpha=0.3)
        if des_pos is not None:
            rms = float(np.sqrt(np.mean((meas_pos[:, i] - des_pos[:, i]) ** 2)))
            _annotate(axs[i], [f"rms {rms:.4g}"])
        else:
            _annotate(axs[i], [f"mean {float(np.mean(meas_pos[:, i])):.4g}"])

    ax = axs[3]
    if clamp is not None:
        ax.fill_between(t, meas_yaw - clamp, meas_yaw + clamp, color="grey",
                        alpha=0.15, linewidth=0, label="clamp band")
    if des_yaw is not None:
        ax.plot(t, des_yaw, **CMD_STYLE)
    ax.plot(t, meas_yaw, **ACT_STYLE)
    ax.set_ylabel("yaw [rad]")
    ax.grid(True, alpha=0.3)
    if des_yaw is not None:
        yaw_err = meas_yaw - des_yaw
        rms = float(np.sqrt(np.mean(yaw_err ** 2)))
        inside = float(np.mean(np.abs(yaw_err) <= clamp))
        _annotate(ax, [f"rms {rms:.4g}", f"inside clamp: {100.0 * inside:.1f}%"])
        ax.legend(loc="upper left", fontsize=9)
    else:
        _annotate(ax, [f"drift {float(meas_yaw[-1] - meas_yaw[0]):.4g} rad"])

    axs[0].legend(loc="upper left", fontsize=9)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(outpath, dpi=dpi)
    print(f"wrote {outpath}")
    if not show:
        plt.close(fig)


def plot_log(logfile, log_tag, outdir, start, stop, show, dpi):
    """Loads one log and writes its complete set of MPC/trot figures."""
    t, data = load_log(logfile)

    # Window before anything else, so the rms annotations describe the
    # interval actually plotted, same as figs/estimation/plot_estimation.py.
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

    # Which variables the log carries decides how much of each figure can be
    # drawn. A missing command leaves the measured trace, which is still worth
    # plotting; only a missing measurement kills the figure outright.
    contact, contact_kind = gait_contact(data)
    swing_note = f"\n(shaded: {contact_kind})" if contact is not None else ""

    # -- Torque, one figure per leg. Guarded independently of everything
    #    below: an older log missing MdlTrot_torquereq should still get the
    #    tracking figures.
    note = missing_note(data, "MdlSimDriver_ctrl")
    if note:
        print(f"note: {note}; skipping torque_<LEG>. Add what is missing "
              "to supervisor.log.vars and re-record.")
    else:
        app = require(data, "MdlSimDriver_ctrl", 12, "applied motor torque")
        req = optional(data, "MdlTrot_torquereq", 12, "requested motor torque",
                       "torque_<LEG> shows the applied torque alone, without "
                       "the pre-saturation request.")
        what = "requested vs applied" if req is not None else "applied"
        for leg, name in enumerate(LEG_NAMES):
            s = slice(3 * leg, 3 * leg + 3)
            direction = np.asarray(JOINT_DIRECTION[leg])
            torque_plot(t, None if req is None else req[:, s],
                       app[:, s] * direction,
                       None if contact is None else contact[:, leg],
                       name, f"{name} motor torque: {what}" + swing_note + log_title,
                       out(f"torque_{name}"), show, dpi)

    # -- Body velocity tracking, world frame.
    note = missing_note(data, "MdlSimDriver_qvel")
    if note:
        print(f"note: {note}; skipping body_velocity_tracking and "
              "body_angular_velocity_tracking.")
    else:
        qvel = require(data, "MdlSimDriver_qvel", 6, "body velocity truth")
        bodyref = optional(data, "MdlTrot_bodyref", 12, "body reference command",
                           "body_velocity_tracking, "
                           "body_angular_velocity_tracking and body_reference "
                           "show the measured signals alone, with no "
                           "commanded trace.")
        what = "commanded vs actual" if bodyref is not None else "actual"

        triple_plot(t, None if bodyref is None else bodyref[:, 4:7], qvel[:, 0:3],
                   f"Body linear velocity (world frame): {what}" + log_title,
                   ["vx [m/s]", "vy [m/s]", "vz [m/s]"],
                   out("body_velocity_tracking"), show, dpi)

        # Roll and pitch rate are always commanded to zero; only yaw rate is
        # a real setpoint (MdlTrot_bodyref[7]).
        cmd_rate = None
        if bodyref is not None:
            cmd_rate = np.zeros((len(t), 3))
            cmd_rate[:, 2] = bodyref[:, 7]
        triple_plot(t, cmd_rate, qvel[:, 3:6],
                   f"Body angular velocity (body frame): {what}\n"
                   "(roll and pitch rate are always commanded to zero)"
                   + log_title,
                   ["roll rate [rad/s]", "pitch rate [rad/s]", "yaw rate [rad/s]"],
                   out("body_angular_velocity_tracking"), show, dpi)

    # -- Body reference: desired position/yaw vs measured.
    note = missing_note(data, "MdlSimDriver_qpos")
    if note:
        print(f"note: {note}; skipping body_reference.")
    elif "MdlTrot_bodyref" in data:
        bodyref = require(data, "MdlTrot_bodyref", 12, "body reference command")
        qpos = require(data, "MdlSimDriver_qpos", 3, "body position truth")
        body_reference_plot(
            t, bodyref[:, 0:3], qpos[:, 0:3], bodyref[:, 3], bodyref[:, 8],
            bodyref[:, 11],
            "Body reference: desired vs measured position and yaw" + log_title,
            out("body_reference"), show, dpi)
    else:
        # The note was already printed with the velocity figures above.
        qpos = require(data, "MdlSimDriver_qpos", 7,
                       "body position and orientation")
        body_reference_plot(
            t, None, qpos[:, 0:3], None, quat_yaw(qpos[:, 3:7]), None,
            "Body pose: measured position and yaw" + log_title,
            out("body_reference"), show, dpi)

    # -- Foot reference, one figure per leg.
    note = missing_note(data, "MdlPosVelEstimator_foottruth", "MdlSimDriver_qpos")
    if note:
        print(f"note: {note}; skipping foot_reference_<LEG>. Add what is "
              "missing to supervisor.log.vars and re-record.")
    else:
        foottruth = require(data, "MdlPosVelEstimator_foottruth", 12,
                            "measured foot position, world frame")
        qpos = require(data, "MdlSimDriver_qpos", 7, "body position and orientation")
        footref = optional(data, "MdlTrot_footref", 36, "commanded foot state",
                           "foot_reference_<LEG> shows the measured foot "
                           "position alone, with no commanded trace.")
        what = "commanded vs measured" if footref is not None else "measured"

        # World-frame foot position relative to the base, rotated into the
        # body frame with R_bw = R_wb^T, so it lives in the same frame as
        # footref without materialising a (N,3,3) rotation matrix per sample.
        base_pos = qpos[:, 0:3]
        for leg, name in enumerate(LEG_NAMES):
            rel_world = foottruth[:, 3 * leg:3 * leg + 3] - base_pos
            meas_body = quat_rotate(qpos[:, 3:7], rel_world, inverse=True)
            cmd_body = None if footref is None else footref[:, 9 * leg:9 * leg + 3]
            triple_plot(t, cmd_body, meas_body,
                       f"{name} foot position (body frame): {what}"
                       + swing_note + log_title,
                       [f"{a} [m]" for a in AXES],
                       out(f"foot_reference_{name}"), show, dpi,
                       shade_contact=None if contact is None else contact[:, leg])


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, "..", ".."))
    default_logfile = os.path.join(repo_root, "bin", "mpcrun.mat")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", nargs="?", metavar="LOGFILE1",
                    help="Supervisor .mat log to plot (default: bin/mpcrun.mat)")
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
                 args.show, args.dpi)

    if args.show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
