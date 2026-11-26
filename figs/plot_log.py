#!/usr/bin/env python3
#
# Copyright (C) 2005-2025 Uluc Saranli. All Rights Reserved
#
# This file is part of the RoboMETU robot control software library
# collection. Unauthorized copying of this file, via any medium is
# strictly prohibited.

"""Plots a Supervisor .mat log -- whatever it happens to contain.

This replaces the pair of hard-coded plotters that used to live in
figs/estimation and figs/mpc. Those each began by require()ing their own
variable set and exiting on anything else, so neither could read the other's
logs and a newly registered variable appeared in no figure at all. This script
starts from the log instead: it reads the fields that are present, and every
column of every field reaches exactly one figure.

How a figure gets chosen:

  * A table (REGISTRY, below) describes each known variable as a sequence of
    column blocks. A block may tag itself with the physical `quantity` it
    represents and the `role` it plays in it -- truth, estimate, command,
    measured, and so on.
  * Blocks sharing a quantity are drawn on one figure, overlaid, with the RMS
    of every other role against the reference role. That is where
    estimate-vs-truth and commanded-vs-actual come from; nothing is wired per
    figure.
  * Blocks with no quantity, blocks whose quantity has only one role present,
    and variables missing from the table entirely are plotted on their own.

So adding a variable to supervisor.log.vars is enough to see it, and tagging it
in REGISTRY is enough to compare it.

See README.md in this directory for the variable reference and for how to read
the figures.
"""

import argparse
import fnmatch
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.io import loadmat

import matplotlib

# Chosen before pyplot is imported. Writing files is the normal use and the
# script must work over ssh with no display; --show overrides it in main().
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


# ---------------------------------------------------------------------------
# A. Constants
# ---------------------------------------------------------------------------

# Leg order is FL, FR, RL, RR throughout the code base
# (QuadrupedKinematics::LegIndex). Joints are abduction, flexion, knee -- named
# "hip" here for the middle one, which is what the torque figures have always
# called it and what figs/mpc/score_run.py imports.
LEG_NAMES = ["FL", "FR", "RL", "RR"]
JOINT_NAMES = ["abduction", "hip", "knee"]
AXES = ["x", "y", "z"]

LEGS = tuple(LEG_NAMES)
XYZ = tuple(AXES)
RPY = ("roll", "pitch", "yaw")
WXYZ = ("w", "x", "y", "z")
JOINTS = tuple(JOINT_NAMES)

GRAVITY = 9.81

# Aggregate mass from config/default/mpc.toml:18, the same number the convex
# model uses. Only ever drawn as a reference line on the total-GRF figure.
BODY_MASS_KG = 15.2064
BODY_WEIGHT_N = BODY_MASS_KG * GRAVITY

# Per-joint torque limit, N*m. Compile-time constants from
# createGo2DynamicsConfig() in src/quadruped/QuadrupedConfigs.cc, applied in
# MdlLegControl.cc:63. Not logged, so hard-coded here.
TORQUE_LIMIT = (23.7, 23.7, 45.43)

# Controller joint sign per leg, from kDirection in
# src/hardware/mujocohw/tests/test_go2_leg_dynamics.cc, and matching
# MdlSimDriver::_polarity. MdlTrot_torquereq is logged in the controller's
# convention, straight out of command_result_t; MdlSimDriver_ctrl is what the
# simulator applies, and the two differ in the sign of the abduction joint on
# the right-hand legs. Comparing them without this makes FR and RR abduction
# look like mirror images of each other (rms 7.6 N*m against 0.01 on every
# other motor), which is a plotting artefact and not a controller fault.
#
# NOTE this is motor-index-major (3*leg + joint), so the negatives are FR and
# RR *abduction*. Read it joint-major and you invert FL/RL hip instead, which
# produces a plausible-looking and entirely wrong figure. It applies to
# MdlSimDriver_ctrl only -- never to the qpos/qvel joint columns, which are
# MuJoCo generalized coordinates and are already in the model's own frame.
JOINT_DIRECTION = ((1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
                   (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0))

# Okabe-Ito, colourblind-safe. One fixed style per role, so a trace means the
# same thing on every figure it appears in. Reference-ish roles are dashed and
# actual-ish ones solid, so an overlaid pair stays tellable apart where they
# agree -- which, for the estimator figures, is the normal case.
ROLE_STYLE = {
    "truth":     dict(color="#0072B2", linestyle="--", linewidth=1.2, zorder=3),
    "injected":  dict(color="#0072B2", linestyle="--", linewidth=1.2, zorder=3),
    "estimate":  dict(color="#D55E00", linestyle="-",  linewidth=1.0, zorder=4),
    "measured":  dict(color="#D55E00", linestyle="-",  linewidth=1.0, zorder=4),
    "applied":   dict(color="#D55E00", linestyle="-",  linewidth=1.0, zorder=4),
    "command":   dict(color="#009E73", linestyle="--", linewidth=1.0, zorder=5),
    "requested": dict(color="#009E73", linestyle="--", linewidth=1.0, zorder=5),
    # The raw sensor trace sits underneath both as a wide translucent band: it
    # is context for the processed traces, not a competitor to them.
    "raw":       dict(color="#E69F00", linestyle="-",  linewidth=2.2, alpha=0.45,
                      zorder=2),
    "schedule":  dict(color="#666666", linestyle="-",  linewidth=1.0,
                      drawstyle="steps-post", zorder=3),
    "trust":     dict(color="#CC79A7", linestyle="-",  linewidth=1.0, zorder=4),
}

# Which role the RMS annotations are measured against, most authoritative
# first. Ground truth wins wherever it exists; failing that the thing that
# physically happened (what the motors applied, what the foot actually did).
REFERENCE_ROLES = ("truth", "injected", "applied", "measured", "estimate",
                   "schedule")

# Generic (untagged) traces cycle through the rest of the Okabe-Ito set.
SERIES_COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7",
                 "#E69F00", "#56B4E9", "#F0E442", "#000000")

LIMIT_STYLE = dict(color="#999999", linestyle=":", linewidth=0.8, zorder=1)
SWING_SHADE = dict(color="grey", alpha=0.07, linewidth=0, zorder=0)
BAND_STYLE = dict(color="grey", alpha=0.15, linewidth=0, zorder=1)


# ---------------------------------------------------------------------------
# B. Math
# ---------------------------------------------------------------------------

def quat_to_rpy(q):
    """ZYX Euler angles from (N,4) w-first quaternions, radians.

    Mirrors MdlOrientationEstimator::rpyFromQuat, so the reconstructed truth
    and the estimator's own logged rpy are the same convention.
    """
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.column_stack([roll, pitch, yaw])


def quat_yaw(q):
    """Yaw about +Z from (N,4) w-first quaternions, radians."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_rotate(q, v, inverse=False):
    """Rotates (N,3) vectors by (N,4) w-first quaternions; (3,) v broadcasts.

    q is body to world throughout this code base, so `inverse=True` is the
    world to body direction.

    Written out as v + 2w(u x v) + 2u x (u x v) rather than by building
    rotation matrices, so a minute of logging at 1 kHz stays a handful of array
    operations instead of 60k small matmuls.
    """
    v = np.broadcast_to(np.asarray(v, dtype=float), (q.shape[0], 3))
    w = q[:, 0:1]
    u = q[:, 1:4]
    if inverse:
        u = -u
    uxv = np.cross(u, v)
    return v + 2.0 * (w * uxv + np.cross(u, uxv))


def wrapped_angle_error(a, b):
    """Shortest signed Euler-angle difference, component by component."""
    delta = a - b
    return np.arctan2(np.sin(delta), np.cos(delta))


def reference_tilt(filtered, ideal):
    """Angle between the filtered gravity reference and the true one [rad].

    This is rho, the attitude error the gravity reference itself asserts, and
    it is the whole reason accel_filter_tau exists. The Mahony loop can do no
    better than its reference: with mahony_ki > 0 the estimate settles at
    exactly the mean of this signal, so its DC is the estimator's steady tilt
    error and its oscillation is not. See
    reports/state_estimator/mahony_error_decomposition.md.
    """
    cross = np.linalg.norm(np.cross(filtered, ideal), axis=1)
    dot = np.sum(filtered * ideal, axis=1)
    return np.arctan2(cross, dot)


def heading_leak(t, dyaw, v_truth):
    """Position error attributable to the drifted heading alone, (N,3).

    Gravity says nothing about heading, so the yaw gyroscope bias is
    unobservable and the estimated heading drifts. What that drift does to the
    *position* estimate is the reason for this view. The filter's kinematic
    velocity measurement is expressed through the estimated attitude, so a body
    moving at v is reconstructed as moving at R_z(dyaw) v, and the difference
    integrates:

        leak(T) = integral of (R_z(dyaw(t)) - I) v_true(t) dt

    Note this is an integral, not a rotation of the final error. For a straight
    run with dyaw growing linearly to dyaw_f it comes to about dyaw_f * x_f / 2,
    which on a 0.1 rad drift over 0.64 m is 32 mm -- larger than everything else
    in the y error put together. Rotating the endpoint instead would remove a
    fraction of a millimetre and leave the whole ramp in place.
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
    the front and rear legs to opposite sides in the raw plots. What survives
    is the error heading drift does not explain -- the part a change to the
    position filter would actually move.

    `lever` must be truth foot minus truth base. Using the estimated foot or
    the estimated base makes the correction self-referential and removes real
    error along with the heading's.
    """
    out = np.array(est, dtype=float, copy=True)
    out -= leak
    if lever is not None:
        c, s = np.cos(dyaw), np.sin(dyaw)
        out[:, 0] -= (c - 1.0) * lever[:, 0] - s * lever[:, 1]
        out[:, 1] -= s * lever[:, 0] + (c - 1.0) * lever[:, 1]
    return out


def skew_shift(dt):
    """Whether the one-sample logger de-skew is valid at this sampling rate.

    MdlSimDriver runs after the estimators, so simulator truth logged at index
    i is one *control cycle* newer than the estimator state at the same index.
    Shifting truth back one *logged sample* only undoes that when
    supervisor.log.period is 1. At the shipped period of 10 the shift moves
    truth 10 ms instead of 1 ms, and the "correction" then makes the reported
    error worse by most of an order of magnitude -- measured on a 20 s
    bin/deneme.mat, the corrected-gyro RMSE goes from [0.027, 0.022, 0.008]
    rad/s unshifted to [0.247, 0.182, 0.047] shifted. So the shift is applied
    only when the log was actually recorded at the control rate, and the figure
    says which happened.
    """
    return 1 if dt is not None and dt <= 1.5e-3 else 0


def mask_runs(mask):
    """Contiguous [start, end) runs of True in a 1-D boolean mask.

    Turning the mask into runs means axvspan is called once per swing phase
    instead of once per sample.
    """
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0 or not mask.any():
        return []
    edges = np.flatnonzero(np.diff(mask.astype(int)))
    starts = [0] if mask[0] else []
    starts += [e + 1 for e in edges if mask[e + 1]]
    ends = [e + 1 for e in edges if not mask[e + 1]]
    if mask[-1]:
        ends.append(len(mask))
    return list(zip(starts, ends))


# ---------------------------------------------------------------------------
# C. The variable table
# ---------------------------------------------------------------------------
#
# A logged variable is an ordered list of blocks. A block is a rectangle of
# columns with one meaning: `labels` components, repeated once per entry in
# `groups`, with `stride` columns between repeats. That single shape covers
# every layout in the table -- contiguous vectors, leg-major triples, the
# interleaved 9-per-leg sub-blocks of MdlTrot_footref, and the joint arrays
# that start part way into qpos/qvel.
#
# A block may also say what physical `quantity` it represents and what `role`
# it plays in it. That, and only that, is what puts two traces on one figure.

@dataclass(frozen=True)
class Block:
    key: str
    start: int
    labels: tuple
    unit: str
    desc: str
    groups: tuple = ()
    stride: int = 0
    frame: str = None
    kind: str = "vec"        # vec | angle | quat | flag | weight | status
                             # | duration | variance
    quantity: str = None     # None -> this block is plotted on its own
    role: str = None
    xform: str = None        # named entry in XFORMS, applied before comparison
    xlabels: tuple = None    # labels after xform, when it changes the width
    xunit: str = None
    sentinel: str = None     # "negative" -> mask < 0 to NaN before plot and stats
    note: str = ""

    @property
    def ncomp(self):
        return len(self.labels)

    @property
    def step(self):
        return self.stride or len(self.labels)

    @property
    def width(self):
        return self.step * len(self.groups) if self.groups else len(self.labels)

    def cols(self, group=None):
        """Absolute column indices; group=None on a grouped block gives all."""
        n = self.ncomp
        if not self.groups:
            return list(range(self.start, self.start + n))
        if group is None:
            out = []
            for g in range(len(self.groups)):
                base = self.start + g * self.step
                out += list(range(base, base + n))
            return out
        base = self.start + group * self.step
        return list(range(base, base + n))

    def out_labels(self):
        return self.xlabels if self.xlabels is not None else self.labels

    def out_unit(self):
        return self.xunit if self.xunit is not None else self.unit


@dataclass(frozen=True)
class Var:
    name: str
    width: int
    frame: str
    summary: str
    blocks: tuple


def _blk(key, start, labels, unit, desc, **kw):
    return Block(key, start, tuple(labels), unit, desc, **kw)


def vec3(key, start, unit, desc, **kw):
    return _blk(key, start, XYZ, unit, desc, **kw)


def scal(key, start, unit, desc, **kw):
    return _blk(key, start, (key,), unit, desc, **kw)


def quat(key, start, desc, **kw):
    return _blk(key, start, WXYZ, "", desc, kind="quat", **kw)


def per_leg_vec3(key, start, unit, desc, stride=3, **kw):
    return _blk(key, start, XYZ, unit, desc, groups=LEGS, stride=stride, **kw)


def per_leg_joint(key, start, unit, desc, stride=3, **kw):
    return _blk(key, start, JOINTS, unit, desc, groups=LEGS, stride=stride, **kw)


def per_leg_scal(key, start, unit, desc, **kw):
    return _blk(key, start, (key,), unit, desc, groups=LEGS, stride=1, **kw)


# -- Column transforms, applied by Log.get() before anything compares traces.

def _xf_joint_direction(arr, block, leg):
    """MuJoCo motor sign -> controller sign. See JOINT_DIRECTION."""
    if leg is None:
        return arr * np.asarray(JOINT_DIRECTION, dtype=float).reshape(-1)
    return arr * np.asarray(JOINT_DIRECTION[leg], dtype=float)


XFORMS = {"joint_direction": _xf_joint_direction}


REGISTRY_LIST = [
    Var("MdlOrientationEstimator_state", 16, "body",
        "attitude estimate, bias-corrected body rate, world specific force, gyro bias",
        (quat("quat", 0, "estimated orientation, body->world", frame="world",
              quantity="orientation quaternion", role="estimate"),
         _blk("rpy", 4, RPY, "rad", "estimated attitude, ZYX Euler", kind="angle",
              frame="world", quantity="body orientation", role="estimate",
              note="numerically exactly rpy(quat), which is also logged"),
         vec3("omega_body", 7, "rad/s", "bias-corrected gyro (gyro - estimated bias)",
              quantity="body angular velocity", role="estimate"),
         vec3("a_world", 10, "m/s^2", "specific force, gravity included", frame="world",
              note="produced as R_est * a_body, so R_est^T recovers the raw "
                   "accelerometer reading exactly"),
         vec3("gyro_bias", 13, "rad/s", "estimated gyro bias",
              quantity="gyro bias", role="estimate"))),

    Var("MdlOrientationEstimator_filter", 10, "body",
        "attitude filter internals: Mahony innovation, fade gain, gravity reference, injected bias",
        (vec3("wcorr", 0, "", "Mahony gravity innovation, (a_filt/|a_filt|) x up_body",
              kind="innovation"),
         scal("gain", 3, "", "correction fade weight from the gravity-magnitude band",
              kind="weight"),
         vec3("acc_filt", 4, "m/s^2", "low-passed specific force used as the gravity reference",
              quantity="gravity reference", role="estimate"),
         vec3("injected_bias", 7, "rad/s", "synthetic gyro bias injected by simnoise",
              quantity="gyro bias", role="injected",
              note="simulation ground truth for the bias the filter is trying to find"))),

    Var("MdlOrientationEstimator_error", 11, "body",
        "attitude filter error against simulator truth, computed in-module at the same tick",
        (vec3("rotvec_err", 0, "rad", "rotation vector of R_true^T R_filter"),
         scal("tilt_err", 3, "rad", "angle between true and estimated up vectors (what gravity observes)",
              kind="angle"),
         scal("heading_err", 4, "rad", "heading error about the world vertical (unobservable)",
              kind="angle"),
         vec3("omega_err", 5, "rad/s", "bias-corrected body rate minus true body rate"),
         vec3("bias_err", 8, "rad/s", "estimated minus injected gyro bias"))),

    Var("MdlPosVelEstimator_innov", 6, "body",
        "KF innovation diagnostics over fully trusted legs",
        (vec3("vel_innov", 0, "m/s", "mean velocity-row innovation", kind="innovation"),
         scal("height_innov", 3, "m", "mean foot-height-row innovation", kind="innovation"),
         scal("nis_v", 4, "", "normalized innovation squared of the mean velocity innovation",
              kind="innovation"),
         scal("n_trusted", 5, "", "number of fully trusted legs", kind="weight"))),

    Var("MdlPosVelEstimator_state", 9, "world",
        "EKF body position and velocity estimate",
        (vec3("pos", 0, "m", "estimated body position",
              quantity="body position", role="estimate"),
         vec3("vel", 3, "m/s", "estimated body velocity",
              quantity="body velocity", role="estimate"),
         vec3("vel_body", 6, "m/s", "the same velocity rotated into the body", frame="body",
              note="numerically exactly R^T * vel"))),

    Var("MdlPosVelEstimator_footholds", 12, "world",
        "EKF foot position estimate, leg-major",
        (per_leg_vec3("pos", 0, "m", "estimated foot position",
                      quantity="foot position", role="estimate"),)),

    Var("MdlPosVelEstimator_error", 9, "world",
        "estimator error against simulator truth, plus the path length it is normalised by",
        (vec3("pos_err", 0, "m", "body position error, estimate - truth"),
         vec3("vel_err", 3, "m/s", "body velocity error, estimate - truth", frame="body",
              note="each velocity in its own body frame: heading is unobservable, so a "
                   "world-frame difference would grow with yaw drift"),
         scal("drift_percent", 6, "%", "position error as a fraction of distance travelled"),
         scal("distance_travelled", 7, "m", "low-passed horizontal path length of the true body"),
         scal("net_displacement", 8, "m", "horizontal straight-line displacement from the true start"))),

    Var("MdlPosVelEstimator_cov", 18, "world",
        "EKF covariance diagonal",
        (vec3("pos_var", 0, "m^2", "body position variance", kind="variance"),
         vec3("vel_var", 3, "m^2/s^2", "body velocity variance", kind="variance"),
         per_leg_vec3("foot_var", 6, "m^2", "foot position variance", kind="variance"))),

    Var("MdlPosVelEstimator_contacts", 4, "",
        "per-leg contact trust weight the filter actually applied",
        (per_leg_scal("trust", 0, "", "contact trust weight, ramped at both stance boundaries",
                      kind="weight", quantity="contact", role="trust",
                      note="a ramp in [0,1], not a binary contact flag"),)),

    Var("MdlPosVelEstimator_footerr", 4, "world",
        "per-leg distance from the estimated foot to simulator truth",
        (per_leg_scal("err", 0, "m", "foot position error magnitude", sentinel="negative",
                      note="negative marks a sample where truth was unavailable or "
                           "the leg was not trusted; those are masked out here"),)),

    Var("MdlPosVelEstimator_foottruth", 12, "world",
        "simulator foot positions, leg-major",
        (per_leg_vec3("pos", 0, "m", "true foot position",
                      quantity="foot position", role="truth"),)),

    Var("MdlTrot_scales", 2, "",
        "gait entry and exit ramps",
        (scal("speed_scale", 0, "", "forward-speed ramp", kind="weight",
              note="duplicates MdlTrot_bodyref[9] by definition"),
         scal("lift_scale", 1, "", "swing-height ramp", kind="weight",
              note="duplicates MdlTrot_bodyref[10] by definition"))),

    Var("MdlTrot_footref", 36, "body",
        "commanded foot state, 9 per leg: position, velocity, acceleration",
        (per_leg_vec3("cmd_pos", 0, "m", "commanded foot position", stride=9,
                      quantity="foot position body", role="command"),
         per_leg_vec3("cmd_vel", 3, "m/s", "commanded foot velocity", stride=9),
         per_leg_vec3("cmd_acc", 6, "m/s^2", "commanded foot acceleration", stride=9))),

    Var("MdlTrot_contact", 4, "",
        "gait contact schedule",
        (per_leg_scal("stance", 0, "", "scheduled stance (1) or swing (0)", kind="flag",
                      quantity="contact", role="schedule",
                      note="the gait schedule, not measured ground contact"),)),

    Var("MdlTrot_torquescale", 4, "",
        "per-leg torque saturation factor applied by MdlLegControl",
        (per_leg_scal("scale", 0, "", "saturation factor in (0,1]", kind="weight"),)),

    Var("MdlTrot_torquereq", 12, "joint",
        "requested joint torque before saturation, controller sign convention",
        (per_leg_joint("torque", 0, "N*m", "requested joint torque",
                       quantity="joint torque", role="requested"),)),

    Var("MdlTrot_bodyref", 12, "world",
        "body reference command and the gait ramp state applied with it",
        (vec3("des_pos", 0, "m", "desired body position",
              quantity="body position", role="command"),
         scal("des_yaw", 3, "rad", "desired body yaw", kind="angle",
              quantity="body yaw", role="command"),
         vec3("des_vel", 4, "m/s", "desired body velocity",
              quantity="body velocity", role="command",
              note="z is identically 0: MdlTrot commands no vertical velocity"),
         scal("cmd_yawrate", 7, "rad/s", "commanded yaw rate",
              quantity="body yaw rate", role="command"),
         scal("meas_yaw", 8, "rad", "measured yaw, unwrapped by the controller", kind="angle",
              quantity="body yaw", role="measured"),
         scal("speed_scale", 9, "", "forward-speed ramp", kind="weight"),
         scal("lift_scale", 10, "", "swing-height ramp", kind="weight"),
         scal("yaw_clamp", 11, "rad", "per-cycle clamp on the yaw setpoint"))),

    Var("MdlConvexMPC_forces", 12, "world",
        "first horizon input of the MPC solve: commanded ground reaction force",
        (per_leg_vec3("u0", 0, "N", "commanded ground reaction force",
                      quantity="ground reaction force", role="command",
                      note="MdlTrot applies f_body = -(R^T * f_world); both this and "
                           "MdlSimDriver_grf are the force on the foot, so they compare "
                           "directly with no sign flip"),)),

    Var("MdlConvexMPC_solver", 2, "",
        "QP exit status and wall-clock solve time",
        (scal("status", 0, "", "qpOASES returnValue (0 = success)", kind="status"),
         scal("solve_time", 1, "s", "QP solve wall time", kind="duration"))),

    Var("MdlSimDriver_qpos", 19, "world",
        "simulator generalized position: free-joint base pose then 12 joint angles",
        (vec3("base_pos", 0, "m", "base position (free-joint origin, the point the "
                                  "position filter's r also describes)",
              quantity="body position", role="truth"),
         quat("base_quat", 3, "base orientation, body->world",
              quantity="orientation quaternion", role="truth"),
         per_leg_joint("joint_pos", 7, "rad", "joint angle", frame="joint",
                       quantity="joint angle", role="measured"))),

    Var("MdlSimDriver_qvel", 18, "world",
        "simulator generalized velocity: base twist then 12 joint velocities",
        (vec3("base_vel", 0, "m/s", "base linear velocity",
              quantity="body velocity", role="truth"),
         vec3("base_angvel", 3, "rad/s", "base angular velocity", frame="body",
              quantity="body angular velocity", role="truth",
              note="body frame, the MuJoCo free-joint convention"),
         per_leg_joint("joint_vel", 6, "rad/s", "joint velocity", frame="joint",
                       quantity="joint velocity", role="measured"))),

    Var("MdlSimDriver_qacc", 18, "world",
        "simulator generalized acceleration",
        (vec3("base_acc", 0, "m/s^2", "base linear acceleration"),
         vec3("base_angacc", 3, "rad/s^2", "base angular acceleration", frame="body"),
         per_leg_joint("joint_acc", 6, "rad/s^2", "joint acceleration", frame="joint"))),

    Var("MdlSimDriver_ctrl", 12, "joint",
        "torque the simulator actually applied",
        (per_leg_joint("torque", 0, "N*m", "applied joint torque",
                       quantity="joint torque", role="applied",
                       xform="joint_direction",
                       note="logged in the MuJoCo sign convention; mapped into the "
                            "controller's here so it compares with MdlTrot_torquereq"),)),

    Var("MdlSimDriver_grf", 12, "world",
        "measured foot ground reaction force, leg-major",
        (per_leg_vec3("force", 0, "N", "measured ground reaction force",
                      quantity="ground reaction force", role="measured"),)),
]

REGISTRY = {v.name: v for v in REGISTRY_LIST}


def _self_check():
    """Asserts the table partitions each variable's columns exactly once.

    A hand-written 22-variable, 250-odd-column table is exactly the kind of
    thing that develops a silent off-by-one, and the symptom would be a figure
    of the wrong numbers rather than an error.
    """
    for var in REGISTRY_LIST:
        keys = [b.key for b in var.blocks]
        assert len(keys) == len(set(keys)), f"{var.name}: duplicate block keys"
        seen = []
        for b in var.blocks:
            assert b.groups or not b.stride, f"{var.name}.{b.key}: stride without groups"
            seen += b.cols()
        assert len(seen) == len(set(seen)), f"{var.name}: blocks overlap"
        assert sorted(seen) == list(range(var.width)), (
            f"{var.name}: blocks cover {sorted(seen)}, expected 0..{var.width - 1}")
        for b in var.blocks:
            assert (b.quantity is None) == (b.role is None), (
                f"{var.name}.{b.key}: quantity and role must be set together")
            if b.xform is not None:
                assert b.xform in XFORMS, f"{var.name}.{b.key}: unknown xform"


_self_check()


# ---------------------------------------------------------------------------
# D. Quantities
# ---------------------------------------------------------------------------
#
# A quantity is a physical thing several variables may each hold a version of.
# This table fixes its figure name, title, panel labels, unit and frame, so the
# axis labels and titles do not depend on which variable happened to be logged.

@dataclass(frozen=True)
class QSpec:
    stem: str
    title: str
    labels: tuple
    unit: str
    frame: str = ""
    per_leg: bool = False    # one figure per leg, panels = labels
    shade: bool = True       # swing-shade the panels when a contact signal exists
    wrap: bool = False       # angle: compare with the shortest signed difference
    hlines: tuple = ()       # per-panel symmetric limit line, e.g. TORQUE_LIMIT
    kind: str = "vec"        # drives the annotation when there is no reference
    note: str = ""


QUANTITIES = {
    "body position": QSpec(
        "body_position", "Body position", XYZ, "m", "world", shade=False),
    "body velocity": QSpec(
        "body_velocity", "Body velocity", ("vx", "vy", "vz"), "m/s", "world",
        shade=False),
    "body orientation": QSpec(
        "body_orientation", "Body orientation", RPY, "rad", "world",
        shade=False, wrap=True, kind="angle"),
    "orientation quaternion": QSpec(
        "orientation_quaternion", "Body orientation quaternion", WXYZ, "",
        "world", shade=False),
    "body angular velocity": QSpec(
        "body_angular_velocity", "Body angular velocity",
        ("omega_x", "omega_y", "omega_z"), "rad/s", "body", shade=False),
    "body yaw": QSpec(
        "body_yaw", "Body yaw", ("yaw",), "rad", "world", shade=False,
        wrap=True, kind="angle"),
    "body yaw rate": QSpec(
        "body_yaw_rate", "Body yaw rate", ("yaw rate",), "rad/s", "body",
        shade=False),
    "gyro bias": QSpec(
        "gyro_bias", "Gyroscope bias", ("b_x", "b_y", "b_z"), "rad/s", "body",
        shade=False),
    "gravity reference": QSpec(
        "gravity_reference", "Gravity reference", ("a_x", "a_y", "a_z"),
        "m/s^2", "body", shade=False),
    "foot position": QSpec(
        "foot_position", "Foot position", XYZ, "m", "world", per_leg=True),
    "foot position body": QSpec(
        "foot_reference", "Foot position", XYZ, "m", "body", per_leg=True),
    "joint torque": QSpec(
        "torque", "Joint torque", JOINTS, "N*m", "joint", per_leg=True,
        hlines=TORQUE_LIMIT),
    "ground reaction force": QSpec(
        "grf", "Ground reaction force", ("F_x", "F_y", "F_z"), "N", "world",
        per_leg=True),
    "contact": QSpec(
        "contact", "Contact", LEGS, "", "", shade=False, kind="flag",
        note="the schedule is what the gait asked for; the trust ramp is what "
             "the estimator weighted the measurement by"),
    "joint angle": QSpec(
        "joint_angle", "Joint angle", JOINTS, "rad", "joint", per_leg=True),
    "joint velocity": QSpec(
        "joint_velocity", "Joint velocity", JOINTS, "rad/s", "joint",
        per_leg=True),
}


# -- Derived traces: the handful of roles that need real arithmetic rather
#    than a slice. Each returns an (N, k) array or None when its sources are
#    not in the log, and claims whatever columns it read.

def _d_rpy_truth(log, leg):
    q = log.get("MdlSimDriver_qpos", "base_quat")
    return None if q is None else quat_to_rpy(q)


def _d_raw_gyro(log, leg):
    """The unprocessed gyro reading, reconstructed exactly.

    The estimator logs the bias-corrected rate and its bias estimate
    separately, so their sum is the raw measurement bit for bit.
    """
    w = log.get("MdlOrientationEstimator_state", "omega_body")
    b = log.get("MdlOrientationEstimator_state", "gyro_bias")
    return None if w is None or b is None else w + b


def _d_ideal_gravity(log, leg):
    q = log.get("MdlSimDriver_qpos", "base_quat")
    return None if q is None else quat_rotate(q, [0.0, 0.0, GRAVITY], inverse=True)


def _d_raw_accel(log, leg):
    """Raw specific force, recovered from the world-frame copy.

    a_world was produced as R_est * a_body, so rotating it back by the estimate
    quaternion returns the accelerometer reading exactly -- which is why this
    is reconstructed rather than logged.
    """
    q = log.get("MdlOrientationEstimator_state", "quat")
    a = log.get("MdlOrientationEstimator_state", "a_world")
    return None if q is None or a is None else quat_rotate(q, a, inverse=True)


def _d_foot_in_body(log, leg):
    """True foot position expressed in the body frame footref lives in."""
    ft = log.get("MdlPosVelEstimator_foottruth", "pos", leg=leg)
    p = log.get("MdlSimDriver_qpos", "base_pos")
    q = log.get("MdlSimDriver_qpos", "base_quat")
    if ft is None or p is None or q is None:
        return None
    return quat_rotate(q, ft - p, inverse=True)


def _d_yawrate_truth(log, leg):
    w = log.get("MdlSimDriver_qvel", "base_angvel")
    return None if w is None else w[:, 2:3]


@dataclass(frozen=True)
class Derived:
    quantity: str
    role: str
    fn: object
    source: str


DERIVED = (
    Derived("body orientation", "truth", _d_rpy_truth,
            "rpy(MdlSimDriver_qpos.base_quat)"),
    Derived("body angular velocity", "raw", _d_raw_gyro,
            "MdlOrientationEstimator_state.omega_body + .gyro_bias"),
    Derived("gravity reference", "truth", _d_ideal_gravity,
            "R(MdlSimDriver_qpos.base_quat)^T * (0,0,g)"),
    Derived("gravity reference", "raw", _d_raw_accel,
            "R(MdlOrientationEstimator_state.quat)^T * .a_world"),
    Derived("foot position body", "measured", _d_foot_in_body,
            "MdlPosVelEstimator_foottruth.pos in the body frame"),
    Derived("body yaw rate", "truth", _d_yawrate_truth,
            "MdlSimDriver_qvel.base_angvel[z]"),
)


# ---------------------------------------------------------------------------
# E. Reading a log
# ---------------------------------------------------------------------------

class LogError(SystemExit):
    """A problem with one log file.

    Caught by name where two logs are being plotted, so one bad log never
    aborts the other. Derived from SystemExit so that an importer which does
    not catch it -- figs/mpc/score_run.py -- still exits with the message
    rather than a traceback, which is what load_log used to do.
    """


def load_log(path):
    """Returns a time vector in seconds plus {varname: (N, width) array}.

    rtclient::WriteML writes a MATLAB v5 *struct array* named `data`, one
    element per logged cycle, each field a column vector. That is the transpose
    of the array-of-samples layout a plot wants, so this stacks it once here
    and everything downstream indexes by sample.

    Which fields exist depends entirely on supervisor.log.vars, so nothing is
    assumed beyond `time`.
    """
    try:
        raw = loadmat(path, squeeze_me=False, struct_as_record=True)
    except Exception as exc:
        # Pointed at something that is not a MATLAB file at all. A scipy
        # traceback here says nothing useful about what to do next.
        raise LogError(f"{path}: cannot be read as a MATLAB file ({exc}). "
                       "Supervisor logs are written with "
                       "supervisor.log.file_format = \"matlab\".")

    rec = None
    for key, val in raw.items():
        if key.startswith("__"):
            continue
        # `params` is also present and carries only metadata. The data struct
        # is the one with a time field.
        if getattr(val, "dtype", None) is not None and val.dtype.names \
                and "time" in val.dtype.names:
            rec = val
            break
    if rec is None:
        raise LogError(f"{path}: no struct with a time field; is this a Supervisor log?")

    rec = rec.ravel()
    if rec.size == 0:
        raise LogError(f"{path}: log is empty. Did the run end before "
                       "supervisor.log.start?")

    out = {}
    for name in rec.dtype.names:
        # Each element is a (width, 1) array. concatenate along axis 1 and
        # transpose is markedly faster than stacking per sample, which matters:
        # a minute of logging at 1 kHz is 60k samples.
        out[name] = np.concatenate([np.atleast_2d(v) for v in rec[name]],
                                   axis=1).T.astype(float)

    # Already seconds, and already relative to when logging started rather than
    # to when the controller did: the offset sits in the file's `params` struct
    # as time_offset, in microseconds. Rebased to zero so --start and --stop
    # read off the plotted axis directly.
    t = out.pop("time")[:, 0].astype(float)
    return t - t[0], out


def _synth_var(name, width, start=0, why="not in the variable table"):
    """A placeholder description for a variable this script has not been told
    about, so that it still gets plotted and still counts towards coverage."""
    labels = tuple(f"c{i}" for i in range(start, start + width))
    return Var(name, start + width, "", why,
               (Block("cols", start, labels, "", f"column ({why})"),))


class Log:
    """One windowed log, plus the bookkeeping that guarantees full coverage."""

    def __init__(self, path, t, data):
        self.path = path
        self.tag = logfile_tag(path)
        self.t = t
        self.data = data
        self.dt = float(np.median(np.diff(t))) if len(t) > 1 else None
        self.warnings = []
        self.vars = {name: self._resolve(name, arr.shape[1])
                     for name, arr in data.items()}
        self.claimed = {name: set() for name in data}
        self.by_block = {}          # name -> {block key: {figure stems}}
        self._stem = None

    # -- construction -------------------------------------------------------

    def _resolve(self, name, width):
        """Matches the logged width against the table, degrading if they differ."""
        var = REGISTRY.get(name)
        if var is None:
            self.warnings.append(
                f"'{name}' is not in the variable table; plotting its {width} "
                "columns generically. Add it to REGISTRY in plot_log.py to give "
                "it labels, units and comparisons.")
            return _synth_var(name, width)
        if width == var.width:
            return var
        # A layout change in C++, or a log older than the current table. Keep
        # every block that still fits and say so, rather than dropping the
        # variable or -- worse -- plotting shifted columns.
        keep = tuple(b for b in var.blocks if max(b.cols()) < width)
        covered = set()
        for b in keep:
            covered |= set(b.cols())
        extra = sorted(set(range(width)) - covered)
        blocks = list(keep)
        if extra:
            blocks.append(Block("unmapped", extra[0],
                                tuple(f"c{i}" for i in extra), "",
                                "column beyond the variable table's layout"))
        self.warnings.append(
            f"'{name}' is {width} columns wide, the table describes {var.width}. "
            f"Using {len(keep)} of {len(var.blocks)} blocks"
            + (f" plus {len(extra)} unmapped columns." if extra else "."))
        return Var(name, width, var.frame, var.summary, tuple(blocks))

    def window(self, start, stop):
        keep = self.t >= start
        if stop is not None:
            keep &= self.t <= stop
        if not keep.any():
            raise LogError(
                f"{self.path}: no samples in [{start}, "
                f"{'end' if stop is None else stop}]; log spans 0 to "
                f"{self.t[-1]:.1f} s")
        out = Log(self.path, self.t[keep], {k: v[keep] for k, v in self.data.items()})
        out.warnings = list(self.warnings)
        return out

    # -- access -------------------------------------------------------------

    def has(self, name):
        return name in self.data

    def block(self, name, key):
        var = self.vars.get(name)
        if var is None:
            return None
        for b in var.blocks:
            if b.key == key:
                return b
        return None

    def begin(self, stem):
        """Names the figure that subsequent get() calls are building."""
        self._stem = stem

    def get(self, name, key, leg=None, claim=True):
        """The columns of one block as (N, k), or None if unavailable.

        Applies the block's xform and sentinel mask, and records the columns as
        covered by the figure named in the last begin().
        """
        blk = self.block(name, key)
        if blk is None:
            return None
        if blk.groups and leg is not None and leg >= len(blk.groups):
            return None
        cols = blk.cols(leg)
        arr = self.data[name][:, cols]
        if blk.sentinel == "negative":
            arr = np.where(arr < 0.0, np.nan, arr)
        if blk.xform is not None:
            arr = XFORMS[blk.xform](arr, blk, leg)
        if claim:
            self.claimed[name] |= set(cols)
            if self._stem:
                self.by_block.setdefault(name, {}).setdefault(key, set()).add(self._stem)
        return arr

    def unclaimed(self, name):
        var = self.vars[name]
        return set(range(var.width)) - self.claimed[name]

    def total_columns(self):
        return sum(v.width for v in self.vars.values())


# ---------------------------------------------------------------------------
# F. Naming and output paths
# ---------------------------------------------------------------------------

def logfile_tag(path):
    """A filesystem-friendly label based on a log's filename."""
    stem = os.path.splitext(os.path.basename(path))[0]
    tag = "".join(c if c.isalnum() or c in "-_." else "_" for c in stem)
    return tag or "log"


def unique_log_tags(paths):
    """Per-path tags, disambiguated when two logs share a basename."""
    base_tags = [logfile_tag(path) for path in paths]
    totals = {tag: base_tags.count(tag) for tag in base_tags}
    seen = {}
    tags = []
    for tag in base_tags:
        seen[tag] = seen.get(tag, 0) + 1
        tags.append(f"{tag}_{seen[tag]}" if totals[tag] > 1 else tag)
    return tags


def resolve_logfile(path, repo_root):
    """Resolves a log path from the cwd first, then from the repository root."""
    candidates = [path]
    if not os.path.isabs(path):
        candidates.append(os.path.join(repo_root, path))
    for cand in candidates:
        if os.path.exists(cand):
            return os.path.abspath(cand)
    # Deduplicated: run from the repository root and both candidates are the
    # same path, and "checked X and X" reads like a bug in the message.
    checked, seen = [], set()
    for cand in candidates:
        full = os.path.abspath(cand)
        if full not in seen:
            seen.add(full)
            checked.append(full)
    raise LogError(f"{path}: not found (checked {' and '.join(checked)})")


# ---------------------------------------------------------------------------
# G. Rendering
# ---------------------------------------------------------------------------

@dataclass
class Trace:
    y: np.ndarray
    style: dict
    label: str
    t: np.ndarray = None        # None -> the figure's own time base


@dataclass
class Panel:
    ylabel: str                 # always "<name> [<unit>]"
    traces: list = field(default_factory=list)
    annot: list = field(default_factory=list)
    shade: np.ndarray = None    # boolean mask over t; True spans get shaded
    band: tuple = None          # (lo, hi, label)
    hlines: tuple = ()          # symmetric limits, drawn at both +v and -v
    reflines: tuple = ()        # one-sided reference lines, drawn at v only
    ylim: tuple = None
    yscale: str = None
    legend: bool = False


@dataclass
class FigureSpec:
    stem: str
    title: str
    t: np.ndarray
    panels: list
    subnote: str = ""


@dataclass
class Ctx:
    outdir: str
    suffix: str = ""
    logs_label: str = ""
    show: bool = False
    dpi: int = 200
    ext: str = "png"
    decimate: int = 0           # 0 -> auto
    dry_run: bool = False
    only: tuple = ()
    skip: tuple = ()
    written: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    def selected(self, stem):
        if self.only and not any(fnmatch.fnmatch(stem, p) for p in self.only):
            return False
        if self.skip and any(fnmatch.fnmatch(stem, p) for p in self.skip):
            return False
        return True

    def path(self, stem):
        return os.path.join(self.outdir, f"{stem}{self.suffix}.{self.ext}")


def _annotate(ax, lines, loc=(0.995, 0.04)):
    """Small boxed text in a subplot corner."""
    if not lines:
        return
    ax.text(
        loc[0], loc[1], "\n".join(lines),
        transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.7", alpha=0.85),
    )


def _finite(a, b=None):
    """Samples where every operand is finite -- the sentinel mask leaves NaNs."""
    ok = np.isfinite(a)
    if b is not None:
        ok &= np.isfinite(b)
    return ok


def track_annot(act, ref=None, wrap=False, prefix="", kind="vec", unit=""):
    """Per-component metric: rms against a reference, else a per-kind statistic.

    This is the one copy of the rule both old scripts implemented by hand in
    five separate plot functions. `kind` and `unit` are only consulted when
    there is no reference to measure against; see kind_annot().
    """
    out = []
    n = act.shape[1]
    for i in range(n):
        a = act[:, i]
        if ref is not None and ref.shape[1] == n:
            r = ref[:, i]
            ok = _finite(a, r)
            if not ok.any():
                out.append([f"{prefix}no valid samples"])
                continue
            err = wrapped_angle_error(a[ok], r[ok]) if wrap else a[ok] - r[ok]
            out.append([f"{prefix}rms {float(np.sqrt(np.mean(err ** 2))):.4g}"])
        else:
            # No reference role. A bare mean+rms of the signal itself reads as a
            # tracking error and is not one -- on a body position it is how far
            # the robot walked. What is worth saying depends on what the signal
            # is, so that is dispatched on the block's kind instead.
            out.append([prefix + line for line in kind_annot(kind, a, unit)])
    return out


def kind_annot(kind, y, unit=""):
    """What is worth saying about a signal that has nothing to compare against.

    Never returns a bare "rms": the only rms this function will name is the rms
    of a signal that is *itself* an error, and it says so.
    """
    ok = np.isfinite(y)
    if not ok.any():
        return ["no valid samples"]
    v = y[ok]
    masked = float(np.mean(~ok))
    extra = [f"masked {100.0 * masked:.0f}%"] if masked > 0.005 else []

    if kind == "innovation":
        # Dimensionless ~sin(tilt), so milliradians is the honest unit.
        return [f"rms innovation {1e3 * float(np.sqrt(np.mean(v ** 2))):.2f} mrad",
                f"peak {1e3 * float(np.max(np.abs(v))):.2f} mrad"] + extra
    if kind == "flag":
        # Duty factor plus how many cycles it went through -- for a contact
        # schedule that is the whole content of the trace.
        cycles = int(np.count_nonzero(np.diff((v > 0.5).astype(int)) > 0))
        return [f"duty {100.0 * float(np.mean(v > 0.5)):.1f}%",
                f"{cycles} cycles"] + extra
    if kind == "weight":
        # A weight that sits just under its ceiling (gain at 0.994) and one that
        # is pinned to it (torque_scale at exactly 1.0) are different states,
        # and "at ceiling 0%" does not distinguish them. The minimum does.
        out = [f"mean {float(np.mean(v)):.4g}", f"min {float(np.min(v)):.4g}"]
        at = float(np.mean(v >= 1.0 - 1e-9))
        if at > 0.005:
            out.append(f"at ceiling {100.0 * at:.0f}%")
        return out + extra
    if kind == "status":
        return [f"{int(np.count_nonzero(v != 0))} nonzero of {v.size}"] + extra
    if kind == "duration":
        q50, q95 = np.percentile(v, [50, 95])
        return [f"p50 {1e3 * q50:.3f} / p95 {1e3 * q95:.3f} / "
                f"max {1e3 * float(np.max(v)):.3f} ms"] + extra
    if kind == "variance":
        return [f"median {float(np.median(v)):.3g} {unit}".rstrip(),
                f"max {float(np.max(v)):.3g}"] + extra
    if kind == "angle":
        return [f"range {float(np.ptp(v)):.4g}",
                f"drift {float(v[-1] - v[0]):.4g}"] + extra
    # Everything else, positions included: the range is what the trace covers.
    # Its mean is rarely meaningful and on a world-frame position it is
    # actively misleading.
    return [f"range {float(np.ptp(v)):.4g}",
            f"min {float(np.min(v)):.4g} max {float(np.max(v)):.4g}"] + extra


def series_style(i, dashed=False):
    return dict(color=SERIES_COLORS[i % len(SERIES_COLORS)],
                linestyle="--" if dashed else "-", linewidth=1.0)


def _is_constant(y):
    ok = np.isfinite(y)
    if not ok.any():
        return False, 0.0
    v = y[ok]
    scale = max(1.0, float(np.max(np.abs(v))))
    return bool(np.ptp(v) <= 1e-12 * scale), float(v[0])


def _legend_style(style):
    """Only the keys Line2D accepts for a legend proxy."""
    keep = ("color", "linestyle", "linewidth", "alpha", "drawstyle", "marker")
    return {k: v for k, v in style.items() if k in keep}


def _legend_entries(spec):
    """Deduplicated (label, style) pairs across every panel of a figure.

    Dropped only when the legend would repeat the y-axis labels -- a generic
    figure whose panels are one trace each, labelled "x" against a "x [m]"
    axis. A single-trace figure whose label names a variable and role
    ("MdlSimDriver_ctrl.torque (applied)") keeps its legend: that label is the
    only thing on the figure saying where the numbers came from, and a log
    carrying just one role for a quantity is the common case.
    """
    redundant = all(
        len(p.traces) <= 1 and p.band is None
        and all(p.ylabel.startswith(tr.label) for tr in p.traces)
        for p in spec.panels)
    if redundant:
        return []
    out, seen = [], set()
    for panel in spec.panels:
        if panel.band is not None and panel.band[2] not in seen:
            seen.add(panel.band[2])
            out.append((panel.band[2], dict(BAND_STYLE, linewidth=6)))
        for tr in panel.traces:
            if tr.label and tr.label not in seen:
                seen.add(tr.label)
                out.append((tr.label, tr.style))
    return out


def render(spec, ctx):
    """Writes one figure. The only place pyplot output happens.

    Everything repetitive lives here: figure sizing, draw order, axis labels,
    the legend, the time axis, the unconditional log name in the title, the
    constant-trace guard, decimation, and closing the figure.
    """
    if not ctx.selected(spec.stem):
        return None
    outpath = ctx.path(spec.stem)
    if ctx.dry_run:
        ctx.written.append(outpath)
        return outpath
    # Created here rather than up front, so a run that fails to read its log
    # leaves no empty directory behind.
    os.makedirs(ctx.outdir, exist_ok=True)

    n = len(spec.panels)
    # The header carries a two- or three-line title plus a legend whose labels
    # name whole variables, so it is sized explicitly rather than left to
    # tight_layout: an automatic legend either overflows the figure or sits on
    # top of the data.
    title = spec.title + spec.subnote + "\n" + ctx.logs_label
    entries = _legend_entries(spec)
    ncol = 1 if entries and max(len(lab) for lab, _ in entries) > 22 \
        else max(1, min(4, len(entries)))
    rows = -(-len(entries) // ncol) if entries else 0
    head_in = 0.30 + 0.19 * (title.count("\n") + 1) + 0.17 * rows
    fig_h = 2.6 * n + head_in + 0.55
    fig, axs = plt.subplots(n, 1, figsize=(10, fig_h), sharex=True,
                            squeeze=False)
    axs = axs[:, 0]

    for ax, panel in zip(axs, spec.panels):
        # 1. shading underneath everything, one axvspan per contiguous run
        if panel.shade is not None:
            for s, e in mask_runs(panel.shade):
                ax.axvspan(spec.t[s], spec.t[min(e, len(spec.t) - 1)], **SWING_SHADE)
        # 2. band
        if panel.band is not None:
            lo, hi, blabel = panel.band
            del blabel  # labelled through the figure legend
            ax.fill_between(spec.t, lo, hi, **BAND_STYLE)
        # 3. traces
        const_notes = []
        n_const = 0
        for tr in panel.traces:
            t = spec.t if tr.t is None else tr.t
            y = np.asarray(tr.y, dtype=float).reshape(len(t), -1)[:, 0]
            step = ctx.decimate or max(1, len(t) // 20000)
            ax.plot(t[::step], y[::step], label=tr.label, **tr.style)
            flat, value = _is_constant(y)
            if flat:
                n_const += 1
                const_notes.append(f"{tr.label}: constant {value:.4g}")
        # 4. limits (symmetric) and references (one-sided). A body-weight
        #    reference mirrored to -149 N would stretch the axis over a region
        #    the signal never visits and flatten the part that matters.
        for value in panel.hlines:
            ax.axhline(value, **LIMIT_STYLE)
            ax.axhline(-value, **LIMIT_STYLE)
        for value in panel.reflines:
            ax.axhline(value, **LIMIT_STYLE)
        ax.set_ylabel(panel.ylabel)
        ax.grid(True, alpha=0.3)
        if panel.yscale:
            ax.set_yscale(panel.yscale)
        if panel.ylim:
            ax.set_ylim(*panel.ylim)
        elif n_const and n_const == len(panel.traces):
            # Autoscaling a panel that holds nothing but a constant renders
            # float noise as though it were a live signal. Widen it to a
            # visibly flat line instead. Only when *every* trace is constant:
            # otherwise the live ones set the scale and clamping would crop
            # them off the figure.
            lo, hi = ax.get_ylim()
            mid = 0.5 * (lo + hi)
            ax.set_ylim(mid - 1.0, mid + 1.0)
        _annotate(ax, list(panel.annot) + const_notes)

    axs[-1].set_xlabel("time [s]")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 1.0 - head_in / fig_h))
    fig.suptitle(title, y=1.0 - 0.10 / fig_h, va="top", fontsize=11)

    # One legend for the whole figure, above the axes: the trace set repeats
    # down the panels, and a per-axes legend would cover the data.
    if entries:
        handles = []
        for lab, style in entries:
            handles.append(plt.Line2D([], [], label=lab, **_legend_style(style)))
        fig.legend(handles=handles, loc="upper center", ncol=ncol, fontsize=8,
                   frameon=False,
                   bbox_to_anchor=(0.5, 1.0 - (0.20 + 0.19 * (title.count("\n") + 1))
                                   / fig_h))
    fig.savefig(outpath, dpi=ctx.dpi)
    print(f"wrote {outpath}")
    ctx.written.append(outpath)
    if not ctx.show:
        plt.close(fig)
    return outpath


# -- Overlaying two logs on one set of axes ---------------------------------

OVERLAY_LINESTYLE = ("-", "--", ":")


def merge_specs(specs, tags):
    """Superimposes per-log FigureSpecs that describe the same figure.

    Each log keeps its role colours and takes a linestyle, so a role stays
    recognisable across logs while the logs stay tellable apart. Traces carry
    their own time base, so logs recorded at different periods overlay
    correctly.
    """
    base = specs[0]
    if any(len(s.panels) != len(base.panels) for s in specs):
        return None
    panels = []
    for i in range(len(base.panels)):
        merged = Panel(ylabel=base.panels[i].ylabel,
                       shade=base.panels[i].shade,
                       band=base.panels[i].band,
                       hlines=base.panels[i].hlines,
                       reflines=base.panels[i].reflines,
                       ylim=base.panels[i].ylim,
                       yscale=base.panels[i].yscale,
                       legend=base.panels[i].legend)
        for k, (spec, tag) in enumerate(zip(specs, tags)):
            panel = spec.panels[i]
            for tr in panel.traces:
                style = dict(tr.style)
                style["linestyle"] = OVERLAY_LINESTYLE[k % len(OVERLAY_LINESTYLE)]
                merged.traces.append(
                    Trace(tr.y, style, f"{tag}: {tr.label}",
                          t=spec.t if tr.t is None else tr.t))
            merged.annot += [f"{tag} {line}" for line in panel.annot]
        panels.append(merged)
    notes = " ".join(s.subnote for s in specs if s.subnote)
    return FigureSpec(base.stem, base.title, base.t, panels, notes)


# ---------------------------------------------------------------------------
# H. Building the quantity figures
# ---------------------------------------------------------------------------

def gait_contact(log):
    """(contact signal, what it means) for swing shading; (None, None) if neither.

    MdlTrot_contact is the gait schedule, which is what the shading is meant to
    show. Failing that, MdlPosVelEstimator_contacts carries the estimator's
    contact trust ramp over the same footfalls -- close enough to read gait
    phase off, but a different quantity, so its name is carried along and
    printed on the figure rather than passed off as the schedule.

    Deliberately not the measured ground reaction force: shading
    foot_reference_<LEG> with measured contact would make the figure
    self-referential exactly where it is read for touchdown timing.

    claim=False throughout -- shading is not what covers these columns; the
    `contact` quantity figure is.
    """
    c = log.get("MdlTrot_contact", "stance", claim=False)
    if c is not None:
        return c, "scheduled swing"
    c = log.get("MdlPosVelEstimator_contacts", "trust", claim=False)
    if c is not None:
        return c, "estimator contact trust < 0.5"
    return None, None


def heading_context(log):
    """(dyaw, leak) for --heading-aligned, or None when the log cannot support it."""
    ori = log.get("MdlOrientationEstimator_state", "rpy", claim=False)
    q = log.get("MdlSimDriver_qpos", "base_quat", claim=False)
    v = log.get("MdlSimDriver_qvel", "base_vel", claim=False)
    if ori is None or q is None or v is None:
        return None
    dyaw = wrapped_angle_error(ori[:, 2], quat_to_rpy(q)[:, 2])
    return dyaw, heading_leak(log.t, dyaw, v)


# -- Per-quantity extras ----------------------------------------------------

def _sub_orientation(log, roles, qspec, leg):
    """Says whether the orientation figure is measuring anything at all.

    Under attitude_source = "imu" the estimate *is* qpos[3:7] -- MdlSimDriver's
    IMU read copies the same array getGroundTruth() reports, and the module
    passes it through untouched -- so the two curves are the same numbers and
    agreement means nothing. Under "filter" the comparison is real. Which one
    produced the log cannot be read from the log, so it is detected.

    The threshold has to scale with the sampling period: the residual is the
    one-control-cycle logging skew, which at log.period = 10 is ten times what
    it is at 1. A fixed 1e-3 misclassifies the coarser logs.
    """
    est, truth = roles.get("estimate"), roles.get("truth")
    if est is None or truth is None:
        return '\n(attitude source unknown: no ground truth in this log)'
    dt = log.dt or 1e-3
    thresh = 5e-4 * max(1.0, dt / 1e-3)
    if float(np.max(np.abs(est[:, 0:2] - truth[:, 0:2]))) < thresh:
        return ('\n(attitude_source = "imu": identical by construction, the IMU '
                "quaternion is the simulator's own)")
    return ('\n(attitude_source = "filter": estimated here, so these curves '
            "can and do differ)")


def _sub_gravity(log, roles, qspec, leg):
    est, truth = roles.get("estimate"), roles.get("truth")
    if est is None or truth is None:
        return ""
    rho = reference_tilt(est, truth)
    return (f"\n(reference tilt rho: mean {1e3 * float(np.mean(rho)):.2f} mrad, "
            f"rms {1e3 * float(np.sqrt(np.mean(rho ** 2))):.2f} mrad)")


def _sub_body_rate(log, roles, qspec, leg):
    """Names what was done about the one-control-cycle logger skew."""
    dt = log.dt
    if skew_shift(dt):
        return "\n(ground truth shifted one sample to match estimator logging)"
    if dt is None:
        return ""
    return (f"\n(sampled every {1e3 * dt:.1f} ms, longer than one control cycle: "
            "the one-cycle logger skew is not corrected)")


SUBTITLE = {
    "body orientation": _sub_orientation,
    "gravity reference": _sub_gravity,
    "body angular velocity": _sub_body_rate,
}

# Only the angular-velocity comparison is fine-grained enough for the
# one-control-cycle skew between MdlSimDriver and the estimators to matter.
SKEW_QUANTITIES = ("body angular velocity",)


def _extra_torque_annot(i, roles, qspec):
    """Saturation, which the applied trace alone can never show.

    The applied torque is clipped at the limit by construction, so it cannot
    distinguish "asked for exactly the limit" from "asked for 130% and got
    clipped". The requested trace makes that visible; these two numbers turn it
    into a reading.
    """
    limit = qspec.hlines[i]
    tag, y = ("req", roles.get("requested"))
    if y is None:
        tag, y = "app", roles.get("applied")
    if y is None:
        return []
    col = y[:, i]
    ok = np.isfinite(col)
    if not ok.any():
        return []
    col = col[ok]
    # For the applied trace an equality test, not a strict >: a saturated
    # sample lands exactly on the limit.
    over = np.mean(np.abs(col) > limit) if tag == "req" \
        else np.mean(np.abs(col) >= limit - 1e-9)
    return [f"|{tag}| at/over limit: {100.0 * float(over):.1f}% of samples",
            f"max|{tag}|: {100.0 * float(np.max(np.abs(col))) / limit:.1f}% of limit"]


EXTRA_ANNOT = {"joint torque": _extra_torque_annot}


def collect_roles(log, qname, qspec, leg, exclude=()):
    """{role: array}, {role: source text}, and which roles a variable supplied.

    The third return separates roles read straight out of the log from roles
    this script computed. A figure whose every trace is computed carries no
    information its sources do not already show on their own figures, so the
    caller drops it -- that is what keeps an MPC-set log from growing an
    orientation figure with truth plotted against nothing.
    """
    arrays, sources, from_log = {}, {}, set()
    ncomp = len(qspec.labels)
    for name in sorted(log.vars):
        for blk in log.vars[name].blocks:
            if blk.quantity != qname or blk.role in exclude:
                continue
            arr = log.get(name, blk.key, leg=leg if qspec.per_leg else None)
            if arr is None or arr.shape[1] != ncomp:
                continue
            arrays[blk.role] = arr
            sources[blk.role] = f"{name}.{blk.key}"
            from_log.add(blk.role)
    for d in DERIVED:
        if d.quantity != qname or d.role in arrays or d.role in exclude:
            continue
        arr = d.fn(log, leg)
        if arr is not None and arr.shape[1] == ncomp:
            arrays[d.role] = arr
            sources[d.role] = d.source
    return arrays, sources, from_log


def build_quantity(log, qname, qspec, leg, contact, contact_kind, heading, ctx,
                   exclude_roles=(), allow_single=False):
    """One FigureSpec for one quantity, or None if the log cannot support it.

    A quantity that resolves only one role is not a comparison. It is dropped
    unless the profile whitelisted it (`allow_single`), which is how a log
    without the estimator variables stops producing four figures of pure ground
    truth.
    """
    roles, sources, from_log = collect_roles(log, qname, qspec, leg, exclude_roles)
    if not roles or not from_log:
        return None
    if len(roles) < 2 and not allow_single:
        return None

    t = log.t
    subnote = ""

    # -- logger skew, where it is both applicable and valid
    if qname in SKEW_QUANTITIES and skew_shift(log.dt):
        t = log.t[1:]
        roles = {r: (a[:-1] if r == "truth" else a[1:]) for r, a in roles.items()}

    # -- heading alignment rewrites the estimate in place
    if heading is not None and qname in ("body position", "foot position") \
            and "estimate" in roles:
        dyaw, leak = heading
        lever = None
        if qname == "foot position":
            ft = log.get("MdlPosVelEstimator_foottruth", "pos", leg=leg, claim=False)
            base = log.get("MdlSimDriver_qpos", "base_pos", claim=False)
            if ft is not None and base is not None:
                lever = ft - base
        roles["estimate"] = heading_align(roles["estimate"], leak, dyaw, lever)
        subnote += "\n(heading aligned: the drifted yaw's contribution removed)"

    ref_role = next((r for r in REFERENCE_ROLES if r in roles), None)
    ncomp = len(qspec.labels)

    shade = None
    if qspec.shade and contact is not None:
        col = contact[:, leg] if (leg is not None and contact.shape[1] > leg) else None
        if col is not None:
            shade = col[-len(t):] < 0.5
            subnote += f"\n(shaded: {contact_kind})"

    panels = []
    for i in range(ncomp):
        unit = f" [{qspec.unit}]" if qspec.unit else ""
        panel = Panel(ylabel=f"{qspec.labels[i]}{unit}", shade=shade,
                      hlines=(qspec.hlines[i],) if qspec.hlines else ())
        for role in sorted(roles, key=lambda r: list(ROLE_STYLE).index(r)
                           if r in ROLE_STYLE else 99):
            style = dict(ROLE_STYLE.get(role, series_style(0)))
            panel.traces.append(Trace(roles[role][:, i], style,
                                      f"{sources[role]} ({role})"))
        # RMS of every other role against the reference one.
        lines = []
        if ref_role is not None and len(roles) > 1:
            for role in roles:
                if role == ref_role:
                    continue
                lines += track_annot(roles[role][:, i:i + 1], roles[ref_role][:, i:i + 1],
                                     wrap=qspec.wrap,
                                     prefix=f"{role} vs {ref_role}: ")[0]
        else:
            lines += track_annot(roles[ref_role or list(roles)[0]][:, i:i + 1],
                                 kind=qspec.kind, unit=qspec.unit)[0]
        lines += EXTRA_ANNOT.get(qname, lambda *a: [])(i, roles, qspec)
        panel.annot = lines
        panels.append(panel)

    # The yaw clamp band is the one place a third variable decorates a panel:
    # MdlTrot_bodyref[11] is the bound the controller actually enforces on
    # desired-minus-measured yaw, so shading it makes clamp activity readable
    # without cross-referencing the config.
    if qname == "body yaw" and "measured" in roles:
        clamp = log.get("MdlTrot_bodyref", "yaw_clamp")
        if clamp is not None:
            m = roles["measured"][:, 0]
            panels[0].band = (m - clamp[:, 0], m + clamp[:, 0],
                              "MdlTrot_bodyref.yaw_clamp band")

    # The three run-summary scalars are not time series, so they belong in the
    # position figure's annotation rather than in three one-panel figures.
    if qname == "body position":
        for key, label, fmt in (("drift_percent", "drift", "{:.2f}%"),
                                ("distance_travelled", "travelled", "{:.4g} m"),
                                ("net_displacement", "net displacement",
                                 "{:.4g} m")):
            col = log.get("MdlPosVelEstimator_error", key)
            if col is not None and len(col):
                panels[0].annot.append(
                    f"{label} " + fmt.format(float(col[-1, 0])))

    title = qspec.title
    if leg is not None:
        title = f"{LEG_NAMES[leg]} {qspec.title.lower()}"
    if qspec.frame:
        title += f" ({qspec.frame} frame)"
    title += ": " + " vs ".join(sorted(roles, key=lambda r: r != ref_role))
    subnote += SUBTITLE.get(qname, lambda *a: "")(log, roles, qspec, leg)
    if qspec.note:
        subnote += f"\n({qspec.note})"

    stem = qspec.stem if leg is None else f"{qspec.stem}_{LEG_NAMES[leg]}"
    return FigureSpec(stem, title, t, panels, subnote)


def build_grf_total(log, contact, contact_kind, ctx):
    """Total ground reaction force, commanded against measured.

    Summed over the legs this is the whole vertical force budget, so the
    horizontal line at body weight says directly whether the controller is
    asking for enough force to hold the robot up. Nothing in the per-leg
    figures makes that readable.
    """
    log.begin("grf_total")
    roles, sources = {}, {}
    for name, key, role in (("MdlSimDriver_grf", "force", "measured"),
                            ("MdlConvexMPC_forces", "u0", "command")):
        legs = [log.get(name, key, leg=i) for i in range(len(LEGS))]
        if any(a is None for a in legs):
            continue
        roles[role] = sum(legs)
        sources[role] = f"sum over legs of {name}.{key}"
    if not roles:
        return None

    ref = "measured" if "measured" in roles else None
    panels = []
    for i, label in enumerate(("F_x", "F_y", "F_z")):
        panel = Panel(ylabel=f"{label} [N]")
        for role in sorted(roles):
            panel.traces.append(Trace(roles[role][:, i], dict(ROLE_STYLE[role]),
                                      f"{sources[role]} ({role})"))
        lines = []
        for role in roles:
            if role != ref:
                lines += track_annot(
                    roles[role][:, i:i + 1],
                    None if ref is None else roles[ref][:, i:i + 1],
                    prefix=f"{role} vs {ref}: " if ref else f"{role}: ")[0]
        if ref is not None:
            lines.append(f"{ref} mean {float(np.mean(roles[ref][:, i])):.4g} N")
            if label == "F_z":
                lines.append(f"body weight {BODY_WEIGHT_N:.1f} N")
        panel.annot = lines
        if label == "F_z":
            panel.reflines = (BODY_WEIGHT_N,)
        panels.append(panel)
    return FigureSpec(
        "grf_total", "Total ground reaction force (world frame): "
        + " vs ".join(sorted(roles)), log.t, panels,
        f"\n(dotted line: body weight, {BODY_MASS_KG:.3f} kg x g = "
        f"{BODY_WEIGHT_N:.1f} N)")


# ---------------------------------------------------------------------------
# I. The generic pass: everything the quantity figures did not claim
# ---------------------------------------------------------------------------

def build_attitude_filter(log, ctx):
    """wcorr, gain, and the product the filter actually integrates.

    MdlOrientationEstimator applies

        _gyroBias -= mahony_ki * _gain * _wcorr * dt

    so neither signal alone says what the filter did: wcorr is how far the
    accelerometer disagrees with the estimate, gain is how much that
    disagreement was trusted this cycle, and only their product reaches the bias
    estimate. Overlaid on the same axes because they share units and the product
    is bounded by wcorr, so the gap between them *is* the fading.
    """
    log.begin("attitude_filter")
    w = log.get("MdlOrientationEstimator_filter", "wcorr")
    g = log.get("MdlOrientationEstimator_filter", "gain")
    if w is None or g is None:
        return None

    applied = w * g
    panels = []
    for i, axis in enumerate(XYZ):
        panel = Panel(ylabel=f"wcorr_{axis} [-]")
        panel.traces.append(Trace(w[:, i], dict(ROLE_STYLE["estimate"]),
                                  "MdlOrientationEstimator_filter.wcorr "
                                  "(innovation)"))
        panel.traces.append(Trace(applied[:, i], dict(ROLE_STYLE["command"]),
                                  "gain x wcorr (integrated into the bias)"))
        panel.annot = kind_annot("innovation", w[:, i])
        panels.append(panel)

    gp = Panel(ylabel="gain [-]", ylim=(-0.05, 1.05))
    gp.traces.append(Trace(g[:, 0], dict(ROLE_STYLE["trust"]),
                           "MdlOrientationEstimator_filter.gain "
                           "(accelerometer trust)"))
    gp.annot = kind_annot("weight", g[:, 0])
    panels.append(gp)

    return FigureSpec(
        "attitude_filter", "Attitude filter: gravity innovation and how much "
        "of it was applied", log.t, panels,
        "\n(gain fades the correction as |a_filt| leaves the gravity band; "
        "the bias integrates gain x wcorr, not wcorr)")


def build_covariance(log, ctx):
    """Three figures from the EKF covariance diagonal: position, velocity, foot.

    Kept separate rather than stacked into one: the three live on different
    scales and are read for different questions -- whether the body estimate is
    confident, whether the velocity rows are converging, and whether a
    particular foot is being trusted.

    Plotted as a standard deviation on a log axis, not a variance: sqrt(P) is in
    the units of the thing it describes, so it compares directly against the
    error on the corresponding comparison figure.
    """
    out = []
    for key, title, unit, groups in (
            ("pos_var", "body position", "m", None),
            ("vel_var", "body velocity", "m/s", None),
            ("foot_var", "foot position", "m", LEGS)):
        stem = f"estimator_cov_{key.split('_')[0]}"
        if not ctx.selected(stem):
            continue
        log.begin(stem)
        panels = []
        for i, axis in enumerate(XYZ):
            panel = Panel(ylabel=f"sigma_{axis} [{unit}]", yscale="log")
            if groups is None:
                arr = log.get("MdlPosVelEstimator_cov", key)
                if arr is None:
                    break
                panel.traces.append(Trace(np.sqrt(np.abs(arr[:, i])),
                                          series_style(i), axis))
                panel.annot = kind_annot("variance", np.sqrt(np.abs(arr[:, i])), unit)
            else:
                ok = True
                for g, gname in enumerate(groups):
                    arr = log.get("MdlPosVelEstimator_cov", key, leg=g)
                    if arr is None:
                        ok = False
                        break
                    sd = np.sqrt(np.abs(arr[:, i]))
                    panel.traces.append(Trace(sd, series_style(g), str(gname)))
                    panel.annot += kind_annot("variance", sd, unit)[:1]
                if not ok:
                    break
                panel.legend = True
            panels.append(panel)
        if len(panels) == len(XYZ):
            out.append(FigureSpec(
                stem, f"EKF {title} uncertainty (standard deviation)",
                log.t, panels,
                "\n(sqrt of the covariance diagonal, so it is in the units of "
                "the quantity and compares directly with its error)"))
    return out


def _generic_title(name, var, blk):
    unit = blk.out_unit()
    frame = blk.frame or var.frame
    bracket = ", ".join(x for x in (unit, f"{frame} frame" if frame else "") if x)
    title = f"{name}.{blk.key}: {blk.desc}"
    return title + (f"  [{bracket}]" if bracket else "")


def _generic_panel_kwargs(blk):
    kw = {}
    if blk.kind in ("flag", "weight"):
        kw["ylim"] = (-0.05, 1.05)
    if blk.kind == "variance":
        kw["yscale"] = "log"
    return kw


def _trace_style(blk, i):
    style = series_style(i)
    if blk.kind in ("flag", "status"):
        style["drawstyle"] = "steps-post"
    return style


def build_generic(log, name, blk, todo, ctx):
    """FigureSpecs for the columns of one block that no quantity figure took."""
    var = log.vars[name]
    labels = blk.out_labels()
    ncomp = len(labels)
    unit = blk.out_unit()
    title = _generic_title(name, var, blk)
    note = f"\n({blk.note})" if blk.note else ""
    if blk.sentinel == "negative" and not blk.note:
        # Only when the block does not already explain its own sentinel.
        note += "\n(negative samples are the unavailable/untrusted sentinel, masked out)"
    stem = f"generic_{name}_{blk.key}"
    pkw = _generic_panel_kwargs(blk)

    def ylab(label):
        return f"{label} [{unit}]" if unit else str(label)

    # A quaternion's components share a scale by construction, so they belong
    # on one panel rather than four.
    if blk.kind == "quat" and blk.xform is None:
        arr = log.get(name, blk.key)
        panel = Panel(ylabel=ylab("component"), **pkw)
        for i, label in enumerate(labels):
            panel.traces.append(Trace(arr[:, i], series_style(i), str(label)))
        panel.annot = [f"|q| mean {float(np.mean(np.linalg.norm(arr, axis=1))):.6g}"]
        return [FigureSpec(stem, title, log.t, [panel], note)]

    if blk.groups:
        # One panel per component, one trace per group -- legs share a scale,
        # so overlaying them is informative rather than destructive.
        panels = []
        for i, label in enumerate(labels):
            panel = Panel(ylabel=ylab(label), **pkw)
            lines = []
            for g, gname in enumerate(blk.groups):
                if blk.cols(g)[i] not in todo:
                    continue
                arr = log.get(name, blk.key, leg=g)
                panel.traces.append(Trace(arr[:, i], _trace_style(blk, g), str(gname)))
                lines += track_annot(arr[:, i:i + 1], prefix=f"{gname} ",
                                     kind=blk.kind, unit=blk.out_unit())[0]
            if not panel.traces:
                continue
            panel.annot = lines
            panel.legend = True
            panels.append(panel)
        return [FigureSpec(stem, title, log.t, panels, note)] if panels else []

    arr = log.get(name, blk.key)
    cols = blk.cols()
    want = [i for i in range(ncomp) if cols[i] in todo]

    # A wide unmapped block would otherwise become a figure of twenty panels.
    if ncomp > 8:
        specs = []
        per_fig, per_panel = 24, 4
        chunks = [want[k:k + per_fig] for k in range(0, len(want), per_fig)]
        for ci, chunk in enumerate(chunks):
            panels = []
            for p in range(0, len(chunk), per_panel):
                panel = Panel(ylabel=ylab("value"), **pkw)
                for j, i in enumerate(chunk[p:p + per_panel]):
                    panel.traces.append(
                        Trace(arr[:, i], _trace_style(blk, j), str(labels[i])))
                panel.legend = True
                panels.append(panel)
            specs.append(FigureSpec(stem if ci == 0 else f"{stem}_{ci + 1}",
                                    title, log.t, panels, note))
        return specs

    panels = []
    for i in want:
        panel = Panel(ylabel=ylab(labels[i]), **pkw)
        panel.traces.append(Trace(arr[:, i], _trace_style(blk, i), str(labels[i])))
        panel.annot = track_annot(arr[:, i:i + 1], kind=blk.kind,
                                  unit=blk.out_unit())[0]
        panels.append(panel)
    return [FigureSpec(stem, title, log.t, panels, note)] if panels else []


def generic_figures(log, ctx):
    specs = []
    for name in sorted(log.vars):
        for blk in log.vars[name].blocks:
            todo = [c for c in blk.cols() if c not in log.claimed[name]]
            if not todo:
                continue
            stem = f"generic_{name}_{blk.key}"
            if not ctx.selected(stem):
                continue
            log.begin(stem)
            specs += build_generic(log, name, blk, todo, ctx)
    return specs


def coverage_report(log, ctx, nfig_quantity, nfig_generic):
    """What was plotted, from what, and whether anything was left over."""
    total = log.total_columns()
    covered = sum(len(s) for s in log.claimed.values())
    span = f"{log.t[0]:.2f}-{log.t[-1]:.2f} s"
    rate = f", dt {1e3 * log.dt:.1f} ms" if log.dt else ""
    print(f"\ncoverage  {log.path}")
    print(f"  {len(log.vars)} variables, {total} columns, {len(log.t)} samples, "
          f"{span}{rate}")
    for name in sorted(log.vars):
        var = log.vars[name]
        got = len(log.claimed[name])
        uses = log.by_block.get(name, {})
        detail = "  ".join(f"{k}>{','.join(sorted(v))}" for k, v in sorted(uses.items()))
        print(f"    {name:32s} {got:3d}/{var.width:<3d} {detail}")
    left = {n: sorted(log.unclaimed(n)) for n in log.vars if log.unclaimed(n)}
    tail = f"{covered}/{total} columns plotted"
    if left:
        tail += "; NOT PLOTTED: " + ", ".join(
            f"{n}[{','.join(map(str, c))}]" for n, c in left.items())
    print(f"  -> {tail}. {nfig_quantity} quantity figures, "
          f"{nfig_generic} generic.")
    for w in log.warnings:
        print(f"  warning: {w}")
    if left and not (ctx.only or ctx.skip):
        print("  warning: the above columns reached no figure. This is a bug in "
              "plot_log.py's block table.")


# ---------------------------------------------------------------------------
# J. Orchestration
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Profiles
# ---------------------------------------------------------------------------
#
# A profile is an explicit, hand-curated list of figures -- deliberately not
# derived from what the log happens to contain. Deriving it is exactly what
# produced thirty figures a run, most of them answering a question nobody asked.
#
#   stems        figure stems to draw. A per-leg quantity is named without its
#                leg suffix and expands to all four.
#   single       stems that are worth drawing even with only one role, because
#                they stand alone honestly (a gait schedule, raw joint state,
#                filter internals). Everything else needs a comparison.
#   exclude      roles to drop from a figure that is otherwise kept.
#   extra        composite figures (grf_total, attitude_filter) by name.

@dataclass(frozen=True)
class Profile:
    summary: str
    stems: tuple
    single: tuple = ()
    exclude: tuple = ()          # ((stem, role), ...)
    extra: tuple = ()


PROFILES = {
    "estimator": Profile(
        "state estimator against simulator ground truth",
        stems=("body_position", "body_velocity", "body_orientation",
               "orientation_quaternion", "body_angular_velocity", "gyro_bias",
               "gravity_reference", "foot_position"),
        single=("estimator_cov_position", "estimator_cov_velocity",
                "estimator_cov_foot"),
        # Position is integrated, not tracked: the commanded trace says nothing
        # about the estimator and crowds the comparison that does.
        exclude=(("body_position", "command"),),
        extra=("attitude_filter", "estimator_covariance")),

    "controller": Profile(
        "MdlTrot / MPC tracking, effort and force",
        stems=("torque", "foot_reference", "grf", "body_velocity", "body_yaw",
               "body_yaw_rate", "contact"),
        # Without MdlTrot_torquereq the applied trace still reads against the
        # limit lines and the saturation percentage, which is the question.
        single=("torque", "contact"),
        extra=("grf_total",)),

    "plant": Profile(
        "raw joint state, no comparison",
        stems=("joint_angle", "joint_velocity"),
        single=("joint_angle", "joint_velocity")),
}

PROFILE_NAMES = tuple(PROFILES) + ("all",)


def _stem_base(stem):
    """foot_position_FL -> foot_position; everything else unchanged."""
    for leg in LEG_NAMES:
        if stem.endswith(f"_{leg}"):
            return stem[: -len(leg) - 1]
    return stem


def pick_profile(log):
    """The profile with the most drawable figures for this log.

    Scored by how many of a profile's quantities resolve at least two roles --
    i.e. how many real comparisons the log can support -- so an estimator
    recording picks `estimator` and a run_cell.sh recording picks `controller`
    without either being named.
    """
    best, best_score = "all", 0
    for name, prof in PROFILES.items():
        score = 0
        for qname, qspec in QUANTITIES.items():
            if qspec.stem not in prof.stems:
                continue
            roles, _, from_log = collect_roles(
                log, qname, qspec, 0 if qspec.per_leg else None)
            if from_log and (len(roles) > 1 or qspec.stem in prof.single):
                score += 1
        if score > best_score:
            best, best_score = name, score
    return best if best_score else "all"


def build_all(log, ctx, heading_aligned, no_generic, generic_only,
              profile="all"):
    """Every FigureSpec for one log, quantity figures first so they claim first."""
    prof = PROFILES.get(profile)
    contact, contact_kind = gait_contact(log)
    heading = None
    if heading_aligned and not generic_only:
        heading = heading_context(log)
        if heading is None:
            print("note: --heading-aligned needs MdlOrientationEstimator_state, "
                  "MdlSimDriver_qpos and MdlSimDriver_qvel; this log has not all "
                  "of them, so the position figures are left unaligned.")

    specs = []
    if not generic_only:
        for qname, qspec in QUANTITIES.items():
            if prof is not None and qspec.stem not in prof.stems:
                continue
            exclude = tuple(r for st, r in (prof.exclude if prof else ())
                            if st == qspec.stem)
            allow_single = prof is None or qspec.stem in prof.single
            legs = range(len(LEGS)) if qspec.per_leg else (None,)
            for leg in legs:
                stem = qspec.stem if leg is None else f"{qspec.stem}_{LEG_NAMES[leg]}"
                if not ctx.selected(stem):
                    continue
                log.begin(stem)
                spec = build_quantity(log, qname, qspec, leg, contact, contact_kind,
                                      heading, ctx, exclude_roles=exclude,
                                      allow_single=allow_single)
                if spec is not None:
                    specs.append(spec)
        for name, builder in (("grf_total", build_grf_total),
                              ("attitude_filter", build_attitude_filter)):
            if prof is not None and name not in prof.extra:
                continue
            if not ctx.selected(name):
                continue
            spec = builder(log, contact, contact_kind, ctx) \
                if name == "grf_total" else builder(log, ctx)
            if spec is not None:
                specs.append(spec)
        if prof is None or "estimator_covariance" in prof.extra:
            specs += build_covariance(log, ctx)
    nq = len(specs)

    if not no_generic and prof is None:
        specs += generic_figures(log, ctx)
    return specs, nq, len(specs) - nq


def open_log(path, start, stop):
    t, data = load_log(path)
    return Log(path, t, data).window(start, stop)


def default_logfile(repo_root):
    """bin/deneme.mat, the name config/default/supervisor.toml actually ships."""
    preferred = os.path.join(repo_root, "bin", "deneme.mat")
    if os.path.exists(preferred):
        return preferred
    import glob
    found = sorted(glob.glob(os.path.join(repo_root, "bin", "*.mat")))
    if len(found) == 1:
        print(f"note: {preferred} is not there; using the only log in bin/, "
              f"{os.path.basename(found[0])}")
        return found[0]
    runs = sorted(glob.glob(os.path.join(repo_root, "runs", "*.mat")))
    raise SystemExit(
        "no log given and bin/deneme.mat does not exist.\n"
        f"candidates in bin/:  {', '.join(os.path.basename(p) for p in found) or '(none)'}\n"
        f"candidates in runs/: {', '.join(os.path.basename(p) for p in runs) or '(none)'}\n"
        "See README.md in this directory for how to record one.")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, ".."))

    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile", nargs="?", metavar="LOGFILE1",
                    help="Supervisor .mat log to plot (default: bin/deneme.mat)")
    ap.add_argument("logfile2", nargs="?", metavar="LOGFILE2",
                    help="optional second Supervisor .mat log")
    ap.add_argument("-o", "--outdir", default=None,
                    help="where to write the figures (default: figs/out/<log>)")
    ap.add_argument("--start", type=float, default=0.0,
                    help="drop samples before this time [s]")
    ap.add_argument("--stop", type=float, default=None,
                    help="drop samples after this time [s]")
    ap.add_argument("--show", action="store_true",
                    help="also open the figures in a window")
    ap.add_argument("--dpi", type=int, default=200,
                    help="figure resolution in dots per inch (default: 200)")
    ap.add_argument("--ext", default="png", choices=("png", "pdf", "svg"),
                    help="output file format (default: png)")
    ap.add_argument("-a", "--heading-aligned", action="store_true",
                    help="subtract the drifted heading's contribution from the "
                         "position figures")
    ap.add_argument("--overlay", action="store_true",
                    help="with two logs, superimpose them on one set of axes "
                         "instead of writing a figure set per log")
    ap.add_argument("--decimate", type=int, default=0,
                    help="plot every Nth sample (default: auto, ~20k points)")
    ap.add_argument("--only", default="",
                    help="comma-separated glob patterns; plot only matching figures")
    ap.add_argument("--skip", default="",
                    help="comma-separated glob patterns; skip matching figures")
    ap.add_argument("--profile", default=None, choices=PROFILE_NAMES,
                    help="which set of figures to draw (default: the one this "
                         "log best supports). 'all' keeps the old behaviour: "
                         "every column of every variable reaches a figure")
    ap.add_argument("--no-generic", action="store_true",
                    help="only the quantity figures, no per-block fallback")
    ap.add_argument("--generic-only", action="store_true",
                    help="one figure per block of every variable, no comparisons")
    ap.add_argument("--list", action="store_true", dest="list_only",
                    help="report what the log holds and what would be written, "
                         "then exit without drawing anything")
    args = ap.parse_args()

    if args.generic_only or args.no_generic:
        # Both are statements about the generic pass, which only exists under
        # 'all'; honouring them under a curated profile would silently do
        # nothing.
        args.profile = args.profile or "all"
    logfiles = [p for p in (args.logfile, args.logfile2) if p is not None]
    if not logfiles:
        logfiles = [default_logfile(repo_root)]

    if args.show and not args.list_only:
        matplotlib.use("TkAgg", force=True)

    resolved = []
    for path in logfiles:
        try:
            resolved.append(resolve_logfile(path, repo_root))
        except LogError as exc:
            raise SystemExit(f"{exc}. See README.md for how to record one.")

    tags = unique_log_tags(resolved)
    overlay = args.overlay and len(resolved) > 1
    outdir = args.outdir or os.path.join(
        here, "out", tags[0] if len(tags) == 1 else "_vs_".join(tags))

    only = tuple(p for p in args.only.split(",") if p)
    skip = tuple(p for p in args.skip.split(",") if p)

    failures = 0
    all_specs = []
    logs = []
    for path, tag in zip(resolved, tags):
        ctx = Ctx(outdir=outdir,
                  suffix="" if (len(resolved) == 1 or overlay) else f"_{tag}",
                  logs_label=f"Log: {os.path.basename(path)}",
                  show=args.show, dpi=args.dpi, ext=args.ext,
                  decimate=args.decimate, dry_run=args.list_only,
                  only=only, skip=skip)
        try:
            log = open_log(path, args.start, args.stop)
        except LogError as exc:
            print(f"error: {exc}", file=sys.stderr)
            failures += 1
            continue
        profile = args.profile
        if profile is None:
            profile = pick_profile(log)
            print(f"note: --profile not given; using '{profile}' "
                  f"({PROFILES[profile].summary})"
                  if profile in PROFILES else
                  "note: --profile not given and no profile matches this log; "
                  "using 'all'")
            if profile in PROFILES:
                print(f"      override with --profile "
                      f"{'|'.join(PROFILE_NAMES)}")
        specs, nq, ng = build_all(log, ctx, args.heading_aligned,
                                  args.no_generic, args.generic_only, profile)
        logs.append((log, ctx, specs))
        # The coverage report exists to prove nothing was silently dropped.
        # Under a curated profile things are dropped on purpose, so it would be
        # a page of noise saying so.
        if profile == "all":
            coverage_report(log, ctx, nq, ng)
        all_specs.append((specs, ctx, tag))

    if not logs:
        return 1

    if overlay and len(all_specs) == 2:
        label = "Logs: " + ", ".join(os.path.basename(p) for p in resolved)
        by_stem = {}
        for specs, _, tag in all_specs:
            for spec in specs:
                by_stem.setdefault(spec.stem, []).append((spec, tag))
        octx = Ctx(outdir=outdir, logs_label=label, show=args.show, dpi=args.dpi,
                   ext=args.ext, decimate=args.decimate, dry_run=args.list_only,
                   only=only, skip=skip)
        print()
        for stem in sorted(by_stem):
            entries = by_stem[stem]
            merged = merge_specs([e[0] for e in entries], [e[1] for e in entries])
            if merged is None:
                print(f"note: {stem} has a different panel layout in each log; "
                      "not overlaid.")
                continue
            if len(entries) == 1:
                merged.subnote += f"\n(only {entries[0][1]} carries this quantity)"
            render(merged, octx)
        written = len(octx.written)
    else:
        print()
        written = 0
        for log, ctx, specs in logs:
            for spec in specs:
                render(spec, ctx)
            written += len(ctx.written)
        if len(logs) == 2:
            stems = [{s.stem for s in specs} for _, _, specs in logs]
            for i, j in ((0, 1), (1, 0)):
                extra = sorted(stems[i] - stems[j])
                if extra:
                    print(f"note: only {tags[i]} has {', '.join(extra)}")

    if args.list_only:
        print(f"\n--list: would write {written} figures to {outdir}")
    else:
        print(f"\nwrote {written} figures to {outdir}")

    if args.show and not args.list_only:
        if written > 12:
            print(f"note: --show is opening {written} windows; "
                  "use --only to narrow it down.")
        plt.show()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
