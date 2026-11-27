/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef TROTAMB_HH
#define TROTAMB_HH

#include <Eigen/Dense>

#include "quadruped/MdlAMB.hh"
#include "quadruped/QuadrupedLegDynamics.hh"

/** \brief Angular-momentum balancing inside a trot's two-foot phases.

  Not a module: MdlTrot owns the legs and the gait, and calls tick() from its
  whole-body path. In every two-foot diagonal phase the thesis' reduced-order
  law (MdlAMB's statics: project, comConsistentX1, templateKin, robotMomentum,
  ambLaw) runs on the current stance pair and asks each swing leg for an
  angular acceleration thdd about an axis through its hip, parallel to the
  support line. Two realizations of that thdd, selected by params_t::mode:

  OFFSET       thdd is integrated into an angle alpha per swing leg and the
               planned foot reference is rotated about the hip by alpha. The
               swing task then tracks the rotated position, velocity and
               acceleration: the same feedforward as PASSTHROUGH, plus feedback
               on the motion the law asked for. Before touchdown alpha returns
               to zero on a quintic, so the foot lands on its target.
  PASSTHROUGH  thesis (6.23), (6.26): the swing task's error is removed along
               the tangential direction e_t and rho thdd e_t is added as a pure
               feedforward. Before touchdown the removed share falls to zero,
               which hands the foot back to the planned trajectory.

  The law's state (observer, offsets) is reset at the first tick of every
  two-foot phase (thesis Sec. 6.4.3); four-foot and flight ticks are idle. */
class TrotAMB {
 public:
  enum Mode { OFFSET, PASSTHROUGH };

  struct params_t {
    /** Template, gains, kp3/kd3, observer_tc, fd_step, foot_acc_max. */
    MdlAMB::params_t law;
    Mode mode = OFFSET;
    /** Authority ramps in over this time after lift-off [s]. */
    double ramp = 0.05;
    /** The law stops this long before input_t::swing_left runs out and the
        swing feet return to their planned trajectory [s]. */
    double ret = 0.15;
    /** A swing shorter than this leaves AMB off for the whole phase [s]. */
    double min_swing = 0.4;
    /** Bound on the OFFSET angle [rad]; integration stops at it. */
    double alpha_max = 0.9;
  };

  struct input_t {
    /** Joint state, body attitude and rate; t is any clock that advances by
        the control period [s]. */
    MdlAMB::input_t body;
    bool stance[4] = {true, true, true, true};
    /** Per swing leg: time since lift-off, and time left until the foot must
        be back on its planned reference (the touchdown, or the start of the
        descent; negative once that has passed) [s]. */
    double swing_time[4] = {0.0, 0.0, 0.0, 0.0};
    double swing_left[4] = {0.0, 0.0, 0.0, 0.0};
    /** Planned swing reference, body frame (MdlTrot's _footpos/vel/acc). */
    Eigen::Vector3d foot_p[4], foot_v[4], foot_a[4];
    /** Scales the authority: MdlTrot's lift scale, 0 while the feet do not
        leave the ground (stop ramp). */
    double scale = 1.0;
  };

  struct output_t {
    /** The reference the swing task should track, body frame: the planned one,
        rotated by alpha in OFFSET mode. */
    Eigen::Vector3d foot_p[4], foot_v[4], foot_a[4];
    /** PASSTHROUGH: unit tangential direction (world axes), the share of the
        swing task's error to remove along it, and the feedforward [m/s^2]
        (world axes, already scaled by the share). Zero otherwise. */
    Eigen::Vector3d dir[4], acc[4];
    double share[4];
  };

  static constexpr int LOG_SIZE = 24;

  explicit TrotAMB(const QuadrupedLegDynamics::params_t& legs);

  void setParams(const params_t& p) { _p = p; }
  const params_t& getParams() const { return _p; }
  /** Forget the current phase; the next two-foot tick starts fresh. */
  void reset();
  void tick(const input_t& in, output_t& out);
  /** Where the robot is against the support line of a diagonal (0: FL-RR,
      1: FR-RL), from the joint angles and the attitude alone: across the
      line, along the unit t_h, the whole-body CoM's offset c_h and the base
      origin's b_h; along the line, along the unit s, the base origin's offset
      a_s from the support midpoint [m]. Directions in world axes, in that
      pair's own orientation. Stateless; false on a kinematics failure. */
  bool lineOffset(const MdlAMB::input_t& body, int pairId, double& c_h, double& b_h, double& a_s,
                  Eigen::Vector3d& t_h, Eigen::Vector3d& s) const;
  /** Ticks since construction on which the law could not be evaluated. */
  int invalidTicks() const { return _invalid; }
  /** The MdlTrot_amb log of the last tick (layout documented at _log). */
  const double* log() const { return _log; }

  /** A hip->foot reference turned by alpha about the unit axis n through the
      hip h, with its first two time derivatives; n and h are taken as constant
      (body frame: the support line moves in it only as the body tilts). */
  static void rotateReference(const Eigen::Vector3d& n, const Eigen::Vector3d& h, double alpha,
                              double alpha_d, double alpha_dd, const Eigen::Vector3d& p,
                              const Eigen::Vector3d& v, const Eigen::Vector3d& a,
                              Eigen::Vector3d& p_out, Eigen::Vector3d& v_out,
                              Eigen::Vector3d& a_out);

 private:
  QuadrupedLegDynamics _legs;
  params_t _p;

  int _pair = -1;      // -1 none, 0 FL-RR stance, 1 FR-RL stance
  bool _first = true;  // first tick of the phase: the observer has no previous sample
  double _tau_hat = 0.0, _L_prev = 0.0, _ch_prev = 0.0, _t_prev = 0.0;
  // OFFSET state per swing leg (index 0: sw2, 1: sw3), and the return
  // quintic's start, latched on the first tick of the return window.
  double _alpha[2] = {0.0, 0.0}, _alpha_d[2] = {0.0, 0.0};
  bool _returning = false;
  double _ret_T = 0.0, _ret_a0[2] = {0.0, 0.0}, _ret_v0[2] = {0.0, 0.0};
  Eigen::Vector3d _n_body = Eigen::Vector3d::Zero();  // last valid support direction
  int _invalid = 0;

  // Logged as MdlTrot_amb:
  //   [0]       stance pair: -1 none, 0 FL-RR, 1 FR-RL
  //   [1]       0 idle, 1 law acting, 2 return window, 3 phase shorter than
  //             min_swing, 4 law not evaluable this tick
  //   [2..4]    L [kg m^2/s], L_dot = M g c_h, L_ddot = M g c_h_dot, in the
  //             pair's own orientation (see MdlAMB::pair_t)
  //   [5]       observed external torque tau_hat [N m]
  //   [6]       c_h [m]
  //   [7..9]    x = (x1 CoM-consistent, theta2 + theta3, theta2 - theta3) [rad]
  //   [10]      u0 = -(k0 L + k1 L_dot + k2 L_ddot) / g
  //   [11]      x3 reference [rad]
  //   [12],[13] thdd of sw2, sw3 from the law [rad/s^2]
  //   [14],[15] measured angle of sw2, sw3 from their planned reference [rad]
  //   [16],[17] OFFSET alpha of sw2, sw3 [rad]
  //   [18],[19] OFFSET alpha rate [rad/s]
  //   [20]      authority: ramp x scale, and in PASSTHROUGH the share
  //   [21]      invalid ticks so far
  //   [22],[23] projected radius rho of sw2, sw3 [m]
  double _log[LOG_SIZE] = {0};
};

#endif
