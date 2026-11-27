/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#include <algorithm>
#include <cmath>
#include "quadruped/TrotAMB.hh"
#include "quadruped/TrajectoryUtils.hh"

namespace {

const double GRAVITY = 9.81;

// Angle about the unit axis s from a to b, both projected into the plane
// normal to s. Positive along e_t = s x e_r, i.e. a positive rate of the plane
// angle MdlAMB::geometry measures.
double angleAbout(const Eigen::Vector3d& s, const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  const Eigen::Vector3d ap = a - a.dot(s) * s, bp = b - b.dot(s) * s;
  return std::atan2(s.dot(ap.cross(bp)), ap.dot(bp));
}

}  // namespace

TrotAMB::TrotAMB(const QuadrupedLegDynamics::params_t& legs) : _legs(legs) {}

void TrotAMB::reset() {
  _pair = -1;
}

bool TrotAMB::lineOffset(const MdlAMB::input_t& body, int pairId, double& c_h, double& b_h,
                         double& a_s, Eigen::Vector3d& t_h, Eigen::Vector3d& s) const {
  MdlAMB::proj_t pr;
  if (!MdlAMB::geometry(_legs, body.q, body.R, pairId == 0 ? MdlAMB::FL_RR : MdlAMB::RL_FR, pr))
    return false;
  c_h = pr.c_h;
  b_h = -pr.th.dot(pr.o);  // pr.o: the support midpoint from the base origin
  a_s = -pr.s.dot(pr.o);
  t_h = pr.th;
  s = pr.s;
  return true;
}

void TrotAMB::rotateReference(const Eigen::Vector3d& n, const Eigen::Vector3d& h, double alpha,
                              double alpha_d, double alpha_dd, const Eigen::Vector3d& p,
                              const Eigen::Vector3d& v, const Eigen::Vector3d& a,
                              Eigen::Vector3d& p_out, Eigen::Vector3d& v_out,
                              Eigen::Vector3d& a_out) {
  const Eigen::AngleAxisd rot(alpha, n);
  // d' = Rot(alpha) d differentiated twice with alpha, d both moving:
  //   d'' = Rot d'' + 2 alpha' n x Rot d' + alpha'' n x d' + alpha'^2 n x (n x d').
  const Eigen::Vector3d d = rot * (p - h), rv = rot * v;
  p_out = h + d;
  v_out = rv + alpha_d * n.cross(d);
  a_out = rot * a + 2.0 * alpha_d * n.cross(rv) + alpha_dd * n.cross(d) +
          alpha_d * alpha_d * n.cross(n.cross(d));
}

void TrotAMB::tick(const input_t& in, output_t& out) {
  for (int leg = 0; leg < 4; leg++) {
    out.foot_p[leg] = in.foot_p[leg];
    out.foot_v[leg] = in.foot_v[leg];
    out.foot_a[leg] = in.foot_a[leg];
    out.dir[leg].setZero();
    out.acc[leg].setZero();
    out.share[leg] = 0.0;
  }
  std::fill(_log, _log + LOG_SIZE, 0.0);
  _log[0] = -1.0;
  _log[21] = _invalid;

  int pairId = -1;
  if (in.stance[0] && in.stance[3] && !in.stance[1] && !in.stance[2]) pairId = 0;
  if (in.stance[1] && in.stance[2] && !in.stance[0] && !in.stance[3]) pairId = 1;
  if (pairId < 0) {
    _pair = -1;
    return;
  }
  if (pairId != _pair) {
    // Thesis Sec. 6.4.3: a new support line, so nothing support-dependent is
    // carried over. L itself is measured, only the observer and the offsets
    // are state.
    _pair = pairId;
    _first = true;
    _tau_hat = 0.0;
    _alpha[0] = _alpha[1] = _alpha_d[0] = _alpha_d[1] = 0.0;
    _returning = false;
  }
  const MdlAMB::pair_t pair = pairId == 0 ? MdlAMB::FL_RR : MdlAMB::RL_FR;
  const int sw[2] = {pair.sw2, pair.sw3};
  const QuadrupedLegDynamics::params_t& prm = _legs.getParams();
  const MdlAMB::params_t& law = _p.law;
  const Eigen::Matrix3d& R = in.body.R;
  const double dt = _first ? 0.0 : in.body.t - _t_prev;
  _log[0] = pairId;

  // The two swing legs lift and land together; the earlier touchdown rules.
  const double t_in = std::min(in.swing_time[sw[0]], in.swing_time[sw[1]]);
  const double t_left = std::min(in.swing_left[sw[0]], in.swing_left[sw[1]]);
  const bool phaseOn = t_in + t_left >= _p.min_swing;

  // -- The reduced-order law on this pair (MdlAMB::computeTick's sequence) ---
  MdlAMB::proj_t pr;
  Eigen::Vector3d d_plan[2], d_meas[2];
  double thdd[2] = {0.0, 0.0}, dev[2] = {0.0, 0.0}, rho[2] = {0.0, 0.0};
  bool valid = MdlAMB::project(law, in.body, _legs, pr, pair);
  for (int k = 0; valid && k < 2; k++) {
    const Eigen::Vector3d& h = prm.leg[sw[k]].joint[0].parent_translation;
    Eigen::Vector3d p_meas;
    valid = _legs.footPosition(sw[k], in.body.q[sw[k]], p_meas);
    d_plan[k] = R * (in.foot_p[sw[k]] - h);
    d_meas[k] = R * (p_meas - h);
    dev[k] = angleAbout(pr.s, d_plan[k], d_meas[k]);
    rho[k] = (d_plan[k] - d_plan[k].dot(pr.s) * pr.s).norm();
  }
  if (valid) {
    _n_body = R.transpose() * pr.s;
    Eigen::Vector3d x(pr.theta(0), pr.theta(1) + pr.theta(2), pr.theta(1) - pr.theta(2));
    Eigen::Vector3d xd(pr.thetad(0), pr.thetad(1) + pr.thetad(2), pr.thetad(1) - pr.thetad(2));
    const double M = pr.mass;
    x(0) = MdlAMB::comConsistentX1(law, x, xd, M * pr.c_h);
    MdlAMB::kin_t k;
    MdlAMB::templateKin(law, x, xd, k);
    const double L =
        MdlAMB::robotMomentum(_legs, in.body.q, in.body.qd, R, in.body.w_body, pr.o, pr.s, pair);
    xd(0) = (L - k.Mt(0, 1) * xd(1) - k.Mt(0, 2) * xd(2)) / k.Mt(0, 0);
    MdlAMB::templateKin(law, x, xd, k);
    const double Ld = M * GRAVITY * pr.c_h, Ldd = M * GRAVITY * pr.ch_dot;
    if (!_first && dt > 0.0) {
      const double r = (L - _L_prev) / dt - 0.5 * M * GRAVITY * (pr.c_h + _ch_prev);
      _tau_hat += dt / (law.observer_tc + dt) * (r - _tau_hat);
    }
    _L_prev = L;
    _ch_prev = pr.c_h;
    // The opening angle the gait itself plans: the measured one less what the
    // two legs have turned away from their planned references. The law's x3
    // channel then holds the gait's opening angle and leaves x2 to balance.
    const double x3d = x(2) - (dev[0] - dev[1]);
    Eigen::Vector3d xdd;
    valid = MdlAMB::ambLaw(law, k, x, xd, L, Ld, Ldd, _tau_hat, x3d, xdd);
    if (valid) {
      thdd[0] = 0.5 * (xdd(1) + xdd(2));
      thdd[1] = 0.5 * (xdd(1) - xdd(2));
    }
    _log[2] = L;
    _log[3] = Ld;
    _log[4] = Ldd;
    _log[5] = _tau_hat;
    _log[6] = pr.c_h;
    for (int a = 0; a < 3; a++) _log[7 + a] = x(a);
    _log[10] = -(law.gains(0) * L + law.gains(1) * (Ld + _tau_hat) + law.gains(2) * Ldd) / GRAVITY;
    _log[11] = x3d;
  }
  _t_prev = in.body.t;
  _first = false;
  if (!valid) _invalid++;

  // -- Authority over the phase -----------------------------------------------
  double g, g_dot;
  TrajectoryUtils::sampleQuintic(t_in / _p.ramp, _p.ramp, g, g_dot);
  g *= std::max(0.0, std::min(1.0, in.scale));
  const bool window = phaseOn && t_left > _p.ret;  // the law may act
  const bool acting = window && valid;
  int status = !phaseOn ? 3 : (t_left <= _p.ret ? 2 : (valid ? 1 : 4));

  if (_p.mode == OFFSET) {
    if (phaseOn && !window && !_returning) {
      _returning = true;
      _ret_T = std::max(t_left, 1e-3);
      for (int k = 0; k < 2; k++) {
        _ret_a0[k] = _alpha[k];
        _ret_v0[k] = _alpha_d[k];
      }
    }
    for (int k = 0; k < 2; k++) {
      double alpha_dd = 0.0;
      if (_returning) {
        // Quintic from (alpha, alpha', 0) to rest at zero at the touchdown.
        const double tau = std::min(1.0, std::max(0.0, 1.0 - t_left / _ret_T));
        const double p0 = _ret_a0[k], v0 = _ret_v0[k] * _ret_T;
        const double c3 = -10.0 * p0 - 6.0 * v0, c4 = 15.0 * p0 + 8.0 * v0, c5 = -6.0 * p0 - 3.0 * v0;
        const double t2 = tau * tau, t3 = t2 * tau;
        _alpha[k] = p0 + v0 * tau + c3 * t3 + c4 * t3 * tau + c5 * t3 * t2;
        _alpha_d[k] = (v0 + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t3 * tau) / _ret_T;
        alpha_dd = (6.0 * c3 * tau + 12.0 * c4 * t2 + 20.0 * c5 * t3) / (_ret_T * _ret_T);
      } else if (phaseOn) {
        if (acting && rho[k] > 0.0) {
          const double lim = law.foot_acc_max / rho[k];
          alpha_dd = std::max(-lim, std::min(lim, g * thdd[k]));
        }
        _alpha_d[k] += alpha_dd * dt;
        _alpha[k] += _alpha_d[k] * dt;
        // At the bound the leg has no travel left: the reference stops there
        // instead of running ahead of what the leg can do.
        if (std::abs(_alpha[k]) >= _p.alpha_max && _alpha[k] * _alpha_d[k] > 0.0) {
          _alpha[k] = std::copysign(_p.alpha_max, _alpha[k]);
          _alpha_d[k] = 0.0;
          alpha_dd = 0.0;
        }
      }
      // A leg that carries no offset keeps the planned reference bit for bit.
      if (_alpha[k] != 0.0 || _alpha_d[k] != 0.0 || alpha_dd != 0.0) {
        const int leg = sw[k];
        rotateReference(_n_body, prm.leg[leg].joint[0].parent_translation, _alpha[k], _alpha_d[k],
                        alpha_dd, in.foot_p[leg], in.foot_v[leg], in.foot_a[leg], out.foot_p[leg],
                        out.foot_v[leg], out.foot_a[leg]);
        // The foot does not go below its planned height. On the arc the two
        // swing feet go opposite ways (their hips lie on opposite sides of
        // the support line): the one swinging outward rises, the one swinging
        // in under its hip comes down, 3 cm below its planned height at 35
        // deg and on its way to the ground (2026-11-27). Where the arc is
        // lower, the planned height is kept and only the horizontal part of
        // the rotation is taken, which there differs from the arc's angle by
        // less than a degree. The rising foot is left on its arc: held level,
        // it reached half the angle asked for and the law's opening-angle
        // channel wound the other leg to its bound. The world vertical is
        // taken as constant in the body frame, as the support line is.
        const Eigen::Vector3d up = R.transpose() * Eigen::Vector3d::UnitZ();
        const double below = up.dot(in.foot_p[leg] - out.foot_p[leg]);
        if (below > 0.0) {
          out.foot_p[leg] += up * below;
          out.foot_v[leg] += up * up.dot(in.foot_v[leg] - out.foot_v[leg]);
          out.foot_a[leg] += up * up.dot(in.foot_a[leg] - out.foot_a[leg]);
        }
      }
      _log[16 + k] = _alpha[k];
      _log[18 + k] = _alpha_d[k];
    }
    _log[20] = acting ? g : 0.0;
  } else {
    // The share runs 1 -> 0 over the return window on the same quintic shape.
    double share = 0.0;
    if (phaseOn && valid) {
      double back, back_dot;
      TrajectoryUtils::sampleQuintic(1.0 - t_left / _p.ret, _p.ret, back, back_dot);
      share = g * (1.0 - back);
    }
    for (int k = 0; share > 0.0 && k < 2; k++) {
      Eigen::Matrix3d R_WP;
      double rho_m;
      if (!MdlAMB::principalFrame(pr.s, d_meas[k], R_WP, rho_m)) continue;
      Eigen::Vector3d a_f;
      MdlAMB::tangentialFeedforward(rho_m, thdd[k], R_WP.col(2), law.foot_acc_max, a_f);
      out.dir[sw[k]] = R_WP.col(2);
      out.share[sw[k]] = share;
      out.acc[sw[k]] = share * a_f;
    }
    _log[20] = share;
  }

  _log[1] = status;
  for (int k = 0; k < 2; k++) {
    _log[12 + k] = thdd[k];
    _log[14 + k] = dev[k];
    _log[22 + k] = rho[k];
  }
  _log[21] = _invalid;
}
