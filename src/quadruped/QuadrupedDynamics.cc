/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#include "quadruped/QuadrupedDynamics.hh"

namespace {

// skew(a) b = a x b.
Eigen::Matrix3d skew(const Eigen::Vector3d& a) {
  Eigen::Matrix3d s;
  s << 0.0, -a(2), a(1), a(2), 0.0, -a(0), -a(1), a(0), 0.0;
  return s;
}

}  // namespace

QuadrupedDynamics::QuadrupedDynamics(const params_t& params)
    : _params(params), _legs(params.legs) {}

bool QuadrupedDynamics::compute(const state_t& state, terms_t& out) const {
  const Eigen::Matrix3d& R = state.R;
  const Eigen::Vector3d& w = state.w_body;
  // v_base enters no output (the world-axes base velocity creates no velocity
  // product), but a non-finite one still means a broken caller.
  if (!R.allFinite() || !state.v_base.allFinite() || !w.allFinite()) return false;
  for (int leg = 0; leg < NUM_LEGS; leg++)
    if (!state.qd[leg].allFinite()) return false;  // linkKinematics checks q

  const QuadrupedLegDynamics::params_t& lp = _params.legs;
  const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
  // Everything below is in BODY axes, as linkKinematics returns it; R only
  // enters through the world-axes base velocity and gravity. For a body with
  // COM c, COM Jacobian Jc and angular Jacobian Jw over its leg's joints,
  //   COM velocity  (body axes) = A v,  A = [R^T, -[c]x, Jc],
  //   angular rate  (body axes) = B v,  B = [0,    I,     Jw],
  // and Kane's equations give the terms of (6.1) as sums over the 13 bodies:
  //   M = sum m A^T A + B^T I B,
  //   h = sum A^T m (a + g R^T e_z) + B^T (I alpha + omega x I omega),
  // a and alpha being the body's COM and angular accelerations at vdot = 0
  // (the bias of the chain recursion below), inertial and in body axes.
  const Eigen::Vector3d g_up = _params.gravity * R.row(2).transpose();  // g R^T e_z
  out.M.setZero();
  out.h.setZero();
  Jacobian A = Jacobian::Zero(), B = Jacobian::Zero();
  A.block<3, 3>(0, 0) = R.transpose();
  B.block<3, 3>(0, 3).setIdentity();
  auto addBody = [&](double m, const Eigen::Vector3d& c, const Eigen::Matrix3d& I,
                     const Eigen::Vector3d& a, const Eigen::Vector3d& alpha,
                     const Eigen::Vector3d& omega) {
    A.block<3, 3>(0, 3) = -skew(c);
    out.M.noalias() += m * A.transpose() * A + B.transpose() * I * B;
    out.h.noalias() +=
        A.transpose() * (m * (a + g_up)) + B.transpose() * (I * alpha + omega.cross(I * omega));
  };

  // The base origin's acceleration is vdot[0:3] and the trunk's angular
  // acceleration R wdot (d/dt (R w) = R (w x w + wdot)), so the trunk's only
  // bias is the centripetal term of its COM.
  const Eigen::Vector3d& c_trunk = _params.trunk_com;
  addBody(_params.trunk_mass, c_trunk, _params.trunk_inertia_com, w.cross(w.cross(c_trunk)),
          Eigen::Vector3d::Zero(), w);
  Eigen::Vector3d mc = _params.trunk_mass * c_trunk;
  out.mass = _params.trunk_mass;

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    QuadrupedLegDynamics::kinematics_t kin;
    if (!_legs.linkKinematics(leg, state.q[leg], kin)) return false;
    const Eigen::Vector3d& qd = state.qd[leg];
    const int col = 6 + 3 * leg;

    // Bias recursion down the chain (RNEA forward pass at vdot = 0). Anchor
    // i is fixed in link i-1 (the trunk for i = 0, whose origin is the base
    // origin with zero bias); its axis is fixed there too, so it turns at
    // omega_{i-1} and adds omega_{i-1} x axis_i qd_i to the link's alpha.
    Eigen::Vector3d omega = w, alpha = Eigen::Vector3d::Zero();
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), a_p = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
      const Eigen::Vector3d d = kin.joint_position[i] - p;
      a_p += alpha.cross(d) + omega.cross(omega.cross(d));
      p = kin.joint_position[i];
      const Eigen::Vector3d s = kin.joint_axis[i] * qd(i);
      alpha += omega.cross(s);
      omega += s;

      const QuadrupedLegDynamics::link_t& link = lp.leg[leg].joint[i].link;
      const Eigen::Vector3d& c = kin.com_position[i];
      const Eigen::Vector3d r = c - p;
      const Eigen::Matrix3d& Rl = kin.link_rotation[i];
      A.block<3, 3>(0, col) = kin.com_linear_jacobian[i];
      B.block<3, 3>(0, col) = kin.com_angular_jacobian[i];
      addBody(link.mass, c, Rl * link.inertia_com * Rl.transpose(),
              a_p + alpha.cross(r) + omega.cross(omega.cross(r)), alpha, omega);
      mc += link.mass * c;
      out.mass += link.mass;
    }
    A.block<3, 3>(0, col).setZero();
    B.block<3, 3>(0, col).setZero();
    // Reflected rotor inertia only scales the joint's own acceleration, as
    // MuJoCo's armature does (it adds nothing to h).
    out.M.diagonal().segment<3>(col) += lp.armature;

    // Foot sphere centre, fixed in the calf (link 2): omega, alpha, p, a_p
    // are the calf's after the loop.
    const Eigen::Vector3d r = kin.foot_position - p;
    point_t& foot = out.foot[leg];
    foot.p = R * kin.foot_position;
    foot.J.setZero();
    foot.J.block<3, 3>(0, 0).setIdentity();
    foot.J.block<3, 3>(0, 3) = -R * skew(kin.foot_position);
    foot.J.block<3, 3>(0, col) = R * kin.foot_jacobian;
    foot.Jdv = R * (a_p + alpha.cross(r) + omega.cross(omega.cross(r)));

    // Rolling contact (6.2): J v = v_centre + r e_z x omega_calf (header). e_z is
    // constant in the world, so d/dt (J v) = a_centre + r e_z x alpha_calf:
    // the contact point slides over the sphere, which is why this is not the
    // acceleration of the material point under the centre (that one adds
    // omega x (omega x (-r e_z))).
    Jacobian Jw = Jacobian::Zero();  // calf angular velocity, world axes
    Jw.block<3, 3>(0, 3) = R;
    Jw.block<3, 3>(0, col) = R * kin.com_angular_jacobian[2];
    const double rf = _params.foot_radius;
    point_t& contact = out.contact[leg];
    contact.p = foot.p - rf * ez;
    contact.J = foot.J + rf * skew(ez) * Jw;
    contact.Jdv = foot.Jdv + rf * ez.cross(R * alpha);

    // Abduction anchor, fixed in the trunk: centripetal bias only.
    const Eigen::Vector3d& h0 = kin.joint_position[0];
    point_t& hip = out.hip[leg];
    hip.p = R * h0;
    hip.J.setZero();
    hip.J.block<3, 3>(0, 0).setIdentity();
    hip.J.block<3, 3>(0, 3) = -R * skew(h0);
    hip.Jdv = R * w.cross(w.cross(h0));
  }

  out.com = R * mc / out.mass;
  return true;
}
