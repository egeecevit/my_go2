/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */


#include <stdio.h>

#include <cmath>

#include "hardware/MotorHW.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedConfigs.hh"

#define DBGPRINT(...) //printf(__VA_ARGS__)

MdlLegControl::MdlLegControl(int ind) : Module(LEGMODULE_NAME, ind, SINGLE_USER) {
  DBGPRINT("MdlLegControl[%d]::MdlLegControl\n", getIndex());
};

MdlLegControl::~MdlLegControl() { DBGPRINT("MdlLegControl[%d]::~MdlLegControl\n", getIndex()); };

void MdlLegControl::init() {
  DBGPRINT("MdlLegControl[%d]::init\n", getIndex());

  _motorhw = MotorHW::instance();
  for (int i = 0; i < 3; i++) _indices[i] = 3 * getIndex() + i;

  // The _mgr guards let a unit test call init() on a bare instance to get the
  // kinematics member, which is all computeCartesianForceCommand() needs. Under
  // the ModuleManager _mgr is never null, so this costs nothing in operation.
  rtcore::ConfigTable hwConfig;
  std::string hwlib;
  if (_mgr && _mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");

  if (hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

  rtcore::ConfigTable lcConfig;
  if (_mgr && _mgr->getConfigTable("legcontrol", lcConfig)) {
    double kp = lcConfig.getDouble("default_kp", 100.0);
    double kd = lcConfig.getDouble("default_kd", 5.0);
    _default_kp = Eigen::Vector3d(kp, kp, kp);
    _default_kd = Eigen::Vector3d(kd, kd, kd);
    _jacobian_damping = lcConfig.getDouble("jacobian_damping", 0.01);
  }
}

void MdlLegControl::uninit() {
  DBGPRINT("MdlLegControl[%d]::uninit\n", getIndex());
  if (_kinematics) delete _kinematics;
  _kinematics = nullptr;
}

void MdlLegControl::activate() {
  DBGPRINT("MdlLegControl[%d]::activate\n", getIndex());

  MotorHW::state_t state;

  for (int i = 0; i < 3; i++) {
    _motorhw->grab(_indices[i]);
    _motorhw->getState(_indices[i], state);
    _cmd[i].pos = state.pos;
    _cmd[i].vel = 0.0;
    _cmd[i].tau = 0.0;
    _cmd[i].kp = _default_kp[i];
    _cmd[i].kd = _default_kd[i];
    // Stage command before enabling — no stale-command gap
    _motorhw->setCommand(_indices[i], _cmd[i]);
    _motorhw->enable(_indices[i]);
  }
}

void MdlLegControl::deactivate() {
  DBGPRINT("MdlLegControl[%d]::deactivate\n", getIndex());

  for (int i = 0; i < 3; i++) {
    _motorhw->release(_indices[i]);
    _motorhw->disable(_indices[i]);
  }
}

void MdlLegControl::update() {
}

void MdlLegControl::_emitCommands() {
  for (int i = 0; i < 3; i++)
    _motorhw->setCommand(_indices[i], _cmd[i]);
}

bool MdlLegControl::setJointCommand(const Eigen::Vector3d &a, const Eigen::Vector3d &adot,
                                     const Eigen::Vector3d &kp, const Eigen::Vector3d &kd,
                                     const Eigen::Vector3d &tau_ff) {
  if (!_kinematics->checkJointLimits(getIndex(), a))
    return false;

  for (int i = 0; i < 3; i++) {
    _cmd[i].pos = a[i];
    _cmd[i].vel = adot[i];
    _cmd[i].kp = kp[i];
    _cmd[i].kd = kd[i];
    _cmd[i].tau = tau_ff[i];
  }
  _emitCommands();
  return true;
}

bool MdlLegControl::setFootCommand(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot,
                                    const Eigen::Vector3d &kp, const Eigen::Vector3d &kd,
                                    const Eigen::Vector3d &tau_ff) {
  Eigen::Vector3d a, adot;
  Eigen::Matrix3d J;

  if (!_kinematics->inverseKinematics(getIndex(), p, a))
    return false;
  if (!_kinematics->jacobian(getIndex(), a, J))
    return false;

  // Damped pseudoinverse: J^T (J J^T + lambda^2 I)^{-1}
  double lam2 = _jacobian_damping * _jacobian_damping;
  Eigen::Matrix3d JJT = J * J.transpose();
  JJT.diagonal().array() += lam2;
  adot = J.transpose() * JJT.inverse() * pdot;

  return setJointCommand(a, adot, kp, kd, tau_ff);
}

bool MdlLegControl::computeCartesianForceCommand(
    const Eigen::Vector3d &q, const Eigen::Vector3d &qdot,
    const Eigen::Vector3d &position_ref_body, const Eigen::Vector3d &velocity_ref_body,
    const Eigen::Vector3d &kp_cartesian, const Eigen::Vector3d &kd_cartesian,
    const Eigen::Vector3d &force_feedforward_body, Eigen::Vector3d &tau_ff) const {
  if (!_kinematics) return false;

  // Reject bad input before touching the kinematics, so a NaN cannot reach the
  // motors through some intermediate that happens to stay finite.
  if (!q.allFinite() || !qdot.allFinite() || !position_ref_body.allFinite() ||
      !velocity_ref_body.allFinite() || !kp_cartesian.allFinite() ||
      !kd_cartesian.allFinite() || !force_feedforward_body.allFinite())
    return false;

  // TODO(ege): implement
  //
  // - From the measured q: forward kinematics gives the foot position p and
  //   jacobian() gives J, and then v = J*qdot -- never the IK-desired joint
  //   velocity, which is a command rather than a measurement. Use the unchecked
  //   FK: measured angles can sit a little outside the configured limits and
  //   must not be rejected for it.
  // - f_cmd = kp.*(p_ref - p) + kd.*(v_ref - v) + f_feedforward, all in the
  //   body frame, and tau_ff = J.transpose() * f_cmd.
  // - Gotcha: f_feedforward is the force the *actuators apply at the foot*. The
  //   caller has already negated and rotated the ground reaction force, so do
  //   not flip the sign again here.
  // - Reject a non-finite result -- an unreachable gain or a degenerate
  //   configuration can produce one from finite inputs -- and return false
  //   without writing anything the caller would emit.

  Eigen::Vector3d p;
  Eigen::Matrix3d J;

  const int leg = _indices[0] / 3;
  if (!_kinematics->forwardKinematicsUnchecked(leg, q, p))
    return false;
  if (!_kinematics->jacobian(leg, q, J))
    return false;

  Eigen::Vector3d v = J * qdot;

  const Eigen::Vector3d f_cmd = kp_cartesian.cwiseProduct(position_ref_body - p) + kd_cartesian.cwiseProduct(velocity_ref_body - v) + force_feedforward_body;

  const Eigen::Vector3d tau = J.transpose() * f_cmd;

  if (!tau.allFinite())
    return false;

  tau_ff = tau;
  return true;
}

bool MdlLegControl::setCartesianForceCommand(const Eigen::Vector3d &position_ref_body,
                                             const Eigen::Vector3d &velocity_ref_body,
                                             const Eigen::Vector3d &kp_cartesian,
                                             const Eigen::Vector3d &kd_cartesian,
                                             const Eigen::Vector3d &force_feedforward_body,
                                             double joint_damping) {
  if (!_motorhw) return false;

  // New behavior for a setter on this module: the existing ones emit whatever
  // they are given and let the hardware sort it out. This one cannot, because
  // it reads the joint state it is about to close a loop around, and a motor
  // that is not READY reports a position that means nothing. Added deliberately
  // rather than copied from getFootPosition(), which checks for the same reason.
  for (int i = 0; i < 3; i++)
    if (_motorhw->getStatus(_indices[i]) != MotorHW::STATUS_READY) return false;

  // Same motor index convention as MdlPosVelEstimator::_readSensors: 3*leg + j.
  Eigen::Vector3d q, qdot;
  for (int i = 0; i < 3; i++) {
    MotorHW::state_t state;
    _motorhw->getState(_indices[i], state);
    q[i] = state.pos;
    qdot[i] = state.vel;
  }

  Eigen::Vector3d tau_ff;
  if (!computeCartesianForceCommand(q, qdot, position_ref_body, velocity_ref_body,
                                    kp_cartesian, kd_cartesian, force_feedforward_body,
                                    tau_ff))
    return false;

  if (!(joint_damping > 0.0) || !std::isfinite(joint_damping)) return false;

  // Emitted through the ordinary joint command path so there is exactly one
  // place that writes MotorHW. Desired position is the measured one and the
  // joint kp is zero, which makes the position term vanish and leaves the
  // damping plus the feedforward torque -- the joint-space form of a pure
  // Cartesian force command.
  //
  // The damping has to be strictly positive: MdlSimDriver substitutes 5.0 for
  // any non-positive kd and does it by mutating the stored command, so the
  // substitution outlives the command that triggered it.
  static const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return setJointCommand(q, zero, zero, Eigen::Vector3d::Constant(joint_damping), tau_ff);
}

bool MdlLegControl::getFootPosition(Eigen::Vector3d &pos) const {
  for (int i = 0; i < 3; i++) {
    if (_motorhw->getStatus(_indices[i]) != MotorHW::STATUS_READY)
      return false;
  }

  Eigen::Vector3d angles;
  for (int i = 0; i < 3; i++) {
    MotorHW::state_t state;
    _motorhw->getState(_indices[i], state);
    angles[i] = state.pos;
  }

  int leg = _indices[0] / 3;
  return _kinematics->forwardKinematicsUnchecked(leg, angles, pos);
}

bool MdlLegControl::setTargetAngles(const Eigen::Vector3d &a, const Eigen::Vector3d &adot) {
  static const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return setJointCommand(a, adot, _default_kp, _default_kd, zero);
}

bool MdlLegControl::setTargetPosition(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot) {
  static const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return setFootCommand(p, pdot, _default_kp, _default_kd, zero);
}
