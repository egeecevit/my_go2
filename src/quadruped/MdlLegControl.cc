/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */


#include <stdio.h>

#include <algorithm>
#include <cmath>

#include "hardware/MotorHW.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedLegDynamics.hh"

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
  std::string robotModel;
  if (_mgr && _mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");

  if (_mgr)
    robotModel = hwConfig.getString(
        "robot_model", hwlib == "simhw" ? "go2" : (hwlib == "go1hw" ? "go1" : ""));
  else
    robotModel = "go2";  // Bare unit-test instances exercise the populated model.

  if (robotModel == "go1" || hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

  if (robotModel == "go2") {
    _dynamics = new QuadrupedLegDynamics(createGo2DynamicsConfig());
    if (!_dynamics->isValid()) {
      delete _dynamics;
      _dynamics = nullptr;
      if (_mgr) _mgr->warning(LEGMODULE_NAME, "Go2 leg dynamics parameters are invalid");
    } else
      _torqueLimit = _dynamics->getParams().torque_limit;
  } else if (_mgr) {
    _mgr->warning(LEGMODULE_NAME,
                  "No inverse-dynamics parameters for robot_model '%s'; "
                  "swing feedforward is unavailable",
                  robotModel.c_str());
  }

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
  delete _dynamics;
  _dynamics = nullptr;
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

bool MdlLegControl::hasSwingDynamics() const {
  return _dynamics && _dynamics->isValid();
}

bool MdlLegControl::_limitTorque(const Eigen::Vector3d& qdot,
                                 const Eigen::Vector3d& task_torque,
                                 double joint_damping,
                                 command_result_t& result) const {
  if (!qdot.allFinite() || !task_torque.allFinite() ||
      !(joint_damping >= 0.0) || !std::isfinite(joint_damping))
    return false;

  result.requested_torque = task_torque - joint_damping * qdot;
  const Eigen::Vector3d& limits = _torqueLimit;
  double scale = 1.0;
  for (int i = 0; i < 3; ++i) {
    const double magnitude = std::fabs(result.requested_torque[i]);
    if (magnitude > limits[i]) scale = std::min(scale, limits[i] / magnitude);
  }
  result.torque_scale = scale;
  result.applied_torque = scale * result.requested_torque;
  result.motor_feedforward_torque = result.applied_torque + joint_damping * qdot;
  return result.requested_torque.allFinite() && result.applied_torque.allFinite() &&
         result.motor_feedforward_torque.allFinite() && std::isfinite(scale) && scale > 0.0;
}

bool MdlLegControl::computeSwingCommand(
    const Eigen::Vector3d& q, const Eigen::Vector3d& qdot,
    const Eigen::Vector3d& position_ref_body,
    const Eigen::Vector3d& velocity_ref_body,
    const Eigen::Vector3d& acceleration_ref_body, const Eigen::Vector3d& gravity_body,
    const Eigen::Vector3d& natural_frequency, const Eigen::Vector3d& damping_ratio,
    double feedforward_scale, double joint_damping, command_result_t& result) const {
  if (!_dynamics || !q.allFinite() || !qdot.allFinite() ||
      !position_ref_body.allFinite() || !velocity_ref_body.allFinite() ||
      !acceleration_ref_body.allFinite() || !gravity_body.allFinite() ||
      !natural_frequency.allFinite() || !damping_ratio.allFinite() ||
      (natural_frequency.array() <= 0.0).any() || (damping_ratio.array() <= 0.0).any() ||
      !(feedforward_scale >= 0.0) || !std::isfinite(feedforward_scale))
    return false;

  QuadrupedLegDynamics::terms_t terms;
  if (!_dynamics->compute(_indices[0] / 3, q, qdot, gravity_body, terms)) return false;

  result = command_result_t();
  result.kp_cartesian = natural_frequency.array().square() *
                        terms.operational_inertia.diagonal().array();
  result.kd_cartesian = 2.0 * damping_ratio.array() * natural_frequency.array() *
                        terms.operational_inertia.diagonal().array();
  const Eigen::Vector3d velocity = terms.foot_jacobian * qdot;
  result.feedback_torque =
      terms.foot_jacobian.transpose() *
      (result.kp_cartesian.cwiseProduct(position_ref_body - terms.foot_position) +
       result.kd_cartesian.cwiseProduct(velocity_ref_body - velocity));
  result.feedforward_torque = feedforward_scale *
      (terms.foot_jacobian.transpose() * terms.operational_inertia *
           (acceleration_ref_body - terms.jdot_qdot) +
       terms.bias);
  if (!result.kp_cartesian.allFinite() || !result.kd_cartesian.allFinite() ||
      !result.feedback_torque.allFinite() || !result.feedforward_torque.allFinite())
    return false;

  return _limitTorque(qdot, result.feedback_torque + result.feedforward_torque,
                      joint_damping, result);
}

bool MdlLegControl::_readJointState(Eigen::Vector3d& q, Eigen::Vector3d& qdot) const {
  if (!_motorhw) return false;
  for (int i = 0; i < 3; ++i) {
    if (_motorhw->getStatus(_indices[i]) != MotorHW::STATUS_READY) return false;
    MotorHW::state_t state;
    _motorhw->getState(_indices[i], state);
    q[i] = state.pos;
    qdot[i] = state.vel;
  }
  return q.allFinite() && qdot.allFinite();
}

bool MdlLegControl::_emitTorqueCommand(const Eigen::Vector3d& q, double joint_damping,
                                       const command_result_t& result) {
  // A Cartesian torque command must remain safe even after a physical limit
  // has moved a measured joint outside the software position window.  Calling
  // setJointCommand() here would reject that sample and retain the previous
  // motor command, possibly a larger torque.  The commanded position has zero
  // gain, so it is informational only; retain physical limits in the motor
  // interface and emit the bounded torque directly.
  if (!q.allFinite() || !result.motor_feedforward_torque.allFinite() ||
      !std::isfinite(joint_damping) || joint_damping <= 0.0)
    return false;

  for (int i = 0; i < 3; ++i) {
    _cmd[i].pos = q[i];
    _cmd[i].vel = 0.0;
    _cmd[i].kp = 0.0;
    _cmd[i].kd = joint_damping;
    _cmd[i].tau = result.motor_feedforward_torque[i];
  }
  _emitCommands();
  return true;
}

bool MdlLegControl::setSwingCommand(
    const Eigen::Vector3d& position_ref_body,
    const Eigen::Vector3d& velocity_ref_body,
    const Eigen::Vector3d& acceleration_ref_body, const Eigen::Vector3d& gravity_body,
    const Eigen::Vector3d& natural_frequency, const Eigen::Vector3d& damping_ratio,
    double feedforward_scale, double joint_damping, command_result_t* result) {
  Eigen::Vector3d q, qdot;
  if (!_readJointState(q, qdot)) return false;
  command_result_t command;
  if (!computeSwingCommand(q, qdot, position_ref_body, velocity_ref_body,
                           acceleration_ref_body, gravity_body, natural_frequency,
                           damping_ratio, feedforward_scale, joint_damping, command))
    return false;
  if (!_emitTorqueCommand(q, joint_damping, command)) return false;
  if (result) *result = command;
  return true;
}

bool MdlLegControl::setCartesianForceCommand(const Eigen::Vector3d &position_ref_body,
                                             const Eigen::Vector3d &velocity_ref_body,
                                             const Eigen::Vector3d &kp_cartesian,
                                             const Eigen::Vector3d &kd_cartesian,
                                             const Eigen::Vector3d &force_feedforward_body,
                                             double joint_damping,
                                             command_result_t *result) {
  Eigen::Vector3d q, qdot;
  if (!_readJointState(q, qdot)) return false;

  Eigen::Vector3d task_torque;
  if (!computeCartesianForceCommand(q, qdot, position_ref_body, velocity_ref_body,
                                    kp_cartesian, kd_cartesian, force_feedforward_body,
                                    task_torque))
    return false;

  command_result_t command;
  command.kp_cartesian = kp_cartesian;
  command.kd_cartesian = kd_cartesian;
  command.feedback_torque = task_torque;
  if (!_limitTorque(qdot, task_torque, joint_damping, command)) return false;
  if (!_emitTorqueCommand(q, joint_damping, command)) return false;
  if (result) *result = command;
  return true;
}

bool MdlLegControl::getFootPosition(Eigen::Vector3d &pos) const {
  Eigen::Vector3d velocity;
  return getFootState(pos, velocity);
}

bool MdlLegControl::getFootState(Eigen::Vector3d& pos, Eigen::Vector3d& vel) const {
  Eigen::Vector3d q, qdot;
  if (!_readJointState(q, qdot)) return false;
  Eigen::Matrix3d jacobian;
  const int leg = _indices[0] / 3;
  if (!_kinematics->forwardKinematicsUnchecked(leg, q, pos) ||
      !_kinematics->jacobian(leg, q, jacobian))
    return false;
  vel = jacobian * qdot;
  return pos.allFinite() && vel.allFinite();
}

bool MdlLegControl::setTargetAngles(const Eigen::Vector3d &a, const Eigen::Vector3d &adot) {
  static const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return setJointCommand(a, adot, _default_kp, _default_kd, zero);
}

bool MdlLegControl::setTargetPosition(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot) {
  static const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return setFootCommand(p, pdot, _default_kp, _default_kd, zero);
}
