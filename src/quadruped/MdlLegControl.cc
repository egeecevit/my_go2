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
#include "quadruped/QuadrupedKinematics.hh"

#define DBGPRINT(...) //printf(__VA_ARGS__)

QuadrupedKinematics::params_t createGo2Config() {
  QuadrupedKinematics::params_t params;

  // Robot identification
  params.robot_name = "Unitree Go2";

  // Hip positions in body frame [x, y, z] for each leg (meters)
  // Go2 dimensions: body length ~0.387m, body width ~0.093m
  params.hip_positions << 0.1934, 0.0465, 0.0,  // Front Left
      0.1934, -0.0465, 0.0,                     // Front Right
      -0.1934, 0.0465, 0.0,                     // Rear Left
      -0.1934, -0.0465, 0.0;                    // Rear Right

  // Link lengths [thigh_length, calf_length] for each leg (meters)
  // Go2 link lengths: thigh ~0.213m, calf ~0.213m
  params.link_lengths << 0.213, 0.213,  // Front Left
      0.213, 0.213,                     // Front Right
      0.213, 0.213,                     // Rear Left
      0.213, 0.213;                     // Rear Right

  // Hip flexion axis offset for each leg (meters)
  params.hip_flexion_offset << 0.0955,  // Front Left
      -0.0955,                          // Front Right
      0.0955,                           // Rear Left
      -0.0955;                          // Rear Right

  // Joint limits [min, max] in radians
  // Hip abduction limits (approximately ±60 degrees)
  params.hip_abduction_limits << -1.0472, 1.0472,  // Front Left
      -1.0472, 1.0472,                             // Front Right
      -1.0472, 1.0472,                             // Rear Left
      -1.0472, 1.0472;                             // Rear Right

  // Hip flexion limits (different for front and rear legs)
  params.hip_flexion_limits << -1.5708, 3.4907,  // Front Left (-90° to +200°)
      -1.5708, 3.4907,                           // Front Right (-90° to +200°)
      -0.5236, 4.5379,                           // Rear Left (-30° to +260°)
      -0.5236, 4.5379;                           // Rear Right (-30° to +260°)

  // Knee limits (approximately -156° to -48° degrees)
  params.knee_limits << -2.7227, -0.83776,  // Front Left
      -2.7227, -0.83776,                    // Front Right
      -2.7227, -0.83776,                    // Rear Left
      -2.7227, -0.83776;                    // Rear Right

  // Joint directions (1.0 or -1.0) for sign conventions
  // [hip_abduction, hip_flexion, knee] directions per leg
  params.joint_directions << 1.0, 1.0, 1.0,  // Front Left
      -1.0, 1.0, 1.0,                        // Front Right
      1.0, 1.0, 1.0,                         // Rear Left
      -1.0, 1.0, 1.0;                        // Rear Right

  return params;
}

QuadrupedKinematics::params_t createGo1Config() {
  QuadrupedKinematics::params_t params;

  params.robot_name = "Unitree Go1";

  // Go1 dimensions: body length ~0.3762m, body width ~0.0935m
  params.hip_positions << 0.1881, 0.04675, 0.0,   // Front Left
      0.1881, -0.04675, 0.0,                       // Front Right
      -0.1881, 0.04675, 0.0,                       // Rear Left
      -0.1881, -0.04675, 0.0;                      // Rear Right

  // Go1 link lengths: thigh ~0.213m, calf ~0.213m (same as Go2)
  params.link_lengths << 0.213, 0.213,
      0.213, 0.213,
      0.213, 0.213,
      0.213, 0.213;

  // Go1 hip flexion offset: 0.08m
  params.hip_flexion_offset << 0.08,
      -0.08,
      0.08,
      -0.08;

  // Joint limits from go1_const.h
  // Hip abduction: +/-1.047 rad (+/-60 degrees)
  params.hip_abduction_limits << -1.047, 1.047,
      -1.047, 1.047,
      -1.047, 1.047,
      -1.047, 1.047;

  // Thigh: -0.663 to 2.966 rad
  params.hip_flexion_limits << -0.663, 2.966,
      -0.663, 2.966,
      -0.663, 2.966,
      -0.663, 2.966;

  // Calf: -2.721 to -0.837 rad
  params.knee_limits << -2.721, -0.837,
      -2.721, -0.837,
      -2.721, -0.837,
      -2.721, -0.837;

  // Joint directions (same convention as Go2)
  params.joint_directions << 1.0, 1.0, 1.0,
      -1.0, 1.0, 1.0,
      1.0, 1.0, 1.0,
      -1.0, 1.0, 1.0;

  return params;
}

MdlLegControl::MdlLegControl(int ind) : Module(LEGMODULE_NAME, ind, SINGLE_USER) {
  DBGPRINT("MdlLegControl[%d]::MdlLegControl\n", getIndex());
};

MdlLegControl::~MdlLegControl() { DBGPRINT("MdlLegControl[%d]::~MdlLegControl\n", getIndex()); };

void MdlLegControl::init() {
  DBGPRINT("MdlLegControl[%d]::init\n", getIndex());

  _motorhw = MotorHW::instance();
  for (int i = 0; i < 3; i++) _indices[i] = 3 * getIndex() + i;

  rtcore::ConfigTable hwConfig;
  std::string hwlib;
  if (_mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");

  if (hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

  rtcore::ConfigTable lcConfig;
  if (_mgr->getConfigTable("legcontrol", lcConfig)) {
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
  return _kinematics->forwardKinematics(leg, angles, pos);
}

bool MdlLegControl::setTargetAngles(const Eigen::Vector3d &a, const Eigen::Vector3d &adot) {
  static const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return setJointCommand(a, adot, _default_kp, _default_kd, zero);
}

bool MdlLegControl::setTargetPosition(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot) {
  static const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  return setFootCommand(p, pdot, _default_kp, _default_kd, zero);
}
