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
