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

MdlLegControl::MdlLegControl(int ind) : Module(LEGMODULE_NAME, ind, SINGLE_USER) {
  DBGPRINT("MdlLegControl[%d]::MdlLegControl\n", getIndex());
};

MdlLegControl::~MdlLegControl() { DBGPRINT("MdlLegControl[%d]::~MdlLegControl\n", getIndex()); };

void MdlLegControl::init() {
  DBGPRINT("MdlLegControl[%d]::init\n", getIndex());

  _motorhw = MotorHW::instance();
  for (int i = 0; i < 3; i++) _indices[i] = 3 * getIndex() + i;

  _kinematics = new QuadrupedKinematics(createGo2Config());
}

void MdlLegControl::uninit() { 
  DBGPRINT("MdlLegControl[%d]::uninit\n", getIndex()); 
  if (_kinematics) delete _kinematics;
  _kinematics = nullptr;
}

void MdlLegControl::activate() {
  DBGPRINT("MdlLegControl[%d]::activate\n", getIndex());

  MotorHW::state_t state;

  // Each module uses only the three motors for the corresponding leg
  for (int i = 0; i < 3; i++) {
    _motorhw->grab(_indices[i]);
    _motorhw->enable(_indices[i]);
    _motorhw->getState(_indices[i], state);
    _cmd[i].kp = 100.0;
    _cmd[i].pos = state.pos + M_PI / 6;  // Set initial position to current position
    _cmd[i].kd = 5;
    _cmd[i].vel = 0.0;
    _cmd[i].tau = 0.0;
  }
}

void MdlLegControl::deactivate() {
  DBGPRINT("MdlLegControl[%d]::deactivate\n", getIndex());

  // Each module uses only the three motors for the corresponding leg
  for (int i = 0; i < 3; i++) {
    _motorhw->release(_indices[i]);
    _motorhw->disable(_indices[i]);
  }
}

void MdlLegControl::update() {
  MotorHW::state_t state;

  // Increment update counter
  _update_counter++;

  // _footpos = _kinematics->getKinematicParams().hip_positions.row(getIndex()).transpose();
  // _footpos[0] += 0.1 * sin(0.5 * M_PI * _mgr->readTime()) - 0.03;
  // _footpos[1] +=
  //     0.1 * (getIndex() % 2 == 0 ? 1 : -1) + 0.05 * cos(0.5 * M_PI * _mgr->readTime());
  // _footpos[2] -= 0.26 + 0.04 * sin(1.7 * M_PI * _mgr->readTime());
  // Eigen::Vector3d joint_angles;
  // bool res = _kinematics->inverseKinematics(getIndex(), _footpos, joint_angles);

  // if (res) {
  //   for (int i = 0; i < 3; i++) {
  //     _cmd[i].pos = joint_angles[i];
  //     _motorhw->setCommand(_indices[i], _cmd[i]);
  //   }
  // }
}

void MdlLegControl::setTargetAngles( Eigen::Vector3d &a, Eigen::Vector3d &adot ) {
  DBGPRINT("MdlLegControl[%d]::setTargetAngles [%f,%f,%f], [%f,%f,%f]\n", getIndex(),
           a[0], a[1], a[2], adot[0], adot[1], adot[2]);

  for (int i = 0; i < 3; i++) {
    _cmd[i].pos = a[i];
    _cmd[i].vel = adot[i];
    _motorhw->setCommand(_indices[i], _cmd[i]);
  }
}

void MdlLegControl::setTargetPosition( Eigen::Vector3d &p, Eigen::Vector3d &pdot ) {
  DBGPRINT("MdlLegControl[%d]::setTargetPosition [%f,%f,%f], [%f,%f,%f]\n", getIndex(),
           p[0], p[1], p[2], pdot[0], pdot[1], pdot[2]);

  Eigen::Vector3d a, adot;
  Eigen::Matrix3d J, Jinv;
  if (!_kinematics->inverseKinematics(getIndex(), p, a)) return;
  if (!_kinematics->jacobian(getIndex(), a, J)) return;
  adot = J.inverse() * pdot;
  setTargetAngles(a, adot);
}
