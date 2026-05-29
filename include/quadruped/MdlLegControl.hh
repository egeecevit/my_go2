/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef LEGMODULE_HH
#define LEGMODULE_HH

#include "rtcore/Module.hh"
#include "hardware/MotorHW.hh"
#include "Eigen/Dense"

#define LEGMODULE_NAME "MdlLegControl"

class QuadrupedKinematics;

class MdlLegControl : public rtcore::Module {
public:
  MdlLegControl(int ind);
  ~MdlLegControl();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  // Full MIT-mode joint-space command. Returns false if joint limits violated.
  bool setJointCommand(const Eigen::Vector3d &a, const Eigen::Vector3d &adot,
                       const Eigen::Vector3d &kp, const Eigen::Vector3d &kd,
                       const Eigen::Vector3d &tau_ff);

  // Foot-space target with joint-space gains. IK and damped Jacobian handled internally.
  bool setFootCommand(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot,
                      const Eigen::Vector3d &kp, const Eigen::Vector3d &kd,
                      const Eigen::Vector3d &tau_ff);

  // FK from current joint state. False if motors not STATUS_READY.
  bool getFootPosition(Eigen::Vector3d &pos) const;

  // Legacy wrappers using default gains. Now return bool.
  bool setTargetAngles(const Eigen::Vector3d &a, const Eigen::Vector3d &adot);
  bool setTargetPosition(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot);

private:
  MotorHW *_motorhw = nullptr;
  MotorHW::cmd_t _cmd[3];

  int _indices[3];
  QuadrupedKinematics *_kinematics = nullptr;

  Eigen::Vector3d _default_kp = Eigen::Vector3d(100.0, 100.0, 100.0);
  Eigen::Vector3d _default_kd = Eigen::Vector3d(5.0, 5.0, 5.0);
  double _jacobian_damping = 0.01;

  void _emitCommands();
};

#endif
