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
  MdlLegControl(int ind );
  ~MdlLegControl();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  void setTargetAngles( Eigen::Vector3d &a, Eigen::Vector3d &adot );  
  void setTargetPosition( Eigen::Vector3d &p, Eigen::Vector3d &pdot );
  
private:
  MotorHW *_motorhw = nullptr;
  MotorHW::cmd_t _cmd[3];

  int _indices[3];
  int _update_counter = 0;  // Counter for periodic joint position printing
  QuadrupedKinematics *_kinematics = nullptr;
  Eigen::Vector3d _footpos; // Target foot position in body frame
};
  
  
#endif
