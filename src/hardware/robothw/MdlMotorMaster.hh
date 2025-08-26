/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef MDLMOTORMASTER_HH
#define MDLMOTORMASTER_HH

#include "hardware/MotorHW.hh"
#include "rtcore/Module.hh"

#define MOTORMASTER_NAME "MdlMotorMaster"

class CubeMARSCAN;

// TODO: This should be configurable and read from a config file
#define MOTORMASTER_CANCNT 4

class MdlMotorMaster : public rtcore::Module {
 public:
  MdlMotorMaster();
  ~MdlMotorMaster();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  void getJointState(unsigned int index, MotorHW::state_t &state);
  void setJointCommand(unsigned int index, MotorHW::cmd_t &cmd);
  void getJointCommand(unsigned int index, MotorHW::cmd_t &cmd);

 private:
  typedef struct {
    CubeMARSCAN *bus = nullptr;
    int index = -1;  // Index within the CubeMARSCAN object
  } caninfo_t;

  CubeMARSCAN *_can[MOTORMASTER_CANCNT];
  caninfo_t _motors[12];
};

#endif
