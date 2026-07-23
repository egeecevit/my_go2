/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include "rtcore/ModuleManager.hh"
#include "MdlMotorMaster.hh"
#include "CubeMARSCAN.hh"

using namespace rtcore;

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...) //printf(__VA_ARGS__);

MdlMotorMaster::MdlMotorMaster() : Module(MOTORMASTER_NAME, 0, SINGLE_USER ) {
  for (int i = 0; i < MOTORMASTER_CANCNT; i++) _can[i] = nullptr;
}

MdlMotorMaster::~MdlMotorMaster() {};

void MdlMotorMaster::init() {
  // Create CubeMARSCAN objects
  
  for (int i = 0; i < MOTORMASTER_CANCNT; i++) {

    // TODO: This configuration should be read from a config file
    CubeMARSCAN::config_t c;
    CubeMARSCAN::axis_t   a;
    caninfo_t info;
    
    for (int j= 0; j < 3; j++) {
      a.canid = j;
      a.motorid = 3*i+j;
      a.polarity = 1;
      a.model = "AK10-9";
      c.axes.push_back(a);
    }
      
    char devname[128];
    snprintf(devname, 128, "can%d", i);
    c.intf = devname;
    _can[i] = new CubeMARSCAN( c, _mgr );

    ConfigTable tc(_mgr->getConfigRoot(), "threads");
    _can[i]->start( devname, tc.getInt("can_priority", 98 ));

    for (int j= 0; j < 3; j++) {
      a = c.axes[j];
      info.bus = _can[i];
      info.index = j;
      _motors[a.motorid] = info;
    }
  }
}

void MdlMotorMaster::uninit() {
  for (int i = 0; i < MOTORMASTER_CANCNT; i++) {
    if (_can[i]) {
      _can[i]->terminate( );
      delete _can[i];
      _can[i] = nullptr;
    }
  }
}

void MdlMotorMaster::activate() {
}

void MdlMotorMaster::deactivate() {
}

void MdlMotorMaster::update() {

  for (int i = 0; i < MOTORMASTER_CANCNT; i++) {
    if (_can[i]) _can[i]->sendSync( );
  }

}

void MdlMotorMaster::getJointState( unsigned int index, MotorHW::state_t &state ) {
  if (index >= 12) return;
  caninfo_t info = _motors[index];
  info.bus->getJointState( info.index, state );
}

void MdlMotorMaster::setJointCommand( unsigned int index, MotorHW::cmd_t &cmd ) {
  if (index >= 12) return;
  caninfo_t info = _motors[index];
  info.bus->setJointCommand( info.index, cmd );
}

void MdlMotorMaster::getJointCommand( unsigned int index, MotorHW::cmd_t &cmd ) {
  if (index >= 12) return;
  caninfo_t info = _motors[index];
  info.bus->getJointCommand( info.index, cmd );
}
