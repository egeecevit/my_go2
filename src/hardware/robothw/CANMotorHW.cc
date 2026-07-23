/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include "rtcore/ModuleManager.hh"
#include "CANMotorHW.hh"
#include "MdlMotorMaster.hh"

CANMotorHW::CANMotorHW( MdlMotorMaster *m ) {
  _master = m;
}

void CANMotorHW::enable(unsigned int ind) {
  if (ind >= max_index() || _enabled[ind]) return;
  _enabled[ind] = true;
  printf("CANMotorHW[%d]: Enabled axis\n", ind);
}

void CANMotorHW::disable(unsigned int ind){
  if (ind >= max_index() || !_enabled[ind]) return;
  _enabled[ind] = false;
  printf("CANMotorHW[%d]: Disabled axis\n", ind);
}

bool CANMotorHW::isenabled(unsigned int ind) {
  if (ind >= max_index()) return false;
  return _enabled[ind];
}

void CANMotorHW::setCommand( unsigned int ind, cmd_t &cmd ){
  if (ind >= max_index()) return;
  _master->setJointCommand( ind, cmd );
}

void CANMotorHW::getCommand( unsigned int ind, cmd_t &cmd ) {
  if (ind >= max_index()) {
    cmd = _cmd_zero;
    return;
  }
  _master->getJointCommand( ind, cmd );
}

void CANMotorHW::getState( unsigned int ind, state_t &state ) {
  if (ind >= max_index()) {
    state = _state_zero;
    return;
  }
  _master->getJointState( ind, state );
}
