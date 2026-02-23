/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include "rtcore/ModuleManager.hh"
#include "Go1MotorHW.hh"
#include "MdlGo1.hh"

Go1MotorHW::Go1MotorHW( MdlGo1 *m ) {
  _gm = m;
}

void Go1MotorHW::enable(unsigned int ind) {
  if (ind > max_index() || _enabled[ind]) return;
  _enabled[ind] = true;
  _gm->setEnable(ind, true);
}

void Go1MotorHW::disable(unsigned int ind){
  if (ind > max_index() || !_enabled[ind]) return;
  _enabled[ind] = false;
  _gm->setEnable(ind, false);
}

bool Go1MotorHW::isenabled(unsigned int ind) { 
  if (ind > max_index()) return false;
  return _enabled[ind];
}

void Go1MotorHW::setCommand( unsigned int ind, cmd_t &cmd ){
  if (ind > max_index()) return;
  _gm->setJointCommand(ind, cmd);
}

void Go1MotorHW::getCommand( unsigned int ind, cmd_t &cmd ) {
  if (ind > max_index()) {
    cmd = _cmd_zero;
    return;
  }
  _gm->getJointCommand(ind, cmd);
}

void Go1MotorHW::getState( unsigned int ind, state_t &state ) {
  if (ind > max_index()) {
    state = _state_zero;
    return;
  }
  _gm->getJointState(ind, state);
}

MotorHW::status_t Go1MotorHW::getStatus( unsigned int ind ) {
  if (ind > max_index()) return MotorHW::STATUS_ERROR;
  return _gm->getJointStatus(ind);
}

