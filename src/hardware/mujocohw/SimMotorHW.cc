#include "rtcore/ModuleManager.hh"
#include "SimMotorHW.hh"
#include "MdlSimDriver.hh"

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...) //printf(__VA_ARGS__);

SimMotorHW::SimMotorHW( MdlSimDriver *d ) {
  _simdriver = d;
}

void SimMotorHW::enable(unsigned int ind) {
  if (ind >= max_index()) return;

  DBGPRINT("SimMotorHW[%d]: Enabling axis...\n", ind);
  _simdriver->setJointEnabled( ind, true );
}

void SimMotorHW::disable(unsigned int ind){
  if (ind >= max_index()) return;

  DBGPRINT("SimMotorHW[%d]: Disabling axis...\n", ind);
  _simdriver->setJointEnabled( ind, false );
}

bool SimMotorHW::isenabled(unsigned int ind) {
  if (ind >= max_index()) return false;
  return _simdriver->getJointEnabled( ind );
}

void SimMotorHW::setCommand( unsigned int ind, cmd_t &cmd ){
  if (ind >= max_index()) return;

  DBGPRINT("SimMotorHW[%d]: Setting new command...\n", ind);
  _simdriver->setJointCommand( ind, cmd );
}

void SimMotorHW::getCommand( unsigned int ind, cmd_t &cmd ) {
  if (ind >= max_index()) {
    cmd = _cmd_zero;
    return;
  }
  _simdriver->getJointCommand( ind, cmd );
}

void SimMotorHW::getState( unsigned int ind, state_t &state ) {
  if (ind >= max_index()) {
    state = _state_zero;
    return;
  }
  _simdriver->getJointState( ind, state );
}
