/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include <stdio.h>
#include <unistd.h>
#include "rtcore/ModuleManager.hh"
#include "CubeMARSCAN.hh"

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...) printf(__VA_ARGS__);

using namespace rtcore;

CubeMARSCAN::CubeMARSCAN( config_t config, ModuleManager *mgr ) {

  _config = config;

  // TODO: Sanity checks on _config to make sure device exists, can
  // bus ids are sensible etc.
  
  // Initialize vector of state and command objects for 
  MotorHW::state_t state_zero;
  MotorHW::cmd_t   cmd_zero;
  for (unsigned int i = 0; i < _config.axes.size(); i++) {
    _state.push_back(state_zero);
    _cmd.push_back(cmd_zero);
  }
  
  DBGPRINT("CubeMARSCAN[%s]: Created with %lu axes\n",
           _config.intf.c_str(), _config.axes.size());

  _mgr = mgr;

  pthread_mutex_init( &_data_lock, NULL );
}

CubeMARSCAN::~CubeMARSCAN( ) {
  pthread_mutex_destroy( &_data_lock );
  
  // Close open files and clean up if necessary
}

void CubeMARSCAN::threadEnter( void ) {
  // TODO: Open and initialize CAN device
}

void CubeMARSCAN::threadLoop( void ) {
  waitSync();

  DBGPRINT("CubeMARSCAN[%s]: LOOP [t:%lf].\n",
           _config.intf.c_str(), _mgr->readTime());

  // TODO: Write commands to CAN devices on the bus from _cmd[]. Make
  // sure to use _data_lock when retrieving data.

  usleep( 750 );

  // TODO: Read back and process CAN responses into _state[]. Make
  // sure to use _data_lock when writing data.
}

void CubeMARSCAN::threadExit( void ) {
  // TODO: Uninitialize and close CAN device
}

void CubeMARSCAN::getJointState( unsigned int ind, MotorHW::state_t &state ) {
  pthread_mutex_lock( &_data_lock );
  state = _state[ind];
  pthread_mutex_unlock( &_data_lock );
}

void CubeMARSCAN::setJointCommand( unsigned int ind, MotorHW::cmd_t &cmd ) {
  pthread_mutex_lock( &_data_lock );
  _cmd[ind] = cmd;
  pthread_mutex_unlock( &_data_lock );
}

void CubeMARSCAN::getJointCommand( unsigned int ind, MotorHW::cmd_t &cmd ) {
  pthread_mutex_lock( &_data_lock );
  cmd = _cmd[ind];
  pthread_mutex_unlock( &_data_lock );
}
  
