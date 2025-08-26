/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef CUBEMARSCAN_HH
#define CUBEMARSCAN_HH

#include <pthread.h>
#include "rtcore/ThreadedLoop.hh"
#include "hardware/MotorHW.hh"

#include <vector>

class CubeMARSCAN : public rtcore::ThreadedLoop {
public:
  typedef struct {
    int canid;        // CAN bus id
    int motorid;      // CAN bus id
    int polarity;     // Polarity, 1 or -1
    std::string model;
  } axis_t;

  typedef struct {
    std::string         intf  = "";  // CAN interface name
    std::vector<axis_t> axes;        // Axis information
  } config_t;

  CubeMARSCAN( config_t config, rtcore::ModuleManager *mgr );
  ~CubeMARSCAN( );

  void threadEnter( void );
  void threadLoop( void );
  void threadExit( void );

  void getJointState( unsigned int index, MotorHW::state_t &state );
  void setJointCommand( unsigned int index, MotorHW::cmd_t &cmd );
  void getJointCommand( unsigned int index, MotorHW::cmd_t &cmd );
  
private:
  rtcore::ModuleManager *_mgr = nullptr;
  config_t _config;

  // Latest joint states retrieved from 
  std::vector<MotorHW::state_t> _state;

  // Latest command setting relayed by MotorHW
  std::vector<MotorHW::cmd_t>   _cmd;

  // Mutex to coordinate data access between the CAN bus thread and
  // joint data access methods
  pthread_mutex_t _data_lock;

};

#endif
