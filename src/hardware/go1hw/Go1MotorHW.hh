/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef _GO1MOTORHW_HH
#define _GO1MOTORHW_HH

#include "hardware/MotorHW.hh"

#define NUM_MOTORS_GO1 12

class MdlGo1;

/** This class, derived from MotorHW, implements the functionality
    required by MotorHW through the Unitree Legged SDK. */
class Go1MotorHW : public MotorHW {

public:
  Go1MotorHW( MdlGo1 *m );
  virtual ~Go1MotorHW() {}
  
  void enable(unsigned int ind);
  void disable(unsigned int ind);
  bool isenabled(unsigned int ind);

  void setCommand( unsigned int ind, cmd_t &cmd );
  void getCommand( unsigned int ind, cmd_t &cmd );

  void getState( unsigned int ind, state_t &state );

  void calibrate( unsigned int ind, double absAngle ) {};
  bool isCalibrated( unsigned int ind ) { return true; };

  status_t getStatus( unsigned int ind );

  //Maximum of one user for each motor
  unsigned int max_users( unsigned int index =0) { (void)index; return 1; }

  //12 motors, 3 for each one of 4 legs
  unsigned int max_index( void ) { return NUM_MOTORS_GO1; }

private:
  MdlGo1 *_gm = nullptr;
  
  bool  _enabled[NUM_MOTORS_GO1] = {};

  cmd_t _cmd_zero;
  state_t _state_zero;
};
  
#endif
