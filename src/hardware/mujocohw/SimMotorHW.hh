/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef _SIMMOTORHW_HH
#define _SIMMOTORHW_HH

#include "hardware/MotorHW.hh"

#define NUM_MOTORS_SIM 12

class MdlSimDriver;

/** \brief This class, derived from MotorHW, implements the functionality
    required by MotorHW through a low level interface to the
    Simulation. */
class SimMotorHW : public MotorHW {

public:
  SimMotorHW( MdlSimDriver *d );
  virtual ~SimMotorHW() {}
  
  void enable(unsigned int ind);
  void disable(unsigned int ind);
  bool isenabled(unsigned int ind);

  void setCommand( unsigned int ind, cmd_t &cmd );
  void getCommand( unsigned int ind, cmd_t &cmd );

  void getState( unsigned int ind, state_t &state );

  void calibrate( unsigned int ind, double absAngle ) {};
  bool isCalibrated( unsigned int ind ) { return true; };

  status_t getStatus( unsigned int ind ) { return STATUS_READY; };

  //Maximum of one user for each motor
  unsigned int max_users( unsigned int ind ) { return 1; }

  //12 motors, 3 for each one of 4 legs
  unsigned int max_index( void ) { return NUM_MOTORS_SIM; }

private:

  // SimDriver module is needed to retrieve state and relay torque
  // commands to the joints. 
  MdlSimDriver *_simdriver = nullptr;
  
  // Blank command and state structures to be returned for invalid indices
  cmd_t   _cmd_zero;
  state_t _state_zero;
};
  
#endif
