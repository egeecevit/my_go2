#ifndef _SIMIMUHW_HH
#define _SIMIMUHW_HH

#include "hardware/IMUHW.hh"

class MdlSimDriver;

/** This class, derived from IMUHW, implements the functionality
    required by IMUHW through a low level interface to the
    Simulation. */
class SimIMUHW : public IMUHW {

public:
  SimIMUHW( MdlSimDriver *d );
  virtual ~SimIMUHW() {}
  
  bool getLastReading( unsigned int ind, imudata_t &data );
  
  status_t getStatus( unsigned int ind );

  unsigned int max_index( void ) { return 1; }

private:

  // SimDriver module is needed to retrieve state and relay torque
  // commands to the joints. 
  MdlSimDriver *_simdriver = nullptr;
  
  // Blank IMU reading for invalid indices
  imudata_t _zero;
};
  
#endif
