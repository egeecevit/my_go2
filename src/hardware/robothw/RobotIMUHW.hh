#ifndef _ROBOTIMUHW_HH
#define _ROBOTIMUHW_HH

#include "hardware/IMUHW.hh"

class VN100Driver;

/** This class, derived from IMUHW, implements the functionality
    required by IMUHW through the VN100 serial IMU driver. */
class RobotIMUHW : public IMUHW {

public:
  RobotIMUHW( VN100Driver *d );
  virtual ~RobotIMUHW() {}
  
  bool getLastReading( unsigned int ind, imudata_t &data );
  
  status_t getStatus( unsigned int ind );

  unsigned int max_index( void ) { return 1; }

private:

  // VN100 driver supplying the IMU readings.
  VN100Driver *_imudriver = nullptr;
  
  // Blank IMU reading for invalid indices
  imudata_t _zero;
};
  
#endif
