#include "RobotIMUHW.hh"
#include "VN100Driver.hh"

RobotIMUHW::RobotIMUHW( VN100Driver *d ) {
  _imudriver = d;
}

bool RobotIMUHW::getLastReading( unsigned int ind, imudata_t &data ) {
  if (ind > 0) {
    data = _zero;
    return true;
  } else return _imudriver->getIMUData( data );
}

RobotIMUHW::status_t RobotIMUHW::getStatus( unsigned int ind ) {
  if (ind > 0) return STATUS_ERROR;
  return STATUS_READY;
}

