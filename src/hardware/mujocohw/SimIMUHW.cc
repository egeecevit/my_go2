#include "SimIMUHW.hh"
#include "MdlSimDriver.hh"

SimIMUHW::SimIMUHW( MdlSimDriver *d ) {
  _simdriver = d;
}

bool SimIMUHW::getLastReading( unsigned int ind, imudata_t &data ) {
  if (ind > 0) {
    data = _zero;
    return true;
  } else return _simdriver->getIMUData( data );
}

SimIMUHW::status_t SimIMUHW::getStatus( unsigned int ind ) {
  if (ind > 0) return STATUS_ERROR;
  return STATUS_READY;
}

