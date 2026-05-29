#include "Go1IMUHW.hh"
#include "MdlGo1.hh"

Go1IMUHW::Go1IMUHW( MdlGo1 *m ) {
  _gm = m;
}

bool Go1IMUHW::getLastReading( unsigned int ind, imudata_t &data ) {
  if (ind > 0) {
    data = _zero;
    return true;
  }
  return _gm->getIMUData( data );
}

Go1IMUHW::status_t Go1IMUHW::getStatus( unsigned int ind ) {
  if (ind > 0) return STATUS_ERROR;
  return STATUS_READY;
}
