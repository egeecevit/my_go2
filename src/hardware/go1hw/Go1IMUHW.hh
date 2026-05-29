#ifndef _GO1IMUHW_HH
#define _GO1IMUHW_HH

#include "hardware/IMUHW.hh"

class MdlGo1;

class Go1IMUHW : public IMUHW {

public:
  Go1IMUHW( MdlGo1 *m );
  virtual ~Go1IMUHW() {}

  bool getLastReading( unsigned int ind, imudata_t &data );

  status_t getStatus( unsigned int ind );

  unsigned int max_index( void ) { return 1; }

private:
  MdlGo1 *_gm = nullptr;
  imudata_t _zero;
};

#endif
