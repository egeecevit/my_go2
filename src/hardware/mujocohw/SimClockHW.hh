/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef SIMCLOCKHW_HH
#define SIMCLOCKHW_HH

#include <chrono>

#include "rtcore/ClockHW.hh"
#include "rtcore/NanoTimer.hh"

class MdlSimDriver;

class SimClockHW : public rtcore::ClockHW {
public:
  SimClockHW( MdlSimDriver *d );
  ~SimClockHW();
  
  rtcore::CLOCK readClock();
  void waitPeriod();

  void setPeriod( rtcore::CLOCK period );

  void setRealtime(bool realtime) { _realtime = realtime; }

  rtcore::CLOCK readRealClock();

private:
  MdlSimDriver *_simdriver = nullptr;

  rtcore::NanoTimer *_timer = nullptr;
  std::chrono::time_point<std::chrono::system_clock> _start;
  bool _realtime = false;
  rtcore::CLOCK _prevUpdate = 0;
};

#endif
