/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef GO1CLOCKHW_HH
#define GO1CLOCKHW_HH

#include <chrono>

#include "rtcore/ClockHW.hh"
#include "rtcore/NanoTimer.hh"

class MdlGo1;

class Go1ClockHW : public rtcore::ClockHW {
public:
  Go1ClockHW();
  ~Go1ClockHW();
  
  rtcore::CLOCK readClock();
  void waitPeriod();

  void setPeriod( rtcore::CLOCK period );

private:
  rtcore::NanoTimer *_timer = nullptr;
  std::chrono::time_point<std::chrono::system_clock> _start;
};

#endif
