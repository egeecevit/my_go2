/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef ROBOTCLOCKHW_HH
#define ROBOTCLOCKHW_HH

#include <chrono>

#include "rtcore/ClockHW.hh"
#include "rtcore/NanoTimer.hh"

class RobotClockHW : public rtcore::ClockHW {
 public:
  RobotClockHW();
  ~RobotClockHW();

  rtcore::CLOCK readClock();
  void waitPeriod();

  void setPeriod(rtcore::CLOCK period);

 private:
  rtcore::NanoTimer *_timer = nullptr;
  std::chrono::time_point<std::chrono::system_clock> _start;
};

#endif
