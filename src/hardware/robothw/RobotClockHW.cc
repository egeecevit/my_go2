/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "RobotClockHW.hh"

#include "rtcore/NanoTimer.hh"

using namespace rtcore;

RobotClockHW::RobotClockHW() {
  _timer = new NanoTimer();
  _timer->start(1000 * _period);

  _start = std::chrono::system_clock::now();
}

RobotClockHW::~RobotClockHW() {
  if (_timer) {
    _timer->stop();
    delete _timer;
    _timer = nullptr;
  }
}

CLOCK RobotClockHW::readClock() {
  auto cur_time = std::chrono::system_clock::now();
  auto diff =
      std::chrono::duration_cast<std::chrono::microseconds>(cur_time - _start).count();
  return diff;
}

void RobotClockHW::waitPeriod() {
  if (_timer) _timer->wait_period();
}

void RobotClockHW::setPeriod(CLOCK period) {
  ClockHW::setPeriod(period);

  // Adjust the timer with the new period (in nanoseconds).
  // start() allocates its timespecs, so stop first to avoid leaking them.
  if (_timer) {
    _timer->stop();
    _timer->start(1000 * _period);
  }
}
