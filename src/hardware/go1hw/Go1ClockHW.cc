/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "Go1ClockHW.hh"
#include "rtcore/NanoTimer.hh"

using namespace rtcore;

Go1ClockHW::Go1ClockHW() {
  _timer = new NanoTimer();
  _timer->start(1000 * _period);

  _start = std::chrono::system_clock::now();
}

Go1ClockHW::~Go1ClockHW() {
  if (_timer) {
    _timer->stop();
    delete _timer;
    _timer = nullptr;
  }
}

CLOCK Go1ClockHW::readClock() {
  auto cur_time = std::chrono::system_clock::now();
  auto diff =
      std::chrono::duration_cast<std::chrono::microseconds>(cur_time - _start).count();
  return diff;
}

void Go1ClockHW::waitPeriod() {
  if (_timer) _timer->wait_period();
}

void Go1ClockHW::setPeriod(long period) {
  ClockHW::setPeriod(period);

  // Adjust the timer with the new period (in nanoseconds)
  if (_timer) {
    _timer->stop();
    _timer->start(1000 * _period);
  }
}
