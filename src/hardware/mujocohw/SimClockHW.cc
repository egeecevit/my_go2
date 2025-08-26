/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "SimClockHW.hh"

#include "MdlSimDriver.hh"
#include "rtcore/NanoTimer.hh"

using namespace rtcore;

SimClockHW::SimClockHW(MdlSimDriver *d) {
  _simdriver = d;
  _timer = new NanoTimer();
  _timer->start(1000 * _period);
  _start = std::chrono::system_clock::now();
}

SimClockHW::~SimClockHW() {
  if (_timer) {
    _timer->stop();
    delete _timer;
    _timer = nullptr;
  }
}

CLOCK SimClockHW::readClock() { return _simdriver->readClock(); }

CLOCK SimClockHW::readRealClock() {
  auto cur_time = std::chrono::system_clock::now();
  auto diff =
      std::chrono::duration_cast<std::chrono::microseconds>(cur_time - _start).count();
  return diff;
}

void SimClockHW::waitPeriod() {
  // We only block for simulation environments when real-time operation is
  // requested
  if (_realtime) {
    auto now = readRealClock();
    if (now - _prevUpdate < _period) {
      if (_timer) _timer->wait_period();
    }
    _prevUpdate = readRealClock();
  }
  return;
}

void SimClockHW::setPeriod(long period) { ClockHW::setPeriod(period); }
