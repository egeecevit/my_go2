/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */
#include <math.h>
#include <unistd.h>

#include "quadruped/ModuleDefs.hh"
#include "MdlGo1.hh"
#include "Go1ClockHW.hh"
#include "Go1MotorHW.hh"
#include "Go1IMUHW.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/Module.hh"
#include "rtcore/ModuleManager.hh"

using namespace rtcore;

HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(IMUHW);

static ModuleManager *_mgr = nullptr;

static Go1ClockHW *_clockhw = nullptr;
static Go1MotorHW *_motorhw = nullptr;
static Go1IMUHW *_imuhw = nullptr;

static MdlGo1 *_go1 = nullptr;

#define HWNAME "go1hw"

static bool initStatus = false;

void initHardware(ModuleManager *mm) {
  _mgr = mm;
  _mgr->message("initHardware(Go1): Initializing hardware components...");

  _mgr->checkHardware(HWNAME); // Will exit on failure
  _mgr->message("initHardware(Go1): Using Go1 hardware libraries");

  if (!_mgr->lockHardware( HWNAME )) {
    _mgr->fatalError("initHardware(Go1)",
                     "Another process has hardware lock. Exiting.");
  }        

  // 2. Create, add and activate modules necessary for what needs to
  // be done below the hardware layer.
  CREATE_MODULE(_mgr, MdlGo1, _go1);
  ACTIVATE_MODULE(_mgr, _go1);

  // 3. Create and register specific instances of all hardware classes
  _clockhw = new Go1ClockHW;
  ClockHW::registerInstance(_clockhw);
  if (_clockhw) _mgr->setClockHW(_clockhw);
  
  _motorhw = new Go1MotorHW(_go1);
  MotorHW::registerInstance(_motorhw);
  _imuhw = new Go1IMUHW(_go1);
  IMUHW::registerInstance(_imuhw);
  initStatus = true;
}

void cleanupHardware() {
  _mgr->message("cleanupHardware(Go1): Cleaning up hardware components...");

  if (initStatus) {
    DEACTIVATE_MODULE(_mgr, _go1);
    DESTROY_MODULE(_mgr, _go1);

    delete _imuhw;   _imuhw = nullptr;
    delete _motorhw; _motorhw = nullptr;
    delete _clockhw; _clockhw = nullptr;

    _mgr->unlockHardware( HWNAME );
    initStatus = false;
  }
}

