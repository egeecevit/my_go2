/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include "rtcore/ModuleManager.hh"
#include "SimMotorHW.hh"
#include "SimIMUHW.hh"
#include "SimClockHW.hh"

#include "MdlSimDriver.hh"

#include "quadruped/ModuleDefs.hh"

using namespace rtcore;

HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);
HARDWARE_IMPL(ClockHW);

static ModuleManager *_mgr = nullptr;

static SimClockHW    * _clockhw = nullptr;
static SimMotorHW * _motorhw = nullptr;
static SimIMUHW   * _imuhw = nullptr;

static MdlSimDriver *_simdriver = nullptr;

#define DBGPRINT(...) //printf(__VA_ARGS__)

void initHardware( ModuleManager *mm ) {
  _mgr = mm;
  _mgr->message("initHardware(SIM): Initializing hardware components...");
  
  // No need for idle time for the simulation. Setting this to nonzero values
  // results in an additional call to waitPeriod for ClockHW
  _mgr->setMinIdleTime(0.0);

  // 1. Create, add and activate modules necessary for what needs to
  // be done below the hardware layer.
  CREATE_MODULE(_mgr, MdlSimDriver, _simdriver );
  ACTIVATE_MODULE(_mgr,_simdriver );
  
  // 2. Create and register specific instances of all hardware classes
  _clockhw = new SimClockHW( _simdriver );
  ClockHW::registerInstance( _clockhw );
  if (_clockhw) {
    _mgr->setClockHW( _clockhw );
    ConfigTable simConfig;
    bool hasSimConfig = _mgr->getConfigTable("simulation", simConfig);
    if (hasSimConfig) {
      bool realtime = simConfig.getBool("realtime", false);
      _clockhw->setRealtime(realtime);
      DBGPRINT("SimHW: Setting realtime flag to %s\n", realtime ? "true" : "false");
    }
  }

  _motorhw = new SimMotorHW( _simdriver );
  MotorHW::registerInstance( _motorhw );
  _imuhw = new SimIMUHW( _simdriver );
  IMUHW::registerInstance( _imuhw );
  
}

void cleanupHardware() {
  _mgr->message("cleanupHardware(SIM): Cleaning up hardware components...");

  // Deactivate, remove and delele modules.
  DEACTIVATE_MODULE(_mgr, _simdriver );
  DESTROY_MODULE(_mgr, _simdriver );

  delete _imuhw; _imuhw = nullptr;
  delete _motorhw; _motorhw = nullptr;
  delete _clockhw; _clockhw = nullptr;
}

