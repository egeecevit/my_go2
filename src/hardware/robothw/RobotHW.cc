/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */
#include <unistd.h>

#include "CANMotorHW.hh"
#include "MdlMotorMaster.hh"
#include "ModuleDefs.hh"
#include "RobotClockHW.hh"
#include "RobotIMUHW.hh"
#include "VN100Driver.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/Module.hh"
#include "rtcore/ModuleManager.hh"

using namespace rtcore;

HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);
HARDWARE_IMPL(ClockHW);

static ModuleManager *_mgr = nullptr;

static RobotClockHW *_clockhw = nullptr;
static CANMotorHW *_motorhw = nullptr;
static RobotIMUHW *_imuhw = nullptr;

static VN100Driver *_vn100 = nullptr;
static MdlMotorMaster *_motormaster = nullptr;

void initHardware(ModuleManager *mm) {
  _mgr = mm;
  _mgr->message("initHardware(CAN): Initializing hardware components...");

  // Configure CPU sets and main thread priority
  int ret;
  pthread_t thread = pthread_self();
  ConfigTable tc(_mgr->getConfigRoot(), "threads");
  ConfigArray cpus(&tc, "main_cpuset");
  if (cpus.isValid()) {
    // Assign CPUs to the main thread as configured
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int i = 0; i < cpus.size(); i++) {
      if (cpus.hasTypeAt(i, ConfigType::Int)) CPU_SET(cpus.getIntAt(i, 0), &cpuset);
    }
    ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (ret)
      _mgr->warning("initHardware(CAN)", "Could not set main thread cpu affinity.");
  }
  if (tc.hasType("main_priority", ConfigType::Int)) {
    // Adjust thread priority to the maximum
    struct sched_param param;
    param.sched_priority = (unsigned int)tc.getInt("main_priority", 99.0);
    ret = sched_setscheduler(::getpid(), SCHED_FIFO, &param);
    if (ret) _mgr->warning("initHardware(CAN)", "Could not set main thread scheduler.");
  }

  // 1. Create any non-module utility classes
  _vn100 = new VN100Driver("/dev/ttyUSB0", _mgr);
  _vn100->start("vn100", tc.getInt("vn100_priority", 88));

  // 2. Create, add and activate modules necessary for what needs to
  // be done below the hardware layer.
  CREATE_MODULE(_mgr, MdlMotorMaster, _motormaster);
  ACTIVATE_MODULE(_mgr, _motormaster);

  // 3. Create and register specific instances of all hardware classes

  _clockhw = new RobotClockHW;
  ClockHW::registerInstance(_clockhw);
  if (_clockhw) _mgr->setClockHW(_clockhw);
  _motorhw = new CANMotorHW(_motormaster);
  MotorHW::registerInstance(_motorhw);
  _imuhw = new RobotIMUHW(_vn100);
  IMUHW::registerInstance(_imuhw);
}

void cleanupHardware() {
  _mgr->message("cleanupHardware(CAN): Cleaning up hardware components...");

  // Deactivate, remove and delele modules.
  DEACTIVATE_MODULE(_mgr, _motormaster);
  DESTROY_MODULE(_mgr, _motormaster);

  // Clean up and delete non-module objects
  _vn100->terminate();
  delete _vn100;
  _vn100 = nullptr;

  delete _imuhw;
  _imuhw = nullptr;
  delete _motorhw;
  _motorhw = nullptr;
  delete _clockhw;
  _clockhw = nullptr;
}
