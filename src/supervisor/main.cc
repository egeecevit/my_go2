/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include "rtcore/Module.hh"
#include "rtcore/ThreadUtil.hh"
#include "rtcore/ModuleManager.hh"

#include "hardware/MotorHW.hh"
#include "quadruped/CoreModules.hh"

#include "app_common.hh"
#include "Supervisor.hh"

using namespace rtcore;

int main(int argc, char **argv) {

  std::string config_string;
  if (!parseArgs(argc, argv, config_string))
    return 0;

  ModuleManager mm;
  installSignalHandlers(&mm);
  loadConfig(mm, config_string);

  initHardware(&mm);

  AddCoreModules(&mm);
  ActivateCoreModules(&mm);

  // Keyboard input is handled by Supervisor; Ctrl-C by the signal handler
  Supervisor *sm = new Supervisor;
  mm.addModule(sm, 1, 0, USER_CONTROLLERS);
  mm.activateModule(sm);

  mm.message("\n** Current list of modules:");
  mm.printModules();
  mm.message("\n** Current list of threads:");
  ThreadUtil::printThreads();

  mm.message("\n** Entering main loop...");
  mm.mainLoop();
  mm.message("\n** Main loop exited...");

  mm.deactivateModule(sm);
  mm.removeModule(sm);
  delete sm;

  DeactivateCoreModules(&mm);
  RemoveCoreModules(&mm);

  mm.message("** Shutting down...");
  cleanupHardware();
  mm.shutdown();

  return 0;
}

